#include "src/motor/motor_control.h"

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "stepper_pulse.pio.h"
#include "FreeRTOS.h"
#include "task.h"

static PIO stepper_pio = pio0;
static uint stepper_sm = 0;
static uint stepper_offset = 0;

static inline float clampf(float v, float min, float max) {
    return (v < min) ? min : ((v > max) ? max : v);
}

static inline uint16_t clamp_u16(uint16_t v, uint16_t min, uint16_t max) {
    return (v < min) ? min : ((v > max) ? max : v);
}

void setup_stepper_pio(void) {
    stepper_offset = pio_add_program(stepper_pio, &stepper_pulse_program);
    pio_gpio_init(stepper_pio, PIN_MOTOR_PULSE);

    pio_sm_config c = stepper_pulse_program_get_default_config(stepper_offset);
    sm_config_set_sideset_pins(&c, PIN_MOTOR_PULSE);

    float div = 3676.47f;
    sm_config_set_clkdiv(&c, div);
    sm_config_set_out_shift(&c, true, false, 32);

    pio_sm_init(stepper_pio, stepper_sm, stepper_offset, &c);
    pio_sm_set_consecutive_pindirs(stepper_pio, stepper_sm, PIN_MOTOR_PULSE, 1, true);
    pio_sm_set_enabled(stepper_pio, stepper_sm, true);

    gpio_init(PIN_MOTOR_DIR);
    gpio_set_dir(PIN_MOTOR_DIR, GPIO_OUT);
    gpio_put(PIN_MOTOR_DIR, 0);
}

void send_stepper_pulses(uint32_t count) {
    pio_sm_put_blocking(stepper_pio, stepper_sm, count - 1);
    printf("[Send stepper pulses] Sending %lu pulses\n", (unsigned long)count);
}

bool is_stepper_busy(void) {
    return !pio_sm_is_tx_fifo_empty(stepper_pio, stepper_sm) ||
           (pio_sm_get_pc(stepper_pio, stepper_sm) != stepper_offset);
}

void wait_stepper_done(void) {
    while (is_stepper_busy()) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void stop_stepper_pio(void) {
    pio_sm_set_enabled(stepper_pio, stepper_sm, false);
    pio_sm_clear_fifos(stepper_pio, stepper_sm);
    pio_sm_restart(stepper_pio, stepper_sm);
    pio_sm_exec(stepper_pio, stepper_sm, pio_encode_jmp(stepper_offset));
    pio_sm_set_enabled(stepper_pio, stepper_sm, true);
}

float potToPistonPos(uint16_t pot) {
    pot = clamp_u16(pot, MINIMAL_THRESHOLD, MAXIMUM_THRESHOLD);
    return (((float)(pot - MINIMAL_THRESHOLD) / POT_RANGE) * PISTON_RANGE) - MAX_PISTON_POSITION;
}

uint16_t pistonPosToPot(float pos_mm) {
    pos_mm = clampf(pos_mm, -MAX_PISTON_POSITION, MAX_PISTON_POSITION);
    float fraction = (pos_mm + MAX_PISTON_POSITION) / PISTON_RANGE;
    return MINIMAL_THRESHOLD + (uint16_t)(fraction * POT_RANGE);
}

float pistonPosToVolume(float pos_mm) {
    return clampf(pos_mm * VOL_MULTIPLIER, -MAX_VOLUME, MAX_VOLUME);
}

float volumeToPistonPos(float vol_cm3) {
    return clampf(vol_cm3 / VOL_MULTIPLIER, -MAX_PISTON_POSITION, MAX_PISTON_POSITION);
}

void sendStopCommand(void) {
    stop_stepper_pio();
    printf("[HW -> Stepper]: Stop pulses\n");
}

void sendEnableCommand(void) {
    gpio_put(PIN_ENABLE_DRIVER, 0);
    printf("[HW -> Stepper]: Enable driver\n");
}

void sendDisableCommand(void) {
    gpio_put(PIN_ENABLE_DRIVER, 1);
    stop_stepper_pio();
    printf("[HW -> Stepper]: **DISABLE** (emergency stop)\n");
}

void gpio_limit_switches_callback(uint gpio, uint32_t events) {
    if (!limit_switches_ready) {
        return;
    }

    if ((events & GPIO_IRQ_EDGE_FALL) == 0) {
        return;
    }

    if (gpio_get(gpio) != 0) {
        return;
    }

    uint32_t now_us = time_us_32();
    volatile uint32_t* last_event_us = (gpio == SW_MIN_LIMIT) ? &last_min_limit_event_us : &last_max_limit_event_us;
    const uint32_t debounce_window_us = 50000;

    if (now_us - *last_event_us < debounce_window_us) {
        return;
    }
    *last_event_us = now_us;

    if (gpio == SW_MIN_LIMIT) {
        pending_min_limit_event = true;
    } else if (gpio == SW_MAX_LIMIT) {
        pending_max_limit_event = true;
    }
}

void evaluate_pending_limit_switches(void) {
    if (pending_min_limit_event) {
        pending_min_limit_event = false;
        if (!gpio_get(SW_MIN_LIMIT) && (time_us_32() - last_min_limit_event_us) > 20000) {
            flag_min_limit_hit = true;
        }
    }

    if (pending_max_limit_event) {
        pending_max_limit_event = false;
        if (!gpio_get(SW_MAX_LIMIT) && (time_us_32() - last_max_limit_event_us) > 20000) {
            flag_max_limit_hit = true;
        }
    }
}

void setup_limit_switches_on_core1(void) {
    limit_switches_ready = false;
    last_min_limit_event_us = 0;
    last_max_limit_event_us = 0;

    gpio_init(SW_MIN_LIMIT);
    gpio_set_dir(SW_MIN_LIMIT, GPIO_IN);
    gpio_pull_up(SW_MIN_LIMIT);
    gpio_set_input_hysteresis_enabled(SW_MIN_LIMIT, true);

    gpio_init(SW_MAX_LIMIT);
    gpio_set_dir(SW_MAX_LIMIT, GPIO_IN);
    gpio_pull_up(SW_MAX_LIMIT);
    gpio_set_input_hysteresis_enabled(SW_MAX_LIMIT, true);

    gpio_set_irq_enabled_with_callback(SW_MIN_LIMIT, GPIO_IRQ_EDGE_FALL, false, &gpio_limit_switches_callback);
    gpio_set_irq_enabled(SW_MAX_LIMIT, GPIO_IRQ_EDGE_FALL, false);

    busy_wait_ms(250);

    gpio_set_irq_enabled_with_callback(SW_MIN_LIMIT, GPIO_IRQ_EDGE_FALL, true, &gpio_limit_switches_callback);
    gpio_set_irq_enabled(SW_MAX_LIMIT, GPIO_IRQ_EDGE_FALL, true);
    limit_switches_ready = true;
}

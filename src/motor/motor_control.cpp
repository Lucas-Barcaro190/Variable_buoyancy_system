#include "src/motor/motor_control.h"

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "pwm_pio.pio.h"
#include "FreeRTOS.h"
#include "task.h"

static PIO stepper_pio = pio0;
static uint stepper_sm = 0;
static uint stepper_offset = 0;
static float pio_clkdiv = 125.0f; // 125 MHz / 125 = 1 MHz PIO clock

// Inicialização do PID
void pid_init(PIDController_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max) {
    if (!pid) return;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void pid_reset(PIDController_t *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// Cálculo do PID discreto
float pid_compute(PIDController_t *pid, float setpoint, float measurement, float dt) {
    if (!pid || dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    // Termo Proporcional
    float P = pid->Kp * error;

    // Termo Integral com anti-windup clamping
    pid->integral += error * dt;
    float I = pid->Ki * pid->integral;
    if (I > pid->out_max) {
        I = pid->out_max;
        if (pid->Ki != 0.0f) pid->integral = I / pid->Ki;
    } else if (I < pid->out_min) {
        I = pid->out_min;
        if (pid->Ki != 0.0f) pid->integral = I / pid->Ki;
    }

    // Termo Derivativo
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    pid->prev_error = error;

    // Saída total saturada
    float output = P + I + D;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}

// Configuração da PIO para PWM contínuo do motor de passo
void setup_stepper_pio(void) {
    stepper_offset = pio_add_program(stepper_pio, &stepper_variable_program);
    pio_gpio_init(stepper_pio, PIN_MOTOR_PULSE);

    pio_sm_config c = stepper_variable_program_get_default_config(stepper_offset);
    sm_config_set_set_pins(&c, PIN_MOTOR_PULSE, 1);
    sm_config_set_clkdiv(&c, pio_clkdiv);
    sm_config_set_out_shift(&c, true, false, 32);

    pio_sm_init(stepper_pio, stepper_sm, stepper_offset, &c);
    pio_sm_set_consecutive_pindirs(stepper_pio, stepper_sm, PIN_MOTOR_PULSE, 1, true);
    pio_sm_exec(stepper_pio, stepper_sm, pio_encode_set(pio_pins, 0));
    pio_sm_set_enabled(stepper_pio, stepper_sm, false);

    gpio_init(PIN_MOTOR_DIR);
    gpio_set_dir(PIN_MOTOR_DIR, GPIO_OUT);
    gpio_put(PIN_MOTOR_DIR, 0);
}

void set_stepper_period(uint32_t period_x) {
    if (period_x == 0) {
        period_x = 1;
    }
    pio_sm_set_enabled(stepper_pio, stepper_sm, true);
    pio_sm_put_blocking(stepper_pio, stepper_sm, period_x);
    pio_sm_put_blocking(stepper_pio, stepper_sm, period_x);
}

void set_stepper_speed_mm_s(float speed_mm_s) {
    float speed_mag = fabsf(speed_mm_s);

    if (speed_mag < 0.0001f) {
        // Velocidade praticamente nula: para o PWM
        stop_stepper_pio();
        return;
    }

    // Direção
    gpio_put(PIN_MOTOR_DIR, (speed_mm_s >= 0.0f) ? 0 : 1);

    // Frequência de passos desejada (Hz)
    float f_step = speed_mag * STEPS_PER_MM;

    if (f_step < 0.1f) {
        stop_stepper_pio();
        return;
    }

    // PIO clock is 1 MHz. Each loop iteration is 1 cycle. The program toggles
    // the pin once per delay period for the high and low half-cycles.
    float f_pio = 1000000.0f;
    float X_float = (f_pio / (2.0f * f_step));
    if (X_float < 1.0f) X_float = 1.0f;
    if (X_float > 16777215.0f) X_float = 16777215.0f;

    uint32_t period_x = (uint32_t)X_float;
    pio_sm_set_enabled(stepper_pio, stepper_sm, true);
    pio_sm_put(stepper_pio, stepper_sm, period_x);
    pio_sm_put(stepper_pio, stepper_sm, period_x);
}

void stop_stepper_pio(void) {
    pio_sm_set_enabled(stepper_pio, stepper_sm, false);
    pio_sm_clear_fifos(stepper_pio, stepper_sm);
    pio_sm_restart(stepper_pio, stepper_sm);
    pio_sm_exec(stepper_pio, stepper_sm, pio_encode_set(pio_pins, 0));
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

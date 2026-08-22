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
static bool stepper_pio_enabled = false;
static float pio_clkdiv = 125.0f; // 125 MHz / 125 = 1 MHz PIO clock


/*
Desc: Initialize PID controller parameters and reset internal state.
params:
    - [PIDController_t*] pid: Pointer to the PID controller instance.
    - [float] Kp: Proportional gain.
    - [float] Ki: Integral gain.
    - [float] Kd: Derivative gain.
    - [float] out_min: Minimum output value.
    - [float] out_max: Maximum output value.
returns:
    - [void]
*/
void pid_init(PIDController_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max) {
    if (!pid) return;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->filtered_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

/*
Desc: Reset the integral and derivative state of the PID controller.
params:
    - [PIDController_t*] pid: Pointer to the PID controller instance.
returns:
    - [void]
*/
void pid_reset(PIDController_t *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->filtered_error = 0.0f;
}

/*
Desc: Compute a PID output value from the current error and elapsed time.
params:
    - [PIDController_t*] pid: Pointer to the PID controller instance.
    - [float] setpoint: Desired target value.
    - [float] measurement: Current measured value.
    - [float] dt: Time step in seconds.
returns:
    - [float]: PID controller output, clamped to configured output limits.
*/
float pid_compute(PIDController_t *pid, float setpoint, float measurement, float dt) {
    if (!pid || dt <= 0.0f) return 0.0f;

    float raw_error = setpoint - measurement;
    float error = raw_error;

    if (fabsf(error) < 0.02f) { //deadband de 0.2mm
        error = 0.0f;
    } else{
        error = (error > 0.0f) ? (error - 0.02f) : (error + 0.02f);
    }
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

/*
Desc: Initialize the stepper motor PIO state machine and GPIO pins.
params:
    - none
returns:
    - [void]
*/
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

/*
Desc: Set the stepper PIO output period directly.
params:
    - [uint32_t] period_x: Period value to send to the PIO state machine.
returns:
    - [void]
*/
void set_stepper_period(uint32_t period_x) {
    if (period_x == 0) {
        period_x = 1;
    }
    pio_sm_put_blocking(stepper_pio, stepper_sm, period_x);
}

/*
Desc: Convert a target speed in mm/s to stepper pulse timing and enable the PIO.
params:
    - [float] speed_mm_s: Desired linear speed in mm/s.
returns:
    - [void]
*/
void set_stepper_speed_mm_s(float speed_mm_s) {
    float speed_mag = fabsf(speed_mm_s);

    if (speed_mag < 0.0001f) {
        stop_stepper_pio();
        return;
    }

    gpio_put(PIN_MOTOR_DIR, (speed_mm_s >= 0.0f) ? 1 : 0);

    float f_step = speed_mag * STEPS_PER_MM;
    if (f_step < 0.1f) {
        stop_stepper_pio();
        return;
    }

    float f_pio = 1000000.0f;
    float X_float = (f_pio / (2.0f * f_step));
    if (X_float < 1.0f) X_float = 1.0f;
    if (X_float > 16777215.0f) X_float = 16777215.0f;
    uint32_t period_x = (uint32_t)X_float;

    if (vbs_should_log(4)) {
        printf("[Motor] set_stepper_speed_mm_s: speed=%.4f mm/s, f_step=%.2f Hz, period_x=%u, pio_enabled=%u\n",
               speed_mag, f_step, (unsigned)period_x, (unsigned)stepper_pio_enabled);
    }

    if (!stepper_pio_enabled) {
        pio_sm_restart(stepper_pio, stepper_sm);
        pio_sm_exec(stepper_pio, stepper_sm, pio_encode_set(pio_pins, 0));
        pio_sm_clear_fifos(stepper_pio, stepper_sm);
        pio_sm_set_enabled(stepper_pio, stepper_sm, true);
        stepper_pio_enabled = true;
    } else {
        pio_sm_clear_fifos(stepper_pio, stepper_sm);
    }

    pio_sm_put_blocking(stepper_pio, stepper_sm, period_x);
}

/*
Desc: Stop the stepper motor PIO state machine and clear FIFOs.
params:
    - none
returns:
    - [void]
*/
void stop_stepper_pio(void) {
    pio_sm_set_enabled(stepper_pio, stepper_sm, false);
    pio_sm_clear_fifos(stepper_pio, stepper_sm);
    pio_sm_restart(stepper_pio, stepper_sm);
    pio_sm_exec(stepper_pio, stepper_sm, pio_encode_set(pio_pins, 0));
    stepper_pio_enabled = false;
}

/*
Desc: Issue a stop command to the motor driver and stop pulses.
params:
    - none
returns:
    - [void]
*/
void sendStopCommand(void) {
    stop_stepper_pio();
    printf("[HW -> Stepper]: Stop pulses\n");
}

/*
Desc: Enable the motor driver output.
params:
    - none
returns:
    - [void]
*/
void sendEnableCommand(void) {
    gpio_put(PIN_ENABLE_DRIVER, 0);
    printf("[HW -> Stepper]: Enable driver\n");
}

/*
Desc: Disable the motor driver and stop all pulses immediately.
params:
    - none
returns:
    - [void]
*/
void sendDisableCommand(void) {
    gpio_put(PIN_ENABLE_DRIVER, 1);
    stop_stepper_pio();
    printf("[HW -> Stepper]: **DISABLE** (emergency stop)\n");
}

/*
Desc: GPIO interrupt handler for limit switches, debounces and schedules pending events.
params:
    - [uint] gpio: GPIO pin number that triggered the interrupt.
    - [uint32_t] events: GPIO event flags.
returns:
    - [void]
*/
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

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    uint32_t now_us = time_us_32();
    volatile uint32_t *last_event_us = (gpio == SW_MIN_LIMIT) ? &last_min_limit_event_us : &last_max_limit_event_us;
    const uint32_t debounce_window_us = 4500000; // 4500 ms debounce window -- the full movement takes 4.95 seconds

    if (now_us - *last_event_us >= debounce_window_us) {
        if (gpio == SW_MIN_LIMIT) {
            if (!pending_min_limit_event && !flag_min_limit_hit) {
                *last_event_us = now_us;
                pending_min_limit_event = true;
                if (vbs_should_log(5)) printf("[Motor DBG] gpio_limit_switches_callback: MIN pending at %lu us\n", (unsigned long)now_us);
            }
        } else if (gpio == SW_MAX_LIMIT) {
            if (!pending_max_limit_event && !flag_max_limit_hit) {
                *last_event_us = now_us;
                pending_max_limit_event = true;
                if (vbs_should_log(5)) printf("[Motor DBG] gpio_limit_switches_callback: MAX pending at %lu us\n", (unsigned long)now_us);
            }
        }
    } else {
        if (vbs_should_log(5)) printf("[Motor DBG] gpio_limit_switches_callback: ignored bounce on pin %u at %lu us\n", gpio, (unsigned long)now_us);
    }

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

/*
Desc: Confirm pending limit switch events and set limit fault flags.
params:
    - none
returns:
    - [void]
*/
void evaluate_pending_limit_switches(void) {
    if (pending_min_limit_event) {
        pending_min_limit_event = false;
        if (!gpio_get(SW_MIN_LIMIT)) {
            flag_min_limit_hit = true;
            if (vbs_should_log(2)) printf("[Motor] evaluate_pending_limit_switches: MIN confirmed\n");
        }
    }

    if (pending_max_limit_event) {
        pending_max_limit_event = false;
        if (!gpio_get(SW_MAX_LIMIT)) {
            flag_max_limit_hit = true;
            if (vbs_should_log(2)) printf("[Motor] evaluate_pending_limit_switches: MAX confirmed\n");
        }
    }
}

/*
Desc: Configure limit switch GPIOs and their interrupt callbacks on core 1.
params:
    - none
returns:
    - [void]
*/
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
    gpio_set_irq_enabled_with_callback(SW_MAX_LIMIT, GPIO_IRQ_EDGE_FALL, false, &gpio_limit_switches_callback);

    busy_wait_ms(250);

    gpio_set_irq_enabled_with_callback(SW_MIN_LIMIT, GPIO_IRQ_EDGE_FALL, true, &gpio_limit_switches_callback);
    gpio_set_irq_enabled_with_callback(SW_MAX_LIMIT, GPIO_IRQ_EDGE_FALL, true, &gpio_limit_switches_callback);
    limit_switches_ready = true;
}

/*
Desc: Handle a confirmed limit-switch fault by stopping motion and queuing recovery.
params:
    - [MotorControlState_t*] mctl_state: Pointer to current motor control state.
returns:
    - [void]
*/
void treat_fault_limit(MotorControlState_t *mctl_state) {
    if (flag_min_limit_hit) {
        printf("[Motor] **CRITICAL** Min limit switch hit! Recovering by queuing move 8188 steps (+1 direction)...\n");
        sys_state = SYS_CRITICAL_ERROR;
        sys_fault_code = FAULT_MIN_LIMIT_HIT;
        diag.fault_count[FAULT_MIN_LIMIT_HIT]++;
        stop_stepper_pio();
        xQueueReset(xMotorCmdQueue);
        if (mctl_state) *mctl_state = MCTL_IDLE;

        MotorCmd_t cmd = {0};
        cmd.cmd_type = MCTL_MOVING_PULSES;
        cmd.pulses = 8188;
        cmd.direction = 0;
        xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);

        flag_min_limit_hit = false;
        flag_max_limit_hit = false;
        pending_min_limit_event = false;
        pending_max_limit_event = false;
        sys_fault_code = FAULT_NONE;
        sys_state = SYS_OPERATIONAL;
        printf("[Motor] Min limit recovery queued: 8188 pulses.\n");
    } else if (flag_max_limit_hit) {
        printf("[Motor] **CRITICAL** Max limit switch hit! Recovering by queuing move 8188 steps (-1 direction)...\n");
        sys_state = SYS_CRITICAL_ERROR;
        sys_fault_code = FAULT_MAX_LIMIT_HIT;
        diag.fault_count[FAULT_MAX_LIMIT_HIT]++;
        stop_stepper_pio();
        xQueueReset(xMotorCmdQueue);
        if (mctl_state) *mctl_state = MCTL_IDLE;

        MotorCmd_t cmd = {0};
        cmd.cmd_type = MCTL_MOVING_PULSES;
        cmd.pulses = 8188;
        cmd.direction = 1;
        xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);

        flag_min_limit_hit = false;
        flag_max_limit_hit = false;
        pending_min_limit_event = false;
        pending_max_limit_event = false;
        sys_fault_code = FAULT_NONE;
        sys_state = SYS_OPERATIONAL;
        printf("[Motor] Max limit recovery queued: 8188 pulses.\n");
    }
}

/*
Desc: Execute closed-loop piston movement toward a potentiometer-based target.
params:
    - [VelocityGenerator_t*] vel_gen: Trajectory generator context.
    - [PIDController_t*] pid: PID controller context.
    - [float] dt: Loop time step in seconds.
    - [float] h_medido: Current piston position in mm.
    - [MotorControlState_t*] mctl_state: Current motor state, updated when complete.
returns:
    - [void]
*/
void potentiometer_movement(VelocityGenerator_t *vel_gen, PIDController_t *pid, float dt, float h_medido, MotorControlState_t *mctl_state) {
    float now_sec = (float)xTaskGetTickCount() * (1.0f / configTICK_RATE_HZ);
    TrajectoryPoint_t traj = velocity_generator_update(vel_gen, now_sec);
    float pos_error = traj.href - h_medido;
    float deadband_mm = 0.2f;
    uint16_t desired_pot = pistonPosToPot(traj.href);
    uint16_t target_pot = pistonPosToPot(vel_gen->h_target);
    float pid_output = pid_compute(pid, traj.href, h_medido, dt);
    float v_control = pid_output;
    
    printf("[Motor] Movement tick: h_medido=%.2f, h_target=%.2f, pos_error=%.2f, v_ref=%.4f mm/s, current_v=%.4f, desired_pot=%u, target_pot=%u, current_pot=%.4f\n",
           h_medido, traj.href, pos_error, traj.vref, v_control, desired_pot, target_pot, getPotValue());
    
    if (traj.is_completed && fabsf(pos_error) <= deadband_mm) {
        stop_stepper_pio();
        if (mctl_state) *mctl_state = MCTL_IDLE;
        diag.motor_move_complete++;
        printf("[Motor] Target reached within deadband +-%.2f mm; desired_pot=%u, target_pot=%u\n",
               deadband_mm, desired_pot, target_pot);
    } else {
        set_stepper_speed_mm_s(v_control);
    }
}

/*
Desc: Execute a pulse-count movement command using fixed speed and pulse consumption.
params:
    - [MotorCmd_t*] current_cmd: Current motor command with remaining pulses.
    - [float] dt: Loop time step in seconds.
    - [MotorControlState_t*] mctl_state: Current motor state, updated when complete.
returns:
    - [void]
*/
void pulses_movement(MotorCmd_t *current_cmd, float dt, MotorControlState_t *mctl_state) {
    static uint32_t pulses_total_init = 0;
    static uint32_t pulses_accel = 0;
    static uint32_t pulses_decel = 0;
    static bool pulses_profile_initialized = false;

    if (!current_cmd) return;

    if (current_cmd->pulses == 0) {
        stop_stepper_pio();
        if (mctl_state) *mctl_state = MCTL_IDLE;
        diag.motor_move_complete++;
        pulses_profile_initialized = false;
        if (vbs_should_log(4)) printf("[Motor] Pulse movement complete\n");
        return;
    }

    if (!pulses_profile_initialized) {
        pulses_total_init = current_cmd->pulses;
        pulses_accel = (uint32_t)ceilf((float)pulses_total_init * 0.10f);
        if (pulses_accel > 4000) pulses_accel = 4000;
        pulses_decel = pulses_accel;
        if (pulses_accel + pulses_decel > pulses_total_init) {
            pulses_accel = pulses_total_init / 2;
            pulses_decel = pulses_total_init - pulses_accel;
        }
        pulses_profile_initialized = true;
        if (vbs_should_log(4)) printf("[Motor] Pulse profile init: total=%lu accel=%lu decel=%lu\n",
                                    (unsigned long)pulses_total_init, (unsigned long)pulses_accel, (unsigned long)pulses_decel);
    }

    uint32_t pulses_remaining = current_cmd->pulses;
    if (vbs_should_log(4)) printf("[Motor] Pulse movement tick: remaining=%lu\n", (unsigned long)pulses_remaining);

    float base_vmax = (current_cmd->speed > 0) ? MOTOR_RPM_TO_MM_S(current_cmd->speed) : DEFAULT_VMAX_MM_S;
    float speed = base_vmax;
    float f_step = fabsf(speed) * STEPS_PER_MM;
    if (vbs_should_log(4)) printf("[Motor] Pulse movement: speed=%.4f mm/s, f_step=%.2f Hz, steps_this_tick=%.4f\n",
           speed, f_step, fabsf(speed) * STEPS_PER_MM * dt);
    set_stepper_speed_mm_s((current_cmd->direction == 0) ? speed : -speed);

    float steps_this_tick = fabsf(speed) * STEPS_PER_MM * dt;
    uint32_t consumed_steps = (uint32_t)steps_this_tick;
    if (consumed_steps == 0) consumed_steps = 1;

    if (current_cmd->pulses <= consumed_steps) {
        float final_pulses = (float)current_cmd->pulses;
        float final_time_s = final_pulses / f_step;
        uint32_t wait_us = (uint32_t)ceilf(final_time_s * 1000000.0f);
        if (wait_us > 0) {
            if (vbs_should_log(4)) printf("[Motor] Waiting %.3f ms for final %.0f pulses\n", wait_us / 1000.0f, final_pulses);
            busy_wait_us(wait_us); // ver se posso substituir
        }

        current_cmd->pulses = 0;
        if (mctl_state) *mctl_state = MCTL_IDLE;
        stop_stepper_pio();
        diag.motor_move_complete++;
        pulses_profile_initialized = false;
        if (vbs_should_log(4)) printf("[Motor] Pulse movement finished after final consumption\n");
    } else {
        current_cmd->pulses -= consumed_steps;
        if (vbs_should_log(4)) printf("[Motor] Pulse movement consumed=%lu new_remaining=%lu\n",
               (unsigned long)consumed_steps, (unsigned long)current_cmd->pulses);
    }
}

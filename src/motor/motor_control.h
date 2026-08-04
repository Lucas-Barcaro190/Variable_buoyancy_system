#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "src/core/shared_state.h"
#include "src/motor/potentiometer_reading.h"
#include "src/motor/velocity_generator.h"

#ifdef __cplusplus
extern "C" {
#endif

// Constante de conversão de mm para passos do motor de passo
// 200 passos/rev * 2 (meio passo) * 61.417 (redução) / 3.0mm (passo do fuso) = ~8188.933 passos/mm
#define STEPS_PER_MM    (200.0f * 2.0f * 61.417f / 3.0f)

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} PIDController_t;

// Funções do Controlador PID
void pid_init(PIDController_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
void pid_reset(PIDController_t *pid);
float pid_compute(PIDController_t *pid, float setpoint, float measurement, float dt);

// Funções da PIO e Motor de Passo
void setup_stepper_pio(void);
void set_stepper_period(uint32_t period_x);
void set_stepper_speed_mm_s(float speed_mm_s);
void stop_stepper_pio(void);

void sendStopCommand(void);
void sendEnableCommand(void);
void sendDisableCommand(void);

// Chaves de Fim de Curso
void setup_limit_switches_on_core1(void);
void gpio_limit_switches_callback(uint gpio, uint32_t events);
void evaluate_pending_limit_switches(void);

#ifdef __cplusplus
}
#endif

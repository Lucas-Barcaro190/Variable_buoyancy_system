#pragma once

#include <stdint.h>
#include <stddef.h>

// Hardware pins
#define PIN_MOTOR_PULSE             4
#define PIN_MOTOR_DIR               5
#define PIN_ENABLE_DRIVER           12
#define SW_MIN_LIMIT                3
#define SW_MAX_LIMIT                2
#define POT_ADC_PIN                 26
#define POT_ADC_CHANNEL             0
#define POT_SAMPLE_COUNT            128

// Potentiometer and motion limits
#define MINIMAL_THRESHOLD           43
#define MAXIMUM_THRESHOLD           435
#define POT_RANGE                   ((float)(MAXIMUM_THRESHOLD - MINIMAL_THRESHOLD))
#define PC_TIMEOUT_MS               300000
#define RECOMMENDED_SPEED_VAL       32
#define MAX_SPEED_VAL               127
#define MAX_PISTON_POSITION         23.0f
#define MAX_VOLUME                  300.0f
#define MAX_PULSES                  200.f * 2.f * 61.417f * 23.f / 3.f
#define VOL_MULTIPLIER              (3.1415f * (14.0f / 2.0f) * (14.0f / 2.0f))
#define PISTON_RANGE                (2.0f * MAX_PISTON_POSITION)

// RTOS sizing
#define MOTOR_CONTROL_STACK_SIZE    (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define PARSER_STACK_SIZE           (512 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define FAULT_MGR_STACK_SIZE        (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define DIAGNOSTICS_STACK_SIZE      (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define MOTOR_CMD_QUEUE_SIZE        5
#define VBS_SPINLOCK_ID             16

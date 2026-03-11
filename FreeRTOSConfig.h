#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION                    1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      125000000
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    10
#define configMINIMAL_STACK_SIZE                (128)
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configTOTAL_HEAP_SIZE                   (64 * 1024)
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0

/* Suporte para o Raspberry Pi Pico (SMP - Opcional) */
#define configNUMBER_OF_CORES                   1

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            1024

#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1

#define configSUPPORT_PICO_SYNC_INTEROP         1
#define configSUPPORT_PICO_I2C_INTEROP          1
#define configUSE_EVENT_GROUPS                  1
#define INCLUDE_xTimerPendFunctionCall          1
#define configUSE_TIMERS                        1

/* Necessário para o porte do RP2040 */
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1

#endif
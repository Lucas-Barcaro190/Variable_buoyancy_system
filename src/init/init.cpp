#include "src/init/init.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "pico/mutex.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "src/core/shared_state.h"
#include "src/motor/motor_control.h"
#include "src/tasks/task_handlers.h"

extern "C" {
    uint32_t __real_spin_lock_blocking(spin_lock_t *lock);
    uint32_t __wrap_spin_lock_blocking(spin_lock_t *lock) {
        if (!lock) {
            static uint32_t dummy_spinlock_wrap = 0xFFFFFFFF;
            lock = (spin_lock_t *)&dummy_spinlock_wrap;
        }
        return __real_spin_lock_blocking(lock);
    }
}

void initialize_hardware_sync(void) {
    static uint32_t dummy_spinlock = 0xFFFFFFFF;
    vbs_shared_lock = (spin_lock_t *)&dummy_spinlock;
}

extern "C" {
    extern mutex_t __mutex_array_start;
    extern mutex_t __mutex_array_end;
}

void initializeHardware(void) {
    spin_locks_reset();
    mutex_t *start = &__mutex_array_start;
    mutex_t *end = &__mutex_array_end;
    for (mutex_t *m = start; m < end; m++) {
        mutex_init(m);
    }
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== VBS v3.0 Hardware Initialization ===\n");
    initialize_hardware_sync();
    printf("[HW] Hardware synchronization spinlocks initialized\n");

    setup_limit_switches_on_core1();
    setup_stepper_pio();
    printf("[HW] Stepper motor PIO initialized (PUL = GPIO4, DIR = GPIO5)\n");

    gpio_init(PIN_ENABLE_DRIVER);
    gpio_set_dir(PIN_ENABLE_DRIVER, GPIO_OUT);
    gpio_put(PIN_ENABLE_DRIVER, 0);
    printf("[HW] Motor driver enabled pin configured\n");

    printf("=== Hardware Initialization Complete ===\n\n");
}

void initializeRTOS(void) {
    printf("=== RTOS Initialization ===\n");
    printf("[RTOS] Potentiometer access protected by hardware spinlocks\n");

    xMotorCmdQueue = xQueueCreateStatic(
        MOTOR_CMD_QUEUE_SIZE,
        sizeof(MotorCmd_t),
        ucMotorCmdQueueStorage,
        &xMotorCmdQueueHandle
    );
    if (xMotorCmdQueue == NULL) {
        printf("[RTOS] ERROR: Failed to create motor command queue\n");
        return;
    }
    printf("[RTOS] Motor command queue created (%d items)\n", MOTOR_CMD_QUEUE_SIZE);

    xMotorControlTaskHandle = xTaskCreateStatic(
        vMotorControlTask,
        "MotorControl",
        MOTOR_CONTROL_STACK_SIZE,
        NULL,
        6,
        xMotorControlStack,
        &xMotorControlTCB
    );
    if (xMotorControlTaskHandle == NULL) {
        printf("[RTOS] ERROR: Failed to create vMotorControlTask\n");
        return;
    }
    vTaskCoreAffinitySet(xMotorControlTaskHandle, (1 << 0) | (1 << 1));
    printf("[RTOS] vMotorControlTask created statically (priority 6)\n");

    xParserTaskHandle = xTaskCreateStatic(
        vParserTask,
        "Parser",
        PARSER_STACK_SIZE,
        NULL,
        4,
        xParserStack,
        &xParserTCB
    );
    if (xParserTaskHandle == NULL) {
        printf("[RTOS] ERROR: Failed to create vParserTask\n");
        return;
    }
    vTaskCoreAffinitySet(xParserTaskHandle, (1 << 0) | (1 << 1));
    printf("[RTOS] vParserTask created statically (priority 4)\n");

    xFaultMgrTaskHandle = xTaskCreateStatic(
        vFaultManagerTask,
        "FaultManager",
        FAULT_MGR_STACK_SIZE,
        NULL,
        3,
        xFaultMgrStack,
        &xFaultMgrTCB
    );
    if (xFaultMgrTaskHandle == NULL) {
        printf("[RTOS] ERROR: Failed to create vFaultManagerTask\n");
        return;
    }
    vTaskCoreAffinitySet(xFaultMgrTaskHandle, (1 << 0) | (1 << 1));
    printf("[RTOS] vFaultManagerTask created statically (priority 3)\n");

    xDiagnosticsTaskHandle = xTaskCreateStatic(
        vDiagnosticsTask,
        "Diagnostics",
        DIAGNOSTICS_STACK_SIZE,
        NULL,
        1,
        xDiagnosticsStack,
        &xDiagnosticsTCB
    );
    if (xDiagnosticsTaskHandle == NULL) {
        printf("[RTOS] ERROR: Failed to create vDiagnosticsTask\n");
        return;
    }
    vTaskCoreAffinitySet(xDiagnosticsTaskHandle, (1 << 0) | (1 << 1));
    printf("[RTOS] vDiagnosticsTask created statically (priority 1)\n");

    printf("=== RTOS Initialization Complete ===\n\n");
    last_pc_heartbeat_ms = xTaskGetTickCount();
}

extern "C" {
    void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                        StackType_t **ppxIdleTaskStackBuffer,
                                        configSTACK_DEPTH_TYPE *puxIdleTaskStackSize) {
        static StaticTask_t xIdleTaskTCB;
        static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

        *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
        *ppxIdleTaskStackBuffer = uxIdleTaskStack;
        *puxIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    }

    void vApplicationGetPassiveIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                               StackType_t **ppxIdleTaskStackBuffer,
                                               configSTACK_DEPTH_TYPE *puxIdleTaskStackSize,
                                               BaseType_t xCoreID) {
        static StaticTask_t xPassiveIdleTaskTCBs[configNUMBER_OF_CORES - 1];
        static StackType_t uxPassiveIdleTaskStacks[configNUMBER_OF_CORES - 1][configMINIMAL_STACK_SIZE];

        if (xCoreID < (configNUMBER_OF_CORES - 1)) {
            *ppxIdleTaskTCBBuffer = &(xPassiveIdleTaskTCBs[xCoreID]);
            *ppxIdleTaskStackBuffer = &(uxPassiveIdleTaskStacks[xCoreID][0]);
            *puxIdleTaskStackSize = configMINIMAL_STACK_SIZE;
        }
    }

    void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                         StackType_t **ppxTimerTaskStackBuffer,
                                         configSTACK_DEPTH_TYPE *puxTimerTaskStackSize) {
        static StaticTask_t xTimerTaskTCB;
        static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

        *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
        *ppxTimerTaskStackBuffer = uxTimerTaskStack;
        *puxTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    }
}

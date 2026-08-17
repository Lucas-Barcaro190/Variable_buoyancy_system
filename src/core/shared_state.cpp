#include "src/core/shared_state.h"

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

volatile SystemState_t sys_state = SYS_INIT;
volatile FaultCode_t sys_fault_code = FAULT_NONE;
volatile uint32_t last_pc_heartbeat_ms = 0;
volatile uint16_t potentiometer_value = 0;
volatile int16_t currentPistonPosition = 0;
volatile float currentVolume = 0.0f;
volatile Diagnostics_t diag = {0};
volatile uint8_t verbose_level = 0;
volatile uint16_t target_pot_shared = 196;
volatile float pid_Kp = 1.0f;
volatile float pid_Ki = 1.0f;
volatile float pid_Kd = 0.0f;
volatile bool flag_min_limit_hit = false;
volatile bool flag_max_limit_hit = false;
volatile bool limit_switches_ready = false;
volatile bool pending_min_limit_event = false;
volatile bool pending_max_limit_event = false;
volatile uint32_t last_min_limit_event_us = 0;
volatile uint32_t last_max_limit_event_us = 0;
volatile uint8_t vbsAddress = 0xE0;
static uint32_t default_dummy_spinlock_hardware = 0xFFFFFFFF;
spin_lock_t* vbs_shared_lock = (spin_lock_t*)&default_dummy_spinlock_hardware;
QueueHandle_t xMotorCmdQueue = NULL;
TaskHandle_t xMotorControlTaskHandle = NULL;
TaskHandle_t xParserTaskHandle = NULL;
TaskHandle_t xFaultMgrTaskHandle = NULL;
TaskHandle_t xDiagnosticsTaskHandle = NULL;
StaticTask_t xMotorControlTCB;
StaticTask_t xParserTCB;
StaticTask_t xFaultMgrTCB;
StaticTask_t xDiagnosticsTCB;
StackType_t xMotorControlStack[MOTOR_CONTROL_STACK_SIZE];
StackType_t xParserStack[PARSER_STACK_SIZE];
StackType_t xFaultMgrStack[FAULT_MGR_STACK_SIZE];
StackType_t xDiagnosticsStack[DIAGNOSTICS_STACK_SIZE];
StaticQueue_t xMotorCmdQueueHandle;
uint8_t ucMotorCmdQueueStorage[MOTOR_CMD_QUEUE_SIZE * sizeof(MotorCmd_t)];

/*
Desc: Convert a system state enum value to a human-readable string.
params:
    - [SystemState_t] state: State value to convert.
returns:
    - [const char*]: String representing the state.
*/
const char* stateToString(SystemState_t state) {
    switch (state) {
    case SYS_INIT: return "INIT";
    case SYS_OPERATIONAL: return "OPERATIONAL";
    case SYS_FAILSAFE_ASCENT: return "FAILSAFE_ASCENT";
    case SYS_CRITICAL_ERROR: return "CRITICAL_ERROR";
    case SYS_CALIBRATION_MIN: return "CALIBRATION_MIN";
    case SYS_CALIBRATION_MAX: return "CALIBRATION_MAX";
    case SYS_MANUAL_CONTROL: return "MANUAL_CONTROL";
    default: return "UNKNOWN";
    }
}

/*
Desc: Convert a fault code enum value to a human-readable string.
params:
    - [FaultCode_t] fault: Fault code to convert.
returns:
    - [const char*]: String representing the fault code.
*/
const char* faultToString(FaultCode_t fault) {
    switch (fault) {
    case FAULT_NONE: return "NONE";
    case FAULT_MIN_LIMIT_HIT: return "MIN_LIMIT_HIT";
    case FAULT_MAX_LIMIT_HIT: return "MAX_LIMIT_HIT";
    case FAULT_PC_TIMEOUT: return "PC_TIMEOUT";
    case FAULT_MOTOR_STALL: return "MOTOR_STALL";
    case FAULT_QUEUE_OVERFLOW: return "QUEUE_OVERFLOW";
    default: return "UNKNOWN";
    }
}

/*
Desc: Return the last sampled potentiometer value stored in shared state.
params:
    - none
returns:
    - [uint16_t]: Current shared potentiometer reading.
*/
uint16_t getPotValue(void) {
    uint16_t value;
    taskENTER_CRITICAL();
    value = potentiometer_value;
    taskEXIT_CRITICAL();
    return value;
}

/*
Desc: Update the shared potentiometer value in global state.
params:
    - [uint16_t] val: New potentiometer value.
returns:
    - [uint16_t]: The value written to shared state.
*/
uint16_t setPotValue(uint16_t val) {
    taskENTER_CRITICAL();
    potentiometer_value = val;
    taskEXIT_CRITICAL();
    return val;
}

/*
Desc: Get the current target potentiometer value used by motor commands.
params:
    - none
returns:
    - [uint16_t]: Currently stored target potentiometer command value.
*/
uint16_t get_target_pot_value(void) {
    uint16_t value;
    taskENTER_CRITICAL();
    value = target_pot_shared;
    taskEXIT_CRITICAL();
    return value;
}

/*
Desc: Store a new target potentiometer value used by motor commands.
params:
    - [uint16_t] val: New target potentiometer value.
returns:
    - [void]
*/
void set_target_pot_value(uint16_t val) {
    taskENTER_CRITICAL();
    target_pot_shared = val;
    taskEXIT_CRITICAL();
}

/*
Desc: Safely change the device communication address in shared state.
params:
    - [uint8_t] newAddress: New address value for the device.
returns:
    - [void]
*/
void changeAddress(uint8_t newAddress) {
    taskENTER_CRITICAL();
    vbsAddress = newAddress;
    taskEXIT_CRITICAL();
}

/*
Desc: Safely read the current device communication address from shared state.
params:
    - none
returns:
    - [uint8_t]: Current device address.
*/
uint8_t getAddress(void) {
    uint8_t address;
    taskENTER_CRITICAL();
    address = vbsAddress;
    taskEXIT_CRITICAL();
    return address;
}

/*
Desc: Decide whether a given debug verbosity level should be logged.
params:
    - [uint8_t] level: Verbosity level for the message.
returns:
    - [bool]: True if the message should be emitted.
*/
bool vbs_should_log(uint8_t level) {
    return (verbose_level == 6) || (verbose_level == level);
}

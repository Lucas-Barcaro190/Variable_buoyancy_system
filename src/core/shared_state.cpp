#include "src/core/shared_state.h"

#include <string.h>

volatile SystemState_t sys_state = SYS_INIT;
volatile FaultCode_t sys_fault_code = FAULT_NONE;
volatile uint32_t last_pc_heartbeat_ms = 0;
volatile uint16_t potentiometer_value = 0;
volatile int16_t currentPistonPosition = 0;
volatile float currentVolume = 0.0f;
volatile Diagnostics_t diag = {0};
volatile uint8_t verbose_level = 0;
volatile uint16_t target_pot_shared = 196;
volatile bool flag_min_limit_hit = false;
volatile bool flag_max_limit_hit = false;
volatile bool limit_switches_ready = false;
volatile bool pending_min_limit_event = false;
volatile bool pending_max_limit_event = false;
volatile uint32_t last_min_limit_event_us = 0;
volatile uint32_t last_max_limit_event_us = 0;
volatile uint8_t vbsAddress = 0xE0;
spin_lock_t* vbs_shared_lock = nullptr;
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

uint16_t getPotValue(void) {
    uint32_t flags = spin_lock_blocking(vbs_shared_lock);
    uint16_t val = potentiometer_value;
    spin_unlock(vbs_shared_lock, flags);
    return val;
}

uint16_t setPotValue(uint16_t val) {
    uint32_t flags = spin_lock_blocking(vbs_shared_lock);
    potentiometer_value = val;
    spin_unlock(vbs_shared_lock, flags);
    return val;
}

uint16_t get_target_pot_value(void) {
    uint32_t flags = spin_lock_blocking(vbs_shared_lock);
    uint16_t val = target_pot_shared;
    spin_unlock(vbs_shared_lock, flags);
    return val;
}

void set_target_pot_value(uint16_t val) {
    uint32_t flags = spin_lock_blocking(vbs_shared_lock);
    target_pot_shared = val;
    spin_unlock(vbs_shared_lock, flags);
}

void changeAddress(uint8_t newAddress) {
    taskENTER_CRITICAL();
    vbsAddress = newAddress;
    taskEXIT_CRITICAL();
}

uint8_t getAddress(void) {
    uint8_t address;
    taskENTER_CRITICAL();
    address = vbsAddress;
    taskEXIT_CRITICAL();
    return address;
}

bool vbs_should_log(uint8_t level) {
    return (verbose_level == 6) || (verbose_level == level);
}

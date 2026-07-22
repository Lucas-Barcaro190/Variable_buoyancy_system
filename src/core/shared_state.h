#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "src/config/hardware_config.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "hardware/sync.h"

typedef enum {
    SYS_INIT,
    SYS_OPERATIONAL,
    SYS_FAILSAFE_ASCENT,
    SYS_CRITICAL_ERROR,
    SYS_CALIBRATION_MIN,
    SYS_CALIBRATION_MAX,
    SYS_MANUAL_CONTROL,
} SystemState_t;

typedef enum {
    FAULT_NONE = 0x00,
    FAULT_MIN_LIMIT_HIT = 0xE1,
    FAULT_MAX_LIMIT_HIT = 0xE2,
    FAULT_PC_TIMEOUT = 0xE3,
    FAULT_MOTOR_STALL = 0xE6,
    FAULT_QUEUE_OVERFLOW = 0xE7,
} FaultCode_t;

typedef enum {
    MCTL_IDLE = 0,
    MCTL_MOVING_ABSOLUTE,
    MCTL_MOVING_UNTIL_POT,
    MCTL_MOVING_PULSES,
} MotorControlState_t;

typedef struct {
    uint8_t cmd_type;
    uint16_t target_pot;
    uint32_t pulses;
    uint8_t direction;
    uint32_t cmd_id;
    uint16_t speed;
} MotorCmd_t;

typedef struct {
    uint8_t fault_code;
    uint32_t timestamp_ms;
} FaultNotif_t;

typedef struct {
    uint32_t motor_cmd_queued;
    uint32_t motor_move_complete;
    uint32_t motor_move_timeout;
    uint32_t fault_count[256];
    uint32_t pc_heartbeats;
} Diagnostics_t;

extern volatile SystemState_t sys_state;
extern volatile FaultCode_t sys_fault_code;
extern volatile uint32_t last_pc_heartbeat_ms;
extern volatile uint16_t potentiometer_value;
extern volatile int16_t currentPistonPosition;
extern volatile float currentVolume;
extern volatile Diagnostics_t diag;
extern volatile uint8_t verbose_level;
extern volatile uint16_t target_pot_shared;
extern volatile bool flag_min_limit_hit;
extern volatile bool flag_max_limit_hit;
extern volatile bool limit_switches_ready;
extern volatile bool pending_min_limit_event;
extern volatile bool pending_max_limit_event;
extern volatile uint32_t last_min_limit_event_us;
extern volatile uint32_t last_max_limit_event_us;
extern volatile uint8_t vbsAddress;
extern spin_lock_t* vbs_shared_lock;
extern QueueHandle_t xMotorCmdQueue;

bool vbs_should_log(uint8_t level);
extern TaskHandle_t xMotorControlTaskHandle;
extern TaskHandle_t xParserTaskHandle;
extern TaskHandle_t xFaultMgrTaskHandle;
extern TaskHandle_t xDiagnosticsTaskHandle;
extern StaticTask_t xMotorControlTCB;
extern StaticTask_t xParserTCB;
extern StaticTask_t xFaultMgrTCB;
extern StaticTask_t xDiagnosticsTCB;
extern StackType_t xMotorControlStack[];
extern StackType_t xParserStack[];
extern StackType_t xFaultMgrStack[];
extern StackType_t xDiagnosticsStack[];
extern StaticQueue_t xMotorCmdQueueHandle;
extern uint8_t ucMotorCmdQueueStorage[];

uint16_t getPotValue(void);
uint16_t setPotValue(uint16_t val);
uint16_t get_target_pot_value(void);
void set_target_pot_value(uint16_t val);
void changeAddress(uint8_t newAddress);
uint8_t getAddress(void);
const char* stateToString(SystemState_t state);
const char* faultToString(FaultCode_t fault);

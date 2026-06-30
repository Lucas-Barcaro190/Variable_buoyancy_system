/*
 * Variable Buoyancy System (VBS) v2.0 - Refactored RTOS Implementation
 * 
 * Phase 1: Foundations (static allocation, structs, stream buffer UART RX)
 * 
 * Architecture:
 * - ISR_RxDriver (ISR) → Stream buffer
 * - ISR_LimitSwitches (ISR) → Task notifications
 * - vMotorControlTask (priority 6)
 * - vReceiverTask (priority 5) 
 * - vParserTask (priority 4)
 * - vPotentiometerTask (priority 3)
 * - vFaultManagerTask (priority 2)
 * - vDiagnosticsTask (priority 1)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "stepper_pulse.pio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "semphr.h"
#include <climits>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define PIN_MOTOR_PULSE             4           // GPIO4 for Step/Pulse (PIO side-set)
#define PIN_MOTOR_DIR               5           // GPIO5 for Direction (GPIO)
#define PIN_ENABLE_DRIVER           16          // High = Driver OFF, Low = Driver ON
#define SW_MIN_LIMIT                3           // Contracted switch for minimum limit
#define SW_MAX_LIMIT                2           // Expanded switch for maximum limit
#define POT_ADC_PIN                 26          // GPIO26 (ADC0)
#define POT_ADC_CHANNEL             0           // ADC Channel 0
#define POT_SAMPLE_COUNT            128

#define MINIMAL_THRESHOLD           43          // Minimum potentiometer value to consider valid
#define MAXIMUM_THRESHOLD           435         // Maximum potentiometer value
#define POT_RANGE                   ((float)(MAXIMUM_THRESHOLD - MINIMAL_THRESHOLD))

#define RECOMMENDED_SPEED_VAL       32
#define MAX_SPEED_VAL               127

// PC Timeout for failsafe (300 seconds = 5 minutes)
#define PC_TIMEOUT_MS               300000

// Physical limits of buoyancy engine
#define MAX_PISTON_POSITION         23.0f       // Max travel from midpoint (mm)
#define MAX_VOLUME                  300.0f      // Max volume change (cm^3)

#define MAX_PULSES                  200.f * 2.f * 61.417f * 23.f / 3.f       // for microstep=2 && 23mm course

#define VOL_MULTIPLIER              (3.1415 * (14.0f / 2.0f) * (14.0f / 2.0f)) // 140mm diameter
#define PISTON_RANGE                (2.0f * MAX_PISTON_POSITION)

// Controle de Spinlock do Alvo do Motor e Sincronização Segura
#define VBS_SPINLOCK_ID             16
static spin_lock_t* vbs_shared_lock = nullptr;
static volatile uint16_t target_pot_shared = 196;
static volatile bool flag_min_limit_hit = false;
static volatile bool flag_max_limit_hit = false;
static volatile uint8_t vbsAddress = 0xE0;

// ============================================================================
// ENUMS & STATE DEFINITIONS
// ============================================================================

typedef enum {
    SYS_INIT,                    // Startup, running calibration
    SYS_OPERATIONAL,             // Normal operation, PC connected
    SYS_FAILSAFE_ASCENT,         // PC timeout → auto-expand
    SYS_CRITICAL_ERROR,          // Unrecoverable fault (e.g., limit hit)
    SYS_CALIBRATION_MIN,         // Seeking min limit
    SYS_CALIBRATION_MAX,         // Seeking max limit
    SYS_MANUAL_CONTROL,          // Ad-hoc move command
} SystemState_t;

typedef enum {
    FAULT_NONE                  =   0x00,
    FAULT_MIN_LIMIT_HIT         =   0xE1,
    FAULT_MAX_LIMIT_HIT         =   0xE2,
    FAULT_PC_TIMEOUT            =   0xE3,
    FAULT_MOTOR_STALL           =   0xE6,
    FAULT_QUEUE_OVERFLOW        =   0xE7,
} FaultCode_t;

typedef enum {
    MCTL_IDLE                   =   0,
    MCTL_MOVING_ABSOLUTE,
    MCTL_MOVING_UNTIL_POT,
    MCTL_MOVING_PULSES,
} MotorControlState_t;

// Task notification bits
#define NOTIF_MIN_LIMIT             (1 << 0)
#define NOTIF_MAX_LIMIT             (1 << 1)
#define NOTIF_FAULT_DETECTED        (1 << 2)

// ============================================================================
// MESSAGE & COMMAND STRUCTURES
// ============================================================================

// --- Commands & Message Types from MAREs ---
typedef enum {
    // Config messages
    CMD_ACK_CMD            = 0x10, // ACK for received command (data: command_valid ? 0x00 : NACK cmd)
    CMD_ACK_MOVE           = 0x12, // ACK for finalized movement (data: 0x00 ACK / 0x01 NACK)
    CMD_TIMEOUT_EMERGENCY  = 0x14, // Timeout in seconds for emergency activation
    CMD_CHANGE_ID          = 0x1A, // Change ID
    CMD_PING_PONG          = 0x1F, // Ping-pong (default: 0xE0)

    // Movement messages
    CMD_FULL_CONTRACT      = 0x20, // Full contract (data: 1 byte)
    CMD_FULL_EXPAND        = 0x2F, // Full expand (data: 1 byte)
    CMD_ABS_VOLUME         = 0x21, // Absolute volume in cm^3 (data: FP32 volume)
    CMD_REL_VOLUME         = 0x23, // Relative volume in cm^3 (data: FP32 volume)
    CMD_REQ_VOLUME         = 0x24, // Request current volume (data: FP32 volume)
    CMD_ABS_PISTON         = 0x29, // Absolute piston position in mm (data: FP32 pos)
    CMD_REL_PISTON         = 0x2B, // Relative piston position in mm (data: FP32 pos)
    CMD_REQ_PISTON         = 0x2C, // Request current piston position (data: FP32 pos)

    // Error messages
    CMD_ERR_OUT_OF_BOUNDS  = 0x1D  // Out of bounds error
} MsgType_t;

/**
 * @brief Combined 16-bit message type codes where:
 *        - High byte: payload size in bytes
 *        - Low byte: message/command type
 */
typedef enum {
    // Config messages
    MSG_ACK_CMD            = 0x0210,
    MSG_ACK_MOVE           = 0x0112,
    MSG_TIMEOUT_EMERGENCY  = 0x0414,
    MSG_CHANGE_ID          = 0x011A,
    MSG_PING_PONG          = 0x011F,

    // Movement messages
    MSG_FULL_CONTRACT      = 0x0120,
    MSG_FULL_EXPAND        = 0x012F,
    MSG_ABS_VOLUME         = 0x0421,
    MSG_REL_VOLUME         = 0x0423,
    MSG_REQ_VOLUME         = 0x0424,
    MSG_ABS_PISTON         = 0x0429,
    MSG_REL_PISTON         = 0x042B,
    MSG_REQ_PISTON         = 0x042C,

    // Error messages
    MSG_ERR_OUT_OF_BOUNDS  = 0x011D
} MsgTypeCode_t;

// Helper functions to decode/encode msg types
static inline uint8_t getMsgSize(uint16_t msg_type_code) {
    return (uint8_t)((msg_type_code >> 8) & 0xFF);
}

static inline uint8_t getMsgType(uint16_t msg_type_code) {
    return (uint8_t)(msg_type_code & 0xFF);
}

static inline uint16_t makeMsgTypeCode(uint8_t size, uint8_t type) {
    return (uint16_t)((size << 8) | type);
}

// --- Message Packet Structure ---

/**
 * @brief Serialized message packet structure as transmitted over wire.
 */
typedef struct __attribute__((packed)) {
    uint8_t crc;          // CRC-8 checksum
    uint8_t address;      // Device address/ID (default 0xE0)
    uint8_t size;         // Size of msgData payload (high byte of 16-bit type)
    uint8_t msgType;      // Message command type identifier (low byte of 16-bit type)
    uint8_t msgData[];    // Flexible array member for payload
} messages_t;

// --- Payload-Specific Structs ---

typedef struct __attribute__((packed)) {
    uint8_t status;       // 0x00 if valid, command code that NACK'd if invalid
    uint8_t command_code; // The command code this ACK/NACK is for
} MsgDataAckCmd_t;

typedef struct __attribute__((packed)) {
    uint8_t status;       // 0x00 for ACK (success), 0x01 for NACK (failure)
} MsgDataAckMove_t;

typedef struct __attribute__((packed)) {
    uint32_t timeout_sec; // Emergency timeout in seconds (big-endian)
} MsgDataTimeoutEmergency_t;

typedef struct __attribute__((packed)) {
    uint8_t new_id;       // New device ID
} MsgDataChangeId_t;

typedef struct __attribute__((packed)) {
    uint8_t id;           // Device ID
} MsgDataPingPong_t;

typedef struct __attribute__((packed)) {
    uint8_t value;        // Data byte
} MsgDataFullMove_t;

typedef struct __attribute__((packed)) {
    float volume;         // Volume in cm^3 (FP32, big-endian)
} MsgDataVolume_t;

typedef struct __attribute__((packed)) {
    float position;       // Position in mm (FP32, big-endian)
} MsgDataPiston_t;

typedef struct __attribute__((packed)) {
    uint8_t causing_cmd;  // Command code that caused the error
} MsgDataErrOutOfBounds_t;

static inline void changeAddress(uint8_t newAddress){
    taskENTER_CRITICAL();
    vbsAddress = newAddress;
    taskEXIT_CRITICAL();
}

static inline uint8_t getAddress(void){
    uint8_t address;
    taskENTER_CRITICAL();
    address = vbsAddress;
    taskEXIT_CRITICAL();
    return address;
}

void sendBinaryPacket(uint8_t msgType, const uint8_t* data, uint8_t size);

typedef struct {
    uint8_t cmd_type;              // MotorControlState_t value
    uint16_t target_pot;           // Target potentiometer value
    uint32_t pulses;               // Number of pulses to send
    uint8_t direction;             // Direction to move (0 or 1)
    uint32_t cmd_id;               // Command ID (optional)
    uint16_t speed;                // Speed (optional/unused)
} MotorCmd_t;

typedef struct {
    uint8_t fault_code;            // FaultCode_t value
    uint32_t timestamp_ms;         // When fault occurred
} FaultNotif_t;

typedef struct {
    uint32_t motor_cmd_queued;
    uint32_t motor_move_complete;
    uint32_t motor_move_timeout;
    uint32_t fault_count[256];     // One counter per fault code
    uint32_t pc_heartbeats;
} Diagnostics_t;

// ============================================================================
// STATIC TASK ALLOCATION
// ============================================================================

// Task stacks (sizes from refactor plan)
#define MOTOR_CONTROL_STACK_SIZE    (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define PARSER_STACK_SIZE           (512 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define FAULT_MGR_STACK_SIZE        (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define DIAGNOSTICS_STACK_SIZE      (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))

static StackType_t xMotorControlStack[MOTOR_CONTROL_STACK_SIZE];
static StackType_t xParserStack[PARSER_STACK_SIZE];
static StackType_t xFaultMgrStack[FAULT_MGR_STACK_SIZE];
static StackType_t xDiagnosticsStack[DIAGNOSTICS_STACK_SIZE];

static StaticTask_t xMotorControlTCB;
static StaticTask_t xParserTCB;
static StaticTask_t xFaultMgrTCB;
static StaticTask_t xDiagnosticsTCB;

// Task handles
TaskHandle_t xMotorControlTaskHandle    =   NULL;
TaskHandle_t xParserTaskHandle          =   NULL;
TaskHandle_t xFaultMgrTaskHandle        =   NULL;
TaskHandle_t xDiagnosticsTaskHandle     =   NULL;

// ============================================================================
// STATIC QUEUE ALLOCATION
// ============================================================================

// Motor Command Queue (Parser → Motor Control)
#define MOTOR_CMD_QUEUE_SIZE            5
static uint8_t ucMotorCmdQueueStorage[MOTOR_CMD_QUEUE_SIZE * sizeof(MotorCmd_t)];
static StaticQueue_t xMotorCmdQueueHandle;
QueueHandle_t xMotorCmdQueue            =   NULL;

// ============================================================================
// SYNCHRONIZATION PRIMITIVES
// ============================================================================

// Mutex for potentiometer_value access (protects shared state)
StaticSemaphore_t xPotMutexBuffer; // kept for compatibility (unused -- check if possible to remove)
SemaphoreHandle_t xPotMutex             =   NULL; // unused

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

volatile SystemState_t sys_state        =   SYS_INIT;
volatile FaultCode_t sys_fault_code     =   FAULT_NONE;
volatile uint32_t last_pc_heartbeat_ms  =   0;

// Protected by xPotMutex
volatile uint16_t potentiometer_value   =   0;
volatile int16_t currentPistonPosition  =   0;
volatile float currentVolume            =   0.0f;

// Diagnostics (non-critical; can be accessed without locks for logging)
volatile Diagnostics_t diag             =   {0};

// Verbose debug level: 0=off (default), 1=Motor, 2=Receiver, 3=Parser,
// 4=Potentiometer, 5=FaultManager/Diagnostics, 6=all
volatile uint8_t verbose_level          =   0;

static inline bool vbs_should_log(uint8_t level) {
    return (verbose_level == 6) || (verbose_level == level);
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static const char* stateToString(SystemState_t state) {
    switch (state) {
    case SYS_INIT:                      return "INIT";
    case SYS_OPERATIONAL:               return "OPERATIONAL";
    case SYS_FAILSAFE_ASCENT:           return "FAILSAFE_ASCENT";
    case SYS_CRITICAL_ERROR:            return "CRITICAL_ERROR";
    case SYS_CALIBRATION_MIN:           return "CALIBRATION_MIN";
    case SYS_CALIBRATION_MAX:           return "CALIBRATION_MAX";
    case SYS_MANUAL_CONTROL:            return "MANUAL_CONTROL";
    default:                            return "UNKNOWN";
    }
}

static const char* faultToString(FaultCode_t fault) {
    switch (fault) {
    case FAULT_NONE:                    return "NONE";
    case FAULT_MIN_LIMIT_HIT:           return "MIN_LIMIT_HIT";
    case FAULT_MAX_LIMIT_HIT:           return "MAX_LIMIT_HIT";
    case FAULT_PC_TIMEOUT:              return "PC_TIMEOUT";
    case FAULT_MOTOR_STALL:             return "MOTOR_STALL";
    case FAULT_QUEUE_OVERFLOW:          return "QUEUE_OVERFLOW";
    default:                            return "UNKNOWN";
    }
}

uint16_t getPotValue(void) {
    uint32_t flags = spin_lock_blocking(vbs_shared_lock);
    uint16_t val = potentiometer_value;
    spin_unlock(vbs_shared_lock, flags);
    return val;
}

void setPotValue(uint16_t val) {
    uint32_t flags = spin_lock_blocking(vbs_shared_lock);
    potentiometer_value = val;
    spin_unlock(vbs_shared_lock, flags);
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

// ============================================================================
// STEPPER PIO CONFIGURATION & CONTROL
// ============================================================================

static PIO stepper_pio = pio0;
static uint stepper_sm = 0;
static uint stepper_offset = 0;

void setup_stepper_pio(void) {
    stepper_offset = pio_add_program(stepper_pio, &stepper_pulse_program);
    pio_gpio_init(stepper_pio, PIN_MOTOR_PULSE);
    
    pio_sm_config c = stepper_pulse_program_get_default_config(stepper_offset);
    sm_config_set_sideset_pins(&c, PIN_MOTOR_PULSE);
    
    // Fixed speed: 2 kHz pulse frequency (17 cycles/pulse) to match 300 RPM (2000 steps/sec)
    // Clock divisor = 125,000,000 / (2,000 * 17) = 3676.47f.
    float div = 3676.47f;
    sm_config_set_clkdiv(&c, div);
    
    sm_config_set_out_shift(&c, true, false, 32);
    
    pio_sm_init(stepper_pio, stepper_sm, stepper_offset, &c);
    pio_sm_set_enabled(stepper_pio, stepper_sm, true);
    
    // Direction pin as GPIO OUT
    gpio_init(PIN_MOTOR_DIR);
    gpio_set_dir(PIN_MOTOR_DIR, GPIO_OUT);
    gpio_put(PIN_MOTOR_DIR, 0);
}

void send_stepper_pulses(uint32_t count) {
    if (count == 0) return;
    pio_sm_put_blocking(stepper_pio, stepper_sm, count - 1);
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

// ============================================================================
// CHECKSUM CALCULATION
// --- CRC-8 Bluetooth (poly 0xA7) ---
// ============================================================================

static uint8_t reflect8(uint8_t val) {
    uint8_t res = 0;
    for (int i = 0; i < 8; i++) {
        if ((val >> i) & 1) {
            res |= (1 << (7 - i));
        }
    }
    return res;
}

static uint8_t calculateCRC8Bluetooth(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc ^= reflect8(data[i]);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xA7;
            } else {
                crc <<= 1;
            }
        }
    }
    return reflect8(crc);
}

// --- Geometric & Potentiometer Translations ---

// Standard utility functions for bounding values
static inline float clampf(float v, float min, float max) {
    return (v < min) ? min : ((v > max) ? max : v);
}
static inline uint16_t clamp_u16(uint16_t v, uint16_t min, uint16_t max) {
    return (v < min) ? min : ((v > max) ? max : v);
}

static float potToPistonPos(uint16_t pot) {
    pot = clamp_u16(pot, MINIMAL_THRESHOLD, MAXIMUM_THRESHOLD);
    return (((float)(pot - MINIMAL_THRESHOLD) / POT_RANGE) * PISTON_RANGE) - MAX_PISTON_POSITION;
}

static uint16_t pistonPosToPot(float pos_mm) {
    pos_mm = clampf(pos_mm, -MAX_PISTON_POSITION, MAX_PISTON_POSITION);
    float fraction = (pos_mm + MAX_PISTON_POSITION) / PISTON_RANGE;
    return MINIMAL_THRESHOLD + (uint16_t)(fraction * POT_RANGE);
}

static float pistonPosToVolume(float pos_mm) {
    return clampf(pos_mm * VOL_MULTIPLIER, -MAX_VOLUME, MAX_VOLUME);
}

static float volumeToPistonPos(float vol_cm3) {
    return clampf(vol_cm3 / VOL_MULTIPLIER, -MAX_PISTON_POSITION, MAX_PISTON_POSITION);
}


// ============================================================================
// UART DRIVER COMMUNICATION
// ============================================================================

void sendStopCommand(void) {
    stop_stepper_pio();
    printf("[HW -> Stepper]: Stop pulses\n");
}

void sendEnableCommand(void) {
    gpio_put(PIN_ENABLE_DRIVER, 0); // Active Low Enable
    printf("[HW -> Stepper]: Enable driver\n");
}

void sendDisableCommand(void) {
    gpio_put(PIN_ENABLE_DRIVER, 1); // Active Low Disable
    stop_stepper_pio();
    printf("[HW -> Stepper]: **DISABLE** (emergency stop)\n");
}

// ============================================================================
// ISR: GPIO LIMIT SWITCHES (COORDINATED CORE 1 ROUTED)
// ============================================================================

void gpio_limit_switches_callback(uint gpio, uint32_t events) {
    (void)events;
    if (gpio == SW_MIN_LIMIT) {
        flag_min_limit_hit = true;
    } else if (gpio == SW_MAX_LIMIT) {
        flag_max_limit_hit = true;
    }
}

void setup_limit_switches_on_core1(void) {
    gpio_init(SW_MIN_LIMIT);
    gpio_set_dir(SW_MIN_LIMIT, GPIO_IN);
    gpio_pull_up(SW_MIN_LIMIT);
    
    gpio_init(SW_MAX_LIMIT);
    gpio_set_dir(SW_MAX_LIMIT, GPIO_IN);
    gpio_pull_up(SW_MAX_LIMIT);
    
    gpio_set_irq_enabled_with_callback(SW_MIN_LIMIT, GPIO_IRQ_EDGE_FALL, true, &gpio_limit_switches_callback);
    gpio_set_irq_enabled(SW_MAX_LIMIT, GPIO_IRQ_EDGE_FALL, true);
}

// ============================================================================
// TASK: vMotorControlTask (Priority 6 - Real-time)
// ============================================================================

void vMotorControlTask(void *pvParameters) {
    (void)pvParameters;
    
    // Initialize ADC locally on Core 0
    adc_init();
    adc_gpio_init(POT_ADC_PIN);
    adc_select_input(POT_ADC_CHANNEL);
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5 Hz
    
    MotorControlState_t mctl_state = MCTL_IDLE;
    MotorCmd_t current_cmd = {0};
    
    if (vbs_should_log(1)) printf("[Motor] 5Hz Task started on Core 0\n");
    
    for (;;) {
        // 1. Guardrail de Segurança Física Absoluta - Check Limit Switch Flags
        if (flag_min_limit_hit) {
            printf("[Motor] **CRITICAL** Min limit switch hit! Recovering with 8188 steps in dir 1...\n");
            sys_state = SYS_CRITICAL_ERROR;
            sys_fault_code = FAULT_MIN_LIMIT_HIT;
            diag.fault_count[FAULT_MIN_LIMIT_HIT]++;
            
            // Stop PIO immediately
            stop_stepper_pio();
            
            // Clear current command and queue
            xQueueReset(xMotorCmdQueue);
            mctl_state = MCTL_IDLE;
            
            // Move opposite: direction 1, 8188 pulses
            gpio_put(PIN_MOTOR_DIR, 1);
            send_stepper_pulses(8188);
            wait_stepper_done();
            
            // Reset flags and return to operational
            flag_min_limit_hit = false;
            flag_max_limit_hit = false;
            sys_fault_code = FAULT_NONE;
            sys_state = SYS_OPERATIONAL;
            printf("[Motor] Min limit recovery complete.\n");
            
            xLastWakeTime = xTaskGetTickCount(); // reset wake time
            continue;
        }
        
        if (flag_max_limit_hit) {
            printf("[Motor] **CRITICAL** Max limit switch hit! Recovering with 8188 steps in dir 0...\n");
            sys_state = SYS_CRITICAL_ERROR;
            sys_fault_code = FAULT_MAX_LIMIT_HIT;
            diag.fault_count[FAULT_MAX_LIMIT_HIT]++;
            
            // Stop PIO immediately
            stop_stepper_pio();
            
            // Clear current command and queue
            xQueueReset(xMotorCmdQueue);
            mctl_state = MCTL_IDLE;
            
            // Move opposite: direction 0, 8188 pulses
            gpio_put(PIN_MOTOR_DIR, 0);
            send_stepper_pulses(8188);
            wait_stepper_done();
            
            // Reset flags and return to operational
            flag_min_limit_hit = false;
            flag_max_limit_hit = false;
            sys_fault_code = FAULT_NONE;
            sys_state = SYS_OPERATIONAL;
            printf("[Motor] Max limit recovery complete.\n");
            
            xLastWakeTime = xTaskGetTickCount(); // reset wake time
            continue;
        }
        
        // 2. Coleta de Comandos do Core 1
        if (xQueueReceive(xMotorCmdQueue, &current_cmd, 0) == pdTRUE) {
            mctl_state = (MotorControlState_t)current_cmd.cmd_type;
            diag.motor_cmd_queued++;
            
            set_target_pot_value(current_cmd.target_pot);
            
            if (vbs_should_log(1)) {
                printf("[Motor] New command: type=%d, target_pot=%d, pulses=%lu, direction=%d\n",
                       mctl_state, current_cmd.target_pot, (unsigned long)current_cmd.pulses, current_cmd.direction);
            }
        }
        
        // 3. Amostragem Local Direta e Média
        uint32_t sample_sum = 0;
        for (int i = 0; i < POT_SAMPLE_COUNT; i++) {
            sample_sum += adc_read();
        }
        uint16_t current_val = (uint16_t)(sample_sum / POT_SAMPLE_COUNT);
        current_val = current_val >> 3;
        setPotValue(current_val);
        
        // Update current geometry representations
        currentPistonPosition = (int16_t)potToPistonPos(current_val);
        currentVolume = pistonPosToVolume(potToPistonPos(current_val));
        
        // 4. Cálculo da Malha de Controle
        if (mctl_state == MCTL_MOVING_UNTIL_POT || mctl_state == MCTL_MOVING_ABSOLUTE) {
            uint16_t target = get_target_pot_value();
            int16_t error = (int16_t)target - (int16_t)current_val;
            
            if (abs(error) <= 3) {
                printf("[Motor] Target pot %d reached (current: %d)\n", target, current_val);
                stop_stepper_pio();
                mctl_state = MCTL_IDLE;
                diag.motor_move_complete++;
                
                if (mctl_state == MCTL_MOVING_UNTIL_POT) {
                    uint8_t ack_status = 0x00; // Success
                    sendBinaryPacket(CMD_ACK_MOVE, &ack_status, 1);
                }
            } else {
                // Out of deadband: execute control step (500 pulses)
                uint8_t direction = error > 0 ? 1 : 0;
                
                // Double check flags before executing PIO
                if (!flag_min_limit_hit && !flag_max_limit_hit) {
                    gpio_put(PIN_MOTOR_DIR, direction);
                    send_stepper_pulses(500);
                    wait_stepper_done();
                }
            }
        } else if (mctl_state == MCTL_MOVING_PULSES) {
            if (current_cmd.pulses == 0) {
                mctl_state = MCTL_IDLE;
                diag.motor_move_complete++;
            } else {
                uint32_t chunk = current_cmd.pulses;
                if (chunk > 500) {
                    chunk = 500;
                }
                
                // Double check flags before executing PIO
                if (!flag_min_limit_hit && !flag_max_limit_hit) {
                    gpio_put(PIN_MOTOR_DIR, current_cmd.direction);
                    send_stepper_pulses(chunk);
                    wait_stepper_done();
                    
                    current_cmd.pulses -= chunk;
                    if (current_cmd.pulses == 0) {
                        mctl_state = MCTL_IDLE;
                        diag.motor_move_complete++;
                    }
                }
            }
        } else {
            // MCTL_IDLE: Ensure stepper is stopped
            stop_stepper_pio();
        }
        
        // Mantém a periodicidade de 200ms (5Hz)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- Command Parser Helper Handlers ---

static void handle_cmd_help(void) {
    printf("Available commands:\n");
    printf("  help         - Show this help\n");
    printf("  move N       - Move N steps (signed)\n");
    printf("                 e.g. move 4094  or move -4094\n");
    printf("  move_pot P   - Move to potentiometer value P (0-511)\n");
    printf("                 e.g. move_pot 48  or move_pot 430\n");
    printf("  diag         - Print diagnostics now\n");
    printf("  pot          - Print current potentiometer value\n");
    printf("  stop         - Send driver stop command\n");
}

static void handle_cmd_change_pid(void) {
    printf("[Parser] PID control is not supported on the pulse/dir driver.\n");
}

static void handle_cmd_verbose(void) {
    char *arg = strtok(NULL, " \t");
    if (arg != NULL) {
        int v = atoi(arg);
        if (v < 0) v = 0;
        if (v > 6) v = 6;
        verbose_level = (uint8_t)v;
        printf("Verbose level set to %d\n", v);
    }
    else {
        printf("Current verbose level: %u\n", (unsigned)verbose_level);
    }
}

static void handle_cmd_diag(void) {
    printf("\n--- DIAGNOSTICS (on-demand) ---\n");
    printf("State: %s\n", stateToString(sys_state));
    printf("Fault: %s\n", faultToString(sys_fault_code));
    printf("Potentiometer: %d [%d - %d]\n", getPotValue(), MINIMAL_THRESHOLD, MAXIMUM_THRESHOLD);
    printf("Time since PC heartbeat: %lu ms\n", xTaskGetTickCount() - last_pc_heartbeat_ms);
    printf("Commands queued: %lu\n", diag.motor_cmd_queued);
    printf("Moves completed: %lu\n", diag.motor_move_complete);
    printf("Move timeouts: %lu\n", diag.motor_move_timeout);
    printf("--- end diagnostics ---\n\n");
}

static void handle_cmd_pot(void) {
    printf("Potentiometer (raw ADC avg): %d\n", getPotValue());
}

static void handle_cmd_stop(void) {
    MotorCmd_t cmd = {0};
    cmd.cmd_type = MCTL_IDLE;
    xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
    printf("Queued stop command\n");
}

static void handle_cmd_move(void) {
    char *arg1 = strtok(NULL, " \t");
    if (arg1 == NULL) {
        printf("Usage: move <steps>\n");
        return;
    }
    long steps = strtol(arg1, NULL, 10);
    
    uint8_t direction = (steps < 0) ? 1 : 0;
    uint32_t pulses = (uint32_t)(steps < 0 ? -steps : steps);
    if (pulses == 0) {
        printf("No pulses to send (0)\n");
    }
    else {
        MotorCmd_t cmd = {0};
        cmd.cmd_type = MCTL_MOVING_PULSES;
        cmd.pulses = pulses;
        cmd.direction = direction;
        xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
        printf("Queued move command: pulses=%lu, direction=%u\n", (unsigned long)pulses, (unsigned)direction);
    }
}

static void handle_cmd_move_pot(void) {
    char *arg1 = strtok(NULL, " \t");
    if (arg1 == NULL) {
        printf("Usage: move_pot <pot_value>\n");
        return;
    }
    int pot = atoi(arg1);
    if (pot < 0) pot = 0;
    if (pot > 511) pot = 511;

    MotorCmd_t cmd = {0};
    cmd.cmd_type = MCTL_MOVING_UNTIL_POT;
    cmd.target_pot = (uint16_t)pot;
    xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
    printf("Queued move_pot target=%d\n", pot);
}

typedef void (*cmd_handler_t)(void);

typedef struct {
    const char *name;
    cmd_handler_t handler;
} cmd_entry_t;

static const cmd_entry_t cmd_table[] = {
    {"help",       handle_cmd_help},
    {"change_pid", handle_cmd_change_pid},
    {"verbose",    handle_cmd_verbose},
    {"diag",       handle_cmd_diag},
    {"pot",        handle_cmd_pot},
    {"stop",       handle_cmd_stop},
    {"move",       handle_cmd_move},
    {"move_pot",   handle_cmd_move_pot},
};

#define NUM_COMMANDS (sizeof(cmd_table) / sizeof(cmd_table[0]))

// --- Binary Command Packet Communication ---

void sendBinaryPacket(uint8_t msgType, const uint8_t* data, uint8_t size) {
    uint8_t packet[8];
    packet[1] = getAddress();
    packet[2] = size;
    packet[3] = msgType;
    if (size > 0 && data != NULL) {
        memcpy(&packet[4], data, size);
    }
    packet[0] = calculateCRC8Bluetooth(&packet[1], 3 + size);
    
    for (int i = 0; i < 4 + size; i++) {
        putchar(packet[i]);
    }
    fflush(stdout);
}

static void send_binary_ack(uint8_t command_code, bool success) {
    uint8_t data[2];
    data[0] = success ? 0x00 : command_code;
    data[1] = command_code;
    sendBinaryPacket(CMD_ACK_CMD, data, 2);
}

static void send_binary_error(uint8_t command_code) {
    uint8_t data[1];
    data[0] = command_code;
    sendBinaryPacket(CMD_ERR_OUT_OF_BOUNDS, data, 1);
}

static void handle_binary_command(uint8_t msgType, const uint8_t* data, uint8_t size) {
    switch (msgType) {
        case CMD_TIMEOUT_EMERGENCY: {
            if (size >= 4) {
                // Parse emergency timeout seconds
                uint32_t seconds = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
                // Failsafe time configuration can be updated here if needed
                send_binary_ack(CMD_TIMEOUT_EMERGENCY, true);
            } else {
                send_binary_ack(CMD_TIMEOUT_EMERGENCY, false);
            }
            break;
        }
        case CMD_CHANGE_ID: {
            if (size >= 1) {
                uint8_t newAddress = data[0];
                changeAddress(newAddress);
                send_binary_ack(CMD_CHANGE_ID, true);
            } else {
                send_binary_ack(CMD_CHANGE_ID, false);
            }
            break;
        }
        case CMD_PING_PONG: {
            uint8_t addr = getAddress();
            sendBinaryPacket(CMD_PING_PONG, &addr, 1);
            break;
        }
        case CMD_FULL_CONTRACT: {
            MotorCmd_t cmd = {0};
            cmd.cmd_type = MCTL_MOVING_UNTIL_POT;
            cmd.target_pot = MINIMAL_THRESHOLD;
            cmd.speed = RECOMMENDED_SPEED_VAL;
            xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
            send_binary_ack(CMD_FULL_CONTRACT, true);
            break;
        }
        case CMD_FULL_EXPAND: {
            MotorCmd_t cmd = {0};
            cmd.cmd_type = MCTL_MOVING_UNTIL_POT;
            cmd.target_pot = MAXIMUM_THRESHOLD;
            cmd.speed = RECOMMENDED_SPEED_VAL;
            xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
            send_binary_ack(CMD_FULL_EXPAND, true);
            break;
        }
        case CMD_ABS_VOLUME: {
            if (size == 4) {
                uint32_t val_uint = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
                float target_vol;
                memcpy(&target_vol, &val_uint, sizeof(float));
                
                if (target_vol < -MAX_VOLUME || target_vol > MAX_VOLUME) {
                    send_binary_error(CMD_ABS_VOLUME);
                    return;
                }
                
                float target_pos_mm = volumeToPistonPos(target_vol);
                uint16_t target_pot = pistonPosToPot(target_pos_mm);
                
                MotorCmd_t cmd = {0};
                cmd.cmd_type = MCTL_MOVING_UNTIL_POT;
                cmd.target_pot = target_pot;
                cmd.speed = RECOMMENDED_SPEED_VAL;
                xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
                send_binary_ack(CMD_ABS_VOLUME, true);
            } else {
                send_binary_ack(CMD_ABS_VOLUME, false);
            }
            break;
        }
        case CMD_REL_VOLUME: {
            if (size >= 4) {
                uint32_t val_uint = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
                float rel_vol;
                memcpy(&rel_vol, &val_uint, sizeof(float));
                
                float current_vol = currentVolume;
                float target_vol = current_vol + rel_vol;
                
                if (target_vol < -MAX_VOLUME || target_vol > MAX_VOLUME) {
                    send_binary_error(CMD_REL_VOLUME);
                    return;
                }
                
                float target_pos_mm = volumeToPistonPos(target_vol);
                uint16_t target_pot = pistonPosToPot(target_pos_mm);
                
                MotorCmd_t cmd = {0};
                cmd.cmd_type = MCTL_MOVING_UNTIL_POT;
                cmd.target_pot = target_pot;
                cmd.speed = RECOMMENDED_SPEED_VAL;
                xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
                send_binary_ack(CMD_REL_VOLUME, true);
            } else {
                send_binary_ack(CMD_REL_VOLUME, false);
            }
            break;
        }
        case CMD_REQ_VOLUME: {
            float vol = currentVolume;
            uint32_t val_uint;
            memcpy(&val_uint, &vol, sizeof(float));
            uint8_t resp[4];
            resp[0] = (val_uint >> 24) & 0xFF;
            resp[1] = (val_uint >> 16) & 0xFF;
            resp[2] = (val_uint >> 8) & 0xFF;
            resp[3] = val_uint & 0xFF;
            sendBinaryPacket(CMD_REQ_VOLUME, resp, 4);
            break;
        }
        case CMD_ABS_PISTON: {
            if (size >= 4) {
                uint32_t val_uint = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
                float target_pos;
                memcpy(&target_pos, &val_uint, sizeof(float));
                
                if (target_pos < -MAX_PISTON_POSITION || target_pos > MAX_PISTON_POSITION) {
                    send_binary_error(CMD_ABS_PISTON);
                    return;
                }
                
                uint16_t target_pot = pistonPosToPot(target_pos);
                
                MotorCmd_t cmd = {0};
                cmd.cmd_type = MCTL_MOVING_UNTIL_POT;
                cmd.target_pot = target_pot;
                cmd.speed = RECOMMENDED_SPEED_VAL;
                xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
                send_binary_ack(CMD_ABS_PISTON, true);
            } else {
                send_binary_ack(CMD_ABS_PISTON, false);
            }
            break;
        }
        case CMD_REL_PISTON: {
            if (size >= 4) {
                uint32_t val_uint = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
                float rel_pos;
                memcpy(&rel_pos, &val_uint, sizeof(float));
                
                float current_pos = potToPistonPos(getPotValue());
                float target_pos = current_pos + rel_pos;
                
                if (target_pos < -MAX_PISTON_POSITION || target_pos > MAX_PISTON_POSITION) {
                    send_binary_error(CMD_REL_PISTON);
                    return;
                }
                
                uint16_t target_pot = pistonPosToPot(target_pos);
                
                MotorCmd_t cmd = {0};
                cmd.cmd_type = MCTL_MOVING_UNTIL_POT;
                cmd.target_pot = target_pot;
                cmd.speed = RECOMMENDED_SPEED_VAL;
                xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
                send_binary_ack(CMD_REL_PISTON, true);
            } else {
                send_binary_ack(CMD_REL_PISTON, false);
            }
            break;
        }
        case CMD_REQ_PISTON: {
            float pos = potToPistonPos(getPotValue());
            uint32_t val_uint;
            memcpy(&val_uint, &pos, sizeof(float));
            uint8_t resp[4];
            resp[0] = (val_uint >> 24) & 0xFF;
            resp[1] = (val_uint >> 16) & 0xFF;
            resp[2] = (val_uint >> 8) & 0xFF;
            resp[3] = val_uint & 0xFF;
            sendBinaryPacket(CMD_REQ_PISTON, resp, 4);
            break;
        }
        default:
            break;
    }
}

// ============================================================================
// TASK: vParserTask (Priority 4 - Real-time)
// ============================================================================

void vParserTask(void *pvParameters) {
    char line_buffer[256];
    int line_idx = 0;
    uint8_t rx_buf[32];
    size_t rx_len = 0;
    
    if (vbs_should_log(3)) printf("[Parser] Task started\n");
    printf("\nVBS v2.0 Ready. Type 'help' for commands.\n");
    printf("> ");
    fflush(stdout);
    
    while (1) {
        // Use timed getchar to avoid blocking lower-priority tasks
        int ch = getchar_timeout_us(10000); // 10 ms timeout

        if (ch == PICO_ERROR_TIMEOUT) {
            // No input available; yield briefly to allow lower-priority tasks
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // --- 1. Accumulate for Binary Parser ---
        if (rx_len < sizeof(rx_buf)) {
            rx_buf[rx_len++] = (uint8_t)ch;
        } else {
            memmove(rx_buf, rx_buf + 1, sizeof(rx_buf) - 1);
            rx_buf[sizeof(rx_buf) - 1] = (uint8_t)ch;
        }

        // Check if we have a valid binary packet in rx_buf
        bool binary_packet_parsed = false;
        while (rx_len >= 4) {
            uint8_t expected_addr = getAddress();
            if (rx_buf[1] == expected_addr) {
                uint8_t payload_size = rx_buf[2];
                if (payload_size <= 4) {
                    size_t expected_packet_len = 4 + payload_size;
                    if (rx_len >= expected_packet_len) {
                        uint8_t calculated_crc = calculateCRC8Bluetooth(&rx_buf[1], expected_packet_len - 1);
                        uint8_t received_crc = rx_buf[0];
                        
                        if (calculated_crc == received_crc) {
                            handle_binary_command(rx_buf[3], &rx_buf[4], payload_size);
                            
                            // Remove the processed packet from the buffer
                            memmove(rx_buf, rx_buf + expected_packet_len, rx_len - expected_packet_len);
                            rx_len -= expected_packet_len;
                            binary_packet_parsed = true;
                            continue;
                        }
                    } else {
                        break; // wait for more data
                    }
                }
            }
            memmove(rx_buf, rx_buf + 1, rx_len - 1);
            rx_len--;
        }

        if (binary_packet_parsed) {
            continue; // Skip string parsing if we consumed binary data
        }

        // --- 2. Accumulate for Interactive Text Parser ---
        if (ch == '\n' || ch == '\r') {
            if (line_idx > 0) {
                line_buffer[line_idx] = '\0';
                
                // Update heartbeat (any USB input counts as PC alive)
                last_pc_heartbeat_ms = xTaskGetTickCount();
                diag.pc_heartbeats++;
                
                if (vbs_should_log(3) || vbs_should_log(6)) printf("\n[Parser] Received: '%s'\n", line_buffer);
                
                // Tokenize the input line (first word = command)
                char *cmd = strtok(line_buffer, " \t");
                if (cmd != NULL) {
                    for (char *p = cmd; *p; ++p) *p = tolower((unsigned char)*p);

                    bool found = false;
                    for (size_t i = 0; i < NUM_COMMANDS; i++) {
                        if (strcmp(cmd, cmd_table[i].name) == 0) {
                            cmd_table[i].handler();
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        printf("Unknown command: '%s'\n", cmd);
                    }
                }
                
                line_idx = 0;
            }
            printf("> ");
            fflush(stdout);
        }
        else if (ch >= 32 && ch < 127) {
            // Printable ASCII
            if (line_idx < (int)sizeof(line_buffer) - 1) {
                line_buffer[line_idx] = (char)ch;
                line_idx++;
                putchar(ch);  // Echo
                fflush(stdout);
            }
        }
        else if (ch == '\b' || ch == 127) {
            // Backspace
            if (line_idx > 0) {
                line_idx--;
                putchar('\b');
                putchar(' ');
                putchar('\b');
                fflush(stdout);
            }
        }
    }
}

// ============================================================================
// TASK: vFaultManagerTask (Priority 2 - Utility)
// ============================================================================

void vFaultManagerTask(void *pvParameters) {
    (void)pvParameters;
    
    // Configure limit switches on Core 1
    setup_limit_switches_on_core1();
    if (vbs_should_log(5)) printf("[FaultMgr] Limit switch interrupts configured on Core 1\n");
    
    if (vbs_should_log(5)) printf("[FaultMgr] Task started on Core 1\n");
    
    // Initialize watchdog (8 second timeout)
    watchdog_enable(8000, true);
    
    while (1) {
        uint32_t now_ms = xTaskGetTickCount();
        
        // Wait periodically (100ms)
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Check for PC timeout (300 seconds)
        uint32_t time_since_heartbeat = now_ms - last_pc_heartbeat_ms;
        if (time_since_heartbeat > PC_TIMEOUT_MS) {
            if (sys_state == SYS_OPERATIONAL) {
                printf("[FaultMgr] PC timeout (>%lu ms) → FAILSAFE_ASCENT\n", (unsigned long)PC_TIMEOUT_MS);
                sys_state = SYS_FAILSAFE_ASCENT;
                sys_fault_code = FAULT_PC_TIMEOUT;
                diag.fault_count[FAULT_PC_TIMEOUT]++;
                
                // Queue auto-expand command
                MotorCmd_t failsafe_cmd = {0};
                failsafe_cmd.cmd_type = MCTL_MOVING_UNTIL_POT;
                failsafe_cmd.target_pot = MAXIMUM_THRESHOLD;
                xQueueSend(xMotorCmdQueue, &failsafe_cmd, 0);
            }
        }
        
        // Kick watchdog every ~100ms
        watchdog_update();
    }
}

// ============================================================================
// TASK: vDiagnosticsTask (Priority 1 - Background)
// ============================================================================

void vDiagnosticsTask(void *pvParameters) {
    /*
     * Low-priority background task that periodically logs diagnostics.
     * Runs every 10 seconds.
     */
    if (vbs_should_log(5)) printf("[Diagnostics] Task started on Core 1\n");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // Every 10 seconds
        if(verbose_level != 0){
            printf("\n========== DIAGNOSTICS ==========\n");
            printf("State: %s\n", stateToString(sys_state));
            printf("Fault: %s\n", faultToString(sys_fault_code));
            printf("Potentiometer: %d [%d - %d]\n",
                getPotValue(), MINIMAL_THRESHOLD, MAXIMUM_THRESHOLD);
            printf("Time since PC heartbeat: %lu ms\n",
                xTaskGetTickCount() - last_pc_heartbeat_ms);
            
            printf("\nMotor Control:\n");
            printf("  Commands queued: %lu\n", diag.motor_cmd_queued);
            printf("  Moves completed: %lu\n", diag.motor_move_complete);
            printf("  Move timeouts: %lu\n", diag.motor_move_timeout);
            
            printf("\nFaults Detected:\n");
            printf("  Min limit: %lu\n", diag.fault_count[FAULT_MIN_LIMIT_HIT]);
            printf("  Max limit: %lu\n", diag.fault_count[FAULT_MAX_LIMIT_HIT]);
            printf("  PC timeout: %lu\n", diag.fault_count[FAULT_PC_TIMEOUT]);
            printf("===================================\n\n");
        }
    }
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

void initialize_hardware_sync(void) {
    vbs_shared_lock = spin_lock_init(VBS_SPINLOCK_ID);
}

void initializeHardware(void) {
    // Initialize USB stdio
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB to settle
    
    printf("\n=== VBS v2.0 Hardware Initialization ===\n");
    
    // Initialize hardware sync spinlocks
    initialize_hardware_sync();
    printf("[HW] Hardware synchronization spinlocks initialized\n");
    
    // Setup Stepper Motor PIO (using GPIO4 for PUL and GPIO5 for DIR)
    setup_stepper_pio();
    printf("[HW] Stepper motor PIO initialized (PUL = GPIO4, DIR = GPIO5)\n");
    
    // Enable Driver (active low, set to 0 to enable)
    gpio_init(PIN_ENABLE_DRIVER);
    gpio_set_dir(PIN_ENABLE_DRIVER, GPIO_OUT);
    gpio_put(PIN_ENABLE_DRIVER, 0); 
    printf("[HW] Motor driver enabled pin configured\n");
    
    printf("=== Hardware Initialization Complete ===\n\n");
}

// ============================================================================
// RTOS INITIALIZATION
// ============================================================================

void initializeRTOS(void) {
    printf("=== RTOS Initialization ===\n");
    
    // Potentiometer protection uses hardware spinlock
    printf("[RTOS] Potentiometer access protected by hardware spinlocks\n");
    
    // Create motor command queue (statically)
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
    
    // 1. Create and bind Motor Control Task to Core 0 (Bit 0)
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
    vTaskCoreAffinitySet(xMotorControlTaskHandle, (1 << 0)); // Core 0
    printf("[RTOS] vMotorControlTask created statically on Core 0 (priority 6)\n");
    
    // 2. Create and bind utility / communication tasks to Core 1 (Bit 1)
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
    vTaskCoreAffinitySet(xParserTaskHandle, (1 << 1)); // Core 1
    printf("[RTOS] vParserTask created statically on Core 1 (priority 4)\n");
    
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
    vTaskCoreAffinitySet(xFaultMgrTaskHandle, (1 << 1)); // Core 1
    printf("[RTOS] vFaultManagerTask created statically on Core 1 (priority 3)\n");
    
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
    vTaskCoreAffinitySet(xDiagnosticsTaskHandle, (1 << 1)); // Core 1
    printf("[RTOS] vDiagnosticsTask created statically on Core 1 (priority 1)\n");
    
    printf("=== RTOS Initialization Complete ===\n\n");

    // Initialize PC heartbeat timestamp to now to avoid immediate PC-timeout
    last_pc_heartbeat_ms = xTaskGetTickCount();
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    initializeHardware();
    initializeRTOS();
    
    printf("Starting FreeRTOS scheduler...\n");
    vTaskStartScheduler();
    
    // Should never reach here
    while (1) {
        printf("[ERROR] vTaskStartScheduler returned!\n");
        sleep_ms(1000);
    }
    
    return 0;
}

// ============================================================================
// FREERTOS STATIC ALLOCATION CALLBACKS
// ============================================================================

extern "C" {
    void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                        StackType_t ** ppxIdleTaskStackBuffer,
                                        configSTACK_DEPTH_TYPE * puxIdleTaskStackSize )
    {
        static StaticTask_t xIdleTaskTCB;
        static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

        *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
        *ppxIdleTaskStackBuffer = uxIdleTaskStack;
        *puxIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    }

    void vApplicationGetPassiveIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                               StackType_t ** ppxIdleTaskStackBuffer,
                                               configSTACK_DEPTH_TYPE * puxIdleTaskStackSize,
                                               BaseType_t xCoreID )
    {
        static StaticTask_t xPassiveIdleTaskTCBs[ configNUMBER_OF_CORES - 1 ];
        static StackType_t uxPassiveIdleTaskStacks[ configNUMBER_OF_CORES - 1 ][ configMINIMAL_STACK_SIZE ];

        if( xCoreID < ( configNUMBER_OF_CORES - 1 ) )
        {
            *ppxIdleTaskTCBBuffer = &( xPassiveIdleTaskTCBs[ xCoreID ] );
            *ppxIdleTaskStackBuffer = &( uxPassiveIdleTaskStacks[ xCoreID ][ 0 ] );
            *puxIdleTaskStackSize = configMINIMAL_STACK_SIZE;
        }
    }

    void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                         StackType_t ** ppxTimerTaskStackBuffer,
                                         configSTACK_DEPTH_TYPE * puxTimerTaskStackSize )
    {
        static StaticTask_t xTimerTaskTCB;
        static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

        *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
        *ppxTimerTaskStackBuffer = uxTimerTaskStack;
        *puxTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    }
}
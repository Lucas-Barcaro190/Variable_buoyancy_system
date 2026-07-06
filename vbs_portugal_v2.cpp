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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "semphr.h"
#include <climits>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define DRIVER_UART                 uart1
#define BAUD_RATE                   38400
#define UART_TX_PIN                 4
#define UART_RX_PIN                 5
#define DRIVER_ADDR                 0xE0

#define PIN_ENABLE_DRIVER           16          // High = Driver OFF, Low = Driver ON
#define SW_MIN_LIMIT                3           // Contracted switch for minimum limit
#define SW_MAX_LIMIT                2           // Expanded switch for maximum limit
#define POT_ADC_PIN                 26          // GPIO26 (ADC0)
#define POT_ADC_CHANNEL             0           // ADC Channel 0
#define POT_SAMPLE_COUNT            128

#define MINIMAL_THRESHOLD           43          // Minimum potentiometer value to consider valid
#define MAXIMUM_THRESHOLD           435         // Maximum potentiometer value

#define MAX_SPEED_VAL               127
#define RECOMMENDED_SPEED_VAL       32
#define MAX_PULSES                  1506752       // tested in 16.02.26 @ speed=32

// PC Timeout for failsafe (300 seconds = 5 minutes)
#define PC_TIMEOUT_MS               300000

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
    FAULT_DRIVER_CHECKSUM       =   0xE4,
    FAULT_DRIVER_ACK_TIMEOUT    =   0xE5,
    FAULT_MOTOR_STALL           =   0xE6,
    FAULT_QUEUE_OVERFLOW        =   0xE7,
} FaultCode_t;

typedef enum {
    MCTL_IDLE                   =   0,
    MCTL_MOVING_ABSOLUTE,
    MCTL_MOVING_UNTIL_POT,
    MCTL_MOVING_CONTINUOUS,
    MCTL_SET_PID_KP,
    MCTL_SET_PID_KD,
} MotorControlState_t;

// Task notification bits
#define NOTIF_MIN_LIMIT             (1 << 0)
#define NOTIF_MAX_LIMIT             (1 << 1)
#define NOTIF_FAULT_DETECTED        (1 << 2)

// ============================================================================
// MESSAGE & COMMAND STRUCTURES
// ============================================================================

typedef struct {
    uint8_t cmd_type;              // MotorControlState_t value
    uint16_t target_pot;           // Target potentiometer value
    uint16_t speed;                // Motor speed (0-127) [300 rpm -> speed 2 @ Mstep=1; speed 4 @ Mstep=2; ... ; speed 32 @ Mstep=16]
    uint16_t max_pulses;           // Max pulses for safety timeout
    uint32_t cmd_id;               // Unique command ID for tracking
} MotorCmd_t;

typedef struct {
    uint8_t response_code;         // Driver response opcode
    uint8_t data[62];              // Response payload
    uint8_t length;                // Total payload length
    uint32_t timestamp_ms;         // Timestamp when received
} DriverMsg_t;

typedef struct {
    uint16_t pot_value;            // Current potentiometer reading
    bool min_limit_hit;            // Minimum limit switch state
    bool max_limit_hit;            // Maximum limit switch state
    uint32_t timestamp_ms;         // Timestamp of last update
} PositionFeedback_t;

typedef struct {
    uint8_t fault_code;            // FaultCode_t value
    uint32_t timestamp_ms;         // When fault occurred
} FaultNotif_t;

typedef struct {
    uint32_t uart_rx_bytes;
    uint32_t uart_rx_frames;
    uint32_t uart_checksum_errors;
    uint32_t uart_frame_errors;
    uint32_t motor_cmd_queued;
    uint32_t motor_move_complete;
    uint32_t motor_move_timeout;
    uint32_t fault_count[256];     // One counter per fault code
    uint32_t pc_heartbeats;
    uint32_t stream_buffer_overflows;
} Diagnostics_t;

// ============================================================================
// STATIC TASK ALLOCATION
// ============================================================================

// Task stacks (sizes from refactor plan)
#define MOTOR_CONTROL_STACK_SIZE    (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define RECEIVER_STACK_SIZE         (512 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define PARSER_STACK_SIZE           (512 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define POT_STACK_SIZE              (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define FAULT_MGR_STACK_SIZE        (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))
#define DIAGNOSTICS_STACK_SIZE      (256 * sizeof(portSTACK_TYPE) / sizeof(StackType_t))

static StackType_t xMotorControlStack[MOTOR_CONTROL_STACK_SIZE];
static StackType_t xReceiverStack[RECEIVER_STACK_SIZE];
static StackType_t xParserStack[PARSER_STACK_SIZE];
static StackType_t xPotStack[POT_STACK_SIZE];
static StackType_t xFaultMgrStack[FAULT_MGR_STACK_SIZE];
static StackType_t xDiagnosticsStack[DIAGNOSTICS_STACK_SIZE];

static StaticTask_t xMotorControlTCB;
static StaticTask_t xReceiverTCB;
static StaticTask_t xParserTCB;
static StaticTask_t xPotTCB;
static StaticTask_t xFaultMgrTCB;
static StaticTask_t xDiagnosticsTCB;

// Task handles
TaskHandle_t xMotorControlTaskHandle    =   NULL;
TaskHandle_t xReceiverTaskHandle        =   NULL;
TaskHandle_t xParserTaskHandle          =   NULL;
TaskHandle_t xPotTaskHandle             =   NULL;
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

// Driver Response Queue (Receiver → Parser for async responses)
#define DRIVER_MSG_QUEUE_SIZE           10
static uint8_t ucDriverMsgQueueStorage[DRIVER_MSG_QUEUE_SIZE * sizeof(DriverMsg_t)];
static StaticQueue_t xDriverMsgQueueHandle;
QueueHandle_t xDriverMsgQueue           =   NULL;

// ============================================================================
// STATIC STREAM BUFFER ALLOCATION (UART RX)
// ============================================================================

// Stream buffer for driver UART RX (256 bytes)
#define DRIVER_RX_STREAM_SIZE           256
static uint8_t ucDriverRxStreamBuffer[DRIVER_RX_STREAM_SIZE];
static StaticStreamBuffer_t xDriverRxStreamBufferStatic;
StreamBufferHandle_t xDriverRxStream    =   NULL;

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
    case FAULT_DRIVER_CHECKSUM:         return "DRIVER_CHECKSUM";
    case FAULT_DRIVER_ACK_TIMEOUT:      return "DRIVER_ACK_TIMEOUT";
    case FAULT_MOTOR_STALL:             return "MOTOR_STALL"; // I need to check this part
    case FAULT_QUEUE_OVERFLOW:          return "QUEUE_OVERFLOW";
    default:                            return "UNKNOWN";
    }
}

uint16_t getPotValue(void) {
    uint16_t val;
    taskENTER_CRITICAL();
    val = potentiometer_value;
    taskEXIT_CRITICAL();
    return val;
}

void setPotValue(uint16_t val) {
    taskENTER_CRITICAL();
    potentiometer_value = val;
    taskEXIT_CRITICAL();
}

// ============================================================================
// CHECKSUM CALCULATION
// ============================================================================

static uint8_t calculateChecksum(const uint8_t* data, int length) {
    uint8_t sum = 0;
    for (int i = 0; i < length; i++) { // Process ALL bytes
        sum = (uint8_t)(data[i] + sum);
    }
    return sum;
}

// ============================================================================
// UART DRIVER COMMUNICATION
// ============================================================================

void sendPacketWithChecksum(const uint8_t* packet, int length) {
    if (length < 2 || length > 64) {
        printf("[Error] Invalid packet length: %d\n", length);
        return;
    }
    
    uint8_t checksum = calculateChecksum(packet, length);
    
    // Send packet bytes
    // Log the exact bytes that will be sent (hex)
    printf("[Pico -> Driver]:");
    for (int i = 0; i < length; i++) {
        printf(" 0x%02X", packet[i]);
    }
    printf(" checksum=0x%02X\n", checksum);

    for (int i = 0; i < length; i++) {
        uart_putc(DRIVER_UART, packet[i]);
    }
    // Send checksum
    uart_putc(DRIVER_UART, checksum);
}

void sendStopCommand(void) {
    uint8_t stop_packet[2];
    stop_packet[0] = DRIVER_ADDR;
    stop_packet[1] = 0xF7;
    sendPacketWithChecksum(stop_packet, 2);
    printf("[Pico -> Driver]: Stop\n");
}

void sendConstantMoveCommand(uint8_t direction, uint8_t speed) {
    uint8_t val = (direction << 7) | (speed & 0x7F);
    uint8_t packet[3];
    packet[0] = DRIVER_ADDR;
    packet[1] = 0xF6;
    packet[2] = val;
    sendPacketWithChecksum(packet, 3);
    printf("[Pico -> Driver]: Constant move, direction: %d, speed: %d\n", direction, speed);
}

void sendEnableCommand(void) {
    uint8_t enable_packet[2];
    enable_packet[0] = DRIVER_ADDR;
    enable_packet[1] = 0xF3;
    uint8_t enable_param[1] = {0x01};  // Enable
    uint8_t full_packet[3] = {enable_packet[0], enable_packet[1], enable_param[0]};
    sendPacketWithChecksum(full_packet, 3);
    printf("[Pico -> Driver]: Enable\n");
}

void sendDisableCommand(void) {
    uint8_t disable_packet[3];
    disable_packet[0] = DRIVER_ADDR;
    disable_packet[1] = 0xF3;
    disable_packet[2] = 0x00;  // Disable
    sendPacketWithChecksum(disable_packet, 3);
    printf("[Pico -> Driver]: **DISABLE** (emergency stop)\n");
}

// Send a single move-by-pulses command to the driver (0xFD)
void sendMovePulses(uint32_t pulses, uint8_t direction, uint8_t speed) {
    uint8_t packet[7];
    packet[0] = DRIVER_ADDR;
    packet[1] = 0xFD; // Move command
    packet[2] = (direction << 7) | (speed & 0x7F);
    for (int i = 0; i < 4; i++) {
        packet[3 + i] = (pulses >> (24 - 8 * i)) & 0xFF;
    }
    sendPacketWithChecksum(packet, 7);
    if (vbs_should_log(1) || vbs_should_log(6)) printf("[Pico -> Driver]: Move pulses=%lu dir=%u speed=%u\n", (unsigned long)pulses, (unsigned)direction, (unsigned)speed);
}



// ============================================================================
// ISR: UART DRIVER RX
// ============================================================================

extern "C" {
    void on_uart_rx_driver(void) {
        /*
         * Reads all available bytes from DRIVER_UART and pushes them to the
         * stream buffer.
         * 
         * This ISR is minimal: only read + push. All parsing happens in
         * vReceiverTask.
         */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        while (uart_is_readable(DRIVER_UART)) {
            uint8_t ch = uart_getc(DRIVER_UART);
            
            if (xDriverRxStream != NULL) {
                // xStreamBufferSendFromISR returns number of bytes written
                // If stream buffer is full, it may discard old data (configurable)
                size_t written = xStreamBufferSendFromISR(
                                xDriverRxStream,
                                &ch,
                                1,
                                &xHigherPriorityTaskWoken
                                );
                
                if (written == 0) {
                    diag.stream_buffer_overflows++;
                }
            }
            
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// ============================================================================
// ISR: GPIO LIMIT SWITCHES
// ============================================================================

extern "C" {
    void gpio_callback(uint gpio, uint32_t events) {
        /*
         * Triggered when either limit switch changes state.
         * 
         * IMMEDIATE ACTIONS (ISR - hardware-level safety):
         * 1. Send DISABLE command (0xF3 0x00) to stop motor via UART
         * 2. Notify vFaultManagerTask to queue 1mm recovery move
         * 
         * RECOVERY (vFaultManagerTask - coordinated move):
         * 1. Sends ENABLE command (0xF3 0x01) to re-enable driver
         * 2. Queues automatic 1mm back-off in opposite direction
         * 3. Enters CRITICAL_ERROR state
         */
        printf("%i\n", gpio);
        printf("%i\n", events);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t now_us = time_us_32();

        const uint32_t debounce_window_us = 200000;

        static uint32_t last_min_trigger_us = 0;
        static uint32_t last_max_trigger_us = 0;
        
        if (gpio == SW_MIN_LIMIT && (events & GPIO_IRQ_EDGE_FALL)) {
            // IMMEDIATE: Send DISABLE command to driver (motor stop via UART)
            if((now_us - last_min_trigger_us) >= debounce_window_us){
                last_min_trigger_us = now_us;

                uint8_t disable_packet[3] = {DRIVER_ADDR, 0xF3, 0x00};
                uint8_t checksum = calculateChecksum(disable_packet, 3);
                for (int i = 0; i < 3; i++) {
                    uart_putc(DRIVER_UART, disable_packet[i]);
                }
                uart_putc(DRIVER_UART, checksum);
                
                // Notify fault manager to handle recovery and diagnostics
                xTaskNotifyFromISR(
                    xFaultMgrTaskHandle,
                    NOTIF_MIN_LIMIT,
                    eSetBits,
                    &xHigherPriorityTaskWoken
                    );
            }
        }
        
        else if (gpio == SW_MAX_LIMIT && (events & GPIO_IRQ_EDGE_FALL)) {
            // IMMEDIATE: Send DISABLE command to driver (motor stop via UART)
            // Minimal ISR version - no printf to avoid bloating ISR
            printf("[ISR] Max limit switch triggered, sending DISABLE command.\n");
            if((now_us - last_max_trigger_us) >= debounce_window_us){
                uint8_t disable_packet[3] = {DRIVER_ADDR, 0xF3, 0x00};
                uint8_t checksum = calculateChecksum(disable_packet, 3);
                for (int i = 0; i < 3; i++) {
                    uart_putc(DRIVER_UART, disable_packet[i]);
                }
                uart_putc(DRIVER_UART, checksum);
                
                // Notify fault manager to handle recovery and diagnostics
                xTaskNotifyFromISR(
                    xFaultMgrTaskHandle,
                    NOTIF_MAX_LIMIT,
                    eSetBits,
                    &xHigherPriorityTaskWoken
                );
            }
        }
        
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


// ============================================================================
// TASK: vMotorControlTask (Priority 6 - Real-time)
// ============================================================================

void vMotorControlTask(void *pvParameters) {
    /*
     * Motor control state machine. Receives MotorCmd_t from parser queue
     * and executes the requested movement, checking potentiometer feedback.
     * 
     * Non-blocking design:
     * - If a move is in progress, continue polling potentiometer
     * - If target reached (within tolerance), stop and go idle
     * - If timeout detected, fault and go idle
     */
    MotorControlState_t mctl_state = MCTL_IDLE;
    MotorCmd_t current_cmd = {0};
    uint32_t move_start_time_ms = 0;
    
    if (vbs_should_log(1)) printf("[Motor] Task started, state: IDLE\n");
    
    while (1) {
        // Check for new commands (non-blocking)
        if (xQueueReceive(xMotorCmdQueue, &current_cmd, pdMS_TO_TICKS(50)) == pdTRUE) {
            mctl_state = (MotorControlState_t)current_cmd.cmd_type;
            move_start_time_ms = xTaskGetTickCount();
                 diag.motor_cmd_queued++;
                 if (vbs_should_log(1) || vbs_should_log(6)) printf("[Motor] New command: type=%d, target_pot=%d, speed=%d\n",
                     mctl_state, current_cmd.target_pot, current_cmd.speed);
        }
        
        // Execute state machine
        switch (mctl_state) {
        case MCTL_IDLE:
            // No movement; just yield
            break;
            
        case MCTL_MOVING_UNTIL_POT:
        {
            uint16_t pot = getPotValue();
            int16_t error = (int16_t)current_cmd.target_pot - (int16_t)pot;
            
            // Check if target reached (within 3 units tolerance)
            if (abs(error) <= 3) {
                printf("[Motor] Target pot %d reached (current: %d)\n", 
                       current_cmd.target_pot, pot);
                sendStopCommand();
                mctl_state = MCTL_IDLE;
                diag.motor_move_complete++;
                break;
            }
            
            // Check timeout (900 seconds per move)
            uint32_t elapsed = xTaskGetTickCount() - move_start_time_ms;
            if (elapsed > pdMS_TO_TICKS(900000)) {
                printf("[Motor] Move timeout (%lu ms)\n", elapsed);
                sendStopCommand();
                sys_fault_code = FAULT_DRIVER_ACK_TIMEOUT;
                sys_state = SYS_CRITICAL_ERROR;
                mctl_state = MCTL_IDLE;
                diag.motor_move_timeout++;
                
                // Notify fault manager
                xTaskNotify(xFaultMgrTaskHandle, NOTIF_FAULT_DETECTED, eSetBits);
                break;
            }
            
            // Send move command in appropriate direction
            uint8_t direction = (error > 0) ? 0 : 1;  // 0=expand, 1=contract
            uint8_t speed = current_cmd.speed & 0x7F;
            if (speed > MAX_SPEED_VAL) speed = MAX_SPEED_VAL;
            
            sendConstantMoveCommand(direction, speed);
            break;
        }
        
        case MCTL_MOVING_ABSOLUTE:
        {
            // Absolute move: target potentiometer value (used for 1mm back-off after limit hits)
            uint16_t pot = getPotValue();
            int16_t error = (int16_t)current_cmd.target_pot - (int16_t)pot;
            
            // Check if target reached (within 3 units tolerance)
            if (abs(error) <= 3) {
                printf("[Motor] Absolute move complete (pot: %d, target: %d)\n", pot, current_cmd.target_pot);
                sendStopCommand();
                mctl_state = MCTL_IDLE;
                diag.motor_move_complete++;
                break;
            }
            
            // Check timeout (900 seconds per move)
            uint32_t elapsed = xTaskGetTickCount() - move_start_time_ms;
            if (elapsed > pdMS_TO_TICKS(900000)) {
                printf("[Motor] Absolute move timeout after %lu ms\n", elapsed);
                sendStopCommand();
                sys_fault_code = FAULT_DRIVER_ACK_TIMEOUT;
                sys_state = SYS_CRITICAL_ERROR;
                mctl_state = MCTL_IDLE;
                diag.motor_move_timeout++;
                
                xTaskNotify(xFaultMgrTaskHandle, NOTIF_FAULT_DETECTED, eSetBits);
                break;
            }
            
            // Send move command in appropriate direction
            uint8_t direction = (error > 0) ? 0 : 1;  // 0=expand, 1=contract
            uint8_t speed = current_cmd.speed & 0x7F;
            if (speed > MAX_SPEED_VAL) speed = MAX_SPEED_VAL;
            
            sendConstantMoveCommand(direction, speed);
            break;
        }
            
        case MCTL_MOVING_CONTINUOUS:
            // Placeholder for future continuous moves
            printf("[Motor] MOVING_CONTINUOUS not yet implemented\n");
            mctl_state = MCTL_IDLE;
            break;
        
        case MCTL_SET_PID_KP:
        case MCTL_SET_PID_KD:{
            uint8_t packet[4];
            packet[0] = DRIVER_ADDR;                                // Slave Addr (0xE0)
            packet[1] = (uint8_t)current_cmd.max_pulses;            // Function Code (0xA1 or 0xA3)
            packet[2] = (uint8_t)((current_cmd.speed >> 8) & 0xFF); // High Data Byte
            packet[3] = (uint8_t)(current_cmd.speed & 0xFF);        // Low Data Byte
            
            // Transmit through our safe checksum driver loop
            sendPacketWithChecksum(packet, 4);
            
            printf("[Motor] PID Updated: Code 0x%02X to Value %u\n", 
                   packet[1], current_cmd.speed);
            
            // Instantly return to IDLE so it doesn't loop transmissions
            mctl_state = MCTL_IDLE;
            diag.motor_move_complete++;
            break;
            }
        }
        // Yield to lower-priority tasks
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
// TASK: vReceiverTask (Priority 5 - Real-time)
// ============================================================================

void vReceiverTask(void *pvParameters) {
    /*
     * Reads bytes from DRIVER_UART via stream buffer, assembles packets,
     * validates checksums, and sends valid DriverMsg_t to parser queue.
     * 
     * Packet format: [DRIVER_ADDR] [CMD] [DATA...] [CHECKSUM]
     * 
     * Strategy:
     * 1. Wait for stream buffer data with 10ms timeout
     * 2. Sync on DRIVER_ADDR start byte
     * 3. Collect remaining bytes with inter-byte timeout
     * 4. Validate checksum
     * 5. Send to driver_msg_queue or discard if invalid
     */
    uint8_t rxBuffer[64];
    int rxIndex = 0;
    
    printf("[Receiver] Cleaned Timeout Task Started\n");
    
    while (1) {
        uint8_t byte;
        // Wait for a single byte from the stream buffer with a 20ms timeout
        size_t received = xStreamBufferReceive(
                            xDriverRxStream,
                            &byte,
                            1,
                            pdMS_TO_TICKS(20)
                          );
        
        if (received > 0) {
            // Accumulate byte into our printing buffer
            if (rxIndex < (int)sizeof(rxBuffer)) {
                rxBuffer[rxIndex++] = byte;
                diag.uart_rx_bytes++;
            }
        } else {
            // A timeout occurred (no bytes arrived for 20ms). 
            // If we have accumulated bytes, flush and print them!
            if (rxIndex > 0) {
                diag.uart_rx_frames++;
                
                printf("\n[Driver -> Pico]:");
                for (int i = 0; i < rxIndex; i++) {
                    printf(" 0x%02X", rxBuffer[i]);
                }
                printf("\n> ");
                fflush(stdout); // Force USB serial output to update instantly
                
                if (xDriverMsgQueue != NULL) {
                    DriverMsg_t msg;
                    msg.response_code = rxBuffer[1]; // Function/Status code
                    msg.length = rxIndex;
                    memcpy(msg.data, rxBuffer, rxIndex);
                    msg.timestamp_ms = xTaskGetTickCount();
                    
                    // Non-blocking send to avoid locking up receiver
                    xQueueSend(xDriverMsgQueue, &msg, 0); 
                }
                
                // Reset index for the next incoming stream packet
                rxIndex = 0;
            }
        }
    }
}

// ============================================================================
// TASK: vParserTask (Priority 4 - Real-time)
// ============================================================================

void vParserTask(void *pvParameters) {
    /*
     * Reads USB input and parses commands.
     * Blocks on getchar() for input.
     * 
     * TODO: Implement command table and modular handlers
     */
    char line_buffer[256];
    int line_idx = 0;
    
    if (vbs_should_log(3)) printf("[Parser] Task started\n");
    printf("\nVBS v2.0 Ready. Type 'help' for commands.\n");
    printf("> ");
    
    while (1) {
        // Use timed getchar to avoid blocking lower-priority tasks
        int ch = getchar_timeout_us(10000); // 10 ms timeout

        if (ch == PICO_ERROR_TIMEOUT) {
            // No input available; yield briefly to allow lower-priority tasks
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (ch == '\n' || ch == '\r') {
            if (line_idx > 0) {
                line_buffer[line_idx] = '\0';
                
                // Update heartbeat (any USB input counts as PC alive)
                last_pc_heartbeat_ms = xTaskGetTickCount();
                diag.pc_heartbeats++;
                
                if (vbs_should_log(3) || vbs_should_log(6)) printf("\n[Parser] Received: '%s'\n", line_buffer);

                // Simple command dispatch
                // Tokenize the input line (first word = command)
                char *cmd = strtok(line_buffer, " \t");
                if (cmd != NULL) {
                    for (char *p = cmd; *p; ++p) *p = tolower((unsigned char)*p);

                    if (strcmp(cmd, "help") == 0) {
                        // Print help immediately
                        printf("Available commands:\n");
                        printf("  help         - Show this help\n");
                        printf("  move N V     - Move N steps (signed) at velocity V (0-127)\n");
                        printf("                 e.g. move 4094 32  or move -4094 32\n");
                        printf("  move_pot P V - Move to potentiometer value P (0-511) at velocity V\n");
                        printf("                 e.g. move_pot 48 32  or move_pot 430 32\n");
                        printf("  diag         - Print diagnostics now\n");
                        printf("  pot          - Print current potentiometer value\n");
                        printf("  stop         - Send driver stop command\n");
                        printf("  change_PID   - change Kp or Kd driver param\n");
                    }
                    
                    else if (strcmp(cmd, "change_pid") == 0) {
                        char *param_name = strtok(NULL, " \t");
                        char *value_str = strtok(NULL, " \t");

                        if (param_name == NULL || value_str == NULL) {
                            printf("[Error] Usage: change_pid <kp|kd> <value>\n");
                        }
                        else {
                            // Normalize parameter name to lowercase
                            for (char *p = param_name; *p; ++p) *p = tolower((unsigned char)*p);
                            
                            long val = strtol(value_str, NULL, 10);
                            if (val < 0 || val > 65535) {
                                printf("[Error] PID value must be between 0 and 65535\n");
                            }
                            else {
                                MotorCmd_t pid_cmd = {0};
                                pid_cmd.speed = (uint16_t)val; // Store the 16-bit PID value
                                
                                if (strcmp(param_name, "kp") == 0) {
                                    pid_cmd.cmd_type = MCTL_SET_PID_KP;
                                    pid_cmd.max_pulses = 0xA1; // Function code for Kp
                                    printf("Queueing Kp update to %ld...\n", val);
                                }
                                else if (strcmp(param_name, "kd") == 0) {
                                    pid_cmd.cmd_type = MCTL_SET_PID_KD;
                                    pid_cmd.max_pulses = 0xA3; // Function code for Kd
                                    printf("Queueing Kd update to %ld...\n", val);
                                }
                                else {
                                    printf("[Error] Unknown parameter '%s'. Use 'kp' or 'kd'.\n", param_name);
                                    pid_cmd.cmd_type = MCTL_IDLE;
                                }

                                if (pid_cmd.cmd_type != MCTL_IDLE) {
                                    xQueueSend(xMotorCmdQueue, &pid_cmd, portMAX_DELAY);
                                    diag.motor_cmd_queued++;
                                }
                            }
                        }
                    }
                    
                    else if (strcmp(cmd, "verbose") == 0) {
                        // Set verbose level: 0..6
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
                    
                    else if (strcmp(cmd, "diag") == 0) {
                        // Print diagnostics snapshot (same info as diagnostics task)
                        printf("\n--- DIAGNOSTICS (on-demand) ---\n");
                        printf("State: %s\n", stateToString(sys_state));
                        printf("Fault: %s\n", faultToString(sys_fault_code));
                        printf("Potentiometer: %d [%d - %d]\n", getPotValue(), MINIMAL_THRESHOLD, MAXIMUM_THRESHOLD);
                        printf("Time since PC heartbeat: %lu ms\n", xTaskGetTickCount() - last_pc_heartbeat_ms);
                        printf("UART RX bytes: %lu\n", diag.uart_rx_bytes);
                        printf("UART RX frames: %lu\n", diag.uart_rx_frames);
                        printf("CRC errors: %lu\n", diag.uart_checksum_errors);
                        printf("Stream buffer overflows: %lu\n", diag.stream_buffer_overflows);
                        printf("Commands queued: %lu\n", diag.motor_cmd_queued);
                        printf("Moves completed: %lu\n", diag.motor_move_complete);
                        printf("Move timeouts: %lu\n", diag.motor_move_timeout);
                        printf("--- end diagnostics ---\n\n");
                    }
                    
                    else if (strcmp(cmd, "pot") == 0) {
                        printf("Potentiometer (raw ADC avg): %d\n", getPotValue());
                    }
                    
                    else if (strcmp(cmd, "stop") == 0) {
                        sendStopCommand();
                        printf("Stop command sent to driver\n");
                    }
                    
                    else if (strcmp(cmd, "move") == 0) {
                        // move <steps> <speed>
                        char *arg1 = strtok(NULL, " \t");
                        char *arg2 = strtok(NULL, " \t");
                        if (arg1 == NULL || arg2 == NULL) {
                            printf("Usage: move <steps> <velocity>\n");
                        }
                        else {
                            long steps = strtol(arg1, NULL, 10);
                            int speed = atoi(arg2) & 0x7F;
                            if (speed > MAX_SPEED_VAL) speed = MAX_SPEED_VAL;
                            // Match original firmware: negative steps => direction=1
                            uint8_t direction = (steps < 0) ? 1 : 0;
                            uint32_t pulses = (uint32_t)(steps < 0 ? -steps : steps);
                            if (pulses == 0) {
                                printf("No pulses to send (0)\n");
                            }
                            else {
                                sendMovePulses(pulses, direction, (uint8_t)speed);
                                printf("Sent move pulses=%lu dir=%u speed=%u\n", (unsigned long)pulses, (unsigned)direction, (unsigned)speed);
                            }
                        }
                    } 
                    
                    else if (strcmp(cmd, "move_pot") == 0) {
                        // move_pot <pot_value> <speed>
                        char *arg1 = strtok(NULL, " \t");
                        char *arg2 = strtok(NULL, " \t");
                        if (arg1 == NULL || arg2 == NULL) {
                            printf("Usage: move_pot <pot_value> <velocity>\n");
                        }
                        else {
                            int pot = atoi(arg1);
                            int speed = atoi(arg2) & 0x7F;
                            if (pot < 0) pot = 0;
                            if (pot > 511) pot = 511;
                            if (speed > MAX_SPEED_VAL) speed = MAX_SPEED_VAL;

                            // Determine direction based on current pot
                            uint16_t cur = getPotValue();
                            uint8_t direction = (cur > pot) ? 1 : 0;

                            // Send single constant-move command and poll potentiometer
                            sendConstantMoveCommand(direction, (uint8_t)speed);
                            printf("Started move_pot target=%d speed=%d (dir=%u)\n", pot, speed, (unsigned)direction);

                            // Poll until within tolerance
                            uint32_t start = xTaskGetTickCount();
                            while (1) {
                                vTaskDelay(pdMS_TO_TICKS(50));
                                uint16_t now = getPotValue();
                                if (abs((int)now - pot) <= 3) {
                                    break;
                                }
                                // Optional timeout (900s)
                                if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(900000)) {
                                    printf("move_pot: timeout waiting for pot target %d (current %d)\n", pot, now);
                                    break;
                                }
                            }

                            // Send stop
                            sendStopCommand();
                            printf("move_pot: stopped at pot=%d (target %d)\n", getPotValue(), pot);
                        }
                    }
                    else {
                        printf("Unknown command: '%s'\n", cmd);
                    }
                }
                
                line_idx = 0;
            }
            printf("> ");
        }
        else if (ch >= 32 && ch < 127) {
            // Printable ASCII
            if (line_idx < sizeof(line_buffer) - 1) {
                line_buffer[line_idx] = (char)ch;
                line_idx++;
                putchar(ch);  // Echo
            }
        }
        else if (ch == '\b' || ch == 127) {
            // Backspace
            if (line_idx > 0) {
                line_idx--;
                putchar('\b');
                putchar(' ');
                putchar('\b');
            }
        }
    }
}

// ============================================================================
// TASK: vPotentiometerTask (Priority 3 - Utility)
// ============================================================================

void vPotentiometerTask(void *pvParameters) {
    /*
     * Samples ADC and updates potentiometer_value (protected by mutex).
     * Runs periodically at 20 Hz (50ms).
     */
    if (vbs_should_log(4)) printf("[Potentiometer] Task started\n");
    
    // Initialize ADC
    adc_init();
    adc_gpio_init(POT_ADC_PIN);
    adc_select_input(POT_ADC_CHANNEL);
    
    uint32_t sample_sum = 0;
    uint32_t loop_count = 0;

    while (1) {
        // Accumulate POT_SAMPLE_COUNT samples using 32-bit accumulator to avoid overflow
        sample_sum = 0;
        for (int i = 0; i < POT_SAMPLE_COUNT; i++) {
            sample_sum += adc_read();
        }

                // Average (result fits into 16-bit)
                uint16_t avg = (uint16_t)(sample_sum / POT_SAMPLE_COUNT);

                // Scale 12-bit ADC (0..4095) down to 0..511
                uint16_t scaled = (uint16_t)(avg >> 3);

                setPotValue(scaled);

                // Debug: print occasional ADC sample to help diagnose wiring
                loop_count++;
                if (loop_count % 20 == 0) { // ~1 second
                    if (vbs_should_log(4) || vbs_should_log(6)) printf("[Pot] ADC raw=%u scaled=%u\n", (unsigned)avg, (unsigned)scaled);
                }

        // 50ms = 20 Hz sampling
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
// TASK: vFaultManagerTask (Priority 2 - Utility)
// ============================================================================

void vFaultManagerTask(void *pvParameters) {
    /*
     * Centralized fault and timeout handling:
     * - Detects PC timeout (300s)
     * - Handles limit switch faults
     * - Transitions to failsafe or critical error states
     * - Kicks watchdog
     */
    uint32_t last_hb_check_ms = xTaskGetTickCount();
    
    if (vbs_should_log(5)) printf("[FaultMgr] Task started\n");
    
    // Initialize watchdog (8 second timeout)
    watchdog_enable(8000, true);
    
    while (1) {
        uint32_t ulNotified = 0;
        uint32_t now_ms = xTaskGetTickCount();
        
        // Wait for fault notifications or periodic timeout check
        xTaskNotifyWait(0, 0xFFFFFFFFUL, &ulNotified, pdMS_TO_TICKS(100));
        
        // Check for PC timeout (300 seconds)
        uint32_t time_since_heartbeat = now_ms - last_pc_heartbeat_ms;
        if (time_since_heartbeat > PC_TIMEOUT_MS) {
            if (sys_state == SYS_OPERATIONAL) {
                printf("[FaultMgr] PC timeout (>%lu ms) → FAILSAFE_ASCENT\n", PC_TIMEOUT_MS);
                sys_state = SYS_FAILSAFE_ASCENT;
                sys_fault_code = FAULT_PC_TIMEOUT;
                diag.fault_count[FAULT_PC_TIMEOUT]++;
                
                // Queue auto-expand command
                MotorCmd_t failsafe_cmd = {
                    .cmd_type = MCTL_MOVING_UNTIL_POT,
                    .target_pot = MAXIMUM_THRESHOLD,
                    .speed = RECOMMENDED_SPEED_VAL,
                    .cmd_id = 0xFFFFFFFF,  // Special failsafe ID
                };
                xQueueSend(xMotorCmdQueue, &failsafe_cmd, 0);
            }
        }
        
        // Handle min limit fault: queue 1mm back-off (expand direction)
        if (ulNotified & NOTIF_MIN_LIMIT) {
            printf("[FaultMgr] **CRITICAL** Min limit switch triggered!\n");
            printf("[FaultMgr] Motor disabled by ISR (0xF3 0x00 sent)\n");
            printf("[FaultMgr] Initiating 1mm recovery back-off...\n");
            
            sys_state = SYS_CRITICAL_ERROR;
            sys_fault_code = FAULT_MIN_LIMIT_HIT;
            diag.fault_count[FAULT_MIN_LIMIT_HIT]++;
            
            // Brief delay before re-enabling to ensure driver received disable
            vTaskDelay(pdMS_TO_TICKS(50));
            
            // Re-enable driver for recovery move only
            sendEnableCommand();
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Send a single pulses move command and let driver run it
            uint8_t direction = 0; // expand away from min
            uint8_t speed = 32;
            uint32_t pulses = 65511;
            sendMovePulses(pulses, direction, speed);

            // Wait for driver ACK and completion (preferred) or fallback to pot stability
            bool ack_received = false;
            bool completed = false;
            DriverMsg_t drvmsg;
            const uint32_t ack_timeout_ms = 500;
            const uint32_t complete_timeout_ms = 15000;

            uint32_t t0 = xTaskGetTickCount();
            while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(ack_timeout_ms)) {
                if (xDriverMsgQueue != NULL && xQueueReceive(xDriverMsgQueue, &drvmsg, pdMS_TO_TICKS(50)) == pdTRUE) {
                    int status = (drvmsg.length > 0) ? drvmsg.data[0] : drvmsg.response_code;
                    if (vbs_should_log(2) || vbs_should_log(6)) {
                        printf("[FaultMgr] Driver msg during ACK wait: status=%d rc=0x%02X len=%u\n", status, drvmsg.response_code, drvmsg.length);
                    }
                    if (status == 1) { ack_received = true; break; }
                    if (status == 0) { break; }
                }
            }

            if (ack_received) {
                t0 = xTaskGetTickCount();
                while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(complete_timeout_ms)) {
                    if (xDriverMsgQueue != NULL && xQueueReceive(xDriverMsgQueue, &drvmsg, pdMS_TO_TICKS(200)) == pdTRUE) {
                        int status = (drvmsg.length > 0) ? drvmsg.data[0] : drvmsg.response_code;
                        if (vbs_should_log(2) || vbs_should_log(6)) {
                            printf("[FaultMgr] Driver msg during run wait: status=%d rc=0x%02X len=%u\n", status, drvmsg.response_code, drvmsg.length);
                        }
                        if (status == 2) { completed = true; break; }
                        if (status == 0) { break; }
                    }
                }
            }

            if (!completed) {
                // Fallback: wait for pot stability
                uint16_t last = getPotValue();
                int stable_count = 0;
                t0 = xTaskGetTickCount();
                while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(complete_timeout_ms)) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    uint16_t now = getPotValue();
                    if (now == last) {
                        stable_count++; if (stable_count >= 3) break;
                    }
                    else {
                        stable_count = 0;
                    }
                    last = now;
                }
            }

            // Stop driver and validate pot near MINIMAL_THRESHOLD
            sendStopCommand();
            uint16_t final_pot = getPotValue();
            if (final_pot < (MINIMAL_THRESHOLD - 3) || final_pot > (MINIMAL_THRESHOLD + 3)) {
                printf("[FaultMgr] Min-limit recovery: pot out of range (%u) after back-off\n", (unsigned)final_pot);
                sys_fault_code = FAULT_MIN_LIMIT_HIT;
            }
            else {
                printf("[FaultMgr] Min-limit recovery: pot OK (%u)\n", (unsigned)final_pot);
                sys_fault_code = FAULT_NONE;
                sys_state = SYS_OPERATIONAL;
            }
        }
        
        // Handle max limit fault: queue 1mm back-off (contract direction)
        if (ulNotified & NOTIF_MAX_LIMIT) {
            printf("[FaultMgr] **CRITICAL** Max limit switch triggered!\n");
            printf("[FaultMgr] Motor disabled by ISR (0xF3 0x00 sent)\n");
            printf("[FaultMgr] Initiating 1mm recovery back-off...\n");
            
            sys_state = SYS_CRITICAL_ERROR;
            sys_fault_code = FAULT_MAX_LIMIT_HIT;
            diag.fault_count[FAULT_MAX_LIMIT_HIT]++;
            
            // Brief delay before re-enabling to ensure driver received disable
            vTaskDelay(pdMS_TO_TICKS(50));
            
            // Re-enable driver for recovery move only
            sendEnableCommand();
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Send a single pulses move command and let driver run it
            uint8_t direction = 1; // contract away from max
            uint8_t speed = 32;
            uint32_t pulses = 65511;
            sendMovePulses(pulses, direction, speed);

            // Wait for driver ACK and completion (preferred) or fallback to pot stability
            bool ack_received = false;
            bool completed = false;
            DriverMsg_t drvmsg;
            const uint32_t ack_timeout_ms = 500;
            const uint32_t complete_timeout_ms = 15000;

            uint32_t t0 = xTaskGetTickCount();
            while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(ack_timeout_ms)) {
                if (xDriverMsgQueue != NULL && xQueueReceive(xDriverMsgQueue, &drvmsg, pdMS_TO_TICKS(50)) == pdTRUE) {
                    int status = (drvmsg.length > 0) ? drvmsg.data[0] : drvmsg.response_code;
                    if (vbs_should_log(2) || vbs_should_log(6)) {
                        printf("[FaultMgr] Driver msg during ACK wait: status=%d rc=0x%02X len=%u\n", status, drvmsg.response_code, drvmsg.length);
                    }
                    if (status == 1) { ack_received = true; break; }
                    if (status == 0) { break; }
                }
            }

            if (ack_received) {
                t0 = xTaskGetTickCount();
                while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(complete_timeout_ms)) {
                    if (xDriverMsgQueue != NULL && xQueueReceive(xDriverMsgQueue, &drvmsg, pdMS_TO_TICKS(200)) == pdTRUE) {
                        int status = (drvmsg.length > 0) ? drvmsg.data[0] : drvmsg.response_code;
                        if (vbs_should_log(2) || vbs_should_log(6)) {
                            printf("[FaultMgr] Driver msg during run wait: status=%d rc=0x%02X len=%u\n", status, drvmsg.response_code, drvmsg.length);
                        }
                        if (status == 2) { completed = true; break; }
                        if (status == 0) { break; }
                    }
                }
            }

            if (!completed) {
                // Fallback: wait for pot stability
                uint16_t last = getPotValue();
                int stable_count = 0;
                t0 = xTaskGetTickCount();
                while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(complete_timeout_ms)) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    uint16_t now = getPotValue();
                    if (now == last) {
                        stable_count++; if (stable_count >= 3) break;
                    }
                    else { stable_count = 0; }
                    last = now;
                }
            }

            // Stop driver and validate pot near MAXIMUM_THRESHOLD
            sendStopCommand();
            uint16_t final_pot = getPotValue();
            if (final_pot < (MAXIMUM_THRESHOLD - 3) || final_pot > (MAXIMUM_THRESHOLD + 3)) {
                printf("[FaultMgr] Max-limit recovery: pot out of range (%u) after back-off\n", (unsigned)final_pot);
                sys_fault_code = FAULT_MAX_LIMIT_HIT;
            }
            else {
                printf("[FaultMgr] Max-limit recovery: pot OK (%u)\n", (unsigned)final_pot);
                sys_fault_code = FAULT_NONE;
                sys_state = SYS_OPERATIONAL;
            }
        }
        
        // Handle other faults
        if (ulNotified & NOTIF_FAULT_DETECTED) {
            printf("[FaultMgr] Fault detected: %s\n", faultToString(sys_fault_code));
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
    if (vbs_should_log(5)) printf("[Diagnostics] Task started\n");
    
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
            
            printf("\nCommunication:\n");
            printf("  UART RX bytes: %lu\n", diag.uart_rx_bytes);
            printf("  UART RX frames: %lu\n", diag.uart_rx_frames);
            printf("  CRC errors: %lu\n", diag.uart_checksum_errors);
            printf("  Frame errors: %lu\n", diag.uart_frame_errors);
            printf("  Stream buffer overflows: %lu\n", diag.stream_buffer_overflows);
            
            printf("\nMotor Control:\n");
            printf("  Commands queued: %lu\n", diag.motor_cmd_queued);
            printf("  Moves completed: %lu\n", diag.motor_move_complete);
            printf("  Move timeouts: %lu\n", diag.motor_move_timeout);
            
            printf("\nFaults Detected:\n");
            printf("  Min limit: %lu\n", diag.fault_count[FAULT_MIN_LIMIT_HIT]);
            printf("  Max limit: %lu\n", diag.fault_count[FAULT_MAX_LIMIT_HIT]);
            printf("  PC timeout: %lu\n", diag.fault_count[FAULT_PC_TIMEOUT]);
            printf("  Driver checksum: %lu\n", diag.fault_count[FAULT_DRIVER_CHECKSUM]);
            printf("  Driver ACK timeout: %lu\n", diag.fault_count[FAULT_DRIVER_ACK_TIMEOUT]);
            printf("===================================\n\n");
        }
    }
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

void initializeHardware(void) {
    // Initialize USB stdio
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB to settle
    
    printf("\n=== VBS v2.0 Hardware Initialization ===\n");
    
    // Initialize UART for driver communication
    uart_init(DRIVER_UART, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    printf("[HW] UART initialized at %d baud\n", BAUD_RATE);
    
    
    uint8_t mode_packet[] = {DRIVER_ADDR, 0x82, 0x02}; 
    sendPacketWithChecksum(mode_packet, 3);

    uint8_t en_level_packet[] = {DRIVER_ADDR, 0x85, 0x00}; 
    sendPacketWithChecksum(en_level_packet, 3);

    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx_driver);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(DRIVER_UART, true, false);
    printf("[HW] UART RX interrupt enabled\n");
    
    // GPIO PIN_ENABLE_DRIVER is in "hold" mode on the driver (always enabled via hardware)
    // Driver enable/disable is controlled via UART commands (0xF3 0x01 = enable, 0xF3 0x00 = disable)
    // This GPIO pin is not actively used for control, but left configured for potential future use
    gpio_init(PIN_ENABLE_DRIVER);
    gpio_set_dir(PIN_ENABLE_DRIVER, GPIO_OUT);
    gpio_put(PIN_ENABLE_DRIVER, 0);  // Set LOW (driver enabled by default, controlled via UART)
    
    // Initialize GPIO for limit switches (inputs, pull-down)
    gpio_init(SW_MIN_LIMIT);
    gpio_set_dir(SW_MIN_LIMIT, GPIO_IN);
    gpio_pull_down(SW_MIN_LIMIT);
    
    gpio_init(SW_MAX_LIMIT);
    gpio_set_dir(SW_MAX_LIMIT, GPIO_IN);
    gpio_pull_down(SW_MAX_LIMIT);
    
    // Set up GPIO interrupt callback for limit switches
    gpio_set_irq_callback(&gpio_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_set_irq_enabled(SW_MIN_LIMIT, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(SW_MAX_LIMIT, GPIO_IRQ_EDGE_FALL, true);
    printf("[HW] Limit switches configured with interrupt\n");
    
    printf("=== Hardware Initialization Complete ===\n\n");
}

// ============================================================================
// RTOS INITIALIZATION
// ============================================================================

void initializeRTOS(void) {
    printf("=== RTOS Initialization ===\n");
    
    // Create synchronization primitives
    // Potentiometer protection uses critical sections; no mutex created here
    printf("[RTOS] Potentiometer access protected by critical sections\n");
    
    // Create stream buffer for driver UART RX
    xDriverRxStream = xStreamBufferCreate(DRIVER_RX_STREAM_SIZE, 1);
    if (xDriverRxStream == NULL) {
        printf("[RTOS] ERROR: Failed to create driver RX stream buffer\n");
        return;
    }
    printf("[RTOS] Driver RX stream buffer created (%d bytes)\n", DRIVER_RX_STREAM_SIZE);
    
    // Create motor command queue
    xMotorCmdQueue = xQueueCreate(MOTOR_CMD_QUEUE_SIZE, sizeof(MotorCmd_t));
    if (xMotorCmdQueue == NULL) {
        printf("[RTOS] ERROR: Failed to create motor command queue\n");
        return;
    }
    printf("[RTOS] Motor command queue created (%d items)\n", MOTOR_CMD_QUEUE_SIZE);
    
    // Create driver response queue
    xDriverMsgQueue = xQueueCreate(DRIVER_MSG_QUEUE_SIZE, sizeof(DriverMsg_t));
    if (xDriverMsgQueue == NULL) {
        printf("[RTOS] ERROR: Failed to create driver response queue\n");
        return;
    }
    printf("[RTOS] Driver response queue created (%d items)\n", DRIVER_MSG_QUEUE_SIZE);
    
    // Create tasks (priority order: higher number = higher priority)
    if (xTaskCreate(vMotorControlTask, "MotorControl", 256, NULL, 6, &xMotorControlTaskHandle) != pdPASS) {
        printf("[RTOS] ERROR: Failed to create vMotorControlTask\n");
        return;
    }
    printf("[RTOS] vMotorControlTask created (priority 6)\n");
    
    if (xTaskCreate(vReceiverTask, "Receiver", 512, NULL, 5, &xReceiverTaskHandle) != pdPASS) {
        printf("[RTOS] ERROR: Failed to create vReceiverTask\n");
        return;
    }
    printf("[RTOS] vReceiverTask created (priority 5)\n");
    
    if (xTaskCreate(vParserTask, "Parser", 512, NULL, 4, &xParserTaskHandle) != pdPASS) {
        printf("[RTOS] ERROR: Failed to create vParserTask\n");
        return;
    }
    printf("[RTOS] vParserTask created (priority 4)\n");
    
    if (xTaskCreate(vPotentiometerTask, "Potentiometer", 256, NULL, 3, &xPotTaskHandle) != pdPASS) {
        printf("[RTOS] ERROR: Failed to create vPotentiometerTask\n");
        return;
    }
    printf("[RTOS] vPotentiometerTask created (priority 3)\n");
    
    if (xTaskCreate(vFaultManagerTask, "FaultManager", 256, NULL, 2, &xFaultMgrTaskHandle) != pdPASS) {
        printf("[RTOS] ERROR: Failed to create vFaultManagerTask\n");
        return;
    }
    printf("[RTOS] vFaultManagerTask created (priority 2)\n");
    
    if (xTaskCreate(vDiagnosticsTask, "Diagnostics", 256, NULL, 1, &xDiagnosticsTaskHandle) != pdPASS) {
        printf("[RTOS] ERROR: Failed to create vDiagnosticsTask\n");
        return;
    }
    printf("[RTOS] vDiagnosticsTask created (priority 1)\n");
    
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

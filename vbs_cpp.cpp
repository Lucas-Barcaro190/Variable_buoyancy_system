// TODO:
// 1) Create a function that calibrates the pulses counter at startup.
// 2) Adapt the expand and contract command to better fit the 10 Hz frequency.
// 3) Implement a protect system that ensures the driver returned the finished message before processing the next user command.
//    - Maybe use semaphore is the easiest way to do this.
//    - Make sure that the the user receives a message when it tries to send a command while the driver is still running.
//        - Maybe print a message when the driver finishes too so the user can implement a function that only sends a new command
//          when it can be sure that the movement stopped.
// 4) ...

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "MS5837.h"

// --- HARDWARE CONFIGS ---
#define DRIVER_UART uart1
#define BAUD_RATE 38400
#define FREQUENCY_HZ_FOR_USER_INPUT 10
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define DRIVER_ADDR 0xE0
#define MAX_SPEED_VAL 127
#define RECOMENDED_SPEED_VAL 2
#define MAX_PULSES 68820      // tested in 16.02.26 @ 2 of speed (300 rpm)

#define PIN_ENABLE_DRIVER 16  // High = Driver OFF, Low = Driver ON
#define SW_MIN_LIMIT 6        // Contracted switch for minimum limit
#define SW_MAX_LIMIT 7        // Expanded switch for maximum limit


// Global secure state variables for fault handling
volatile bool sys_fault = false;
volatile uint8_t sys_error = 0;
volatile bool sys_back = false;

// Depth sensor global variables
volatile float depth_sensor_value = 0;  // Current depth reading in meters
volatile float pressure_offset = 0.0f;  // Pressure offset for depth calibration (set via 'set_pressure' command)
volatile bool depth_sensor_initialized = false;  // Track if depth sensor is initialized

// Internal pulses counter
volatile uint32_t pulses_counter = 34410; // start in the middle (first test only).

MS5837 depthSensor; // Create an instance of the MS5837 class to be used for reading depth sensor data

// --- STRUCTS ---
struct SimpleCommand {
    const char* name;
    uint8_t bytes[2];
    int length;
};
 
const SimpleCommand simpleCommands[] = {
    {"enable",       {0xF3, 0x01}, 2},
    {"disable",      {0xF3, 0x00}, 2},
    {"stop",         {0xF7},       1},
    {"read_encoder", {0x30},       1},
    {"set_en_low",   {0xB3, 0x00}, 2},
    {"read_pulses",  {0x33},       1},
    {"set_ms",       {0x84},       1},
    {"cal",          {0x80, 0x00}, 2},
    {"expand",       {0x42},       1},
    {"contract",     {0x43},       1},
    {"depth_sensor", {0x44},       1}
};
const int numSimpleCommands = sizeof(simpleCommands) / sizeof(simpleCommands[0]);

QueueHandle_t xUartQueue;

// --- INTERRUPT HANDLER (RX DRIVER) ---
extern "C" {
    void on_uart_rx_driver() {
        /*
        params:
         - This function is triggered by the UART RX interrupt when a byte is received from the driver.
        behavior:
         - It reads all available bytes from the UART RX buffer and sends them to the xUartQueue for processing by the receiver task.
         - It uses portYIELD_FROM_ISR to yield to a higher priority task if one was woken by sending to the queue, ensuring prompt 
           processing of incoming messages from the driver.
         - The use of BaseType_t and pdFALSE is standard practice in FreeRTOS ISR handlers for managing task wakeup and yielding 
           correctly.
        */
        while (uart_is_readable(DRIVER_UART)) {
            uint8_t ch = uart_getc(DRIVER_UART);
            
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            
            if (xUartQueue != NULL) {
                xQueueSendFromISR(xUartQueue, &ch, &xHigherPriorityTaskWoken);
            }
            
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

// --- AUXILIARES DE COMUNICAÇÃO ---
void sendPacketWithChecksum(uint8_t* packetData, int len) {
    /*
    params:
     - packetData: Array of bytes representing the packet to be sent to the driver (excluding checksum).
     - len: The length of the packetData array.
    behavior:
     - Calculates a simple checksum by summing all bytes in the packetData and taking the result modulo 256
       (using uint8_t to automatically wrap around).
    */
    uint8_t checksum = 0; // Uses uint8_t to automatically && with 0xFF
    
    for (int i = 0; i < len; i++) {
        checksum += packetData[i];
        uart_putc(DRIVER_UART, packetData[i]);
    }
    
    uart_putc(DRIVER_UART, checksum);
}

// --- MOVE COMMAND HANDLER ---
void handleMoveCommand(char* args) {
    /*
    params:
     - args: The argument string passed to the move command (e.g. "3200 30").
    behavior:
     - Parses the number of pulses and speed from the args string.
    */
    if (args == NULL || strlen(args) == 0) {
        printf("\n[Error]: 'move' requires <pulses> and <speed> (e.g. move 3200 30)\n");
        return;
    }

    char* next_ptr;
    long pulses = strtol(args, &next_ptr, 10);
    int speed_raw = (int)strtol(next_ptr, NULL, 10);
    
    // Fault handling for limit switches
    if(sys_fault){
        switch (sys_error) {
        
        case SW_MIN_LIMIT:
            if (pulses < 0) {
                printf("\n[Error]: Move to MIN LIMIT blocked by min switch.\n");
                return;
            }
            break;

        case SW_MAX_LIMIT:
            if (pulses > 0) {
                printf("\n[Error]: Move to MAX LIMIT blocked by max switch.\n");
                return;
            }
            break;
        default:
            // to do: other errors (?)
            break;
        }
    }

    uint8_t speed_val = (uint8_t)abs(speed_raw); // makes sure it's positive
    if (speed_val > MAX_SPEED_VAL) speed_val = MAX_SPEED_VAL; // caps speed at max value

    uint8_t packet[7];
    packet[0] = DRIVER_ADDR;
    packet[1] = 0xFD; // Command 'move'
    
    uint8_t direction = (pulses < 0) ? 1 : 0;
    
    // Makes sure it doesnt try to move further if it is already in the limit switch, but allows to move in the opposite
    // direction to get out of the switch
    if(!direction && sys_error == SW_MIN_LIMIT && !sys_back){
        sys_error = 0;
        sys_fault = false;
    }
    if(direction && sys_error == SW_MAX_LIMIT && !sys_back){
        sys_error = 0;
        sys_fault = false;
    }

    uint32_t abs_pulses = (uint32_t)labs(pulses);
    
    // Byte 2: bit 7 = Direction, bits 0-6 = speed
    packet[2] = (direction << 7) | (speed_val & 0x7F); // Direction in MSB, speed in lower 7 bits

    // Pulses in 4 bytes (Big Endian)
    for (int i = 0; i < 4; i++) {
        packet[3 + i] = (abs_pulses >> (24 - 8 * i)) & 0xFF;
    }
    // equivalent to:
    //
    // packet[3] = (abs_pulses >> 24) & 0xFF;
    // packet[4] = (abs_pulses >> 16) & 0xFF;
    // packet[5] = (abs_pulses >> 8) & 0xFF;
    // packet[6] = abs_pulses & 0xFF;
    //
    //or:
    //
    // packet: [ADDR][CMD][DIR+SPEED ( 1 bit: dir; 7 bits: speed )][PULSES (4 bytes)]

    sendPacketWithChecksum(packet, 7);
    printf("\n[Pico -> Driver]: Move %ld pulses, speed: %d\n", pulses, speed_val);
}

void read_depth_sensor() {
    /*
    params:
     - depth_sensor_value: A global variable to hold the current value read from the depth sensor.
    behavior:
     - sends via serial the last read value from the depth sensor, which is updated in the background by another task that continuously reads the sensor (to be implemented).
     - This allows the user to get real-time depth information on demand without blocking the main command
    */
    printf("CURRENT_DEPTH: %.3f millimeters\n", depth_sensor_value);
}

// --- STRING PROCESS ---
void processCommand(const char* line) {
    /*
    params:
     - line: The raw input command string from the user (e.g. "move 3200 30").
    behavior:
     - Parses the command name and arguments from the input line.
    */
    char localBuffer[64];
    strncpy(localBuffer, line, sizeof(localBuffer));
    localBuffer[sizeof(localBuffer) - 1] = '\0'; // Ensure null-termination

    for(int i = 0; localBuffer[i]; i++) localBuffer[i] = (char)tolower((char)localBuffer[i]); // Convert to lowercase for case-insensitive comparison

    char* cmd_name = strtok(localBuffer, " "); // Get the command name (first token)
    char* args = strtok(NULL, ""); // Get the rest of the line as arguments (if any)

    if (cmd_name == NULL) return; // to do: maybe print error for empty command?
    if (strcmp(cmd_name, "help") == 0) {
        printf("\nAvailable commands:\n");
        printf(" - move <pulses> <speed>: Move the motor by a specified number of pulses at a given speed (e.g. move 3200 30)\n");
        printf(" - expand: Shortcut to expand with predefined parameters (equivalent to move 99 2)\n");
        printf(" - contract: Shortcut to contract with predefined parameters (equivalent to move -99 2)\n");
        printf(" - stop: Immediately stops the motor\n");
        printf(" - enable: Enables the driver (allows movement)\n");
        printf(" - disable: Disables the driver (cuts power, no movement possible)\n");
        printf(" - depth_sensor: Reads the depth sensor value (if connected)\n");

        for (int i = 0; i < numSimpleCommands; i++) {
            printf(" - %s\n", simpleCommands[i].name);
        }
        return;
    }

    if (strcmp(cmd_name, "move") == 0) {
        handleMoveCommand(args); // handles move command separately due to its complexity and argument parsing
        return;
    }
    // baudrate: 115200
    // W.C.S
    // bit time: 1/115200 = 8.68 us
    // pkg time: (1 start bit + 8 data bits + 1 parity bit + 1 stop bit) * 8.68 us = 11 bits * 8.68 us =  95.48 us per pkg
    // pkg expand => 6 bytes => 6 pkgs => 572.88 us
    // pkg contract => 8 bytes => 8 pkgs => 763.84 us
    // 10 Hz => 100 ms => - 1ms of pkg send time
    // driver speed = 2 => 300 rpm => 5 rps => 200 pulses per rotation => 1000 pulses per second => 100 pulses per 100 ms
    // 99 pulses per 100 ms excluding the time it takes to receive the command from user.
    // The time it takes to the driver to receive the command doesnt matter because it can work in parallel with the movement
    // and the last command is still running while the user is sending the next command.

    if (strcmp(cmd_name, "expand") == 0) {
        handleMoveCommand("50 2"); // Shortcut command to expand with predefined parameters
        return;
    }
    if (strcmp(cmd_name, "contract") == 0) {
        handleMoveCommand("-50 2"); // Same as above but for contract
        return;
    }
    if (strcmp(cmd_name, "depth_sensor") == 0) {
        read_depth_sensor();
        return;
    }

    for (int i = 0; i < numSimpleCommands; i++) {
        if (strcmp(cmd_name, simpleCommands[i].name) == 0) {
            uint8_t packet[3]; // max length is 3 for simple commands (ADDR + CMD + 1 optional ARG)
            
            packet[0] = DRIVER_ADDR;
            
            memcpy(&packet[1], simpleCommands[i].bytes, simpleCommands[i].length);
            
            sendPacketWithChecksum(packet, simpleCommands[i].length + 1);
            
            printf("\n[Pico -> Driver]: Sending '%s' command.\n", cmd_name);
            return;
        }
    }
    printf("\n[Error]: '%s' unknown command.\n", cmd_name);
}

// --- INTERRUPT HANDLERS (LIMIT SWITCHES) ---
static absolute_time_t last_interrupt_time;
void gpio_callback_edge_fall(uint gpio, uint32_t events) {
    /*
    params:
     - gpio: The GPIO number that triggered the interrupt (either SW_MIN_LIMIT or SW_MAX_LIMIT).
     - events: The type of event that triggered the interrupt (should be GPIO_IRQ_EDGE_FALL in this case).
    behavior:
     - When a limit switch is triggered (falling edge), it immediately stops the driver to prevent damage.
     - It sets the sys_fault flag to true and records which switch was triggered in sys_error for diagnostic purposes.
     - It then re-enables the driver but since the switch is still pressed, any attempt to move further in that direction
       will be blocked until the switch is released.
     - The function uses disable_interrupts() and enable_interrupts() to ensure that the critical section of stopping the 
       driver and updating fault state is not interrupted by other interrupts, ensuring consistent behavior.
     - The processCommand("stop") is used to send a stop command to the driver immediately when a limit switch is hit.
    */
    // Imediate hardware action: Cuts Driver movement to prevent damage
    disable_interrupts();
    processCommand("stop\0");
    
    sys_fault = true;

    // diagnostic info for fault handling and making sure the system knows which switch was hit to prevent further
    // movement in that direction until the switch is released
    if (gpio == SW_MIN_LIMIT) {
        sys_error = SW_MIN_LIMIT;
    } else if (gpio == SW_MAX_LIMIT) {
        sys_error = SW_MAX_LIMIT;
    }
    sys_back = true;
    processCommand("enable\0"); // Re-enable driver after stopping, so it can only be moved in the opposite direction until it
                                // get out of the limit switch
    if (sys_error == SW_MIN_LIMIT) {
        processCommand("move 1720 2");    // 2102 for Portugal VBS and 1720 for LaSub VBS (1mm)
    } else if (sys_error == SW_MAX_LIMIT) {
        processCommand("move -1720 2");   // 2102 for Portugal VBS and 1720 for LaSub VBS (1mm)
    }
    sys_back = false;
    enable_interrupts();
}

// --- TASKS ---
void vDepthSensorTask(void *pvParameters) {
    /*
    params:
     - depth_sensor_value: A global variable to hold the current value read from the depth sensor.
     - pressure_offset: A global variable to hold the reference pressure for depth calibration.
    behavior:
     - Continuously reads the depth sensor value at a defined frequency (e.g. 20 Hz) and updates the global variable.
     - This allows the system to have an up-to-date depth value that can be accessed on demand by the command parser when the
       user requests it.
     - The task uses vTaskDelay to wait between readings, allowing other tasks to run and ensuring that it does not block the system.
     - This task takes up to 90 ms (up to 40ms for reading the sensor and 50 ms for the delay) to update the depth_sensor_value.
     - The sensor is pre-initialized in prvSetupHardware() before the RTOS scheduler starts.
    */
    // Check if sensor was properly initialized at startup

    if (!depthSensor.init(i2c0)) {
        printf("[ERROR] Sensor fail\n");
    } else {
        // sets pressure offset to the initial pressure reading at startup.
        depthSensor.read(); 
        pressure_offset = depthSensor.pressure(MS5837::Pa);
        depth_sensor_initialized = true;
        printf("[Depth sensor]: Successfully initialized on I2C0. Initial pressure offset set to %.2f Pa at startup. Depth is now 0 meters.\n", pressure_offset);
    }

    while (1) {
        if (depth_sensor_initialized) {
            depthSensor.read();
            // Calculate depth relative to pressure offset (reference pressure)
            // When set_pressure is called, pressure_offset stores the reference pressure
            float current_pressure = depthSensor.pressure(MS5837::Pa);  // Get absolute pressure in Pa
            depth_sensor_value = (current_pressure - pressure_offset) / (depthSensor.mbar * 1000.0f * 9.80665f);
            
            vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms delay here + up to 40 ms for reading the sensor = up to 90 ms update time for
                                           // depth_sensor_value, enough for a 10 Hz update rate
        }
        else{
            printf("[Depth Sensor Task]: Sensor not initialized. Depth readings unavailable.\n");
            
            static uint32_t retry_delay = 1000;                            // Start with 1 second
            vTaskDelay(pdMS_TO_TICKS(retry_delay));                        // exponential wait between retries
            retry_delay = (retry_delay < 32000) ? retry_delay * 2 : 32000; // Cap at 32 seconds
        }
    }
}

void vReceiverTask(void *pvParameters) {
    /*
    params:
     - xUartQueue: Queue where the UART RX interrupt handler sends received bytes from the driver.
     - rxBuffer: Buffer to accumulate received bytes until a timeout occurs (indicating end of message).
     - rxIndex: Index to keep track of the current position in the rxBuffer.
    behavior:
     - Continuously waits for bytes from the UART RX interrupt via xUartQueue.
    */
    uint8_t byte;
    uint8_t rxBuffer[64];
    int rxIndex = 0;

    while (true) {
        if (xQueueReceive(xUartQueue, &byte, pdMS_TO_TICKS(20))) { // Waits for a byte with a timeout of 20ms
            if (rxIndex < (int)sizeof(rxBuffer)) rxBuffer[rxIndex++] = byte;
        } else if (rxIndex > 0) { // If timeout occurs and we have received bytes, process the message
            printf("\n[Driver -> Pico]: ");
            for (int i = 0; i < rxIndex; i++) printf("0x%02X ", rxBuffer[i]); // Print received bytes in hex format
            printf("\n> ");
            rxIndex = 0;
        }
    }
}

void vParserTask(void *pvParameters) {
    /*
    params:
     - inputBuffer: Buffer to accumulate user input from the console until the user presses Enter.
     - inputIndex: Index to keep track of the current position in the inputBuffer.
    behavior:
        - Continuously reads characters from the console input.
        - Handles backspace for editing input.
        - When Enter is pressed, processes the command in inputBuffer and resets it for the next command.
        - Uses vTaskDelay to yield to other tasks while waiting for user input.
    */
    char inputBuffer[128];
    int inputIndex = 0;
    printf("\n--- VBS Console Online ---\n> ");

    while (true) {
        int c = getchar_timeout_us(10000); 
        if (c != PICO_ERROR_TIMEOUT) { // If a character is received (not a timeout)
            if (c == '\r' || c == '\n') {
                putchar('\n');
                inputBuffer[inputIndex] = '\0';
            
                if (inputIndex > 0) processCommand(inputBuffer); // Process the command when Enter is pressed
            
                inputIndex = 0;
                printf("> ");
            
            } else if (c == 8 || c == 127) { // Backspace
                if (inputIndex > 0) {
                    inputIndex--;
                    printf("\b \b"); // Move cursor back, print space to erase character, and move back again
                }
            } else {
                putchar(c);
                if (inputIndex < (int)sizeof(inputBuffer) - 1) inputBuffer[inputIndex++] = (char)c; // Add character to input buffer if there's space (leave room for null terminator)
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- SETUP ---
extern "C" void prvSetupHardware(void) {
    stdio_init_all();

    gpio_init(PIN_ENABLE_DRIVER);
    gpio_set_dir(PIN_ENABLE_DRIVER, GPIO_OUT);
    gpio_put(PIN_ENABLE_DRIVER, 0); // Initiates connected (Active Low)

    gpio_init(SW_MIN_LIMIT);    // Initialize GPIO for minimum limit switch
    gpio_set_dir(SW_MIN_LIMIT, GPIO_IN);
    gpio_pull_up(SW_MIN_LIMIT);

    gpio_init(SW_MAX_LIMIT);    // Initialize GPIO for maximum limit switch
    gpio_set_dir(SW_MAX_LIMIT, GPIO_IN);
    gpio_pull_up(SW_MAX_LIMIT);

    // Initialize I2C at 100kHz
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(8, GPIO_FUNC_I2C); // SDA
    gpio_set_function(9, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(8);
    gpio_pull_up(9);

    // Registers an interrupt for the falling edge (when the switch closes).
    gpio_set_irq_enabled_with_callback(SW_MIN_LIMIT, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_edge_fall);
    // The callback will be triggered when the switch is pushed, which is the critical moment to stop the driver to prevent damage.
    // The falling edge is used because the switch is active low (connected to ground when pressed).
    gpio_set_irq_enabled(SW_MAX_LIMIT, GPIO_IRQ_EDGE_FALL, true);

    uart_init(DRIVER_UART, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Initial configuration packets to set up the driver in the desired mode (e.g. microstepping, current level, etc.)
    uint8_t mode_packet[] = {DRIVER_ADDR, 0x82, 0x02}; 
    sendPacketWithChecksum(mode_packet, 3);

    uint8_t en_level_packet[] = {DRIVER_ADDR, 0x85, 0x00}; 
    sendPacketWithChecksum(en_level_packet, 3);

    // Create the UART queue for communication between the RX interrupt handler and the receiver task
    xUartQueue = xQueueCreate(128, sizeof(uint8_t));
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx_driver); // Set the UART RX interrupt handler for DRIVER_UART (uart1)
    irq_set_enabled(UART1_IRQ, true); // Enable UART1 interrupt in the NVIC
    uart_set_irq_enables(DRIVER_UART, true, false); // Enable RX interrupt, disable TX interrupt (we don't need it for this project)
    
}

int main() {
    // Hardware setup
    prvSetupHardware();

    // Create tasks for command parsing and UART reception
    xTaskCreate(vParserTask, "Parser", 2048, NULL, 4, NULL); // Higher priority for command parsing from the user
    xTaskCreate(vReceiverTask, "Receiver", 2048, NULL, 5, NULL); // Higher priority for receiver to ensure we process incoming messages 
                                                                 // from the driver promptly

    xTaskCreate(vDepthSensorTask, "Depth", 2048, NULL, 3, NULL); // lower priority than communication with driver and user. Reads and updates depth with offset calibration
    
    // Remember: while the raspberry receives messages from the computer, the driver can still be running, since it only stops
    //           the last command if a stop command is sent, if it hits a limit switch or it ends the last command.
    //           So the movement can still occur while the user is sending a command to raspberry.
    //
    // NOTE: The driver does stop the last command if it receives a new command, but it doesnt say when it stopped (how many steps it did)
    // it is up to the user to keep in mind this and not send two commands in a row without waiting for the first one to end, otherwise 
    // it can cause unexpected behavior 
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}
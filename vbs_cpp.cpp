// TODO:
// 1. Create a function that calibrates the pulses counter at startup.
// 2. Adapt the expand and contract command to better fit the 10 Hz frequency.
// 3. Implement a protect system that ensures the driver returned the finished message before processing the next user command.
//    - Maybe use semaphore is the easiest way to do this.
//    - Make sure that the the user receives a message when it tries to send a command while the driver is still running.
//        - Maybe print a message when the driver finishes too so the user can implement a function that only sends a new command
//          when it can be sure that the movement stopped.
// 4. ...

// Remember: while the raspberry receives messages from the computer, the driver can still be running, since it only stops
//           the last command if a stop command is sent, if it hits a limit switch or it ends the last command.
//           So the movement can still occur while the user is sending a command to raspberry.
//
// NOTE: The driver does stop the last command if it receives a new command, but it doesnt say when it stopped (how many steps it did)
//       it is up to the user to keep in mind this and not send two commands in a row without waiting for the first one to end, otherwise 
//       it can cause unexpected behavior 
//       The biggest frequency now is about 20 Hz (50 steps) but it is recommended to use 10 Hz just to make sure

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
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
#define MAX_speedVal 127
#define RECOMENDED_speedVal 2
#define MAX_PULSES 68820        // tested in 16.02.26 @ 2 of speed (300 rpm)

#define PIN_ENABLE_DRIVER 16    // High = Driver OFF, Low = Driver ON
#define SW_MIN_LIMIT 6          // Contracted switch for minimum limit
#define SW_MAX_LIMIT 7          // Expanded switch for maximum limit
#define SW_MIN_LIMIT_ERROR -100
#define SW_MAX_LIMIT_ERROR -101
#define NOT_CUSTOM_COMMAND -2

// Global secure state variables for fault handling
volatile bool sysFault = false;
volatile uint8_t sysError = 0;

// Depth sensor global variables
volatile float depthSensorValue = 0;  // Current depth reading in meters
volatile float pressureOffset = 0.0f;  // Pressure offset for depth calibration (set via 'set_pressure' command)
volatile bool depthSensorInitialized = false;  // Track if depth sensor is initialized

// Internal pulses counter
volatile uint32_t pulsesCounter = 34410; // start in the middle (first test only).

volatile uint8_t driverBuffer[64];
volatile uint8_t *driverBufferPtr = driverBuffer;

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
            
            BaseType_t xHigherPriorityTaskWoken = pdFALSE; // ensures the compiler uses the most efficient type
            
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
     - Calculates a simple checksum by summing all bytes in the packetData and taking the result modulo 0xFF
       (using uint8_t to automatically wrap around).
    */
    uint8_t checksum = 0; // Uses uint8_t to automatically && with 0xFF
    
    for (int i = 0; i < len; i++) {
        checksum += packetData[i];
        uart_putc(DRIVER_UART, packetData[i]);
    }
    
    uart_putc(DRIVER_UART, checksum);
}

int8_t isSysFault(long pulses){
    /*
    params:
     - pulses: The number of pulses for which to check fault conditions.
    behavior:
     - Checks if a system fault is active and handles the appropriate error case based on the current system error state.
    */
    if(sysFault){
        switch (sysError) {
        
        case SW_MIN_LIMIT:
            if (pulses < 0) {
                printf("\n[Error]: Move to MIN LIMIT blocked by min switch.\n");
                return SW_MIN_LIMIT_ERROR;
            }
            break;

        case SW_MAX_LIMIT:
            if (pulses > 0) {
                printf("\n[Error]: Move to MAX LIMIT blocked by max switch.\n");
                return SW_MAX_LIMIT_ERROR;
            }
            break;
        default:
            // to do: other errors (?)
            break;
        }
    }
    return 0;
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

    char* nextPtr;
    long pulses = strtol(args, &nextPtr, 10);
    int speedRaw = (int)strtol(nextPtr, NULL, 10);
    // Fault handling for limit switches
    

    uint8_t speedVal = (uint8_t)abs(speedRaw); // makes sure it's positive
    if (speedVal > MAX_speedVal) speedVal = MAX_speedVal; // caps speed at max value

    uint8_t packet[7];
    packet[0] = DRIVER_ADDR;
    packet[1] = 0xFD; // Command 'move'
    
    uint8_t direction = (pulses < 0) ? 1 : 0;
    
    // Makes sure it doesnt try to move further if it is already in the limit switch, but allows to move in the opposite
    // direction to "get out" of the switch   
    int8_t faultResult = isSysFault(pulses);
    if(!direction && faultResult != SW_MAX_LIMIT_ERROR){ // could be written as !(direction || faultResult != SW_MAX_LIMIT_ERROR)
        sysError = 0;                                    // by the resulting truth table, same to the below one
        sysFault = false;
    }
    if(direction && faultResult != SW_MIN_LIMIT_ERROR){
        sysError = 0;
        sysFault = false;
    }
    pulsesCounter += (uint32_t)pulses; // TODO: continue implementing the absolute pulses counter 
    
    uint32_t absPulses = (uint32_t)labs(pulses);
    
    // Byte 2: bit 7 = Direction, bits 0-6 = speed
    packet[2] = (direction << 7) | (speedVal & 0x7F); // Direction in MSB, speed in lower 7 bits

    // Pulses in 4 bytes (Big Endian)
    for (int i = 0; i < 4; i++) {
        packet[3 + i] = (absPulses >> (24 - 8 * i)) & 0xFF;
    }
    // equivalent to:
    //
    // packet[3] = (absPulses >> 24) & 0xFF;
    // packet[4] = (absPulses >> 16) & 0xFF;
    // packet[5] = (absPulses >> 8) & 0xFF;
    // packet[6] = absPulses & 0xFF;
    //
    // or:
    //
    // packet: [ADDR][CMD][DIR+SPEED ( 1 bit: dir; 7 bits: speed )][PULSES (4 bytes)]

    sendPacketWithChecksum(packet, 7);
    printf("\n[Pico -> Driver]: Move %ld pulses, speed: %d\n", pulses, speedVal);
}

void readDepthSensor() {
    /*
    params:
     - depthSensorValue: A global variable to hold the current value read from the depth sensor.
    behavior:
     - sends via serial the last read value from the depth sensor, which is updated in the background by another task that continuously reads the sensor (to be implemented).
     - This allows the user to get real-time depth information on demand without blocking the main command
    */
    printf("CURRENT_DEPTH: %.3f centimeters\n", depthSensorValue);
}

void printHelp(void){
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
}

int8_t treatCustomCommand(const char* cmdName, char* args = NULL) {
    /*
    params:
     - cmdName: The name of the command to check and execute if it matches a custom command.
     - args: A string containing the arguments for the command (if applicable).
    behavior:
     - Checks if the cmdName matches any of the custom commands (e.g. "move", "depth_sensor", "help").
     - If it matches, it executes the corresponding function for that command and returns true.
     - If it does not match any custom command, it returns false, allowing the caller to check for simple commands
       or print an error message.
    */

    if (cmdName == NULL) return -1; // to do: maybe print error for empty command?
    
    if (strcmp(cmdName, "help") == 0) {
        printHelp();
        return 0;
    }

    if (strcmp(cmdName, "move") == 0) {
        handleMoveCommand(args); // handles move command separately due to its complexity and argument parsing
        return 0;
    }

    if (strcmp(cmdName, "expand") == 0) {
        handleMoveCommand("50 2"); // Shortcut command to expand with predefined parameters
        return 0;
    }

    if (strcmp(cmdName, "contract") == 0) {
        handleMoveCommand("-50 2"); // Same as above but for contract
        return 0;
    }

    if (strcmp(cmdName, "depth_sensor") == 0) {
        readDepthSensor();
        return 0;
    }
    
    if (strcmp(cmdName, "send_current_pulses") == 0) {
        printf("pulsesCounter: %u pulses (remember to update this value after each restart)\n", pulsesCounter); 
        return 0;
    }
    
    return NOT_CUSTOM_COMMAND; // Not a custom command
}

void treatSimpleCommand(const char* cmdName) {
    /*
    params:
     - cmdName: The name of the command to check and execute if it matches a simple command.
    behavior:
     - Checks if the cmdName matches any of the predefined simple commands in the simpleCommands array.
     - If it matches, it sends the corresponding byte sequence to the driver using sendPacketWithChecksum.
     - If it does not match any simple command, it returns without doing anything, allowing the caller to print an error message.
    */
    for (int i = 0; i < numSimpleCommands; i++) {
        if (strcmp(cmdName, simpleCommands[i].name) == 0) {
            uint8_t packet[3]; // max length is 3 for simple commands (ADDR + CMD + 1 optional ARG)
            packet[0] = DRIVER_ADDR;
            
            memcpy(&packet[1], simpleCommands[i].bytes, simpleCommands[i].length);
            
            sendPacketWithChecksum(packet, simpleCommands[i].length + 1);
            
            printf("\n[Pico -> Driver]: Sending '%s' command.\n", cmdName);
            return;
        }
    }
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

    for (int i = 0; localBuffer[i]; i++) localBuffer[i] = (char)tolower((char)localBuffer[i]); // Convert to lowercase

    char* cmdName = strtok(localBuffer, " "); // Get the command name (first token)
    char* args = strtok(NULL, ""); // Get the rest of the line as arguments (if any)

    int8_t cmdCustom = treatCustomCommand(cmdName, args);
    if (cmdCustom != NOT_CUSTOM_COMMAND){
        treatSimpleCommand(cmdName);
        return;
    }
    else{
        if (cmdCustom == -1) {
            printf("\n[Error]: Empty command.\n");
            return;
        }
    }
}

// --- INTERRUPT HANDLERS (LIMIT SWITCHES) ---
volatile absolute_time_t last_interrupt_time;
void gpio_callback_edge_fall(uint gpio, uint32_t events) {
    /*
    params:
     - gpio: The GPIO number that triggered the interrupt (either SW_MIN_LIMIT or SW_MAX_LIMIT).
     - events: The type of event that triggered the interrupt (should be GPIO_IRQ_EDGE_FALL in this case).
    behavior:
     - When a limit switch is triggered (falling edge), it immediately stops the driver to prevent damage.
     - It sets the sysFault flag to true and records which switch was triggered in sysError for diagnostic purposes.
     - It then re-enables the driver but since the switch is still pressed, any attempt to move further in that direction
       will be blocked until the switch is released.
     - The function uses disable_interrupts() and enable_interrupts() to ensure that the critical section of stopping the 
       driver and updating fault state is not interrupted by other interrupts, ensuring consistent behavior.
     - The processCommand("stop") is used to send a stop command to the driver immediately when a limit switch is hit.
    */
    // Imediate hardware action: Cuts Driver movement to prevent damage
    disable_interrupts();
    
    processCommand("stop\0");
    
    if ((last_interrupt_time - get_absolute_time()) < 1000000l) return; // acts as a debouncer
    last_interrupt_time = get_absolute_time();

    sysFault = true;

    // diagnostic info for fault handling and making sure the system knows which switch was hit to prevent further
    // movement in that direction until the switch is released
    if (gpio == SW_MIN_LIMIT) {
        sysError = SW_MIN_LIMIT;
    } else if (gpio == SW_MAX_LIMIT) {
        sysError = SW_MAX_LIMIT;
    }
    processCommand("enable\0"); // Re-enable driver after stopping, so it can only be moved in the opposite direction until it
                                // get out of the limit switch
    if (sysError == SW_MIN_LIMIT) {
        processCommand("move 1720 2");    // 2102 for Portugal VBS and 1720 for LaSub VBS (1mm)

    } else if (sysError == SW_MAX_LIMIT) {
        processCommand("move -1720 2");   // 2102 for Portugal VBS and 1720 for LaSub VBS (1mm)
    }
    enable_interrupts();
}

// --- TASKS ---
void vDepthSensorTask(void *pvParameters) {
    /*
    params:
     - depthSensorValue: A global variable to hold the current value read from the depth sensor.
     - pressureOffset: A global variable to hold the reference pressure for depth calibration.
    behavior:
     - Continuously reads the depth sensor value at a defined frequency (e.g. 20 Hz) and updates the global variable.
     - This allows the system to have an up-to-date depth value that can be accessed on demand by the command parser when the
       user requests it.
     - The task uses vTaskDelay to wait between readings, allowing other tasks to run and ensuring that n  it does not block the system.
     - This task takes up to 90 ms (up to 40ms for reading the sensor and 50 ms for the delay) to update the depthSensorValue.
     - The sensor is pre-initialized in prvSetupHardware() before the RTOS scheduler starts.
    */
    // Check if sensor was properly initialized at startup

    if (!depthSensor.init(i2c1)) {
        printf("[ERROR] Sensor fail\n");
    } else {
        // sets pressure offset to the initial pressure reading at startup.
        depthSensor.read(); 
        pressureOffset = depthSensor.pressure(MS5837::Pa);
        depthSensorInitialized = true;
        printf("[Depth sensor]: Successfully initialized on I2C1. Initial pressure offset set to %.2f Pa at startup. Depth is now 0 meters.\n", pressureOffset);
    }

    while (1) {
        if (depthSensorInitialized) {
            depthSensor.read();
            // Calculate depth relative to pressure offset (reference pressure)
            // When set_pressure is called, pressureOffset stores the reference pressure
            float currentPressure = depthSensor.pressure(MS5837::Pa);  // Get absolute pressure in Pa
            depthSensorValue = (currentPressure - pressureOffset) / (depthSensor.mbar * 10.0f * 9.80665f);
            
            vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms delay here + up to 40 ms for reading the sensor = up to 90 ms update time for
                                           // depthSensorValue
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
            if (rxIndex < (int)sizeof(rxBuffer) - 1) rxBuffer[rxIndex++] = byte;

        } else if (rxIndex > 0) { // If timeout occurs and we have received bytes, process the message
            rxBuffer[rxIndex] = '\0';
            printf("\n[Driver -> Pico]: ");

            for (int i = 0; i < rxIndex; i++) printf("0x%02X ", rxBuffer[i]); // Print received bytes in hex format
            
            memcpy(&driverBufferPtr, (uint8_t *)rxBuffer, rxIndex);
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
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(10, GPIO_FUNC_I2C); // SDA
    gpio_set_function(11, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(10);
    gpio_pull_up(11);

    // Registers an interrupt for the falling edge (when the switch closes).
    gpio_set_irq_enabled_with_callback(SW_MIN_LIMIT, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_edge_fall);
    // The callback will be triggered when the switch is pushed, which is the critical moment to stop the driver to prevent damage.
    // The falling edge is used because the switch is active low (connected to ground when pressed).
    gpio_set_irq_enabled(SW_MAX_LIMIT, GPIO_IRQ_EDGE_FALL, true);

    uart_init(DRIVER_UART, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Initial configuration packets to set up the driver in the desired mode (e.g. microstepping, current level, etc.)
    uint8_t modePacket[] = {DRIVER_ADDR, 0x82, 0x02}; 
    sendPacketWithChecksum(modePacket, 3);

    uint8_t enLevelPacket[] = {DRIVER_ADDR, 0x85, 0x00}; 
    sendPacketWithChecksum(enLevelPacket, 3);

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
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}

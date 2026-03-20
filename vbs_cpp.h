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

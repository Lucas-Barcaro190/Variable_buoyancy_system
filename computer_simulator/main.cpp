#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define SIM_UART uart1
#define SIM_UART_TX_PIN 4
#define SIM_UART_RX_PIN 5
#define BAUD_RATE 38400
#define VBS_ADDRESS 0xE0
#define CMD_PING_PONG 0x1F

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
                crc = (uint8_t)((crc << 1) ^ 0xA7);
            } else {
                crc <<= 1;
            }
        }
    }
    return reflect8(crc);
}

static void print_vbs_byte(uint8_t byte) {
    if (byte >= 32 && byte <= 126) {
        putchar((char)byte);
    } else {
        printf("\\x%02X", byte);
    }
    fflush(stdout);
}

static void send_ping(void) {
    uint8_t packet[5];
    packet[1] = VBS_ADDRESS;
    packet[2] = 1; // payload size
    packet[3] = CMD_PING_PONG;
    packet[4] = 0x01;
    packet[0] = calculateCRC8Bluetooth(&packet[1], 4);
    uart_write_blocking(SIM_UART, packet, sizeof(packet));
    printf("[SIM] UART1 TX -> VBS: ");
    for (size_t i = 0; i < sizeof(packet); i++) {
        printf("%02X ", packet[i]);
    }
    printf("\n");
    fflush(stdout);
}

int main(void) {
    stdio_init_all();
    uart_init(SIM_UART, BAUD_RATE);
    gpio_set_function(SIM_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SIM_UART_RX_PIN, GPIO_FUNC_UART);

    printf("[SIM] computer_simulator started. UART1 TX=%d RX=%d @ %d baud\n",
           SIM_UART_TX_PIN, SIM_UART_RX_PIN, BAUD_RATE);
    fflush(stdout);
    sleep_ms(2000);

    while (true) {
        printf("[SIM] Sending PING packet...\n");
        fflush(stdout);
        send_ping();

        uint8_t rx_buf[16] = {0};
        size_t rx_len = 0;
        uint32_t start_ms = to_ms_since_boot(get_absolute_time());
        bool got_response = false;

        while (to_ms_since_boot(get_absolute_time()) - start_ms < 3000) {
            while (uart_is_readable(SIM_UART) && rx_len < sizeof(rx_buf)) {
                uint8_t byte = uart_getc(SIM_UART);
                rx_buf[rx_len++] = byte;
                printf("[SIM] UART1 RX <- VBS: %02X\n", byte);
                fflush(stdout);
            }

            while (rx_len >= 4) {
                uint8_t payload_size = rx_buf[2];
                size_t expected_len = 4 + payload_size;
                if (payload_size <= 10 && rx_len >= expected_len) {
                    uint8_t expected_crc = calculateCRC8Bluetooth(&rx_buf[1], 3 + payload_size);
                    if (rx_buf[0] == expected_crc && rx_buf[1] == VBS_ADDRESS && rx_buf[3] == CMD_PING_PONG) {
                        printf("[SIM] Received PONG packet. payload(size=%u):", payload_size);
                        for (size_t i = 0; i < payload_size; i++) {
                            printf(" %02X", rx_buf[4 + i]);
                        }
                        printf("\n");
                        got_response = true;
                        memmove(rx_buf, rx_buf + expected_len, rx_len - expected_len);
                        rx_len -= expected_len;
                        break;
                    }
                }
                break;
            }

            if (got_response) {
                break;
            }
            sleep_ms(100);
        }

        if (!got_response) {
            printf("[SIM] No valid PONG response received within timeout.\n");
        }
        fflush(stdout);
        sleep_ms(2000);
    }

    return 0;
}

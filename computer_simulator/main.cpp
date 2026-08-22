#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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
#define CMD_MOVE_POT 0x25
#define CMD_ACK_CMD 0x10
#define CMD_ACK_MOVE 0x12
#define CMD_TIMEOUT_EMERGENCY 0x14
#define CMD_CHANGE_ID 0x1A
#define CMD_FULL_CONTRACT 0x20
#define CMD_FULL_EXPAND 0x2F
#define CMD_ABS_VOLUME 0x21
#define CMD_REL_VOLUME 0x23
#define CMD_REQ_VOLUME 0x24
#define CMD_REQ_POT 0x26
#define CMD_ABS_PISTON 0x29
#define CMD_REL_PISTON 0x2B
#define CMD_REQ_PISTON 0x2C
#define CMD_ERR_OUT_OF_BOUNDS 0x1D
#define MSG(type, size) (((uint16_t)(size) << 8) | (type))

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

static void send_packet(uint16_t msgTypeCode, const uint8_t* payload) {
    uint8_t packet[8];
    uint8_t size = (uint8_t)(msgTypeCode >> 8);
    packet[1] = VBS_ADDRESS;
    packet[2] = (uint8_t)(msgTypeCode >> 8);
    packet[3] = (uint8_t)(msgTypeCode & 0xFF);
    if (size > 0 && payload != NULL) {
        memcpy(&packet[4], payload, size);
    }
    packet[0] = calculateCRC8Bluetooth(&packet[1], 3 + size);
    uart_write_blocking(SIM_UART, packet, 4 + size);
}

static void send_uint16_command(uint16_t msgTypeCode, uint16_t value) {
    uint8_t payload[2];
    payload[0] = (uint8_t)(value >> 8);
    payload[1] = (uint8_t)(value & 0xFF);
    send_packet(msgTypeCode, payload);
}

static void send_float_command(uint16_t msgTypeCode, float value) {
    union { float f; uint32_t u; } conv;
    conv.f = value;
    uint8_t payload[4];
    payload[0] = (uint8_t)(conv.u >> 24);
    payload[1] = (uint8_t)(conv.u >> 16);
    payload[2] = (uint8_t)(conv.u >> 8);
    payload[3] = (uint8_t)(conv.u & 0xFF);
    send_packet(msgTypeCode, payload);
}

static void send_ping(void) {
    uint8_t payload[1] = { VBS_ADDRESS };
    send_packet(MSG(CMD_PING_PONG, 1), payload);
    printf("[SIM] UART1 TX -> VBS msgType=0x%04X payload=%02X\n", MSG(CMD_PING_PONG, 1), VBS_ADDRESS);
    fflush(stdout);
}

static void send_move_pot(uint16_t target_pot) {
    send_uint16_command(MSG(CMD_MOVE_POT, 2), target_pot);
    printf("[SIM] UART1 TX -> VBS MOVE_POT target=%u\n", (unsigned)target_pot);
    fflush(stdout);
}

static void send_full_contract(void) {
    uint8_t payload[1] = { 0x00 };
    send_packet(MSG(CMD_FULL_CONTRACT, 1), payload);
    printf("[SIM] UART1 TX -> VBS FULL_CONTRACT\n");
    fflush(stdout);
}

static void send_full_expand(void) {
    uint8_t payload[1] = { 0xFF };
    send_packet(MSG(CMD_FULL_EXPAND, 1), payload);
    printf("[SIM] UART1 TX -> VBS FULL_EXPAND\n");
    fflush(stdout);
}

static void send_abs_volume(float volume) {
    send_float_command(MSG(CMD_ABS_VOLUME, 4), volume);
    printf("[SIM] UART1 TX -> VBS ABS_VOLUME %.3f\n", volume);
    fflush(stdout);
}

static void send_rel_volume(float volume) {
    send_float_command(MSG(CMD_REL_VOLUME, 4), volume);
    printf("[SIM] UART1 TX -> VBS REL_VOLUME %.3f\n", volume);
    fflush(stdout);
}

static void send_req_volume(void) {
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    send_packet(MSG(CMD_REQ_VOLUME, 4), payload);
    printf("[SIM] UART1 TX -> VBS REQ_VOLUME\n");
    fflush(stdout);
}

static void send_abs_piston(float position) {
    send_float_command(MSG(CMD_ABS_PISTON, 4), position);
    printf("[SIM] UART1 TX -> VBS ABS_PISTON %.3f\n", position);
    fflush(stdout);
}

static void send_rel_piston(float position) {
    send_float_command(MSG(CMD_REL_PISTON, 4), position);
    printf("[SIM] UART1 TX -> VBS REL_PISTON %.3f\n", position);
    fflush(stdout);
}

static void send_req_piston(void) {
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    send_packet(MSG(CMD_REQ_PISTON, 4), payload);
    printf("[SIM] UART1 TX -> VBS REQ_PISTON\n");
    fflush(stdout);
}

static void send_change_id(uint8_t new_id) {
    uint8_t payload[1] = { new_id };
    send_packet(MSG(CMD_CHANGE_ID, 1), payload);
    printf("[SIM] UART1 TX -> VBS CHANGE_ID %02X\n", new_id);
    fflush(stdout);
}

static void send_timeout_emergency(uint32_t seconds) {
    uint8_t payload[4];
    payload[0] = (uint8_t)(seconds >> 24);
    payload[1] = (uint8_t)(seconds >> 16);
    payload[2] = (uint8_t)(seconds >> 8);
    payload[3] = (uint8_t)(seconds & 0xFF);
    send_packet(MSG(CMD_TIMEOUT_EMERGENCY, 4), payload);
    printf("[SIM] UART1 TX -> VBS TIMEOUT_EMERGENCY %u\n", seconds);
    fflush(stdout);
}

int main(void) {
    stdio_init_all();
    uart_init(SIM_UART, BAUD_RATE);
    gpio_set_function(SIM_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SIM_UART_RX_PIN, GPIO_FUNC_UART);

    printf("[SIM] computer_simulator started. UART1 TX=%d RX=%d @ %d baud\n",
           SIM_UART_TX_PIN, SIM_UART_RX_PIN, BAUD_RATE);
    printf("[SIM] USB command interface ready. Type 'move_pot <value>' on PC terminal.\n");
    fflush(stdout);
    sleep_ms(2000);

    char line_buffer[64];
    bool waiting_pong = false;
    uint32_t send_ms = 0;

    while (true) {
        printf("> ");
        fflush(stdout);
        if (fgets(line_buffer, sizeof(line_buffer), stdin) != NULL) {
            char *cmd = strtok(line_buffer, " \t\r\n");
            if (cmd != NULL) {
                if (strcmp(cmd, "move_pot") == 0) {
                    char *arg = strtok(NULL, " \t\r\n");
                    if (arg != NULL) {
                        int value = atoi(arg);
                        if (value < 0) value = 0;
                        if (value > 511) value = 511;
                        printf("[SIM] USB CMD: move_pot %d\n", value);
                        fflush(stdout);
                        send_move_pot((uint16_t)value);
                    } else {
                        printf("[SIM] Usage: move_pot <0-511>\n");
                        fflush(stdout);
                    }
                } else if (strcmp(cmd, "full_contract") == 0) {
                    printf("[SIM] USB CMD: full_contract\n");
                    fflush(stdout);
                    send_full_contract();
                } else if (strcmp(cmd, "full_expand") == 0) {
                    printf("[SIM] USB CMD: full_expand\n");
                    fflush(stdout);
                    send_full_expand();
                } else if (strcmp(cmd, "abs_volume") == 0) {
                    char *arg = strtok(NULL, " \t\r\n");
                    if (arg != NULL) {
                        float value = strtof(arg, NULL);
                        printf("[SIM] USB CMD: abs_volume %.3f\n", value);
                        fflush(stdout);
                        send_abs_volume(value);
                    } else {
                        printf("[SIM] Usage: abs_volume <float>\n");
                        fflush(stdout);
                    }
                } else if (strcmp(cmd, "rel_volume") == 0) {
                    char *arg = strtok(NULL, " \t\r\n");
                    if (arg != NULL) {
                        float value = strtof(arg, NULL);
                        printf("[SIM] USB CMD: rel_volume %.3f\n", value);
                        fflush(stdout);
                        send_rel_volume(value);
                    } else {
                        printf("[SIM] Usage: rel_volume <float>\n");
                        fflush(stdout);
                    }
                } else if (strcmp(cmd, "req_volume") == 0) {
                    printf("[SIM] USB CMD: req_volume\n");
                    fflush(stdout);
                    send_req_volume();
                } else if (strcmp(cmd, "abs_piston") == 0) {
                    char *arg = strtok(NULL, " \t\r\n");
                    if (arg != NULL) {
                        float value = strtof(arg, NULL);
                        printf("[SIM] USB CMD: abs_piston %.3f\n", value);
                        fflush(stdout);
                        send_abs_piston(value);
                    } else {
                        printf("[SIM] Usage: abs_piston <float>\n");
                        fflush(stdout);
                    }
                } else if (strcmp(cmd, "rel_piston") == 0) {
                    char *arg = strtok(NULL, " \t\r\n");
                    if (arg != NULL) {
                        float value = strtof(arg, NULL);
                        printf("[SIM] USB CMD: rel_piston %.3f\n", value);
                        fflush(stdout);
                        send_rel_piston(value);
                    } else {
                        printf("[SIM] Usage: rel_piston <float>\n");
                        fflush(stdout);
                    }
                } else if (strcmp(cmd, "req_piston") == 0) {
                    printf("[SIM] USB CMD: req_piston\n");
                    fflush(stdout);
                    send_req_piston();
                } else if (strcmp(cmd, "change_id") == 0) {
                    char *arg = strtok(NULL, " \t\r\n");
                    if (arg != NULL) {
                        int value = atoi(arg);
                        if (value < 0) value = 0;
                        if (value > 255) value = 255;
                        printf("[SIM] USB CMD: change_id %d\n", value);
                        fflush(stdout);
                        send_change_id((uint8_t)value);
                    } else {
                        printf("[SIM] Usage: change_id <0-255>\n");
                        fflush(stdout);
                    }
                } else if (strcmp(cmd, "timeout_emergency") == 0) {
                    char *arg = strtok(NULL, " \t\r\n");
                    if (arg != NULL) {
                        uint32_t value = (uint32_t)strtoul(arg, NULL, 10);
                        printf("[SIM] USB CMD: timeout_emergency %u\n", value);
                        fflush(stdout);
                        send_timeout_emergency(value);
                    } else {
                        printf("[SIM] Usage: timeout_emergency <seconds>\n");
                        fflush(stdout);
                    }
                } else if (strcmp(cmd, "ping") == 0) {
                    printf("[SIM] USB CMD: ping\n");
                    fflush(stdout);
                    send_ping();
                    waiting_pong = true;
                    send_ms = to_ms_since_boot(get_absolute_time());
                } else {
                    printf("[SIM] Unknown command: %s\n", cmd);
                    fflush(stdout);
                }
            }
        }

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
                    if (rx_buf[0] == expected_crc && rx_buf[1] == VBS_ADDRESS) {
                                        uint16_t msg_type_code = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
                        uint8_t payload_size = (uint8_t)(msg_type_code >> 8);
                        uint8_t msg_type = (uint8_t)(msg_type_code & 0xFF);
                        if (msg_type == CMD_ACK_CMD) {
                            const char *status = (rx_buf[4] == 0x00) ? "ACK" : "NACK";
                            printf("[SIM] Received CMD_ACK_CMD %s\n", status);
                        } else if (msg_type == CMD_ACK_MOVE) {
                            const char *status = (rx_buf[4] == 0x00) ? "ACK" : "NACK";
                            printf("[SIM] Received CMD_ACK_MOVE %s\n", status);
                        } else if (msg_type == CMD_ERR_OUT_OF_BOUNDS) {
                            printf("[SIM] Received ERR_OUT_OF_BOUNDS caused by %02X\n", rx_buf[4]);
                        } else if (msg_type == CMD_PING_PONG) {
                            uint32_t elapsed_ms = to_ms_since_boot(get_absolute_time()) - send_ms;
                            printf("[SIM] Received PONG packet. payload(size=%u):", payload_size);
                            for (size_t i = 0; i < payload_size; i++) {
                                printf(" %02X", rx_buf[4 + i]);
                            }
                            printf("\n");
                            printf("[SIM] Ping-pong RTT: %u ms\n", (unsigned)elapsed_ms);
                            fflush(stdout);
                            waiting_pong = false;
                        } else if (rx_buf[3] == CMD_REQ_VOLUME || rx_buf[3] == CMD_REQ_PISTON) {
                            if (payload_size == 4) {
                                uint32_t val = ((uint32_t)rx_buf[4] << 24) | ((uint32_t)rx_buf[5] << 16) | ((uint32_t)rx_buf[6] << 8) | rx_buf[7];
                                float f;
                                memcpy(&f, &val, sizeof(float));
                                printf("[SIM] Received %s response: %.3f\n", rx_buf[3] == CMD_REQ_VOLUME ? "REQ_VOLUME" : "REQ_PISTON", f);
                            } else {
                                printf("[SIM] Received %02X payload(size=%u):", rx_buf[3], payload_size);
                                for (size_t i = 0; i < payload_size; i++) {
                                    printf(" %02X", rx_buf[4 + i]);
                                }
                                printf("\n");
                            }
                        } else {
                            printf("[SIM] Received msg type %02X payload(size=%u):", rx_buf[3], payload_size);
                            for (size_t i = 0; i < payload_size; i++) {
                                printf(" %02X", rx_buf[4 + i]);
                            }
                            printf("\n");
                        }
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

        if (waiting_pong && !got_response) {
            printf("[SIM] Ping-pong timed out after 3000 ms\n");
            fflush(stdout);
            waiting_pong = false;
        } else if (!got_response && !waiting_pong) {
            printf("[SIM] No valid response received within timeout.\n");
            fflush(stdout);
        }
    }

    return 0;
}

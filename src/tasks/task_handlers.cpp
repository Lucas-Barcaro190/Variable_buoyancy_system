#include "src/tasks/task_handlers.h"

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

#include "src/motor/motor_control.h"
#include "src/comms/protocol.h"
#include "src/core/shared_state.h"

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
    {"help", handle_cmd_help},
    {"change_pid", handle_cmd_change_pid},
    {"verbose", handle_cmd_verbose},
    {"diag", handle_cmd_diag},
    {"pot", handle_cmd_pot},
    {"stop", handle_cmd_stop},
    {"move", handle_cmd_move},
    {"move_pot", handle_cmd_move_pot},
};

#define NUM_COMMANDS (sizeof(cmd_table) / sizeof(cmd_table[0]))

void vMotorControlTask(void *pvParameters) {
    (void)pvParameters;
    adc_init();
    adc_gpio_init(POT_ADC_PIN);
    adc_select_input(POT_ADC_CHANNEL);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200);

    MotorControlState_t mctl_state = MCTL_IDLE;
    MotorCmd_t current_cmd = {0};

    if (vbs_should_log(1)) printf("[Motor] 5Hz Task started on Core 0\n");

    for (;;) {
        evaluate_pending_limit_switches();

        if (flag_min_limit_hit) {
            printf("[Motor] **CRITICAL** Min limit switch hit! Recovering with 8188 steps in dir 1...\n");
            sys_state = SYS_CRITICAL_ERROR;
            sys_fault_code = FAULT_MIN_LIMIT_HIT;
            diag.fault_count[FAULT_MIN_LIMIT_HIT]++;
            stop_stepper_pio();
            xQueueReset(xMotorCmdQueue);
            mctl_state = MCTL_IDLE;
            gpio_put(PIN_MOTOR_DIR, 1);
            send_stepper_pulses(8188);
            wait_stepper_done();
            flag_min_limit_hit = false;
            flag_max_limit_hit = false;
            sys_fault_code = FAULT_NONE;
            sys_state = SYS_OPERATIONAL;
            printf("[Motor] Min limit recovery complete.\n");
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        if (flag_max_limit_hit) {
            printf("[Motor] **CRITICAL** Max limit switch hit! Recovering with 8188 steps in dir 0...\n");
            sys_state = SYS_CRITICAL_ERROR;
            sys_fault_code = FAULT_MAX_LIMIT_HIT;
            diag.fault_count[FAULT_MAX_LIMIT_HIT]++;
            stop_stepper_pio();
            xQueueReset(xMotorCmdQueue);
            mctl_state = MCTL_IDLE;
            gpio_put(PIN_MOTOR_DIR, 0);
            send_stepper_pulses(8188);
            wait_stepper_done();
            flag_min_limit_hit = false;
            flag_max_limit_hit = false;
            pending_min_limit_event = false;
            pending_max_limit_event = false;
            sys_fault_code = FAULT_NONE;
            sys_state = SYS_OPERATIONAL;
            printf("[Motor] Max limit recovery complete.\n");
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        if (xQueueReceive(xMotorCmdQueue, &current_cmd, 0) == pdTRUE) {
            mctl_state = (MotorControlState_t)current_cmd.cmd_type;
            diag.motor_cmd_queued++;
            set_target_pot_value(current_cmd.target_pot);
            if (vbs_should_log(1)) {
                printf("[Motor] New command: type=%d, target_pot=%d, pulses=%lu, direction=%d\n",
                       mctl_state, current_cmd.target_pot, (unsigned long)current_cmd.pulses, current_cmd.direction);
            }
        }

        uint32_t sample_sum = 0;
        for (int i = 0; i < POT_SAMPLE_COUNT; i++) {
            sample_sum += adc_read();
        }
        uint16_t current_val = (uint16_t)(sample_sum / POT_SAMPLE_COUNT);
        current_val = current_val >> 3;
        setPotValue(current_val);

        currentPistonPosition = (int16_t)potToPistonPos(current_val);
        currentVolume = pistonPosToVolume(potToPistonPos(current_val));

        printf("[Motor] mctl_state: %d, current pot: %d, piston pos: %.2f mm, volume: %.2f cm^3\n",
               mctl_state, current_val, currentPistonPosition / 100.0f, currentVolume);

        if (mctl_state == MCTL_MOVING_UNTIL_POT || mctl_state == MCTL_MOVING_ABSOLUTE) {
            uint16_t target = get_target_pot_value();
            int16_t error = (int16_t)target - (int16_t)current_val;
            if (abs(error) <= 3) {
                printf("[Motor] Target pot %d reached (current: %d)\n", target, current_val);
                stop_stepper_pio();
                mctl_state = MCTL_IDLE;
                diag.motor_move_complete++;
            } else {
                uint8_t direction = error > 0 ? 1 : 0;
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
                if (!flag_min_limit_hit && !flag_max_limit_hit) {
                    gpio_put(PIN_MOTOR_DIR, !current_cmd.direction);
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
            stop_stepper_pio();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

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
        int ch = getchar_timeout_us(10000);

        if (ch == PICO_ERROR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (rx_len < sizeof(rx_buf)) {
            rx_buf[rx_len++] = (uint8_t)ch;
        } else {
            memmove(rx_buf, rx_buf + 1, sizeof(rx_buf) - 1);
            rx_buf[sizeof(rx_buf) - 1] = (uint8_t)ch;
        }

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
                            memmove(rx_buf, rx_buf + expected_packet_len, rx_len - expected_packet_len);
                            rx_len -= expected_packet_len;
                            binary_packet_parsed = true;
                            continue;
                        }
                    } else {
                        break;
                    }
                }
            }
            memmove(rx_buf, rx_buf + 1, rx_len - 1);
            rx_len--;
        }

        if (binary_packet_parsed) {
            continue;
        }

        if (ch == '\n' || ch == '\r') {
            if (line_idx > 0) {
                line_buffer[line_idx] = '\0';

                last_pc_heartbeat_ms = xTaskGetTickCount();
                diag.pc_heartbeats++;

                if (vbs_should_log(3) || vbs_should_log(6)) printf("\n[Parser] Received: '%s'\n", line_buffer);

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
            if (line_idx < (int)sizeof(line_buffer) - 1) {
                line_buffer[line_idx] = (char)ch;
                line_idx++;
                putchar(ch);
                fflush(stdout);
            }
        }
        else if (ch == '\b' || ch == 127) {
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

void vFaultManagerTask(void *pvParameters) {
    (void)pvParameters;
    if (vbs_should_log(5)) printf("[FaultMgr] Limit switch interrupts configured on Core 1\n");
    if (vbs_should_log(5)) printf("[FaultMgr] Task started on Core 1\n");
    watchdog_enable(8000, true);

    while (1) {
        uint32_t now_ms = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(100));
        uint32_t time_since_heartbeat = now_ms - last_pc_heartbeat_ms;
        (void)time_since_heartbeat;
        watchdog_update();
    }
}

void vDiagnosticsTask(void *pvParameters) {
    (void)pvParameters;
    if (vbs_should_log(5)) printf("[Diagnostics] Task started on Core 1\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (verbose_level != 0) {
            printf("\n========== DIAGNOSTICS ==========");
            printf("\nState: %s\n", stateToString(sys_state));
            printf("Fault: %s\n", faultToString(sys_fault_code));
            printf("Potentiometer: %d [%d - %d]\n", getPotValue(), MINIMAL_THRESHOLD, MAXIMUM_THRESHOLD);
            printf("Time since PC heartbeat: %lu ms\n", xTaskGetTickCount() - last_pc_heartbeat_ms);
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

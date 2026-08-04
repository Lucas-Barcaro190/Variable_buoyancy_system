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
#include "src/motor/motor_control.h"
#include "src/motor/potentiometer_reading.h"
#include "src/motor/velocity_generator.h"
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
    potentiometer_init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Loop de 20 Hz (50 ms)
    const float dt = 0.050f;

    MotorControlState_t mctl_state = MCTL_IDLE;
    MotorCmd_t current_cmd = {0};

    VelocityGenerator_t vel_gen;
    velocity_generator_init(&vel_gen);

    PIDController_t pid;
    pid_init(&pid, 1.5f, 0.05f, 0.01f, -DEFAULT_VMAX_MM_S, DEFAULT_VMAX_MM_S);

    if (vbs_should_log(1)) printf("[Motor] 20Hz Motor Control & PID Loop started on Core 0\n");

    for (;;) {
        evaluate_pending_limit_switches();

        if (flag_min_limit_hit) {
            printf("[Motor] **CRITICAL** Min limit switch hit! Recovering by moving 8189 steps (+1 direction)...\n");
            sys_state = SYS_CRITICAL_ERROR;
            sys_fault_code = FAULT_MIN_LIMIT_HIT;
            diag.fault_count[FAULT_MIN_LIMIT_HIT]++;
            stop_stepper_pio();
            xQueueReset(xMotorCmdQueue);
            mctl_state = MCTL_IDLE;
            set_stepper_speed_mm_s(DEFAULT_VMAX_MM_S);
            uint32_t move_ticks = pdMS_TO_TICKS((uint32_t)((8189.0f / STEPS_PER_MM) / DEFAULT_VMAX_MM_S * 1000.0f));
            vTaskDelay(move_ticks);
            stop_stepper_pio();
            flag_min_limit_hit = false;
            flag_max_limit_hit = false;
            sys_fault_code = FAULT_NONE;
            sys_state = SYS_OPERATIONAL;
            printf("[Motor] Min limit recovery complete (moved 8189 steps).\n");
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        if (flag_max_limit_hit) {
            printf("[Motor] **CRITICAL** Max limit switch hit! Recovering by moving 8189 steps (-1 direction)...\n");
            sys_state = SYS_CRITICAL_ERROR;
            sys_fault_code = FAULT_MAX_LIMIT_HIT;
            diag.fault_count[FAULT_MAX_LIMIT_HIT]++;
            stop_stepper_pio();
            xQueueReset(xMotorCmdQueue);
            mctl_state = MCTL_IDLE;
            set_stepper_speed_mm_s(-DEFAULT_VMAX_MM_S);
            uint32_t move_ticks = pdMS_TO_TICKS((uint32_t)((8189.0f / STEPS_PER_MM) / DEFAULT_VMAX_MM_S * 1000.0f));
            vTaskDelay(move_ticks);
            stop_stepper_pio();
            flag_min_limit_hit = false;
            flag_max_limit_hit = false;
            pending_min_limit_event = false;
            pending_max_limit_event = false;
            sys_fault_code = FAULT_NONE;
            sys_state = SYS_OPERATIONAL;
            printf("[Motor] Max limit recovery complete (moved 8189 steps).\n");
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        // Leitura atual do potenciômetro e atualização do estado do sistema
        uint16_t current_val = read_potentiometer_raw();
        setPotValue(current_val);

        float h_medido = potToPistonPos(current_val);
        currentPistonPosition = (int16_t)(h_medido * 100.0f);
        currentVolume = pistonPosToVolume(h_medido);

        // Processamento de novos comandos da fila
        if (xQueueReceive(xMotorCmdQueue, &current_cmd, 0) == pdTRUE) {
            mctl_state = (MotorControlState_t)current_cmd.cmd_type;
            diag.motor_cmd_queued++;
            set_target_pot_value(current_cmd.target_pot);

            float h_target = potToPistonPos(current_cmd.target_pot);
            float now_sec = (float)xTaskGetTickCount() * (1.0f / configTICK_RATE_HZ);

            velocity_generator_start(&vel_gen, h_medido, h_target, now_sec, DEFAULT_VMAX_MM_S, 3.0f);
            pid_reset(&pid);

            if (vbs_should_log(1)) {
                printf("[Motor] New command: type=%d, target_pot=%d (%.2f mm)\n",
                       mctl_state, current_cmd.target_pot, h_target);
            }
        }

        // Execução da malha de controle PID e gerador de trajetória
        if (mctl_state == MCTL_MOVING_UNTIL_POT || mctl_state == MCTL_MOVING_ABSOLUTE) {
            float now_sec = (float)xTaskGetTickCount() * (1.0f / configTICK_RATE_HZ);
            TrajectoryPoint_t traj = velocity_generator_update(&vel_gen, now_sec);

            float pos_error = traj.href - h_medido;

            if (traj.is_completed && fabsf(pos_error) < 0.2f) {
                stop_stepper_pio();
                mctl_state = MCTL_IDLE;
                diag.motor_move_complete++;
                printf("[Motor] Target reached! h_medido=%.2f mm, error=%.2f mm\n", h_medido, pos_error);
            } else {
                float v_control = pid_compute(&pid, traj.href, h_medido, dt);
                set_stepper_speed_mm_s(v_control);
            }
        } else if (mctl_state == MCTL_MOVING_PULSES) {
            if (current_cmd.pulses == 0) {
                stop_stepper_pio();
                mctl_state = MCTL_IDLE;
                diag.motor_move_complete++;
            } else {
                float speed = (current_cmd.direction == 0) ? DEFAULT_VMAX_MM_S : -DEFAULT_VMAX_MM_S;
                set_stepper_speed_mm_s(speed);

                float steps_this_tick = fabsf(speed) * STEPS_PER_MM * dt;
                uint32_t consumed_steps = (uint32_t)steps_this_tick;
                if (consumed_steps == 0) {
                    consumed_steps = 1;
                }

                if (current_cmd.pulses <= consumed_steps) {
                    current_cmd.pulses = 0;
                    stop_stepper_pio();
                    mctl_state = MCTL_IDLE;
                    diag.motor_move_complete++;
                    printf("[Motor] Move complete after %lu steps\n", (unsigned long)consumed_steps);
                } else {
                    current_cmd.pulses -= consumed_steps;
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
    printf("\nVBS v3.0 Ready. Type 'help' for commands.\n");
    printf("> ");
    fflush(stdout);

    while (1) {
        int ch = PICO_ERROR_TIMEOUT;
        if (uart_is_readable(uart0)) {
            ch = uart_getc(uart0);
        } else {
            ch = getchar_timeout_us(100);
        }

        if (ch == PICO_ERROR_TIMEOUT || ch < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // ASCII line processing (commands like 'help', 'diag', 'move N', 'move_pot P')
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
                rx_len = 0;
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
        else {
            // Non-ASCII binary protocol packet processing
            if (rx_len < sizeof(rx_buf)) {
                rx_buf[rx_len++] = (uint8_t)ch;
            } else {
                memmove(rx_buf, rx_buf + 1, sizeof(rx_buf) - 1);
                rx_buf[sizeof(rx_buf) - 1] = (uint8_t)ch;
            }

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
        }
    }
}

void vFaultManagerTask(void *pvParameters) {
    (void)pvParameters;
    if (vbs_should_log(5)) printf("[FaultMgr] Limit switch interrupts configured on Core 1\n");
    if (vbs_should_log(5)) printf("[FaultMgr] Task started on Core 1\n");
    // watchdog_enable(8000, true);

    while (1) {
        uint32_t now_ms = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(100));
        uint32_t time_since_heartbeat = now_ms - last_pc_heartbeat_ms;
        (void)time_since_heartbeat;
        // watchdog_update();
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

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

/*
Desc: Print the list of available serial commands.
params:
    - none
returns:
    - [void]
*/
static void handle_cmd_help(void) {
    printf("Available commands:\n");
    printf("  help         - Show this help\n");
    printf("  move N       - Move N steps (signed)\n");
    printf("                 e.g. move 4094  or move -4094\n");
    printf("  move_constant N - Move N steps at constant speed\n");
    printf("                 e.g. move_constant 4094  or move_constant -4094\n");
    printf("  move_pot P   - Move to potentiometer value P (0-511)\n");
    printf("                 e.g. move_pot 48  or move_pot 430\n");
    printf("  diag         - Print diagnostics now\n");
    printf("  pot          - Print current potentiometer value\n");
    printf("  stop         - Send driver stop command\n");
}

/*
Desc: Inform the user that PID is not supported by this driver.
params:
    - none
returns:
    - [void]
*/
static void handle_cmd_change_pid(void) {
    printf("[Parser] PID control is not supported on the pulse/dir driver.\n");
}

/*
Desc: Set or report the current verbosity logging level.
params:
    - none
returns:
    - [void]
*/
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

/*
Desc: Print on-demand system diagnostics to the console.
params:
    - none
returns:
    - [void]
*/
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

/*
Desc: Print the latest shared potentiometer reading.
params:
    - none
returns:
    - [void]
*/
static void handle_cmd_pot(void) {
    printf("Potentiometer (raw ADC avg): %d\n", getPotValue());
}

/*
Desc: Queue an immediate stop command for the motor driver.
params:
    - none
returns:
    - [void]
*/
static void handle_cmd_stop(void) {
    MotorCmd_t cmd = {0};
    cmd.cmd_type = MCTL_IDLE;
    xQueueSend(xMotorCmdQueue, &cmd, portMAX_DELAY);
    printf("Queued stop command\n");
}

/*
Desc: Parse and queue a pulse-count movement command from ASCII input.
params:
    - none
returns:
    - [void]
*/
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

/*
Desc: Parse and queue a constant-speed pulse movement command from ASCII input.
params:
    - none
returns:
    - [void]
*/
static void handle_cmd_move_constant(void) {
    char *arg1 = strtok(NULL, " \t");
    if (arg1 == NULL) {
        printf("Usage: move_constant <steps>\n");
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
        printf("Queued constant-speed move command: pulses=%lu, direction=%u\n", (unsigned long)pulses, (unsigned)direction);
    }
}

/*
Desc: Parse and queue a potentiometer target movement command from ASCII input.
params:
    - none
returns:
    - [void]
*/
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
    {"move_constant", handle_cmd_move_constant},
    {"move_pot", handle_cmd_move_pot},
};

#define NUM_COMMANDS (sizeof(cmd_table) / sizeof(cmd_table[0]))

// Process a completed ASCII line (commands like 'help', 'diag', 'move N', 'move_pot P')
/*
Desc: Process a completed ASCII command line and dispatch the matching handler.
params:
    - [char*] line_buffer: Buffer containing the input line.
    - [int*] line_idx: Current index in the line buffer.
    - [size_t*] rx_len: Length of the binary receive buffer (cleared after processing).
returns:
    - [void]
*/
static void process_ascii_line(char *line_buffer, int *line_idx, size_t *rx_len) {
    if (*line_idx > 0) {
        line_buffer[*line_idx] = '\0';

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

        *line_idx = 0;
        *rx_len = 0;
    }
    printf("> ");
    fflush(stdout);
}

// Process binary/non-ASCII stream in rx_buf.
/*
Desc: Parse and process binary protocol packets from the receive buffer.
params:
    - [uint8_t*] rx_buf: Buffer containing received bytes.
    - [size_t*] rx_len: Number of bytes currently in the buffer.
returns:
    - [void]
*/
static void process_binary_stream(uint8_t *rx_buf, size_t *rx_len) {
    while (*rx_len >= 4) {
        uint8_t expected_addr = getAddress();
        if (rx_buf[1] == expected_addr) {
            uint8_t payload_size = rx_buf[2];
            if (payload_size <= 4) {
                size_t expected_packet_len = 4 + payload_size;
                if (*rx_len >= expected_packet_len) {
                    uint8_t calculated_crc = calculateCRC8Bluetooth(&rx_buf[1], expected_packet_len - 1);
                    uint8_t received_crc = rx_buf[0];

                    if (calculated_crc == received_crc) {
                        handle_binary_command(rx_buf[3], &rx_buf[4], payload_size);
                        memmove(rx_buf, rx_buf + expected_packet_len, *rx_len - expected_packet_len);
                        *rx_len -= expected_packet_len;
                        continue;
                    }
                } else {
                    break;
                }
            }
        }
        memmove(rx_buf, rx_buf + 1, *rx_len - 1);
        (*rx_len)--;
    }
}

/*
Desc: Main motor control task. Reads the potentiometer, updates state, and executes motion commands.
params:
    - [void*] pvParameters: Task parameters (unused).
returns:
    - [void]
*/
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
        if (flag_min_limit_hit || flag_max_limit_hit) {
            treat_fault_limit(&mctl_state);
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        // Leitura atual do potenciômetro com filtro de mediana e atualização do estado do sistema
        uint16_t current_val = read_potentiometer_median();
        printf("%u\n", current_val);
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
            float requested_vmax = (current_cmd.speed > 0) ? MOTOR_RPM_TO_MM_S(current_cmd.speed) : DEFAULT_VMAX_MM_S;

            //printf("[Motor DBG] cmd received: type=%u target_pot=%u current_pot=%u h_medido=%.3f h_target=%.3f\n",
            //       (unsigned)mctl_state, (unsigned)current_cmd.target_pot, (unsigned)current_val, h_medido, h_target);

            velocity_generator_start(&vel_gen, h_medido, h_target, now_sec, requested_vmax, 3.0f);
            pid_reset(&pid);

            if (vbs_should_log(1)) {
                printf("[Motor] New command: type=%d, target_pot=%d (%.2f mm)\n",
                       mctl_state, current_cmd.target_pot, h_target);
            }
        }

        // Execução da malha de controle PID e gerador de trajetória
        if (mctl_state == MCTL_MOVING_UNTIL_POT || mctl_state == MCTL_MOVING_ABSOLUTE) {
            potentiometer_movement(&vel_gen, &pid, dt, h_medido, &mctl_state);
        } else if (mctl_state == MCTL_MOVING_PULSES) {
            pulses_movement(&current_cmd, dt, &mctl_state);
        } else {
            stop_stepper_pio();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*
Desc: Parser task for serial ASCII and binary command input.
params:
    - [void*] pvParameters: Task parameters (unused).
returns:
    - [void]
*/
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
            process_ascii_line(line_buffer, &line_idx, &rx_len);
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
            process_binary_stream(rx_buf, &rx_len);
        }
    }
}

/*
Desc: Fault manager task that monitors heartbeat and limit switch events.
params:
    - [void*] pvParameters: Task parameters (unused).
returns:
    - [void]
*/
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

/*
Desc: Periodic diagnostics task that prints system health and statistics.
params:
    - [void*] pvParameters: Task parameters (unused).
returns:
    - [void]
*/
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

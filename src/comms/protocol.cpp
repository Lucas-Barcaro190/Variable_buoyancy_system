#include "src/comms/protocol.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "src/motor/motor_control.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

static uint8_t reflect8(uint8_t val) {
    uint8_t res = 0;
    for (int i = 0; i < 8; i++) {
        if ((val >> i) & 1) {
            res |= (1 << (7 - i));
        }
    }
    return res;
}

uint8_t calculateCRC8Bluetooth(const uint8_t* data, size_t length) {
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

uint8_t getMsgSize(uint16_t msg_type_code) {
    return (uint8_t)((msg_type_code >> 8) & 0xFF);
}

uint8_t getMsgType(uint16_t msg_type_code) {
    return (uint8_t)(msg_type_code & 0xFF);
}

uint16_t makeMsgTypeCode(uint8_t size, uint8_t type) {
    return (uint16_t)((size << 8) | type);
}

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

void handle_binary_command(uint8_t msgType, const uint8_t* data, uint8_t size) {
    switch (msgType) {
        case CMD_TIMEOUT_EMERGENCY: {
            if (size >= 4) {
                uint32_t seconds = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
                (void)seconds;
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

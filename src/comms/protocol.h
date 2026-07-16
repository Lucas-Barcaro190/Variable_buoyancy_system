#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "src/core/shared_state.h"

typedef enum {
    CMD_ACK_CMD = 0x10,
    CMD_ACK_MOVE = 0x12,
    CMD_TIMEOUT_EMERGENCY = 0x14,
    CMD_CHANGE_ID = 0x1A,
    CMD_PING_PONG = 0x1F,
    CMD_FULL_CONTRACT = 0x20,
    CMD_FULL_EXPAND = 0x2F,
    CMD_ABS_VOLUME = 0x21,
    CMD_REL_VOLUME = 0x23,
    CMD_REQ_VOLUME = 0x24,
    CMD_ABS_PISTON = 0x29,
    CMD_REL_PISTON = 0x2B,
    CMD_REQ_PISTON = 0x2C,
    CMD_ERR_OUT_OF_BOUNDS = 0x1D
} MsgType_t;

typedef enum {
    MSG_ACK_CMD = 0x0210,
    MSG_ACK_MOVE = 0x0112,
    MSG_TIMEOUT_EMERGENCY = 0x0414,
    MSG_CHANGE_ID = 0x011A,
    MSG_PING_PONG = 0x011F,
    MSG_FULL_CONTRACT = 0x0120,
    MSG_FULL_EXPAND = 0x012F,
    MSG_ABS_VOLUME = 0x0421,
    MSG_REL_VOLUME = 0x0423,
    MSG_REQ_VOLUME = 0x0424,
    MSG_ABS_PISTON = 0x0429,
    MSG_REL_PISTON = 0x042B,
    MSG_REQ_PISTON = 0x042C,
    MSG_ERR_OUT_OF_BOUNDS = 0x011D
} MsgTypeCode_t;

typedef struct __attribute__((packed)) {
    uint8_t crc;
    uint8_t address;
    uint8_t size;
    uint8_t msgType;
    uint8_t msgData[];
} messages_t;

typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t command_code;
} MsgDataAckCmd_t;

typedef struct __attribute__((packed)) {
    uint8_t status;
} MsgDataAckMove_t;

typedef struct __attribute__((packed)) {
    uint32_t timeout_sec;
} MsgDataTimeoutEmergency_t;

typedef struct __attribute__((packed)) {
    uint8_t new_id;
} MsgDataChangeId_t;

typedef struct __attribute__((packed)) {
    uint8_t id;
} MsgDataPingPong_t;

typedef struct __attribute__((packed)) {
    uint8_t value;
} MsgDataFullMove_t;

typedef struct __attribute__((packed)) {
    float volume;
} MsgDataVolume_t;

typedef struct __attribute__((packed)) {
    float position;
} MsgDataPiston_t;

typedef struct __attribute__((packed)) {
    uint8_t causing_cmd;
} MsgDataErrOutOfBounds_t;

uint8_t getMsgSize(uint16_t msg_type_code);
uint8_t getMsgType(uint16_t msg_type_code);
uint16_t makeMsgTypeCode(uint8_t size, uint8_t type);
uint8_t calculateCRC8Bluetooth(const uint8_t* data, size_t length);
void sendBinaryPacket(uint8_t msgType, const uint8_t* data, uint8_t size);
void handle_binary_command(uint8_t msgType, const uint8_t* data, uint8_t size);

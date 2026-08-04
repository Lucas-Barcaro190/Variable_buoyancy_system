"use strict";
/**
 * RP2040 GDB Server
 *
 * Copyright (C) 2021, Uri Shaked
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.GDBServer = exports.STOP_REPLY_TRAP = exports.STOP_REPLY_SIGINT = void 0;
const cortex_m0_core_js_1 = require("../cortex-m0-core.js");
const logging_js_1 = require("../utils/logging.js");
const gdb_utils_js_1 = require("./gdb-utils.js");
exports.STOP_REPLY_SIGINT = 'S02';
exports.STOP_REPLY_TRAP = 'S05';
/* string value: armv6m-none-unknown-eabi */
const lldbTriple = '61726d76366d2d6e6f6e652d756e6b6e6f776e2d65616269';
const registers = [
    `name:r0;bitsize:32;offset:0;encoding:int;format:hex;set:General Purpose Registers;generic:arg1;gcc:0;dwarf:0;`,
    `name:r1;bitsize:32;offset:4;encoding:int;format:hex;set:General Purpose Registers;generic:arg2;gcc:1;dwarf:1;`,
    `name:r2;bitsize:32;offset:8;encoding:int;format:hex;set:General Purpose Registers;generic:arg3;gcc:2;dwarf:2;`,
    `name:r3;bitsize:32;offset:12;encoding:int;format:hex;set:General Purpose Registers;generic:arg4;gcc:3;dwarf:3;`,
    `name:r4;bitsize:32;offset:16;encoding:int;format:hex;set:General Purpose Registers;gcc:4;dwarf:4;`,
    `name:r5;bitsize:32;offset:20;encoding:int;format:hex;set:General Purpose Registers;gcc:5;dwarf:5;`,
    `name:r6;bitsize:32;offset:24;encoding:int;format:hex;set:General Purpose Registers;gcc:6;dwarf:6;`,
    `name:r7;bitsize:32;offset:28;encoding:int;format:hex;set:General Purpose Registers;gcc:7;dwarf:7;`,
    `name:r8;bitsize:32;offset:32;encoding:int;format:hex;set:General Purpose Registers;gcc:8;dwarf:8;`,
    `name:r9;bitsize:32;offset:36;encoding:int;format:hex;set:General Purpose Registers;gcc:9;dwarf:9;`,
    `name:r10;bitsize:32;offset:40;encoding:int;format:hex;set:General Purpose Registers;gcc:10;dwarf:10;`,
    `name:r11;bitsize:32;offset:44;encoding:int;format:hex;set:General Purpose Registers;generic:fp;gcc:11;dwarf:11;`,
    `name:r12;bitsize:32;offset:48;encoding:int;format:hex;set:General Purpose Registers;gcc:12;dwarf:12;`,
    `name:sp;bitsize:32;offset:52;encoding:int;format:hex;set:General Purpose Registers;generic:sp;alt-name:r13;gcc:13;dwarf:13;`,
    `name:lr;bitsize:32;offset:56;encoding:int;format:hex;set:General Purpose Registers;generic:ra;alt-name:r14;gcc:14;dwarf:14;`,
    `name:pc;bitsize:32;offset:60;encoding:int;format:hex;set:General Purpose Registers;generic:pc;alt-name:r15;gcc:15;dwarf:15;`,
    `name:cpsr;bitsize:32;offset:64;encoding:int;format:hex;set:General Purpose Registers;generic:flags;alt-name:psr;gcc:16;dwarf:16;`,
];
const targetXML = `<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target version="1.0">
<architecture>arm</architecture>
<feature name="org.gnu.gdb.arm.m-profile">
<reg name="r0" bitsize="32" regnum="0" save-restore="yes" type="int" group="general"/>
<reg name="r1" bitsize="32" regnum="1" save-restore="yes" type="int" group="general"/>
<reg name="r2" bitsize="32" regnum="2" save-restore="yes" type="int" group="general"/>
<reg name="r3" bitsize="32" regnum="3" save-restore="yes" type="int" group="general"/>
<reg name="r4" bitsize="32" regnum="4" save-restore="yes" type="int" group="general"/>
<reg name="r5" bitsize="32" regnum="5" save-restore="yes" type="int" group="general"/>
<reg name="r6" bitsize="32" regnum="6" save-restore="yes" type="int" group="general"/>
<reg name="r7" bitsize="32" regnum="7" save-restore="yes" type="int" group="general"/>
<reg name="r8" bitsize="32" regnum="8" save-restore="yes" type="int" group="general"/>
<reg name="r9" bitsize="32" regnum="9" save-restore="yes" type="int" group="general"/>
<reg name="r10" bitsize="32" regnum="10" save-restore="yes" type="int" group="general"/>
<reg name="r11" bitsize="32" regnum="11" save-restore="yes" type="int" group="general"/>
<reg name="r12" bitsize="32" regnum="12" save-restore="yes" type="int" group="general"/>
<reg name="sp" bitsize="32" regnum="13" save-restore="yes" type="data_ptr" group="general"/>
<reg name="lr" bitsize="32" regnum="14" save-restore="yes" type="int" group="general"/>
<reg name="pc" bitsize="32" regnum="15" save-restore="yes" type="code_ptr" group="general"/>
<reg name="xPSR" bitsize="32" regnum="16" save-restore="yes" type="int" group="general"/>
</feature>
<feature name="org.gnu.gdb.arm.m-system">
<reg name="msp" bitsize="32" regnum="17" save-restore="yes" type="data_ptr" group="system"/>
<reg name="psp" bitsize="32" regnum="18" save-restore="yes" type="data_ptr" group="system"/>
<reg name="primask" bitsize="1" regnum="19" save-restore="yes" type="int8" group="system"/>
<reg name="basepri" bitsize="8" regnum="20" save-restore="yes" type="int8" group="system"/>
<reg name="faultmask" bitsize="1" regnum="21" save-restore="yes" type="int8" group="system"/>
<reg name="control" bitsize="2" regnum="22" save-restore="yes" type="int8" group="system"/>
</feature>
</target>`;
const LOG_NAME = 'GDBServer';
class GDBServer {
    constructor(target) {
        this.target = target;
        this.logger = new logging_js_1.ConsoleLogger(logging_js_1.LogLevel.Warn, true);
        this.connections = new Set();
    }
    processGDBMessage(cmd) {
        const { rp2040 } = this.target;
        const { core } = rp2040;
        if (cmd === 'Hg0') {
            return (0, gdb_utils_js_1.gdbMessage)('OK');
        }
        switch (cmd[0]) {
            case '?':
                return (0, gdb_utils_js_1.gdbMessage)(exports.STOP_REPLY_TRAP);
            case 'q':
                // Query things
                if (cmd.startsWith('qSupported:')) {
                    return (0, gdb_utils_js_1.gdbMessage)('PacketSize=4000;vContSupported+;qXfer:features:read+');
                }
                if (cmd === 'qAttached') {
                    return (0, gdb_utils_js_1.gdbMessage)('1');
                }
                if (cmd.startsWith('qXfer:features:read:target.xml')) {
                    return (0, gdb_utils_js_1.gdbMessage)('l' + targetXML);
                }
                if (cmd.startsWith('qRegisterInfo')) {
                    const index = parseInt(cmd.substring(13), 16);
                    const register = registers[index];
                    if (register) {
                        return (0, gdb_utils_js_1.gdbMessage)(register);
                    }
                    else {
                        return (0, gdb_utils_js_1.gdbMessage)(`E45`);
                    }
                }
                if (cmd === 'qHostInfo') {
                    return (0, gdb_utils_js_1.gdbMessage)(`triple:${lldbTriple};endian:little;ptrsize:4;`);
                }
                if (cmd === 'qProcessInfo') {
                    return (0, gdb_utils_js_1.gdbMessage)('pid:1;endian:little;ptrsize:4;');
                }
                return (0, gdb_utils_js_1.gdbMessage)('');
            case 'v':
                if (cmd === 'vCont?') {
                    return (0, gdb_utils_js_1.gdbMessage)('vCont;c;C;s;S');
                }
                if (cmd.startsWith('vCont;c')) {
                    if (!this.target.executing) {
                        this.target.execute();
                    }
                    return;
                }
                if (cmd.startsWith('vCont;s')) {
                    rp2040.step();
                    const registerStatus = [];
                    for (let i = 0; i < 17; i++) {
                        const value = i === 16 ? core.xPSR : core.registers[i];
                        registerStatus.push(`${(0, gdb_utils_js_1.encodeHexByte)(i)}:${(0, gdb_utils_js_1.encodeHexUint32)(value)}`);
                    }
                    return (0, gdb_utils_js_1.gdbMessage)(`T05${registerStatus.join(';')};reason:trace;`);
                }
                break;
            case 'c':
                if (!this.target.executing) {
                    this.target.execute();
                }
                return (0, gdb_utils_js_1.gdbMessage)('OK');
            case 'g': {
                // Read registers
                const buf = new Uint32Array(17);
                buf.set(core.registers);
                buf[16] = core.xPSR;
                return (0, gdb_utils_js_1.gdbMessage)((0, gdb_utils_js_1.encodeHexBuf)(new Uint8Array(buf.buffer)));
            }
            case 'p': {
                // Read register
                const registerIndex = parseInt(cmd.substr(1), 16);
                if (registerIndex >= 0 && registerIndex <= 15) {
                    return (0, gdb_utils_js_1.gdbMessage)((0, gdb_utils_js_1.encodeHexUint32)(core.registers[registerIndex]));
                }
                const specialRegister = (sysm) => (0, gdb_utils_js_1.gdbMessage)((0, gdb_utils_js_1.encodeHexUint32)(core.readSpecialRegister(sysm)));
                switch (registerIndex) {
                    case 0x10:
                        return (0, gdb_utils_js_1.gdbMessage)((0, gdb_utils_js_1.encodeHexUint32)(core.xPSR));
                    case 0x11:
                        return specialRegister(cortex_m0_core_js_1.SYSM_MSP);
                    case 0x12:
                        return specialRegister(cortex_m0_core_js_1.SYSM_PSP);
                    case 0x13:
                        return specialRegister(cortex_m0_core_js_1.SYSM_PRIMASK);
                    case 0x14:
                        this.logger.warn(LOG_NAME, 'TODO BASEPRI');
                        return (0, gdb_utils_js_1.gdbMessage)((0, gdb_utils_js_1.encodeHexUint32)(0)); // TODO BASEPRI
                    case 0x15:
                        this.logger.warn(LOG_NAME, 'TODO faultmask');
                        return (0, gdb_utils_js_1.gdbMessage)((0, gdb_utils_js_1.encodeHexUint32)(0)); // TODO faultmask
                    case 0x16:
                        return specialRegister(cortex_m0_core_js_1.SYSM_CONTROL);
                }
                break;
            }
            case 'P': {
                // Write register
                const params = cmd.substring(1).split('=');
                const registerIndex = parseInt(params[0], 16);
                const registerValue = params[1].trim();
                const registerBytes = registerIndex > 0x12 ? 1 : 4;
                const decodedValue = (0, gdb_utils_js_1.decodeHexBuf)(registerValue);
                if (registerIndex < 0 || registerIndex > 0x16 || decodedValue.length !== registerBytes) {
                    return (0, gdb_utils_js_1.gdbMessage)('E00');
                }
                const valueBuffer = new Uint8Array(4);
                valueBuffer.set(decodedValue.slice(0, 4));
                const value = new DataView(valueBuffer.buffer).getUint32(0, true);
                switch (registerIndex) {
                    case 0x10:
                        core.xPSR = value;
                        break;
                    case 0x11:
                        core.writeSpecialRegister(cortex_m0_core_js_1.SYSM_MSP, value);
                        break;
                    case 0x12:
                        core.writeSpecialRegister(cortex_m0_core_js_1.SYSM_PSP, value);
                        break;
                    case 0x13:
                        core.writeSpecialRegister(cortex_m0_core_js_1.SYSM_PRIMASK, value);
                        break;
                    case 0x14:
                        this.logger.warn(LOG_NAME, 'TODO BASEPRI');
                        break; // TODO BASEPRI
                    case 0x15:
                        this.logger.warn(LOG_NAME, 'TODO faultmask');
                        break; // TODO faultmask
                    case 0x16:
                        core.writeSpecialRegister(cortex_m0_core_js_1.SYSM_CONTROL, value);
                        break;
                    default:
                        core.registers[registerIndex] = value;
                        break;
                }
                return (0, gdb_utils_js_1.gdbMessage)('OK');
            }
            case 'm': {
                // Read memory
                const params = cmd.substr(1).split(',');
                const address = parseInt(params[0], 16);
                const length = parseInt(params[1], 16);
                let result = '';
                for (let i = 0; i < length; i++) {
                    result += (0, gdb_utils_js_1.encodeHexByte)(rp2040.readUint8(address + i));
                }
                return (0, gdb_utils_js_1.gdbMessage)(result);
            }
            case 'M': {
                // Write memory
                const params = cmd.substr(1).split(/[,:]/);
                const address = parseInt(params[0], 16);
                const length = parseInt(params[1], 16);
                const data = (0, gdb_utils_js_1.decodeHexBuf)(params[2].substr(0, length * 2));
                for (let i = 0; i < data.length; i++) {
                    this.debug(`Write ${data[i].toString(16)} to ${(address + i).toString(16)}`);
                    rp2040.writeUint8(address + i, data[i]);
                }
                return (0, gdb_utils_js_1.gdbMessage)('OK');
            }
        }
        return (0, gdb_utils_js_1.gdbMessage)('');
    }
    addConnection(connection) {
        const { rp2040 } = this.target;
        this.connections.add(connection);
        rp2040.onBreak = () => {
            this.target.stop();
            rp2040.core.PC -= rp2040.core.breakRewind;
            for (const connection of this.connections) {
                connection.onBreakpoint();
            }
        };
    }
    removeConnection(connection) {
        this.connections.delete(connection);
    }
    debug(msg) {
        this.logger.debug(LOG_NAME, msg);
    }
    info(msg) {
        this.logger.info(LOG_NAME, msg);
    }
    warn(msg) {
        this.logger.warn(LOG_NAME, msg);
    }
    error(msg) {
        this.logger.error(LOG_NAME, msg);
    }
}
exports.GDBServer = GDBServer;

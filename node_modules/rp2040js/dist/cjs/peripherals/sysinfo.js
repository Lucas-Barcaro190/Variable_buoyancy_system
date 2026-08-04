"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.RP2040SysInfo = void 0;
const peripheral_js_1 = require("./peripheral.js");
const CHIP_ID = 0;
const PLATFORM = 0x4;
const GITREF_RP2040 = 0x40;
class RP2040SysInfo extends peripheral_js_1.BasePeripheral {
    readUint32(offset) {
        // All the values here were verified against the silicon
        switch (offset) {
            case CHIP_ID:
                return 0x10002927;
            case PLATFORM:
                return 0x00000002;
            case GITREF_RP2040:
                return 0xe0c912e8;
        }
        return super.readUint32(offset);
    }
}
exports.RP2040SysInfo = RP2040SysInfo;

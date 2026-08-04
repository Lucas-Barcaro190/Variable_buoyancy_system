"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.RPTBMAN = void 0;
const peripheral_js_1 = require("./peripheral.js");
const PLATFORM = 0;
const ASIC = 1;
class RPTBMAN extends peripheral_js_1.BasePeripheral {
    readUint32(offset) {
        switch (offset) {
            case PLATFORM:
                return ASIC;
            default:
                return super.readUint32(offset);
        }
    }
}
exports.RPTBMAN = RPTBMAN;

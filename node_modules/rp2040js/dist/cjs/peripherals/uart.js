"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.RPUART = void 0;
const fifo_js_1 = require("../utils/fifo.js");
const peripheral_js_1 = require("./peripheral.js");
const UARTDR = 0x0;
const UARTFR = 0x18;
const UARTIBRD = 0x24;
const UARTFBRD = 0x28;
const UARTLCR_H = 0x2c;
const UARTCR = 0x30;
const UARTIMSC = 0x38;
const UARTIRIS = 0x3c;
const UARTIMIS = 0x40;
const UARTICR = 0x44;
const UARTPERIPHID0 = 0xfe0;
const UARTPERIPHID1 = 0xfe4;
const UARTPERIPHID2 = 0xfe8;
const UARTPERIPHID3 = 0xfec;
const UARTPCELLID0 = 0xff0;
const UARTPCELLID1 = 0xff4;
const UARTPCELLID2 = 0xff8;
const UARTPCELLID3 = 0xffc;
// UARTFR bits:
const TXFE = 1 << 7;
const RXFF = 1 << 6;
const RXFE = 1 << 4;
// UARTLCR_H bits:
const FEN = 1 << 4;
// UARTCR bits:
const RXE = 1 << 9;
const TXE = 1 << 8;
const UARTEN = 1 << 0;
// Interrupt bits
const UARTTXINTR = 1 << 5;
const UARTRXINTR = 1 << 4;
class RPUART extends peripheral_js_1.BasePeripheral {
    constructor(rp2040, name, irq, dreq) {
        super(rp2040, name);
        this.irq = irq;
        this.dreq = dreq;
        this.ctrlRegister = RXE | TXE;
        this.lineCtrlRegister = 0;
        this.rxFIFO = new fifo_js_1.FIFO(32);
        this.interruptMask = 0;
        this.interruptStatus = 0;
        this.intDivisor = 0;
        this.fracDivisor = 0;
    }
    get enabled() {
        return !!(this.ctrlRegister & UARTEN);
    }
    get txEnabled() {
        return !!(this.ctrlRegister & TXE);
    }
    get rxEnabled() {
        return !!(this.ctrlRegister & RXE);
    }
    get fifosEnabled() {
        return !!(this.lineCtrlRegister & FEN);
    }
    /**
     * Number of bits per UART character
     */
    get wordLength() {
        switch ((this.lineCtrlRegister >>> 5) & 0x3) {
            case 0b00:
                return 5;
            case 0b01:
                return 6;
            case 0b10:
                return 7;
            case 0b11:
                return 8;
        }
    }
    get baudDivider() {
        return this.intDivisor + this.fracDivisor / 64;
    }
    get baudRate() {
        return Math.round(this.rp2040.clkPeri / (this.baudDivider * 16));
    }
    get flags() {
        return (this.rxFIFO.full ? RXFF : 0) | (this.rxFIFO.empty ? RXFE : 0) | TXFE;
    }
    checkInterrupts() {
        // TODO We should actually implement a proper FIFO for TX
        this.interruptStatus |= UARTTXINTR;
        this.rp2040.setInterrupt(this.irq, !!(this.interruptStatus & this.interruptMask));
    }
    feedByte(value) {
        this.rxFIFO.push(value);
        // TODO check if the FIFO has reached the threshold level
        this.interruptStatus |= UARTRXINTR;
        this.checkInterrupts();
    }
    readUint32(offset) {
        switch (offset) {
            case UARTDR: {
                const value = this.rxFIFO.pull();
                if (!this.rxFIFO.empty) {
                    this.interruptStatus |= UARTRXINTR;
                }
                else {
                    this.interruptStatus &= ~UARTRXINTR;
                }
                this.checkInterrupts();
                return value;
            }
            case UARTFR:
                return this.flags;
            case UARTIBRD:
                return this.intDivisor;
            case UARTFBRD:
                return this.fracDivisor;
            case UARTLCR_H:
                return this.lineCtrlRegister;
            case UARTCR:
                return this.ctrlRegister;
            case UARTIMSC:
                return this.interruptMask;
            case UARTIRIS:
                return this.interruptStatus;
            case UARTIMIS:
                return this.interruptStatus & this.interruptMask;
            case UARTPERIPHID0:
                return 0x11;
            case UARTPERIPHID1:
                return 0x10;
            case UARTPERIPHID2:
                return 0x34;
            case UARTPERIPHID3:
                return 0x00;
            case UARTPCELLID0:
                return 0x0d;
            case UARTPCELLID1:
                return 0xf0;
            case UARTPCELLID2:
                return 0x05;
            case UARTPCELLID3:
                return 0xb1;
        }
        return super.readUint32(offset);
    }
    writeUint32(offset, value) {
        var _a, _b, _c;
        switch (offset) {
            case UARTDR:
                (_a = this.onByte) === null || _a === void 0 ? void 0 : _a.call(this, value & 0xff);
                break;
            case UARTIBRD:
                this.intDivisor = value & 0xffff;
                (_b = this.onBaudRateChange) === null || _b === void 0 ? void 0 : _b.call(this, this.baudRate);
                break;
            case UARTFBRD:
                this.fracDivisor = value & 0x3f;
                (_c = this.onBaudRateChange) === null || _c === void 0 ? void 0 : _c.call(this, this.baudRate);
                break;
            case UARTLCR_H:
                this.lineCtrlRegister = value;
                break;
            case UARTCR:
                this.ctrlRegister = value;
                if (this.enabled) {
                    this.rp2040.dma.setDREQ(this.dreq.tx);
                }
                else {
                    this.rp2040.dma.clearDREQ(this.dreq.tx);
                }
                break;
            case UARTIMSC:
                this.interruptMask = value & 0x7ff;
                this.checkInterrupts();
                break;
            case UARTICR:
                this.interruptStatus &= ~this.rawWriteValue;
                this.checkInterrupts();
                break;
            default:
                super.writeUint32(offset, value);
        }
    }
}
exports.RPUART = RPUART;

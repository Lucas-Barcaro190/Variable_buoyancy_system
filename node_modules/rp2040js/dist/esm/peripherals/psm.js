import { BasePeripheral } from './peripheral.js';
const FRCE_ON = 0x00;
const FRCE_OFF = 0x04;
const WDSEL = 0x08;
const DONE = 0x0c;
const PSM_BITS_MASK = 0x0001ffff;
export class RPPSM extends BasePeripheral {
    constructor() {
        super(...arguments);
        this.frceOn = 0;
        this.frceOff = 0;
        this.wdsel = 0;
    }
    readUint32(offset) {
        switch (offset) {
            case FRCE_ON:
                return this.frceOn;
            case FRCE_OFF:
                return this.frceOff;
            case WDSEL:
                return this.wdsel;
            case DONE:
                // Domains are ready unless forced off (FRCE_ON overrides FRCE_OFF)
                return (PSM_BITS_MASK & ~this.frceOff) | (this.frceOn & this.frceOff);
        }
        return super.readUint32(offset);
    }
    writeUint32(offset, value) {
        switch (offset) {
            case FRCE_ON:
                this.frceOn = value & PSM_BITS_MASK;
                break;
            case FRCE_OFF:
                this.frceOff = value & PSM_BITS_MASK;
                break;
            case WDSEL:
                this.wdsel = value & PSM_BITS_MASK;
                break;
            default:
                super.writeUint32(offset, value);
                break;
        }
    }
}

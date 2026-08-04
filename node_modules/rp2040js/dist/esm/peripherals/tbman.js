import { BasePeripheral } from './peripheral.js';
const PLATFORM = 0;
const ASIC = 1;
export class RPTBMAN extends BasePeripheral {
    readUint32(offset) {
        switch (offset) {
            case PLATFORM:
                return ASIC;
            default:
                return super.readUint32(offset);
        }
    }
}

import { BasePeripheral, Peripheral } from './peripheral.js';
export declare class RPTBMAN extends BasePeripheral implements Peripheral {
    readUint32(offset: number): number;
}

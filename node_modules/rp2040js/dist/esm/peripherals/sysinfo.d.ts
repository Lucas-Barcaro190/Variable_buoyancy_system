import { BasePeripheral, Peripheral } from './peripheral.js';
export declare class RP2040SysInfo extends BasePeripheral implements Peripheral {
    readUint32(offset: number): number;
}

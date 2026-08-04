import { BasePeripheral, Peripheral } from './peripheral.js';
export declare class RP2040RTC extends BasePeripheral implements Peripheral {
    setup0: number;
    setup1: number;
    ctrl: number;
    baseline: Date;
    baselineNanos: number;
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
}

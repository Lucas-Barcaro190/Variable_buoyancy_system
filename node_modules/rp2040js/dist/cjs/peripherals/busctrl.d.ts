import { RP2040 } from '../rp2040.js';
import { BasePeripheral, Peripheral } from './peripheral.js';
export declare class RPBUSCTRL extends BasePeripheral implements Peripheral {
    voltageSelect: number;
    readonly perfCtr: number[];
    readonly perfSel: number[];
    constructor(rp2040: RP2040, name: string);
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
}

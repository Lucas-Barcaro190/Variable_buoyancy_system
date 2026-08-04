import { BasePeripheral, Peripheral } from './peripheral.js';
export declare class RPXOSC extends BasePeripheral implements Peripheral {
    private ctrl;
    private status;
    private dormant;
    private startup;
    private count;
    private enabled;
    private stable;
    private isDormant;
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
}

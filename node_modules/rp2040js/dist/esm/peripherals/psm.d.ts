import { BasePeripheral, Peripheral } from './peripheral.js';
export declare class RPPSM extends BasePeripheral implements Peripheral {
    private frceOn;
    private frceOff;
    private wdsel;
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
}

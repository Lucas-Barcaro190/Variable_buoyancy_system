import { BasePeripheral, Peripheral } from './peripheral.js';
export declare class RPSSI extends BasePeripheral implements Peripheral {
    private dr0;
    private txflr;
    private rxflr;
    private baudr;
    private crtlr0;
    private crtlr1;
    private ssienr;
    private spictlr0;
    private rxsampldly;
    private txddriveedge;
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
}

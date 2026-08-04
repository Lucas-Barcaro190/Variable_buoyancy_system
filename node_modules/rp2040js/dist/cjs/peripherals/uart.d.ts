import { RP2040 } from '../rp2040.js';
import { DREQChannel } from './dma.js';
import { BasePeripheral, Peripheral } from './peripheral.js';
export interface IUARTDMAChannels {
    rx: DREQChannel;
    tx: DREQChannel;
}
export declare class RPUART extends BasePeripheral implements Peripheral {
    readonly irq: number;
    readonly dreq: IUARTDMAChannels;
    private ctrlRegister;
    private lineCtrlRegister;
    private rxFIFO;
    private interruptMask;
    private interruptStatus;
    private intDivisor;
    private fracDivisor;
    onByte?: (value: number) => void;
    onBaudRateChange?: (baudRate: number) => void;
    constructor(rp2040: RP2040, name: string, irq: number, dreq: IUARTDMAChannels);
    get enabled(): boolean;
    get txEnabled(): boolean;
    get rxEnabled(): boolean;
    get fifosEnabled(): boolean;
    /**
     * Number of bits per UART character
     */
    get wordLength(): 5 | 6 | 7 | 8 | undefined;
    get baudDivider(): number;
    get baudRate(): number;
    get flags(): number;
    checkInterrupts(): void;
    feedByte(value: number): void;
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
}

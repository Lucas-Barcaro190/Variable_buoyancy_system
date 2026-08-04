import { IClock } from '../clock/clock.js';
import { Timer32, Timer32PeriodicAlarm } from '../utils/timer32.js';
import { BasePeripheral, Peripheral } from './peripheral.js';
declare enum PWMDivMode {
    FreeRunning = 0,
    BGated = 1,
    BRisingEdge = 2,
    BFallingEdge = 3
}
declare class PWMChannel {
    private pwm;
    readonly clock: IClock;
    readonly index: number;
    readonly timer: Timer32;
    readonly alarmA: Timer32PeriodicAlarm;
    readonly alarmB: Timer32PeriodicAlarm;
    readonly alarmBottom: Timer32PeriodicAlarm;
    csr: number;
    div: number;
    cc: number;
    top: number;
    lastBValue: boolean;
    countingUp: boolean;
    ccUpdated: boolean;
    topUpdated: boolean;
    tickCounter: number;
    divMode: PWMDivMode;
    readonly pinA1: number;
    readonly pinB1: number;
    readonly pinA2: number;
    readonly pinB2: number;
    constructor(pwm: RPPWM, clock: IClock, index: number);
    readRegister(offset: number): number;
    writeRegister(offset: number, value: number): void;
    reset(): void;
    private updateDoubleBuffered;
    private wrap;
    setA(value: boolean): void;
    setB(value: boolean): void;
    get gpioBValue(): boolean;
    setBDirection(value: boolean): void;
    gpioBChanged(): void;
    updateEnable(): void;
    set en(value: number);
}
export declare class RPPWM extends BasePeripheral implements Peripheral {
    readonly channels: PWMChannel[];
    private intRaw;
    private intEnable;
    private intForce;
    gpioValue: number;
    gpioDirection: number;
    get intStatus(): number;
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
    get clockFreq(): number;
    channelInterrupt(index: number): void;
    checkInterrupts(): void;
    gpioSet(index: number, value: boolean): void;
    gpioSetDir(index: number, output: boolean): void;
    gpioRead(index: number): boolean;
    gpioOnInput(index: number): void;
    reset(): void;
}
export {};

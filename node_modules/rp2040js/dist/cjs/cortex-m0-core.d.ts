import { RP2040 } from './rp2040.js';
export declare const SYSM_MSP = 8;
export declare const SYSM_PSP = 9;
export declare const SYSM_PRIMASK = 16;
export declare const SYSM_CONTROL = 20;
declare enum ExecutionMode {
    Mode_Thread = 0,
    Mode_Handler = 1
}
declare enum StackPointerBank {
    SPmain = 0,
    SPprocess = 1
}
export declare class CortexM0Core {
    readonly rp2040: RP2040;
    readonly registers: Uint32Array<ArrayBuffer>;
    bankedSP: number;
    cycles: number;
    eventRegistered: boolean;
    waiting: boolean;
    N: boolean;
    C: boolean;
    Z: boolean;
    V: boolean;
    breakRewind: number;
    PM: boolean;
    SPSEL: StackPointerBank;
    nPRIV: boolean;
    currentMode: ExecutionMode;
    IPSR: number;
    interruptNMIMask: number;
    pendingInterrupts: number;
    enabledInterrupts: number;
    interruptPriorities: number[];
    pendingNMI: boolean;
    pendingPendSV: boolean;
    pendingSVCall: boolean;
    pendingSystick: boolean;
    interruptsUpdated: boolean;
    VTOR: number;
    SHPR2: number;
    SHPR3: number;
    /** Hook to listen for function calls - branch-link (BL/BLX) instructions */
    blTaken: (core: CortexM0Core, blx: boolean) => void;
    constructor(rp2040: RP2040);
    get logger(): import("./index.js").Logger;
    reset(): void;
    get SP(): number;
    set SP(value: number);
    get LR(): number;
    set LR(value: number);
    get PC(): number;
    set PC(value: number);
    get APSR(): number;
    set APSR(value: number);
    get xPSR(): number;
    set xPSR(value: number);
    checkCondition(cond: number): boolean;
    readUint32(address: number): number;
    readUint16(address: number): number;
    readUint8(address: number): number;
    writeUint32(address: number, value: number): void;
    writeUint16(address: number, value: number): void;
    writeUint8(address: number, value: number): void;
    switchStack(stack: StackPointerBank): void;
    get SPprocess(): number;
    set SPprocess(value: number);
    get SPmain(): number;
    set SPmain(value: number);
    exceptionEntry(exceptionNumber: number): void;
    exceptionReturn(excReturn: number): void;
    get pendSVPriority(): number;
    get svCallPriority(): number;
    get systickPriority(): number;
    exceptionPriority(n: number): number;
    get vectPending(): number;
    setInterrupt(irq: number, value: boolean): void;
    checkForInterrupts(): boolean;
    readSpecialRegister(sysm: number): number;
    writeSpecialRegister(sysm: number, value: number): 0 | undefined;
    BXWritePC(address: number): void;
    private substractUpdateFlags;
    private addUpdateFlags;
    cyclesIO(addr: number, write?: boolean): 0 | 1 | 3 | 4;
    executeInstruction(): number;
}
export {};

import { RP2040 } from '../rp2040.js';
import { Timer32, Timer32PeriodicAlarm } from '../utils/timer32.js';
import { BasePeripheral, Peripheral } from './peripheral.js';
export declare const CPUID = 3328;
export declare const ICSR = 3332;
export declare const VTOR = 3336;
export declare const SHPR2 = 3356;
export declare const SHPR3 = 3360;
/** PPB stands for Private Periphral Bus.
 * These are peripherals that are part of the ARM Cortex Core, and there's one copy for each processor core.
 *
 * Included peripheral: NVIC, SysTick timer
 */
export declare class RPPPB extends BasePeripheral implements Peripheral {
    systickCountFlag: boolean;
    systickClkSource: boolean;
    systickIntEnable: boolean;
    systickReload: number;
    readonly systickTimer: Timer32;
    readonly systickAlarm: Timer32PeriodicAlarm;
    constructor(rp2040: RP2040, name: string);
    reset(): void;
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
}

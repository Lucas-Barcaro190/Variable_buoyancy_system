import { RP2040 } from '../rp2040.js';
import { Timer32, Timer32PeriodicAlarm } from '../utils/timer32.js';
import { BasePeripheral, Peripheral } from './peripheral.js';
export declare class RPWatchdog extends BasePeripheral implements Peripheral {
    readonly timer: Timer32;
    readonly alarm: Timer32PeriodicAlarm;
    readonly scratchData: Uint32Array<ArrayBuffer>;
    private enable;
    private tickEnable;
    private reason;
    private pauseDbg0;
    private pauseDbg1;
    private pauseJtag;
    /** Called when the watchdog triggers - override with your own soft reset implementation */
    onWatchdogTrigger: () => void;
    constructor(rp2040: RP2040, name: string);
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
}

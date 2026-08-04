import { SimulationClock } from './clock/simulation-clock.js';
import { IGDBTarget } from './gdb/gdb-target.js';
import { RP2040 } from './rp2040.js';
export declare class Simulator implements IGDBTarget {
    readonly clock: SimulationClock;
    executeTimer: ReturnType<typeof setTimeout> | null;
    rp2040: RP2040;
    stopped: boolean;
    constructor(clock?: SimulationClock);
    execute(): void;
    stop(): void;
    get executing(): boolean;
}

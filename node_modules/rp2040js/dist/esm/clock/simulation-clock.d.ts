import { AlarmCallback, IAlarm, IClock } from './clock.js';
type ClockEventCallback = () => void;
export declare class ClockAlarm implements IAlarm {
    private readonly clock;
    readonly callback: AlarmCallback;
    next: ClockAlarm | null;
    nanos: number;
    scheduled: boolean;
    constructor(clock: SimulationClock, callback: AlarmCallback);
    schedule(deltaNanos: number): void;
    cancel(): void;
}
export declare class SimulationClock implements IClock {
    readonly frequency: number;
    private nextAlarm;
    private nanosCounter;
    constructor(frequency?: number);
    get nanos(): number;
    get micros(): number;
    createAlarm(callback: ClockEventCallback): ClockAlarm;
    linkAlarm(nanos: number, alarm: ClockAlarm): ClockAlarm;
    unlinkAlarm(alarm: ClockAlarm): boolean;
    tick(deltaNanos: number): void;
    get nanosToNextAlarm(): number;
}
export {};

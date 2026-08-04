import { IClock } from '../clock/clock.js';
export declare enum TimerMode {
    Increment = 0,
    Decrement = 1,
    ZigZag = 2
}
export declare class Timer32 {
    readonly clock: IClock;
    private baseFreq;
    private baseValue;
    private baseNanos;
    private topValue;
    private prescalerValue;
    private timerMode;
    private enabled;
    readonly listeners: (() => void)[];
    constructor(clock: IClock, baseFreq: number);
    reset(): void;
    set(value: number, zigZagDown?: boolean): void;
    /**
     * Advances the counter by the given amount. Note that this will
     * decrease the counter if the timer is running in Decrement mode.
     *
     * @param delta The value to add to the counter. Can be negative.
     */
    advance(delta: number): void;
    get rawCounter(): number;
    get counter(): number;
    get top(): number;
    set top(value: number);
    get frequency(): number;
    set frequency(value: number);
    get prescaler(): number;
    set prescaler(value: number);
    toNanos(cycles: number): number;
    get enable(): boolean;
    set enable(value: boolean);
    get mode(): TimerMode;
    set mode(value: TimerMode);
    private updated;
}
export declare class Timer32PeriodicAlarm {
    readonly timer: Timer32;
    readonly callback: () => void;
    private targetValue;
    private enabled;
    private clockAlarm;
    constructor(timer: Timer32, callback: () => void);
    get enable(): boolean;
    set enable(value: boolean);
    get target(): number;
    set target(value: number);
    handleAlarm: () => void;
    update: () => void;
    private schedule;
    private cancel;
}

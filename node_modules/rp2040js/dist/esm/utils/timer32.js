export var TimerMode;
(function (TimerMode) {
    TimerMode[TimerMode["Increment"] = 0] = "Increment";
    TimerMode[TimerMode["Decrement"] = 1] = "Decrement";
    TimerMode[TimerMode["ZigZag"] = 2] = "ZigZag";
})(TimerMode || (TimerMode = {}));
export class Timer32 {
    constructor(clock, baseFreq) {
        this.clock = clock;
        this.baseFreq = baseFreq;
        this.baseValue = 0;
        this.baseNanos = 0;
        this.topValue = 0xffffffff;
        this.prescalerValue = 1;
        this.timerMode = TimerMode.Increment;
        this.enabled = true;
        this.listeners = [];
    }
    reset() {
        this.baseNanos = this.clock.nanos;
        this.baseValue = 0;
        this.updated();
    }
    set(value, zigZagDown = false) {
        this.baseValue = zigZagDown ? this.topValue * 2 - value : value;
        this.baseNanos = this.clock.nanos;
        this.updated();
    }
    /**
     * Advances the counter by the given amount. Note that this will
     * decrease the counter if the timer is running in Decrement mode.
     *
     * @param delta The value to add to the counter. Can be negative.
     */
    advance(delta) {
        this.baseValue += delta;
    }
    get rawCounter() {
        const { baseFreq, prescalerValue, baseNanos, baseValue, enabled, timerMode } = this;
        if (!baseFreq || !prescalerValue || !enabled) {
            return this.baseValue;
        }
        const zigzag = timerMode == TimerMode.ZigZag;
        const ticks = ((this.clock.nanos - baseNanos) / 1e9) * (baseFreq / prescalerValue);
        const topModulo = zigzag ? this.topValue * 2 : this.topValue + 1;
        const delta = timerMode == TimerMode.Decrement ? topModulo - (ticks % topModulo) : ticks;
        let currentValue = Math.round(baseValue + delta);
        if (this.topValue != 0xffffffff) {
            currentValue %= topModulo;
        }
        return currentValue;
    }
    get counter() {
        let currentValue = this.rawCounter;
        if (this.timerMode == TimerMode.ZigZag && currentValue > this.topValue) {
            currentValue = this.topValue * 2 - currentValue;
        }
        return currentValue >>> 0;
    }
    get top() {
        return this.topValue;
    }
    set top(value) {
        const { counter } = this;
        this.topValue = value;
        this.set(counter <= this.topValue ? counter : 0);
    }
    get frequency() {
        return this.baseFreq;
    }
    set frequency(value) {
        this.baseValue = this.counter;
        this.baseNanos = this.clock.nanos;
        this.baseFreq = value;
        this.updated();
    }
    get prescaler() {
        return this.prescalerValue;
    }
    set prescaler(value) {
        this.baseValue = this.counter;
        this.baseNanos = this.clock.nanos;
        this.enabled = this.prescalerValue !== 0;
        this.prescalerValue = value;
        this.updated();
    }
    toNanos(cycles) {
        const { baseFreq, prescalerValue } = this;
        return (cycles * 1e9) / (baseFreq / prescalerValue);
    }
    get enable() {
        return this.enabled;
    }
    set enable(value) {
        if (value !== this.enabled) {
            if (value) {
                this.baseNanos = this.clock.nanos;
            }
            else {
                this.baseValue = this.counter;
            }
            this.enabled = value;
            this.updated();
        }
    }
    get mode() {
        return this.timerMode;
    }
    set mode(value) {
        if (this.timerMode !== value) {
            const { counter } = this;
            this.timerMode = value;
            this.set(counter);
        }
    }
    updated() {
        for (const listener of this.listeners) {
            listener();
        }
    }
}
export class Timer32PeriodicAlarm {
    constructor(timer, callback) {
        this.timer = timer;
        this.callback = callback;
        this.targetValue = 0;
        this.enabled = false;
        this.handleAlarm = () => {
            this.callback();
            if (this.enabled && this.timer.enable) {
                this.schedule();
            }
        };
        this.update = () => {
            this.cancel();
            if (this.enabled && this.timer.enable) {
                this.schedule();
            }
        };
        this.clockAlarm = this.timer.clock.createAlarm(this.handleAlarm);
        timer.listeners.push(this.update);
    }
    get enable() {
        return this.enabled;
    }
    set enable(value) {
        if (value !== this.enabled) {
            this.enabled = value;
            if (value && this.timer.enable) {
                this.schedule();
            }
            else {
                this.cancel();
            }
        }
    }
    get target() {
        return this.targetValue;
    }
    set target(value) {
        if (value === this.targetValue) {
            return;
        }
        this.targetValue = value;
        if (this.enabled && this.timer.enable) {
            this.cancel();
            this.schedule();
        }
    }
    schedule() {
        const { timer, targetValue } = this;
        const { top, mode, rawCounter } = timer;
        let cycleDelta = targetValue - rawCounter;
        if (mode === TimerMode.ZigZag && cycleDelta < 0) {
            if (cycleDelta < -top) {
                cycleDelta += 2 * top;
            }
            else {
                cycleDelta = top * 2 - targetValue - rawCounter;
            }
        }
        if (top != 0xffffffff) {
            if (cycleDelta <= 0) {
                cycleDelta += top + 1;
            }
            if (targetValue > top) {
                // Skip alarm
                return;
            }
        }
        if (mode === TimerMode.Decrement) {
            cycleDelta = top + 1 - cycleDelta;
        }
        const cyclesToAlarm = cycleDelta >>> 0;
        const nanosToAlarm = timer.toNanos(cyclesToAlarm);
        this.clockAlarm.schedule(nanosToAlarm);
    }
    cancel() {
        this.clockAlarm.cancel();
    }
}

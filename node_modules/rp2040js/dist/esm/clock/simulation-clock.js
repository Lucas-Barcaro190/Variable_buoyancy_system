export class ClockAlarm {
    constructor(clock, callback) {
        this.clock = clock;
        this.callback = callback;
        this.next = null;
        this.nanos = 0;
        this.scheduled = false;
    }
    schedule(deltaNanos) {
        if (this.scheduled) {
            this.cancel();
        }
        this.clock.linkAlarm(deltaNanos, this);
    }
    cancel() {
        this.clock.unlinkAlarm(this);
        this.scheduled = false;
    }
}
export class SimulationClock {
    constructor(frequency = 125e6) {
        this.frequency = frequency;
        this.nextAlarm = null;
        this.nanosCounter = 0;
    }
    get nanos() {
        return this.nanosCounter;
    }
    get micros() {
        return this.nanos / 1000;
    }
    createAlarm(callback) {
        return new ClockAlarm(this, callback);
    }
    linkAlarm(nanos, alarm) {
        alarm.nanos = this.nanos + nanos;
        let alarmListItem = this.nextAlarm;
        let lastItem = null;
        while (alarmListItem && alarmListItem.nanos < alarm.nanos) {
            lastItem = alarmListItem;
            alarmListItem = alarmListItem.next;
        }
        if (lastItem) {
            lastItem.next = alarm;
            alarm.next = alarmListItem;
        }
        else {
            this.nextAlarm = alarm;
            alarm.next = alarmListItem;
        }
        alarm.scheduled = true;
        return alarm;
    }
    unlinkAlarm(alarm) {
        let alarmListItem = this.nextAlarm;
        if (!alarmListItem) {
            return false;
        }
        let lastItem = null;
        while (alarmListItem) {
            if (alarmListItem === alarm) {
                if (lastItem) {
                    lastItem.next = alarmListItem.next;
                }
                else {
                    this.nextAlarm = alarmListItem.next;
                }
                return true;
            }
            lastItem = alarmListItem;
            alarmListItem = alarmListItem.next;
        }
        return false;
    }
    tick(deltaNanos) {
        const targetNanos = this.nanosCounter + deltaNanos;
        let alarm = this.nextAlarm;
        while (alarm && alarm.nanos <= targetNanos) {
            this.nextAlarm = alarm.next;
            this.nanosCounter = alarm.nanos;
            alarm.callback();
            alarm = this.nextAlarm;
        }
        this.nanosCounter = targetNanos;
    }
    get nanosToNextAlarm() {
        if (this.nextAlarm) {
            return this.nextAlarm.nanos - this.nanos;
        }
        return 0;
    }
}

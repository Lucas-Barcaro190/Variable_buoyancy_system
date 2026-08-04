"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.Simulator = void 0;
const simulation_clock_js_1 = require("./clock/simulation-clock.js");
const rp2040_js_1 = require("./rp2040.js");
class Simulator {
    constructor(clock = new simulation_clock_js_1.SimulationClock()) {
        this.clock = clock;
        this.executeTimer = null;
        this.stopped = true;
        this.rp2040 = new rp2040_js_1.RP2040(clock);
        this.rp2040.onBreak = () => this.stop();
    }
    execute() {
        const { rp2040, clock } = this;
        this.executeTimer = null;
        this.stopped = false;
        const cycleNanos = 1e9 / 125000000; // 125 MHz
        for (let i = 0; i < 1000000 && !this.stopped; i++) {
            if (rp2040.core.waiting) {
                const { nanosToNextAlarm } = clock;
                clock.tick(nanosToNextAlarm);
                i += nanosToNextAlarm / cycleNanos;
            }
            else {
                const cycles = rp2040.core.executeInstruction();
                clock.tick(cycles * cycleNanos);
            }
        }
        if (!this.stopped) {
            this.executeTimer = setTimeout(() => this.execute(), 0);
        }
    }
    stop() {
        this.stopped = true;
        if (this.executeTimer != null) {
            clearTimeout(this.executeTimer);
            this.executeTimer = null;
        }
    }
    get executing() {
        return !this.stopped;
    }
}
exports.Simulator = Simulator;

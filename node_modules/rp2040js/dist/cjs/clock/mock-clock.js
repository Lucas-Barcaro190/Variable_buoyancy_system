"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.MockClock = void 0;
const simulation_clock_js_1 = require("./simulation-clock.js");
class MockClock extends simulation_clock_js_1.SimulationClock {
    advance(deltaMicros) {
        this.tick(this.nanos + deltaMicros * 1000);
    }
}
exports.MockClock = MockClock;

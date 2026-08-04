import { SimulationClock } from './simulation-clock.js';
export class MockClock extends SimulationClock {
    advance(deltaMicros) {
        this.tick(this.nanos + deltaMicros * 1000);
    }
}

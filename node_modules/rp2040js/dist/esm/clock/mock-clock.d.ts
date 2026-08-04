import { SimulationClock } from './simulation-clock.js';
export declare class MockClock extends SimulationClock {
    advance(deltaMicros: number): void;
}

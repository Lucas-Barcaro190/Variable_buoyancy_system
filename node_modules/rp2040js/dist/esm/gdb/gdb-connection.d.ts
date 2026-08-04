import { GDBServer } from './gdb-server.js';
export type GDBResponseHandler = (value: string) => void;
export declare class GDBConnection {
    private server;
    private onResponse;
    readonly target: import("./gdb-target.js").IGDBTarget;
    private buf;
    constructor(server: GDBServer, onResponse: GDBResponseHandler);
    feedData(data: string): void;
    onBreakpoint(): void;
}

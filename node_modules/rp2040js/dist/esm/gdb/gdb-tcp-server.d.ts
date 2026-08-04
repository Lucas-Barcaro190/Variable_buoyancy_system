import { Socket } from 'net';
import { GDBServer } from './gdb-server.js';
import { IGDBTarget } from './gdb-target.js';
export declare class GDBTCPServer extends GDBServer {
    readonly port: number;
    private socketServer;
    constructor(target: IGDBTarget, port?: number);
    handleConnection(socket: Socket): void;
}

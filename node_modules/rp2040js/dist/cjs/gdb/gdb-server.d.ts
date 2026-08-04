/**
 * RP2040 GDB Server
 *
 * Copyright (C) 2021, Uri Shaked
 */
import { Logger } from '../utils/logging.js';
import { GDBConnection } from './gdb-connection.js';
import { IGDBTarget } from './gdb-target.js';
export declare const STOP_REPLY_SIGINT = "S02";
export declare const STOP_REPLY_TRAP = "S05";
export declare class GDBServer {
    readonly target: IGDBTarget;
    logger: Logger;
    private readonly connections;
    constructor(target: IGDBTarget);
    processGDBMessage(cmd: string): string | undefined;
    addConnection(connection: GDBConnection): void;
    removeConnection(connection: GDBConnection): void;
    debug(msg: string): void;
    info(msg: string): void;
    warn(msg: string): void;
    error(msg: string): void;
}

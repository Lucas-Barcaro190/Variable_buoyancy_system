import { RP2040 } from '../rp2040.js';
import { USBDevice } from '../usb/usb-device.js';
import { BasePeripheral } from './peripheral.js';
export declare class RPUSBController extends BasePeripheral {
    private addrEndp;
    private mainCtrl;
    private intRaw;
    private intEnable;
    private intForce;
    private sieStatus;
    private buffStatus;
    private sieCtrl;
    private sofFrameNumber;
    private devAddrCtrl;
    private intEpAddrCtrl;
    private intEpCtrl;
    private usbPwr;
    private nakPoll;
    private epAbort;
    private epAbortDone;
    private epStallArm;
    private epStatusStallNak;
    private hostMode;
    private sofEnabled;
    private connectedDevice;
    private pendingSetupResponse;
    private controlDataPid;
    private expectingStatusPhase;
    private readonly endpointReadAlarms;
    private readonly endpointWriteAlarms;
    private readonly resetAlarm;
    private readonly sofAlarm;
    private readonly hostTransactionAlarm;
    onUSBEnabled?: () => void;
    onResetReceived?: () => void;
    onEndpointWrite?: (endpoint: number, buffer: Uint8Array) => void;
    onEndpointRead?: (endpoint: number, byteCount: number) => void;
    readDelayMicroseconds: number;
    writeDelayMicroseconds: number;
    hostTransactionDelayMicroseconds: number;
    get intStatus(): number;
    constructor(rp2040: RP2040, name: string);
    readUint32(offset: number): number;
    writeUint32(offset: number, value: number): void;
    private readEndpointControlReg;
    private getEndpointBufferOffset;
    DPRAMUpdated(offset: number, value: number): void;
    endpointReadDone(endpoint: number, buffer: Uint8Array, delay?: number): void;
    private finishRead;
    private checkInterrupts;
    resetDevice(): void;
    sendSetupPacket(setupPacket: Uint8Array): void;
    private indicateBufferReady;
    private buffStatusUpdated;
    private sieStatusUpdated;
    /**
     * Connect a simulated USB device to the host controller.
     */
    connectDevice(device: USBDevice): void;
    /**
     * Disconnect the simulated USB device from the host controller.
     */
    disconnectDevice(): void;
    private onDeviceConnected;
    private handleSieCtrlWrite;
    private scheduleSofPacket;
    private generateSOF;
    private pollInterruptEndpoints;
    private startHostTransaction;
    private handleSetupTransaction;
    private handleInTransaction;
    private handleOutTransaction;
    private completeHostTransaction;
}

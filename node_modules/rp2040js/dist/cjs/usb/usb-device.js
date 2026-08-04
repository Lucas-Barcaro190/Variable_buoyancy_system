"use strict";
/**
 * Interface for simulated USB devices that can be connected to the RP2040 USB host controller.
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.USB_RECIP_OTHER = exports.USB_RECIP_ENDPOINT = exports.USB_RECIP_INTERFACE = exports.USB_RECIP_DEVICE = exports.USB_TYPE_VENDOR = exports.USB_TYPE_CLASS = exports.USB_TYPE_STANDARD = exports.USB_DIR_IN = exports.USB_DIR_OUT = void 0;
exports.parseSetupPacket = parseSetupPacket;
// USB Setup packet fields - bmRequestType bit masks
// Direction (bit 7)
exports.USB_DIR_OUT = 0x00;
exports.USB_DIR_IN = 0x80;
// Type (bits 6:5)
exports.USB_TYPE_STANDARD = 0x00;
exports.USB_TYPE_CLASS = 0x20;
exports.USB_TYPE_VENDOR = 0x40;
// Recipient (bits 4:0)
exports.USB_RECIP_DEVICE = 0x00;
exports.USB_RECIP_INTERFACE = 0x01;
exports.USB_RECIP_ENDPOINT = 0x02;
exports.USB_RECIP_OTHER = 0x03;
function parseSetupPacket(setup) {
    return {
        bmRequestType: setup[0],
        bRequest: setup[1],
        wValue: setup[2] | (setup[3] << 8),
        wIndex: setup[4] | (setup[5] << 8),
        wLength: setup[6] | (setup[7] << 8),
        // Helpers
        direction: setup[0] & 0x80 ? 'in' : 'out',
        type: (setup[0] >> 5) & 0x03, // 0=Standard, 1=Class, 2=Vendor
        recipient: setup[0] & 0x1f, // 0=Device, 1=Interface, 2=Endpoint, 3=Other
    };
}

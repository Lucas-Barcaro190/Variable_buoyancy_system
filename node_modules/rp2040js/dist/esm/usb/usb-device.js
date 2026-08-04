/**
 * Interface for simulated USB devices that can be connected to the RP2040 USB host controller.
 */
// USB Setup packet fields - bmRequestType bit masks
// Direction (bit 7)
export const USB_DIR_OUT = 0x00;
export const USB_DIR_IN = 0x80;
// Type (bits 6:5)
export const USB_TYPE_STANDARD = 0x00;
export const USB_TYPE_CLASS = 0x20;
export const USB_TYPE_VENDOR = 0x40;
// Recipient (bits 4:0)
export const USB_RECIP_DEVICE = 0x00;
export const USB_RECIP_INTERFACE = 0x01;
export const USB_RECIP_ENDPOINT = 0x02;
export const USB_RECIP_OTHER = 0x03;
export function parseSetupPacket(setup) {
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

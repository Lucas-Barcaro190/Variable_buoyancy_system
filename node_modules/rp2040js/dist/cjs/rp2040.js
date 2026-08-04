"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.RP2040 = exports.SIO_START_ADDRESS = exports.DPRAM_START_ADDRESS = exports.APB_START_ADDRESS = exports.RAM_START_ADDRESS = exports.FLASH_END_ADDRESS = exports.FLASH_START_ADDRESS = void 0;
const simulation_clock_js_1 = require("./clock/simulation-clock.js");
const cortex_m0_core_js_1 = require("./cortex-m0-core.js");
const gpio_pin_js_1 = require("./gpio-pin.js");
const irq_js_1 = require("./irq.js");
const adc_js_1 = require("./peripherals/adc.js");
const busctrl_js_1 = require("./peripherals/busctrl.js");
const clocks_js_1 = require("./peripherals/clocks.js");
const dma_js_1 = require("./peripherals/dma.js");
const i2c_js_1 = require("./peripherals/i2c.js");
const io_js_1 = require("./peripherals/io.js");
const pads_js_1 = require("./peripherals/pads.js");
const peripheral_js_1 = require("./peripherals/peripheral.js");
const pio_js_1 = require("./peripherals/pio.js");
const ppb_js_1 = require("./peripherals/ppb.js");
const psm_js_1 = require("./peripherals/psm.js");
const pwm_js_1 = require("./peripherals/pwm.js");
const reset_js_1 = require("./peripherals/reset.js");
const rtc_js_1 = require("./peripherals/rtc.js");
const spi_js_1 = require("./peripherals/spi.js");
const ssi_js_1 = require("./peripherals/ssi.js");
const syscfg_js_1 = require("./peripherals/syscfg.js");
const sysinfo_js_1 = require("./peripherals/sysinfo.js");
const tbman_js_1 = require("./peripherals/tbman.js");
const timer_js_1 = require("./peripherals/timer.js");
const uart_js_1 = require("./peripherals/uart.js");
const usb_js_1 = require("./peripherals/usb.js");
const watchdog_js_1 = require("./peripherals/watchdog.js");
const xosc_js_1 = require("./peripherals/xosc.js");
const sio_js_1 = require("./sio.js");
const logging_js_1 = require("./utils/logging.js");
exports.FLASH_START_ADDRESS = 0x10000000;
exports.FLASH_END_ADDRESS = 0x14000000;
exports.RAM_START_ADDRESS = 0x20000000;
exports.APB_START_ADDRESS = 0x40000000;
exports.DPRAM_START_ADDRESS = 0x50100000;
exports.SIO_START_ADDRESS = 0xd0000000;
const LOG_NAME = 'RP2040';
const KB = 1024;
const MB = 1024 * KB;
const MHz = 1000000;
class RP2040 {
    constructor(clock = new simulation_clock_js_1.SimulationClock()) {
        this.clock = clock;
        this.bootrom = new Uint32Array(4 * KB);
        this.sram = new Uint8Array(264 * KB);
        this.sramView = new DataView(this.sram.buffer);
        this.flash = new Uint8Array(16 * MB);
        this.flash16 = new Uint16Array(this.flash.buffer);
        this.flashView = new DataView(this.flash.buffer);
        this.usbDPRAM = new Uint8Array(4 * KB);
        this.usbDPRAMView = new DataView(this.usbDPRAM.buffer);
        this.core = new cortex_m0_core_js_1.CortexM0Core(this);
        /* Clocks */
        this.clkSys = 125 * MHz;
        this.clkPeri = 125 * MHz;
        this.ppb = new ppb_js_1.RPPPB(this, 'PPB');
        this.sio = new sio_js_1.RPSIO(this);
        this.uart = [
            new uart_js_1.RPUART(this, 'UART0', irq_js_1.IRQ.UART0, {
                rx: dma_js_1.DREQChannel.DREQ_UART0_RX,
                tx: dma_js_1.DREQChannel.DREQ_UART0_TX,
            }),
            new uart_js_1.RPUART(this, 'UART1', irq_js_1.IRQ.UART1, {
                rx: dma_js_1.DREQChannel.DREQ_UART1_RX,
                tx: dma_js_1.DREQChannel.DREQ_UART1_TX,
            }),
        ];
        this.i2c = [new i2c_js_1.RPI2C(this, 'I2C0', irq_js_1.IRQ.I2C0), new i2c_js_1.RPI2C(this, 'I2C1', irq_js_1.IRQ.I2C1)];
        this.pwm = new pwm_js_1.RPPWM(this, 'PWM_BASE');
        this.adc = new adc_js_1.RPADC(this, 'ADC');
        this.gpio = [
            new gpio_pin_js_1.GPIOPin(this, 0),
            new gpio_pin_js_1.GPIOPin(this, 1),
            new gpio_pin_js_1.GPIOPin(this, 2),
            new gpio_pin_js_1.GPIOPin(this, 3),
            new gpio_pin_js_1.GPIOPin(this, 4),
            new gpio_pin_js_1.GPIOPin(this, 5),
            new gpio_pin_js_1.GPIOPin(this, 6),
            new gpio_pin_js_1.GPIOPin(this, 7),
            new gpio_pin_js_1.GPIOPin(this, 8),
            new gpio_pin_js_1.GPIOPin(this, 9),
            new gpio_pin_js_1.GPIOPin(this, 10),
            new gpio_pin_js_1.GPIOPin(this, 11),
            new gpio_pin_js_1.GPIOPin(this, 12),
            new gpio_pin_js_1.GPIOPin(this, 13),
            new gpio_pin_js_1.GPIOPin(this, 14),
            new gpio_pin_js_1.GPIOPin(this, 15),
            new gpio_pin_js_1.GPIOPin(this, 16),
            new gpio_pin_js_1.GPIOPin(this, 17),
            new gpio_pin_js_1.GPIOPin(this, 18),
            new gpio_pin_js_1.GPIOPin(this, 19),
            new gpio_pin_js_1.GPIOPin(this, 20),
            new gpio_pin_js_1.GPIOPin(this, 21),
            new gpio_pin_js_1.GPIOPin(this, 22),
            new gpio_pin_js_1.GPIOPin(this, 23),
            new gpio_pin_js_1.GPIOPin(this, 24),
            new gpio_pin_js_1.GPIOPin(this, 25),
            new gpio_pin_js_1.GPIOPin(this, 26),
            new gpio_pin_js_1.GPIOPin(this, 27),
            new gpio_pin_js_1.GPIOPin(this, 28),
            new gpio_pin_js_1.GPIOPin(this, 29),
        ];
        this.qspi = [
            new gpio_pin_js_1.GPIOPin(this, 0, 'SCLK'),
            new gpio_pin_js_1.GPIOPin(this, 1, 'SS'),
            new gpio_pin_js_1.GPIOPin(this, 2, 'SD0'),
            new gpio_pin_js_1.GPIOPin(this, 3, 'SD1'),
            new gpio_pin_js_1.GPIOPin(this, 4, 'SD2'),
            new gpio_pin_js_1.GPIOPin(this, 5, 'SD3'),
        ];
        this.dma = new dma_js_1.RPDMA(this, 'DMA');
        this.pio = [
            new pio_js_1.RPPIO(this, 'PIO0', irq_js_1.IRQ.PIO0_IRQ0, 0),
            new pio_js_1.RPPIO(this, 'PIO1', irq_js_1.IRQ.PIO1_IRQ0, 1),
        ];
        this.usbCtrl = new usb_js_1.RPUSBController(this, 'USB');
        this.spi = [
            new spi_js_1.RPSPI(this, 'SPI0', irq_js_1.IRQ.SPI0, {
                rx: dma_js_1.DREQChannel.DREQ_SPI0_RX,
                tx: dma_js_1.DREQChannel.DREQ_SPI0_TX,
            }),
            new spi_js_1.RPSPI(this, 'SPI1', irq_js_1.IRQ.SPI1, {
                rx: dma_js_1.DREQChannel.DREQ_SPI1_RX,
                tx: dma_js_1.DREQChannel.DREQ_SPI1_TX,
            }),
        ];
        this.logger = new logging_js_1.ConsoleLogger(logging_js_1.LogLevel.Debug, true);
        this.peripherals = {
            0x18000: new ssi_js_1.RPSSI(this, 'SSI'),
            0x40000: new sysinfo_js_1.RP2040SysInfo(this, 'SYSINFO_BASE'),
            0x40004: new syscfg_js_1.RP2040SysCfg(this, 'SYSCFG'),
            0x40008: new clocks_js_1.RPClocks(this, 'CLOCKS_BASE'),
            0x4000c: new reset_js_1.RPReset(this, 'RESETS_BASE'),
            0x40010: new psm_js_1.RPPSM(this, 'PSM_BASE'),
            0x40014: new io_js_1.RPIO(this, 'IO_BANK0_BASE'),
            0x40018: new peripheral_js_1.UnimplementedPeripheral(this, 'IO_QSPI_BASE'),
            0x4001c: new pads_js_1.RPPADS(this, 'PADS_BANK0_BASE', 'bank0'),
            0x40020: new pads_js_1.RPPADS(this, 'PADS_QSPI_BASE', 'qspi'),
            0x40024: new xosc_js_1.RPXOSC(this, 'XOSC_BASE'),
            0x40028: new peripheral_js_1.UnimplementedPeripheral(this, 'PLL_SYS_BASE'),
            0x4002c: new peripheral_js_1.UnimplementedPeripheral(this, 'PLL_USB_BASE'),
            0x40030: new busctrl_js_1.RPBUSCTRL(this, 'BUSCTRL_BASE'),
            0x40034: this.uart[0],
            0x40038: this.uart[1],
            0x4003c: this.spi[0],
            0x40040: this.spi[1],
            0x40044: this.i2c[0],
            0x40048: this.i2c[1],
            0x4004c: this.adc,
            0x40050: this.pwm,
            0x40054: new timer_js_1.RPTimer(this, 'TIMER_BASE'),
            0x40058: new watchdog_js_1.RPWatchdog(this, 'WATCHDOG_BASE'),
            0x4005c: new rtc_js_1.RP2040RTC(this, 'RTC_BASE'),
            0x40060: new peripheral_js_1.UnimplementedPeripheral(this, 'ROSC_BASE'),
            0x40064: new peripheral_js_1.UnimplementedPeripheral(this, 'VREG_AND_CHIP_RESET_BASE'),
            0x4006c: new tbman_js_1.RPTBMAN(this, 'TBMAN_BASE'),
            0x50000: this.dma,
            0x50110: this.usbCtrl,
            0x50200: this.pio[0],
            0x50300: this.pio[1],
        };
        // Debugging
        // eslint-disable-next-line @typescript-eslint/no-unused-vars
        this.onBreak = (code) => {
            // TODO: raise HardFault exception
            // console.error('Breakpoint!', code);
        };
        this.reset();
    }
    loadBootrom(bootromData) {
        this.bootrom.set(bootromData);
        this.reset();
    }
    reset() {
        this.core.reset();
        this.pwm.reset();
        this.flash.fill(0xff);
    }
    readUint32(address) {
        address = address >>> 0; // round to 32-bits, unsigned
        if (address & 0x3) {
            this.logger.error(LOG_NAME, `read from address ${address.toString(16)}, which is not 32 bit aligned`);
        }
        const { bootrom } = this;
        if (address < bootrom.length * 4) {
            return bootrom[address / 4];
        }
        else if (address >= exports.FLASH_START_ADDRESS && address < exports.FLASH_END_ADDRESS) {
            // Flash is mirrored four times:
            // - 0x10000000 XIP
            // - 0x11000000 XIP_NOALLOC
            // - 0x12000000 XIP_NOCACHE
            // - 0x13000000 XIP_NOCACHE_NOALLOC
            const offset = address & 16777215;
            return this.flashView.getUint32(offset, true);
        }
        else if (address >= exports.RAM_START_ADDRESS && address < exports.RAM_START_ADDRESS + this.sram.length) {
            return this.sramView.getUint32(address - exports.RAM_START_ADDRESS, true);
        }
        else if (address >= exports.DPRAM_START_ADDRESS &&
            address < exports.DPRAM_START_ADDRESS + this.usbDPRAM.length) {
            return this.usbDPRAMView.getUint32(address - exports.DPRAM_START_ADDRESS, true);
        }
        else if (address >>> 12 === 0xe000e) {
            return this.ppb.readUint32(address & 0xfff);
        }
        else if (address >= exports.SIO_START_ADDRESS && address < exports.SIO_START_ADDRESS + 0x10000000) {
            return this.sio.readUint32(address - exports.SIO_START_ADDRESS);
        }
        const peripheral = this.findPeripheral(address);
        if (peripheral) {
            return peripheral.readUint32(address & 0x3fff);
        }
        this.logger.warn(LOG_NAME, `Read from invalid memory address: ${address.toString(16)}`);
        return 0xffffffff;
    }
    findPeripheral(address) {
        return this.peripherals[(address >>> 14) << 2];
    }
    /** We assume the address is 16-bit aligned */
    readUint16(address) {
        if (address >= exports.FLASH_START_ADDRESS && address < exports.FLASH_START_ADDRESS + this.flash.length) {
            return this.flashView.getUint16(address - exports.FLASH_START_ADDRESS, true);
        }
        else if (address >= exports.RAM_START_ADDRESS && address < exports.RAM_START_ADDRESS + this.sram.length) {
            return this.sramView.getUint16(address - exports.RAM_START_ADDRESS, true);
        }
        const value = this.readUint32(address & 0xfffffffc);
        return address & 0x2 ? (value & 0xffff0000) >>> 16 : value & 0xffff;
    }
    readUint8(address) {
        if (address >= exports.FLASH_START_ADDRESS && address < exports.FLASH_START_ADDRESS + this.flash.length) {
            return this.flash[address - exports.FLASH_START_ADDRESS];
        }
        else if (address >= exports.RAM_START_ADDRESS && address < exports.RAM_START_ADDRESS + this.sram.length) {
            return this.sram[address - exports.RAM_START_ADDRESS];
        }
        const value = this.readUint16(address & 0xfffffffe);
        return (address & 0x1 ? (value & 0xff00) >>> 8 : value & 0xff) >>> 0;
    }
    writeUint32(address, value) {
        address = address >>> 0;
        const { bootrom } = this;
        const peripheral = this.findPeripheral(address);
        if (peripheral) {
            const atomicType = (address & 0x3000) >> 12;
            const offset = address & 0xfff;
            peripheral.writeUint32Atomic(offset, value, atomicType);
        }
        else if (address < bootrom.length * 4) {
            bootrom[address / 4] = value;
        }
        else if (address >= exports.FLASH_START_ADDRESS &&
            address < exports.FLASH_START_ADDRESS + this.flash.length) {
            this.flashView.setUint32(address - exports.FLASH_START_ADDRESS, value, true);
        }
        else if (address >= exports.RAM_START_ADDRESS && address < exports.RAM_START_ADDRESS + this.sram.length) {
            this.sramView.setUint32(address - exports.RAM_START_ADDRESS, value, true);
        }
        else if (address >= exports.DPRAM_START_ADDRESS &&
            address < exports.DPRAM_START_ADDRESS + this.usbDPRAM.length) {
            const offset = address - exports.DPRAM_START_ADDRESS;
            this.usbDPRAMView.setUint32(offset, value, true);
            this.usbCtrl.DPRAMUpdated(offset, value);
        }
        else if (address >= exports.SIO_START_ADDRESS && address < exports.SIO_START_ADDRESS + 0x10000000) {
            this.sio.writeUint32(address - exports.SIO_START_ADDRESS, value);
        }
        else if (address >>> 12 === 0xe000e) {
            this.ppb.writeUint32(address & 0xfff, value);
        }
        else {
            this.logger.warn(LOG_NAME, `Write to undefined address: ${address.toString(16)}`);
        }
    }
    writeUint8(address, value) {
        if (address >= exports.RAM_START_ADDRESS && address < exports.RAM_START_ADDRESS + this.sram.length) {
            this.sram[address - exports.RAM_START_ADDRESS] = value;
            return;
        }
        const alignedAddress = (address & 0xfffffffc) >>> 0;
        const offset = address & 0x3;
        const peripheral = this.findPeripheral(address);
        if (peripheral) {
            const atomicType = (alignedAddress & 0x3000) >> 12;
            const offset = alignedAddress & 0xfff;
            peripheral.writeUint32Atomic(offset, (value & 0xff) | ((value & 0xff) << 8) | ((value & 0xff) << 16) | ((value & 0xff) << 24), atomicType);
            return;
        }
        const originalValue = this.readUint32(alignedAddress);
        const newValue = new Uint32Array([originalValue]);
        new DataView(newValue.buffer).setUint8(offset, value);
        this.writeUint32(alignedAddress, newValue[0]);
    }
    writeUint16(address, value) {
        // we assume that addess is 16-bit aligned.
        // Ideally we should generate a fault if not!
        if (address >= exports.RAM_START_ADDRESS && address < exports.RAM_START_ADDRESS + this.sram.length) {
            this.sramView.setUint16(address - exports.RAM_START_ADDRESS, value, true);
            return;
        }
        const alignedAddress = (address & 0xfffffffc) >>> 0;
        const offset = address & 0x3;
        const peripheral = this.findPeripheral(address);
        if (peripheral) {
            const atomicType = (alignedAddress & 0x3000) >> 12;
            const offset = alignedAddress & 0xfff;
            peripheral.writeUint32Atomic(offset, (value & 0xffff) | ((value & 0xffff) << 16), atomicType);
            return;
        }
        const originalValue = this.readUint32(alignedAddress);
        const newValue = new Uint32Array([originalValue]);
        new DataView(newValue.buffer).setUint16(offset, value, true);
        this.writeUint32(alignedAddress, newValue[0]);
    }
    get gpioValues() {
        const { gpio } = this;
        let result = 0;
        for (let gpioIndex = 0; gpioIndex < gpio.length; gpioIndex++) {
            if (gpio[gpioIndex].inputValue) {
                result |= 1 << gpioIndex;
            }
        }
        return result;
    }
    setInterrupt(irq, value) {
        this.core.setInterrupt(irq, value);
    }
    updateIOInterrupt() {
        let interruptValue = false;
        for (const pin of this.gpio) {
            if (pin.irqValue) {
                interruptValue = true;
            }
        }
        this.setInterrupt(irq_js_1.IRQ.IO_BANK0, interruptValue);
    }
    step() {
        this.core.executeInstruction();
    }
}
exports.RP2040 = RP2040;

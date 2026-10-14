# LPC1769 UsbEp0Demo

Import `ioc` and `../../lib/ioc` into IOcomposer. Build `IOsonata_LPC1769`,
then `UsbEp0Demo` using the same configuration. Debug defaults to semihosting;
enable semihosting in the debugger. Release uses nosys without semihosting.
The application has no console output. It uses the generic USB core and
`AppRun` with a 16-entry application event queue.

## Board setup

Edit `src/board.h` to match the board oscillator. The supplied configuration
assumes a 16 MHz external crystal and a 32.768 kHz RTC crystal. The system
clock must provide a valid 48 MHz USB clock; this controller does not retune
an incompatible clock. The USB pin assignments are fixed:

| Signal | Pin | Function |
| --- | --- | --- |
| VBUS sense | P1.30 | FUNC2 input |
| D+ | P0.29 | FUNC1 |
| D- | P0.30 | FUNC1 |
| SoftConnect | P2.9 | FUNC1, active low |

SoftConnect must drive the board's external D+ pull-up switching circuit.
Verify the board schematic before connecting. USB VBUS must be present when
the application starts. Automatic attachment after startup and reattachment
after unplugging are not implemented; reset the MCU with the cable attached.
The linker places this standalone application at flash address zero.

## Host check

The device presents one vendor interface with no data endpoints. Its default
serial is the 32-digit ROM unique ID. VID/PID 1209:0005 are the existing test
values used by the link fixture; select appropriate IDs before distribution.
With PyUSB and a libusb implementation available, run:

```python
import usb.core

for device in usb.core.find(find_all=True, idVendor=0x1209, idProduct=0x0005):
    if device.product != "LPC1769 EP0 Demo":
        continue
    device.set_configuration()
    print("Serial:", device.serial_number)
    for _ in range(100):
        reply = bytes(device.ctrl_transfer(0xC0, 1, 0, 0, 8, timeout=1000))
        assert reply == b"LPC1769\x00", reply
    print("100 EP0 identity requests passed")
    break
else:
    raise RuntimeError("LPC1769 EP0 Demo was not found")
```

This checks control requests only. CDC, HID and bulk data transfers are not
implemented by the LPC1769 driver yet. SOF, remote wake and low-power suspend
are also unsupported. There is no Windows vendor-interface driver supplied.

## Validation

`python3 tests/lpc/usb/link17.py --prefix arm-none-eabi-` builds this application
against the real MCU library sources in Debug and Release. It checks project
source links and semihosting options, vector checksum, USB IRQ vector and ROM
IAP linkage. GNU Arm 13.2.1 links pass: text/data/bss are 35152/476/2680 bytes
in Debug and 15296/88/2176 bytes in Release. Physical-board enumeration and the
host check above have not been run.

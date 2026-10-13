# LPC11U35 UsbCustomBulkLoopback

Import `ioc` and `../../lib/ioc` into IOcomposer/Eclipse. Build the LPC11U35
library first, then UsbCustomBulkLoopback, with matching configurations.
The project links the shared `exemples/usb/usb_custom_bulk_loopback.cpp`.
It returns received bytes through a vendor-specific USB bulk interface using
the generic UsbdBulk class and native LPC11U35 controller.

Debug uses -O0, newlib-nano and rdimon semihosting. Enable semihosting in the
IDAP-Link debugger when running Debug. Release uses -Os and nosys without
semihosting. Both use C++23, no exceptions and no RTTI.

## Board

Use J1 for USB traffic and the separate SWD connection for IDAP-Link. The
board configuration matches UsbCdcLoopback: a 12 MHz external crystal, USB
clock enabled, VBUS sense on P0.3 FUNC1, and soft-connect control on P0.6
FUNC1. The native driver configures USB_CONNECT.

The carrier schematic does not show the module's oscillator or internal
VBUS/soft-connect wiring. See `../UsbCdcLoopback/README.md` for the carrier
connections and required module wiring. Hardware testing remains pending.

## Host check

The existing host script needs PyUSB and a working libusb installation.
From the repository root, after programming and connecting J1:

```
python3 -m pip install pyusb
python3 Python/usb_custom_bulk_loopback.py --cycles 100
```

The script selects VID 0x1209, PID 0x0002 and product string
`IOsonata Custom Bulk Loopback`. It discovers the vendor interface and bulk
endpoints from descriptors, then compares echoed payloads of 1, 7, 63, 64,
65, 127, 128, 129, 512 and 1024 bytes. Use `--serial` to select one device
when several match. These identifiers are the shared demo values.

Repeat with `--reconfigure` to exercise SET_CONFIGURATION again. Unplug/replug
J1 and rerun the test to check enumeration and loopback. Use independent board
power if testing detach/attach without an MCU reset. This is a vendor USB
interface; it does not create a serial port.

## Build validation

`python3 tests/lpc/usb/link.py` checks the project links, board header, C++23,
Debug/Release semihosting settings, real USB IRQ, vector checksum, IAP RAM
reservation and USB SRAM placement. It builds the shared application against
a library archive and the production linker script, not every optional source
in the IOcomposer library project.

Physical enumeration and throughput have not been tested on the replacement
board. Host models separately cover vendor-bulk transfers and reconnect in a
CDC/HID/vendor-bulk composite device.

Measured with Arm GNU GCC 13.2.1 and binutils 2.42:

| Build | Flash / 65536 bytes | Main RAM / 8160 bytes | USB SRAM / 2048 bytes | Linker stack allowance |
| --- | ---: | ---: | ---: | ---: |
| Debug | 47580 | 4448 | 832 | 1236 |
| Release | 20964 | 3652 | 832 | 1500 |

The stack allowance is not a measured runtime high-water mark.

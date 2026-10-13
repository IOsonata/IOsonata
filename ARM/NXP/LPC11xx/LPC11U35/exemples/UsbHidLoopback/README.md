# LPC11U35 UsbHidLoopback

Import `ioc` and `../../lib/ioc` into IOcomposer/Eclipse. Build the LPC11U35
library first, then UsbHidLoopback, with matching configurations. The project
links the shared `exemples/usb/usb_hid_loopback.cpp` application and uses the
native LPC11U35 controller with the generic HID class and interrupt transport.
It echoes 64-byte vendor HID output reports as input reports.

Debug uses -O0, newlib-nano and rdimon semihosting. Enable semihosting in the
IDAP-Link debugger when running Debug. Release uses -Os and nosys without
semihosting. Both use C++23, no exceptions and no RTTI.

## Board

Use J1 for USB data and the separate SWD connection for IDAP-Link. The board
configuration matches UsbCdcLoopback: a 12 MHz external crystal, USB clock
enabled, VBUS sense on P0.3 FUNC1, and soft-connect control on P0.6 FUNC1.
The driver configures USB_CONNECT; only VBUS is listed in USB_PINS.

The supplied carrier schematic does not show the module's oscillator or
internal VBUS/soft-connect wiring. See `../UsbCdcLoopback/README.md` for the
carrier connections and module requirements. Hardware validation remains
pending while the board is being replaced.

## Host check

From the repository root, after programming and connecting J1:

```
python3 -m pip install hidapi
python3 Python/usb_hid_loopback.py --rounds 1000
```

The script selects VID 0x1209, PID 0x0005 and product name
`IOsonata USB HID Loopback`. It compares each returned 64-byte report with
what was sent. Use `--serial` if multiple matching devices are attached.
The VID/PID are the shared demo values, not a product allocation.

Unplug/replug J1 and rerun the script to check enumeration and report traffic.
Use independent board power to exercise detach/attach without an MCU reset.
The optional `--manual-suspend-wake` phase checks host suspend/resume; this
example does not enable device-initiated remote wake or low-power suspend.

## Build validation

`python3 tests/lpc/usb/link.py` builds both CDC and HID examples in Debug and
Release, using the production LPC11U35 linker script. It checks project source
links, C++23, semihosting selection, vector checksum, real USB interrupt handler,
IAP RAM reservation and USB SRAM placement. This is not a full IOcomposer
library build or a physical USB test.

Measured with Arm GNU GCC 13.2.1 and binutils 2.42:

| Build | Flash / 65536 bytes | Main RAM / 8160 bytes | USB SRAM / 2048 bytes | Linker stack allowance |
| --- | ---: | ---: | ---: | ---: |
| Debug | 50976 | 3420 | 832 | 1576 |
| Release | 22184 | 2624 | 832 | 1844 |

The stack allowance is not a measured runtime high-water mark. Host-model
CDC/HID tests cover descriptor routing, reports and reconnect; hardware
report traffic and throughput remain untested.

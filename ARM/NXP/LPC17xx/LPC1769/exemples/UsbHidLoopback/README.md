# LPC1769 UsbHidLoopback

Import `ioc` and `../../lib/ioc` into IOcomposer. Build `IOsonata_LPC1769`
first, then `UsbHidLoopback`, with matching configurations. Debug defaults to
rdimon semihosting; enable it in IDAP-Link's debugger. Release uses nosys.

The project links the shared `exemples/usb/usb_hid_loopback.cpp` application.
It uses the generic `UsbdHid` class, interrupt transport, `AppRun` and a
16-entry event queue. A single vendor-page HID interface echoes 64-byte output
reports as input reports using LPC1769 interrupt OUT/IN EP1. The application
supplies the report descriptor and GET_REPORT/SET_REPORT handling.

## Board setup

The board configuration matches `../UsbCdcLoopback/src/board.h`: a 16 MHz
external crystal and 32.768 kHz RTC crystal, VBUS P1.30 FUNC2, D+ P0.29 FUNC1,
D- P0.30 FUNC1 and active-low SoftConnect P2.9 FUNC1. SoftConnect must drive
the external D+ pull-up switch. Verify these assumptions against the board.
The system must supply a valid 48 MHz USB clock.

VBUS must be present at startup. Automatic attach and reattach after unplugging
are not implemented; reset with USB attached. Remote wake and low-power suspend
are disabled. This example uses one HID interface; arbitrary composite endpoint
allocation is not supported by the generic allocator on fixed-type LPC1769
endpoints.

## Host check

After programming and resetting with the USB cable connected, run from the
repository root:

```sh
python3 -m pip install hidapi
python3 Python/usb_hid_loopback.py --rounds 1000
```

The host script selects VID 1209, PID 0005 and product `IOsonata USB HID Loopback`.
Use `--serial` if multiple devices match. These are existing demo IDs; select
appropriate IDs before product distribution. The serial comes from the ROM UID.
The script sends one report and checks its echo before sending the next.

## Validation

```sh
python3 tests/lpc/usb/link17.py --prefix arm-none-eabi-
```

This builds the actual EP0, CDC and HID examples in Debug and Release with the
production linker script. It verifies project source links, semihosting,
vector checksum and extraction of the HID class, interrupt transport and USB
IRQ. These command-line builds do not replace an IOcomposer or hardware test.
Physical enumeration and report transfers remain pending.

GNU Arm 13.2.1 text/data/bss: Debug 53480/456/4552 bytes;
Release 23816/152/4064 bytes. Both links pass.

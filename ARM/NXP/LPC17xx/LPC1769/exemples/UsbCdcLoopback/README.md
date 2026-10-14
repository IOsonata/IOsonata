# LPC1769 UsbCdcLoopback

Import `ioc` and `../../lib/ioc` into IOcomposer. Build `IOsonata_LPC1769`
first, then `UsbCdcLoopback`, using matching configurations. Debug defaults to
rdimon semihosting; enable it in the debugger. Release uses nosys.

The project links the shared `exemples/usb/usb_cdc_loopback.cpp` application.
It uses the generic `UsbdCdc` class with one CDC ACM port, a 16-entry
application event queue and `AppRun`. `AppCheckStatus` echoes received bytes
using `Rx`/`Tx`, retaining partial writes. FIFO `bBlocking = true` prevents
overwriting unread data; it does not make `Tx` wait for USB hardware. The shared
example sends its greeting when the host asserts DTR. The application does not call controller APIs.
The single CDC instance uses interrupt IN EP1 and bulk IN/OUT EP2. The generic
allocator now respects LPC1769 fixed endpoint types. See UsbCompositeLoopback
for the integrated CDC/HID/vendor-bulk application.

## Board setup

`src/board.h` assumes a 16 MHz external crystal and a 32.768 kHz RTC crystal.
Match these to the board; the system must provide a valid 48 MHz USB clock.

| Signal | Pin | Function |
| --- | --- | --- |
| VBUS sense | P1.30 | FUNC2 input |
| D+ | P0.29 | FUNC1 |
| D- | P0.30 | FUNC1 |
| SoftConnect | P2.9 | FUNC1, active low |

SoftConnect requires the board's external D+ pull-up switching circuit.
USB VBUS must be present at startup. Automatic attach after startup and
reattachment after unplugging are not implemented; reset with USB attached.
Remote wake and low-power suspend are disabled. Board wiring is unverified.
VID/PID 1209:0001 are existing demo values; use appropriate IDs for distribution.
The product string is `IOsonata CDC Loopback`; the serial comes from the ROM UID.

## Host check

Open the USB serial port with DTR asserted and terminal local echo disabled.
Typed bytes should return unchanged. Baud selection does not set a UART speed.
For a binary packet-boundary check, install pyserial and replace the port below:

```python
import serial

with serial.Serial("/dev/cu.usbmodemXXXX", 115200,
                   timeout=2, write_timeout=2) as port:
    port.dtr = True
    greeting = b"\r\nIOsonata USB CDC Loopback\r\n"
    assert port.read(len(greeting)) == greeting
    for length in (1, 7, 8, 9, 63, 64, 65, 127, 128, 129, 1024):
        payload = bytes((i * 37 + length) & 255 for i in range(length))
        assert port.write(payload) == length
        reply = port.read(length)
        assert reply == payload, (length, len(reply))
    print("CDC binary loopback passed")
```

Repeat after closing/reopening the serial port. Enumeration, CDC line requests,
interrupt notifications and sustained traffic still require physical-board tests.

## Build validation

From the repository root:

```sh
python3 tests/lpc/usb/link17.py --prefix arm-none-eabi-
```

The script links the actual EP0 and CDC examples against the MCU library
sources in Debug and Release. It verifies IOC source links, semihosting,
vector checksum, the USB IRQ and extraction of the CDC data path. This is a
command-line firmware build, not a full IOcomposer build or hardware test.

GNU Arm 13.2.1 text/data/bss: Debug 52212/456/5680 bytes;
Release 23348/152/5192 bytes. Both firmware links pass. Hardware testing is pending.

# LPC54628 USB1 high-speed custom bulk loopback

Import `ioc` and the LPC54628 IOsonata library into IOcomposer. Build the
library and application with matching configurations. The project links the
shared `exemples/usb/usb_custom_bulk_loopback.cpp` application. It echoes bytes
through a vendor-specific bulk OUT/IN pair on USB1, using UsbdBulk and the
16-entry AppEvt queue. It does not create a serial port.

Debug uses semihosting; enable it in the IDAP-Link debugger. Release uses nosys
without semihosting. Both configurations use C++23, no exceptions and no RTTI.

`src/board.h` selects USB1 (`USB_DEVNO 1`), the high-speed controller, and
the 12 MHz crystal of the LPCXpresso54628 as the core oscillator (`MCUOSC`):
the USB PLL makes the 48 MHz USB1 clock from it. Use the USB1 high-speed
connector. Its VBUS reaches the USB1_VBUS pin on the board, which the PHY needs
to connect its pull-up. No wiring is needed: board.h gives no `USB_PINS`, so
USB1 runs from `UsbEnable` with the PLL and the PHY on and reads VBUS from the
controller. The first bus reset of the host is the attachment; a charger alone
is not reported. To keep USB1 off until a cable is attached, define `USB_PINS`
with a GPIO input wired to the VBUS of the USB1 connector through a divider to
about 3.3 V, for example 10 kOhm from VBUS and 20 kOhm to ground; board.h shows
it on P0_16 (Arduino A0). USB0 and the USB host controllers must remain unused.

The bus runs at high speed (512-byte bulk packets) on a high-speed host or hub
and at full speed (64-byte packets) behind a full-speed one. The port handles
the errata USB.2, USB.6 and USB.15 of ES_LPC546XX; USB.6 adds about 0.6 s to
an attachment that starts at full speed. See `ARM/NXP/README.md` for the USB1
notes.

After programming and connecting USB1, run from the repository root with
PyUSB and a working libusb installation:

```sh
python3 -m pip install pyusb
python3 Python/usb_custom_bulk_loopback.py --cycles 100
python3 Python/usb_custom_bulk_loopback.py --cycles 100 --reconfigure
```

The script selects VID 0x1209, PID 0x0002 and product name
`IOsonata Custom Bulk Loopback`. It compares echoed payloads of 1 to 1024
bytes; at high speed 512 and 1024 bytes are one and two full packets. Close
other programs that use the device first.

`python3 tests/lpc/usb/link546.py` checks the project links and builds this
application in Debug and Release against the production linker script. It
checks the USB0 and USB1 vectors, checksum, memory limits, semihosting
selection and the crystal setting of `g_McuOsc`. This is not a full
IOcomposer library build. Physical enumeration at high speed, transfers,
throughput and reconnect testing remain pending board testing.

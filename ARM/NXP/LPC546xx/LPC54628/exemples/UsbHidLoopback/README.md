# LPC54628 USB HID loopback

Import `ioc` and the LPC54628 IOsonata library into IOcomposer. Build the
library and application with matching configurations. This project links the
shared `exemples/usb/usb_hid_loopback.cpp` application. It echoes 64-byte vendor
HID output reports as input reports through the generic HID class and interrupt
transport, using the 16-entry AppEvt queue.

Debug uses semihosting; enable it in the IDAP-Link debugger. Release uses nosys
without semihosting. Both configurations use C++23, no exceptions and no RTTI.

Use USB0 full-speed DM/DP on the dedicated pads, with VBUS on P0.22 FUNC7.
Check this routing against your board schematic. The driver starts the FRO
clock and controls the PHY's internal pull-up. USB1 and the USB host
controllers must remain unused. See `../UsbCdcLoopback/README.md` for port limits.
This HID application disables remote wakeup and low-power suspend.

After programming and connecting USB0, run from the repository root:

```sh
python3 -m pip install hidapi
python3 Python/usb_hid_loopback.py --rounds 1000
```

The script selects VID 0x1209, PID 0x0005 and product name
`IOsonata USB HID Loopback`. Use `--serial` when several matching devices are
connected. These are shared demo identifiers, not a product allocation.
Unplug/replug and repeat; independent board power allows cable detach/attach
without resetting the MCU. `--manual-suspend-wake` checks host suspend/resume.

`python3 tests/lpc/usb/link546.py` checks the project links and builds the HID
application in Debug and Release against the production linker script. It
checks the USB0 vector, vector checksum, semihosting selection and USB SRAM.
This is not a full IOcomposer library build. Physical enumeration, HID report
traffic and throughput remain pending board testing.

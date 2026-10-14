# LPC54628 USB custom bulk loopback

Import `ioc` and the LPC54628 IOsonata library into IOcomposer. Build the
library and application with matching configurations. The project links the
shared `exemples/usb/usb_custom_bulk_loopback.cpp` application. It echoes bytes
through a vendor-specific bulk OUT/IN pair using UsbdBulk and the 16-entry
AppEvt queue. It does not create a serial port.

Debug uses semihosting; enable it in the IDAP-Link debugger. Release uses nosys
without semihosting. Both configurations use C++23, no exceptions and no RTTI.

Use USB0 full-speed DM/DP on the dedicated pads, with VBUS on P0.22 FUNC7.
Check this routing against your board schematic. The driver starts the FRO
clock and controls the PHY's internal pull-up. USB1 and the USB host
controllers must remain unused. See `../UsbCdcLoopback/README.md` for port limits.
This application disables remote wakeup and low-power suspend.

After programming and connecting USB0, run from the repository root with
PyUSB and a working libusb installation:

```sh
python3 -m pip install pyusb
python3 Python/usb_custom_bulk_loopback.py --cycles 100
python3 Python/usb_custom_bulk_loopback.py --cycles 100 --reconfigure
```

The script selects VID 0x1209, PID 0x0002 and product name
`IOsonata Custom Bulk Loopback`. It discovers the vendor interface and endpoints,
then compares echoed payloads of 1, 7, 63, 64, 65, 127, 128, 129, 512 and 1024
bytes. Use `--serial` when several matching devices are connected. These are
shared demo identifiers, not a product allocation. The second command forces
SET_CONFIGURATION before transferring data. Unplug/replug and repeat;
independent board power allows cable detach/attach without resetting the MCU.

`python3 tests/lpc/usb/link546.py` checks project links and builds this application
in Debug and Release against the production linker script. It checks the USB0
vector, vector checksum, semihosting selection and USB SRAM. This is not a full
IOcomposer library build. Physical enumeration, transfers and throughput remain
pending board testing.

The shared application uses AppRun and AppCheckStatus with the 16-entry event
queue. It checks USB status while idle, retains partial TX writes under FIFO
backpressure, and waits only when no application data or queued event remains.

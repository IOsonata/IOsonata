# LPC54628 USB CDC loopback

Import `ioc` and the LPC54628 IOsonata library into IOcomposer. Build the
library and application in the same configuration. Debug uses semihosting;
Release does not. The project links the shared `usb_cdc_loopback.cpp` example.

Use the USB0 full-speed connector, with DM/DP on the dedicated USB0 pads and
VBUS routed to P0.22 (FUNC7). Confirm this routing against your board schematic.
The driver starts its FRO clock and uses the internal PHY pull-up. USB1 and the
USB host controllers must remain unused. No board register writes are needed.

The generic USB core handles enumeration and the CDC class. The application
runs its 16-entry AppEvt queue and echoes serial data. For example:

```sh
python3 Python/uartprbs_lb.py --baud 1000000 --port /dev/cu.usbmodemYOUR_DEVICE
```

CDC baud rate is host metadata; it does not set USB bus speed. This port supports
full-speed bulk and interrupt transfers, plus remote wakeup with clocks kept
on during suspend. The board header advertises remote wakeup; the host must
enable it before an application can request it through `UsbRemoteWakeup(0)`.
The loopback application does not generate wake requests on its own. ISO and
low-power suspend are not implemented. USB1, the high-speed controller, is
DevNo 1 of the same port (see `ARM/NXP/README.md`).

Host register-model tests and ARM Debug/Release links have been checked.
Physical enumeration, cable behavior and throughput still need board testing.

# LPC54628 USB CDC PRBS transmit test

Import `ioc` and the LPC54628 IOsonata library into IOcomposer. Build the
library and application with matching configurations. This project links the
shared `exemples/usb/usb_cdc_prbs_tx.cpp` application. It sends a PRBS stream
through CDC using a 2048-byte TX FIFO and a 16-entry AppEvt queue.

Debug uses semihosting; enable it in the IDAP-Link debugger. Release uses nosys
without semihosting. Both configurations use C++23, no exceptions and no RTTI.

Use USB0 full-speed DM/DP on the dedicated pads, with VBUS on P0.22 FUNC7.
Check this routing against your board schematic. The driver starts the FRO
clock and controls the PHY's internal pull-up. USB1 and the USB host
controllers must remain unused. See `../UsbCdcLoopback/README.md` for port limits.
The shared application advertises remote wakeup but does not request it;
low-power suspend is disabled.

From the repository root, with pyserial installed:

```sh
python3 Python/uartprbs_rx.py --baud 1000000 --port /dev/cu.usbmodemDEVICE
```

Replace the port with the enumerated CDC port. The receiver asserts DTR by
default, opening the data stream. The baud value is CDC line coding, not the
USB transfer rate. Check the bytes per second and PRBS error counters during
a sustained run. Stop with Ctrl-C. Close/reopen and unplug/reconnect as
separate runs so each receiver run establishes a fresh sequence baseline.

The application uses the shared demonstration VID/PID and product string
`IOsonata CDC PRBS Tx`. These identifiers are not a product allocation.

`python3 tests/lpc/usb/link546.py` checks the project links and builds this
application in Debug and Release against the production linker script. It
checks the USB0 vector, checksum, memory limits and semihosting selection.
This is not a full IOcomposer library build. Physical enumeration, PRBS
integrity, throughput and reconnect testing remain pending board testing.

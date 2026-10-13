# LPC11U35 USB CDC PRBS transmit test

Build the matching Debug or Release IOsonata_LPC11U35 library, then import
`ioc` into IOcomposer. The project links `exemples/usb/usb_cdc_prbs_tx.cpp`
and streams PRBS data through the native USB device driver.

Connect the host to J1 on the IBK-IDAP-M carrier for USB traffic. Use the
separate SWD connector with IDAP-Link for programming and debugging.
The board configuration matches UsbCdcLoopback: a 12 MHz external crystal,
VBUS sensing on P0.3 FUNC1 and USB_CONNECT on P0.6 FUNC1. The carrier
schematic does not verify the module's internal clock or soft-connect
circuit; see [UsbCdcLoopback](../UsbCdcLoopback/README.md) for the wiring
and module requirements.

Debug uses semihosting; enable it in the debugger before running. Release
uses nosys without semihosting. Both configurations use the production
LPC11U35 linker script, including the dedicated USB SRAM section.

From the repository root, with pyserial installed:

```
python3 Python/uartprbs_rx.py --baud 1000000 --port /dev/cu.usbmodemDEVICE
```

Replace the port with the enumerated CDC port. The receiver asserts DTR by
default, which opens the CDC data stream. The baud setting is CDC line coding
and does not set the USB transfer rate. Check the reported bytes per second
and PRBS error counters over a sustained run. Stop with Ctrl-C.

Test closing and reopening the port and unplugging/reconnecting USB as
separate runs. A new run establishes a fresh PRBS sequence baseline. The
application uses a 2048-byte TX FIFO and a 16-entry application event queue;
it services deferred USB work while waiting for port open or FIFO space.

The shared example retains its demonstration VID/PID and strings. Allocate
an appropriate VID/PID before distributing firmware as a product.

Run `python3 tests/lpc/usb/link.py` to compile and link the USB examples in
Debug and Release and check their memory limits, vector checksum, USB IRQ
and semihosting selection. Physical enumeration, PRBS integrity, throughput
and reconnect testing remain pending while the board is being replaced.

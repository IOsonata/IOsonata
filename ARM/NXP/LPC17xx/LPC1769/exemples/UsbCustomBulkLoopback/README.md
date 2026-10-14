# LPC1769 custom bulk loopback

Import `ioc` and the LPC1769 library into IOcomposer; build matching Debug or
Release configurations. The project uses the shared
`exemples/usb/usb_custom_bulk_loopback.cpp`, UsbdBulk and a 16-entry AppEvt
queue through AppRun. It echoes binary bytes and does not create a serial port.
The allocator selects a legal bulk endpoint pair (EP2 when used alone).

Board clock and pin routing match `../UsbCdcLoopback/src/board.h`: external
16 MHz crystal, VBUS P1.30, DP P0.29, DM P0.30 and active-low SoftConnect P2.9.
Check your board schematic; P2.9 must control the external D+ pull-up circuit.
Connect USB before reset. Reset after cable reattachment. Remote wakeup,
automatic cable reattachment, ISO, DMA and low-power suspend are unsupported.

Debug uses rdimon semihosting with IDAP-Link; Release uses nosys without it.
From the repository root, with PyUSB and libusb installed:

```sh
python3 Python/usb_custom_bulk_loopback.py --cycles 100
python3 Python/usb_custom_bulk_loopback.py --cycles 100 --reconfigure
```

The host script discovers the endpoints for VID 0x1209, PID 0x0002, product
`IOsonata Custom Bulk Loopback`. Use `--serial` for multiple matching boards.
The 1024-byte TX FIFO retains bytes under backpressure.

Both firmware links pass; physical enumeration and transfers remain pending.

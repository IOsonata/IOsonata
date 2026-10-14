# LPC11U35 CDC/HID/vendor bulk composite loopback

Import `ioc` and the matching LPC11U35 library project into IOcomposer. Build
both with the same configuration. The project links the shared
`exemples/usb/usb_composite_loopback.cpp`; it contains CDC serial, a 64-byte
vendor HID report interface, and a vendor-specific bulk interface.

All interfaces echo received bytes. The classes allocate interface and endpoint
numbers. AppRun drains the 16-entry AppEvt queue; AppCheckStatus services all
three transports and retains partial TX writes. The example disables remote
wakeup and low-power suspend. It uses small FIFOs to fit LPC11U35 main RAM.

Debug uses rdimon semihosting; enable it in the IDAP-Link debug session.
Release uses nosys without semihosting. Board routing matches
`../UsbCdcLoopback/src/board.h`; review that example's README and your schematic
before connecting. Cable detach/reattach is handled by the controller and generic USB layer.


From the repository root, with PyUSB and libusb installed:

```sh
python3 Python/usb_composite_loopback.py --cycles 100
python3 Python/usb_composite_loopback.py --cycles 100 --reconfigure
```

The host test discovers the CDC, HID and bulk endpoints from descriptors and
interleaves packet-sized transfers. It compares exact bytes and can repeat
SET_CONFIGURATION. It is a correctness test, not a throughput measurement.
Close serial terminals and other programs using this USB device first. The
host must allow libusb to claim every interface; kernel-driver detachment is
attempted where supported. Use `--serial` to select one of several boards.

Demo identity: VID 0x1209, PID 0x0005, product
`IOsonata USB Composite Loopback`. These are shared test identifiers.

Debug/Release firmware links pass. See `tests/lpc/usb/README.md` for sizes and
host-model coverage. Physical enumeration, traffic and sleep/wakeup remain
pending board validation.

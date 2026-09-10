# USB Interrupt Loopback Hardware Test

This project validates the reusable `UsbIntIntrf` transport with a real USB
host and device controller. It is vendor-specific test firmware, not HID.

## Components

- Firmware: `exemples/usb/usb_int_loopback.cpp`
- nRF52840 IOcomposer project:
  `ARM/Nordic/nRF52/nRF52840/exemples/UsbIntLoopback/ioc`
- Host runner: `Python/usb_int_loopback.py`

The firmware enumerates as `1209:0004` with product string
`IOsonata USB Interrupt Loopback`. The USB core allocates one interface and one
bidirectional endpoint number. Alternate setting 0 closes the transport;
alternate settings 1, 2, and 3 open 64-byte interrupt IN/OUT endpoints with
intervals 1, 4, and 16 respectively.

## Build and flash

Build the nRF52840 IOsonata library first. Import the `UsbIntLoopback/ioc`
project into IOcomposer, build its Debug or Release configuration, and flash
the resulting image to the nRF52840 target.

The project links both the reusable `src/usb/usb_int.cpp` implementation and
the loopback application into the target application. It does not allocate a
CFifo.

## Run

Install PyUSB and a libusb backend, then run from the repository root:

```sh
python3 -m pip install pyusb
python3 Python/usb_int_loopback.py
```

Linux access may require an appropriate udev rule or root privileges. Use
`--serial` when more than one test device is connected. The descriptor-based
interface and endpoint discovery can be overridden with `--interface` and
`--ep` for diagnosis.

The automated phase verifies:

- endpoint type, pair topology, MPS, and all three intervals;
- alternate-setting close/open transitions;
- interrupt OUT and IN loopback at lengths 0, 1, 7, 63, and 64;
- TX busy rejection followed by ready-state recovery;
- sustained bidirectional traffic;
- transport diagnostic counters and absence of completion errors.

Run the explicit manual suspend/resume phase with:

```sh
python3 Python/usb_int_loopback.py --manual-suspend-wake
```

The equivalent test Makefile entry is:

```sh
make -C tests/usb int-hw
```

The host runner and project make the hardware test reproducible; their
presence does not constitute a hardware-validation result. Record the target,
host, commit, and runner output when the test is executed on hardware.

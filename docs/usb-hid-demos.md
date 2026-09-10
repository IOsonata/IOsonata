# USB HID demos

These demos show application-defined reports above `UsbdHid`. They do not add
keyboard, mouse or sensor policy to the reusable HID class.

## Boot keyboard

Project:

```text
ARM/Nordic/nRF52/nRF52840/exemples/UsbHidKeyboard/ioc
```

Hardware: Nordic nRF52840 DK.

- Button 1: P0.11, active low
- LED 1: P0.13, active low
- USB: nRF52840 native USB connector

The device enumerates as a boot-protocol keyboard. Holding Button 1 sends the
HID usage for the `A` key; releasing it sends the empty keyboard report. The
Caps Lock bit received through either the Interrupt OUT endpoint or
`SET_REPORT` controls LED 1.

Build and flash the project, select a text field on the host, and press Button
1. One `a` should be entered for each press. Toggle Caps Lock to check the LED
output report.

The example VID/PID is `1209:0006`. These values are application configuration,
not values owned by `UsbdHid`.

## BMI323 3D mouse

Project:

```text
ARM/Nordic/nRF52/nRF52840/exemples/UsbHid3dMouse/ioc
```

Hardware: the nRF52840 Bosch BMI323 application-board setup already used by
`BoschAppBoard` in this repository. Its `board.h` supplies the power-control
and SPI pin map.

The device enumerates as a HID Generic Desktop Multi-axis Controller with one
12-byte relative input report:

| HID axes | BMI323 source |
|---|---|
| X, Y, Z | Accelerometer X, Y, Z |
| Rx, Ry, Rz | Gyroscope X, Y, Z |

Keep the board stationary during startup. The demo averages 64 samples to
establish the center, then applies separate acceleration and gyro dead zones.
Move or rotate the board after enumeration to generate six-axis reports.

A normal desktop does not necessarily turn Multi-axis Controller reports into
pointer movement. Validate it in a 3D application that accepts a six-axis HID
controller, or inspect its 12-byte input reports with a HID monitor.

The example VID/PID is `1209:0007`. These values are application configuration,
not values owned by `UsbdHid`.

## Compile checks

The host compile check validates both application sources and project-local
board headers without emulating either device:

```bash
make -C tests/usb hid-demo-build
```

Physical enumeration, keyboard output and BMI323 motion still require their
respective hardware.

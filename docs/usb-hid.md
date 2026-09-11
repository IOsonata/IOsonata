# USB HID

`UsbdHid` is the reusable USB device HID class. It embeds `UsbIntIntrf`, owns
the HID descriptor and class requests, and leaves report meaning to the
application.

Use the complete loopback example as the starting point:

- firmware: [`../exemples/usb/usb_hid_loopback.cpp`](../exemples/usb/usb_hid_loopback.cpp)
- nRF52840 project: [`../ARM/Nordic/nRF52/nRF52840/exemples/UsbHidLoopback/ioc`](../ARM/Nordic/nRF52/nRF52840/exemples/UsbHidLoopback/ioc)
- host test: [`../Python/usb_hid_loopback.py`](../Python/usb_hid_loopback.py)

## Layering

```text
USB controller
    |
UsbIntIntrf
    |
UsbdHid
    |
application reports
```

`UsbIntIntrf` remains a role-neutral Interrupt transport. It does not parse HID
reports, serve HID descriptors or handle HID requests.

`UsbdHid` follows the same device-function pattern as `UsbdBulk`:

- one interface and one bidirectional endpoint number are allocated internally;
- the application supplies storage for the descriptor fragment;
- configuration opens the Interrupt OUT/IN endpoint pair;
- reset and unconfiguration close the transport;
- no dynamic allocation is used.

## Configuration

The application supplies:

- a report descriptor and its length;
- HID subclass, protocol and country code;
- full/high-speed MPS and polling intervals, or zero for defaults;
- a staged `ReportHandler` for `GET_REPORT` and `SET_REPORT`;
- optional Interrupt OUT/IN completion callbacks;
- a `UsbdHidDesc_t` inside its configuration descriptor.

USB constants and standard descriptors are defined in `usb_def.h`. HID request
codes, report types and HID descriptors are defined in `usb_hiddef.h`.

`UsbdHid` handles `GET_DESCRIPTOR` for the HID and report descriptors,
`GET_IDLE`, `SET_IDLE`, and boot-subclass `GET_PROTOCOL` and `SET_PROTOCOL`.
Report payload policy stays in the application callback.

## Hardware test

Build and flash the `UsbHidLoopback` project, install the Python hidapi package
in the repository virtual environment, and run:

```bash
./.venv/bin/python3 Python/usb_hid_loopback.py
```

For the manual suspend/resume phase:

```bash
./.venv/bin/python3 Python/usb_hid_loopback.py --manual-suspend-wake
```

The runner discovers the HID by VID/PID and product string, opens it through
the native HID driver, sends 64-byte output reports and verifies the matching
input reports. Use `--serial` when more than one matching device is attached.

## Application demos

The [HID demos](usb-hid-demos.md) build a boot keyboard and a BMI323-driven
six-axis Multi-axis Controller above this same class. Report meanings and
board configuration remain in the applications.

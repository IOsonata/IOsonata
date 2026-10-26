# CDC and ISO stress on EK-RA4M1

Import `ioc` as the `UsbCdcIsoStress` Eclipse project. Build both
`IOsonata_RA4M1` and this example in Release, flash the example, and connect
the board's device USB connector. The device identifies as
`IOsonata USB CDC ISO Stress` (VID 1209, PID 0008) and exposes one CDC port.

From the repository root:

```sh
../HciController/.venv/bin/python3 Python/usb_cdc_iso_stress.py \
  --port /dev/cu.usbmodem1421401 --duration 60
```

Use the actual port reported after flashing. The selected Python environment
needs `pyserial` and `libusb1`; HID is not used. The runner imports the existing
combo and ISO helpers, so keep the Python directory together.

The CDC path streams PRBS bytes and validates their echo. Concurrent ISO bursts
validate packet contents and sequence at alternate setting 6 (63-byte packets,
1 ms full-speed interval). ISO rates count validated echoed payload once, not
both bus directions. CDC TX and RX rates are reported separately. A pass requires
traffic on both paths, zero data errors, and no outstanding CDC echo after drain.
Host scheduling misses are reported separately; a detected host pause makes the
run inconclusive. On macOS the runner uses caffeinate when available.

The firmware uses generic UsbdCdc and UsbIsoIntrf objects, static FIFO storage,
and the same ISO descriptors, callbacks and diagnostics as UsbComboStress.
There are three USB interfaces: CDC control, CDC data, and ISO. ISO alternate
setting zero has no endpoints; settings 1..6 select 9/17/25/33/49/63 bytes.
No application controller transfers are added.

RA4M1 uses five physical data pipes: CDC bulk on 3/4, notification on 6, and
ISO on 1/2. The full dual-CDC/HID/INT/ISO combo needs more pipes than USBFS has
and cannot be used as this target's simultaneous stress configuration.

Validation: `tests/usb/cdc_iso_init_test.py` compiles the production example and
generic USB classes with host controller models, including the RA4M1 capability
header, and checks descriptors, endpoint counts, alternate settings and close.
`tests/usb/cdc_iso_host_test.py` checks pass/fail accounting and CDC drain order.
These are host tests; the ARM firmware build and hardware stress result remain
pending.

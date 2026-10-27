# Renesas USB controller integration

Status: **RA4M1 USBFS device operation is hardware validated on EK-RA4M1
for the tests recorded below.** USBFS is full speed (12 Mbit/s), not high speed.

## Architecture

The generic IOsonata USB stack remains in `include/usb` and `src/usb`.
Renesas-wide controller declarations remain in `ARM/Renesas/include/usb_ctrlr.h`.
Applications use the generic USB interfaces and never select a controller
implementation.

RA4M1 USBFS is implemented in
`ARM/Renesas/RA4/RA4M1/src/usb_ctrlr_ra4m1.cpp`. It is target-local because
the current supported Renesas targets do not share a proven register-compatible
USB controller. Do not move it to `ARM/Renesas/src` until a second target
proves that compatibility.

The implementation is FSP-free. It uses the native RA4M1 device header and the existing
IOsonata ICU helper. Renesas FSP register sequences were used only as a hardware
reference.

## RA4M1 USBFS implementation

- USBFS module-stop control: MSTPCRB bit 11.
- 48 MHz USB clock is required from `SystemUsbClockGet()`.
- ICU USBI event 0x33 is routed dynamically through the existing RA4M1
  interrupt allocator. FIFO0/FIFO1 events are not allocated as CPU IRQs.
- DCP implements EP0 setup, control IN/OUT, status completion and stalls.
- Pipes 1..2 support isochronous endpoints up to 256 bytes. Bulk allocation
  prefers pipes 3..5 and falls back to 1..2, preserving ISO-capable pipes when
  possible. Pipes 6..9 are allocated to interrupt endpoints. IN and OUT consume
  the same physical pipe pool.
- CFIFO handles data transfers. USBFS uses its fixed full-speed pipe-buffer
  organization; `PIPEBUF` is not programmed. D0FIFO/D1FIFO remain unused so
  DMA can be added later without changing the controller ABI.
- BRDY, BEMP and NRDY are interrupt driven. Idle OUT NRDY produces the generic
  `DRDY` retry event.
- Reset, VBUS, connect/disconnect, SOF, suspend/resume and remote wakeup are
  implemented.
- SET_ADDRESS is left to the USBFS hardware.
- Endpoint close/reset reclaims the physical pipe and FIFO ownership.
- ISO IN is SOF-driven through the existing generic `UsbIsoIntrf` path. At
  each service interval the controller arms ISO OUT first, then submits ISO IN.
  ISO OUT does not use NRDY retry because an isochronous host transaction is
  not retried.
- `bLowPowerSuspend=true` is rejected. Normal USB suspend/resume is supported;
  low-power clock/power entry must not be advertised until validated.

Board headers remain responsible for USB pin mapping. The controller accepts the
optional `UsbCtrlrCfg_t::pIOPinMap` and does not hard-code EK-RA4M1 pins.

## Build integration

The RA4M1 IOC library already links the generic USB headers and sources. The
RA4M1 USBFS controller source is now linked into it. The e2 library project
links the same generic USB headers/sources and the RA4M1 controller source.
`RA4/RA4M1/exemples/UsbCdcLoopback/ioc` builds the existing generic CDC
loopback application for the EK-RA4M1 device USB connector.
`RA4/RA4M1/exemples/UsbCdcIsoStress/ioc` builds the CDC loopback plus
isochronous stress application. `RA4/RA4M1/exemples/UsbDualCdcStress/ioc`
builds the dual-CDC stress application. These use the same controller.

## Recorded hardware validation

Maintainer tests on EK-RA4M1 confirmed enumeration, CDC PRBS reception,
sustained CDC loopback, a 60-second CDC plus ISO stress PASS with zero errors,
interrupt loopback at all three tested intervals, and MSC RAM-disk mounting
on an iMac. Repeated unplug/replug recovered without a board reset.
See the [RA4M1 hardware record](RA4/RA4M1/README.md#usbfs-supply-and-validation)
for details and limitations. These results describe the tested firmware;
they do not establish that every later change has been rerun on hardware.

## Remaining validation and regression coverage

Software review and strict host-side C++ syntax compilation cover the controller
ABI and register access structure, but these tests do not prove electrical or
USB protocol behavior. The following is the regression checklist; the results
above cover only the recorded configurations. Suspend/resume and remote wakeup,
HID, pipe exhaustion/reuse and the additional protocol edge cases still need
explicit hardware results:

1. CDC loopback enumeration and sustained traffic.
2. PRBS/bulk, interrupt, HID and MSC class tests.
3. Concurrent IN/OUT endpoint traffic and pipe exhaustion/reuse.
4. Bus reset, VBUS detach/attach and repeated re-enumeration.
5. Suspend/resume and remote wakeup.
6. EP0 standard/class requests, ZLP, malformed request stall and clear-stall
   recovery.
7. ISO IN/OUT at 1 ms and longer service intervals, including RX FIFO pressure
   and zero-length ISO frames.

USBHS remains future work for a target with confirmed USBHS register, clock and
PHY compatibility.

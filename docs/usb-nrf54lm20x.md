# nRF54LM20x USBHS device port

The LM20 port implements the current `UsbCtrlr` API on the integrated DWC2
USBHS controller. Build the MCU library and applications from the same revision.

| Port | Status for this change |
| --- | --- |
| nRF52 USBD | Full-speed reference; nRF52840 bare-metal and TaktOS combo builds and 2,000-second hardware runs pass |
| nRF54LM20x USBHS | Device-mode implementation; bare-metal and TaktOS combo builds and 2,000-second hardware runs pass, with host register-model coverage |
| nRF54H20 | Not covered by this port integration; its NRFS power service requires a separate implementation |
| Legacy LPC / host USB | Different controller APIs; unchanged |

## Supported transfers

EP0 handles SETUP, IN/OUT data and status stages, address assignment, stalls and
aborted control requests. Separate IN and OUT DMA buffers allow a new SETUP to
arrive while an IN packet is still pending. The port implements DWC2 buffer-DMA
SETUP completion and status-phase handling.

The DWC2 port programs `DCFG.DEVADDR` from `UsbCtrlrSetAddress`, before the
generic core arms the status IN packet. It does not defer the register write
until that packet's completion. This follows the SET_ADDRESS sequence in the
[TinyUSB DWC2 driver](https://github.com/hathach/tinyusb/blob/master/src/portable/synopsys/dwc2/dcd_dwc2.c).
The generic core still commits its software address on status completion.

Endpoint numbers 1 through 15 support bulk, interrupt and isochronous transfers.
Bulk packets are at most 64 bytes at full speed or 512 bytes at high speed.
Interrupt packets are at most 64 or 1024 bytes respectively. ISO supports one
transaction per service interval, at most 1023 bytes at full speed or 1024 bytes
at high speed. High-bandwidth additional-transaction bits are rejected.
Allocation is limited by the controller's shared FIFO RAM; opening endpoints
fails if their combined FIFO requirements do not fit. Closing an endpoint
releases its TX extent, so alternate settings can reopen with another MPS.

Endpoint callbacks run in the controller interrupt, as on nRF52. A completed
endpoint is idle before its owner is called, so the owner can start the next
transfer from the callback; a regular OUT owner then gets DRDY to supply its
next buffer. Close, reset and detach cancel an active transfer without a
callback. A full blocking RX FIFO waits for `RxData` to release space;
controller processing does not repeatedly poll it. Byte TX alignment uses the
same short-prefix scratch policy as nRF52. Packet/direct buffers must be word
aligned.

EP0 and ISO callbacks also run in the controller interrupt. DWC2 ISO is armed for the
next frame (full speed) or microframe (high speed); the SOF value delivered to
`UsbIsoIntrf` identifies that upcoming service opportunity. Missed intervals
retire through NAK/endpoint-disabled interrupts, release the frame, and allow a
later interval to proceed. A full ISO RX FIFO drops service opportunities until
space is available, without holding off the host.

VBUS removal resets the wrapper before any further core register access, then
resets class state through the USB process event. Reattachment restarts the
controller. Suspend preserves endpoint storage; `bLowPowerSuspend` gates the
PHY clock through PCGCCTL. Resume and remote wake restore it. This is clock
gating, not DWC2 hibernation. The weak 24 MHz clock-release hook retains the
existing shared-clock policy; a platform clock owner may override it.

## Projects and bench procedure

Build `ARM/Nordic/nRF54/nRF54LM20x/lib/ioc`, then one of these projects:

- `exemples/UsbCdcLoopback/ioc`: portable CDC loopback.
- `exemples/UsbCdcPrbsTx/ioc`: CDC throughput and stream integrity.
- `exemples/UsbIsoLoopback/ioc`: portable ISO loopback and alternate-setting tests.
- `exemples/UsbDualCdcStress/ioc`: concurrent CDC loopback and PRBS.
- `exemples/UsbComboStress/ioc`: dual CDC, HID, interrupt and ISO stress.
- `exemples/UsbCustomBulkLoopback/ioc`: vendor bulk loopback.
- `exemples/UsbHidLoopback/ioc`: vendor HID report loopback.
- `exemples/UsbHidKeyboard/ioc`: boot keyboard using the LM20 DK buttons and LED.
- `exemples/UsbIntLoopback/ioc`: interrupt loopback and alternate settings.
- `exemples/UsbMscRamDisk/ioc`: disposable 64 KiB FAT12 RAM disk.
- `exemples/UsbCdcLoopbackTaktOS/ioc`: CDC loopback with a heartbeat thread.
- `exemples/UsbComboStressTaktOS/ioc`: composite stress with separate service,
  loopback, PRBS and heartbeat threads.

The nine added projects provide Debug and Release builds. Build the matching
Debug or Release LM20 library configuration first. The applications use the
USB stack from that library; they do not enable a Bluetooth stack. The two
TaktOS projects also link `TaktOS_M33` from the matching Debug or Release build
of `TaktOS/ARM/cm33/ioc`, with TaktOS beside IOsonata. Their sources retain the
existing USB work-queue overrides. See
[USB with TaktOS](../exemples/usb/usb_taktos/README.md) for thread ownership.

The keyboard pin map follows the existing LM20 Nordic DK examples: A on
P1.26, Caps Lock on P1.09 and the Caps Lock LED on P1.22, all active low.
Adapt `src/board.h` for another board.

`UsbHid3dMouse` still needs an LM20 SPI driver and a BMI323 board pin map.
The LM20 library currently links only the generic SPI helpers, which do not
define `SPIInit`; the existing Nordic SPI driver is not ported to LM20.
The legacy nRF5 SDK USB demos and TinyUSB comparison projects are separate
integrations and are not part of this portable USB project set.
`exemples/TinyUsbComboStress/ioc` runs the combo workload on TinyUSB's DWC2
driver, with incomplete ISO OUT handling added; see the
[TinyUSB comparison notes](../exemples/usb/tinyusb_common/README.md).

These USB projects select `NRF54LM20B_XXAA`, matching the library, and use the
shared LM20 memory-layout script. For LM20A, select `NRF54LM20A_XXAA` in both the
library and application. Use the USB connector connected to the LM20 USBHS pins,
not the board's debugger USB connector. The project uses the Nordic FICR/system
startup PHY tuning.

Run the existing Python USB host runners against the matching firmware. For ISO,
use `Python/usb_iso_loopback.py --manual-suspend-wake`. The portable ISO bench
advertises its existing six small, odd-sized alternate settings; the register
model additionally covers the 1024-byte USBHS limit.

Check both a direct high-speed connection and a forced full-speed connection.
Exercise repeated configuration/alternate changes, short packets and ZLPs,
unplug/replug with traffic active, suspend/resume, remote wake, and simultaneous
IN/OUT traffic. Check CDC PRBS continuity and ISO diagnostics across these cases.
The register model cannot establish PHY timing, electrical compliance or host
interoperability.

### USB CDC to BLE central bridge

`exemples/UsbCdcBleCentralDemo/ioc` and `exemples/UsbCdcBleCentralTaktOS/ioc`
build `exemples/bluetooth/usb_cdc_ble_central.cpp` and
`usb_cdc_ble_central_taktos.cpp`, the same sources as the nRF52840 projects.
Each has four configurations:

- Debug and Release use the S145 SoftDevice with the Debug or Release library.
  Program `s145_nrf54lm20_10.0.1_softdevice.hex` from
  `sdk-nrf-bm/components/softdevice/nrf54lm/s145` before the application.
- Debug_SDC and Release_SDC use the SoftDevice Controller with the Debug_SDC or
  Release_SDC library, and link `mpsl`, `mpsl_fem_common` and
  `softdevice_controller_multirole` from sdk-nrfxlib.

All four use `gcc_nrf54lm20a_xxaa_s145.ld`; with the SoftDevice Controller its
SoftDevice partition stays unused. The TaktOS project also links `TaktOS_M33`.
`src/board.h` maps the four DK LEDs; LED4 shows the Bluetooth link.

These projects compile and link in Debug and Debug_SDC; they have not been run
on hardware. The USB port starts HFCLK24M directly through CLOCK and runs USB at
interrupt priority 6; check enumeration with the radio stack enabled first.

## Enumeration trace

The LM20 controller enables `NRF54_USB_TRACE` by default in Debug builds
(`NDEBUG` absent). Rebuild the Debug MCU library and link the Debug application.
Use the existing rdimon/semihosting debug configuration and run without USB
breakpoints. Release builds compile out the trace; `NRF54_USB_TRACE=0` also
disables it explicitly in Debug.

`UsbCtrlrInit` first prints and flushes `USB TRACE enabled dev=0` directly,
before enabling USB hardware. This banner does not depend on event processing.
If it appears but no records follow, check whether `UsbCtrlrProcess` runs.
If it is absent, check that the linked MCU library contains the trace and that
stdout reaches the debugger; the USB event queue cannot suppress this banner.

The ISR stores up to 64 fixed records. `UsbCtrlrProcess` prints them with
`printf` in foreground, with interrupts restored, and flushes stdout. Overflow
reports `USB TRACE lost=...`. No SOF or regular endpoint traffic is logged.
The capture covers startup, reset, speed, EP0 interrupts, SETUP, transfer arms
and completions, address assignment, stalls, suspend/resume and VBUS changes.
In `USB SETUP`, `type_req` packs `bmRequestType` in the high byte and `bRequest`
in the low byte; `value` and `index` are hexadecimal and `len` is decimal.
For example, `type_req=0005 value=000f index=0000 len=0` is SET_ADDRESS(15).
`USB ADDRESS programmed` should precede the status `USB EP0 ARM ep=80`.
`USB EP0 NEXT` snapshots EP0 OUT control, transfer size, DMA and interrupt
registers after completion and preparation for the next transfer.
Capture output from `USB START` through the first failure or repeated reset.
Semihosting output still affects timing, so use Release for throughput runs.

## Recorded hardware validation

The maintainer reported passing 2,000-second LM20 composite runs in both bare
metal and TaktOS. Both runs had zero loopback, PRBS, target RX, HID, interrupt
and ISO data errors, zero pending loopback bytes, and zero ISO SOF/OUT losses.
Bare metal reported eight ISO host misses, four host skews and two host pauses
totalling 33.2 seconds; TaktOS reported none. These results validate concurrent
traffic for the tested builds; they do not establish every lifecycle case or
every project configuration above.

The [0.13 release notes](releases/0.13.md#recorded-validation) record firmware
sizes, per-stream throughput and host diagnostics. Exact firmware SHAs and
compiler versions were not supplied with those results.

## Automated validation

`make -C tests/usb test-combo-init` links the production combo example and
generic USB stack against a no-op controller. It checks endpoint ownership,
descriptors and ISO initialization with dedicated ISO endpoints and the LM20
capability header, including refusal when no ISO pair remains. On LM20, the
combo examples try supported ISO pairs until an unoccupied pair is allocated;
the first ISO-capable endpoint may already belong to CDC.

`make -C tests/usb test-nrf54-data` compiles the complete controller source with
modeled MMIO and the real Nordic API header, `UsbIntrf`, `UsbIsoIntrf` and CFifo.
It covers reset handshakes, powered-register access, FIFO allocation/reuse,
endpoint lifetimes, queue refusal, RX backpressure, aligned DMA and byte-prefix
repair, EP0 SETUP/data/status, ISO frame parity and missed intervals, suspend,
remote wake and detach/restart. The model includes W1C interrupts and the
NAK/disable handshakes. This is a host build, not an MCU firmware build.

`make -C tests/usb test` also runs the generic USB and nRF52 controller tests.
No USBHS hardware result or Arm cross-build is claimed by these host tests.

# TinyUSB performance comparison

These examples use TinyUSB for USB and the same IOsonata MCU platform
library as the corresponding IOsonata examples.

| Benchmark | IOsonata source | TinyUSB source | Target project |
|---|---|---|---|
| PRBS TX | `usb_cdc_prbs_tx.cpp` | `tinyusb_cdc_prbs_tx/main.cpp` | nRF52840 TinyUsbCdcPrbsTx/ioc |
| Loopback | `usb_cdc_loopback.cpp` | `tinyusb_cdc_loopback/main.cpp` | nRF52840 TinyUsbCdcLoopback/ioc |
| Dual CDC | `usb_dual_cdc_stress.cpp` | `tinyusb_dual_cdc_stress/main.cpp` | nRF52840 TinyUsbDualCdcStress/ioc |
| Composite | `usb_combo_stress.cpp` | `tinyusb_combo_stress/main.cpp` | nRF52840 TinyUsbComboStress/ioc |
| Composite | `usb_combo_stress.cpp` | `tinyusb_combo_stress/main.cpp` | nRF54LM20x TinyUsbComboStress/ioc |

Sources are relative to [exemples/usb](../).
Target projects are below
[ARM/Nordic/nRF52/nRF52840/exemples](../../../ARM/Nordic/nRF52/nRF52840/exemples)
and
[ARM/Nordic/nRF54/nRF54LM20x/exemples](../../../ARM/Nordic/nRF54/nRF54LM20x/exemples).

## Build in IOcomposer

Open the chosen `ioc/` project in IOcomposer and select the same build
configuration used for the IOsonata comparison image. Dual CDC and composite
projects use the managed builder. The single-CDC projects also contain
Makefiles with `release` and `debug` targets; do not assume every example
has a checked-in Makefile.

Use TinyUSB tag `0.21.0` for the documented baseline, and record the actual
revision if using a different checkout. Place dependencies beside IOsonata:

```text
<workspace>/
    IOsonata/
    external/
        tinyusb/
        nrfx/
```

TinyUSB source: https://github.com/hathach/tinyusb

The nRF52840 projects link the selected `IOsonata_nRF52840` platform library
and use the IOsonata linker script and CMSIS headers. TinyUSB supplies the
device core, class sources and the base Nordic DCD.

Do not compile the IOsonata USB controller source separately into these
applications. Their USB interrupt handler belongs to TinyUSB.

## Composite source layout

Both composite projects build the same
[tinyusb_combo_stress/main.cpp](../tinyusb_combo_stress/main.cpp). It holds the
descriptors, the HID and raw interrupt handling, an application ISO class
driver for the EP8 alternate settings and the main loop. The high-speed
configuration, device qualifier and other-speed configuration are built only
when `tusb_config.h` selects high speed.

The chip glue is in each project's `src` folder, the same way an example takes
its pin map from the project's `board.h`:

- `tusb_config.h`: TinyUSB configuration for the chip.
- `tinyusb_combo_port.cpp`: the functions declared in
  [tinyusb_combo_port.h](../tinyusb_combo_stress/tinyusb_combo_port.h), which
  power the controller, define the USB interrupt handler, report USB power
  changes and read the device identifier.

The nRF52840 composite project also links
[this repository's modified Nordic DCD](../../../ARM/Nordic/nRF52/nRF52840/exemples/TinyUsbComboStress/src/dcd_nrf5x.c)
instead of TinyUSB's, which changes ISO EasyDMA scheduling. Its scheduling
diagnostics (vendor request 0x5B) are built only with
`TINYUSB_COMBO_ISO_DIAG` set to 1 in
[its tusb_config.h](../../../ARM/Nordic/nRF52/nRF52840/exemples/TinyUsbComboStress/src/tusb_config.h);
keep it 0 for size and performance comparisons. Raw Interrupt traffic uses
TinyUSB's Vendor class. Record both the IOsonata revision (which identifies
the modified DCD) and the external TinyUSB revision with comparison results;
the nRF52840 composite is not a stock upstream TinyUSB benchmark.

## nRF54LM20 composite

The LM20 project links the stock TinyUSB device core, classes and
`dwc2_common.c` from `external/tinyusb`, in TinyUSB's default configuration.
For this controller that is slave (FIFO) mode, the mode TinyUSB's LM20 board
support uses; the IOsonata port uses the controller's buffer DMA. Record this
with any comparison.

The DWC2 device driver is the project's
[src/dcd_dwc2.c](../../../ARM/Nordic/nRF54/nRF54LM20x/exemples/TinyUsbComboStress/src/dcd_dwc2.c),
a copy of TinyUSB 064afa3 `src/portable/synopsys/dwc2/dcd_dwc2.c` with two
changes for isochronous endpoints. The stock driver enables an ISO transfer
for the microframe after the current one and retries only IN, through the
following microframes. With the combo's ISO bInterval 4 at high speed the
host services the endpoints every 8 microframes, so the stock driver fails
the ISO test on its first burst: the OUT endpoint re-armed after a reception
targets the other microframe parity and never receives again.

- An ISO transfer with an interval above one microframe is kept pending and
  enabled at the SOF of the microframe before its service microframe, a
  whole microframe ahead of the host's token. Service microframes are the
  multiples of the interval, as in the IOsonata LM20 port, whose combo runs
  show this host servicing the endpoints there.
- The incomplete ISO OUT interrupt is handled: the endpoint is disabled under
  global OUT NAK, as the Zephyr, Linux and IOsonata DWC2 drivers do, and the
  transfer completes as failed so the class arms it again. A missed IN takes
  TinyUSB's existing incomplete ISO IN path.

A first version without the SOF start retargeted the endpoints one
microframe at a time through the incomplete interrupts. It ran ISO for 412 s
and then lost a frame; there the enable for the service microframe depends
on interrupt latency at the end of the microframe before it. As with the
nRF52840 composite, record the IOsonata revision with results; this is not
stock TinyUSB.

The project's `src` folder supplies what TinyUSB's own LM20 board support
supplies outside the driver. TinyUSB's `dwc2_nrf.h` runs the USBHS power-up
only when `NRF54LM20A_ENGA_XXAA` is defined, so `tinyusb_combo_port.cpp` runs
the same sequence before `tusb_init`: 24 MHz crystal clock, USB regulator,
core and PHY enable, then the core reset release. It also defines
`USBHS_IRQHandler`. `tusb_config.h` maps `NRF_USBHSCORE0` to the MDK name
`NRF_USBHSCORE`. TinyUSB includes `soc/nrfx_coredep.h`, which nrfx 4 moved to
`lib/`; `src/soc/nrfx_coredep.h` forwards to the new location.

The descriptors, workload and host runner match the IOsonata LM20 combo:
512-byte CDC bulk packets at high speed, HID and ISO at bInterval 4, the
interrupt alternate settings at bInterval 1, 4 and 16, and PRBS written one
byte per main loop pass. ISO uses EP8 as on nRF52840; the host runner finds
the interrupt and ISO endpoints from the descriptors.
TinyUSB's LM20 support has no VBUS event handling, and unplug/replug of the
USBHS connector was not checked with this project.

Build the matching Debug or Release `nRF54LM20x/lib/ioc` configuration first.
The project was build-checked against TinyUSB 064afa3. The runner's optional
ISO trace and controller diagnostic requests (0x5C and 0x5B) stall on this
firmware and are skipped; the ISO counters (0x5A) are available.

## Workload and storage

The nRF52840 projects use full-speed USB and 64-byte CDC bulk packets on both
sides. The
matching examples use the same VID/PID and host workload. The composite
firmware deliberately shares the IOsonata product identity so the same host
runner can select it without changed filters.

The single-port comparisons have matched CDC payload queue capacities:

| Benchmark | RX payload queue | TX payload queue |
|---|---:|---:|
| PRBS TX | 256 bytes | 2048 bytes |
| Loopback | 256 bytes | 1024 bytes |

In the dual-CDC and composite examples, TinyUSB's `CFG_TUD_CDC_TX_BUFSIZE`
is 2048 bytes for each CDC instance. IOsonata uses 1024 bytes for loopback TX
and 2048 bytes for PRBS TX. Both use 256-byte CDC RX payload capacities on
nRF52840. On LM20 both use 2048-byte CDC RX payload capacities: four
high-speed packets on IOsonata, `CFG_TUD_CDC_RX_BUFSIZE` on TinyUSB. This
difference must accompany memory comparisons; the composite queues are not
byte-for-byte matched.

`CFIFO_MEMSIZE(n)` adds CFifo metadata to the payload capacity. IOsonata's
packet RX storage also includes packet headers. Compare capacities separately
from raw allocation sizes.

Both composite configuration descriptors are 398 bytes. TinyUSB keeps its
fixed descriptor in const storage; IOsonata builds the selected composition
in a 768-byte nRF52840 configuration buffer. That buffer supports runtime
composition and is not the whole USB stack. Device, string and HID report
descriptors have separate storage.

## Run the same host test

Install the dependencies listed in the
[USB User Guide](../../../docs/usb-user-guide.md). Rediscover serial paths
after flashing each image.

Single CDC loopback:

```bash
python3 Python/usb_cdc_loopback.py --port /dev/cu.usbmodemXXXX --duration 60
```

Dual CDC:

```bash
python3 Python/usb_dual_cdc_stress.py \
  --loop-port /dev/cu.usbmodemXXXX01 \
  --prbs-port /dev/cu.usbmodemXXXX03 --duration 2000
```

Composite:

```bash
python3 Python/usb_combo_stress.py \
  --loop-port /dev/cu.usbmodemXXXX01 \
  --prbs-port /dev/cu.usbmodemXXXX03 --duration 2000
```

PRBS-only firmware uses the same PRBS receiver as the IOsonata version.
A missing IOsonata banner warning from the single-CDC loopback runner does
not by itself indicate a data-integrity failure.

Keep the board, cable, host USB port, host software, duration and command
options fixed. Use the same compiler, optimization and selected IOsonata
platform library. Record firmware and TinyUSB revisions, clean-build sizes,
complete results and any host scheduling diagnostics. Do not compare Debug
against Release.

For memory, report flash as `text + data` and static RAM as `data + bss`.
Report whole-image costs, queue capacities and application buffers; ELF size
alone does not isolate a USB-stack cost. Repeat hardware runs before drawing
a performance conclusion from a single throughput result.

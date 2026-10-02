# TinyUSB performance comparison

These nRF52840 examples use TinyUSB for USB and the same IOsonata MCU
platform library as the corresponding IOsonata examples.

| Benchmark | IOsonata source | TinyUSB source | Target project |
|---|---|---|---|
| PRBS TX | `usb_cdc_prbs_tx.cpp` | `tinyusb_cdc_prbs_tx/main.cpp` | TinyUsbCdcPrbsTx/ioc |
| Loopback | `usb_cdc_loopback.cpp` | `tinyusb_cdc_loopback/main.cpp` | TinyUsbCdcLoopback/ioc |
| Dual CDC | `usb_dual_cdc_stress.cpp` | `tinyusb_dual_cdc_stress/main.cpp` | TinyUsbDualCdcStress/ioc |
| Composite | `usb_combo_stress.cpp` | `tinyusb_combo_stress/main.cpp` | TinyUsbComboStress/ioc |

Sources are relative to [exemples/usb](../).
Target projects are below
[ARM/Nordic/nRF52/nRF52840/exemples](../../../ARM/Nordic/nRF52/nRF52840/exemples).

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

The projects link the selected `IOsonata_nRF52840` platform library and use
the IOsonata linker script and CMSIS headers. TinyUSB supplies the device core,
class sources and the base Nordic DCD. The composite project instead links
[this repository's modified Nordic DCD](../tinyusb_combo_stress/dcd_nrf5x.c),
which changes ISO EasyDMA scheduling and adds scheduling diagnostics. It also
uses an application ISO class driver for EP8 alternate settings and TinyUSB's
Vendor class for raw Interrupt traffic. Record both the IOsonata revision
(which identifies the modified DCD) and the external TinyUSB revision with
comparison results; this is not a stock upstream TinyUSB composite benchmark.

Do not compile the IOsonata USB controller source separately into these
applications. Their USB interrupt handler belongs to TinyUSB.

## Workload and storage

Both sides use nRF52840 full-speed USB and 64-byte CDC bulk packets. The
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
and 2048 bytes for PRBS TX. Both use 256-byte CDC RX payload capacities.
This difference must accompany memory comparisons; the composite queues are
not byte-for-byte matched.

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

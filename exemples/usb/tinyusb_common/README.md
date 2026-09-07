# TinyUSB CDC performance comparison

These examples provide a native TinyUSB baseline for the existing IOsonata USB
CDC benchmarks on nRF52840.

The comparison intentionally keeps the hardware and application shape close to
the IOsonata examples:

| Benchmark | IOsonata example | TinyUSB example |
| --- | --- | --- |
| PRBS TX | `usb_cdc_prbs_tx.cpp` | `tinyusb_cdc_prbs_tx/main.cpp` |
| Loopback | `usb_cdc_loopback.cpp` | `tinyusb_cdc_loopback/main.cpp` |

Both implementations use:

- nRF52840 full-speed USB;
- CDC notification endpoint 1 IN;
- CDC data endpoint 2 OUT/IN;
- 64-byte bulk maximum packet size;
- USB interrupt priority 6;
- VID `0x1209`;
- the same PID as the matching IOsonata benchmark.

The FIFO payload capacities are matched to the IOsonata examples:

| Benchmark | RX FIFO | TX FIFO |
| --- | ---: | ---: |
| PRBS TX | 256 bytes | 2048 bytes |
| Loopback | 256 bytes | 1024 bytes |

`CFIFO_MEMSIZE(n)` includes the CFifo header in addition to `n` bytes of byte-mode
payload storage, so these values compare payload queue capacity rather than raw
allocation size.

## Eclipse projects

Import these project directories into Eclipse Embedded CDT:

```text
ARM/Nordic/nRF52/nRF52840/exemples/TinyUsbCdcPrbsTx/ioc
ARM/Nordic/nRF52/nRF52840/exemples/TinyUsbCdcLoopback/ioc
```

Each directory contains `.project`, `.cproject`, `Makefile` and `.gitignore`.
The Eclipse project links the benchmark source and the required TinyUSB source
files into its `src` folder. The default Eclipse build target is `release`.
`debug` is also available as a Make Target.

## TinyUSB version

Use TinyUSB 0.21.0 for a reproducible comparison. Place it in the external
dependency directory beside IOsonata:

```text
<workspace>/
    IOsonata/
    external/
        tinyusb/
        nrfx/
```

TinyUSB repository:

```text
https://github.com/hathach/tinyusb.git
```

Checkout tag `0.21.0`.

Each benchmark project compiles the example `main.cpp` plus:

```text
external/tinyusb/src/tusb.c
external/tinyusb/src/common/tusb_fifo.c
external/tinyusb/src/device/usbd.c
external/tinyusb/src/device/usbd_control.c
external/tinyusb/src/class/cdc/cdc_device.c
external/tinyusb/src/portable/nordic/nrf5x/dcd_nrf5x.c
```

The Makefile adds the TinyUSB and nrfx include directories and reuses the
IOsonata nRF52840 linker script, CMSIS headers and `IOsonata_nRF52840` library.
Do not compile the IOsonata USB controller source separately into the TinyUSB
benchmark application. The application provides a strong `USBD_IRQHandler()`
that forwards the interrupt to TinyUSB.

## Running the comparison

PRBS TX uses the same host PRBS receiver as the IOsonata benchmark. Run the
same receiver, port and reporting setup for both firmware images.

Loopback uses:

```sh
python3 Python/usb_cdc_loopback.py --port /dev/cu.usbmodemXXXX --duration 60
```

The current host script may warn that the IOsonata loopback banner was not
seen when TinyUSB firmware is loaded. The warning does not stop the throughput
or integrity test.

Measure Release against Release, using the same board, cable, host port and
host command. Do not compare a Debug TinyUSB build against a Release IOsonata
build.

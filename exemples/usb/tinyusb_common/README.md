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

The TinyUSB PRBS example uses a 2048-byte CDC TX FIFO. The TinyUSB loopback
example uses a 1024-byte CDC TX FIFO and a 256-byte CDC RX FIFO, matching the
queue scale of the corresponding IOsonata benchmarks.

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

## Sources to add to each nRF52840 benchmark project

Compile the example `main.cpp` plus these TinyUSB files:

```text
external/tinyusb/src/tusb.c
external/tinyusb/src/common/tusb_fifo.c
external/tinyusb/src/device/usbd.c
external/tinyusb/src/device/usbd_control.c
external/tinyusb/src/class/cdc/cdc_device.c
external/tinyusb/src/portable/nordic/nrf5x/dcd_nrf5x.c
```

Use the matching example directory as an include path so TinyUSB finds its
`tusb_config.h`, and add:

```text
external/tinyusb/src
external/nrfx
external/nrfx/drivers/include
external/nrfx/hal
external/nrfx/bsp/stable/mdk
```

The existing IOsonata nRF52840 startup, linker script, CMSIS and
`IOsonata_nRF52840` library can remain in the project. Do not compile the
IOsonata USB controller source separately into the TinyUSB benchmark
application. The application provides a strong `USBD_IRQHandler()` that
forwards the interrupt to TinyUSB.

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

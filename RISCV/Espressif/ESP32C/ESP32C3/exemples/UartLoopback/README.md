# ESP32-C3 UART0 loopback — shared IOsonata PRBS test

## Hardware and firmware

ESP32-C3-DevKitM-1 with the on-board CP2102N USB/UART bridge,
UART0 TX = GPIO21 and RX = GPIO20. Use 1,000,000 baud, 8N1, no
hardware flow control. **No jumper or second adapter is needed**;
do not short GPIO21 to GPIO20.

The Eclipse Embedded CDT / IOcomposer managed project is:

`RISCV/Espressif/ESP32C/ESP32C3/exemples/UartLoopback/ioc/`

It links the **unchanged generic** `exemples/uart/uart_loopback.cpp`.
The default C++ UART configuration uses interrupt-driven RX and TX
with 256-byte caller-owned FIFOs, and DMA disabled. The firmware
emits binary echo data only; do not open a text console concurrently.

Clean-build the `IOsonata_ESP32C3` library and this project's
Release configuration, then flash the board. Close any other terminal
before running the host test.

## Standard PRBS loopback test

**Use IOsonata's existing cross-MCU tester**, not a separate ESP32
script. From the repository root:

```sh
python3 Python/uartprbs_lb.py --port /dev/cu.usbserial-142140 --baud 1000000
```

This uses the shared PRBS generator and threaded sliding-window echo
checker. Its default parameters are `--block-size 16 --window 128`,
and it reports per-second RX B/s, errors, pending bytes and any
lost/corrupt/repeated data. Stop with Ctrl-C after a stable run.

For a second comparison with a larger outstanding window:

```sh
python3 Python/uartprbs_lb.py --port /dev/cu.usbserial-142140 --baud 1000000 --block-size 64 --window 256
```

Use the same settings across MCU comparisons. An increased window can
also reveal UART RX FIFO service or buffering problems; do not assume
that larger windows always improve throughput. Shared host-side unit
tests are in `tests/uart/uartprbs_lb_test.py`.

For **one-way TX throughput**, use the generic
`exemples/uart/uart_prbs_tx.cpp` and existing
`Python/uartprbs_rx.py` together, built in Release with specified
byte or buffer mode. The legacy ESP32-C3 `UartPrbsTxTest` project has
a bring-up-specific `main.cpp`, so do not mistake it for a standard
PRBS throughput firmware unless its project is changed to link the
generic transmitter.

## Recorded ESP32-C3 loopback evidence — November 8, 2026

The first hardware test used a **temporary stop-and-wait** host script,
which sent one 64-byte frame, waited for the entire echoed frame,
and only then transmitted the next frame.

- USB serial port: `/dev/cu.usbserial-142140`, UART0 at 1 Mbaud.
- Synchronization, 7 binary patterns and 15 lengths (1..255) passed.
- Sustained run: 15,584 echo frames; 997,376 verified bytes.
- Data integrity: **0 mismatches / 0 errors** over 30.0 seconds.
- Reported host stop-and-wait echo rate: **33,245 B/s**.

This is useful evidence that UART0 can receive and echo the tested
binary data without corruption or loss. The 33,245 B/s figure is **not
a meaningful maximum-UART-throughput comparison**: it includes a
host/USB round trip for every 64 bytes, without pipelining. No maximum
UART0 RX or echo throughput is claimed from that measurement.

**Status:** data-integrity PASS in the specified test; actual streaming
RX/loopback throughput still awaiting the standard
`Python/uartprbs_lb.py` result. The application and UART driver
have not been modified to interpret the earlier rate as a driver bug.

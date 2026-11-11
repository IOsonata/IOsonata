# ESP32-C3 UART0 loopback - shared IOsonata PRBS test

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
`Python/uartprbs_rx.py` together. The ESP32-C3 `UartPrbsTxTest` managed
project now builds the unchanged generic transmitter in both configurations.
The old bring-up `main.cpp` is excluded. Board configuration selects
1 Mbaud, 8N1, interrupt mode, and no DMA. The generic transmitter's
existing byte mode and event callback remain unchanged.

Build the library and `UartPrbsTxTest` in Release, flash, then run:

```sh
python3 Python/uartprbs_rx.py --port /dev/cu.usbserial-142140 --baud 1000000
```

This measures continuous TX without the host's echo window or firmware RX
path. A rate near 100,000 B/s with no continuing errors would show that
continuous TX can fill the line in this configuration. It would not by
itself distinguish firmware RX latency from USB/host delays in loopback.
A lower TX-only rate requires further investigation of TX and the receive
measurement path before attributing the loopback gap to RX.

## Recorded ESP32-C3 loopback evidence - November 8, 2026

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

## Baseline with the shared benchmark

The maintainer ran the unmodified `Python/uartprbs_lb.py` on November 8,
2026, with `--baud 1000000` and the shared defaults
(`--block-size 16 --window 128`). Sixteen consecutive reports showed
**zero errors**. RX throughput averaged **66,549 B/s**, ranging from
63,724 to 67,645 B/s; pending bytes were mostly 127. This is the actual
streaming-loopback baseline for the ESP32-C3 port.

The subsequent UART driver revision on `esp32_port` changes the target
implementation, not the shared Python test or generic example:

- `Esp32UartTxData` writes a short frame directly into available
  hardware TX FIFO space, before using the software FIFO;
- the TX-empty interrupt is armed only when queued bytes remain or an
  application registered a completion callback;
- the UART interrupt uses `CFifoPutByte` and `CFifoGetByte`, and
  records framing, parity, and overflow conditions;
- the blocking RX FIFO is peeked, copied, then released without masking
  UART interrupts during the copy, following the existing Renesas UART
  pattern. A non-blocking FIFO retains the protected copy path.

**The first driver revision was measured on hardware, with zero errors,**
but its throughput remained below the other MCU ports. The result and
next changes are described below.

The source checks for these MCU-specific paths are in
`tests/uart/esp32_uart_flow_source_test.py`. They do not replace
hardware testing.

## First UART driver optimization: hardware result

The maintainer reran `Python/uartprbs_lb.py` at 1,000,000 baud, using the
same 16-byte block and 128-byte window. Twenty reported samples averaged
**67,872 B/s** (range 64,239-68,980 B/s), with **0 errors**. This is only
about 2% faster than the 66,549 B/s baseline and is not satisfactory.

## Next UART revision on esp32_port (hardware test pending)

The revised UART source now uses contiguous CFIFO reservations to transfer
hardware RX and TX FIFO batches instead of performing a FIFO-index operation
for each byte. The receive-full interrupt threshold is **16 bytes rather
than 64 bytes**, matching the generic `uart_loopback.cpp` application
buffer size and reducing the wait before an interrupt on short bursts.
The driver's entire C++ source has been corrected to tab indentation, ASCII,
declarations at function entry, and expanded control statements.

This revision has **not** yet been run on ESP32-C3 hardware. Compare it with
the shared PRBS test at the same baud, block size and window. Do not treat
source checks as measured throughput or functional hardware acceptance.

### Follow-up transfer cleanup (hardware test pending)

Blocking TX now reserves FIFO memory, copies with interrupts enabled, and
publishes only after the copy. Non-blocking RX/TX copies mask interrupts for
at most 128 bytes per span. Direct TX uses available hardware FIFO space
whenever the software queue is empty, including while a TX-ready event is
pending. RX clears its ready flag before checking for remaining data, so
a newly arriving RX interrupt cannot be overwritten by a stale empty result.

`python3 tests/uart/run_esp32_uart_transfer.py` runs the actual UART transfer
functions and CFIFO with simulated registers for C3/C5/C6, at `-O0` and
`-O2` with undefined-behavior checks. It covers wrap, partial acceptance,
non-blocking overflow, copy interrupt protection, and an interrupt before
a reserved TX span is filled. These are host tests, not MCU firmware builds
or throughput measurements. The shared UART interface and PRBS test remain
unchanged.

## Follow-up transfer cleanup: hardware result

The maintainer's extended run of commit `6ed776f5` reported 99 samples at
1,000,000 baud with the unchanged 16-byte block and 128-byte window:

- Arithmetic mean of the reported rates: **91,140 B/s**.
- First 40 reports: **93,388 B/s** average.
- Minimum / maximum: **73,403 / 94,015 B/s**.
- **0 errors** throughout the supplied log.

The full-run mean is 34.3% above the earlier 67,872 B/s mean. The later
samples fluctuate below the initial 93-94 KB/s plateau. These reports do
not distinguish firmware latency from USB bridge or host scheduling delays.
At 1 Mbaud with 8N1 framing, the wire limit is 100,000 data bytes/s.

### C3 interrupt optimization: reverted after hardware regression

Commit `fc3dde2b` removed the forced `-O0` attributes from the C3 trap
handler and matrix dispatcher. Although cross-compilation passed and the
Release functions were smaller, the maintainer's next board run reported
15 samples averaging **84,565 B/s**, ranging from **71,041 to 90,681 B/s**,
with **0 errors**. This was worse than the preceding run.

The complete C3 vector source is restored byte-for-byte from `6ed776f5`.
The UART FIFO and critical-section improvements from that commit remain.
Smaller compiled functions did not establish a throughput improvement;
the cause of the measured slowdown has not been isolated. A repeat board
run is needed to confirm throughput after the restoration.

## Continuous TX result and RX threshold correction

The maintainer ran the generic `UartPrbsTxTest` from `19e423d2` at
1 Mbaud, 8N1, byte mode, interrupts enabled and DMA disabled. The host
`uartprbs_rx.py` log ended with a cumulative average of **99,997.11 B/s**
and no reported error counters. Continuous TX therefore reaches the
100,000 B/s wire limit in this configuration. This does not measure RX or
exclude USB/host round-trip latency from the loopback result.

The earlier RX threshold comment was incorrect: RX-full asserts when FIFO
occupancy is **greater than** the register value. A value of 16 waits for
17 bytes or timeout. The driver now writes **15** to deliver a 16-byte
burst at its final byte. TX-empty remains 16; interrupt dispatch, timeout
configuration, generic examples, and host test scripts are unchanged.

Reference: ESP32-C3 Technical Reference Manual, UART FIFO description:
https://documentation.espressif.com/esp32-c3_technical_reference_manual_en.pdf

The threshold correction needs hardware measurement. Rebuild the library
and UartLoopback in Release and use the original loopback command, with
its unchanged 16-byte block and 128-byte window. No gain is claimed before
that measurement.

## Window comparison and four-byte RX batching trial

With the RX-full register at 15, the maintainer reported:

- Window 128: 30 samples, mean **88,573 B/s**, zero errors.
- Window 256: 52 samples, mean **98,014 B/s**, zero errors.
- Both runs kept the host write block at 16 bytes and baud at 1 Mbaud.

The larger window substantially reduces the throughput deficit without
changing the firmware. This supports a round-trip latency limit; it does
not isolate the MCU's share from USB bridge and host scheduling delays.

The next controlled change sets RX-full to **3**, delivering four-byte
batches instead of 16. At 1 Mbaud, 8N1, four bytes take 40 us on the wire
versus 160 us for 16 bytes. These are batch serialization times, not
measured interrupt latency. Continuous input can now generate roughly
25,000 RX-full interrupts/s instead of 6,250, so the reduced batching delay
must be weighed against the extra CPU work on hardware.

TX configuration, interrupt dispatch, RX timeout configuration, generic
loopback application and Python scripts are unchanged. Keep the benchmark
at `--block-size 16 --window 128`. Tails shorter than four bytes
remain handled by the existing RX timeout interrupt. Throughput and CPU cost
of this change have not yet been measured on the board.

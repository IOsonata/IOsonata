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
`Python/uartprbs_rx.py` together, built in Release with specified
byte or buffer mode. The legacy ESP32-C3 `UartPrbsTxTest` project has
a bring-up-specific `main.cpp`, so do not mistake it for a standard
PRBS throughput firmware unless its project is changed to link the
generic transmitter.

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

### C3 interrupt optimization (hardware test pending)

`Vectors_esp32c3.c` forced both `RISCV_TrapHandler` and
`Esp32C3IntMtxDispatch` to `-O0`, including in Release. Both now use the
project-selected optimization level. The file also uses tabs and declares
locals at function entry. Routing, IRQ counters, exception diagnostics,
IRAM placement, and the compiler's machine-interrupt attribute are preserved.

xPack RISC-V GCC 15.2.0-1 compiled the C3 vector source with
`-march=rv32imc_zicsr -mabi=ilp32 -Wall -Wextra -Werror` at `-O0` and `-Os`.
In the unlinked `-Os` object, the trap handler shrank from 318 to 214 bytes
and the matrix dispatcher from 392 to 250 bytes. Debug function sizes were
unchanged. Disassembly retains register saves/restores, `mret`, the
128-byte vector table, and no undefined helper calls. The UART and CFIFO
sources also cross-compiled at both optimization levels. These are source
compilation checks, not complete Eclipse firmware builds or hardware results.

Clean-build the C3 library and UartLoopback in Release and repeat the same
PRBS command to measure this change. No throughput gain is claimed yet.

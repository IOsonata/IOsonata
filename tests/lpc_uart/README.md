# Shared legacy LPC UART checks

Run from the repository root on Linux with Python 3 and g++ installed:

```sh
python3 tests/lpc_uart/run.py
```

The test compiles the common UART implementation twice, using the LPC11Uxx
and LPC17xx vendor device headers and the corresponding family source.
CMSIS core operations and UART register side effects are simulated. The
real CFIFO and generic DeviceIntrf implementation are included. Undefined
behavior sanitization is enabled.

Checks cover:

- Nearest baud selection at fixed 16x sampling, checked against all legal
  divisors and fractions for selected clocks and requested rates.
- Previously failing high baud rates, minimum and maximum rate requests,
  and zero for automatic baud detection.
- Polling TX across multiple hardware FIFO bursts and a bounded stalled TX.
- Interrupt TX, RX, FIFO overflow, error counters and callbacks.
- Parity, reset, enable/disable, instance ownership and independent FIFOs.
- LPC17xx UART1 IRQ selection and hardware flow-control restrictions.
- Explicit rejection of unsupported DMA configuration.

The LPC11U35 PRBS TX example selects interrupt mode and disables DMA.
The shared implementation supports asynchronous full-duplex UART. It does
not implement synchronous, RS-485 half-duplex, software flow-control or DMA
operation. LPC546xx Flexcomm remains a separate implementation.

Host tests do not validate pin muxing, electrical timing, interrupt latency,
IrDA waveforms, modem handshakes or silicon auto-baud behavior. ARM object
compilation is a separate check; a full firmware build and board tests are
still required before claiming hardware validation.

LPC11Uxx pin validation runs before clocks, registers, FIFOs, IRQs or ownership
are changed. Supplied pins must match their signal's route; duplicate pads
are rejected. Negative port/pin entries remain unconnected, but hardware flow
control requires both CTS and RTS. Pin maps contain at most UART_NB_PINS entries.
The generic DTE slot is the DSR input. Select pads available on the package.
Routes follow NXP UM10462 chapter 7:

| Signal | Routes |
| --- | --- |
| RX | P0.18 FUNC1, P1.14 FUNC3, P1.17 FUNC2, P1.26 FUNC2 |
| TX | P0.19 FUNC1, P1.13 FUNC3, P1.18 FUNC2, P1.27 FUNC2 |
| CTS | P0.7 FUNC1 |
| RTS | P0.17 FUNC1 |
| DCD | P1.15 FUNC1, P1.21 FUNC1 |
| DSR | P1.14 FUNC1, P1.20 FUNC1 |
| DTR | P1.13 FUNC1, P1.19 FUNC1 |
| RI | P1.16 FUNC1, P1.22 FUNC1 |

Tests cover all RX/TX combinations and modem-route combinations, wrong
functions, invalid pads, swapped RX/TX, missing hardware-flow pins, oversized
maps and overlapping data/modem pads. Rejected maps preserve driver state,
queued bytes, registers and IRQ state and make no pin configuration calls.
These checks do not replace electrical or hardware handshake validation.

Interrupt mode selects the nearest implemented NVIC priority: negative values
become zero and excessive values become the maximum defined by the family's
__NVIC_PRIO_BITS. LPC11Uxx therefore clamps to 0..3. The effective priority is
stored in DevIntrf.IntPrio and retained across Disable/Enable; the caller's
configuration is unchanged. Polling ignores IntPrio and never programs NVIC
priority. Both family tests cover every valid level and out-of-range values,
including INT_MIN and INT_MAX.

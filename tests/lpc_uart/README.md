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

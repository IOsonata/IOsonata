# STM32 port regression checks

Run from a Linux checkout with GCC, G++ and Python 3:

```
python3 tests/stm32f0/run.py
python3 tests/stm32f4/run.py
```

These compile the production drivers against the repository's ST device headers.
The core headers, NVIC operations and delays are host shims; peripheral addresses
are mapped into the test process. The model checks data flow and register fields,
not peripheral timing, automatic flag clearing, interrupt preemption, oscillator
startup or ARM instructions. No extra Python packages are required.

F0 coverage: F030x6/x8/xC and F070x6/xB; interrupt RX without phantom bytes,
RX error discard, TX interrupt arming, parity/word length/stop bits, unsupported
DMA rejection, polling transfers, shared USART dispatch, pin-specific EXTI
allocation, AHB/APB clock calculations. F4 coverage: F401 startup selects the
linked vector symbol instead of forcing FLASH_BASE.

Hardware validation remains required: UART loopback and framing checks, EXTI
on a nonzero pin, external oscillator startup, and F401 application interrupts
when launched from the DFU slot at 0x08010000. Full ARM firmware builds were not
run in the host validation environment.

Port scope remains partial. F0 supplies UART/GPIO/startup; this change does not
add missing I2C/SPI/timer/ADC/USB implementations. F3 remains vector scaffolding,
and F4 startup/DFU support is not a complete peripheral port. F0 UART supports
polling or FIFO interrupts, 7-bit data with parity and 8-bit data with or without
parity, and one or two stop bits. DMA, synchronous mode, IrDA and software flow
control requests fail initialization rather than reporting false success.

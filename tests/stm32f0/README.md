# STM32 port regression checks

Run from a Linux checkout with GCC, G++ and Python 3:

```
python3 tests/stm32f0/run.py
python3 tests/stm32/run_uart.py
python3 tests/stm32f0/run_timer.py
python3 tests/stm32f0/run_timer_arm.py
python3 tests/stm32f4/run.py
```

These compile the production drivers against the repository's ST device headers.
The core headers, NVIC operations and delays are host shims; peripheral addresses
are mapped into the test process. No extra Python packages are required.

`tests/stm32f0/run.py`: F030x6/x8/xC and F070x6/xB system clock calculation and
pin-specific EXTI allocation with the shared GPIO driver
(`ARM/ST/src/iopincfg_stm32.cpp`). `tests/stm32f4/run.py`: F401 startup selects
the linked vector symbol instead of forcing FLASH_BASE.

## UART

`tests/stm32/run_uart.py` runs the STM32 UART driver shared by all families,
`ARM/ST/src/uart_stm32.cpp`, for F030x6, F030x8, F030xC, F070x6, F070xB,
F091xC, L432, L476, L496, L4S9, F401xC, F405, F413 and WBA65, at -O0 and -O2
with UBSan. The USART, DMA and RCC pages of the test process are protected:
each register access of the driver stops, runs as a single step, then a model
applies its side effects (RDR read clears RXNE and loads the next character,
ICR and DMA flag clear registers, TDR write to the wire, SR and DR clearing on
F4, GPDMA suspend and reset, RCC reset of the instance). The test moves the DMA
transfers itself: TX blocks to the wire, RX bytes into the ring with the half
and full flags, the circular reload or the GPDMA linked list node. Linux x86-64
only, the single step uses the trap flag.

For each instance, in the DevNo order of the driver: RCC enable and reset bits,
kernel clock selection, vector and NVIC number, the PCLK used for the rate,
polling and interrupt TX and RX, RX framing and overrun errors, full RX FIFO,
Disable and Enable. Where the driver has DMA: the channel or stream and its
request (CSELR, DMAMUX1, stream channel or GPDMA request, against the reference
manual tables), peripheral addresses, circular RX, TX blocks with FIFO wrap and
back pressure, the RX ring with half, full and IDLE moves, a TX transfer error,
polled DMA, a channel already set up by another driver, and interrupt mode after
DMA. Instances without DMA refuse it. Framing on USART1, LPUART1 LSE, HSI16 and
PCLK selection. With arm-none-eabi-g++ found, the driver is also compiled for
each part at -O0 and -Os with warnings as errors.

Not a timing test: no baud accuracy, throughput, interrupt preemption or DMA bus
timing. Hardware validation remains required.

## Other checks

The shared SLIP RX example regression (`python3 tests/stm32f0/run_slip_example.py`)
uses the production decoder with buffered and fragmented fake UART input. It
covers empty frames, exact-buffer and oversized frames, split escape sequences,
and PRBS continuity under UBSan. This does not model UART interrupt timing.

F030x8 supplies startup, GPIO, UART and all seven peripheral timers. See
[port notes](../../ARM/ST/STM32F0xx/README.md) for timer coverage and limits.
Other F0 variants retain their existing scope. I2C/SPI/ADC/USB implementations
are not added here. F3 remains vector scaffolding, and F4 startup/DFU support is
not a complete peripheral port.

Hardware validation remains required for EXTI on a nonzero pin, external
oscillator startup, and F401 application interrupts when launched from the DFU
slot at 0x08010000.

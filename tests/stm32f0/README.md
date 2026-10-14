# STM32 port regression checks

Run from a Linux checkout with GCC, G++ and Python 3:

```
python3 tests/stm32f0/run.py
python3 tests/stm32/run_uart.py
python3 tests/stm32/run_timer.py
python3 tests/stm32f4/run.py
python3 tests/stm32g4/run.py
python3 tests/stm32wba/run.py
```

These compile the production drivers against the repository's ST device headers.
The core headers, NVIC operations and delays are host shims; peripheral addresses
are mapped into the test process. No extra Python packages are required.

`tests/stm32f0/run.py`: F030x6/x8/xC and F070x6/xB system clock calculation and
pin-specific EXTI allocation with the shared GPIO driver
(`ARM/ST/src/iopincfg_stm32.cpp`). `tests/stm32f4/run.py`: F401 startup selects
the linked vector symbol instead of forcing FLASH_BASE. `tests/stm32g4/run.py`: G474
system clock, see the [G4 port notes](../../ARM/ST/STM32G4xx/README.md).
`tests/stm32wba/run.py`: WBA65 LPTIM1 and LPTIM2 with the shared timer
driver, see the [WBA notes](../../ARM/ST/STM32WBAxx/README.md).

## UART

`tests/stm32/run_uart.py` runs the STM32 UART driver shared by all families,
`ARM/ST/src/uart_stm32.cpp`, for F030x6, F030x8, F030xC, F070x6, F070xB,
F091xC, L432, L476, L496, L4S9, G474, G431, F401xC, F405, F413 and WBA65, at
-O0 and -O2
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

## Timer

`tests/stm32/run_timer.py` runs the STM32 timer driver shared by all families,
`ARM/ST/src/timer_stm32.cpp`, `timer_lptim_stm32.cpp` and `timer_tim_stm32.cpp`.

`tests/stm32/timer_tim_test.cpp` runs TimerInit and the TIM driver with a
model of the TIM registers (CNT wrap on read, SR clear by writing 0) for
F030x8, F072xB, F401xC, F446, L432, L476, L496, L4S9, G474 and WBA65, at -O0
and -O2 with UBSan: the DevNo order of `ARM/ST/include/timer_stm32.h`, RCC
enable and reset bits, timer clock from PCLK, the NVIC number of each vector,
DevNo and IntPrio refusal, the four compare triggers of a TIM with the counter
wrapping between two reads, deadlines past 65536 ticks, a late interrupt,
trigger disable, frequency change, the basic timer period, the vectors shared
by two timers and the move of a TimerDev_t to another timer. A second build
puts an application handler on one timer vector: TimerInit refuses the timers
on that vector.

`tests/stm32/timer_lptim_test.cpp` runs the LPTIM of L432, L476, L4S9 and
G474 with the registers as plain memory at their addresses: clock selection
and prescaler, ARR and CMP write order with ARROK and CMPOK, tick count
across the counter wrap, continuous and single triggers, Disable and Enable.
The WBA LPTIM is run by `tests/stm32wba/run.py`.

The runner then checks that the vector file of each library aliases the timer
vectors of its part to `Stm32TimerVector`. With arm-none-eabi-g++ found, the
three sources are compiled for every F0, F4, L4, G4 and WBA device that
`stm32.h` selects, at -O0 and -Os with warnings as errors, and F030x8, L476
and WBA65 images are linked from the driver archive: without TimerInit the
timer driver is not linked, with an application TIM3 handler the alias is
replaced.

Not a timing test. Hardware validation remains required.

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

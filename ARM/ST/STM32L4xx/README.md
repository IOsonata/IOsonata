# STM32L4 port status

Library projects exist for STM32L476, STM32L496 and STM32L4S9.
An MCU needs startup, UART and timer support to be considered supported.
Peripheral source files and project metadata alone do not establish this.
Hardware validation is recorded separately from implementation coverage.

## Virtual device numbers

UART indices are 0 for LPUART1, 1 for USART1, 2 for USART2, 3 for USART3,
4 for UART4 and 5 for UART5. IRQ selection, reset and clock control use this
mapping. These indices are not hardware instance numbers.

UART supports seven or eight payload bits, none/even/odd parity and one or
two stop bits. LPUART1 cannot use seven payload bits without parity. Invalid
framing and nine-bit payload requests fail before device state is changed.
When the receive FIFO is full, the ISR consumes and counts the dropped byte;
foreground reads drain only the software FIFO.

SPI indices 0-2 select SPI1-3. On L476/L496, index 3 selects QUADSPI.
On L4S9, indices 3 and 4 select OCTOSPI1 and OCTOSPI2. Extended SPI
controllers currently accept polling master configurations only; interrupt,
DMA and slave requests fail initialization.

## Timer and I2C limits

Timer indices 0 and 1 select the existing LPTIM1 and LPTIM2 implementation.
General-purpose TIM indices remain reserved but are not implemented.
Their initialization returns false without publishing an instance or changing
registers; the high-frequency timer availability count is zero.

I2C currently accepts polling master configurations only. Slave, interrupt
and DMA requests return false before changing the device or hardware state.
L4S9 I2C4 uses virtual index 3.

These restrictions make unfinished paths explicit; they do not certify the
remaining transfer implementations or LPTIM behavior on hardware.

## Validation

Run the focused host regressions from the repository root:

```sh
python3 tests/stm32l4/run.py
```

The tests compile extracted production function bodies against small register
and dispatch models, with UBSan. They cover UART IRQ/NVIC/reset/clock mapping,
SPI versus extended-controller dispatch, OSPI clock masks and prescaler limits,
I2C configuration refusal, and TIM refusal while preserving LPTIM dispatch.
They also cover direct/PLL clock sources and MSI range selection in both
startup files, UART framing and RX overflow, and restoring LPTIM overflow
interrupts across disable/enable cycles.
They do not simulate the complete UART, I2C, SPI or timer hardware.

The changed UART, SPI, I2C, timer-dispatch and TIM translation units compile
for L476, L496 and L4S9 with Arm GNU 14.3.1, GNU C++17, Cortex-M4 hard-float,
no exceptions/RTTI, at -O0 and -Os. The changed OSPI translation unit also
compiles for L4S9. Startup and LPTIM compile for all three targets at both
optimization levels; the alternative system_stm32l4plus.c compiles for L4S9
(the L4S9 project uses system_stm32l4xx.c). These are source compilation
checks with staged headers,
not complete IOC library/application links or on-board tests.

Before claiming a release validation baseline, verify startup clocks, UART
traffic and LPTIM ticks/triggers on the named MCU and board. Record the tested
revision, toolchain and results. Other peripheral modes require their own
validation.

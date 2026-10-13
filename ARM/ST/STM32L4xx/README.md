# STM32L4 port status

STM32L476, STM32L496 and STM32L4S9 are supported at the minimum MCU level:
startup, GPIO, UART and LPTIM1/2 are implemented. Library projects
exist for each MCU. The maintainer confirms these STM32L4 ports are hardware
validated and used in existing projects (2026-10-08). The UART was then the
L4 copy `src/uart_stm32l4xx.cpp`. The libraries now build the STM32 UART
driver shared with the other families, `ARM/ST/src/uart_stm32.cpp`, and the
L4 copy is removed; the shared driver has not been run on L4 hardware yet.
The automated checks
below describe the scope of the recent source review; they do not mean the
ports lack hardware validation. See the
[central MCU support list](../../../docs/supported-targets.md).
An MCU needs startup, GPIO, UART and timer support to be considered supported.
Peripheral source files and project metadata alone do not establish this.
Hardware validation is recorded separately from implementation coverage.

STM32L432 (STM32L432KC on the NUCLEO-L432KC) has a library project and two
UART example projects built from the same L4 sources (2026-10-13). It is not
tested on hardware yet; see the STM32L432 paragraph under Validation.

## Virtual device numbers

UART indices are 0 for LPUART1, 1 for USART1, 2 for USART2, 3 for USART3,
4 for UART4 and 5 for UART5. IRQ selection, reset and clock control use this
mapping. These indices are not hardware instance numbers. The STM32L432 has
LPUART1, USART1 and USART2 only; indices 3 to 5 fail initialization.

UART supports seven or eight data bits, none/even/odd parity and one or
two stop bits on every instance, LPUART1 included, in polling, interrupt or
DMA mode. Other framing, nine data bits, synchronous mode and IrDA fail
initialization before the device is changed. USART1 to UART5 run from PCLK,
the driver writes their CCIPR kernel clock selection to 0. LPUART1 selects
LSE, HSI16 or PCLK1 for the requested rate. A character received with the RX
CFIFO full is dropped and counted in `RxDropCnt` with a blocking CFIFO
(`bFifoBlocking`); a non-blocking CFIFO drops its oldest byte instead.

UART DMA (`bDMAMode`, with or without `bIntMode`) sends TX blocks straight
from the TX CFIFO and receives into a circular ring moved into the RX CFIFO,
as on the F0 parts. With a non-blocking CFIFO each TX block, at most 32
bytes, is a copy taken out of the CFIFO, see `ARM/ST/STM32F0xx/README.md`
(STM32L432 in the emulator at 80 MHz and 1 Mbaud: loopback 96.6 kB/s, 99.0
with a blocking CFIFO). Each channel has its own vector, defined by the UART
driver. L476 and L496 select the request in the DMA1 or DMA2 CSELR, L4+ parts
route it through DMAMUX1 (DMA1 channel n on DMAMUX1 channel n - 1, DMA2
channel n on channel n + 6). Not tested on hardware.

| DevNo | Hardware | TX DMA | RX DMA | CSELR | DMAMUX1 TX, RX |
|---|---|---|---|---|---|
| 0 | LPUART1 | DMA2 channel 6 | DMA2 channel 7 | 4 | 35, 34 |
| 1 | USART1 | DMA1 channel 4 | DMA1 channel 5 | 2 | 25, 24 |
| 2 | USART2 | DMA1 channel 7 | DMA1 channel 6 | 2 | 27, 26 |
| 3 | USART3 | DMA1 channel 2 | DMA1 channel 3 | 2 | 29, 28 |
| 4 | UART4 | DMA2 channel 3 | DMA2 channel 5 | 2 | 31, 30 |
| 5 | UART5 | DMA2 channel 1 | DMA2 channel 2 | 2 | 33, 32 |

The DMAMUX1 numbers are those of the L4R and L4S; the L4P and L4Q use one
more.

SPI indices 0-2 select SPI1-3. On L432/L476/L496, index 3 selects QUADSPI.
On L4S9, indices 3 and 4 select OCTOSPI1 and OCTOSPI2. The L432 has no SPI2:
index 1 fails initialization. Extended SPI
controllers currently accept polling master configurations only; interrupt,
DMA and slave requests fail initialization.

## Timer and I2C limits

Timer indices 0 and 1 select the existing LPTIM1 and LPTIM2 implementation.
General-purpose TIM indices remain reserved but are not implemented.
Their initialization returns false without publishing an instance or changing
registers; the high-frequency timer availability count is zero.

I2C currently accepts polling master configurations only. Slave, interrupt
and DMA requests return false before changing the device or hardware state.
L4S9 I2C4 uses virtual index 3. The L432 has no I2C2: index 1 fails
initialization, I2C1 and I2C3 keep indices 0 and 2. The timer table has no
register for TIM3, TIM4, TIM5, TIM8 and TIM17 on the L432.

These restrictions identify unsupported modes within the hardware-validated
ports. Hardware validation does not imply that every optional mode is implemented.

## Validation

Run the focused host regressions from the repository root:

```sh
python3 tests/stm32l4/run.py
python3 tests/stm32/run_uart.py
```

`tests/stm32l4/run.py` compiles extracted production function bodies against
small register and dispatch models, with UBSan. It covers SPI versus
extended-controller dispatch (L432, L476, L496 and L4S9), OSPI clock masks
and prescaler limits, I2C configuration refusal, the refusal of an instance
the MCU does not have (SPI2 and I2C2 on the L432), and TIM refusal while
preserving LPTIM dispatch. It
also covers direct/PLL clock sources and MSI range selection in both startup
files, and restoring LPTIM overflow interrupts across disable/enable cycles.
It does not simulate the complete I2C, SPI or timer hardware.

`tests/stm32/run_uart.py` runs the shared UART driver with the L432, L476,
L496 and L4S9 device headers and a model of the USART, DMA and RCC registers:
instance clocks, reset and kernel clock selection, vectors and NVIC numbers,
polling, interrupt and DMA transfers, the CSELR and DMAMUX1 requests, RX
errors and LPUART1 clock selection. See
[tests/stm32f0/README.md](../../../tests/stm32f0/README.md).

The changed SPI, I2C, timer-dispatch and TIM translation units compile
for L476, L496 and L4S9 with Arm GNU 14.3.1, GNU C++17, Cortex-M4 hard-float,
no exceptions/RTTI, at -O0 and -Os. The changed OSPI translation unit also
compiles for L4S9. Startup and LPTIM compile for all three targets at both
optimization levels; the alternative system_stm32l4plus.c compiles for L4S9
(the L4S9 project uses system_stm32l4xx.c). These are source compilation
checks with staged headers,
not complete IOC library/application links or on-board tests.

UART hardware run: the STM32L476 examples UartLoopback
(`exemples/uart/uart_loopback.cpp`) and UartPrbsTxTest
(`exemples/uart/uart_prbs_tx.cpp`) select their board with `#define BOARD` in
`board.h`, from the pin maps of `ARM/ST/include/stm32_boards.h`. The project
builds for B_L475E_IOT01A (the default, STM32L475VG: UART4, DevNo 4, on
Arduino D0 PA1 and D1 PA0, AF8; ST-LINK virtual COM port on USART1 PB6/PB7)
and STM32L476G_DISCO (USART2, DevNo 2, on PD5 TX and PD6 RX, AF7, also the
virtual COM port). The STM32L476 library leaves out the old `src/sysstatus.c`,
which no longer compiles with the current `sysstatus.h`. The library and these
two examples were built for Debug and Release from their `.project` and
`.cproject` settings with GNU Arm 13.2, outside IOcomposer. The other L476
examples still name `stm32l476xx.ld`, which is not in `ldscript/`
(`gcc_stm32l476xx.ld` is), and their board files pair DevNo 1 with the USART2
pins and PinOp `0x72`, which the GPIO driver does not take as AF7. The L496
and L4S9 libraries still build `src/sysstatus.c`.

STM32L432: `STM32L432/lib/ioc` has the Debug, Release, Debug_FPU and
Release_FPU configurations of the L476 library. It adds the STM32L432xx
vector table, `STM32L432/lib/src/Vector_STM32L432xx.c`, and
`ldscript/gcc_stm32l432xx.ld`: 240 KB of the 256 KB flash for the image, two
8 KB storage partitions above it, 48 KB SRAM1 as RAM and the 16 KB SRAM2 at
0x10000000 as RAM2. SystemInit runs the core at 80 MHz from the MSI PLL, with
PCLK1 and PCLK2 at 40 MHz, and starts the 32.768 kHz LSE crystal by default
(`g_McuOsc`), waiting for it to run. The I2C, SPI and timer sources leave out
the instances the L432 does not have; the code generated for the L476, L496
and L4S9 differs only by the new check in I2CInit and SPIInit.

The STM32L432 examples UartLoopback and UartPrbsTxTest select NUCLEO_L432KC
in `board.h`: USART1, DevNo 1, on Arduino Nano D0 PA10 RX and D1 PA9 TX, AF7.
The ST-LINK virtual COM port is USART2, DevNo 2, RX PA15 AF3 and TX PA2 AF7.
Like the L476 examples they link `rdimon` semihosting and use the example
defaults at 1 Mbaud: interrupts without DMA for the loopback, DMA without
interrupts for PRBS TX. The library in its four configurations and both
examples in Debug and Release were built from their `.project` and `.cproject`
settings with GNU Arm 13.2, outside IOcomposer; all 99 vector table entries
match the STM32L432xx interrupt list. The images ran in an instruction set
emulator with USART1, DMA1 (channels 4 and 5, CSELR request 2) and RCC models:
the loopback echoes in order with no lost character in polling, interrupt,
DMA without interrupts and DMA with interrupts, and the PRBS output matches
`Prbs8()`. This does not replace a run on the board.

For a new release-candidate run, record the project, MCU/board, revision,
toolchain and exercised modes alongside the existing hardware-validation
record. Validate changes affecting clocks, GPIO, UART or LPTIM on the relevant
project; optional peripheral modes retain their documented limits.

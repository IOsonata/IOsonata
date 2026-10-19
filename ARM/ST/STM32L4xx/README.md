# STM32L4 port status

STM32L476, STM32L496 and STM32L4S9 are supported at the minimum MCU level:
startup, GPIO, UART and timers are implemented. Library projects
exist for each MCU. The maintainer confirms these STM32L4 ports are hardware
validated and used in existing projects (2026-10-08). The UART was then the
L4 copy `src/uart_stm32l4xx.cpp` and the timer the L4 LPTIM copy
`src/timer_lptim_stm32l4xx.cpp`. The libraries now build the STM32 UART and
timer drivers shared with the other families, `ARM/ST/src/uart_stm32.cpp` and
`ARM/ST/src/timer_*stm32.cpp`, and the L4 copies are removed; the shared
drivers have not been run on L4 hardware yet.
The automated checks
below describe the scope of the recent source review; they do not mean the
ports lack hardware validation. See the
[central MCU support list](../../../docs/supported-targets.md).
An MCU needs startup, GPIO, UART and timer support to be considered supported.
Peripheral source files and project metadata alone do not establish this.
Hardware validation is recorded separately from implementation coverage.

STM32L432 (STM32L432KC on the NUCLEO-L432KC) has a library project and the
example projects Blinky, TimerDemo, UartLoopback and UartPrbsTxTest built from
the same L4 sources (2026-10-13, Blinky 2026-10-19). It is not tested on
hardware yet; see the STM32L432 paragraph under Validation.

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
as on the F0 parts. With a non-blocking CFIFO each TX block, at most 32 bytes,
is a copy taken out of the CFIFO, see `ARM/ST/STM32F0xx/README.md` (STM32L432
in the emulator at 80 MHz and 1 Mbaud: loopback 96.6 kB/s, 99.0 with a
blocking CFIFO). Each channel has its own vector. Since 2026-10-19 the vectors
belong to the DMA channel manager `ARM/ST/src/dma_stm32.cpp`, which all four
L4 libraries build: a driver claims the channels the reference manual gives
its request, the first to ask has them (UART, SPI and I2C). L476 and L496
select the request in the DMA1 or DMA2 CSELR, L4+ parts route it through
DMAMUX1 (DMA1 channel n on DMAMUX1 channel n - 1, DMA2 channel n on DMAMUX1
channel n + 6). On the NUCLEO-L432KC (2026-10-19), UartPrbsTxTest on USART2,
the ST-LINK virtual COM port, at 1 Mbaud: DMA without interrupts and DMA with
interrupts 99.4 kB/s, interrupts without DMA 99.9 kB/s, no loss after the
start of the stream.

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

The STM32L432 library builds the shared STM32 SPI driver,
`ARM/ST/src/spi_stm32.cpp`, in place of `src/spi_stm32l4xx.cpp` (2026-10-19):
SPI1 and SPI3 (DevNo 0 and 2), master in polling and interrupt mode, slave in
interrupt mode with the hardware NSS pin, 4 to 16 bit data, the rate from the
BR divider. DMA for master and slave through the DMA channel manager: SPI1 TX
DMA1 channel 3 and RX channel 2 (CSELR 1), or DMA2 channels 4 and 3 (CSELR 4)
when those are taken; SPI2 DMA1 channels 5 and 4; SPI3 DMA2 channels 2 and 1
(CSELR 3). QUADSPI (DevNo 3) is not in the L432 library for now: SPIInit
returns false for it. The STM32L432 example SPIMasterSlave runs SPI1 as master
(A1 PA1 SCK, D10 PA11 MISO, A6 PA7 MOSI, D3 PB0 chip select GPIO) against SPI3
as slave (D13 PB3 SCK, D12 PB4 MISO, D11 PB5 MOSI, A3 PA4 NSS) on one
NUCLEO-L432KC, wired A1 to D13, A6 to D11, D10 to D12 and D3 to A3. The A6 and
D12 lines are shared with the I2CMasterSlave wiring. On the NUCLEO-L432KC
(2026-10-19), master polling and master interrupt, each against the interrupt
driven slave, pass in SPI modes 0 to 3 at 625 kHz, 8 bit data: master transmit
and receive frames of 1, 16 and 129 bytes, and a 1 byte frame that discards
the slave transmit prefetch before the next frame. With DMA (2026-10-19) the
same test passes in all 8 combinations: master polling, interrupt, DMA and DMA
with interrupt, each against the interrupt driven slave and the DMA slave. The
L476, L496 and L4S9 libraries keep `src/spi_stm32l4xx.cpp`.

## Timer

The libraries build the STM32 timer driver: `ARM/ST/src/timer_stm32.cpp`
(TimerInit), `timer_lptim_stm32.cpp` (LPTIM) and `timer_tim_stm32.cpp` (TIM),
described in `ARM/ST/include/timer_stm32.h`. Timer DevNo are virtual
indices, the LPTIM first (`TimerGetLowFreqDevCount` 2), then the TIM from the
basic timers to the advanced timers:

| DevNo | L476, L496, L4S9 | L432 | Triggers |
|---|---|---|---|
| 0 | LPTIM1 | LPTIM1 | 1 |
| 1 | LPTIM2 | LPTIM2 | 1 |
| 2 | TIM6 | TIM6 | 1, the counter period |
| 3 | TIM7 | TIM7 | 1, the counter period |
| 4 | TIM16 | TIM16 | 1 |
| 5 | TIM17 | TIM15 | 1 (TIM17), 2 (TIM15) |
| 6 | TIM15 | TIM2 | 2 (TIM15), 4 (TIM2) |
| 7 | TIM2 | TIM1 | 4 |
| 8 to 12 | TIM3, TIM4, TIM5, TIM1, TIM8 | | 4 |

An LPTIM counts at its kernel clock, LSI, LSE (crystal or bypass with
`OSC_TYPE_TCXO`) or HSI16, divided by 1 to 128, 16 bits extended to 64 in
software; one compare trigger (CMP). CFGR and IER are written with the timer
disabled, ARR and CMP with the timer enabled, confirmed by ARROK and CMPOK.
A continuous trigger is scheduled one period after its previous compare
value. A TIM counts at the APB timer clock divided by 1 to 65536,
`TIMER_CLKSRC_DEFAULT` only; its triggers are its compare channels, a
deadline longer than the counter period is kept in software. External
trigger and the tick interrupt are not supported. The old L4 LPTIM driver
also took an external trigger; no IOsonata code used it, it wrote CFGR with
the timer enabled, and its timeout mode restarts the counter of the 64 bits
count.

The timer vectors of the vector file are weak aliases of
`Stm32TimerVector`: TimerInit refuses a timer whose vector is replaced by an
application handler (TIM6 shares its vector with the DAC). The details are
in `ARM/ST/STM32F0xx/README.md` and `ARM/ST/include/timer_stm32.h`.

## I2C limits

The STM32L432 library builds the shared STM32 I2C driver,
`ARM/ST/src/i2c_stm32.cpp`, in place of `src/i2c_stm32l4xx.cpp` (2026-10-19):
master in polling and interrupt mode, slave in interrupt mode, 7 bit
addresses. DMA for master and slave through the DMA channel manager: I2C1 TX
DMA1 channel 6 and RX channel 7 (CSELR 3), or DMA2 channels 7 and 6 (CSELR 5)
when those are taken; I2C2 DMA1 channels 4 and 5; I2C3 DMA1 channels 2 and 3
(CSELR 3); I2C4 DMA2 channels 2 and 1 (CSELR 0). The STM32L432 example
I2CMasterSlave runs I2C1 (DevNo 0, D1 PA9 SCL, D0 PA10 SDA) as master against
I2C3 (DevNo 2, A6 PA7 SCL, D12 PB4 SDA) as slave on one NUCLEO-L432KC, wired
D1 to A6 and D0 to D12. The Release build prints on the ST-LINK virtual COM
port at 1 Mbaud. On the NUCLEO-L432KC (2026-10-19), master polling and master
interrupt, each against the interrupt driven slave, pass: write 8 bytes at an
offset, read them back, and a 4 byte read, at 100 kHz with the internal
pull-ups. With DMA (2026-10-19) the same test passes in all 8 combinations:
master polling, interrupt, DMA and DMA with interrupt, each against the
interrupt driven slave and the DMA slave.

The L476, L496 and L4S9 libraries keep `src/i2c_stm32l4xx.cpp`, which
accepts polling master configurations only. Slave, interrupt and DMA requests
return false before changing the device or hardware state.
L4S9 I2C4 uses virtual index 3. The L432 has no I2C2: index 1 fails
initialization, I2C1 and I2C3 keep indices 0 and 2.

These restrictions identify unsupported modes within the hardware-validated
ports. Hardware validation does not imply that every optional mode is implemented.

## Validation

Run the focused host regressions from the repository root:

```sh
python3 tests/stm32l4/run.py
python3 tests/stm32/run_uart.py
python3 tests/stm32/run_timer.py
```

`tests/stm32l4/run.py` compiles extracted production function bodies against
small register and dispatch models, with UBSan. It covers SPI versus
extended-controller dispatch (L432, L476, L496 and L4S9), OSPI clock masks
and prescaler limits, I2C configuration refusal, and the refusal of an
instance the MCU does not have (SPI2 and I2C2 on the L432). It
also covers direct/PLL clock sources and MSI range selection in both startup
files. It does not simulate the complete I2C or SPI hardware.

`tests/stm32/run_timer.py` runs the shared timer driver: the TIM with a
register model and the LPTIM with plain memory registers, for L432, L476,
L496 (TIM) and L4S9, and checks the timer vectors of each vector file, see
[tests/stm32f0/README.md](../../../tests/stm32f0/README.md).

`tests/stm32/run_uart.py` runs the shared UART driver with the L432, L476,
L496 and L4S9 device headers and a model of the USART, DMA and RCC registers:
instance clocks, reset and kernel clock selection, vectors and NVIC numbers,
polling, interrupt and DMA transfers, the CSELR and DMAMUX1 requests, RX
errors and LPUART1 clock selection. See
[tests/stm32f0/README.md](../../../tests/stm32f0/README.md).

The changed SPI and I2C translation units compile
for L476, L496 and L4S9 with Arm GNU 14.3.1, GNU C++17, Cortex-M4 hard-float,
no exceptions/RTTI, at -O0 and -Os. The changed OSPI translation unit also
compiles for L4S9. Startup compiles for all three targets at both
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
virtual COM port). The old `src/sysstatus.c` is retired; `syslog.cpp` and `sysstatus.h` replace it.
The STM32L476 library and these
two examples were built for Debug and Release from their `.project` and
`.cproject` settings with GNU Arm 13.2, outside IOcomposer. The other L476
examples, except TimerDemo, still name `stm32l476xx.ld`, which is not in
`ldscript/` (`gcc_stm32l476xx.ld` is), and their board files pair DevNo 1
with the USART2 pins and PinOp `0x72`, which the GPIO driver does not take as
AF7.

Timer hardware run: TimerDemo (`exemples/timer/timer_demo.cpp`), in
`STM32L476/exemples/TimerDemo` for B_L475E_IOT01A and
`STM32L432/exemples/TimerDemo` for NUCLEO_L432KC. Output on the ST-LINK
virtual COM port at 115200 8N1 (USART1 PB7/PB6 on the B-L475E-IOT01A,
USART2 PA15/PA2 on the NUCLEO-L432KC), the first LED toggles on trigger 0.
`TIMER_DEMO_DEVNO` in `board.h` selects the timer, LPTIM1 by default, from
the LSE crystal; the TIM run at 100 kHz. Expected output, LPTIM1:

```
Timer 0 trigger 0: 100 ms
Count = 100 ms, T0 = 100097 us
Count = 200 ms, T0 = 100006 us
...
```

100 ms is 3277 ticks of the LSE, 100.006 ms. A four channel TIM prints
triggers 0 to 3 (100 ms, 1 s, 250 ms, 500 ms) and their periods. The
STM32L496 and STM32L4S9 TimerDemo projects keep their board files, without
UART output. The four TimerDemo projects build for Debug and Release; in an
instruction set emulator with LPTIM, TIM, RCC and UART models the L432 image
runs each of its eight timers and the L476 image LPTIM1, LPTIM2, TIM6, TIM17,
TIM5, TIM1 and TIM8 for 3.3 s, with the expected periods and no LPTIM write
order violation. Not tested on the boards.

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

The STM32L432 Blinky example (`exemples/misc/blinky.c`) selects NUCLEO_L432KC
in `board.h` and blinks LD3, PB3. The board has no user button, so the LED
blinks until reset. It links `nosys`, no semihosting, and runs without a
debugger. Debug and Release were built from the `.project` and `.cproject`
settings with xPack GNU Arm 15.2 outside IOcomposer. In an instruction set
emulator with an RCC model the image sets PB3 as output, runs the core from
the MSI PLL at 80 MHz and toggles PB3 at a fixed interval. Not tested on the
board.

For a new release-candidate run, record the project, MCU/board, revision,
toolchain and exercised modes alongside the existing hardware-validation
record. Validate changes affecting clocks, GPIO, UART or timers on the
relevant project; optional peripheral modes retain their documented limits.

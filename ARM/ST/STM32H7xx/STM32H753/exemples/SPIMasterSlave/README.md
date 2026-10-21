# NUCLEO-H753ZI SPI master/slave test

Connect these four jumpers with the board powered off:

| Signal | SPI1 master (DevNo 0) | SPI3 slave (DevNo 2) |
|---|---|---|
| SCK | PA5, D13, CN7 pin 10 | PC10, D45, CN8 pin 6 |
| MISO | PA6, D12, CN7 pin 12 | PC11, D46, CN8 pin 8 |
| MOSI | PB5, D11, CN7 pin 14 | PC12, D47, CN8 pin 10 |
| CS/NSS | PD14, D10, CN7 pin 16 | PA15, D20, CN7 pin 9 |

The master uses AF5 and GPIO CS; the slave uses AF6 and hardware NSS.
PB5 is the default D11 routing (SB33 off, SB35 on). Keep that routing;
PA7 is connected to the Ethernet PHY. No second board is required.

The test requests 1 MHz, 8-bit MSB-first transfers in modes 0, 1, 2 and 3.
Each enabled master/slave combination covers 1-, 16- and 129-byte transfers,
read/write patterns, completion counts and short CS boundaries.

Import this directory's `ioc` project and `STM32H753/lib/ioc` into IOcomposer.
Build the MCU library first, then this example, both in Release. The library
name is `IOsonata_STM32H753`; the configuration is selected by its directory.
Open the ST-LINK virtual COM port at **1000000 baud, 8N1**, then reset the board.
Debug uses semihosting: enable `monitor arm semihosting enable` in the debugger
startup commands. Use Release for the hardware test.

The project links the shared IOsonata example; `src/board.h` supplies only
board mappings and test configuration. Four master modes (polling, interrupt,
DMA polling, DMA interrupt) run against both interrupt and DMA slave modes.
Slave priority 6 can preempt master priority 7. SPI additionally covers modes
0-3. Expected final summary: **8 passed, 0 failed, 0 skipped**.

The HAS_DMA flags describe the selected hardware. They must not be cleared to
hide an unfinished driver. Initialization failure counts as FAIL. The transfer
payloads, completion counts and data checks are unchanged.

Hardware validation (2026-10-21): **8 passed, 0 failed, 0 skipped** at
1 MHz, 8-bit MSB-first. All four SPI modes pass in each of the eight
master/slave execution combinations: 416 individual transfers, including
1-, 16- and 129-byte frames and short-frame rearming. This validates the
SPI1 master/SPI3 slave configuration above, not every H753 SPI instance
or higher clock rates.

The earlier mode-1 failure reported 17 bytes for a 16-byte TX, with identical
corruption in polling, interrupt and DMA combinations. Changing only the
master SCK slew from TURBO to LOW resolved it with the original transfers
and checks unchanged. This establishes slew sensitivity in the jumper setup;
clock ringing was not measured with an oscilloscope.

The shared driver selects LOW SCK slew for H753 masters at achieved rates
up to 1 MHz, and retains TURBO above 1 MHz. The policy follows the actual
selected rate, including subsequent SetRate calls. It uses the configured
SCK pin, not a hard-coded board pin. Slave pins retain their previous settings
because the slave's configured Rate does not determine the external SCK.
Only 1 MHz has been hardware validated with this policy; faster buses need
separate timing and signal-integrity validation.

The shared driver also retains the ES0392 CRC-size and EOT-disable corrections,
enables slave interrupts only after SPE/preload, and restricts AFCNTR to
master mode. The CRC-size and AFCNTR corrections alone did not resolve the
observed mode-1 failure.

Build validation: H753 soft/hard-float, L476 and WBA65 shared bus sources
compile at -O0 and -O3 with -Wall -Werror using GNU Arm GCC 13.2.1. The H753
SPI example compiles in Debug and Release; full library/example linking was
not rerun locally. Prior H753 bus register tests pass at -O0 and -O3 and the
WBA SPI model passes 64 checks. Register models do not validate physical
clock edges or the electrical effect of GPIO slew.

Pin references: ST [UM2407, connector tables 18-20](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf)
and [STM32H753 datasheet, alternate functions](https://www.st.com/resource/en/datasheet/stm32h753xi.pdf).

The failure-only `spi_diagnostics.cpp` hook prints the master completion count,
SPI1/SPI3 configuration and status, GPIO alternate functions, kernel clock
selection and EXTI/NVIC state. It reads registers without consuming RXDR or
changing pin configuration. Rebuild the example and capture the first failed
combination including this dump. Transfer lengths and assertions are unchanged.

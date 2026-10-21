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

Hardware run (2026-10-21) failed on the first one-byte frame in each
interrupt-slave combination and stalled while initializing the DMA slave.
The debugger showed SPI3 TXP/TXC interrupts before SPE was set. The shared
driver now enables SPI and preloads CPU TX data before enabling interrupts.
After correcting a misplaced jumper, all 13 mode-0 transfers pass in each
of the eight combinations. Each combination then passes three one-byte
mode-1 transfers and fails its first 16-byte mode-1 TX: the slave reports
17 bytes, with the first mismatch at byte 5. Modes 2 and 3 were not reached.

The shared driver now keeps CRCSIZE at its reset value of 7 even when CRC
is disabled, and waits at least one SCK period after EOT before disabling
the H753 master, including between TSIZE chunks. These address ES0392
sections 2.22.5 and 2.22.6. Hardware retesting is pending; the observed
mode-1 failure is not yet confirmed resolved. The shared hardware test,
payloads, mode selection and assertions remain unchanged.

Validation: GNU Arm GCC 13.2.1 compiles the shared application with this
board configuration in Debug (-O0) and Release (-O3). Project XML and linked
source paths were checked. Full library/example linking was not rerun.
The existing H753 bus register tests pass at -O0 and -O3 (with
`-fno-threadsafe-statics` in the local test toolchain); these model controller
behavior and do not validate physical clock edges. The added CRCSIZE check
fails with the preceding driver and passes with the correction. H753
soft/hard-float, L476 and WBA65 shared bus sources compile at -O0 and -O3
with -Wall -Werror; the WBA SPI model passes 64 checks. The optimized H753
assembly retains the delay loop before clearing SPE.

Pin references: ST [UM2407, connector tables 18-20](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf)
and [STM32H753 datasheet, alternate functions](https://www.st.com/resource/en/datasheet/stm32h753xi.pdf).

The failure-only `spi_diagnostics.cpp` hook prints the master completion count,
SPI1/SPI3 configuration and status, GPIO alternate functions, kernel clock
selection and EXTI/NVIC state. It reads registers without consuming RXDR or
changing pin configuration. Rebuild the example and capture the first failed
combination including this dump. Transfer lengths and assertions are unchanged.

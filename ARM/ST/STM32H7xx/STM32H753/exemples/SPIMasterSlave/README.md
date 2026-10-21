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
Hardware retest is pending; the first-frame failures are not yet cleared.
Each combination returns at its first failure, so later lengths and modes
are not printed until that failure is fixed. The test checks are unchanged.

Validation: GNU Arm GCC 13.2.1 compiles the shared application with this
board configuration in Debug (-O0) and Release (-O3). Project XML and linked
source paths were checked. Full library/example linking was not rerun.
The existing H753 bus register tests pass at -O0 and -O3 (with
`-fno-threadsafe-statics` in the local test toolchain); these model controller
behavior and do not validate the physical jumper connections.

Pin references: ST [UM2407, connector tables 18-20](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf)
and [STM32H753 datasheet, alternate functions](https://www.st.com/resource/en/datasheet/stm32h753xi.pdf).

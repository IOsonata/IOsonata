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
8 MHz, 8-bit MSB-first, using a shorter SCK jumper. All four SPI modes pass
in each of the eight master/slave execution combinations: 416 transfers,
including 1-, 16- and 129-byte frames and short-frame rearming. The same
suite previously passed at 1 MHz. These results cover SPI1 master/SPI3 slave
with the configuration above, not every H753 SPI instance or word size.

Use a short SCK jumper between CN7 pin 10 and CN8 pin 6. The longer wire
produced corrupt data and extra received bytes. An independent ST HAL
comparison also failed at 8 MHz in modes 1 and 2; replacing only the SCK
wire with a shorter one made all HAL modes pass, followed by all IOsonata
combinations. This establishes wiring sensitivity; the electrical waveform
was not measured.

After cleanup, the maintainer rebuilt and reran the suite with the common
TURBO pin setup restored and the short SCK wire (2026-10-21). All 416
transfers passed again: 8 combinations passed, 0 failed, 0 skipped. The
trial LOW/MEDIUM rate-specific overrides are no longer needed for this setup.
Set SPI_MASTER_RATE to 8000000 in your local board.h for the same 8 MHz
run; the repository default remains 1 MHz.

Temporary H753 register dumps, RAM trace hooks and the standalone HAL
comparison project have been removed. The shared validation checks and DMA
coverage remain. The driver retains the documented ES0392 CRC-size and
EOT-disable corrections, enables slave interrupts after SPE/preload, and
restricts AFCNTR to master mode.

Pin references: ST [UM2407, connector tables 18-20](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf)
and [STM32H753 datasheet, alternate functions](https://www.st.com/resource/en/datasheet/stm32h753xi.pdf).


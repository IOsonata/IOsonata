# NUCLEO-H753ZI I2C master/slave test

Connect these two jumpers with the board powered off:

| Signal | I2C1 master (DevNo 0) | I2C2 slave (DevNo 1) |
|---|---|---|
| SCL | PB8, D15, CN7 pin 2 | PF1, D69, CN9 pin 19 |
| SDA | PB9, D14, CN7 pin 4 | PF0, D68, CN9 pin 21 |

Add a 4.7 kohm pull-up from each bus line to 3V3 (CN8 pin 7).
All four pins use AF4. The test uses 100 kHz and slave address 0x22.
It checks writes, repeated-START reads, ordinary reads and completion behavior.
No second board is required.

Import this directory's `ioc` project and `STM32H753/lib/ioc` into IOcomposer.
Build the MCU library first, then this example, both in Release. The library
name is `IOsonata_STM32H753`; the configuration is selected by its directory.
Open the ST-LINK virtual COM port at **1000000 baud, 8N1**, then reset the board.
Debug uses semihosting: enable `monitor arm semihosting enable` in the debugger
startup commands. Use Release for the hardware test.

The project links the shared IOsonata example; `src/board.h` supplies only
board mappings and test configuration. Four master modes (polling, interrupt,
DMA polling, DMA interrupt) run against both interrupt and DMA slave modes.
Slave priority 6 can preempt master priority 7. Expected final summary: **8 passed, 0 failed, 0 skipped**.

The HAS_DMA flags describe the selected hardware. They must not be cleared to
hide an unfinished driver. Initialization failure counts as FAIL. The transfer
payloads, completion counts and data checks are unchanged.

Hardware run on NUCLEO-H753ZI (2026-10-21): **8 passed, 0 failed, 0 skipped**.
All four I2C1 master modes passed against both I2C2 slave modes, including
254-byte writes, short reads, repeated START, NACK and recovery. This result
does not validate the I2C3/I2C4 routes.

Validation: GNU Arm GCC 13.2.1 compiles the shared application with this
board configuration in Debug (-O0) and Release (-O3). Project XML and linked
source paths were checked. Full library/example linking was not rerun.
The existing H753 bus register tests pass at -O0 and -O3 (with
`-fno-threadsafe-statics` in the local test toolchain); these model controller
behavior and do not validate the physical jumper connections.

Pin references: ST [UM2407, connector tables 18-20](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf)
and [STM32H753 datasheet, alternate functions](https://www.st.com/resource/en/datasheet/stm32h753xi.pdf).

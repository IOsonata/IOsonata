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
board mappings and test configuration. The master runs polling and interrupt
modes against an interrupt-driven slave on the same MCU. Slave priority 6
can preempt master priority 7. DMA is excluded explicitly: H753 I2C/SPI DMA
integration is not implemented. These projects do not establish DMA support.

Expected final summary: **2 passed, 0 failed, 0 skipped**. Hardware validation
is pending; report the full output if a combination fails.

Validation: GNU Arm GCC 13.2.1 compiles the shared application with this
board configuration in Debug (-O0) and Release (-O3). Project XML and linked
source paths were checked. Full library/example linking was not rerun.
The existing H753 bus register tests pass at -O0 and -O3 (with
`-fno-threadsafe-statics` in the local test toolchain); these model controller
behavior and do not validate the physical jumper connections.

Pin references: ST [UM2407, connector tables 18-20](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf)
and [STM32H753 datasheet, alternate functions](https://www.st.com/resource/en/datasheet/stm32h753xi.pdf).

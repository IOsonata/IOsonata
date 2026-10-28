# LPCXpresso54628 I2C master/slave test

One board runs both ends. Connect these two wires with the board powered off:

| Signal | Master, Flexcomm 2 (DevNo 2) | Slave, Flexcomm 8 (DevNo 8) |
|---|---|---|
| SDA | P3_23, J11 (PMod) pin 10 | P1_17, J9 pin 4 |
| SCL | P3_24, J11 (PMod) pin 9 | P1_18, J9 pin 2 |

Flexcomm 2 is the on board I2C bus, its board pull-ups serve both ends. The
audio codec (0x1A) and the accelerometer (0x1D) stay on the bus; the test uses
slave address 0x22 and the free address 0x23 for the address NACK test. P1_17
and P1_18 are the Flexcomm 8 pins of the NXP SDK I2C board to board examples,
which give J9 pin 2 for SCL and J9 pin 4 for SDA. The test runs at 100 kHz.

Import this directory's `ioc` project and `LPC54628/lib/ioc` into IOcomposer.
Build the library first, then this example, both in Release. Open the LPC-Link2
VCOM port (Flexcomm 0) at **1000000 baud, 8N1**, then reset the board. Debug
prints through semihosting.

The project links the shared `exemples/i2c/i2c_master_slave.cpp`; `src/board.h`
supplies only pins and test configuration. Four master modes (polling,
interrupt, DMA, DMA with interrupt) run against the interrupt and DMA slave
modes. The driver interrupt handlers run with interrupts masked: slave priority 6 is
served before master priority 7 when both are pending, neither preempts the
other. Expected final summary:
**8 passed, 0 failed, 0 skipped**.

Each combination checks writes of 0, 1, 2, 8 and 254 data bytes after the
command byte, repeated START reads, plain reads, the address NACK and the
completion events.

Status: Debug and Release build and link. The driver passes the host bus
co-simulation in `tests/lpc/lpc546xx` (all eight combinations). No hardware
result yet.

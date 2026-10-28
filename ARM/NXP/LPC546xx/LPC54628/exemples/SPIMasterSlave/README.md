# LPCXpresso54628 SPI master/slave test

One board runs both ends. With the board powered off, install JP6 and connect
these four wires:

| Signal | Master, Flexcomm 3 (DevNo 3) | Slave, Flexcomm 9 (DevNo 9) |
|---|---|---|
| SCK | P0_0, J14 pin 4 | P3_20, J9 pin 9 (Arduino D13) |
| MISO | P0_2, J14 pin 3 | P3_22, J9 pin 11 (Arduino D12) |
| MOSI | P0_3, J14 pin 2 | P3_21, J9 pin 13 (Arduino D11) |
| CS | P0_1 GPIO, J14 pin 1 | P3_30 SSEL0, J9 pin 15 (Arduino D10) |

J14 is the only header with a second full Flexcomm SPI on this board. Its SPI
lines go to the LPC-Link2 SPI bridge while JP6 is open; JP6 installed gives
them to J14. On revision C boards JP6 also turns off the LPC-Link2 VCOM, the
Flexcomm 0 UART is then on the P4 header (an external USB serial adapter).
The debugger still works with JP6, so the Debug build, which prints through
semihosting, needs no serial port. The J9 pins are those of the NXP SDK SPI
board to board examples, the J14 pins those of the board manual (Flexcomm 3
SPI on PIO0_0 to PIO0_3).

Import this directory's `ioc` project and `LPC54628/lib/ioc` into IOcomposer.
Build the library first, then this example, in the same configuration. Debug:
enable semihosting in the debugger. Release: open the serial port at
**1000000 baud, 8N1**, then reset the board.

The project links the shared `exemples/spi/spi_master_slave.cpp`;
`src/board.h` supplies only pins and test configuration. Four master modes
(polling, interrupt, DMA, DMA with interrupt) run against the interrupt and
DMA slave modes, each in SPI modes 0 to 3 at 1 MHz, with frames of 1, 16 and
129 bytes: master writes, master reads of the slave data, and a 1 byte frame
before a long one (the slave must drop the data it loaded and did not send).
The driver interrupt handlers run with interrupts masked: slave priority 6 is
served before master priority 7 when both are pending, neither preempts the
other. Expected final summary:
**8 passed, 0 failed, 0 skipped**.

Status: Debug and Release build and link. The driver passes the host bus
co-simulation in `tests/lpc/lpc546xx` (all eight combinations). No hardware
result yet.

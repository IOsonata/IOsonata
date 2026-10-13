# LPC1769 SPI loopback

Build the matching Debug or Release IOsonata_LPC1769 library, then import
`ioc` into IOcomposer. The project links `exemples/spi/spi_loopback.cpp`
and uses the shared LPC SSP driver.

Connect **P0.18 (MOSI) to P0.17 (MISO)**. SSP0 uses function 2 on P0.15
(SCK), P0.17 (MISO), and P0.18 (MOSI). P0.16 is GPIO chip select; it needs
no jumper for this loopback. These are MCU pin names; locate them on your
board's headers and disconnect other peripherals from these signals.

The default is 1 MHz, 8-bit, MSB first, polling master operation. Interrupt
and DMA are disabled because the LPC SSP driver does not support them.
The shared example exercises all four SPI modes, six byte patterns,
16-byte and 129-byte reads, writes, and command/read transfers.
Successful output ends with `SPI master loopback PASS`. Reset to run again.

Debug prints through semihosting; attach a debugger with semihosting enabled.
Release uses nosys without semihosting and prints through UART0 at 1000000
baud, 8N1, no flow control. Connect a 3.3 V USB-UART adapter RX to P0.2
(UART0 TX) and ground. P0.3 is UART0 RX. The shared example initializes
UART0 in both builds.

Debug and Release application compilation and links against the complete
LPC1769 library pass. Physical loopback validation remains pending.

# LPC11U35 SPI loopback

Import `ioc` into IOcomposer and build the matching Debug or Release
IOsonata_LPC11U35 library first. The project links the shared
`exemples/spi/spi_loopback.cpp`; there is no local copy of the application.

For IBK-IDAP-M revision 0, connect **P3 pin 11 (MOSI) to P3 pin 7 (MISO)**.
SSP1 uses P1.15/function 3 for SCK (P2 pin 1), P0.22/function 3 for MISO,
and P0.21/function 2 for MOSI. P0.7 (P3 pin 10) is GPIO chip select; it needs
no jumper for this loopback. SWD and USB pins are not used by SSP1.

Debug output uses semihosting through IDAP-Link. Release has no semihosting
and prints through UART0 TX P0.19 (P2 pin 4), 1000000 baud, 8N1.
Use a 3.3 V UART adapter with a common ground. UART RX is P0.18 (P2 pin 3).

The LPC SSP driver supports polling master operation. Interrupt and DMA are
disabled. The shared example exercises all four SPI modes, several byte
patterns, 16-byte and 129-byte reads, writes, and command/read transfers.
Successful output ends with `SPI master loopback PASS`.

Debug and Release firmware links are checked with the production LPC11U35
startup and linker script. Physical loopback validation remains pending.

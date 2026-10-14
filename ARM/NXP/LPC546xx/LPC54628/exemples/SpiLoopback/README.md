# LPC54628 SPI loopback

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
uses the shared SPI loopback example in polling master mode. Its board header
selects Flexcomm 9: SCK P3.20 FUNC1, MOSI P3.21 FUNC1, MISO P3.22 FUNC1,
with GPIO chip select P3.30. Verify those routes against your board.

Jumper P3.21 MOSI to P3.22 MISO. The example checks modes 0-3, test patterns
and command/read transfers. SPI slave, interrupt and DMA transfers are not
implemented by this port. Debug uses semihosting; Release uses nosys.
Both firmware configurations link. Physical loopback and timing remain pending.

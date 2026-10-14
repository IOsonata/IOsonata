# LPC54628 I2C scanner

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
uses the shared polling-master scanner. Its board header selects Flexcomm 2,
SDA P3.23 FUNC1 and SCL P3.24 FUNC1, with external board pull-ups. Verify those
routes against your board. The optional register check reads device 0x1D,
register 0x0D (the configured MMA8652FC WHO_AM_I register).

The scanner defaults to 100 kHz and probes addresses 0x08 through 0x77. I2C
slave, interrupt and DMA operation are not supported by this driver.
Debug prints through semihosting. Release uses Flexcomm 0 UART at 115200 baud,
P0.30 TX/P0.29 RX, no flow control. Both firmware configurations link; physical
bus scanning and electrical timing still require board validation.

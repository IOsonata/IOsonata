# LPC1769 I2C scanner

Build the matching Debug or Release IOsonata_LPC1769 library, then import
`ioc` into IOcomposer. The project links `exemples/i2c/i2c_scan.cpp` and uses
the shared LPC I2C polling master.

Connect the device SDA to P0.27 (I2C0 SDA), SCL to P0.28 (I2C0 SCL), and
connect a common ground. Use external SDA/SCL pull-ups to 3.3 V and supply
the device according to its requirements. These are MCU pin names; locate
them on your board's headers.

The default rate is 100 kHz. The scanner probes 7-bit addresses 0x08 through
0x77 once after reset, prints each acknowledged address and the total, then
waits. Reset to scan again. Leave I2C_SCAN_INT_ENABLE false: this driver
supports polling master operation without I2C interrupts or DMA.

Debug prints through semihosting; attach a debugger with semihosting enabled.
Release uses nosys without semihosting and prints through UART0 at 1000000
baud, 8N1, no flow control. Connect a 3.3 V USB-UART adapter RX to P0.2
(UART0 TX) and ground. P0.3 is reserved for UART0 RX.

To read one register after scanning, define both I2C_SCAN_REG_DEVADDR
(the device's 7-bit address) and I2C_SCAN_REG (an 8-bit register address).
Choose a register that supports a one-byte read on the connected device.
I2C_SCAN_RATE can override the default bus rate; the driver supports rates
up to 400 kHz.

Debug and Release application compilation and links against the complete
LPC1769 library pass. Physical bus scanning and register reads remain pending.

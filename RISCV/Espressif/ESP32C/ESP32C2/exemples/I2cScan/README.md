# ESP32C2 I2cScan

Uses the generic `exemples/i2c/i2c_scan.cpp` with HP I2C0, polling,
7-bit addressing and 100 kHz. SDA is GPIO6 and SCL is GPIO7. Connect
external pull-ups to 3.3 V and a common ground. Adjust `src/board.h`
for your board; these pin assignments have not been hardware validated.

Build the matching ESP32C2 IOsonata library configuration, then this
managed project. Flash the binary at offset 0 with IOcomposer.
UART0 console is 1,000,000 baud, 8N1; UART pins are in `board.h`.
The scan reports acknowledged addresses from 0x08 through 0x77.

To check a repeated-start register read, define `I2C_SCAN_REG_DEVADDR`
and `I2C_SCAN_REG` in `board.h` for a known device and register.
A disconnected bus with pull-ups should report zero devices.

Slave, DMA, interrupt transfers and LP I2C are not implemented.

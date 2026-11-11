# ESP32C3 I2cSlave

Uses `exemples/i2c/i2c_slave_demo.cpp`. The hardware controller serves a
20-byte memory at 7-bit address 0x22, using interrupts and clock stretching.
A write starts with a one-byte memory offset. Reads return data from that
offset; a repeated START preserves the write offset.

Connect a second board running I2cMaster: SDA GPIO6 to SDA, SCL GPIO7 to SCL,
and ground to ground. Add external pull-ups from SDA and SCL to 3.3 V.
Build the matching library and example, then flash at offset 0.
UART output uses 1,000,000 baud, 8N1, in Debug and Release.

The HP I2C controller cannot be master and slave simultaneously. This
project requires an external master; the C6 LP I2C master is not implemented
here. Hardware validation is pending.

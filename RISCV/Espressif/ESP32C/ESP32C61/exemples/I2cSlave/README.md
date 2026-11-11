# ESP32C61 I2cSlave

Uses `exemples/i2c/i2c_slave_demo.cpp`. The hardware controller serves a
20-byte memory at 7-bit address 0x22, using interrupts and clock stretching.
A write starts with a one-byte memory offset. Reads return data from that
offset; a repeated START preserves the write offset.

Connect a second board running I2cMaster: SDA GPIO6 to SDA, SCL GPIO7 to SCL,
and ground to ground. Add external pull-ups from SDA and SCL to 3.3 V.
Build the matching library and example, then flash at offset 0.
UART output uses 1,000,000 baud, 8N1, in Debug and Release.

The HP I2C controller cannot be master and slave simultaneously. This
project requires an external master. Hardware validation is pending.

Console pins match the ESP32-C61-DevKitC-1 USB-to-UART bridge: TX GPIO11,
RX GPIO10. These differ from the GPIO4/5 external-adapter assignments
in the earlier bring-up examples. Use the USB-to-UART port for output.
See the [board guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c61/esp32-c61-devkitc-1/user_guide.html).

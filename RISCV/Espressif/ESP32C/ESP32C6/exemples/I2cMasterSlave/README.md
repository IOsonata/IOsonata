# ESP32C6 I2cMasterSlave

Uses the generic `exemples/i2c/i2c_master_slave.cpp` on one devkit.
I2C device 1 is the LP hardware master; device 0 is the HP hardware slave.
Both run on the main CPU, without an LP-core application.

| Signal | LP master GPIO | HP slave GPIO |
| --- | --- | --- |
| SDA | 6 | 4 |
| SCL | 7 | 5 |

Connect each row with a short jumper. The C6 `SpiMasterSlave` example uses
the same GPIO4-GPIO6 and GPIO5-GPIO7 jumpers. Its additional GPIO0-GPIO1 and
GPIO2-GPIO3 jumpers can remain fitted; this I2C example does not use those pins.

The example enables internal pull-ups
on both the LP master and HP slave pins, so start this bench test without
external resistors. If the bus rise time is too slow, add a 4.7 kOhm pull-up
from SDA to 3.3 V and another from SCL to 3.3 V. Internal pull-ups alone do
not guarantee I2C timing for longer wires or higher bus capacitance.
Build the matching library and example, then flash at offset 0. Both Debug
and Release print results over UART at 1,000,000 baud, 8N1.

The generic test uses 100 kHz, polling master and interrupt slave. It checks
register writes, repeated-START reads, reads without a command, writes up to
254 data bytes plus the command, address NACK, and recovery after NACK.
The long write crosses both the 16-byte LP FIFO and the 32-byte HP FIFO.

Expected final message: `I2C master/slave loopback PASS`.
Hardware validation is pending. The C3 has no LP I2C controller; its I2cSlave
test still requires an external master.

The C6 board enables a bounded slave callback trace (`I2C_TEST_TRACE_SIZE`).
The example captures the first 24 callbacks and prints them after the test,
without UART output inside the I2C interrupt. Each entry shows the event,
reported length, previous write state, offset before and after the callback,
first nine bytes of the receive buffer, and slave memory at offsets 4 through
11. Buffer snapshots can contain bytes from earlier writes; the reported
length determines how many bytes belong to the current event. This trace is
for investigating the first register-read failure; hardware validation is
still pending.

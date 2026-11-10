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

The C6 configuration runs the generic test at 100 kHz and 400 kHz, with
polling master and interrupt slave. Requested and configured rates are printed. It checks
register writes, repeated-START reads, reads without a command, writes up to
254 data bytes plus the command, address NACK, and recovery after NACK.
The long write crosses both the 16-byte LP FIFO and the 32-byte HP FIFO.

Boundary reads use lengths 15, 16, 17, 31, 32, 33, 63, 64 and 65 bytes.
Every read checks all bytes, then a command-free one-byte read checks that
the slave advanced its offset correctly. The test memory has 128 bytes;
these reads start at offset 32. Each rate reruns the write and NACK checks.

Expected final message:
`I2C master/slave loopback PASS (2 passed, 0 failed, 0 skipped)`.
The earlier 100 kHz C6 hardware runs passed with diagnostics enabled and
disabled, including
consecutive one-byte reads advancing from offset 16 to 17 and then 18.
The address-NACK test completes in about 1.6 ms with five retries; recovery
reads also pass. Slave TX completion counts byte-completion interrupts because
FIFO occupancy includes prefetched bytes and does not reliably count bytes
sent on the bus. FIFO filling remains in blocks.

The default build uses interrupt-driven UART and disables register tracing.
The expanded boundary reads and 400 kHz run still require hardware validation. The C3 has no LP I2C controller; its I2cSlave test
requires an external master.

For diagnostics, define `I2C_TEST_BOARD_TRACE` in the example compiler settings
and rebuild the example. This enables polling UART, the diagnostic timer,
24 slave callback snapshots, 128 slave TX snapshots, and 24 master failure
snapshots. Capture uses fixed storage and prints only after the transfers.
Each output line is flushed and paced by 2 ms. DATA is never read by the
capture code, so diagnostics do not consume received bytes. Remove the define
to restore normal operation; the diagnostic source then compiles without
capture storage or trace overrides.

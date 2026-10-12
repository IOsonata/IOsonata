# LPC11Uxx I2C master

Run `python3 tests/lpc_i2c/run.py` from the repository root. The tests compile
the actual family adapter, shared polling master and generic DeviceIntrf
implementation against a host register model at O0 and O2 with undefined
behavior sanitization.

LPC11U35 now provides I2C DevNo 0, polling master, standard I2C and 7-bit
addresses. Set bIntEn and bDmaEn to false. Slave, SMBus and 10-bit address
configurations are rejected. MaxRetry is passed to the generic interface;
zero means no retry. There are no asynchronous completion callbacks.

The pin map must contain SDA=P0.5 then SCL=P0.4, both IOPINOP_FUNC1. These
are dedicated open-drain pins and need external pull-up resistors. This
implementation uses standard/fast pin mode and supports rates up to 400 kHz.
Higher requests select the closest supported rate. At 48 MHz the slowest
rate is approximately 366.22 Hz (reported as integer 366 Hz). Init rejects a
zero rate; SetRate(0) leaves the current rate unchanged. Rate changes during
an active transaction also leave the current rate unchanged.

Shared operations are in ARM/NXP/src/i2c_lpc_master.cpp. The family file
owns clocks, reset and pin validation, and i2c_lpc_target.h accesses the
vendor CMSIS register type. The older LPC17xx driver and excluded legacy
shared C source have not been migrated. LPC546xx uses a different Flexcomm
I2C controller and keeps its existing driver.

START and repeated START send the address before returning success. TX counts
only acknowledged data bytes. RX acknowledges all but the last byte and
NACKs the last byte before STOP. Each RxData call completes a receive phase;
use another StartRx for a subsequent receive phase. A failed command phase
blocks its following repeated-START read until Stop releases the transaction.
The generic layer owns the busy lock.

All hardware waits are bounded by register-read counts, including STOP
completion (STO clears without SI). The budget includes an allowance based
on the programmed divider so slow rates can finish a byte. This is not an
elapsed-time timeout and does not support indefinite clock stretching.
Arbitration loss releases controller state without sending STOP. A timeout
while waiting for START or STOP resets the peripheral; physical bus recovery
and detection of a permanently stuck device still need board validation.
Disable/Enable retains timing and restores clock and pin configuration.
Reset restarts the controller; it does not pulse SCL. PowerOff clears enable
references so a later Enable can turn it on again.

Tests cover address and data NACK, repeated START, final-byte NACK, stale-data
prevention after a failed read address, command failure, arbitration loss,
START/STOP timeouts, rate bounds, ownership and enable reference counting.
ARM compiler-to-assembly checks also pass at O0 and O2 with real vendor headers.
Full firmware linking and physical I2C bus tests remain pending.

Hardware reference: NXP UM10462, chapter 14 (I2C), and chapter 3 (clock/reset).
https://www.keil.com/dd/docs/datashts/nxp/lpc11uxx/um10462.pdf

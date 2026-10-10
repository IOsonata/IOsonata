# LPC546xx host regressions

Run from a Linux checkout with G++ and Python 3:

```
python3 tests/lpc546xx/run.py
```

`timer_test.cpp` compiles the production CTIMER driver
(`ARM/NXP/LPC546xx/src/timer_lpc546xx.cpp`) against the repository's
LPC54628 device header. The CTIMER, SYSCON and ASYNC_SYSCON registers and the
NVIC are a host model: the counter jumps from match to match, the match
flags of the enabled match interrupts are set and the IRQ handler is called
like the NVIC would. It checks the 64 bit count over counter wraps, trigger
periods longer than a counter cycle, single shot and continuous triggers, the
pending interrupt for a deadline passed while arming, prescaler selection,
and the configuration checks of TimerInit.

`i2c_spi_test.cpp` compiles the Flexcomm selection, I2C master and SPI master
drivers with the generic `device_intrf.cpp`. The I2C model is the master state
machine of the Flexcomm I2C with one simulated slave. It checks the START,
repeated START, data acknowledge and the NACK of the last byte read, NACK of
the address and of data, the SCL low and high times selected for 100 kHz,
400 kHz and 1 MHz, and the configuration checks of I2CInit. The SPI model is a
MOSI to MISO loopback. It checks the dummy byte on receive, the RX FIFO not
written on transmit, the chip select held for a command and read, the frames
in flight, the clock divider, and the Flexcomm function change and lock.

The models do not cover the prescaler counter, bus timing through the async
APB bridge, I2C clock stretching or interrupt preemption. Hardware validation
is still required.

# nRF54L I2C and SPI checks

The shared Nordic drivers are included in the nRF54L15 and nRF54LM20x library
projects. The C and C++ I2C/SPI interfaces remain unchanged.

Supported modes:

- I2C master with polling EasyDMA, 7-bit addresses, including register reads
  with a repeated start. Select `bIntEn = false`.
- I2C slave with EasyDMA and interrupts, up to two addresses. Supply buffers
  through the existing read/write request callbacks.
- SPI master with polling EasyDMA, 8-bit words, software chip select and the
  existing clock-polarity, phase and bit-order settings. Select `bIntEn = false`.
- SPI slave with EasyDMA and interrupts. Supply buffers when the callback
  reports `DEVINTRF_EVT_STATECHG`; completion reports the received byte count.

Master interrupt operation is unfinished in the shared drivers. nRF54L
initialization returns false for a master configuration with `bIntEn = true`.
EasyDMA is required by these peripherals, including when `bDmaEn` is false.
DMA buffers must remain in accessible RAM for the duration of the operation.
Slave buffers are limited to 65535 bytes; masters divide longer buffers into
DMA blocks. I2C receive blocks each finish with STOP, matching the shared
Nordic driver behavior; a read longer than 65535 bytes is not one continuous
I2C transaction.

Rates are rounded to the closest supported value, including requests outside
the peripheral range. I2C offers 100, 250, 400 and 1000 kbit/s. SPI00 uses a
128 MHz clock with even divisors 4 through 126; other SPI instances use 16 MHz
with even divisors 2 through 126. Slave rate settings do not program a master
clock register: the external master supplies the clock.

Device numbering and MCU-specific instances are listed in
[MCU support](../../docs/supported-targets.md). Do not share an active serial
instance between UART, I2C and SPI.

## Automated checks

Install Python packages `unicorn` and `pyelftools`, and provide an Arm GNU C++
compiler and a Nordic MDK with L15, LM20A and LM20B headers:

```sh
python tests/nrf54_buses/build.py --mdk /path/to/nrfx/bsp/stable/mdk
```

Use `--cxx /path/to/arm-none-eabi-g++` or `--cmsis /path/to/CMSIS/Core/Include`
when they are outside the normal locations.

The script compiles both drivers for L15, LM20A/B, nRF52832/840, both nRF5340
cores, nRF9160 and nRF9120. For nRF54 it also links the production drivers and
generic interface code into Arm executables, then runs them with a small DMA
register model. The fixture stubs pin configuration and supplies no physical
bus signals.

Checks cover instance addresses, unavailable I2C device 4, closest rates at
both limits and between supported values, avoiding master clock-register
writes in slave mode, slave buffer limits and completion callbacks, master
DMA pointer/count updates across 65535 bytes, and register reads without a
STOP between the command and receive phases.

These checks passed with Arm GNU 14.3.Rel1. They are driver compilation and
register tests, not complete IOC application builds or hardware validation.

## Hardware checks still needed

Use the existing `exemples/i2c/i2c_master_slave.cpp` and
`exemples/spi/spi_master_slave.cpp`. Select polling DMA for the master and
interrupt DMA for the slave. The SPI example needs
`SPI_MASTER_INT_ENABLE false` in `board.h` for this port. Keep wiring and device
selection in `board.h`, using pins permitted for each selected peripheral.
Avoid the UART instance used for test output.

Run I2C register write/read and repeated-start reads at each rate. Check SPI
TX and RX at slow and fast rates, then check the application's required clock
modes and bit order. Exercise both an ordinary SPI instance and SPI00, plus
LM20's additional instances when those pins are available. Record the MCU,
board, wiring, firmware commit, returned rates and observed results.

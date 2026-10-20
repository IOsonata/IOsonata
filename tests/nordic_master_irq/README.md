# Nordic master interrupt transfers

Register level checks of the SPIM and TWIM master drivers with `bIntEn = true`
and EasyDMA, built for each MCU register layout (nRF9160, nRF52832, nRF54L15 and
nRF54LM20A by default, `--chip NRFxxxx_XXAA` to select). They run the production
`spi_nrfx.cpp`, `i2c_nrfx.cpp`, `shared_intrf_nrfx.cpp` and `device_intrf.cpp`
in an Arm emulator. Each START task is recorded; the test then raises the
completion events and runs the shared interrupt handler.

Checks cover:

- SPI TX and RX return -1 and keep the bus until the last DMA block;
- blocks longer than the DMA count field are chained by the interrupt handler
  with contiguous buffers;
- one `DEVINTRF_EVT_COMPLETED` with the byte count, `bTxReady` after a TX;
- SPI command then read: the command phase keeps the bus and reports no
  completion;
- TWIM transmit and receive, single and several DMA blocks: transmit blocks
  suspend the bus and the last one ends with STOP, receive blocks end with
  STOP, the bus is released and one `DEVINTRF_EVT_COMPLETED` is reported only
  after the hardware has stopped, with no extra STOP task;
- TWIM register read: the command phase suspends the bus, sets `bTxReady`
  and reports nothing, `DeviceIntrfRead()` waits for it, then the receive;
- TWIM bus error: the driver stops the bus, the completion reports the bytes
  moved, the bus can be used again; an address NACK on the command phase still
  ends the read;
- polled transfers (`bIntEn = false`) are unchanged.

Install Python packages `unicorn` and `pyelftools`, and provide an Arm GNU C++
compiler and a Nordic MDK:

```sh
python tests/nordic_master_irq/build.py --mdk /path/to/nrfx/bsp/stable/mdk
```

These are driver and register tests, not hardware validation. Interrupt
latency, bus timing and errata are not modeled.

## Hardware checks still needed

Use `exemples/spi/spi_master_slave.cpp` and `exemples/i2c/i2c_master_slave.cpp`
with the master interrupt option enabled on an nRF91 board. Check a short and a
long (more than one DMA block) SPI write and read, a register read on I2C, the
completion callback count, and that no transfer is repeated on the bus.

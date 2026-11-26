# Nordic master interrupt transfers

Register level checks of the SPIM and TWIM master drivers with `bIntEn = true`
and EasyDMA, built for each MCU register layout (nRF9160, nRF52832, nRF54L15 and
nRF54LM20A by default, `--chip NRFxxxx_XXAA` to select). They run the production
`spi_nrfx.cpp`, `i2c_nrfx.cpp`, `shared_intrf_nrfx.cpp` and `device_intrf.cpp`
in an Arm emulator. Each START task is recorded. A transfer waits in the
driver for its interrupt handler: when the firmware runs on without returning,
the test raises the next completion events and runs the shared interrupt
handler on another stack, then lets the firmware continue.

Checks cover:

- SPI and TWIM TX and RX return their byte count when the last DMA block is
  done, not before, and the bus is released on return;
- the blocks advance through the caller's buffer, SPI with CS kept asserted
  between blocks;
- one `DEVINTRF_EVT_COMPLETED` with the byte count; `bTxReady` and one
  `DEVINTRF_EVT_TX_READY` after a TX;
- command then read on SPI and TWIM: the command phase keeps the bus, sets
  `bTxReady` and reports no completion, the read returns the data count;
- TWIM: the hardware puts the STOP on the bus, no extra STOP task follows;
- TWIM combined write: sent from the buffer `DeviceIntrfWrite()` builds on its
  stack, which returns the data count;
- TWIM bus error: the driver stops the bus and returns the bytes moved, the
  bus can be used again; an address NACK on the command phase still runs the
  receive; an absent device is retried as in polling mode and returns 0;
- no interrupt at all (SPIM and TWIM): the transfer ends on the time limit of
  the driver, TWIM stops the bus, the next transfer works;
- TWI without EasyDMA (nRF52832): each byte from the interrupt, the receive
  suspends the bus after each byte and ends with STOP after the last one, a
  one byte receive stops at once, the command of a register read keeps the
  bus, a bus error stops it, the transfer returns only on STOPPED;
- SPI without EasyDMA (nRF52832): one byte per READY event, the receive
  buffer in the completion, the command of a register read keeps the bus;
- polled TWI after TWIM transfers (nRF52832): the TXDSENT and RXDREADY events
  a TWIM transfer leaves in the shared block are cleared before the start, a
  one byte receive stops through the BB shortcut, a refused byte ends the
  transmit with nothing more written to TXD;
- one byte SPIM receive on nRF52832 (anomaly 58): done by the SPI of the same
  instance, the SPIM never starts and is enabled again, polled, with
  interrupt and after a command phase;
- a new configuration clears the interrupts of the previous one (SPI and
  SPIM, TWI and TWIM, are one block);
- polled transfers (`bIntEn = false`) are unchanged.

Install Python packages `unicorn` and `pyelftools`, and provide an Arm GNU C++
compiler and a Nordic MDK:

```sh
python tests/nordic_master_irq/build.py --mdk /path/to/nrfx/bsp/stable/mdk
```

These are driver and register tests, not hardware validation. Interrupt
latency, bus timing and errata are not modeled.

## Interrupt driven transfers

A Nordic I2C or SPI master transfer behaves the same with `bIntEn` set or
cleared: it returns when the data has moved, with the byte count. The data is
sent from and received into the caller's buffer, of any length: the driver
keeps the pointer and the remaining count and has no copy. With EasyDMA a
transfer longer than one hardware DMA block continues with the next block
from the interrupt handler. The TWIM starts each block with a START and the
address, as the polling transfer does; the SPIM keeps CS asserted.

The wait has a time limit that follows the length of what the hardware moves
between two interrupts. The interrupt priority of the bus must be higher than
the priority of any interrupt handler that calls a transfer on it.

## Hardware checks still needed

Use `exemples/spi/spi_master_slave.cpp` and `exemples/i2c/i2c_master_slave.cpp`
with the master interrupt option enabled. Check a short and a long (more than
one DMA block) SPI write and read, a register read on I2C, the returned
counts, and that no transfer is repeated on the bus. Check an I2C transfer to
an address with no device, then a normal one.

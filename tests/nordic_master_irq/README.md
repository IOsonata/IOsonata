# Nordic master interrupt transfers

Register level checks of the SPIM and TWIM master drivers with `bIntEn = true`
and EasyDMA, built for each MCU register layout (nRF9160, nRF52832, nRF54L15 and
nRF54LM20A by default, `--chip NRFxxxx_XXAA` to select). They run the production
`spi_nrfx.cpp`, `i2c_nrfx.cpp`, `shared_intrf_nrfx.cpp` and `device_intrf.cpp`
in an Arm emulator. Each START task is recorded; the test then raises the
completion events and runs the shared interrupt handler. An I2C or SPI
master transfer is synchronous and waits for its interrupt: each time the
firmware has run for a while without returning, the test ends what the
controller was doing and runs the handler when the event is enabled in INTEN.

Checks cover:

- SPI master with `bIntEn` (SPIM, and SPI on nRF52832): transmit, receive,
  register read and combined write return their byte count, the DMA blocks
  advance through the caller's buffer, each block or byte ends on its
  interrupt, the master reports no event, the bus is released on return; an
  event whose interrupt is held off does not end the wait;
- I2C master with `bIntEn` (TWIM, and TWI on nRF52832): transmit, receive,
  register read and combined write return their byte count, the blocks or
  bytes come from and go into the caller's buffer (the combined write from
  the buffer `DeviceIntrfWrite()` builds), each block or byte ends on its
  interrupt, the master reports no event, the bus is released on return;
- I2C master with `bIntEn`: an event that is in the register but whose
  interrupt is held off does not end the wait, the transfer ends on its time
  limit and the bus is usable after; an address NACK is retried as in
  polling and returns 0; a refused byte on the TWI ends the transmit with
  the bytes sent before it;
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

A Nordic I2C or SPI master transfer is synchronous with or without `bIntEn`:
it returns when the data has moved, with the byte count. With `bIntEn` its
wait functions wait for the interrupt instead of reading the events in a
loop; the handler masks the master interrupts and tells the wait, which reads
the events as in polling mode. It must not be called from an interrupt
handler of the same or a higher priority than the bus.

## Host lifetime and timeout checks

The smaller host runner needs a C++17 compiler, with AddressSanitizer and
UndefinedBehaviorSanitizer support:

```sh
ASAN_OPTIONS=detect_leaks=0 python3 tests/nordic_master_irq/run_lifetime.py
```

It extracts unchanged production function bodies and links the real generic
`device_intrf.cpp`. Register tasks and IRQ masking use host models. It runs at
O0 and O2 with 8-bit and 16-bit DMA counts and with the default and an enlarged
I2C copy buffer. It checks:

- I2C TX, combined write and oversized read-command rejection, retry count,
  no bus task or byte on rejection, and the next valid transfer;
- SPI copy and DMA-block boundaries, including a 66000-byte transfer;
- completion immediately before IRQ masking, including a callback that starts
  another transfer;
- a suppressed completion, a pending END before STOPPED, detached caller
  pointers, held CS and busy state, one late timeout callback, and reuse;
- a timed-out `DeviceIntrfWrite()` whose combined buffer was on the stack;
- timeout completion before a failed read enters RX, and during RX cleanup,
  including callback reentry and a later receive-only operation.

These checks do not compile the complete MCU translation unit or validate MDK
register layouts. Use the Arm runner for that integration check. Leak detection
is disabled in the command above because some restricted host environments
cannot inspect process mappings; address and undefined-behavior checks remain
enabled.

## Hardware checks still needed

Use `exemples/spi/spi_master_slave.cpp` and `exemples/i2c/i2c_master_slave.cpp`
with the master interrupt option enabled on an nRF91 board. Check a short and a
long (more than one DMA block) SPI write and read, a register read on I2C, the
completion callback count, and that no transfer is repeated on the bus. Check
I2C writes at the configured copy limit and rejection one byte above it, and
SPI timeout recovery after confirmed STOPPED.

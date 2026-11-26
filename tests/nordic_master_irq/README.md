# Nordic master interrupt transfers

Register level checks of the SPIM and TWIM master drivers with `bIntEn = true`
and EasyDMA, built for each MCU register layout (nRF9160, nRF52832, nRF54L15 and
nRF54LM20A by default, `--chip NRFxxxx_XXAA` to select). They run the production
`spi_nrfx.cpp`, `i2c_nrfx.cpp`, `shared_intrf_nrfx.cpp` and `device_intrf.cpp`
in an Arm emulator. Each START task is recorded; the test then raises the
completion events and runs the shared interrupt handler.

Checks cover:

- SPI TX and RX return -1 and keep the bus until the last DMA block;
- SPI TX and RX blocks advance through the caller's buffer with CS kept
  asserted between blocks;
- one `DEVINTRF_EVT_COMPLETED` with the byte count; `bTxReady` and one
  `DEVINTRF_EVT_TX_READY` after a TX;
- SPI command then read: the command phase keeps the bus and reports no
  completion;
- TWIM transmit and receive from the caller's buffer, over several DMA blocks.
  The bus is released and one `DEVINTRF_EVT_COMPLETED` is reported only after
  the hardware has stopped, with no extra STOP task;
- TWIM combined write: `DeviceIntrfWrite()` waits for the end of the transmit
  of its own buffer;
- TWIM register read: the command phase suspends the bus, sets `bTxReady`
  and reports nothing, `DeviceIntrfRead()` waits for it, then the receive;
- device register read through `Device::Read()` on SPIM and TWIM: the
  interface returns -1, the device waits for the transfer to be closed and
  returns the length, the event handler gets the byte count; unchanged when
  polled;
- TWIM bus error: the driver stops the bus, the completion reports the bytes
  moved, the bus can be used again; an address NACK on the command phase still
  ends the read;
- TWI without EasyDMA (nRF52832): each byte from the interrupt, the receive
  suspends the bus after each byte and ends with STOP after the last one, a
  one byte receive stops at once, the command of a register read keeps the
  bus, a bus error stops it, completion only on STOPPED;
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

## Interrupt-driven transmit limits

Nordic I2C TX is sent from the caller's buffer, of any length: the driver
keeps the pointer and the remaining count and has no copy. The buffer must stay
valid until `DEVINTRF_EVT_COMPLETED`. `DeviceIntrfWrite()` waits for the end of
the transmit of the buffer it builds on its stack. With EasyDMA a transmit
longer than one hardware DMA block continues with the next block, as the
polling transmit does: the TWIM starts each block with a START and the
address.

Nordic SPI TX is sent from the caller's buffer in the same way, one DMA block
after the other with CS kept asserted. The command phase of a register read
keeps the bus and sets `bTxReady`; `DeviceIntrfRead()` waits for it. The
driver has no transmit timeout of its own: an SPI master clocks the transfer
itself.

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

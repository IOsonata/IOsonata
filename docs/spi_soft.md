# GPIO SPI master

`SPISoft` implements a synchronous SPI master using IOsonata GPIO and delay
functions. It follows the C driver / C++ wrapper pattern used by `EInkIntrf`,
without the display-specific D/C, BUSY or reset protocol. Hardware SPI and
software SPI can coexist; this implementation does not define `SPIInit` or use
hardware controller slots.

Include `coredev/spi_soft.h` and compile `src/coredev/spi_soft.cpp` into the MCU
library. The SAM4LCxC and SAM4LSxC library projects include it. Other target
projects can link the same source beside `src/coredev/spi.cpp`.

Use the existing `SPICfg_t` with master mode, DMA and interrupt flags false,
and GPIO pin functions. The map order is SCK, MISO, MOSI, then chip selects.
`DevNo` does not reserve a hardware controller. Initialize the object in main;
the constructor does not touch GPIOs. Keep the pin map alive for the object's
lifetime and stop transfers before reinitializing or changing configuration.

```cpp
SPISoft master;
// cfg is an SPICfg_t with a persistent GPIO map.
if (!master.Init(cfg)) {
    // Configuration is unsupported or invalid.
}
DeviceIntrf *bus = &master;
uint8_t command = 0x80;
uint8_t data[4];
bus->Read(0, &command, 1, data, sizeof(data));
```

`Rx`, `Tx`, `Read`, `Write`, `StartRx/Tx`, `RxData/TxData` and `StopRx/Tx`
use the normal DeviceIntrf hooks. A command followed by RX keeps CS asserted
through both phases. The framework owns the interface busy flag. Polling
operations return their byte count and do not issue completion callbacks.

For simultaneous TX/RX, `master.Transfer(cs, tx, rx, length)` wraps one
synchronous frame. A null TX sends `DummyByte`; a null RX discards input.
Exact in-place TX/RX is supported; partially overlapping buffers are not.
The corresponding C functions are `SPISoftInit` and `SPISoftTransfer`, using
caller-owned `SPISoftDev_t` storage. Standard C SPI transfer helpers can use
`&dev.Spi`. Use SPISoftInit to change the physical mode; do not call a target's
hardware `SPIInit` or `SPISetPhy` on this software device.

Supported formats:

- Modes 0-3, MSB or LSB first, 4-16 bits per word.
- Up to 8 bits: one byte per word. 9-16 bits: two little-endian bytes per word;
  an odd byte count is rejected. Unused upper bits are ignored/zeroed.
- Normal SPI with separate data pins. An unused MISO or MOSI can be -1/-1.
- Three-wire half duplex: MOSI is the bidirectional data pin and MISO must be
  -1/-1. RX releases the data pin before generating clocks; TX drives it.
  Simultaneous TX/RX is rejected in this mode.
- Automatic active-low GPIO chip selects, or manual CS entirely owned by the
  application. No GPIO configuration or writes are made to manual CS pins.

The nominal clock is rounded down using whole-microsecond half periods, with
500 kHz as the maximum delay-based setting. `Rate()` reports that nominal
ceiling, not a measured bus frequency. GPIO overhead and interrupts extend
periods. Interrupts are masked only around each sampling edge and input read;
there is no transaction-long critical section. Use hardware SPI when precise
clock timing or throughput is required.

`Disable` stops the selected session and prevents transfers. `Enable` restores
GPIO configuration. `PowerOff` also releases owned pins; `Enable` restores them.
`Reset` retires a selected session through the framework stop helper. Lifecycle
operations must be serialized with synchronous transfers by the caller.

All generic SPI examples with a master support the same selection in board.h:

```cpp
#define SPI_MASTER_SOFTWARE true
#define SPI_MASTER_RATE 100000
```

This applies to spi_master_demo.cpp, spi_polling_master_demo.cpp,
spi_loopback.cpp and spi_master_slave.cpp. The default is hardware SPI.
The examples select GPIO pin functions and disable master DMA/interrupts
automatically for SPISoft, retaining the board's master pin numbers. Choose
free GPIO-capable pins in board.h and compile spi_soft.cpp into the target
library as described above. Startup output identifies the selected driver
and DMA/interrupt settings. Software clock rates are nominal.

The two polling demos remain synchronous with either driver. spi_loopback.cpp
and spi_master_slave.cpp honor SPI_MASTER_DMA_ENABLE and SPI_MASTER_INT_ENABLE
for hardware masters. The master/slave example also accepts
SPI_SLAVE_DMA_ENABLE and SPI_SLAVE_INT_ENABLE; its software master uses Transfer
for full duplex frames, while its hardware master uses standard Tx/Rx APIs for
separate frames. spi_slave_demo.cpp remains a hardware-slave example because
SPISoft implements master mode only.

The SAM4L target project is now SPIMasterSlave (formerly SPISlaveLoopback).
Reimport the renamed project after rebuilding the SAM4L library. Wiring is
unchanged: EXT1 10-12 and 17-18; EXT2 7-8 and 15-16.
The migrated example requires a fresh hardware run: the earlier passes used
its inline bit-bang implementation. Host tests cover the real driver's edge
sequence, data order, generic APIs, CS lifetime and invalid configurations.

# nRF5340 application-core SPI

Use the generic `SPI` class or `SPIInit` C API with `SPICfg_t`.
Device numbers 0 through 3 select shared serial peripherals; device 4 selects
SPIM4; device 5 selects QSPI. SPIM4 has a dedicated interrupt vector and does
not register in the four-entry shared UART/TWI/SPI dispatcher.

SPIM4 supports master mode, 8-bit words, MSB/LSB order, all four clock modes,
software chip selects and three-wire pin switching. It always uses EasyDMA,
even when `bDmaEn` is false. `bIntEn` selects polling or interrupt completion.
Configuration and pin maps remain ordinary IOsonata data.

The driver selects and returns the nearest supported rate. SPIM0..3 are limited
to 8 MHz. SPIM4 supports up to 16 MHz with receive enabled. Its 32 MHz transmit-only
mode requires a 128 MHz CPU, SCK on P0.08, MOSI on P0.09, and an unused MISO
(`PinNo = -1`). The driver configures the dedicated peripheral pin routing and
high drive for that mode. Other configurations select a lower supported rate.
An unused receive pin causes receive operations to return zero. Call `Rate()`
again after changing the CPU clock; do not change clocks or rates during a transfer.

These limits follow Nordic's [nRF5340 product specification](https://docs.nordicsemi.com/r/bundle/ps_nrf5340/)
and [pin assignments](https://docs.nordicsemi.com/r/bundle/ps_nrf5340/page/chapters/pin.html).

Polling returns the transferred byte count. Interrupt-driven transfers return
`-1` and complete through `DevIntrf_t::EvtCB`. The driver releases the generic
transaction lock before reporting completion. A command phase with `bNoStop`
retains chip select for the following read. Small asynchronous transmits use an
owned RAM copy; larger transmits wait for completion or bounded cancellation.
Polling TX also stages data in RAM so constants in flash are usable.

Transfers larger than the DMA counter are split into blocks with explicit
buffer addresses. Polling waits include wire time at the selected rate. On a
timeout, STOP is requested before returning the caller's buffer; if STOP does
not complete, the peripheral is disabled and must be reinitialized. The driver
uses the documented ENABLE register on nRF5340, not the older Nordic undocumented
power register at offset 0xFFC.

Run the production-driver register model with:

```sh
python3 tests/nrf5340_spi/run.py
```

It covers polling and interrupt completion, a 70000-byte receive, copied TX,
long TX completion/cancellation, command/read sequencing, chip-select and busy
release, timeout recovery, rate limits, invalid configurations and QSPI dispatch
isolation. It builds at `-O0` and `-O2` with ASan/UBSan. QSPI itself is stubbed;
this is not a QSPI hardware test. SPIM4 pin timing and physical loopback still
require hardware validation. AudioDkTest uses the same driver with device 4,
8 MHz, polling and two software chip selects.

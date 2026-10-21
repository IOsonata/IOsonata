# H753 ST HAL SPI reference

Diagnostic comparison for the mode-1 payload corruption. This is not the
IOsonata SPI validation suite and does not establish that the port is fixed.
It uses ST's unmodified SPI HAL, pinned to commit
`7e541d92019e18f98d211fc4ab9197ec8e8105f6`, with IOsonata startup, GPIO and UART.
The IOsonata SPI driver is not instantiated or linked by this application.

From the IOsonata repository root, prepare the ST sources once:

```sh
python3 ARM/ST/STM32H7xx/STM32H753/exemples/SPIHalReference/prepare_hal.py
```

The script downloads only the files listed in `hal_manifest.json` from the
pinned ST repository and verifies SHA-256 before installing them into the
ignored `vendor` directory. `--source /path/to/stm32h7xx-hal-driver` uses an
offline copy of that revision instead. ST's license is included there.

Import `ioc` as project **SPIHalReference**, refresh it after preparation,
and build Release against the existing `IOsonata_STM32H753` Release library.
Use the existing H753 OpenOCD configuration with `SPIHalReference.elf` selected.
Read ST-LINK UART at **1000000 baud, 8N1**. Both configurations log to UART.

Keep the existing jumpers:

| Signal | Master SPI1 | Slave SPI3 |
|---|---|---|
| SCK | PA5 / CN7-10 | PC10 / CN8-6 |
| MISO | PA6 / CN7-12 | PC11 / CN8-8 |
| MOSI | PB5 / CN7-14 | PC12 / CN8-10 |
| CS | PD14 / CN7-16 | PA15 / CN7-9 |

The master uses HAL polling and the slave HAL interrupts. This combination
already reproduced the IOsonata failure, so DMA is not needed for this first
comparison. The HAL DMA source is included to satisfy the HAL IRQ error paths;
this application does not request DMA transfers.

Each run covers 1 MHz and 8 MHz, modes 0-3, and the same sequence of 13 frames
per mode used by SPIMasterSlave: lengths 1, 16 and 129, seeds 0x30/0x50,
receive clocks with 0xFF, and short/long frames with seeds 0x70/0x90.
SCK slew matches the current diagnostic driver: LOW at 1 MHz and MEDIUM at
8 MHz; other pins remain TURBO. The printed core clock and SCK_OSPEED fields
make this comparison explicit. SPI kernel clock is HSI64 through PER.

HAL requires a known transfer length. Its slave is therefore explicitly armed
for each frame and closes on EOT; it does not validate IOsonata's open-length,
NSS-driven slave rearming or detect extra clocks after HAL has closed the slave.
A HAL PASS narrows the investigation but does not prove a driver root cause.
A HAL failure with the same bit shift makes the IOsonata receive-count logic
less likely to be the cause. Capture the entire output either way.

For the IOsonata comparison, rebuild the library and the existing
SPIMasterSlave example. Its failure dump now includes the last 32 lifecycle
and error events captured in RAM. Event numbers are printed with the trace.
`count` is a software item count (DMA updates it at frame end); NSS is the
sampled PA15 level. The trace does not consume RXDR, print during a transfer,
or change any test payload or assertion. Recording briefly masks interrupts,
so its timing effect must be considered when comparing results.

## Debugging without a scope

After the ordinary reference summary, failed modes replay automatically with
a RAM trace. Keep the same wiring and UART settings. No new HAL download is
needed. Capture the entire output, including the `Diagnostic replay` sections.
The replay uses the same lengths, payloads, comparisons, rate and GPIO settings.
It is separate from the reference pass because instrumentation affects timing.
No IOsonata driver or generic validation test is changed by this diagnostic.

The trace retains the last 32 events and freezes on the first observed slave
UDR, OVR, MODF or TIFRE flag. IRQ entry is recorded before calling ST's unchanged
handler; IRQ exit is recorded after it. The recorder never reads RXDR, clears
flags or prints during a transfer. It briefly masks interrupts while writing
an entry. The ordinary reference pass does not record events, but its IRQ
wrapper has an added trace-enable check.

Each entry reports core cycles since arming began, NSS, both SPI status/control
registers, slave interrupt enables and HAL TX/RX items remaining. These software
counts are not wire byte counts; the polling master uses local counters and
its handle counts can lag transfer progress. SR retains the hardware CTSIZE
and FIFO flags for comparison. The pre-CS slave snapshot is retained separately
even if the ring wraps. TX=0 there means HAL queued all requested slave bytes
before CS, not that they have been transmitted. Both received payloads print
after a replay failure. A replay that passes when the reference fails is itself
evidence of timing sensitivity, not a fixed port.

Hardware reference result: all four modes pass at 1 MHz/LOW SCK slew. At
8 MHz/MEDIUM, modes 0 and 3 pass; modes 1 and 2 fail on the first 16-byte TX,
with the same corrupt slave payload and HAL error 0x80 (slave TX underrun).
Both failing modes sample on falling SCK edges. This does not establish an
electrical cause or explain the corrupt receive data by itself.

Validation: reference application and unchanged HAL sources compile at -O0
and -O3 with -Wall -Werror; a relocatable link confirms HAL dependencies resolve.
The reference has been built and run on hardware by the maintainer. The new
RAM-trace replay is compiled but awaits hardware execution. Shared SPI builds
previously passed for H753 soft/hard-float, L476 and WBA65 at -O0/-O3.

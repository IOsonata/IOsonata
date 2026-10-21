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

Validation: reference application and unchanged HAL sources compile at -O0
and -O3 with -Wall -Werror; a relocatable link confirms HAL dependencies resolve.
Full firmware linking and hardware execution are pending. Shared SPI builds
pass for H753 soft/hard-float, L476 and WBA65 at -O0/-O3.

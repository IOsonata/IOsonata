# PeiSpiDemo — nRF52832 / BLYST-NANO-DK

Import `ioc` as an Eclipse Embedded CDT managed project.

No external SPI slave is required. The nRF52832 SPIM2 master transmits
a fixed eight-byte pattern from its driver-owned TX buffer. Its
SCK (P0.14) and MOSI (P0.15) are not connected to any external
device; MISO (P0.16) is an unused input. No chip select pin is used.

## Functional test

1. On boot, inspect `g_PeiSpiStatus`; **1** means setup and PPI routes
   are armed. A negative value identifies the failed configuration step.
2. Press BUT1 (P0.02). The GPIOTE event triggers SPIM2 START via PPI.
3. SPIM2 END triggers both SPIM2 STOP and physical LED1 (P0.30) toggle.
4. SPIM2 STOPPED toggles RGB blue (P0.27) separately.
5. Repeated BUT1 presses repeat the same prepared eight-byte transfer.

All four event/task links are real PPI hardware; no peripheral IRQ
forwards events to tasks. LED1 and RGB blue are distinct outputs.

To check actual SPI wire-level timing or bytes, attach a logic
analyzer to SCK and MOSI. LED indications alone confirm PPI event
routing, not slave communication or correct received data.

The example deliberately keeps SPIM2 armed indefinitely. A normal
application must disable and disconnect all routes, then call
`SPIReleasePeiTransfer()` before using the same SPI instance for
ordinary transfers, changing the configuration, or releasing the
caller-owned RX DMA buffer. Hardware build, pin behavior and
cross-variant tests are pending.

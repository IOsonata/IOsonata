# nRF5340 application-core network starter

This is the existing minimal application-core firmware for running a standalone
nRF5340 network-core image on the nRF5340 Audio DK (default) or the nRF5340 DK
(with `NRF5340_DK_PCA10095` defined).

The application core normally holds the network core off after reset.
`NetCoreStart` uses `IOPinCoreCfg` to assign the network UART pins to the
network core, then clears `NETWORK.FORCEOFF`. It lights the board LED when
the release operation has completed and remains in `WFI`.

On the Audio DK the network-core VCOM uses P1.09 TX and P1.08 RX, exposed
through the board's second virtual COM port. The standard nRF5340 DK uses
P1.01 TX and P1.00 RX. See `src/board.h` for the pin definitions.

## Build and flash

1. Import the existing `lib/ioc` managed library project for nRF5340 App,
   and `NetCoreStart/ioc` into IOcomposer. Build matching Debug/Release
   configurations.
2. Build and flash the desired **network-core** firmware, such as
   `nRF5340_Net/exemples/ClockDemo/ioc`, to the **network core**.
3. Flash `NetCoreStart` to the **application core**.
4. Open the network-core virtual COM port at the baud rate configured in the
   network example (1,000,000 baud, 8N1 for ClockDemo; no hardware flow
   control). Reset the board and inspect the network core's UART output.

Flash the two images to their respective cores, not to the same core.
This application-core launcher replaces any previous application-core image.
It does not implement power/clock validation itself; that remains in the
network-core demo. It does not require changing the IOsonata power/clock API.

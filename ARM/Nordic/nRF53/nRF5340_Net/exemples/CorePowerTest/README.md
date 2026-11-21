# nRF5340 Network Core PowerApiTest

The nRF5340 network-core PowerApiTest uses the shared
`exemples/misc/power_api_test.cpp` source without target-specific sleep logic.
It exercises resumable `SystemSleep()` using a 1,000 ms LF timer wake, 
the network-core UARTE0 console at 1,000,000 baud, and static/stack RAM
continuity checks.

The default board is the nRF5340 Audio DK (PCA10121), with network VCOM
RX P1.08 and TX P1.09. The `NRF5340_DK_PCA10095` symbol selects the
standard DK (RX P1.00, TX P1.01).

Flash `NetCoreStart` on the application core first. It assigns the network
UART pins, prepares/holds shared LFXO, then releases the network core.
Flash `PowerApiTest` to the network core and power-cycle the board.

Expected console progression: `PowerApiTest: Standby`,
`Entering Standby; automatic timer wake`, repeated resume counts,
RAM sample checks, and clock/UART restoration. A failed timer initialization
or unexpected immediate return is not a pass.

The network-core SystemSleep backend uses WFI with SLEEPDEEP cleared,\npreserving the original PRIMASK and SCR values. It does not issue an\nindependent system-off request or shut down the application core's LFCLK.\n\nOnly resumable SystemSleep is tested. This example does not attempt
network-core System OFF, which is an application-core power-domain operation.

Build via IOcomposer managed projects:

```sh
python3 tests/nordic/build_managed_power_clock.py --target nrf5340_net
```

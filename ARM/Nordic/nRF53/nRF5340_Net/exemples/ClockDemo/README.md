# nRF5340_Net ClockDemo

Import `ioc` as an existing Eclipse managed project and build the matching
IOsonata library in the same Debug or Release configuration. Console pins and
oscillator selections are in `src/board.h`, copied from this target's
TimerDemo project. Default console speed is 1,000,000 baud unless board.h
specifies another rate. No debugger is required for UART logging.

Bare-metal HFXO requests/releases are balanced. No BLE stack is started.

This project runs three clock-selection cycles. UART is drained and powered
off before each change, then initialized again to update its baud divider.
Clock selection completes before the timer is initialized because some ports
retain timer module ownership after TimerDisable. It then checks the first
high-frequency timer at 1 MHz requested rate, with a 10 ms trigger over 250 ms.
The returned timer rate may differ from 1 MHz. Delay and interrupt checks use
the actual rate/period. They do not independently measure oscillator accuracy.
No watchdog is enabled. No sleep is requested by this configuration; use
PowerOffDemo for target wake validation. Reset restores startup clocks.

An omitted selector prints SKIP. A configured selector that fails prints FAIL;
unsupported selections are not silently counted as successful tests. Runtime
clock ownership, board wiring and hardware operation still require board
validation. See the cross-target power/clock demo matrix for build results.

## nRF5340 Audio DK hardware test (network core)

The network-core image cannot start alone after a board reset. Use the existing
application-core `NetCoreStart` image to assign the VCOM GPIO pins to the
network core and release it from reset. `NetCoreStart` is an Eclipse managed
project in `ARM/Nordic/nRF53/nRF5340_App/exemples/NetCoreStart/ioc`.

1. Build `IOsonata_nRF5340_App` and `NetCoreStart` in the same Debug or Release
   configuration. Build `IOsonata_nRF5340_Net` and this `ClockDemo` in the
   corresponding configuration. Use the existing IOcomposer managed projects.
2. Flash `ClockDemo` to the **network core**, then `NetCoreStart` to the
   **application core**. Keep both images installed; resetting or power cycling
   the board restarts the application core first.
3. Connect the Audio DK through the onboard debug USB connector and open the
   **network core's second virtual COM port** at **1,000,000 baud, 8N1**,
   without hardware flow control. On macOS, list available ports with
   `ls /dev/cu.usbmodem*`; identify the two board ports rather than assuming
   their device-name suffix or ordering.
4. Reset the board. `NetCoreStart` hands Audio DK P1.08 (network UART RX) and
   P1.09 (network UART TX) to the network core using `IOPinCoreCfg`, then
   clears `NETWORK.FORCEOFF`. The network `ClockDemo` should print its clock
   cycle and timer results on that second virtual COM port.

`NetCoreStart` is deliberately a minimal application-core launcher. Do not
flash it over an application-core firmware that must remain operational. If
there is no UART output, first verify that both core images are flashed and
that the application-core launcher actually ran. This test validates the
network core's clock behavior; it does not independently validate application-
core sleep or network-core system-power transitions.

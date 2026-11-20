# HFXO test with Nordic BLE clock ownership

The existing HfxoTest is the bare-metal test. Each MCU has one IOcomposer managed project with four configurations for
SoftDevice and MPSL clock ownership.

| MCU | Project | Configuration | Clock owner |
| --- | --- | --- | --- |
| nRF52832 | HfxoTestBle | Debug / Release | S132 SoftDevice |
| nRF52832 | HfxoTestBle | Debug_SDC / Release_SDC | MPSL |
| nRF52840 | HfxoTestBle | Debug / Release | S140 SoftDevice |
| nRF52840 | HfxoTestBle | Debug_SDC / Release_SDC | MPSL |

The Debug/Release configurations call `nrf_sdh_enable_request()` before testing
`SystemHighFreqClockEnable()` and `SystemHighFreqClockDisable()`. Program the matching SoftDevice image before
starting this application. Use the corresponding IOsonata library build.

The Debug_SDC/Release_SDC configurations call `MpslInit()`. MPSL handles the clock requests used by
SDC, but this example does not initialize the full Bluetooth controller or
start radio activity. It tests MPSL clock ownership only.

Debug and Debug_SDC use `printf` with semihosting. Enable semihosting in
the debugger. Release and Release_SDC use the board UART at 1000000 baud.

The test enables HFXO twice and disables it twice, reporting the running
state after each operation. The clock may remain running after the final
disable when another part of the system owns it. Disable returns void, so
the test observes the clock state instead of checking a return value. An
additional unmatched disable must not change the clock state.

These projects do not test enabling HFXO before starting the BLE stack.
That handover remains unvalidated. No automatic hardware pass is implied by
a successful build.

# HFXO test with Nordic BLE clock ownership

The existing HfxoTest is the bare-metal test. These are separate IOcomposer
managed projects for the clock owners used by Bluetooth applications.

| MCU | Project | Configuration | Clock owner |
| --- | --- | --- | --- |
| nRF52832 | HfxoTestSd | Debug / Release | S132 SoftDevice |
| nRF52840 | HfxoTestSd | Debug / Release | S140 SoftDevice |
| nRF52832 | HfxoTestSdc | Debug_SDC / Release_SDC | MPSL |
| nRF52840 | HfxoTestSdc | Debug_SDC / Release_SDC | MPSL |

HfxoTestSd calls `nrf_sdh_enable_request()` before testing
`SystemCoreClockSelect()`. Program the matching SoftDevice image before
starting this application. Use the corresponding IOsonata library build.

HfxoTestSdc calls `MpslInit()`. MPSL handles the clock requests used by
SDC, but this example does not initialize the full Bluetooth controller or
start radio activity. It tests MPSL clock ownership only.

Debug and Debug_SDC use `printf` with semihosting. Enable semihosting in
the debugger. Release and Release_SDC use the board UART at 1000000 baud.

The test makes two HFXO requests and releases them in order, reporting the
running state after each operation. The clock may remain running after the
last release when another part of the system needs it. The final release
therefore checks the return value, not whether HFXO stopped. An additional
release without a matching request must fail.

These projects do not test a clock request made before enabling a BLE stack.
That handover remains unvalidated. No automatic hardware pass is implied by
a successful build.

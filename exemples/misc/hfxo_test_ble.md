# HFXO clock ownership test with Nordic BLE stacks

This is an internal clock-backend diagnostic. It exercises HFXO
reference counting across Nordic Bluetooth-stack initialization and
shutdown. It is **not** an example of how an application should initialize
Bluetooth.

Applications use IOsonata's generic `BtAppInit()` interface. The Nordic
Bluetooth backend initializes and owns S132/S140 or SDC/MPSL as required.
An application does not call `MpslInit()`, `nrf_sdh_enable_request()`,
`nrf_sdh_disable_request()`, or any Nordic clock-control API.

Likewise, an application does not select the HF oscillator or choose a
clock-owner mode. The board oscillator is described by `g_McuOsc`, which
is established at system initialization. Drivers request and release the
clocks they need via generic IOsonata clock interfaces.

## Diagnostic builds

| MCU | Project | Build configuration | Vendor stack |
| --- | --- | --- | --- |
| nRF52832 | HfxoTestBle | Debug / Release | S132 |
| nRF52832 | HfxoTestBle | Debug_SDC / Release_SDC | SDC/MPSL |
| nRF52840 | HfxoTestBle | Debug / Release | S140 |
| nRF52840 | HfxoTestBle | Debug_SDC / Release_SDC | SDC/MPSL |

Debug builds use semihosting and Release builds use UART at
1,000,000 baud. UART Release output is interrupt-driven with EasyDMA.
The diagnostic waits for pending UART output before its final idle loop.

The diagnostic automatically:

1. Acquires HFXO through the generic clock API before initializing BLE.
2. Initializes the selected Nordic stack from its test harness.
3. Verifies HFXO continuity across initialization.
4. Releases the pre-stack request and checks clock ownership.
5. Reacquires and releases HFXO after stack initialization.
6. Runs the normal two-enable/two-disable reference-count test.
7. For S132, disables SoftDevice and checks direct HFXO reacquisition
   and release after shutdown.

There is no `HFXO_TEST_PRESTACK` mode or application clock-source
selection in this diagnostic.

### What the diagnostic does not prove

It tests clock ownership, not advertising, connections, radio timing,
LFRC calibration, or current consumption. Real Bluetooth traffic and
low-power measurements remain separate integration checks.

On nRF52 LFXO systems, the MPSL external clock callbacks allow the
shared backend to retain HF/LF clock ownership across initialization.
The LFRC calibration and external-clock integration still require
separate review and validation.

## Running the checks

```sh
python3 tests/nordic/check_clock_hfxo_source.py
python3 tests/nordic/build_managed_power_clock.py --target nrf52832 --config Release_SDC
```

Build the application with the matching Eclipse managed configuration.
Neither the application nor the MCU HAL library needs an additional
clock-owner configuration switch.

The S132 startup/handover/shutdown sequences and nRF52832 SDC LFXO
handover sequences have been reported PASS in hardware testing.
Results for other MCU targets, LFRC, and ongoing BLE radio operation
remain open.

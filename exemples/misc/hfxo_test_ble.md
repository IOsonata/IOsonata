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

## Optional pre-stack HFXO diagnostic

The default HfxoTestBle still initializes the BLE owner first. To exercise
the opposite order, add `HFXO_TEST_PRESTACK=1` to the **application**
compiler preprocessor definitions in the desired Eclipse managed
configuration, clean-build and flash that configuration. Do not change the
library project settings or the generic clock API.

The diagnostic:

1. Selects HFXO and calls `SystemHighFreqClockEnable()` before the stack.
2. Checks that the oscillator is running.
3. Initializes S132 or MPSL using the existing `HfxoStackInit()`.
4. Checks the oscillator is still running.
5. Calls `SystemHighFreqClockDisable()`, then probes the existing
   `SystemCoreClockSelect()` interface to detect an outstanding shared
   clock request. A refusal is reported as FAIL.
6. Only when step 5 succeeds, tests another HF clock enable/disable
   through the initialized BLE owner.

This is a **diagnostic**, not a claim that direct-to-stack clock ownership
transfer already works. The current Nordic backend deliberately refuses a
direct CLOCK STOP after the stack becomes active. It may retain a clock
reference; in that case the diagnostic stops with a clear FAIL rather than
writing protected registers or pretending the release succeeded.

Run separately with S132 and MPSL. Save the full output, including
`Pre-stack request disabled` and `Pre-stack handover`. An unsuccessful or
hanging stack initialization is also a failed handover and must be
investigated before any production owner-transfer change. Do not use this
mode in a product or with ongoing RADIO activity. There is no assertion
that a clock can be transferred without interruption on all Nordic MCUs.

The existing stack-first tests have already passed on nRF52832 using
S132 and SDC/MPSL. The pre-stack diagnostic has not been hardware-tested,
and a successful managed build is not hardware evidence.

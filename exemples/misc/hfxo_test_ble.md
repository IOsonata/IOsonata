# Portable Bluetooth clock test

`HfxoTestBle` is a common IOsonata application source for Nordic
SoftDevice, Nordic SDC/MPSL, and STM32WBA. It contains no vendor BLE
headers, stack-selection macros, vendor initialization calls, or
vendor-specific clock register access.

The application uses only these IOsonata interfaces:

- `BtAppInit()` to initialize Bluetooth and its platform stack
- `SystemHighFreqClockEnable()` and `SystemHighFreqClockDisable()`
  to exercise shared clock requests before and after Bluetooth startup
- `AppRun()` to serve queued Bluetooth events

The board describes its oscillators through `g_McuOsc`; normal
peripheral applications rely on drivers to request clocks automatically.

## Test sequence

1. Request the shared high-frequency clock.
2. Initialize Bluetooth using `BtAppInit()`.
3. Request the clock a second time.
4. Release both requests in reverse order.
5. Request and release the clock again after Bluetooth initialization.
6. Print PASS only if all four checked requests and Bluetooth
   initialization succeed.
7. Drain UART output and run the generic Bluetooth event loop.

This portable test verifies API availability and request success.
It **does not** assert a physical oscillator stopped after a logical
clock release. The STM32WBA high-frequency oscillator may be the core
system clock and must stay on while the MCU is running.

Release builds use interrupt-driven UART and EasyDMA where supported.
Debug builds retain semihosting.

## Hardware-specific validation

The former low-level Nordic HFXO diagnostic was preserved at
`tests/nordic/hfxo_test_ble_vendor_diagnostic.cpp`. It tests
physical HFXO state, SoftDevice shutdown, and direct-clock
reacquisition, which do not belong in portable application code.

The vendor diagnostic must be built as an internal platform test
rather than compiled into the common application.

For the nRF52832 SDC LFRC ownership check, compile the separate
`tests/nordic/hfxo_test_ble_vendor_diagnostic.cpp` diagnostic with the
SDC library. It selects LFRC before MPSL initializes, keeps MPSL's own
LFRC calibration, holds a direct HFXO request across MPSL initialization,
and reports the physical HFXO state immediately after that request is
released. Confirm `Select LFRC: PASS` and `Clock survives stack init: PASS`.
After the pre-stack release, the diagnostic samples HFXO every 10 ms for
200 samples (2 seconds) and reports how many times it was stopped.
LFRC calibration can temporarily run HFXO, so one running sample is not
evidence of a leaked clock reference. If HFXO never stops during this
window, the diagnostic reports `Pre-stack physical release: FAIL`.
The portable `HfxoTestBle: PASS` cannot establish physical HFXO release.

The nRF52832 SDC/LFRC hardware test recorded zero stopped samples
out of 200 over two seconds after the pre-stack HFXO release. This
shows a continuously running HFXO during the observation window. The
implementation previously tried an MPSL HF request/release pair, but
that pair does not release the HFXO request started directly before
MPSL initialization. That incorrect transfer has been removed: the
original request remains recorded instead of silently disappearing.
A safe ownership transfer is still required, without disabling MPSL's
LFRC calibration or stopping a clock required by the radio.

Do not replace the shared `exemples/misc/hfxo_test_ble.cpp` source
with Nordic-specific initialization. The LFRC diagnostic was compiled
in a locally modified Eclipse SDC project and run on nRF52832 hardware;
a dedicated managed diagnostic project has not yet been committed.

## Status

The source refactor has been committed but **not** compiled or
hardware-tested across all three Bluetooth implementations. The
existing Nordic HfxoTestBle Eclipse configurations must link all
necessary IOsonata Bluetooth implementation objects, not only
the MPSL or SoftDevice clock setup used by the previous diagnostic.
STM32WBA also requires an Eclipse managed application configuration
that links this same source. Those project-build integrations remain
to be verified.

The STM32WBA `system_stm32wbaxx.c` implementation provides the generic HF request
functions by validating that its selected core oscillator is ready.
It does not gate an oscillator that supplies the CPU.

Run the Nordic clock source checks:

```sh
python3 tests/nordic/check_clock_hfxo_source.py
```

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

The nRF52832 SDC/LFRC hardware test previously recorded zero stopped
samples out of 200 over two seconds after the pre-stack HFXO release.
The former internal-MPSL implementation could not release the original
direct HFXO start.

The new implementation registers external clock control for LFRC as
well as LFXO. IOsonata now handles LFRC calibration with a separate
HFXO reference, the CLOCK DONE interrupt, and a four-second calibration
timer. The change is **not yet built or hardware-validated**. The
diagnostic must show HFXO stopping between calibration intervals without
interrupting subsequent clock requests or Bluetooth operation.

## Periodic LFRC calibration test

After the pre-stack handover and five HFXO lifecycle checks, the
nRF52832 SDC diagnostic observes another 12 seconds. It reads an internal
counter incremented only when CLOCK reports `EVENTS_DONE`, and samples
the HFXO state at 100 ms intervals.

Expected after rebuilding the IOsonata library and the diagnostic:

```text
LFRC calibration: 2 completed in 12 s, HFXO running 0/120, stopped 120/120
LFRC periodic calibration: PASS
```

The counts above illustrate the output format, not a predicted hardware
measurement. The test requires at least two completed calibrations and at
least one stopped HFXO sample. It verifies periodic calibration of the
nRF52 CLOCK peripheral with MPSL initialized, **not** BLE advertising,
connections, LFRC frequency accuracy or temperature compensation. These
need independent hardware tests before Nordic sign-off.

Do not replace the shared `exemples/misc/hfxo_test_ble.cpp` source
with Nordic-specific initialization. The LFRC diagnostic was compiled
in a locally modified Eclipse SDC project and run on nRF52832 hardware;
a dedicated managed diagnostic project has not yet been committed.

## Status

The portable `HfxoTestBle` application has passed on nRF52832
with S132 and SDC. The nRF52832 `ClockDemo` and `PowerOffDemo` have
also passed on hardware. The new external LFRC calibration implementation
has not yet been compiled or hardware-tested. Run the Nordic diagnostic
with the rebuilt SDC library before assessing oscillator release.

STM32WBA is deferred until Nordic power and clock work is complete.

The STM32WBA `system_stm32wbaxx.c` implementation provides the generic HF request
functions by validating that its selected core oscillator is ready.
It does not gate an oscillator that supplies the CPU.

Run the Nordic clock source checks:

```sh
python3 tests/nordic/check_clock_hfxo_source.py
```

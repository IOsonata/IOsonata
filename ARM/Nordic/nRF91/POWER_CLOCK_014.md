# nRF91 power and clock port — IOsonata 0.14

## Targets and scope

The nRF9160 and nRF91x1 libraries use the Nordic shared `system_clock_nrfx.cpp`,
`system_power_nrfx.cpp` and RTC/HF timer drivers. The 0.14 port is not
hardware-complete merely because these sources build.

- Core clock: fixed 64 MHz; PCLK16M is 16 MHz. Requests for unsupported
  clock divisors return the hardware frequency rather than changing it.
- HF clock and LF clock: the shared system Enable/Disable owners count requests.
  RTC timers request LFCLK through those APIs, not a private oscillator owner.
- System ON IDLE: `SystemSleep()` preserves SCR and the original interrupt mask,
  and returns after an interrupt. System OFF remains reset-on-wake.
- `sDelay()`: uses an available low-frequency RTC with WFE and returns it to
  the timer pool. On nRF91, the clock owner's observed cold-start polling
  duration is compensated, as on the nRF5340 application core.
- An nRF91 LFRC is not factory-calibrated through CLOCK CAL tasks. To avoid
  reporting a substantially incorrect seconds delay, `sDelay()` refuses
  the uncalibrated LFRC source. It uses LFXO for nominally accurate seconds.
  This is not a synthesized LFCLK and does not change `g_McuOsc`.
- nRF91 LFCLK start completion is checked against the actual LFCLKSTAT
  source, not just the LFCLKSTARTED event. Existing modem ownership must
  be preserved; the modem requires LFXO.

## nRF9160 hardware observation (2026-11-23)

The first CoreClockTest transcript reports four LFRC-only timing failures:

| Timer | CPU delay | RTC-reported elapsed | Independent HF elapsed |
| --- | --- | --- | --- |
| RTC0 | msDelay 250 ms | 342224 us | 250215 us |
| RTC0 | nsDelay 250 ms | 342377 us | 250214 us |
| RTC1 | msDelay 250 ms | 342285 us | 250213 us |
| RTC1 | nsDelay 250 ms | 342346 us | 250212 us |

These imply about 44.83 kHz of physical RTC tick rate when the driver uses
a nominal 32768 Hz. The RC is documented by Nordic as having large
frequency variation (nominal 32768 Hz, 30% tolerance), and the observed
ratio is about 36.8% above nominal. The physical oscillator/source register
state and test conditions therefore require investigation; do not waive
these four failures or label them as PASS.

After restoring the crystal source, RTC0/RTC1 measured approximately
250152 us for a real 250156–250158 us. All three 16 MHz HF timers also
passed. CPU delay calibration and 64 MHz / 16 MHz clock reporting passed.
That transcript predated the new nRF91 `sDelay()` integration.

## Remaining validation

1. Run source tests: `python3 tests/nordic/check_nrf91_power_clock_014.py`.
2. Run Debug and Release managed builds for `nrf9160` and `nrf91x1`
   through `tests/nordic/build_managed_power_clock.py`.
3. Run CoreClockTest on nRF9160 using the updated code and inspect
   LFXO cold/warm `sDelay`, ownership and LFRC timing separately.
4. Diagnose LFRC frequency directly against HF timer, including
   LFCLKSTAT active-source confirmation. Preserve the failing timing
   result until its cause is established.
5. Run CorePowerTest on nRF9160 and nRF91x1, including timer wake,
   GPIO/System OFF wake, retention, UART restoration, and reset reasons.
   A bare-metal System OFF test does not validate modem-active shutdown.
6. Preserve all previously validated nRF52, nRF5340 and nRF54L behavior.

No default watchdog is enabled. No target-specific code belongs in the
shared CoreClockTest or CorePowerTest examples.

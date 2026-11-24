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

## nRF9160 DK CorePowerTest wake button

The nRF9160 DK's default hardware routing connects Button 1 to GPIO P0.06,
active low. The nRF9160 CorePowerTest board file now defines this button
as a wake input; the generic test applies the required pull-up resistor,
configures GPIO SENSE, and verifies wake after System OFF without a debugger.
Button routing depends on the nRF52840 board-controller firmware, so test
on the DK with its standard routing. No Thingy:91 button mapping is
assumed. The existing VCOM0 UART pins are unchanged.

Prior validation of three successive System OFF GPIO wakes on nRF91x1
applies to that target's earlier bare-metal power demo, not to the
updated nRF9160 CorePowerTest or to modem-active shutdown.

## nRF9160 delay divisors (2026-11-23)

CoreClockTest on the nRF9160 at 64 MHz measured `msDelay(100)` at
100,010 us and `nsDelay(100 ms)` at 100,011 us with the 16,000,000 and
3,000,000 divisors, 3,999,600 us loops/s and 21,330,986 ns loops/s.
`system_nrf91.c` now uses the measured divisors 16,001,600 and 3,000,330.
The file is shared by the nRF9160 and nRF91x1 libraries; the values come
from one nRF9160 run and are not yet measured on an nRF91x1. Rerun
CoreClockTest to confirm both delays at 100,000 us.

## Remaining validation

1. Run source tests: `python3 tests/nordic/check_nrf91_power_clock_014.py`.
2. Run Debug and Release managed builds for `nrf9160` and `nrf91x1`
   through `tests/nordic/build_managed_power_clock.py`.
3. Run CoreClockTest on nRF9160 using the updated code and inspect
   LFXO cold/warm `sDelay`, ownership and LFRC timing separately.
4. Diagnose LFRC frequency directly against HF timer, including
   LFCLKSTAT source confirmation. The generic CoreClockTest now reports
   raw LF RTC tick rate calculated against the independent HF timer,
   distinguishing physical oscillator drift from a faulty conversion.
   Preserve the failing timing result until its cause is established.
5. Run CorePowerTest on nRF9160 and nRF91x1, including timer wake,
   GPIO/System OFF wake, retention, UART restoration, and reset reasons.
   A bare-metal System OFF test does not validate modem-active shutdown.
6. Preserve all previously validated nRF52, nRF5340 and nRF54L behavior.

No default watchdog is enabled. No target-specific code belongs in the
shared CoreClockTest or CorePowerTest examples.

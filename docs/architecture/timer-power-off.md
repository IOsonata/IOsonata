# TimerPowerOff: generic timer ownership lifecycle

IOsonata uses Init, Enable, Disable and PowerOff vocabulary. TimerPowerOff
completes the timer lifecycle, and is independent of any MCU family.

```cpp
TimerDev_t timer = {};
TimerCfg_t cfg = {};
// Configure cfg.DevNo, cfg.ClkSrc, cfg.Freq, cfg.IntPrio.
if (TimerInit(&timer, &cfg))
{
    TimerDisable(&timer);  // Stops counting, retains ownership.
    TimerEnable(&timer);   // Resumes the same handle.
    TimerPowerOff(&timer); // Releases the timer for another handle.
}
```

## Contract

- `TimerInit` establishes hardware ownership.
- `TimerDisable` stops the timer but preserves its handle, trigger
  configuration and right to `TimerEnable` later.
- `TimerPowerOff` stops counting, releases a clock reference held by
  the timer, cleans interrupts and locally owned compare triggers,
  and removes the driver's ownership registration.
- A successful PowerOff returns true, resets the handle's DevNo to -1
  and Freq to 0, and requires another TimerInit before further use.
  A repeated PowerOff without initialization returns false.
- A failed PowerOff returns false without reporting resources free.
  Drivers refuse shutdown while a PEI route or other externally owned
  event-to-task binding cannot safely be disconnected.
- Timer power-off is not the same as system power-off. Shared clock
  roots and counters are preserved when owned by another driver.
- This is a static-allocation API. No heap, Acquire/Release vocabulary,
  or target-specific application code is required.

The C interface is `bool TimerPowerOff(TimerDev_t *pTimer)`; the C++
`Timer` wrapper exposes `bool PowerOff()`. Both use the same backend
callback. An unsupported backend sets the callback to null and the
generic API returns false instead of pretending the peripheral was
released.

## Backend status — release 0.14

| Family | TimerPowerOff support |
| --- | --- |
| Nordic nRF52/nRF53/nRF91 HF TIMER and RTC | Implemented, PEI routes must be unbound |
| Nordic nRF54L GRTC | Implemented per logical compare group; shared SYSCOUNTER retained |
| Microchip SAMD RTC, TC, TCC and SysTick wrapper | Implemented with existing release path |
| Microchip SAM4E TC | Implemented |
| Microchip SAM4L AST and TC | Implemented |
| ST STM32 TIM and LPTIM | Implemented through existing release routines |
| NXP LPC, Renesas RA4/RE01/R9A02, Espressif ESP32-C, standalone SysTick | API present; backend release pending (returns false) |

These are **source-implemented** statuses, not hardware validation. They
require the normal Debug/Release compile matrix and target testing before
sign-off. Existing CoreClockTest results do not validate a new PowerOff
path. Do not weaken exclusive hardware ownership merely to make an
example's use of two independent timer handles succeed.

## nRF9160 validation to add

1. Initialize RTC0, disable it, PowerOff, then initialize it using a
   different TimerDev_t and verify timer callbacks and LFCLK ownership.
2. Repeat for TIMER0 and HFXO ownership.
3. PowerOff an active timer and verify no callbacks after shutdown,
   then reinitialize and verify compare triggers again.
4. PowerOff must refuse live PEI bindings. A second PowerOff must
   return false. Changing a timer handle must not lose other timers.
5. Verify sDelay and SystemSleep retain correct clock timing after
   repeated timer PowerOff/Init cycles.

For nRF91, modem-active timer ownership is a separate validation
boundary; a bare-metal pass is not proof of modem interaction safety.

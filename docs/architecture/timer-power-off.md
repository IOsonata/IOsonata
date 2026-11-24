# TimerPowerOff

## Timer functions

`TimerInit()` configures a hardware timer and assigns it to a
`TimerDev_t`. `TimerDisable()` stops counting without removing that
assignment. `TimerEnable()` resumes the same timer.

`TimerPowerOff()` stops counting, disables the timer interrupts, clears
the timer's compare settings, removes its device assignment and turns
off its peripheral clock when that clock is not needed by another
timer. A subsequent `TimerInit()` may assign the same hardware timer
to a different `TimerDev_t`.

The generic C and C++ functions are:

```cpp
bool TimerPowerOff(TimerDev_t *pTimer);
bool Timer::PowerOff();
```

Example:

```cpp
TimerDev_t timer = {};
TimerCfg_t cfg = {
	.DevNo = TimerGetHighFreqDevNo(),
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0U,
	.IntPrio = 2,
	.EvtHandler = nullptr,
};
if (TimerInit(&timer, &cfg))
{
	TimerDisable(&timer);
	TimerEnable(&timer);
	if (TimerPowerOff(&timer))
	{
		if (TimerInit(&timer, &cfg))
		{
			TimerPowerOff(&timer);
		}
	}
}
```

`TimerPowerOff()` returns `true` only when the timer has been stopped
and its device assignment removed. The old `TimerDev_t` must not be
used again until it has been initialized. A second PowerOff on the
same structure returns `false`. PowerOff also returns `false` if
the timer is not initialized or hardware resources cannot safely be
disconnected.

A timer with active PEI event connections must first have those
connections disconnected. The Nordic RTC/HF TIMER drivers refuse
PowerOff while such a connection is present. The nRF54L GRTC driver
frees the selected compare group without stopping the shared
SYSCOUNTER. Shared peripheral clocks on other MCUs are not stopped
while another timer still uses them.

TimerPowerOff uses static storage only. It does not allocate memory,
configure a new oscillator or change the operation of TimerDisable.

## MCU timer drivers

The following source files now contain TimerPowerOff functions.
**Compiling and testing these changes on hardware remains necessary.**

| MCU | Hardware timer | TimerPowerOff implementation |
| --- | --- | --- |
| Nordic nRF52, nRF53, nRF91 | RTC, TIMER | Stop, clear interrupts/compares, disable held clocks, clear timer assignment |
| Nordic nRF54L | GRTC compare groups and TIMER | Free selected compare channels; preserve shared SYSCOUNTER |
| Microchip SAMD | RTC, TC, TCC, SysTick wrapper | Use existing timer shutdown functions |
| Microchip SAM4L | AST, TC | Stop timer, disable triggers and IRQs, free device |
| Microchip SAM4E | TC | Stop timer, disable interrupts and peripheral clock, free device |
| NXP LPC11xx, LPC17xx, LPC546xx | Hardware TIMER/CTIMER | Stop, disable IRQs, disable timer clock, free device |
| ST STM32 | TIM, LPTIM | Use existing TIM/LPTIM shutdown functions |
| Renesas RA4M1 | AGT, GPT | Stop, unregister interrupts, stop unused GPT module clocks, free device |
| Renesas RE01 | AGT, TMR, GPT | Stop, unregister interrupts, stop unused module clocks, free device |
| Renesas R9A02G021 | Machine timer, TAU | Stop, unregister interrupt handlers; stop TAU clock when no channel uses it |
| Espressif ESP32-C | Timer Group | Stop, disconnect CPU interrupt, disable timer clock, free device |
| ARM SysTick-only timer | SysTick | Stop SysTick, clear pending interrupt and free timer structure |

The generic CoreClockTest calls the same Timer API on every MCU. Its
PowerOff checks include initializing the first HF timer again with the
same structure after PowerOff, shutting it down while running and
rejecting an additional PowerOff.

## Required tests

1. Compile the affected MCU libraries in Debug and Release.
2. Run the generic CoreClockTest on each available development board.
3. Verify that TimerDisable and TimerEnable retain the timer's count
   and interrupt settings.
4. Verify that TimerPowerOff stops callbacks, releases the peripheral
   clock if appropriate and allows another timer structure to use it.
5. Verify that powering off one timer does not stop another timer or
   a peripheral sharing its clock.
6. For Nordic, verify LFRC/LFXO timing, HF oscillator requests, sDelay
   and PEI connections separately.
7. On modem-equipped nRF91 MCUs, verify the application and modem
   peripheral usage before using timer PowerOff in modem-active code.

A successful source check or prior CoreClockTest run does not replace
a hardware test of TimerPowerOff.

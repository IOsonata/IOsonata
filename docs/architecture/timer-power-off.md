# TimerPowerOff

## Timer functions

`TimerDisable()` stops timer operation and preserves the configuration.
Call `TimerEnable()` to resume, or `TimerInit()` to change the configuration.

`TimerPowerOff()` powers down the timer. The driver disables its interrupts,
clears its compare settings and turns off clocks used only by that timer.
Call `TimerInit()` to start it again.

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
PowerOff while such a connection is present. The nRF54L and nRF7120x GRTC driver
frees the selected compare group without stopping the shared
SYSCOUNTER.

## Nordic timer drivers

| MCU | Hardware |
| --- | --- |
| nRF52, nRF5340 application and network cores, nRF9160 and nRF91x1 | RTC and TIMER |
| nRF54L15, nRF54LM20A/B and nRF7120x application core | GRTC groups 0 through 3 and TIMER00, TIMER10, TIMER20 through TIMER24 |

## Microchip timer drivers

| MCU | Hardware | PowerOff clock behavior |
| --- | --- | --- |
| SAM D11 and D21 | RTC, TC, TCC and SysTick | Stops the peripheral and disables its bus clock. TC and TCC keep a shared generic clock channel while a sibling uses it. SysTick stops its counter and clears its pending interrupt. |
| SAM4L | AST and TC | Stops the AST prescaler or TC channel. Releases a driver-enabled TC block clock after the last assigned channel powers off. |
| SAM4E | TC | Stops the channel, disables its interrupt and gates its peripheral clock. |

## NXP timer drivers

The LPC11Uxx, LPC17xx and LPC546xx ports share `ARM/NXP/src/timer_lpcxx.cpp`.
PowerOff stops the counter, clears the match interrupts, the triggers and the
pending NVIC interrupt, releases the high frequency clock request of a running
timer and turns the timer clock off.

| MCU | Hardware | PowerOff clock behavior |
| --- | --- | --- |
| LPC11Uxx | CT16B0, CT16B1, CT32B0 and CT32B1 | Clears the SYSAHBCLKCTRL bit of the timer. |
| LPC17xx | TIMER0 to TIMER3 | Clears the PCONP bit of the timer. The PCLKSEL divider is kept. |
| LPC546xx | CTIMER3 and 4 on the async APB bridge, CTIMER0 to 2 | Clears the AHBCLKCTRL or async bridge clock bit of the timer. Turns the async bridge off after the last of CTIMER3 and 4. |

Initialize C timer structures to zero before their first use. Drivers with
no PowerOff callback return false. Other MCU families will implement
TimerPowerOff with their respective Power & Clock ports.

CoreClockTest checks PowerOff while disabled and while running, Init on the
same structure after PowerOff, and rejection of a repeated PowerOff.
The supplied nRF9160 CoreClockTest log passed these checks in both runs.

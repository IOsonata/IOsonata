/**-------------------------------------------------------------------------
@file	pwm_lpc546xx.cpp

@brief	LPC546xx PWM on the SCTimer (SCT0)

Generic PWM API (coredev/pwm.h) on SCT0, one device, DevNo 0. The SCT runs
as one 32 bit counter on the bus clock (SystemCoreClock), counting up from 0
to the period count - 1: event 0 on match 0 ends the period (LIMIT), event 9
on match 9 (0) starts it.

Channel Chan 0 to 7. Each channel uses match and event Chan + 1 and drives
the SCT0 output of its pin: Port, Pin and PinOp (IOPINOP_FUNCx) must be one
of the SCT0_OUT0 to SCT0_OUT9 pin functions of the LPC546xx pin data, two
open channels cannot drive the same output. The LPCXpresso54628 has SCT0_OUT4
on LED1 (P3_14, IOPINOP_FUNC1) and SCT0_OUT6 on LED3 (P2_2, IOPINOP_FUNC3),
both LEDs active low.

Active high (PWM_POL_HIGH): the start event sets the output, the channel
event clears it after the active counts. Active low swaps set and clear. The
active count of a duty cycle is the period count times the percentage,
rounded, it is also the match value of the channel: 0 is the count of the
start event, the conflict resolution (RES) keeps the output inactive, 100
percent gives the period count that the counter never reaches. Each period
sets its output level from its own match values. New duty cycles are written to the match reload registers,
they take effect at the next period. A new frequency takes effect at once:
the counter is halted, set and restarted.

The period count is SystemCoreClock / Freq rounded, at least 2:
PwmDev_t::Freq is the frequency obtained. The SCT writes COUNT, STATE, MATCH
and OUTPUT only while the counter is halted, the driver halts it for them.

The counter is halted, the outputs inactive, from Init to Start and after
Stop or Disable. An open channel drives its pin at its inactive level while
stopped. CloseChannel puts the pin back as a GPIO output at the inactive
level before the SCT output is released.

PWMInit of another PwmDev_t takes SCT0 over: the open channels of the first
one are closed and its calls fail from then on. PWMInit fails while SCT0 is
counting for other code (clock on, counter not halted, no owner).

Not supported, PWMInit or PWMStart fail: center mode (PWM_MODE_CENTER),
interrupt (bIntEn) and events, Start with a duration (msDur not 0).

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "LPC546xx.h"

#include "coredev/pwm.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"

#define LPC546XX_PWM_MAX_CHAN		8			// Matches and events 1 to 8
#define LPC546XX_PWM_EV_START		9			// Event and match of the period start
#define LPC546XX_PWM_RES_SET		1U			// Conflict resolution: set
#define LPC546XX_PWM_RES_CLR		2U			// Conflict resolution: clear
#define LPC546XX_PWM_EV_MATCH		1U			// Event COMBMODE: match only

typedef struct {
	PwmDev_t *pDev;							//!< Owner, null when SCT0 is free
	uint32_t Top;							//!< Counts of a period
	uint16_t OpenMask;						//!< Open channels
	uint8_t Out[LPC546XX_PWM_MAX_CHAN];		//!< SCT0 output of each open channel
	uint8_t Duty[LPC546XX_PWM_MAX_CHAN];	//!< Duty cycle of each channel, percent
	PwmChanCfg_t Chan[LPC546XX_PWM_MAX_CHAN];
} Lpc546xxPwm_t;

// SCT0 output pins of the LPC546xx: port, pin, function, output
static const uint8_t s_Lpc546xxPwmPin[][4] = {
	{ 0, 2, 3, 0 }, { 0, 17, 4, 0 }, { 1, 4, 4, 0 }, { 1, 23, 2, 0 }, { 3, 26, 2, 0 },
	{ 0, 3, 3, 1 }, { 0, 18, 4, 1 }, { 1, 8, 4, 1 }, { 1, 24, 2, 1 }, { 3, 27, 2, 1 },
	{ 0, 15, 4, 2 }, { 0, 19, 4, 2 }, { 1, 9, 4, 2 }, { 1, 25, 2, 2 }, { 3, 28, 2, 2 },
	{ 0, 22, 4, 3 }, { 0, 31, 4, 3 }, { 1, 10, 4, 3 }, { 1, 26, 2, 3 }, { 3, 10, 1, 3 }, { 3, 29, 2, 3 },
	{ 0, 23, 4, 4 }, { 1, 3, 4, 4 }, { 1, 17, 4, 4 }, { 3, 14, 1, 4 }, { 3, 30, 2, 4 },
	{ 0, 26, 4, 5 }, { 1, 18, 4, 5 }, { 3, 18, 5, 5 }, { 3, 31, 2, 5 },
	{ 0, 27, 4, 6 }, { 1, 31, 4, 6 }, { 2, 2, 3, 6 }, { 3, 19, 5, 6 },
	{ 0, 28, 4, 7 }, { 1, 19, 2, 7 }, { 2, 22, 2, 7 }, { 3, 20, 5, 7 },
	{ 0, 29, 4, 8 }, { 2, 23, 2, 8 }, { 3, 12, 1, 8 },
	{ 0, 30, 4, 9 }, { 2, 24, 2, 9 }, { 3, 13, 1, 9 },
};

static Lpc546xxPwm_t s_Lpc546xxPwm;

// Device data of the owner, null otherwise
static Lpc546xxPwm_t *Lpc546xxPwmDev(PwmDev_t * const pDev)
{
	if (pDev == nullptr || pDev->pDevData != &s_Lpc546xxPwm || s_Lpc546xxPwm.pDev != pDev)
	{
		return nullptr;
	}

	return &s_Lpc546xxPwm;
}

// SCT0 output of a pin function, -1 when the pin has none
static int Lpc546xxPwmOut(const PwmChanCfg_t &Cfg)
{
	for (const uint8_t *p : s_Lpc546xxPwmPin)
	{
		if (p[0] == Cfg.Port && p[1] == Cfg.Pin && Cfg.PinOp == IOPINOP_FUNC0 + p[2])
		{
			return p[3];
		}
	}

	return -1;
}

// Period count of the closest frequency, at least 2
static bool Lpc546xxPwmTop(uint32_t Freq, uint32_t *pTop, uint32_t *pFreq)
{
	uint32_t clk = SystemCoreClock;

	if (Freq == 0 || clk == 0)
	{
		return false;
	}

	uint32_t top = (uint32_t)(((uint64_t)clk + Freq / 2U) / Freq);

	if (top < 2)
	{
		top = 2;
	}

	*pTop = top;
	*pFreq = clk / top;

	return true;
}

// Match value of a channel, its active counts: 0 on the start event, the
// period count for 100 percent
static uint32_t Lpc546xxPwmMatch(Lpc546xxPwm_t *pDev, int Chan)
{
	return (uint32_t)(((uint64_t)pDev->Top * pDev->Duty[Chan] + 50U) / 100U);
}

// Halt the counter, true when it was running
static bool Lpc546xxPwmHalt(void)
{
	bool run = (SCT0->CTRL & SCT_CTRL_HALT_L_MASK) == 0;

	SCT0->CTRL |= SCT_CTRL_HALT_L_MASK;

	return run;
}

// Output level of a channel, active or inactive. Counter halted.
static void Lpc546xxPwmLevel(Lpc546xxPwm_t *pDev, int Chan, bool bActive)
{
	uint32_t bit = 1UL << pDev->Out[Chan];
	bool high = bActive == (pDev->Chan[Chan].Pol == PWM_POL_HIGH);

	SCT0->OUTPUT = high ? SCT0->OUTPUT | bit : SCT0->OUTPUT & ~bit;
}

// Match registers of the period and of the open channels, counter halted
static void Lpc546xxPwmLoad(Lpc546xxPwm_t *pDev)
{
	SCT0->MATCH[0] = pDev->Top - 1U;
	SCT0->MATCHREL[0] = pDev->Top - 1U;
	for (int i = 0; i < LPC546XX_PWM_MAX_CHAN; i++)
	{
		if (pDev->OpenMask & (1U << i))
		{
			uint32_t m = Lpc546xxPwmMatch(pDev, i);

			SCT0->MATCH[i + 1] = m;
			SCT0->MATCHREL[i + 1] = m;
		}
	}
}

// Counter from 0, then run: the events of count 0 set the level of every
// output
static void Lpc546xxPwmRun(void)
{
	SCT0->CTRL |= SCT_CTRL_CLRCTR_L_MASK;
	SCT0->CTRL &= ~SCT_CTRL_HALT_L_MASK;
}

// Pin of a closed channel: GPIO output at the inactive level
static void Lpc546xxPwmPinIdle(const PwmChanCfg_t &Chan)
{
	if (Chan.Pol == PWM_POL_HIGH)
	{
		IOPinClear(Chan.Port, Chan.Pin);
	}
	else
	{
		IOPinSet(Chan.Port, Chan.Pin);
	}
	IOPinConfig(Chan.Port, Chan.Pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

// The pin goes to GPIO at the inactive level before the output is released
static void Lpc546xxPwmClose(Lpc546xxPwm_t *pDev, int Chan)
{
	if (!(pDev->OpenMask & (1U << Chan)))
	{
		return;
	}

	int out = pDev->Out[Chan];

	Lpc546xxPwmPinIdle(pDev->Chan[Chan]);
	SCT0->OUT[out].SET = 0;
	SCT0->OUT[out].CLR = 0;
	SCT0->RES &= ~(SCT_RES_O0RES_MASK << (out * 2));
	SCT0->EV[Chan + 1].STATE = 0;
	pDev->OpenMask &= ~(1U << Chan);
}

bool PWMInit(PwmDev_t *pDev, const PwmCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->DevNo != 0 || pCfg->bIntEn || pCfg->Mode != PWM_MODE_EDGE)
	{
		return false;
	}

	Lpc546xxPwm_t *dev = &s_Lpc546xxPwm;

	// Counting without this driver: used by other code
	if (dev->pDev == nullptr && (SYSCON->AHBCLKCTRL[1] & SYSCON_AHBCLKCTRL_SCT0_MASK) &&
		(SCT0->CTRL & SCT_CTRL_HALT_L_MASK) == 0)
	{
		return false;
	}

	uint32_t top, freq;

	if (Lpc546xxPwmTop(pCfg->Freq, &top, &freq) == false)
	{
		return false;
	}

	// Initialized again, or taken from another PwmDev_t: the pins of the open
	// channels go back to idle
	for (int i = 0; dev->pDev != nullptr && i < LPC546XX_PWM_MAX_CHAN; i++)
	{
		Lpc546xxPwmClose(dev, i);
	}

	SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_SCT0_MASK;
	SYSCON->PRESETCTRLSET[1] = SYSCON_PRESETCTRL_SCT0_RST_MASK;
	SYSCON->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL_SCT0_RST_MASK;

	// One 32 bit counter on the bus clock, halted, counting up
	SCT0->CONFIG = SCT_CONFIG_UNIFY_MASK;
	SCT0->CTRL = SCT_CTRL_HALT_L_MASK | SCT_CTRL_CLRCTR_L_MASK;

	*dev = {};
	dev->Top = top;
	dev->pDev = pDev;
	Lpc546xxPwmLoad(dev);

	// Event 0 on match 0 in state 0 ends a period, event 9 on match 9 starts it
	SCT0->MATCH[LPC546XX_PWM_EV_START] = 0;
	SCT0->MATCHREL[LPC546XX_PWM_EV_START] = 0;
	SCT0->EV[0].STATE = 1;
	SCT0->EV[0].CTRL = SCT_EV_CTRL_MATCHSEL(0) | SCT_EV_CTRL_COMBMODE(LPC546XX_PWM_EV_MATCH);
	SCT0->EV[LPC546XX_PWM_EV_START].STATE = 1;
	SCT0->EV[LPC546XX_PWM_EV_START].CTRL = SCT_EV_CTRL_MATCHSEL(LPC546XX_PWM_EV_START) |
										   SCT_EV_CTRL_COMBMODE(LPC546XX_PWM_EV_MATCH);
	SCT0->LIMIT = 1;

	pDev->DevNo = 0;
	pDev->Freq = freq;
	pDev->Mode = pCfg->Mode;
	pDev->pEvtHandler = pCfg->pEvtHandler;
	pDev->pDevData = dev;

	return true;
}

bool PWMEnable(PwmDev_t *pDev)
{
	if (Lpc546xxPwmDev(pDev) == nullptr)
	{
		return false;
	}

	SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_SCT0_MASK;

	return true;
}

void PWMDisable(PwmDev_t *pDev)
{
	PWMStop(pDev);
}

bool PWMOpenChannel(PwmDev_t *pDev, const PwmChanCfg_t *pChanCfg, int NbChan)
{
	Lpc546xxPwm_t *dev = Lpc546xxPwmDev(pDev);

	if (dev == nullptr || pChanCfg == nullptr || NbChan <= 0)
	{
		return false;
	}

	for (int i = 0; i < NbChan; i++)
	{
		const PwmChanCfg_t &c = pChanCfg[i];
		int out = Lpc546xxPwmOut(c);

		if (c.Chan < 0 || c.Chan >= LPC546XX_PWM_MAX_CHAN || (c.Pol != PWM_POL_LOW && c.Pol != PWM_POL_HIGH) ||
			out < 0)
		{
			return false;
		}
		for (int j = 0; j < i; j++)
		{
			if (pChanCfg[j].Chan == c.Chan || Lpc546xxPwmOut(pChanCfg[j]) == out)
			{
				return false;
			}
		}

		// An output open on a channel this call does not set again
		for (int k = 0; k < LPC546XX_PWM_MAX_CHAN; k++)
		{
			if (!(dev->OpenMask & (1U << k)) || dev->Out[k] != out)
			{
				continue;
			}

			bool reset = false;

			for (int j = 0; j < NbChan; j++)
			{
				reset = reset || pChanCfg[j].Chan == k;
			}
			if (!reset)
			{
				return false;
			}
		}
	}

	for (int i = 0; i < NbChan; i++)
	{
		Lpc546xxPwmClose(dev, pChanCfg[i].Chan);
	}

	bool run = Lpc546xxPwmHalt();

	for (int i = 0; i < NbChan; i++)
	{
		const PwmChanCfg_t &c = pChanCfg[i];
		int ev = c.Chan + 1;
		int out = Lpc546xxPwmOut(c);
		uint32_t m;

		dev->Chan[c.Chan] = c;
		dev->Out[c.Chan] = out;
		dev->OpenMask |= 1U << c.Chan;
		m = Lpc546xxPwmMatch(dev, c.Chan);

		// Output at its inactive level before the pin goes to the SCT. Its
		// match from the next period, Start loads it in MATCH.
		Lpc546xxPwmLevel(dev, c.Chan, false);
		SCT0->MATCHREL[ev] = m;
		SCT0->EV[ev].STATE = 1;
		SCT0->EV[ev].CTRL = SCT_EV_CTRL_MATCHSEL(ev) | SCT_EV_CTRL_COMBMODE(LPC546XX_PWM_EV_MATCH);
		if (c.Pol == PWM_POL_HIGH)
		{
			SCT0->OUT[out].SET = 1UL << LPC546XX_PWM_EV_START;
			SCT0->OUT[out].CLR = 1UL << ev;
		}
		else
		{
			SCT0->OUT[out].SET = 1UL << ev;
			SCT0->OUT[out].CLR = 1UL << LPC546XX_PWM_EV_START;
		}
		// Start and channel events on the same count: inactive. Close left
		// the field of the output at 0.
		SCT0->RES |= (c.Pol == PWM_POL_HIGH ? LPC546XX_PWM_RES_CLR : LPC546XX_PWM_RES_SET) << (out * 2);
		IOPinConfig(c.Port, c.Pin, c.PinOp, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	}

	if (run)
	{
		SCT0->CTRL &= ~SCT_CTRL_HALT_L_MASK;
	}

	return true;
}

void PWMCloseChannel(PwmDev_t *pDev, int Chan)
{
	Lpc546xxPwm_t *dev = Lpc546xxPwmDev(pDev);

	if (dev == nullptr || Chan < 0 || Chan >= LPC546XX_PWM_MAX_CHAN)
	{
		return;
	}

	Lpc546xxPwmClose(dev, Chan);
}

bool PWMStart(PwmDev_t *pDev, uint32_t msDur)
{
	Lpc546xxPwm_t *dev = Lpc546xxPwmDev(pDev);

	if (dev == nullptr || msDur != 0 || PWMEnable(pDev) == false)
	{
		return false;
	}

	Lpc546xxPwmHalt();
	Lpc546xxPwmLoad(dev);
	Lpc546xxPwmRun();

	return true;
}

void PWMStop(PwmDev_t *pDev)
{
	Lpc546xxPwm_t *dev = Lpc546xxPwmDev(pDev);

	if (dev == nullptr)
	{
		return;
	}

	Lpc546xxPwmHalt();
	for (int i = 0; i < LPC546XX_PWM_MAX_CHAN; i++)
	{
		if (dev->OpenMask & (1U << i))
		{
			Lpc546xxPwmLevel(dev, i, false);
		}
	}
}

bool PWMSetFrequency(PwmDev_t *pDev, uint32_t Freq)
{
	Lpc546xxPwm_t *dev = Lpc546xxPwmDev(pDev);
	uint32_t top, freq;

	if (dev == nullptr || Lpc546xxPwmTop(Freq, &top, &freq) == false)
	{
		return false;
	}

	bool run = Lpc546xxPwmHalt();

	dev->Top = top;
	Lpc546xxPwmLoad(dev);
	if (run)
	{
		Lpc546xxPwmRun();
	}
	pDev->Freq = freq;

	return true;
}

bool PWMSetDutyCycle(PwmDev_t *pDev, int Chan, int Percent)
{
	Lpc546xxPwm_t *dev = Lpc546xxPwmDev(pDev);

	if (dev == nullptr || Chan < 0 || Chan >= LPC546XX_PWM_MAX_CHAN)
	{
		return false;
	}

	dev->Duty[Chan] = Percent < 0 ? 0 : Percent > 100 ? 100 : Percent;
	if (dev->OpenMask & (1U << Chan))
	{
		uint32_t m = Lpc546xxPwmMatch(dev, Chan);

		// Next period while running, MATCH only while halted
		SCT0->MATCHREL[Chan + 1] = m;
		if (SCT0->CTRL & SCT_CTRL_HALT_L_MASK)
		{
			SCT0->MATCH[Chan + 1] = m;
		}
	}

	return true;
}

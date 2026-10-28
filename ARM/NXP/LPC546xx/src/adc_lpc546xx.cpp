/**-------------------------------------------------------------------------
@file	adc_lpc546xx.cpp

@brief	LPC546xx ADC0 polling implementation

Sequence A of ADC0, software triggered, read by polling. The pin inputs of a
scan are converted in one pass of the sequence, each result read from its
channel data register. The temperature sensor is converted alone, in burst
mode. See adc_lpc546xx.h.

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
#include <string.h>
#include <math.h>

#include "LPC546xx.h"

#include "idelay.h"
#include "adc_lpc546xx.h"

#define LPC546XX_ADC_CLK_MAX		60000000UL	// Conversions, clock of the NXP SDK example
#define LPC546XX_ADC_CAL_CLK_MAX	30000000UL	// Calibration (NXP SDK)
#define LPC546XX_ADC_DIV_MAX		256U		// CLKDIV + 1
#define LPC546XX_ADC_TSAMP_MAX		7U			// 9.5 ADC clocks of sampling
#define LPC546XX_ADC_RESOL_12BIT	3U
#define LPC546XX_ADC_CONV_CLK		15U			// 12 bit conversion, plus TSAMP
#define LPC546XX_ADC_CAL_CLK		81U			// Calibration cycle, about 81 ADC clocks
#define LPC546XX_ADC_INIT_CLK		6U			// Initialization conversion, about 6 ADC clocks
#define LPC546XX_ADC_PWRUP_US		10U			// ADC power up to ADC_ENA
#define LPC546XX_ADC_TS_US			100U		// Temperature sensor settling, see adc_lpc546xx.h
#define LPC546XX_ADC_TEMP_BURST		9			// Burst conversions, the 9th result is used
#define LPC546XX_ADC_INSEL_TEMP		3U			// Temperature sensor on channel 0
#define LPC546XX_ADC_RESULT_MAX		4095.0f

// Supplies of the ADC. VD2_ANA also supplies the system oscillator.
#define LPC546XX_ADC_SUPPLY			(SYSCON_PDRUNCFG_PDEN_VDDA_MASK | SYSCON_PDRUNCFG_PDEN_VD2_ANA_MASK | \
									 SYSCON_PDRUNCFG_PDEN_VREFP_MASK)

// Port and pin of the ADC0 inputs 0 to 11
static const uint8_t s_Lpc546xxAdcPin[ADC_LPC546XX_NB_PIN_INPUT][2] = {
	{ 0, 10 }, { 0, 11 }, { 0, 12 }, { 0, 15 }, { 0, 16 }, { 0, 31 },
	{ 1, 0 }, { 2, 0 }, { 2, 1 }, { 3, 21 }, { 3, 22 }, { 0, 23 },
};

static AdcLpc546xx *s_pLpc546xxAdc = nullptr;

static bool Lpc546xxAdcWait(bool (*IsDone)(void), uint32_t Us)
{
	for (uint32_t n = 0; ; n++)
	{
		if (IsDone())
		{
			return true;
		}
		if (n >= Us)
		{
			return false;
		}
		usDelay(1);
	}
}

static bool Lpc546xxAdcCalDone(void)
{
	return (ADC0->CALIB & ADC_CALIB_CALIB_MASK) == 0;
}

static bool Lpc546xxAdcInitDone(void)
{
	return (ADC0->STARTUP & ADC_STARTUP_ADC_INIT_MASK) == 0;
}

// Time of Clk ADC clocks in usec, rounded up
static uint32_t Lpc546xxAdcClkUs(uint32_t Clk, uint32_t Ctrl, uint32_t SysClk)
{
	uint32_t div = ((Ctrl & ADC_CTRL_CLKDIV_MASK) >> ADC_CTRL_CLKDIV_SHIFT) + 1U;

	return (uint32_t)(((uint64_t)Clk * div * 1000000ULL + SysClk - 1U) / SysClk);
}

// Time of Count conversions in usec, rounded up
static uint32_t Lpc546xxAdcConvUs(uint32_t Count, uint32_t Ctrl, uint32_t SysClk)
{
	uint32_t tsamp = (Ctrl & ADC_CTRL_TSAMP_MASK) >> ADC_CTRL_TSAMP_SHIFT;

	return Lpc546xxAdcClkUs(Count * (LPC546XX_ADC_CONV_CLK + tsamp), Ctrl, SysClk);
}

// Smallest divider, then shortest TSAMP, whose sampling time of 2.5 + TSAMP
// ADC clocks covers Us. False when 9.5 clocks at SysClk / 256 are too short.
static bool Lpc546xxAdcSampling(uint32_t SysClk, uint32_t Us, uint32_t &Div, uint32_t &Tsamp)
{
	// Sampling time in half ADC clocks is 5 + 2 * TSAMP, 19 at most
	uint64_t num = 2ULL * Us * SysClk;
	uint64_t div = (num + 19ULL * 1000000ULL - 1U) / (19ULL * 1000000ULL);
	uint64_t min = (SysClk + LPC546XX_ADC_CLK_MAX - 1U) / LPC546XX_ADC_CLK_MAX;

	if (div < min)
	{
		div = min;
	}
	if (div > LPC546XX_ADC_DIV_MAX)
	{
		return false;
	}

	uint64_t half = (num + div * 1000000ULL - 1U) / (div * 1000000ULL);

	Div = (uint32_t)div;
	Tsamp = half <= 5U ? 0U : (uint32_t)((half - 5U + 1U) / 2U);

	return true;
}

static uint32_t Lpc546xxAdcCtrl(uint32_t Div, uint32_t Tsamp)
{
	return ADC_CTRL_CLKDIV(Div - 1U) | ADC_CTRL_RESOL(LPC546XX_ADC_RESOL_12BIT) | ADC_CTRL_TSAMP(Tsamp);
}

// ADC channel of an input
static int Lpc546xxAdcHwChan(int Input)
{
	return Input == ADC_LPC546XX_INPUT_TEMP ? 0 : Input;
}

// Read of a channel data register, it clears DATAVALID and OVERRUN
static void Lpc546xxAdcClear(int Ch)
{
	uint32_t d = ADC0->DAT[Ch];

	(void)d;
}

AdcLpc546xx::AdcLpc546xx()
{
	memset(vInput, 0, sizeof(vInput));
	memset(vbNew, 0, sizeof(vbNew));
	memset(vRaw, 0, sizeof(vRaw));
	memset(vAcqUs, 0, sizeof(vAcqUs));
	memset(vPinCfg, 0, sizeof(vPinCfg));
	memset(vbPinOut, 0, sizeof(vbPinOut));
	vResolution = 12;
	vRate = 0;
	vMode = ADC_CONV_MODE_SINGLE;
	vbInterrupt = false;
	vIntPrio = 0;
	vpRefVolt = nullptr;
	vNbRefVolt = 0;
	vRef = 0;
	vSysClk = 0;
	vCtrl = 0;
	vTempCtrl = 0;
	vPwrOwned = 0;
	vbOn = false;
}

AdcLpc546xx::~AdcLpc546xx()
{
	PowerOff();
}

bool AdcLpc546xx::Owned()
{
	return s_pLpc546xxAdc == this;
}

bool AdcLpc546xx::Init(const AdcCfg_t &Cfg, Timer *pTimer, DeviceIntrf *pIntrf)
{
	if ((s_pLpc546xxAdc && !Owned()) || pIntrf || Cfg.Mode != ADC_CONV_MODE_SINGLE || Cfg.bInterrupt ||
		Cfg.OvrSample < 0 || Cfg.OvrSample > 1 || Cfg.NbRefVolt != 1 || !Cfg.pRefVolt ||
		(Cfg.pRefVolt[0].Type != ADC_REFVOLT_TYPE_SUPPLY && Cfg.pRefVolt[0].Type != ADC_REFVOLT_TYPE_EXTERNAL) ||
		!isfinite(Cfg.pRefVolt[0].Voltage) || Cfg.pRefVolt[0].Voltage <= 0 || SystemCoreClock == 0)
	{
		return false;
	}

	PowerOff();
	s_pLpc546xxAdc = this;
	vSysClk = SystemCoreClock;

	// Erratum ADC.1: digital buffer of the ADC0 pins on until the ADC is set up
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_IOCON_MASK | SYSCON_AHBCLKCTRL_ADC0_MASK;
	for (int i = 0; i < ADC_LPC546XX_NB_PIN_INPUT; i++)
	{
		int port = s_Lpc546xxAdcPin[i][0];
		int pin = s_Lpc546xxAdcPin[i][1];

		if ((IOCON->PIO[port][pin] & IOCON_PIO_DIGIMODE_MASK) == 0)
		{
			IOCON->PIO[port][pin] |= IOCON_PIO_DIGIMODE_MASK;
		}
	}

	// Supplies that are off now, PowerOff turns them off again
	vPwrOwned = SYSCON->PDRUNCFG[0] & LPC546XX_ADC_SUPPLY;

	SYSCON->PRESETCTRLSET[0] = SYSCON_PRESETCTRL_ADC0_RST_MASK;
	SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_ADC0_RST_MASK;

	SetRefVoltage(Cfg.pRefVolt, 1);
	vRef = Cfg.pRefVolt[0].Voltage;
	vpTimer = pTimer;
	SetEvtHandler(Cfg.EvtHandler);
	vResolution = 12;
	Timing();

	if (!PowerUp())
	{
		PowerOff();

		return false;
	}

	Valid(true);

	return true;
}

// Power the ADC, then ADC_ENA and a calibration or the initialization
// conversion, as the NXP SDK. The open pins go analog after it.
bool AdcLpc546xx::PowerUp()
{
	bool temp = false;

	for (int c = 0; c < ADC_LPC546XX_MAX_CHAN; c++)
	{
		temp = temp || vInput[c] == ADC_LPC546XX_INPUT_TEMP + 1;
	}

	SYSCON->PDRUNCFGCLR[0] = LPC546XX_ADC_SUPPLY | SYSCON_PDRUNCFG_PDEN_ADC0_MASK |
							 (temp ? SYSCON_PDRUNCFG_PDEN_TS_MASK : 0U);
	usDelay(LPC546XX_ADC_PWRUP_US);

	uint32_t div = (vSysClk + LPC546XX_ADC_CAL_CLK_MAX - 1U) / LPC546XX_ADC_CAL_CLK_MAX;
	uint32_t cal = Lpc546xxAdcCtrl(div, 0);
	bool done;

	ADC0->CTRL = cal;
	ADC0->STARTUP = ADC_STARTUP_ADC_ENA_MASK;
	if ((ADC0->STARTUP & ADC_STARTUP_ADC_ENA_MASK) == 0)
	{
		ADC0->CTRL = vCtrl;

		return false;
	}

	if (ADC0->CALIB & ADC_CALIB_CALREQD_MASK)
	{
		ADC0->CALIB = ADC_CALIB_CALIB_MASK;
		done = Lpc546xxAdcWait(Lpc546xxAdcCalDone,
							   Lpc546xxAdcClkUs(2U * LPC546XX_ADC_CAL_CLK, cal, vSysClk) + 10U);
	}
	else
	{
		// Not in the write that sets ADC_ENA
		ADC0->STARTUP = ADC_STARTUP_ADC_ENA_MASK | ADC_STARTUP_ADC_INIT_MASK;
		done = Lpc546xxAdcWait(Lpc546xxAdcInitDone,
							   Lpc546xxAdcClkUs(2U * LPC546XX_ADC_INIT_CLK, cal, vSysClk) + 10U);
	}

	ADC0->CTRL = vCtrl;
	if (!done)
	{
		return false;
	}

	ADC0->INSEL = temp ? LPC546XX_ADC_INSEL_TEMP : 0U;
	vbOn = true;

	for (int c = 0; c < ADC_LPC546XX_MAX_CHAN; c++)
	{
		if (vInput[c] && vInput[c] != ADC_LPC546XX_INPUT_TEMP + 1)
		{
			PinAnalog(c);
		}
	}

	if (temp)
	{
		usDelay(LPC546XX_ADC_TS_US);
	}

	return true;
}

// Pin of a logical channel analog: function 0, no pull, digital buffer off,
// GPIO input
void AdcLpc546xx::PinAnalog(int Chan)
{
	int port = s_Lpc546xxAdcPin[vInput[Chan] - 1][0];
	int pin = s_Lpc546xxAdcPin[vInput[Chan] - 1][1];

	IOCON->PIO[port][pin] = 0;
	GPIO->DIR[port] &= ~(1UL << pin);
}

// Pin of a logical channel back as OpenChannel found it, digital buffer on
void AdcLpc546xx::PinRestore(int Chan)
{
	int port = s_Lpc546xxAdcPin[vInput[Chan] - 1][0];
	int pin = s_Lpc546xxAdcPin[vInput[Chan] - 1][1];

	IOCON->PIO[port][pin] = vPinCfg[Chan] | IOCON_PIO_DIGIMODE_MASK;
	if (vbPinOut[Chan])
	{
		GPIO->DIR[port] |= 1UL << pin;
	}
	else
	{
		GPIO->DIR[port] &= ~(1UL << pin);
	}
}

// CTRL of the pin and of the temperature sensor conversions, from the
// sampling time of the open channels
void AdcLpc546xx::Timing()
{
	uint32_t pinUs = 0;
	uint32_t tempUs = 0;
	uint32_t div = 1;
	uint32_t tsamp = 0;

	for (int c = 0; c < ADC_LPC546XX_MAX_CHAN; c++)
	{
		if (vInput[c] == ADC_LPC546XX_INPUT_TEMP + 1)
		{
			tempUs = (uint32_t)vAcqUs[c];
		}
		else if (vInput[c] && (uint32_t)vAcqUs[c] > pinUs)
		{
			pinUs = (uint32_t)vAcqUs[c];
		}
	}

	// OpenChannel checked the times
	Lpc546xxAdcSampling(vSysClk, pinUs, div, tsamp);
	vCtrl = Lpc546xxAdcCtrl(div, tsamp);
	Lpc546xxAdcSampling(vSysClk, tempUs, div, tsamp);
	vTempCtrl = Lpc546xxAdcCtrl(div, LPC546XX_ADC_TSAMP_MAX);
}

uint16_t AdcLpc546xx::Resolution(uint16_t Val)
{
	(void)Val;
	vResolution = 12;

	return vResolution;
}

bool AdcLpc546xx::OpenChannel(const AdcChanCfg_t *pCfg, int Count)
{
	if (!Owned() || !Valid() || !pCfg || Count <= 0 || Count > ADC_LPC546XX_MAX_CHAN)
	{
		return false;
	}

	for (int i = 0; i < Count; i++)
	{
		const AdcChanCfg_t &c = pCfg[i];
		int input = c.PinP.PinNo;
		uint32_t div, tsamp;

		if (c.Chan < 0 || c.Chan >= ADC_LPC546XX_MAX_CHAN || input < 0 || input > ADC_LPC546XX_INPUT_TEMP ||
			c.RefVoltIdx != 0 || c.Type != ADC_CHAN_TYPE_SINGLE_ENDED || c.Gain != 0x100 ||
			c.PinP.Conn != ADC_PIN_CONN_NONE || c.BurstMode || c.pFifoMem || c.FifoMemSize || c.AcqTime < 0 ||
			!Lpc546xxAdcSampling(vSysClk, (uint32_t)c.AcqTime, div, tsamp))
		{
			return false;
		}

		int hw = Lpc546xxAdcHwChan(input);

		for (int j = 0; j < i; j++)
		{
			if (pCfg[j].Chan == c.Chan || Lpc546xxAdcHwChan(pCfg[j].PinP.PinNo) == hw)
			{
				return false;
			}
		}

		// An ADC channel open on a logical channel this call does not set again
		for (int k = 0; k < ADC_LPC546XX_MAX_CHAN; k++)
		{
			if (vInput[k] == 0 || Lpc546xxAdcHwChan(vInput[k] - 1) != hw)
			{
				continue;
			}

			bool reset = false;

			for (int j = 0; j < Count; j++)
			{
				reset = reset || pCfg[j].Chan == k;
			}
			if (!reset)
			{
				return false;
			}
		}
	}

	// Logical channels set again release their previous input first
	for (int i = 0; i < Count; i++)
	{
		CloseChannel(pCfg[i].Chan);
	}

	bool wait = false;

	for (int i = 0; i < Count; i++)
	{
		const AdcChanCfg_t &c = pCfg[i];
		int input = c.PinP.PinNo;

		if (input == ADC_LPC546XX_INPUT_TEMP)
		{
			if (vbOn)
			{
				SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFG_PDEN_TS_MASK;
				ADC0->INSEL = LPC546XX_ADC_INSEL_TEMP;
				wait = true;
			}
		}
		else
		{
			int port = s_Lpc546xxAdcPin[input][0];
			int pin = s_Lpc546xxAdcPin[input][1];

			SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_IOCON_MASK | (SYSCON_AHBCLKCTRL_GPIO0_MASK << port);
			vPinCfg[c.Chan] = IOCON->PIO[port][pin];
			vbPinOut[c.Chan] = (GPIO->DIR[port] >> pin) & 1U;
		}

		vInput[c.Chan] = input + 1;
		vAcqUs[c.Chan] = c.AcqTime;
		vbNew[c.Chan] = false;
		if (vbOn && input != ADC_LPC546XX_INPUT_TEMP)
		{
			PinAnalog(c.Chan);
		}
	}

	Timing();

	if (wait)
	{
		usDelay(LPC546XX_ADC_TS_US);
	}

	return true;
}

void AdcLpc546xx::CloseChannel(int Chan)
{
	if (!Owned() || Chan < 0 || Chan >= ADC_LPC546XX_MAX_CHAN || !vInput[Chan])
	{
		return;
	}

	if (vInput[Chan] == ADC_LPC546XX_INPUT_TEMP + 1)
	{
		SYSCON->PDRUNCFGSET[0] = SYSCON_PDRUNCFG_PDEN_TS_MASK;
		ADC0->INSEL = 0;
	}
	else
	{
		PinRestore(Chan);
	}

	vInput[Chan] = 0;
	vAcqUs[Chan] = 0;
	vbNew[Chan] = false;
	Timing();
}

bool AdcLpc546xx::Enable()
{
	if (!Owned())
	{
		return false;
	}

	return vbOn || PowerUp();
}

void AdcLpc546xx::StopConversion()
{
	if (!Owned())
	{
		return;
	}

	ADC0->SEQ_CTRL[0] = 0;
	memset(vbNew, 0, sizeof(vbNew));
}

void AdcLpc546xx::Disable()
{
	if (!Owned() || !vbOn)
	{
		return;
	}

	StopConversion();

	// Erratum ADC.1: digital buffer of the pins on before ADC0 powers down
	for (int c = 0; c < ADC_LPC546XX_MAX_CHAN; c++)
	{
		if (vInput[c] && vInput[c] != ADC_LPC546XX_INPUT_TEMP + 1)
		{
			int port = s_Lpc546xxAdcPin[vInput[c] - 1][0];
			int pin = s_Lpc546xxAdcPin[vInput[c] - 1][1];

			IOCON->PIO[port][pin] |= IOCON_PIO_DIGIMODE_MASK;
		}
	}

	SYSCON->PDRUNCFGSET[0] = SYSCON_PDRUNCFG_PDEN_ADC0_MASK | SYSCON_PDRUNCFG_PDEN_TS_MASK;
	vbOn = false;
}

bool AdcLpc546xx::Calibrate()
{
	if (!Owned() || !Valid() || !Enable())
	{
		return false;
	}

	uint32_t div = (vSysClk + LPC546XX_ADC_CAL_CLK_MAX - 1U) / LPC546XX_ADC_CAL_CLK_MAX;
	uint32_t cal = Lpc546xxAdcCtrl(div, 0);

	ADC0->SEQ_CTRL[0] = 0;
	ADC0->CTRL = cal;
	ADC0->CALIB = ADC_CALIB_CALIB_MASK;

	bool done = Lpc546xxAdcWait(Lpc546xxAdcCalDone,
								Lpc546xxAdcClkUs(2U * LPC546XX_ADC_CAL_CLK, cal, vSysClk) + 10U);

	ADC0->CTRL = vCtrl;
	if (!done)
	{
		Valid(false);
	}

	return done;
}

void AdcLpc546xx::Reset()
{
	if (!Owned())
	{
		return;
	}

	Disable();
	SYSCON->PRESETCTRLSET[0] = SYSCON_PRESETCTRL_ADC0_RST_MASK;
	SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_ADC0_RST_MASK;
	memset(vbNew, 0, sizeof(vbNew));
	Valid(PowerUp());
}

void AdcLpc546xx::PowerOff()
{
	if (!Owned())
	{
		return;
	}

	Disable();
	for (int c = 0; c < ADC_LPC546XX_MAX_CHAN; c++)
	{
		CloseChannel(c);
	}

	// Also after a power up that failed, the pins are digital now
	SYSCON->PDRUNCFGSET[0] = SYSCON_PDRUNCFG_PDEN_ADC0_MASK | SYSCON_PDRUNCFG_PDEN_TS_MASK;
	SYSCON->PRESETCTRLSET[0] = SYSCON_PRESETCTRL_ADC0_RST_MASK;
	SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_ADC0_RST_MASK;
	SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_ADC0_MASK;

	// VD2_ANA stays on while the system oscillator runs
	uint32_t off = vPwrOwned;

	if ((SYSCON->PDRUNCFG[1] & SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK) == 0)
	{
		off &= ~SYSCON_PDRUNCFG_PDEN_VD2_ANA_MASK;
	}
	if (off)
	{
		SYSCON->PDRUNCFGSET[0] = off;
	}

	Valid(false);
	memset(vbNew, 0, sizeof(vbNew));
	vPwrOwned = 0;
	s_pLpc546xxAdc = nullptr;
}

// Sequence stopped after a conversion that did not end, Reset recovers
void AdcLpc546xx::Abort()
{
	ADC0->SEQ_CTRL[0] = 0;
	memset(vbNew, 0, sizeof(vbNew));
	Valid(false);
}

// One pass of sequence A over the pin inputs of Mask, pRaw indexed by input
bool AdcLpc546xx::ScanPins(uint32_t Mask, uint16_t *pRaw)
{
	uint32_t seq = ADC_SEQ_CTRL_CHANNELS(Mask) | ADC_SEQ_CTRL_TRIGPOL_MASK;
	uint32_t us = Lpc546xxAdcConvUs(__builtin_popcount(Mask), vCtrl, vSysClk) * 2U + 10U;
	uint32_t left = Mask;

	ADC0->SEQ_CTRL[0] = 0;
	ADC0->CTRL = vCtrl;

	// Results left from earlier conversions
	for (int ch = 0; ch < ADC_LPC546XX_NB_PIN_INPUT; ch++)
	{
		if (Mask & (1UL << ch))
		{
			Lpc546xxAdcClear(ch);
		}
	}

	// Channels and trigger set while the sequence is off, then enable, then start
	ADC0->SEQ_CTRL[0] = seq;
	ADC0->SEQ_CTRL[0] = seq | ADC_SEQ_CTRL_SEQ_ENA_MASK;
	ADC0->SEQ_CTRL[0] = seq | ADC_SEQ_CTRL_SEQ_ENA_MASK | ADC_SEQ_CTRL_START_MASK;

	for (uint32_t n = 0; ; n++)
	{
		for (int ch = 0; ch < ADC_LPC546XX_NB_PIN_INPUT; ch++)
		{
			if (left & (1UL << ch))
			{
				uint32_t d = ADC0->DAT[ch];

				if (d & ADC_DAT_DATAVALID_MASK)
				{
					pRaw[ch] = (uint16_t)((d & ADC_DAT_RESULT_MASK) >> ADC_DAT_RESULT_SHIFT);
					left &= ~(1UL << ch);
				}
			}
		}
		if (left == 0)
		{
			break;
		}
		if (n >= us)
		{
			return false;
		}
		usDelay(1);
	}

	ADC0->SEQ_CTRL[0] = 0;

	return true;
}

// Temperature sensor in burst mode on channel 0, the 9th result. A result
// not read in time is replaced by the next one, the 9th result read comes
// from the 9th conversion or a later one.
bool AdcLpc546xx::ScanTemp(uint16_t *pRaw)
{
	uint32_t seq = ADC_SEQ_CTRL_CHANNELS(1U) | ADC_SEQ_CTRL_TRIGPOL_MASK;
	uint32_t us = LPC546XX_ADC_TEMP_BURST * (Lpc546xxAdcConvUs(1, vTempCtrl, vSysClk) + 2U) + 10U;
	int cnt = 0;
	uint32_t n = 0;

	ADC0->SEQ_CTRL[0] = 0;
	ADC0->CTRL = vTempCtrl;
	Lpc546xxAdcClear(0);

	ADC0->SEQ_CTRL[0] = seq;
	ADC0->SEQ_CTRL[0] = seq | ADC_SEQ_CTRL_SEQ_ENA_MASK;
	ADC0->SEQ_CTRL[0] = seq | ADC_SEQ_CTRL_SEQ_ENA_MASK | ADC_SEQ_CTRL_BURST_MASK;

	while (cnt < LPC546XX_ADC_TEMP_BURST)
	{
		uint32_t d = ADC0->DAT[0];

		if (d & ADC_DAT_DATAVALID_MASK)
		{
			*pRaw = (uint16_t)((d & ADC_DAT_RESULT_MASK) >> ADC_DAT_RESULT_SHIFT);
			cnt++;
		}
		else if (n++ >= us)
		{
			return false;
		}
		else
		{
			usDelay(1);
		}
	}

	// Burst off, the conversion in progress ends
	ADC0->SEQ_CTRL[0] = seq | ADC_SEQ_CTRL_SEQ_ENA_MASK;
	usDelay(Lpc546xxAdcConvUs(1, vTempCtrl, vSysClk));
	ADC0->SEQ_CTRL[0] = 0;
	Lpc546xxAdcClear(0);
	ADC0->CTRL = vCtrl;

	return true;
}

bool AdcLpc546xx::Scan(const int *pChan, int Count)
{
	if (!Owned() || !Valid() || vMode != ADC_CONV_MODE_SINGLE || !Enable())
	{
		return false;
	}

	uint16_t raw[ADC_LPC546XX_INPUT_TEMP + 1];
	uint32_t mask = 0;
	bool temp = false;

	for (int i = 0; i < Count; i++)
	{
		int input = vInput[pChan[i]] - 1;

		if (input == ADC_LPC546XX_INPUT_TEMP)
		{
			temp = true;
		}
		else
		{
			mask |= 1UL << input;
		}
	}

	if ((mask && !ScanPins(mask, raw)) || (temp && !ScanTemp(&raw[ADC_LPC546XX_INPUT_TEMP])))
	{
		Abort();

		return false;
	}

	for (int i = 0; i < Count; i++)
	{
		vRaw[pChan[i]] = raw[vInput[pChan[i]] - 1];
		vbNew[pChan[i]] = true;
	}

	return true;
}

bool AdcLpc546xx::StartConversion()
{
	if (!Owned() || !Valid())
	{
		return false;
	}

	int chan[ADC_LPC546XX_MAX_CHAN];
	int n = 0;

	memset(vbNew, 0, sizeof(vbNew));
	for (int c = 0; c < ADC_LPC546XX_MAX_CHAN; c++)
	{
		if (vInput[c])
		{
			chan[n++] = c;
		}
	}

	return n > 0 && Scan(chan, n);
}

bool AdcLpc546xx::UpdateData()
{
	if (!Owned() || !Valid())
	{
		return false;
	}

	for (bool fresh : vbNew)
	{
		if (fresh)
		{
			return true;
		}
	}

	return false;
}

int AdcLpc546xx::Read(AdcData_t *pBuff, int Len)
{
	if (!pBuff || Len <= 0 || !Owned() || !Valid())
	{
		return 0;
	}

	int n = 0;

	for (int c = 0; c < ADC_LPC546XX_MAX_CHAN && n < Len; c++)
	{
		if (vbNew[c] && Read(c, &pBuff[n]))
		{
			n++;
		}
	}

	return n;
}

bool AdcLpc546xx::Read(int Chan, AdcData_t *pBuff)
{
	if (!pBuff || Chan < 0 || Chan >= ADC_LPC546XX_MAX_CHAN || !Owned() || !Valid() || !vInput[Chan])
	{
		return false;
	}

	if (!vbNew[Chan] && !Scan(&Chan, 1))
	{
		return false;
	}

	pBuff->Chan = Chan;
	pBuff->Timestamp = 0;	// Single conversion, as AdcData_t specifies
	pBuff->Data = vRaw[Chan] * vRef / LPC546XX_ADC_RESULT_MAX;
	vbNew[Chan] = false;

	return true;
}

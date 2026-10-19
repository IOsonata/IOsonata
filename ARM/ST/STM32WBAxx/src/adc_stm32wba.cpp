/**-------------------------------------------------------------------------
@file	adc_stm32wba.cpp

@brief	STM32WBA ADC4 polling implementation


@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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

#include <string.h>
#include <math.h>
#include "stm32.h"
#include "adc_stm32wba.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "idelay.h"

static AdcSTM32WBA *s_Owner;
static const uint8_t s_Pin[] = {8, 7, 6, 5, 4, 3, 2, 1, 0, 9};
static bool Wait(volatile uint32_t *pReg, uint32_t Mask, uint32_t Value)
{
	for (unsigned n = 0; n < 100000; n++)
	{
		if ((*pReg & Mask) == Value) { return true; }
	}
	return false;
}
// CR command bits are set by software and cleared by hardware. Write only
// the requested command, keeping ADVREGEN; zero does not clear ADEN.
static void Command(uint32_t Bits) { ADC4->CR = ADC_CR_ADVREGEN | Bits; }

AdcSTM32WBA::AdcSTM32WBA()
{
	memset(vInput, 0, sizeof(vInput));
	memset(vbNew, 0, sizeof(vbNew));
	memset(vRaw, 0, sizeof(vRaw));
	vResolution = 12;
	vRate = 0;
	vMode = ADC_CONV_MODE_SINGLE;
	vbInterrupt = false;
	vIntPrio = 0;
	vpRefVolt = nullptr;
	vNbRefVolt = 0;
	vRef = 0;
}
AdcSTM32WBA::~AdcSTM32WBA() { PowerOff(); }
bool AdcSTM32WBA::Owned() { return s_Owner == this; }

bool AdcSTM32WBA::Init(const AdcCfg_t &Cfg, Timer *pTimer, DeviceIntrf *pIntrf)
{
	if ((s_Owner && !Owned()) || pIntrf || Cfg.Mode != ADC_CONV_MODE_SINGLE ||
		Cfg.bInterrupt || Cfg.OvrSample > 1 || Cfg.OvrSample < 0 ||
		Cfg.NbRefVolt != 1 || !Cfg.pRefVolt ||
		(Cfg.pRefVolt[0].Type != ADC_REFVOLT_TYPE_SUPPLY &&
		 Cfg.pRefVolt[0].Type != ADC_REFVOLT_TYPE_EXTERNAL) ||
		!isfinite(Cfg.pRefVolt[0].Voltage) || Cfg.pRefVolt[0].Voltage <= 0)
	{
		return false;
	}
	uint32_t clock = SystemCoreClockGet() / 16;
	if (clock < 140000 || clock > 55000000) { return false; }
	PowerOff();
	s_Owner = this;
	RCC->AHB4ENR |= RCC_AHB4ENR_ADC4EN;
	(void)RCC->AHB4ENR;
	RCC->AHB4RSTR |= RCC_AHB4RSTR_ADC4RST;
	RCC->AHB4RSTR &= ~RCC_AHB4RSTR_ADC4RST;
	RCC->CCIPR3 &= ~RCC_CCIPR3_ADCSEL; // HCLK
	ADC4_COMMON->CCR = 7U << ADC_CCR_PRESC_Pos; // /16
	ADC4->IER = 0;
	ADC4->CFGR1 = 0; // Software trigger, fixed channel mask, right aligned
	ADC4->CFGR2 = ADC_CFGR2_LFTRIG;
	ADC4->SMPR = ADC_SMPR_SMP1; // 814.5 cycles, all channels on common 1
	ADC4->CR = ADC_CR_ADVREGEN;
	if (!Wait(&ADC4->ISR, ADC_ISR_LDORDY, ADC_ISR_LDORDY)) { PowerOff(); return false; }
	SetRefVoltage(Cfg.pRefVolt, 1);
	vRef = Cfg.pRefVolt[0].Voltage;
	vpTimer = pTimer;
	SetEvtHandler(Cfg.EvtHandler);
	Resolution(Cfg.Resolution);
	if (!Calibrate() || !Enable()) { PowerOff(); return false; }
	Valid(true);
	return true;
}

uint16_t AdcSTM32WBA::Resolution(uint16_t Val)
{
	uint16_t bits = Val >= 11 ? 12 : Val >= 9 ? 10 : Val >= 7 ? 8 : 6;
	if (!Owned()) { vResolution = bits; return bits; }
	if (ADC4->CR & ADC_CR_ADSTART) { return vResolution; }
	ADC4->CFGR1 = (ADC4->CFGR1 & ~ADC_CFGR1_RES) |
		((12U - bits) / 2U << ADC_CFGR1_RES_Pos);
	vResolution = bits;
	memset(vbNew, 0, sizeof(vbNew));
	return bits;
}

bool AdcSTM32WBA::OpenChannel(const AdcChanCfg_t *pCfg, int Count)
{
	if (!Owned() || !Valid() || !pCfg || Count <= 0 || Count > 8 ||
		(ADC4->CR & ADC_CR_ADSTART)) { return false; }
	for (int i = 0; i < Count; i++)
	{
		const AdcChanCfg_t &c = pCfg[i];
		if (c.Chan < 0 || c.Chan >= 8 || c.PinP.PinNo < 1 || c.PinP.PinNo > 10 ||
			c.RefVoltIdx != 0 || c.Type != ADC_CHAN_TYPE_SINGLE_ENDED || c.Gain != 0x100 ||
			c.PinP.Conn != ADC_PIN_CONN_NONE || c.BurstMode || c.pFifoMem || c.FifoMemSize ||
			c.AcqTime < 0 || (uint64_t)c.AcqTime * (SystemCoreClockGet() / 16) * 2 > 1629000000ULL)
		{
			return false;
		}
		for (int j = 0; j < i; j++) { if (pCfg[j].Chan == c.Chan) { return false; } }
	}
	for (int i = 0; i < Count; i++)
	{
		const AdcChanCfg_t &c = pCfg[i];
		int input = c.PinP.PinNo;
		int port = input == 10 ? IOPORTB : IOPORTA;
		// Config enables the port clock; Disable selects analog/no pulls.
		IOPinConfig(port, s_Pin[input - 1], IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
		IOPinDisable(port, s_Pin[input - 1]);
		vInput[c.Chan] = input;
		vbNew[c.Chan] = false;
	}
	return true;
}
void AdcSTM32WBA::CloseChannel(int Chan)
{
	if (Owned() && Chan >= 0 && Chan < 8)
	{
		vInput[Chan] = 0;
		vbNew[Chan] = false;
	}
}

bool AdcSTM32WBA::Enable()
{
	if (!Owned() || (ADC4->CR & (ADC_CR_ADCAL | ADC_CR_ADDIS | ADC_CR_ADSTP))) { return false; }
	if (ADC4->CR & ADC_CR_ADEN) { return true; }
	ADC4->ISR = ADC_ISR_ADRDY;
	Command(ADC_CR_ADEN);
	if (!Wait(&ADC4->ISR, ADC_ISR_ADRDY, ADC_ISR_ADRDY)) { return false; }
	// tSTAB: three ADC clocks plus a conversion (DS14736), including
	// the configured 814.5-cycle sampling time.
	uint32_t clock = SystemCoreClockGet() / 16;
	usDelay((828000000U + clock - 1) / clock);
	return true;
}
void AdcSTM32WBA::StopConversion()
{
	if (!Owned()) { return; }
	if (ADC4->CR & ADC_CR_ADSTART)
	{
		Command(ADC_CR_ADSTP);
		if (!Wait(&ADC4->CR, ADC_CR_ADSTART | ADC_CR_ADSTP, 0)) { Valid(false); }
	}
	memset(vbNew, 0, sizeof(vbNew));
}
void AdcSTM32WBA::Disable()
{
	if (!Owned()) { return; }
	StopConversion();
	if (ADC4->CR & (ADC_CR_ADSTART | ADC_CR_ADSTP)) { return; }
	if (ADC4->CR & ADC_CR_ADEN)
	{
		Command(ADC_CR_ADDIS);
		if (!Wait(&ADC4->CR, ADC_CR_ADEN | ADC_CR_ADDIS, 0)) { Valid(false); }
	}
}
bool AdcSTM32WBA::Calibrate()
{
	if (!Owned()) { return false; }
	bool enabled = ADC4->CR & ADC_CR_ADEN;
	Disable();
	if (ADC4->CR & (ADC_CR_ADEN | ADC_CR_ADSTART | ADC_CR_ADSTP | ADC_CR_ADDIS | ADC_CR_ADCAL)) { return false; }
	Command(ADC_CR_ADCAL);
	if (!Wait(&ADC4->CR, ADC_CR_ADCAL, 0)) { Valid(false); return false; }
	usDelay(20); // At least two ADC clocks at fADC >= 140 kHz
	return !enabled || Enable();
}
void AdcSTM32WBA::PowerOff()
{
	if (!Owned()) { return; }
	Disable();
	// Reset also terminates a conversion whose stop timed out.
	RCC->AHB4RSTR |= RCC_AHB4RSTR_ADC4RST;
	RCC->AHB4RSTR &= ~RCC_AHB4RSTR_ADC4RST;
	RCC->AHB4ENR &= ~RCC_AHB4ENR_ADC4EN;
	Valid(false);
	memset(vInput, 0, sizeof(vInput));
	memset(vbNew, 0, sizeof(vbNew));
	s_Owner = nullptr;
}
void AdcSTM32WBA::Reset()
{
	if (!Owned()) { return; }
	if (!Calibrate() || !Enable()) { Valid(false); }
}

bool AdcSTM32WBA::Convert(int Chan)
{
	if (!Owned() || !Valid() || vMode != ADC_CONV_MODE_SINGLE || !Enable()) { return false; }
	ADC4->CHSELR = 1U << vInput[Chan];
	ADC4->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
	Command(ADC_CR_ADSTART);
	if (!Wait(&ADC4->ISR, ADC_ISR_EOC, ADC_ISR_EOC) || (ADC4->ISR & ADC_ISR_OVR) ||
		!Wait(&ADC4->CR, ADC_CR_ADSTART, 0))
	{
		StopConversion();
		return false;
	}
	vRaw[Chan] = ADC4->DR;
	vbNew[Chan] = true;
	return true;
}
bool AdcSTM32WBA::StartConversion()
{
	if (!Owned() || !Valid()) { return false; }
	memset(vbNew, 0, sizeof(vbNew));
	bool any = false;
	for (int c = 0; c < 8; c++)
	{
		if (!vInput[c]) { continue; }
		any = true;
		if (!Convert(c)) { memset(vbNew, 0, sizeof(vbNew)); return false; }
	}
	return any;
}
bool AdcSTM32WBA::UpdateData()
{
	if (!Owned() || !Valid()) { return false; }
	for (bool fresh : vbNew) { if (fresh) { return true; } }
	return false;
}
int AdcSTM32WBA::Read(AdcData_t *pBuff, int Len)
{
	if (!pBuff || Len <= 0 || !Owned() || !Valid()) { return 0; }
	int n = 0;
	for (int c = 0; c < 8 && n < Len; c++)
	{
		if (vbNew[c] && Read(c, &pBuff[n])) { n++; }
	}
	return n;
}
bool AdcSTM32WBA::Read(int Chan, AdcData_t *pBuff)
{
	if (!pBuff || Chan < 0 || Chan >= 8 || !Owned() || !Valid() || !vInput[Chan]) { return false; }
	if (!vbNew[Chan] && !Convert(Chan)) { return false; }
	pBuff->Chan = Chan;
	pBuff->Timestamp = 0; // Single conversion, as AdcData_t specifies
	pBuff->Data = vRaw[Chan] * vRef / (1U << vResolution);
	vbNew[Chan] = false;
	return true;
}

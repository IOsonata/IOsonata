/**-------------------------------------------------------------------------
@file	adc_stm32.cpp

@brief	STM32 ADC1 polling implementation, STM32L4, L4+ and G4

Register sequences from RM0351, RM0432, RM0440 and the datasheets, see
adc_stm32.h for the supported modes and the inputs.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

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
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "adc_stm32.h"

// The L4, L4+ and G4 ADC: deep power down, oversampler, auto delay, and not
// the 16 bit ADC of the H7 (PCSEL)
#if !defined(ADC_CR_DEEPPWD) || !defined(ADC_CFGR2_OVSS) || !defined(ADC_CFGR_AUTDLY) || defined(ADC_PCSEL_PCSEL)
#error "adc_stm32.cpp: ADC of this STM32 not implemented"
#endif

// HCLK as the family system file computes it
extern "C" uint32_t SystemHFClockGet(void);

// The common registers are named after the ADC instances of the part
#if defined(ADC123_COMMON)
#define ADC_STM32_COMMON			ADC123_COMMON
#elif defined(ADC12_COMMON)
#define ADC_STM32_COMMON			ADC12_COMMON
#else
#define ADC_STM32_COMMON			ADC1_COMMON
#endif

#ifdef RCC_AHB2ENR_ADC12EN
#define ADC_STM32_RCC_EN			RCC_AHB2ENR_ADC12EN
#define ADC_STM32_RCC_RST			RCC_AHB2RSTR_ADC12RST
#else
#define ADC_STM32_RCC_EN			RCC_AHB2ENR_ADCEN
#define ADC_STM32_RCC_RST			RCC_AHB2RSTR_ADCRST
#endif

#define ADC_STM32_NO_PIN			0xFF		// Input without a pin

// G4 and L4 ADC1, the G4 names its temperature sensor bit VSENSESEL
#ifdef ADC_CCR_VSENSESEL
#define ADC_STM32_CCR_TEMP			ADC_CCR_VSENSESEL
#define ADC_STM32_CCR_VBAT			ADC_CCR_VBATSEL
#define ADC_STM32_CLK_MAX			60000000UL	// Range 1, single ADC operation

// Port and pin of the ADC1 inputs 0 to 18
static const uint8_t s_InputPin[ADC_STM32_MAX_INPUT + 1][2] = {
	{ ADC_STM32_NO_PIN, 0 }, { IOPORTA, 0 }, { IOPORTA, 1 }, { IOPORTA, 2 },
	{ IOPORTA, 3 }, { IOPORTB, 14 }, { IOPORTC, 0 }, { IOPORTC, 1 },
	{ IOPORTC, 2 }, { IOPORTC, 3 }, { IOPORTF, 0 }, { IOPORTB, 12 },
	{ IOPORTB, 1 }, { ADC_STM32_NO_PIN, 0 }, { IOPORTB, 11 }, { IOPORTB, 0 },
	{ ADC_STM32_NO_PIN, 0 }, { ADC_STM32_NO_PIN, 0 }, { ADC_STM32_NO_PIN, 0 },
};
#else
#define ADC_STM32_CCR_TEMP			ADC_CCR_TSEN
#define ADC_STM32_CCR_VBAT			ADC_CCR_VBATEN
#define ADC_STM32_CLK_MAX			80000000UL	// Range 1

// Port and pin of the ADC1 inputs 0 to 18
static const uint8_t s_InputPin[ADC_STM32_MAX_INPUT + 1][2] = {
	{ ADC_STM32_NO_PIN, 0 }, { IOPORTC, 0 }, { IOPORTC, 1 }, { IOPORTC, 2 },
	{ IOPORTC, 3 }, { IOPORTA, 0 }, { IOPORTA, 1 }, { IOPORTA, 2 },
	{ IOPORTA, 3 }, { IOPORTA, 4 }, { IOPORTA, 5 }, { IOPORTA, 6 },
	{ IOPORTA, 7 }, { IOPORTC, 4 }, { IOPORTC, 5 }, { IOPORTB, 0 },
	{ IOPORTB, 1 }, { ADC_STM32_NO_PIN, 0 }, { ADC_STM32_NO_PIN, 0 },
};
#endif

#define ADC_STM32_CLK_MIN			140000UL	// fADC range of the datasheets
#define ADC_STM32_REGUL_US			20			// tADCVREG_STUP, regulator start
#define ADC_STM32_TEMP_US			120			// tSTART, temperature sensor start
#define ADC_STM32_VREFINT_US		12			// tstart_vrefint, reference buffer start
#define ADC_STM32_BUF_US			15			// Temperature sensor buffer start from ADC enable
#define ADC_STM32_CAL_CLK			116			// tCAL, in ADC clocks

static AdcStm32 *s_Owner = nullptr;

// Sampling times of the SMP codes 0 to 7, in half ADC clocks: 2.5 to 640.5
static const uint16_t s_SmpHalfClk[8] = { 5, 13, 25, 49, 95, 185, 495, 1281 };

static bool Wait(volatile uint32_t *pReg, uint32_t Mask, uint32_t Value, uint32_t Us)
{
	for (uint32_t n = 0; ; n++)
	{
		if ((*pReg & Mask) == Value)
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

// Time of n ADC clocks in usec, rounded up
static uint32_t ClkUs(uint32_t Clk, uint32_t AdcClk)
{
	return (uint32_t)(((uint64_t)Clk * 1000000ULL + AdcClk - 1) / AdcClk);
}

// CR command bits are set by software and cleared by hardware, writing 0 has
// no effect on them. Write only the requested command, keeping ADVREGEN.
static void Command(uint32_t Bits)
{
	ADC1->CR = ADC_CR_ADVREGEN | Bits;
}

// ADC_CCR enable bit of an internal input, 0 for a pin
static uint32_t CommonBit(int Input)
{
	return Input == ADC_STM32_INPUT_VREFINT ? ADC_CCR_VREFEN :
		   Input == ADC_STM32_INPUT_TEMP ? ADC_STM32_CCR_TEMP :
		   Input == ADC_STM32_INPUT_VBAT ? ADC_STM32_CCR_VBAT : 0;
}

// GPIOx_ASCR connects the pin to the ADC on the parts that have it
static void AnalogSwitch(int Input, bool bOn)
{
#ifdef GPIO_ASCR_ASC0
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + s_InputPin[Input][0] * (GPIOB_BASE - GPIOA_BASE));
	uint32_t mask = 1UL << s_InputPin[Input][1];

	gpio->ASCR = bOn ? gpio->ASCR | mask : gpio->ASCR & ~mask;
#else
	(void)Input;
	(void)bOn;
#endif
}

AdcStm32::AdcStm32()
{
	memset(vInput, 0, sizeof(vInput));
	memset(vbNew, 0, sizeof(vbNew));
	memset(vRaw, 0, sizeof(vRaw));
	memset(vConvUs, 0, sizeof(vConvUs));
	vResolution = 12;
	vRate = 0;
	vMode = ADC_CONV_MODE_SINGLE;
	vbInterrupt = false;
	vIntPrio = 0;
	vpRefVolt = nullptr;
	vNbRefVolt = 0;
	vRef = 0;
	vAdcClk = 0;
	vOvs = 1;
}

AdcStm32::~AdcStm32()
{
	PowerOff();
}

bool AdcStm32::Owned()
{
	return s_Owner == this;
}

bool AdcStm32::Init(const AdcCfg_t &Cfg, Timer *pTimer, DeviceIntrf *pIntrf)
{
	int ovs = Cfg.OvrSample > 1 ? Cfg.OvrSample : 1;

	if ((s_Owner && !Owned()) || pIntrf || Cfg.Mode != ADC_CONV_MODE_SINGLE || Cfg.bInterrupt ||
		Cfg.OvrSample < 0 || ovs > 256 || (ovs & (ovs - 1)) || Cfg.NbRefVolt != 1 || !Cfg.pRefVolt ||
		(Cfg.pRefVolt[0].Type != ADC_REFVOLT_TYPE_SUPPLY && Cfg.pRefVolt[0].Type != ADC_REFVOLT_TYPE_EXTERNAL) ||
		!isfinite(Cfg.pRefVolt[0].Voltage) || Cfg.pRefVolt[0].Voltage <= 0)
	{
		return false;
	}

	// Synchronous clock from HCLK, CKMODE 2 is HCLK / 2, 3 is HCLK / 4
	uint32_t hclk = SystemHFClockGet();
	uint32_t ckmode = hclk / 2 > ADC_STM32_CLK_MAX ? 3 : 2;
	uint32_t clk = hclk >> (ckmode - 1);

	if (clk < ADC_STM32_CLK_MIN || clk > ADC_STM32_CLK_MAX)
	{
		return false;
	}

	PowerOff();
	s_Owner = this;
	vAdcClk = clk;
	vOvs = ovs;

	RCC->AHB2ENR |= ADC_STM32_RCC_EN;
	(void)RCC->AHB2ENR;
	RCC->AHB2RSTR |= ADC_STM32_RCC_RST;
	RCC->AHB2RSTR &= ~ADC_STM32_RCC_RST;
	// The reset leaves interrupts off, single ended inputs, oversampler off
	ADC_STM32_COMMON->CCR = ckmode << ADC_CCR_CKMODE_Pos;

	// Out of deep power down, then the regulator
	ADC1->CR = 0;
	ADC1->CR = ADC_CR_ADVREGEN;
	usDelay(ADC_STM32_REGUL_US);

	// Software trigger, one scan per start, right aligned, wait for the data
	// register to be read before the next conversion
	ADC1->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_AUTDLY;

	if (ovs > 1)
	{
		uint32_t shift = __builtin_ctz(ovs);

		ADC1->CFGR2 = ADC_CFGR2_ROVSE | ((shift - 1) << ADC_CFGR2_OVSR_Pos) | (shift << ADC_CFGR2_OVSS_Pos);
	}

	SetRefVoltage(Cfg.pRefVolt, 1);
	vRef = Cfg.pRefVolt[0].Voltage;
	vpTimer = pTimer;
	SetEvtHandler(Cfg.EvtHandler);
	Resolution(Cfg.Resolution);

	if (!Calibrate() || !Enable())
	{
		PowerOff();

		return false;
	}

	Valid(true);

	return true;
}

uint16_t AdcStm32::Resolution(uint16_t Val)
{
	uint16_t bits = Val == 0 || Val >= 11 ? 12 : Val >= 9 ? 10 : Val >= 7 ? 8 : 6;

	if (!Owned())
	{
		vResolution = bits;

		return bits;
	}

	if (ADC1->CR & ADC_CR_ADSTART)
	{
		return vResolution;
	}

	ADC1->CFGR = (ADC1->CFGR & ~ADC_CFGR_RES) | ((12U - bits) / 2U << ADC_CFGR_RES_Pos);
	vResolution = bits;
	memset(vbNew, 0, sizeof(vbNew));

	return bits;
}

bool AdcStm32::OpenChannel(const AdcChanCfg_t *pCfg, int Count)
{
	if (!Owned() || !Valid() || !pCfg || Count <= 0 || Count > ADC_STM32_MAX_CHAN ||
		(ADC1->CR & ADC_CR_ADSTART))
	{
		return false;
	}

	uint8_t smp[ADC_STM32_MAX_CHAN];

	for (int i = 0; i < Count; i++)
	{
		const AdcChanCfg_t &c = pCfg[i];
		int input = c.PinP.PinNo;

		if (c.Chan < 0 || c.Chan >= ADC_STM32_MAX_CHAN || input < 0 || input > ADC_STM32_MAX_INPUT ||
			(s_InputPin[input][0] == ADC_STM32_NO_PIN && CommonBit(input) == 0) || c.RefVoltIdx != 0 || c.Type != ADC_CHAN_TYPE_SINGLE_ENDED || c.Gain != 0x100 ||
			c.PinP.Conn != ADC_PIN_CONN_NONE || c.BurstMode || c.pFifoMem || c.FifoMemSize || c.AcqTime < 0)
		{
			return false;
		}

		for (int j = 0; j < i; j++)
		{
			if (pCfg[j].Chan == c.Chan || pCfg[j].PinP.PinNo == input)
			{
				return false;
			}
		}

		// An input open on a logical channel this call does not set again
		for (int k = 0; k < ADC_STM32_MAX_CHAN; k++)
		{
			if (vInput[k] != input + 1)
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

		// Datasheet minimum sampling time of the internal inputs
		uint32_t us = c.AcqTime;
		uint32_t min = input == ADC_STM32_INPUT_VREFINT ? 4 : input == ADC_STM32_INPUT_TEMP ? 5 :
					   input == ADC_STM32_INPUT_VBAT ? 12 : 0;

		if (us < min)
		{
			us = min;
		}

		uint64_t need = ((uint64_t)us * vAdcClk * 2U + 999999U) / 1000000U;
		int s = 0;

		while (s < 8 && s_SmpHalfClk[s] < need)
		{
			s++;
		}
		if (s >= 8)
		{
			return false;
		}
		smp[i] = s;
	}

	// Logical channels set again release their previous input first
	for (int i = 0; i < Count; i++)
	{
		int chan = pCfg[i].Chan;

		if (vInput[chan])
		{
			Release(vInput[chan] - 1);
			vInput[chan] = 0;
		}
	}

	uint32_t wait = 0;

	for (int i = 0; i < Count; i++)
	{
		const AdcChanCfg_t &c = pCfg[i];
		int input = c.PinP.PinNo;
		uint32_t bit = CommonBit(input);

		if (bit)
		{
			if (!(ADC_STM32_COMMON->CCR & bit))
			{
				ADC_STM32_COMMON->CCR |= bit;
				if (input == ADC_STM32_INPUT_TEMP)
				{
					wait = ADC_STM32_TEMP_US;
				}
				else if (input == ADC_STM32_INPUT_VREFINT && wait < ADC_STM32_VREFINT_US)
				{
					wait = ADC_STM32_VREFINT_US;
				}
			}
		}
		else
		{
			int port = s_InputPin[input][0];
			int pin = s_InputPin[input][1];

			// Config enables the port clock, Disable selects analog without pull
			IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
			IOPinDisable(port, pin);
			AnalogSwitch(input, true);
		}

		uint32_t pos = (input % 10) * 3;

		if (input < 10)
		{
			ADC1->SMPR1 = (ADC1->SMPR1 & ~(7UL << pos)) | ((uint32_t)smp[i] << pos);
		}
		else
		{
			ADC1->SMPR2 = (ADC1->SMPR2 & ~(7UL << pos)) | ((uint32_t)smp[i] << pos);
		}

		// Sampling plus 12.5 clocks for 12 bits, for each oversampled result
		vConvUs[c.Chan] = ClkUs(((uint32_t)s_SmpHalfClk[smp[i]] + 25U) * vOvs / 2U + 1U, vAdcClk);
		vInput[c.Chan] = input + 1;
		vbNew[c.Chan] = false;
	}

	if (wait)
	{
		usDelay(wait);
	}

	return true;
}

void AdcStm32::Release(int Input)
{
	uint32_t bit = CommonBit(Input);

	if (bit)
	{
		ADC_STM32_COMMON->CCR &= ~bit;
	}
	else
	{
		AnalogSwitch(Input, false);
	}
}

void AdcStm32::CloseChannel(int Chan)
{
	if (!Owned() || Chan < 0 || Chan >= ADC_STM32_MAX_CHAN || !vInput[Chan] ||
		(ADC1->CR & ADC_CR_ADSTART))
	{
		return;
	}

	Release(vInput[Chan] - 1);
	vInput[Chan] = 0;
	vbNew[Chan] = false;
}

bool AdcStm32::Enable()
{
	if (!Owned() || (ADC1->CR & (ADC_CR_ADCAL | ADC_CR_ADDIS | ADC_CR_ADSTP)))
	{
		return false;
	}

	if (ADC1->CR & ADC_CR_ADEN)
	{
		return true;
	}

	ADC1->ISR = ADC_ISR_ADRDY;
	Command(ADC_CR_ADEN);
	if (!Wait(&ADC1->ISR, ADC_ISR_ADRDY, ADC_ISR_ADRDY, ClkUs(1000, vAdcClk) + 10))
	{
		return false;
	}
	ADC1->ISR = ADC_ISR_ADRDY;

	// Buffers of the reference and of the temperature sensor start with the ADC
	if (ADC_STM32_COMMON->CCR & (ADC_CCR_VREFEN | ADC_STM32_CCR_TEMP))
	{
		usDelay(ADC_STM32_BUF_US);
	}

	return true;
}

// Stop a running scan, keeping the results already read
void AdcStm32::Stop()
{
	if (!(ADC1->CR & ADC_CR_ADSTART))
	{
		return;
	}

	uint32_t us = 0;

	for (int i = 0; i < ADC_STM32_MAX_CHAN; i++)
	{
		if (vConvUs[i] > us)
		{
			us = vConvUs[i];
		}
	}

	Command(ADC_CR_ADSTP);
	if (!Wait(&ADC1->CR, ADC_CR_ADSTART | ADC_CR_ADSTP, 0, us + ClkUs(1000, vAdcClk) + 10))
	{
		Valid(false);
	}
}

void AdcStm32::StopConversion()
{
	if (!Owned())
	{
		return;
	}

	Stop();
	memset(vbNew, 0, sizeof(vbNew));
}

void AdcStm32::Disable()
{
	if (!Owned())
	{
		return;
	}

	StopConversion();
	if (ADC1->CR & (ADC_CR_ADSTART | ADC_CR_ADSTP))
	{
		return;
	}

	if (ADC1->CR & ADC_CR_ADEN)
	{
		Command(ADC_CR_ADDIS);
		if (!Wait(&ADC1->CR, ADC_CR_ADEN | ADC_CR_ADDIS, 0, ClkUs(1000, vAdcClk) + 10))
		{
			Valid(false);
		}
	}
}

bool AdcStm32::Calibrate()
{
	if (!Owned())
	{
		return false;
	}

	bool enabled = ADC1->CR & ADC_CR_ADEN;

	Disable();
	if (ADC1->CR & (ADC_CR_ADEN | ADC_CR_ADSTART | ADC_CR_ADSTP | ADC_CR_ADDIS | ADC_CR_ADCAL))
	{
		return false;
	}

	// Single ended calibration, ADCALDIF 0
	Command(ADC_CR_ADCAL);
	if (!Wait(&ADC1->CR, ADC_CR_ADCAL, 0, ClkUs(ADC_STM32_CAL_CLK * 2, vAdcClk) + 10))
	{
		Valid(false);

		return false;
	}

	// ADEN is ignored for 4 ADC clocks after the calibration (RM0351, RM0440)
	usDelay(ClkUs(4, vAdcClk));

	return !enabled || Enable();
}

void AdcStm32::PowerOff()
{
	if (!Owned())
	{
		return;
	}

	Disable();

	// Pins off the ADC. The reset clears the internal inputs of ADC_CCR, it
	// also ends a conversion whose stop timed out and puts the ADC back in
	// deep power down.
	for (int i = 0; i < ADC_STM32_MAX_CHAN; i++)
	{
		if (vInput[i] && !CommonBit(vInput[i] - 1))
		{
			AnalogSwitch(vInput[i] - 1, false);
		}
	}

	RCC->AHB2RSTR |= ADC_STM32_RCC_RST;
	RCC->AHB2RSTR &= ~ADC_STM32_RCC_RST;
	RCC->AHB2ENR &= ~ADC_STM32_RCC_EN;

	Valid(false);
	memset(vInput, 0, sizeof(vInput));
	memset(vbNew, 0, sizeof(vbNew));
	memset(vConvUs, 0, sizeof(vConvUs));
	s_Owner = nullptr;
}

void AdcStm32::Reset()
{
	if (!Owned())
	{
		return;
	}

	if (!Calibrate() || !Enable())
	{
		Valid(false);
	}
}

// One software triggered scan of Count inputs
bool AdcStm32::Run(const uint8_t *pInput, int Count, uint16_t *pRaw, uint32_t Us)
{
	// Rank 1 to 4 in SQR1 from bit 6, 5 to 9 in SQR2, 10 to 14 in SQR3,
	// 15 and 16 in SQR4, 6 bits apart. SQR1 bits 3:0 is the length - 1.
	uint32_t sqr[4] = { (uint32_t)(Count - 1), 0, 0, 0 };

	for (int r = 1; r <= Count; r++)
	{
		int reg = r < 5 ? 0 : (r - 5) / 5 + 1;
		int pos = reg == 0 ? 6 * r : 6 * (r - 5 * reg);

		sqr[reg] |= (uint32_t)pInput[r - 1] << pos;
	}

	ADC1->SQR1 = sqr[0];
	ADC1->SQR2 = sqr[1];
	ADC1->SQR3 = sqr[2];
	ADC1->SQR4 = sqr[3];
	ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
	Command(ADC_CR_ADSTART);

	for (int r = 0; r < Count; r++)
	{
		if (!Wait(&ADC1->ISR, ADC_ISR_EOC, ADC_ISR_EOC, Us))
		{
			Stop();

			return false;
		}

		// Reading the data clears EOC and starts the next conversion (AUTDLY)
		pRaw[r] = ADC1->DR;
	}

	if ((ADC1->ISR & ADC_ISR_OVR) || !Wait(&ADC1->CR, ADC_CR_ADSTART, 0, Us))
	{
		Stop();

		return false;
	}

	return true;
}

bool AdcStm32::Scan(const int *pChan, int Count)
{
	if (!Owned() || !Valid() || vMode != ADC_CONV_MODE_SINGLE || !Enable())
	{
		return false;
	}

	uint8_t input[ADC_STM32_MAX_CHAN];
	uint16_t raw[ADC_STM32_MAX_CHAN];
	uint32_t us = 0;

	for (int i = 0; i < Count; i++)
	{
		input[i] = vInput[pChan[i]] - 1;
		if (vConvUs[pChan[i]] > us)
		{
			us = vConvUs[pChan[i]];
		}
	}
	us = us * 2 + 10;

	// The first conversion is not used (ES0250 2.8.11, ES0430 2.7.9)
	if (!Run(input, 1, raw, us) || !Run(input, Count, raw, us))
	{
		return false;
	}

	for (int i = 0; i < Count; i++)
	{
		vRaw[pChan[i]] = raw[i];
		vbNew[pChan[i]] = true;
	}

	return true;
}

bool AdcStm32::StartConversion()
{
	if (!Owned() || !Valid())
	{
		return false;
	}

	int chan[ADC_STM32_MAX_CHAN];
	int n = 0;

	memset(vbNew, 0, sizeof(vbNew));
	for (int c = 0; c < ADC_STM32_MAX_CHAN; c++)
	{
		if (vInput[c])
		{
			chan[n++] = c;
		}
	}

	return n > 0 && Scan(chan, n);
}

bool AdcStm32::UpdateData()
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

int AdcStm32::Read(AdcData_t *pBuff, int Len)
{
	if (!pBuff || Len <= 0 || !Owned() || !Valid())
	{
		return 0;
	}

	int n = 0;

	for (int c = 0; c < ADC_STM32_MAX_CHAN && n < Len; c++)
	{
		if (vbNew[c] && Read(c, &pBuff[n]))
		{
			n++;
		}
	}

	return n;
}

bool AdcStm32::Read(int Chan, AdcData_t *pBuff)
{
	if (!pBuff || Chan < 0 || Chan >= ADC_STM32_MAX_CHAN || !Owned() || !Valid() || !vInput[Chan])
	{
		return false;
	}

	if (!vbNew[Chan] && !Scan(&Chan, 1))
	{
		return false;
	}

	pBuff->Chan = Chan;
	pBuff->Timestamp = 0;	// Single conversion, as AdcData_t specifies
	pBuff->Data = vRaw[Chan] * vRef / (float)((1U << vResolution) - 1U);
	if (vInput[Chan] - 1 == ADC_STM32_INPUT_VBAT)
	{
		pBuff->Data *= 3.0f;
	}
	vbNew[Chan] = false;

	return true;
}

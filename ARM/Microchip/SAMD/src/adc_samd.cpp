/**-------------------------------------------------------------------------
@file	adc_samd.cpp

@brief	SAM D11/D21 ADC polling implementation

Software triggered conversions read by polling, one input at a time with its
own reference, gain, sampling time and mode. See adc_samd.h.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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

#include "samdxxx.h"
#include "clock_samd.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"
#include "idelay.h"
#include "analog_samd.h"
#include "adc_samd.h"

#define SAMD_ADC_CLK_MIN			30000UL		// CLK_ADC range of the datasheet
#define SAMD_ADC_CLK_MAX			2100000UL
#define SAMD_ADC_SAMPLE_NS			250U		// Minimum sampling time, 10 us for the
#define SAMD_ADC_SAMPLE_REF_NS		10000U		// temperature sensor and the bandgap,
#define SAMD_ADC_SAMPLE_DAC_NS		3000U		// 3 us for the DAC
#define SAMD_ADC_SAMPLEN_MAX		63U			// Sampling of 64 half ADC clocks
#define SAMD_ADC_PRESC_MAX			7U			// DIV512
#define SAMD_ADC_ACQ_MAX_US			100000		// AcqTime checked against this before ns
#define SAMD_ADC_SYNC_TIMEOUT		100000U
#define SAMD_ADC_INTVCC1_MIN		2.0f		// INTVCC1 only for VDDANA above 2.0 V
#define SAMD_ADC_INTVCC0_DIV		1.48f		// INTVCC0 is VDDANA / 1.48
#define SAMD_ADC_GAIN_DIV2			0xFU		// GAIN code of 1/2
#define SAMD_ADC_MUXNEG_GND			((int)ADC_INPUTCTRL_MUXNEG_GND_Val)
#define SAMD_ADC_NEG_PIN_MAX		((int)ADC_INPUTCTRL_MUXNEG_PIN7_Val)
#define SAMD_ADC_INPUT_TEMP			((int)ADC_INPUTCTRL_MUXPOS_TEMP_Val)
#define SAMD_ADC_INPUT_BANDGAP		((int)ADC_INPUTCTRL_MUXPOS_BANDGAP_Val)
#define SAMD_ADC_INPUT_COREVCC		((int)ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC_Val)
#define SAMD_ADC_INPUT_IOVCC		((int)ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val)
#define SAMD_ADC_INPUT_DAC			((int)ADC_INPUTCTRL_MUXPOS_DAC_Val)
#define SAMD_ADC_NO_PIN				0xFFU

// Port and pin of the AIN inputs, PIN_ number of the device header:
// port * 32 + pin
#ifdef PIN_PB08B_ADC_AIN2
static const uint8_t s_SamdAdcPin[ADC_EXTCHANNEL_MSB + 1] = {
	PIN_PA02B_ADC_AIN0, PIN_PA03B_ADC_AIN1, PIN_PB08B_ADC_AIN2, PIN_PB09B_ADC_AIN3,
	PIN_PA04B_ADC_AIN4, PIN_PA05B_ADC_AIN5, PIN_PA06B_ADC_AIN6, PIN_PA07B_ADC_AIN7,
	PIN_PB00B_ADC_AIN8, PIN_PB01B_ADC_AIN9, PIN_PB02B_ADC_AIN10, PIN_PB03B_ADC_AIN11,
	PIN_PB04B_ADC_AIN12, PIN_PB05B_ADC_AIN13, PIN_PB06B_ADC_AIN14, PIN_PB07B_ADC_AIN15,
	PIN_PA08B_ADC_AIN16, PIN_PA09B_ADC_AIN17, PIN_PA10B_ADC_AIN18, PIN_PA11B_ADC_AIN19,
};
#else
static const uint8_t s_SamdAdcPin[ADC_EXTCHANNEL_MSB + 1] = {
	PIN_PA02B_ADC_AIN0, PIN_PA03B_ADC_AIN1, PIN_PA04B_ADC_AIN2, PIN_PA05B_ADC_AIN3,
	PIN_PA06B_ADC_AIN4, PIN_PA07B_ADC_AIN5, PIN_PA14B_ADC_AIN6, PIN_PA15B_ADC_AIN7,
	PIN_PA10B_ADC_AIN8, PIN_PA11B_ADC_AIN9,
};
#endif

// VREFA and VREFB pins, AREFA and AREFB references. The device header names
// PA03 after the DAC (VREFP), the ADC also takes it as VREFA.
static const uint8_t s_SamdAdcRefPin[2] = { PIN_PA03B_DAC_VREFP, PIN_PA04B_ADC_VREFP };

static AdcSamd *s_pSamdAdc = nullptr;

static bool SamdAdcSync(void)
{
	for (uint32_t i = 0; i < SAMD_ADC_SYNC_TIMEOUT; i++)
	{
		if ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) == 0U)
		{
			return true;
		}
	}

	return false;
}

// Pin on its analog function while the ADC uses it, shared with the DAC
static void SamdAdcPinAnalog(uint8_t Pin)
{
	SamdAnalogPin(SAMD_ANALOG_ADC, Pin, true);
}

static void SamdAdcPinRelease(uint8_t Pin)
{
	SamdAnalogPin(SAMD_ANALOG_ADC, Pin, false);
}

// Gain in half steps: 0x002 is 1/2, 0x100 to 0x1000 are 1 to 16, 0x2000 is
// 32. 0 when not one of them.
static uint32_t SamdAdcGainHalf(uint32_t Gain)
{
	if (Gain == 0x002U)
	{
		return 1;
	}
	if ((Gain & 0xFFU) != 0 || Gain < 0x100U || Gain > 0x2000U || (Gain & (Gain - 1U)) != 0)
	{
		return 0;
	}

	return Gain >> 7;
}

// Sampling time in ns of an input: AcqTime, at least the datasheet minimum
static uint32_t SamdAdcSampleNs(int Input, int AcqUs)
{
	uint32_t ns = (uint32_t)AcqUs * 1000U;
	uint32_t min = Input == SAMD_ADC_INPUT_TEMP || Input == SAMD_ADC_INPUT_BANDGAP ? SAMD_ADC_SAMPLE_REF_NS :
				   Input == SAMD_ADC_INPUT_DAC ? SAMD_ADC_SAMPLE_DAC_NS : SAMD_ADC_SAMPLE_NS;

	return ns > min ? ns : min;
}

// Half ADC clocks of Ns at the ADC clock Clk, rounded up
static uint64_t SamdAdcHalfClk(uint32_t Ns, uint32_t Clk)
{
	return ((uint64_t)Ns * Clk * 2U + 999999999U) / 1000000000U;
}

// GAIN code of a hardware gain in half steps, 1 to 32
static uint8_t SamdAdcGainCode(uint32_t Half)
{
	return Half == 1U ? SAMD_ADC_GAIN_DIV2 : (uint8_t)(__builtin_ctz(Half) - 1);
}

AdcSamd::AdcSamd()
{
	memset(vInput, 0, sizeof(vInput));
	memset(vNeg, 0, sizeof(vNeg));
	memset(vInputCtrl, 0, sizeof(vInputCtrl));
	memset(vRefCtrl, 0, sizeof(vRefCtrl));
	memset(vSampLen, 0, sizeof(vSampLen));
	memset(vSampNs, 0, sizeof(vSampNs));
	memset(vFull, 0, sizeof(vFull));
	memset(vRaw, 0, sizeof(vRaw));
	memset(vbNew, 0, sizeof(vbNew));
	memset(vRefPin, SAMD_ADC_NO_PIN, sizeof(vRefPin));
	vResolution = 12;
	vRate = 0;
	vMode = ADC_CONV_MODE_SINGLE;
	vbInterrupt = false;
	vIntPrio = 0;
	vpRefVolt = nullptr;
	vNbRefVolt = 0;
	vClk = 0;
	vPresc = 0;
	vPrescMin = 0;
	vPrescMax = 0;
	vOvs = 0;
	vbFirst = true;
}

AdcSamd::~AdcSamd()
{
	PowerOff();
}

bool AdcSamd::Owned()
{
	return s_pSamdAdc == this;
}

bool AdcSamd::Init(const AdcCfg_t &Cfg, Timer *pTimer, DeviceIntrf *pIntrf)
{
	int ovs = Cfg.OvrSample > 1 ? Cfg.OvrSample : 1;

	if ((s_pSamdAdc && !Owned()) || pIntrf || Cfg.Mode != ADC_CONV_MODE_SINGLE || Cfg.bInterrupt ||
		Cfg.OvrSample < 0 || ovs > 1024 || (ovs & (ovs - 1)) || Cfg.NbRefVolt < 1 ||
		Cfg.NbRefVolt > ADC_SAMD_MAX_REF || !Cfg.pRefVolt)
	{
		return false;
	}

	for (int i = 0; i < Cfg.NbRefVolt; i++)
	{
		const AdcRefVolt_t &r = Cfg.pRefVolt[i];

		if (!isfinite(r.Voltage) || r.Voltage <= 0 ||
			(r.Type != ADC_REFVOLT_TYPE_INTERNAL && r.Type != ADC_REFVOLT_TYPE_SUPPLY &&
			 r.Type != ADC_REFVOLT_TYPE_EXTERNAL) ||
			(r.Type == ADC_REFVOLT_TYPE_EXTERNAL && (r.Pin < 0 || r.Pin > 1)))
		{
			return false;
		}
	}

	// Prescalers that keep the ADC clock in its range
	uint32_t clk = SystemPeriphClockGet(SAMD_GCLK_MAIN);
	uint8_t pmin = 0;
	uint8_t pmax;

	while (pmin < SAMD_ADC_PRESC_MAX && clk / (4UL << pmin) > SAMD_ADC_CLK_MAX)
	{
		pmin++;
	}
	if (clk / (4UL << pmin) > SAMD_ADC_CLK_MAX || clk / (4UL << pmin) < SAMD_ADC_CLK_MIN)
	{
		return false;
	}
	pmax = pmin;
	while (pmax < SAMD_ADC_PRESC_MAX && clk / (4UL << (pmax + 1)) >= SAMD_ADC_CLK_MIN)
	{
		pmax++;
	}

	PowerOff();
	s_pSamdAdc = this;
	vClk = clk;
	vPrescMin = pmin;
	vPrescMax = pmax;
	vOvs = (uint8_t)__builtin_ctz((uint32_t)ovs);

	SetRefVoltage(Cfg.pRefVolt, Cfg.NbRefVolt);
	vpTimer = pTimer;
	SetEvtHandler(Cfg.EvtHandler);
	Resolution((uint16_t)Cfg.Resolution);

	// Reference pins on their analog function while the driver owns the ADC
	for (int i = 0; i < Cfg.NbRefVolt; i++)
	{
		if (Cfg.pRefVolt[i].Type == ADC_REFVOLT_TYPE_EXTERNAL)
		{
			uint8_t pin = s_SamdAdcRefPin[Cfg.pRefVolt[i].Pin];

			vRefPin[Cfg.pRefVolt[i].Pin] = pin;
			SamdAdcPinAnalog(pin);
		}
	}

	Timing();

	if (!Setup())
	{
		PowerOff();

		return false;
	}

	Valid(true);

	return true;
}

// Clocks, software reset, factory calibration, averaging, then enable
bool AdcSamd::Setup()
{
	PM_REGS->PM_APBCMASK |= PM_APBCMASK_ADC_Msk;
	if (!SamdGclkChannelEnable(ADC_GCLK_ID, SAMD_GCLK_MAIN))
	{
		return false;
	}

	ADC_REGS->ADC_CTRLA = ADC_CTRLA_SWRST_Msk;
	if (!SamdAdcSync() || (ADC_REGS->ADC_CTRLA & ADC_CTRLA_SWRST_Msk))
	{
		return false;
	}

	LoadCalib();

	// Averaging: the 16 bit mode, ADJRES brings the sum back to 12 bits
	if (vOvs)
	{
		ADC_REGS->ADC_AVGCTRL = (uint8_t)(ADC_AVGCTRL_SAMPLENUM(vOvs) |
										  ADC_AVGCTRL_ADJRES(vOvs < 4 ? vOvs : 4U));
	}

	// ADC clock in range before the enable
	ADC_REGS->ADC_CTRLB = Ctrlb(false);
	if (!SamdAdcSync())
	{
		return false;
	}

	vbFirst = true;

	return Enable();
}

// CTRLB of a conversion: prescaler, resolution or averaging, mode
uint16_t AdcSamd::Ctrlb(bool bDiff)
{
	return (uint16_t)(ADC_CTRLB_PRESCALER(vPresc) | (bDiff ? ADC_CTRLB_DIFFMODE_Msk : 0U) |
					  (vOvs ? ADC_CTRLB_RESSEL_16BIT :
					   vResolution == 10 ? ADC_CTRLB_RESSEL_10BIT :
					   vResolution == 8 ? ADC_CTRLB_RESSEL_8BIT : ADC_CTRLB_RESSEL_12BIT));
}

// Factory linearity and bias calibration of the NVM software calibration row
void AdcSamd::LoadCalib()
{
	const volatile uint32_t *otp4 = (const volatile uint32_t *)OTP4_ADDR;
	uint32_t lin = ((otp4[0] & FUSES_OTP4_WORD_0_ADC_LINEARITY_0_Msk) >> FUSES_OTP4_WORD_0_ADC_LINEARITY_0_Pos) |
				   (((otp4[1] & FUSES_OTP4_WORD_1_ADC_LINEARITY_1_Msk) >> FUSES_OTP4_WORD_1_ADC_LINEARITY_1_Pos) << 5);
	uint32_t bias = (otp4[1] & FUSES_OTP4_WORD_1_ADC_BIASCAL_Msk) >> FUSES_OTP4_WORD_1_ADC_BIASCAL_Pos;

	ADC_REGS->ADC_CALIB = (uint16_t)(ADC_CALIB_LINEARITY_CAL(lin) | ADC_CALIB_BIAS_CAL(bias));
}

// Prescaler from the longest sampling time of the open channels, then
// SAMPLEN of each logical channel
void AdcSamd::Timing()
{
	uint32_t ns = 0;

	for (int c = 0; c < ADC_SAMD_MAX_CHAN; c++)
	{
		if (vInput[c] && vSampNs[c] > ns)
		{
			ns = vSampNs[c];
		}
	}

	// OpenChannel checked that the slowest ADC clock covers the times
	vPresc = vPrescMin;
	while (vPresc < vPrescMax && SamdAdcHalfClk(ns, vClk / (4UL << vPresc)) > SAMD_ADC_SAMPLEN_MAX + 1U)
	{
		vPresc++;
	}

	uint32_t adc = vClk / (4UL << vPresc);

	for (int c = 0; c < ADC_SAMD_MAX_CHAN; c++)
	{
		uint64_t half = SamdAdcHalfClk(vSampNs[c], adc);

		vSampLen[c] = half <= 1U ? 0U : (uint8_t)(half - 1U);
	}
}

uint16_t AdcSamd::Resolution(uint16_t Val)
{
	// The averaged result is 12 bits
	vResolution = vOvs || Val == 0 || Val >= 11 ? 12 : Val >= 9 ? 10 : 8;
	memset(vbNew, 0, sizeof(vbNew));

	return vResolution;
}

bool AdcSamd::RefPin(int Input)
{
	if (Input > ADC_EXTCHANNEL_MSB)
	{
		return false;
	}

	return s_SamdAdcPin[Input] == vRefPin[0] || s_SamdAdcPin[Input] == vRefPin[1];
}

bool AdcSamd::OpenChannel(const AdcChanCfg_t *pCfg, int Count)
{
	if (!Owned() || !Valid() || !pCfg || Count <= 0 || Count > ADC_SAMD_MAX_CHAN)
	{
		return false;
	}

	uint32_t slow = vClk / (4UL << vPrescMax);

	for (int i = 0; i < Count; i++)
	{
		const AdcChanCfg_t &c = pCfg[i];
		int input = c.PinP.PinNo;
		bool diff = c.Type == ADC_CHAN_TYPE_DIFFERENTIAL;
		bool pin = input >= 0 && input <= ADC_EXTCHANNEL_MSB;

		if (c.Chan < 0 || c.Chan >= ADC_SAMD_MAX_CHAN ||
			(!pin && (input < SAMD_ADC_INPUT_TEMP || input > SAMD_ADC_INPUT_DAC)) ||
			c.RefVoltIdx < 0 || c.RefVoltIdx >= vNbRefVolt ||
			(c.Type != ADC_CHAN_TYPE_SINGLE_ENDED && !diff) ||
			(diff && (!pin || c.PinN.PinNo < 0 || c.PinN.PinNo > SAMD_ADC_NEG_PIN_MAX ||
					  c.PinN.PinNo == input || c.PinN.Conn != ADC_PIN_CONN_NONE || RefPin(c.PinN.PinNo))) ||
			c.PinP.Conn != ADC_PIN_CONN_NONE || c.BurstMode || c.pFifoMem || c.FifoMemSize ||
			c.AcqTime < 0 || c.AcqTime > SAMD_ADC_ACQ_MAX_US ||
			SamdAdcHalfClk(SamdAdcSampleNs(input, c.AcqTime), slow) > SAMD_ADC_SAMPLEN_MAX + 1U || RefPin(input) ||
			HwGain(c) == 0)
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
		for (int k = 0; k < ADC_SAMD_MAX_CHAN; k++)
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
	}

	// Logical channels set again release their previous input first
	for (int i = 0; i < Count; i++)
	{
		CloseChannel(pCfg[i].Chan);
	}

	for (int i = 0; i < Count; i++)
	{
		const AdcChanCfg_t &c = pCfg[i];
		const AdcRefVolt_t &r = vpRefVolt[c.RefVoltIdx];
		int input = c.PinP.PinNo;
		bool diff = c.Type == ADC_CHAN_TYPE_DIFFERENTIAL;
		uint32_t half = HwGain(c);
		float ref = r.Voltage;
		uint8_t sel;

		switch (r.Type)
		{
			case ADC_REFVOLT_TYPE_INTERNAL:
				sel = ADC_REFCTRL_REFSEL_INT1V_Val;
				break;
			case ADC_REFVOLT_TYPE_EXTERNAL:
				sel = r.Pin == 0 ? ADC_REFCTRL_REFSEL_AREFA_Val : ADC_REFCTRL_REFSEL_AREFB_Val;
				break;
			default:
				if (r.Voltage > SAMD_ADC_INTVCC1_MIN)
				{
					sel = ADC_REFCTRL_REFSEL_INTVCC1_Val;
					ref = r.Voltage / 2.0f;
				}
				else
				{
					sel = ADC_REFCTRL_REFSEL_INTVCC0_Val;
					ref = r.Voltage / SAMD_ADC_INTVCC0_DIV;
				}
				break;
		}

		if (input <= ADC_EXTCHANNEL_MSB)
		{
			SamdAdcPinAnalog(s_SamdAdcPin[input]);
		}
		if (diff)
		{
			SamdAdcPinAnalog(s_SamdAdcPin[c.PinN.PinNo]);
		}

		// Full scale in volt, the scaled supplies are read as the supply
		vFull[c.Chan] = ref * 2.0f / (float)half;
		if (input == SAMD_ADC_INPUT_COREVCC || input == SAMD_ADC_INPUT_IOVCC)
		{
			vFull[c.Chan] *= 4.0f;
		}

		vInput[c.Chan] = (uint8_t)(input + 1);
		vNeg[c.Chan] = diff ? (uint8_t)c.PinN.PinNo : SAMD_ADC_MUXNEG_GND;
		vInputCtrl[c.Chan] = ADC_INPUTCTRL_MUXPOS(input) | ADC_INPUTCTRL_MUXNEG(vNeg[c.Chan]) |
							 ADC_INPUTCTRL_GAIN(SamdAdcGainCode(half));
		vRefCtrl[c.Chan] = sel;
		vSampNs[c.Chan] = SamdAdcSampleNs(input, c.AcqTime);
		vbNew[c.Chan] = false;
	}

	Vref();
	Timing();

	return true;
}

// SYSCTRL VREF outputs of the open channels: TSEN for the temperature
// sensor, BGOUTEN for the bandgap input and for the INT1V reference (as ASF)
void AdcSamd::Vref()
{
	uint32_t need = 0;

	for (int c = 0; c < ADC_SAMD_MAX_CHAN; c++)
	{
		if (vInput[c] == SAMD_ADC_INPUT_TEMP + 1)
		{
			need |= SYSCTRL_VREF_TSEN_Msk;
		}
		if (vInput[c] && (vInput[c] == SAMD_ADC_INPUT_BANDGAP + 1 || vRefCtrl[c] == ADC_REFCTRL_REFSEL_INT1V_Val))
		{
			need |= SYSCTRL_VREF_BGOUTEN_Msk;
		}
	}

	SamdAnalogVref(SAMD_ANALOG_ADC, need);
}

// Hardware gain in half steps for a channel, 0 when the reference cannot
// give that gain. On the supply the ADC uses VDDANA / 2 (or / 1.48): the
// gain asked is twice the gain stage.
uint32_t AdcSamd::HwGain(const AdcChanCfg_t &Cfg)
{
	uint32_t half = SamdAdcGainHalf(Cfg.Gain);

	if (half == 0 || Cfg.RefVoltIdx < 0 || Cfg.RefVoltIdx >= vNbRefVolt)
	{
		return 0;
	}
	if (vpRefVolt[Cfg.RefVoltIdx].Type == ADC_REFVOLT_TYPE_SUPPLY)
	{
		half >>= 1;
	}

	return half <= 32U ? half : 0;
}

bool AdcSamd::Used(int Input, int Except)
{
	for (int k = 0; k < ADC_SAMD_MAX_CHAN; k++)
	{
		if (k != Except && vInput[k] &&
			(vInput[k] - 1 == Input || (vNeg[k] != SAMD_ADC_MUXNEG_GND && vNeg[k] == Input)))
		{
			return true;
		}
	}

	return false;
}

void AdcSamd::CloseChannel(int Chan)
{
	if (!Owned() || Chan < 0 || Chan >= ADC_SAMD_MAX_CHAN || !vInput[Chan])
	{
		return;
	}

	int input = vInput[Chan] - 1;

	if (input <= ADC_EXTCHANNEL_MSB && !Used(input, Chan))
	{
		SamdAdcPinRelease(s_SamdAdcPin[input]);
	}
	if (vNeg[Chan] != SAMD_ADC_MUXNEG_GND && !Used(vNeg[Chan], Chan))
	{
		SamdAdcPinRelease(s_SamdAdcPin[vNeg[Chan]]);
	}

	vInput[Chan] = 0;
	vNeg[Chan] = 0;
	vSampNs[Chan] = 0;
	vbNew[Chan] = false;
	Vref();
	Timing();
}

bool AdcSamd::Enable()
{
	if (!Owned())
	{
		return false;
	}

	if (ADC_REGS->ADC_CTRLA & ADC_CTRLA_ENABLE_Msk)
	{
		return true;
	}

	ADC_REGS->ADC_CTRLA = ADC_CTRLA_ENABLE_Msk;
	vbFirst = true;

	return SamdAdcSync();
}

void AdcSamd::StopConversion()
{
	if (!Owned())
	{
		return;
	}

	memset(vbNew, 0, sizeof(vbNew));
}

void AdcSamd::Disable()
{
	if (!Owned() || !(PM_REGS->PM_APBCMASK & PM_APBCMASK_ADC_Msk))
	{
		return;
	}

	// A write during a synchronization that does not end would stall the bus
	StopConversion();
	if (!SamdAdcSync())
	{
		Valid(false);

		return;
	}
	ADC_REGS->ADC_CTRLA = 0;
	if (!SamdAdcSync())
	{
		Valid(false);
	}
}

bool AdcSamd::Calibrate()
{
	if (!Owned() || !Valid())
	{
		return false;
	}

	bool enabled = ADC_REGS->ADC_CTRLA & ADC_CTRLA_ENABLE_Msk;

	Disable();
	LoadCalib();

	return !enabled || Enable();
}

void AdcSamd::Reset()
{
	if (!Owned())
	{
		return;
	}

	memset(vbNew, 0, sizeof(vbNew));
	Valid(Setup());
}

void AdcSamd::PowerOff()
{
	if (!Owned())
	{
		return;
	}

	for (int c = 0; c < ADC_SAMD_MAX_CHAN; c++)
	{
		CloseChannel(c);
	}
	for (int i = 0; i < 2; i++)
	{
		if (vRefPin[i] != SAMD_ADC_NO_PIN)
		{
			SamdAdcPinRelease(vRefPin[i]);
			vRefPin[i] = SAMD_ADC_NO_PIN;
		}
	}

	if ((PM_REGS->PM_APBCMASK & PM_APBCMASK_ADC_Msk) && SamdAdcSync())
	{
		ADC_REGS->ADC_CTRLA = ADC_CTRLA_SWRST_Msk;
		(void)SamdAdcSync();
	}
	(void)SamdGclkChannelDisable(ADC_GCLK_ID);
	PM_REGS->PM_APBCMASK &= ~PM_APBCMASK_ADC_Msk;

	Valid(false);
	memset(vbNew, 0, sizeof(vbNew));
	s_pSamdAdc = nullptr;
}

// Conversion of one logical channel. The first conversion after an enable
// or a reference change is not used.
bool AdcSamd::Convert(int Chan, int16_t *pRaw)
{
	bool diff = vNeg[Chan] != SAMD_ADC_MUXNEG_GND;
	uint16_t ctrlb = Ctrlb(diff);

	if (ADC_REGS->ADC_REFCTRL != vRefCtrl[Chan])
	{
		ADC_REGS->ADC_REFCTRL = vRefCtrl[Chan];
		vbFirst = true;
	}
	if (ADC_REGS->ADC_CTRLB != ctrlb)
	{
		ADC_REGS->ADC_CTRLB = ctrlb;
	}
	ADC_REGS->ADC_SAMPCTRL = vSampLen[Chan];
	if (!SamdAdcSync())
	{
		return false;
	}
	ADC_REGS->ADC_INPUTCTRL = vInputCtrl[Chan];
	if (!SamdAdcSync())
	{
		return false;
	}

	// Conversion time: sampling, then up to 10 ADC clocks for 12 bits and the
	// gain stage, for each averaged sample; twice that plus 10 us
	uint32_t adc = vClk / (4UL << vPresc);
	uint32_t clk = ((uint32_t)vSampLen[Chan] + 1U + 20U) << vOvs;
	uint32_t us = (uint32_t)(((uint64_t)clk * 1000000ULL + adc - 1U) / adc) + 10U;

	for (int n = vbFirst ? 2 : 1; n > 0; n--)
	{
		ADC_REGS->ADC_INTFLAG = ADC_INTFLAG_RESRDY_Msk | ADC_INTFLAG_OVERRUN_Msk;
		ADC_REGS->ADC_SWTRIG = ADC_SWTRIG_START_Msk;
		if (!SamdAdcSync())
		{
			return false;
		}

		uint32_t t = 0;

		while ((ADC_REGS->ADC_INTFLAG & ADC_INTFLAG_RESRDY_Msk) == 0U)
		{
			if (t++ >= us)
			{
				return false;
			}
			usDelay(1);
		}

		// The read clears RESRDY. Right adjusted, the differential result
		// is sign extended from its top bit.
		uint32_t r = ADC_REGS->ADC_RESULT;
		uint32_t bits = vOvs ? 12U : vResolution;

		*pRaw = diff ? (int16_t)((int32_t)(r << (32U - bits)) >> (32U - bits)) : (int16_t)r;
	}
	vbFirst = false;

	return true;
}

// Conversion stopped by a flush after a conversion that did not end, Reset
// recovers
void AdcSamd::Abort()
{
	if (SamdAdcSync())
	{
		ADC_REGS->ADC_SWTRIG = ADC_SWTRIG_FLUSH_Msk;
		(void)SamdAdcSync();
	}
	memset(vbNew, 0, sizeof(vbNew));
	Valid(false);
}

bool AdcSamd::Scan(const int *pChan, int Count)
{
	if (!Owned() || !Valid() || vMode != ADC_CONV_MODE_SINGLE || !Enable())
	{
		return false;
	}

	int16_t raw[ADC_SAMD_MAX_CHAN];

	for (int i = 0; i < Count; i++)
	{
		if (!Convert(pChan[i], &raw[i]))
		{
			Abort();

			return false;
		}
	}

	for (int i = 0; i < Count; i++)
	{
		vRaw[pChan[i]] = raw[i];
		vbNew[pChan[i]] = true;
	}

	return true;
}

bool AdcSamd::StartConversion()
{
	if (!Owned() || !Valid())
	{
		return false;
	}

	int chan[ADC_SAMD_MAX_CHAN];
	int n = 0;

	memset(vbNew, 0, sizeof(vbNew));
	for (int c = 0; c < ADC_SAMD_MAX_CHAN; c++)
	{
		if (vInput[c])
		{
			chan[n++] = c;
		}
	}

	return n > 0 && Scan(chan, n);
}

bool AdcSamd::UpdateData()
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

int AdcSamd::Read(AdcData_t *pBuff, int Len)
{
	if (!pBuff || Len <= 0 || !Owned() || !Valid())
	{
		return 0;
	}

	int n = 0;

	for (int c = 0; c < ADC_SAMD_MAX_CHAN && n < Len; c++)
	{
		if (vbNew[c] && Read(c, &pBuff[n]))
		{
			n++;
		}
	}

	return n;
}

bool AdcSamd::Read(int Chan, AdcData_t *pBuff)
{
	if (!pBuff || Chan < 0 || Chan >= ADC_SAMD_MAX_CHAN || !Owned() || !Valid() || !vInput[Chan])
	{
		return false;
	}

	if (!vbNew[Chan] && !Scan(&Chan, 1))
	{
		return false;
	}

	uint32_t bits = vOvs ? 12U : vResolution;

	// Differential: -full scale to +full scale over -2^(bits-1) to 2^(bits-1) - 1
	if (vNeg[Chan] != SAMD_ADC_MUXNEG_GND)
	{
		bits--;
	}

	pBuff->Chan = Chan;
	pBuff->Timestamp = 0;	// Single conversion, as AdcData_t specifies
	pBuff->Data = vRaw[Chan] * vFull[Chan] / (float)((1UL << bits) - 1U);
	vbNew[Chan] = false;

	return true;
}

/**-------------------------------------------------------------------------
@file	temp_stm32wba.cpp

@brief	STM32WBA internal temperature sensor over the IOsonata ADC


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

#include "stm32.h"
#include "adc_stm32wba.h"
#include "temp_stm32wba.h"
#include <string.h>
#include <math.h>

// Factory calibration locations from the STM32WBA LL ADC definitions.
#if defined(STM32WBA62xx) || defined(STM32WBA63xx) || defined(STM32WBA64xx) || defined(STM32WBA65xx) || defined(STM32WBA6Mxx)
#define ADC_WBA_CAL_BASE	0x0BFA0700UL
#elif defined(STM32WBA20xx) || defined(STM32WBA23xx) || defined(STM32WBA25xx)
#define ADC_WBA_CAL_BASE	0x0BF8D700UL
#else
#define ADC_WBA_CAL_BASE	0x0BF90700UL
#endif

// VREFINT calibration is at an odd address. Read bytes, not an unaligned
// halfword, so this also works with UNALIGN_TRP enabled.
static uint16_t AdcWbaCalRead(uint32_t Offset)
{
	const volatile uint8_t *p = (const volatile uint8_t *)(ADC_WBA_CAL_BASE + Offset);

	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

bool TempSTM32WBA::Init(const TempSensorCfg_t &Cfg, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (pIntrf || Cfg.OpMode != SENSOR_OPMODE_SINGLE || Cfg.Inter || Cfg.TempOvrs > 1)
	{
		return false;
	}
	vpTimer = pTimer;
	vOpMode = SENSOR_OPMODE_SINGLE;
	vDataRdyHandler = Cfg.DataRdyCB;
	memset(&vData, 0, sizeof(vData));
	Valid(true);

	return true;
}

bool TempSTM32WBA::Enable()
{
	return Valid();
}

void TempSTM32WBA::Disable()
{
}

void TempSTM32WBA::PowerOff()
{
	Valid(false);
}

void TempSTM32WBA::Reset()
{
	memset(&vData, 0, sizeof(vData));
}

bool TempSTM32WBA::StartSampling()
{
	return UpdateData();
}

bool TempSTM32WBA::UpdateData()
{
	static AdcSTM32WBA adc;
	static const AdcRefVolt_t ref = { ADC_REFVOLT_TYPE_SUPPLY, 3.0f, -1 };
	static const AdcCfg_t cfg = {
		ADC_CONV_MODE_SINGLE, &ref, 1, 0, 12, 0, 0, false, 0, nullptr
	};
	static const AdcChanCfg_t channels[2] = {
		{ 0, 0, ADC_CHAN_TYPE_SINGLE_ENDED, 0x100, 0, false,
			{ ADC_STM32WBA_INPUT_VREFINT, ADC_PIN_CONN_NONE }, { -1, ADC_PIN_CONN_NONE }, 0, nullptr },
		{ 1, 0, ADC_CHAN_TYPE_SINGLE_ENDED, 0x100, 0, false,
			{ ADC_STM32WBA_INPUT_TEMP, ADC_PIN_CONN_NONE }, { -1, ADC_PIN_CONN_NONE }, 0, nullptr }
	};
	AdcSTM32WBA *owner = AdcSTM32WBA::Owner();
	AdcData_t data[2];
	uint8_t input[8];
	uint16_t raw[8];
	bool fresh[8];
	uint32_t cfgr;
	uint32_t selection;
	uint32_t common;
	uint16_t resolution;
	uint16_t calRef = AdcWbaCalRead(0xA5);
	uint16_t cal1 = AdcWbaCalRead(0x10);
	uint16_t cal2 = AdcWbaCalRead(0x42);
	float value;
	int32_t normalized;
	int32_t temperature;
	bool enabled;
	bool ok;

	if (!Valid() || __get_IPSR() != 0 || calRef == 0 || calRef >= 4095 ||
		cal1 == 0 || cal1 >= 4095 || cal2 == 0 || cal2 >= 4095)
	{
		return false;
	}
	if (owner == nullptr)
	{
		if (!adc.Init(cfg))
		{
			return false;
		}
		owner = &adc;
	}
	else if (!owner->Valid() ||
		(ADC4->CR & (ADC_CR_ADSTART | ADC_CR_ADSTP | ADC_CR_ADCAL | ADC_CR_ADDIS)))
	{
		return false;
	}

	// All ADC4 access remains in the IOsonata port. Preserve its owner's
	// configuration and buffered samples while using two logical channels.
	cfgr = ADC4->CFGR1;
	selection = ADC4->CHSELR;
	common = ADC4_COMMON->CCR;
	enabled = (ADC4->CR & ADC_CR_ADEN) != 0;
	resolution = owner->Resolution();
	memcpy(input, owner->vInput, sizeof(input));
	memcpy(raw, owner->vRaw, sizeof(raw));
	memcpy(fresh, owner->vbNew, sizeof(fresh));
	owner->Resolution(12);
	ok = owner->OpenChannel(channels, 2) && owner->Read(0, &data[0]) && owner->Read(1, &data[1]);
	if (ADC4->CR & (ADC_CR_ADSTART | ADC_CR_ADSTP))
	{
		owner->Valid(false);
		ok = false;
	}
	else
	{
		if (!enabled)
		{
			owner->Disable();
			ok = ok && owner->Valid();
		}
		ADC4->CFGR1 = cfgr;
		ADC4->CHSELR = selection;
		ADC4_COMMON->CCR = common;
	}
	owner->vResolution = resolution;
	memcpy(owner->vInput, input, sizeof(input));
	memcpy(owner->vRaw, raw, sizeof(raw));
	memcpy(owner->vbNew, fresh, sizeof(fresh));
	if (owner == &adc)
	{
		adc.PowerOff();
	}
	if (!ok || data[0].Data <= 0 || data[1].Data <= 0)
	{
		return false;
	}

	// Ratio removes the configured reference voltage. The two factory
	// points are 30 C and 130 C at 3.0 V; VREFINT compensates supply drift.
	value = data[1].Data / data[0].Data * calRef;
	if (!isfinite(value) || value < 1.0f || value >= 4095.0f)
	{
		return false;
	}
	normalized = (int32_t)(value + 0.5f);
	if (cal1 != cal2)
	{
		temperature = (normalized - cal1) * 10000 / ((int32_t)cal2 - cal1) + 3000;
	}
	else
	{
		// Cube fallback: 760 mV at 30 C and positive 2.5 mV/C slope.
		temperature = (normalized * 3000 / 4095 - 760) * 40 + 3000;
	}
	if (temperature < -4000 || temperature > 13000)
	{
		return false;
	}
	vData.Temperature = temperature;
	vData.Timestamp = vpTimer ? vpTimer->uSecond() : 0;
	if (vDataRdyHandler)
	{
		vDataRdyHandler(this, &vData);
	}

	return true;
}

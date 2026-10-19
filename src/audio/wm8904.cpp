/**-------------------------------------------------------------------------
@file	wm8904.cpp

@brief	WM8904 digital microphone control


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

#include "audio/wm8904.h"
#include "idelay.h"

// WM8904 Rev 4.1 register addresses. Digital microphone operation follows
// sections Digital Microphone Interface, Clocking and Control Interface.
#define WM8904_RESET 0x00
#define WM8904_BIAS 0x04
#define WM8904_VMID 0x05
#define WM8904_MIC_BIAS 0x06
#define WM8904_ADC_OSR 0x0A
#define WM8904_INPUT_POWER 0x0C
#define WM8904_CONVERTER_POWER 0x12
#define WM8904_CLOCK_RATE 0x15
#define WM8904_CLOCK_ENABLE 0x16
#define WM8904_AUDIO_SOURCE 0x18
#define WM8904_AUDIO_FORMAT 0x19
#define WM8904_ADC_LEFT_VOLUME 0x24
#define WM8904_ADC_RIGHT_VOLUME 0x25
#define WM8904_DMIC 0x27
#define WM8904_GPIO1 0x79

typedef struct __Wm8904_Sample_Rate
{
	uint32_t Freq;
	uint16_t Value;
} Wm8904SampleRate_t;

static const Wm8904SampleRate_t s_Rates[] = {
	{ 8000, 0 },
	{ 11025, 1 },
	{ 12000, 1 },
	{ 16000, 2 },
	{ 22050, 3 },
	{ 24000, 3 },
	{ 32000, 4 },
	{ 44100, 5 },
	{ 48000, 5 }
};

bool Wm8904::ReadReg(uint8_t Reg, uint16_t &Value)
{
	uint8_t data[2];

	if (!Interface() || Interface()->Read(DeviceAddress(), &Reg, 1, data, 2) != 2)
	{
		return false;
	}
	Value = uint16_t(data[0]) << 8 | data[1];
	return true;
}

bool Wm8904::WriteReg(uint8_t Reg, uint16_t Value)
{
	uint8_t data[] = { uint8_t(Value >> 8), uint8_t(Value) };

	if (!Interface() || Interface()->Write(DeviceAddress(), &Reg, 1, data, 2) != 2)
	{
		Valid(false);
		vbEnabled = false;
		return false;
	}
	return true;
}

bool Wm8904::Init(const Wm8904Cfg_t &Cfg, DeviceIntrf *pIntrf)
{
	uint16_t id;
	uint16_t rate = 0;
	uint32_t difference = UINT32_MAX;
	uint16_t format;

	if (Device::Valid())
	{
		PowerOff();
	}
	Valid(false);
	vbEnabled = false;
	if (!pIntrf || Cfg.DevAddr > 0x7F || !Cfg.Freq ||
		(Cfg.SampleWidth != I2SSWIDTH_16BITS && Cfg.SampleWidth != I2SSWIDTH_24BITS) ||
		(Cfg.Format != I2SFMT_NORMAL && Cfg.Format != I2SFMT_ALIGN_LEFT && Cfg.Format != I2SFMT_ALIGN_RIGHT) ||
		(Cfg.MicInput != 1 && Cfg.MicInput != 2) ||
		uint64_t(Cfg.Freq) * 256 > Cfg.MClkFreq || uint64_t(Cfg.Freq) * 256 + 255 < Cfg.MClkFreq)
	{
		return false;
	}
	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vCfg = Cfg;
	if (!ReadReg(WM8904_RESET, id) || id != 0x8904 || !WriteReg(WM8904_RESET, 0))
	{
		return false;
	}
	DeviceID(id);
	for (const auto &entry : s_Rates)
	{
		uint32_t diff = Cfg.Freq > entry.Freq ? Cfg.Freq - entry.Freq : entry.Freq - Cfg.Freq;

		if (diff < difference)
		{
			difference = diff;
			rate = entry.Value;
		}
	}
	format = 0;
	if (Cfg.Format == I2SFMT_NORMAL)
	{
		format = 2;
	}
	else if (Cfg.Format == I2SFMT_ALIGN_LEFT)
	{
		format = 1;
	}
	if (Cfg.SampleWidth == I2SSWIDTH_24BITS)
	{
		format |= 2U << 2;
	}
	// Outputs and analogue input PGAs remain powered down for DMIC capture.
	if (!WriteReg(WM8904_CLOCK_ENABLE, 4) || !WriteReg(WM8904_BIAS, 0x18) ||
		!WriteReg(WM8904_VMID, 0x47))
	{
		PowerOff();
		return false;
	}
	msDelay(100);
	if (!WriteReg(WM8904_VMID, 0x43) || !WriteReg(WM8904_BIAS, 0x19) ||
		!WriteReg(WM8904_MIC_BIAS, 1) || !WriteReg(WM8904_INPUT_POWER, 0) ||
		!WriteReg(WM8904_CLOCK_RATE, 0x0C00 | rate) ||
		!WriteReg(WM8904_AUDIO_FORMAT, format) ||
		!WriteReg(WM8904_AUDIO_SOURCE, 0x10) ||
		!WriteReg(WM8904_ADC_OSR, Cfg.Freq < 32000 ? 1 : 0) ||
		!WriteReg(WM8904_GPIO1, 8))
	{
		PowerOff();
		return false;
	}
	Valid(true);
	return Enable();
}

bool Wm8904::Enable()
{
	if (!Device::Valid())
	{
		return false;
	}
	if (vbEnabled)
	{
		return true;
	}
	if (!WriteReg(WM8904_CLOCK_ENABLE, 6) ||
		!WriteReg(WM8904_DMIC, vCfg.MicInput == 2 ? 0x1800 : 0x1000) ||
		!WriteReg(WM8904_CONVERTER_POWER, 2))
	{
		PowerOff();
		return false;
	}
	Gain(vCfg.Gain);
	vbEnabled = Device::Valid();
	if (!vbEnabled)
	{
		PowerOff();
	}
	return vbEnabled;
}

void Wm8904::Disable()
{
	if (!Device::Valid())
	{
		return;
	}
	if (!WriteReg(WM8904_ADC_LEFT_VOLUME, 0x100) ||
		!WriteReg(WM8904_ADC_RIGHT_VOLUME, 0x100) ||
		!WriteReg(WM8904_CONVERTER_POWER, 0) || !WriteReg(WM8904_DMIC, 0) ||
		!WriteReg(WM8904_CLOCK_ENABLE, 4))
	{
		PowerOff();
	}
	vbEnabled = false;
}

void Wm8904::PowerOff()
{
	if (Interface())
	{
		WriteReg(WM8904_RESET, 0);
	}
	Valid(false);
	vbEnabled = false;
}

void Wm8904::Reset()
{
	if (Interface())
	{
		Init(vCfg, Interface());
	}
}

int Wm8904::Gain(int MilliDb)
{
	int gain = MilliDb;

	if (gain < -71625)
	{
		gain = -71625;
	}
	else if (gain > 23625)
	{
		gain = 23625;
	}
	int code = 192 + (gain + (gain < 0 ? -187 : 187)) / 375;

	vCfg.Gain = (code - 192) * 375;
	if (Device::Valid())
	{
		if (!WriteReg(WM8904_ADC_LEFT_VOLUME, uint16_t(code)) ||
			!WriteReg(WM8904_ADC_RIGHT_VOLUME, uint16_t(code) | 0x100))
		{
			return 0;
		}
	}
	return vCfg.Gain;
}

int Wm8904::Gain() const
{
	return vCfg.Gain;
}

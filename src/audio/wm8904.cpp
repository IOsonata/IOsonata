/**-------------------------------------------------------------------------
@file	wm8904.cpp

@brief	WM8904 digital microphone and headphone control


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
#define WM8904_HEADPHONE_POWER 0x0E
#define WM8904_CONVERTER_POWER 0x12
#define WM8904_CLOCK_RATE 0x15
#define WM8904_CLOCK_ENABLE 0x16
#define WM8904_AUDIO_SOURCE 0x18
#define WM8904_AUDIO_FORMAT 0x19
#define WM8904_DAC_LEFT_VOLUME 0x1E
#define WM8904_DAC_RIGHT_VOLUME 0x1F
#define WM8904_DAC_CONTROL 0x21
#define WM8904_ADC_LEFT_VOLUME 0x24
#define WM8904_ADC_RIGHT_VOLUME 0x25
#define WM8904_DMIC 0x27
#define WM8904_HP_LEFT_VOLUME 0x39
#define WM8904_HP_RIGHT_VOLUME 0x3A
#define WM8904_SERVO_ENABLE 0x43
#define WM8904_SERVO_START 0x44
#define WM8904_SERVO_STATUS 0x4D
#define WM8904_HP_CONTROL 0x5A
#define WM8904_CHARGE_PUMP 0x62
#define WM8904_CLASS_W 0x68
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

bool Wm8904::VerifyReg(uint8_t Reg, uint16_t Value, uint16_t Mask)
{
	uint16_t actual;

	return ReadReg(Reg, actual) && (actual & Mask) == (Value & Mask);
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
		(Cfg.MicInput > 2 || (!Cfg.MicInput && !Cfg.bHeadphoneEn)) ||
		uint64_t(Cfg.Freq) * 256 > Cfg.MClkFreq || uint64_t(Cfg.Freq) * 256 + 255 < Cfg.MClkFreq)
	{
		return false;
	}
	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vCfg = Cfg;
	// R22 remains accessible without MCLK, even after an MCU-only reset
	// left the codec running. All other accesses below use CLK_SYS_ENA=0.
	if (!WriteReg(WM8904_CLOCK_ENABLE, 0) || !ReadReg(WM8904_RESET, id) ||
		id != 0x8904 || !WriteReg(WM8904_RESET, 0))
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
	// Analogue input PGAs stay off. Output stages are enabled separately.
	if (!WriteReg(WM8904_BIAS, 0x18) ||
		!WriteReg(WM8904_VMID, 0x47))
	{
		PowerOff();
		return false;
	}
	msDelay(100);
	if (!WriteReg(WM8904_VMID, 0x43) || !WriteReg(WM8904_BIAS, 0x19) ||
		!WriteReg(WM8904_MIC_BIAS, Cfg.MicInput ? 1 : 0) || !WriteReg(WM8904_INPUT_POWER, 0) ||
		!WriteReg(WM8904_CLOCK_RATE, 0x0C00 | rate) ||
		!WriteReg(WM8904_AUDIO_FORMAT, format) ||
		!WriteReg(WM8904_AUDIO_SOURCE, 0x10) ||
		!WriteReg(WM8904_ADC_OSR, Cfg.Freq < 32000 ? 1 : 0) ||
		!WriteReg(WM8904_GPIO1, Cfg.MicInput ? 8 : 0x14) ||
		!VerifyReg(WM8904_CLOCK_RATE, 0x0C00 | rate, 0x3C07) ||
		!VerifyReg(WM8904_AUDIO_FORMAT, format, 0x000F) ||
		!VerifyReg(WM8904_AUDIO_SOURCE, 0x10, 0x00F0) ||
		!VerifyReg(WM8904_GPIO1, Cfg.MicInput ? 8 : 0x14, 0x001F))
	{
		PowerOff();
		return false;
	}
	Valid(true);
	return Enable();
}

bool Wm8904::StartHeadphones()
{
	uint16_t status = 0;

	HeadphoneGain(vCfg.HeadphoneGain);
	if (!Device::Valid() ||
		!VerifyReg(WM8904_HP_LEFT_VOLUME, 57 + vCfg.HeadphoneGain / 1000, 0x003F) ||
		!VerifyReg(WM8904_CONVERTER_POWER, vCfg.MicInput ? 0x0E : 0x0C, 0x000F) ||
		!WriteReg(WM8904_HEADPHONE_POWER, 3) ||
		!WriteReg(WM8904_CHARGE_PUMP, 1) ||
		!WriteReg(WM8904_HP_CONTROL, 0x11) ||
		!WriteReg(WM8904_HP_CONTROL, 0x33) ||
		!WriteReg(WM8904_SERVO_ENABLE, 3) ||
		!WriteReg(WM8904_SERVO_START, 0x30))
	{
		return false;
	}
	// Keep output shorts engaged and DAC muted until both channels calibrate.
	for (unsigned count = 0; count < 30; count++)
	{
		msDelay(10);
		if (!ReadReg(WM8904_SERVO_STATUS, status))
		{
			return false;
		}
		if ((status & 3) == 3)
		{
			return WriteReg(WM8904_HP_CONTROL, 0x77) &&
				   WriteReg(WM8904_HP_CONTROL, 0xFF) &&
				   WriteReg(WM8904_CLASS_W, 1) &&
				   WriteReg(WM8904_DAC_CONTROL, 0x0640);
		}
	}
	return false;
}

bool Wm8904::StopHeadphones()
{
	// Attempt every shutdown step even if a control transfer fails.
	uint16_t state = 0;
	bool result = ReadReg(WM8904_HP_CONTROL, state);

	result = WriteReg(WM8904_DAC_CONTROL, 0x0648) && result;
	result = WriteReg(WM8904_HP_CONTROL, state & ~0x88U) && result;
	result = WriteReg(WM8904_HP_CONTROL, 0) && result;
	result = WriteReg(WM8904_SERVO_ENABLE, 0) && result;
	result = WriteReg(WM8904_CHARGE_PUMP, 0) && result;
	result = WriteReg(WM8904_HEADPHONE_POWER, 0) && result;
	result = WriteReg(WM8904_CLASS_W, 0) && result;
	return result;
}

bool Wm8904::Enable()
{
	uint16_t dmic = vCfg.MicInput ? (vCfg.MicInput == 2 ? 0x1800 : 0x1000) : 0;
	uint16_t power = (vCfg.MicInput ? 2 : 0) | (vCfg.bHeadphoneEn ? 0x0C : 0);

	if (!Device::Valid())
	{
		return false;
	}
	if (vbEnabled)
	{
		return true;
	}
	// Configure and read back the capture path before enabling SYSCLK.
	// Capture-only Init may run while the audio interface has MCLK stopped.
	if (!WriteReg(WM8904_CLOCK_ENABLE, 0) ||
		!WriteReg(WM8904_DMIC, dmic) ||
		!WriteReg(WM8904_CONVERTER_POWER, power))
	{
		PowerOff();
		return false;
	}
	Gain(vCfg.Gain);
	if (vCfg.bHeadphoneEn &&
		(!WriteReg(WM8904_DAC_CONTROL, 0x0648) ||
		 !WriteReg(WM8904_DAC_LEFT_VOLUME, 0x00C0) ||
		 !WriteReg(WM8904_DAC_RIGHT_VOLUME, 0x01C0)))
	{
		PowerOff();
		return false;
	}
	if (!Device::Valid() ||
		!VerifyReg(WM8904_DMIC, dmic, 0x1800) ||
		!VerifyReg(WM8904_CONVERTER_POWER, power, 0x000F) ||
		!VerifyReg(WM8904_ADC_LEFT_VOLUME, 192 + vCfg.Gain / 375, 0x00FF) ||
		!WriteReg(WM8904_CLOCK_ENABLE, 6) ||
		!VerifyReg(WM8904_CLOCK_ENABLE, 6, 0x0006) ||
		(vCfg.bHeadphoneEn && !StartHeadphones()))
	{
		PowerOff();
		return false;
	}
	vbEnabled = true;
	return true;
}

void Wm8904::Disable()
{
	if (!Device::Valid())
	{
		return;
	}
	if (!WriteReg(WM8904_CLOCK_ENABLE, 0) ||
		(vCfg.bHeadphoneEn && !StopHeadphones()) ||
		!WriteReg(WM8904_ADC_LEFT_VOLUME, 0x100) ||
		!WriteReg(WM8904_ADC_RIGHT_VOLUME, 0x100) ||
		!WriteReg(WM8904_CONVERTER_POWER, 0) || !WriteReg(WM8904_DMIC, 0))
	{
		PowerOff();
	}
	vbEnabled = false;
}

void Wm8904::PowerOff()
{
	if (Interface())
	{
		WriteReg(WM8904_CLOCK_ENABLE, 0);
		if (vCfg.bHeadphoneEn)
		{
			StopHeadphones();
		}
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
	else if (gain > 17625)
	{
		gain = 17625;
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

int Wm8904::HeadphoneGain(int MilliDb)
{
	int gain = MilliDb;

	if (gain < -57000)
	{
		gain = -57000;
	}
	else if (gain > 6000)
	{
		gain = 6000;
	}
	int code = 57 + (gain + (gain < 0 ? -500 : 500)) / 1000;

	vCfg.HeadphoneGain = (code - 57) * 1000;
	if (Device::Valid() && vCfg.bHeadphoneEn)
	{
		if (!WriteReg(WM8904_HP_LEFT_VOLUME, code) ||
			!WriteReg(WM8904_HP_RIGHT_VOLUME, code | 0x80))
		{
			return 0;
		}
	}
	return vCfg.HeadphoneGain;
}

int Wm8904::HeadphoneGain() const
{
	return vCfg.HeadphoneGain;
}

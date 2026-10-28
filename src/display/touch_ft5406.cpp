/**-------------------------------------------------------------------------
@file	touch_ft5406.cpp

@brief	FocalTech FT5406 capacitive touch panel controller, I2C

Registers as the NXP SDK fsl_ft5406.c: DEVICE_MODE (0x00) 0 for the normal
operating mode, the contacts read from 0x01 in one transfer: GEST_ID,
TD_STATUS (number of contacts), then 6 bytes per contact: event and X high,
X low, id and Y high, Y low. ID_G_CIPHER (0xA3) is the chip id, ID_G_PMODE
(0xA5) 3 the hibernate mode.

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
#include "idelay.h"
#include "display/touch_ft5406.h"

#define FT5406_REG_DEVICE_MODE		0x00
#define FT5406_REG_TOUCH			0x01
#define FT5406_REG_CIPHER			0xA3
#define FT5406_REG_PMODE			0xA5

#define FT5406_PMODE_HIBERNATE		3
#define FT5406_DEVICE_MODE_MASK		0x70

#define FT5406_EVT_DOWN				0
#define FT5406_EVT_CONTACT			2

#define FT5406_RESET_MS				5
#define FT5406_START_MS				300

bool TouchFt5406::Init(const TouchCfg_t &Cfg, DeviceIntrf *pIntrf)
{
	Valid(false);
	vbSleep = false;
	if (pIntrf == nullptr || SetPanel(Cfg, FT5406_I2C_ADDR) == false)
	{
		return false;
	}
	Interface(pIntrf);
	if (Start() == false)
	{
		return false;
	}
	Valid(true);

	return true;
}

// Reset, then the normal operating mode
bool TouchFt5406::Start()
{
	uint8_t reg = FT5406_REG_DEVICE_MODE;
	uint8_t val = 0;

	if (HasResetPin())
	{
		ResetPin(false);
		msDelay(FT5406_RESET_MS);
		ResetPin(true);
		msDelay(FT5406_START_MS);
	}
	if (Device::Write(&reg, 1, &val, 1) != 1)
	{
		return false;
	}
	val = 0xFF;
	if (Device::Read(&reg, 1, &val, 1) != 1 || (val & FT5406_DEVICE_MODE_MASK) != 0)
	{
		return false;
	}
	reg = FT5406_REG_CIPHER;
	if (Device::Read(&reg, 1, &val, 1) != 1)
	{
		return false;
	}
	DeviceID(val);

	return true;
}

bool TouchFt5406::Enable()
{
	if (Valid() == false)
	{
		return false;
	}
	if (vbSleep)
	{
		if (Start() == false)
		{
			return false;
		}
		vbSleep = false;
	}

	return true;
}

void TouchFt5406::Disable()
{
	uint8_t reg = FT5406_REG_PMODE;
	uint8_t val = FT5406_PMODE_HIBERNATE;

	if (Valid() && HasResetPin() && vbSleep == false)
	{
		Device::Write(&reg, 1, &val, 1);
		vbSleep = true;
	}
}

void TouchFt5406::Reset()
{
	if (Valid())
	{
		vbSleep = Start() == false;
	}
}

int TouchFt5406::Read(TouchPoint_t *pPoints, int MaxPoints)
{
	uint8_t reg = FT5406_REG_TOUCH;
	uint8_t d[2 + 6 * FT5406_MAX_POINTS];
	int count = 0;

	if (Valid() == false || pPoints == nullptr || MaxPoints < 0)
	{
		return -1;
	}
	if (vbSleep)
	{
		return 0;
	}
	if (Device::Read(&reg, 1, d, sizeof(d)) != (int)sizeof(d))
	{
		return -1;
	}

	int n = d[1] & 0x0F;

	if (n > FT5406_MAX_POINTS)
	{
		n = 0;
	}
	for (int i = 0; i < n && count < MaxPoints; i++)
	{
		const uint8_t *p = &d[2 + 6 * i];
		int evt = p[0] >> 6;

		// A contact lifted (1) or no event (3) is not a point
		if (evt != FT5406_EVT_DOWN && evt != FT5406_EVT_CONTACT)
		{
			continue;
		}
		uint16_t x = (uint16_t)(((p[0] & 0x0FU) << 8) | p[1]);
		uint16_t y = (uint16_t)(((p[2] & 0x0FU) << 8) | p[3]);

		if (Convert(x, y, (uint8_t)(p[2] >> 4), pPoints[count]))
		{
			count++;
		}
	}

	return count;
}

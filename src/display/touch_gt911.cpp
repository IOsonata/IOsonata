/**-------------------------------------------------------------------------
@file	touch_gt911.cpp

@brief	Goodix GT911 capacitive touch panel controller, I2C

Registers as the NXP SDK fsl_gt911.c, 16 bit addresses sent high byte
first: command (0x8040) 5 for the sleep mode, the configuration from 0x8047
(version, X and Y output maximum), the product id "911" at 0x8140, the
status at 0x814E (bit 7 new coordinates, bits 3:0 number of contacts, set to
0 by the host after the read), the contacts from 0x814F, 8 bytes each: id,
X low, X high, Y low, Y high, size. Reset sequence of the SDK: reset low
20 ms, the interrupt at the level of the address 1 ms, reset high, 5 ms,
interrupt low 50 ms, then input.

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
#include "display/touch_gt911.h"

#define GT911_REG_COMMAND			0x8040
#define GT911_REG_CONFIG			0x8047
#define GT911_REG_PRODUCT_ID		0x8140
#define GT911_REG_STATUS			0x814E
#define GT911_REG_POINT				0x814F

#define GT911_CMD_SLEEP				5
#define GT911_STATUS_READY			0x80
#define GT911_STATUS_COUNT			0x0F
#define GT911_POINT_SIZE			8

int TouchGt911::ReadReg(uint16_t Reg, uint8_t *pData, int Len)
{
	uint8_t addr[2] = { (uint8_t)(Reg >> 8), (uint8_t)Reg };

	return Device::Read(addr, 2, pData, Len);
}

int TouchGt911::WriteReg(uint16_t Reg, const uint8_t *pData, int Len)
{
	uint8_t addr[2] = { (uint8_t)(Reg >> 8), (uint8_t)Reg };

	return Device::Write(addr, 2, pData, Len);
}

bool TouchGt911::Init(const TouchCfg_t &Cfg, DeviceIntrf *pIntrf)
{
	Valid(false);
	vbSleep = false;
	if (pIntrf == nullptr || SetPanel(Cfg, GT911_I2C_ADDR0) == false ||
		(vCfg.DevAddr != GT911_I2C_ADDR0 && vCfg.DevAddr != GT911_I2C_ADDR1))
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

// Reset sequence for the address, then the product id and the coordinate
// range
bool TouchGt911::Start()
{
	uint8_t d[5];

	if (HasResetPin())
	{
		ResetPin(false);
		msDelay(20);
		IntPinOut(vCfg.DevAddr == GT911_I2C_ADDR1);
		msDelay(1);
		ResetPin(true);
		msDelay(5);
		IntPinOut(false);
		msDelay(50);
		IntPinIn();
	}
	if (ReadReg(GT911_REG_PRODUCT_ID, d, 4) != 4 || d[0] != '9' || d[1] != '1' || d[2] != '1')
	{
		return false;
	}
	DeviceID(911);
	if (ReadReg(GT911_REG_CONFIG, d, 5) != 5)
	{
		return false;
	}

	uint16_t x = (uint16_t)(d[1] | (d[2] << 8));
	uint16_t y = (uint16_t)(d[3] | (d[4] << 8));

	// The panel size in pixels when the configuration has none
	if (x != 0 && y != 0)
	{
		Range(x, y);
	}
	vNbRaw = 0;

	return true;
}

bool TouchGt911::Enable()
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

void TouchGt911::Disable()
{
	uint8_t cmd = GT911_CMD_SLEEP;

	if (Valid() && HasResetPin() && HasIntPin() && vbSleep == false)
	{
		// The interrupt pin held low in the sleep mode
		IntPinOut(false);
		WriteReg(GT911_REG_COMMAND, &cmd, 1);
		vbSleep = true;
	}
}

void TouchGt911::Reset()
{
	if (Valid())
	{
		vbSleep = Start() == false;
	}
}

int TouchGt911::Read(TouchPoint_t *pPoints, int MaxPoints)
{
	uint8_t status;
	int count = 0;

	if (Valid() == false || pPoints == nullptr || MaxPoints < 0)
	{
		return -1;
	}
	if (vbSleep)
	{
		return 0;
	}
	if (ReadReg(GT911_REG_STATUS, &status, 1) != 1)
	{
		return -1;
	}
	if (status & GT911_STATUS_READY)
	{
		uint8_t d[GT911_POINT_SIZE * GT911_MAX_POINTS];
		int n = status & GT911_STATUS_COUNT;

		if (n > GT911_MAX_POINTS)
		{
			n = 0;
		}
		if (n > 0 && ReadReg(GT911_REG_POINT, d, n * GT911_POINT_SIZE) != n * GT911_POINT_SIZE)
		{
			return -1;
		}
		// The status cleared for the next coordinates
		status = 0;
		if (WriteReg(GT911_REG_STATUS, &status, 1) != 1)
		{
			return -1;
		}
		for (int i = 0; i < n; i++)
		{
			const uint8_t *p = &d[GT911_POINT_SIZE * i];

			vRaw[i].Id = p[0];
			vRaw[i].X = (uint16_t)(p[1] | (p[2] << 8));
			vRaw[i].Y = (uint16_t)(p[3] | (p[4] << 8));
		}
		vNbRaw = n;
	}
	for (int i = 0; i < vNbRaw && count < MaxPoints; i++)
	{
		if (Convert(vRaw[i].X, vRaw[i].Y, vRaw[i].Id, pPoints[count]))
		{
			count++;
		}
	}

	return count;
}

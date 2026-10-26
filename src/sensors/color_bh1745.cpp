/**-------------------------------------------------------------------------
@file	color_bh1745.cpp

@brief	BH1745NUC RGBC sensor.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

/** BH1745NUC RGBC sensor. SPDX-License-Identifier: MIT */
#include "sensors/color_bh1745.h"
#include "idelay.h"

ColorBh1745::ColorBh1745()
{
	vEvtHandler = nullptr;
	vIntId = 0;
	vByteOrder = DEV_BYTEORDER_LITTLE;
}

bool ColorBh1745::ReadReg(uint8_t Reg, uint8_t *pData, int Len)
{
	return vpIntrf != nullptr && Device::Read(&Reg, 1, pData, Len) == Len;
}

bool ColorBh1745::WriteReg(uint8_t Reg, const uint8_t *pData, int Len)
{
	return vpIntrf != nullptr && Device::Write(&Reg, 1, pData, Len) == Len;
}

bool ColorBh1745::Init(DeviceIntrf *pIntrf, uint32_t Addr)
{
	uint8_t id;
	uint8_t manufacturer;

	Valid(false);
	if (pIntrf == nullptr || pIntrf->Type() != DEVINTRF_TYPE_I2C ||
		(Addr != BH1745_I2C_ADDR0 && Addr != BH1745_I2C_ADDR1))
		return false;
	Interface(pIntrf);
	DeviceAddress(Addr);
	msDelay(2);
	if (!ReadReg(0x40, &id, 1) || (id & 0x3F) != 0x0B ||
		!ReadReg(0x92, &manufacturer, 1) || manufacturer != 0xE0)
		return false;
	DeviceID(id & 0x3F);
	return SoftReset();
}

bool ColorBh1745::SoftReset()
{
	uint8_t reset = 0x80;
	uint8_t required = 0x02;

	Valid(false);
	if (!WriteReg(0x40, &reset, 1))
		return false;
	msDelay(2);
	if (!WriteReg(0x44, &required, 1))
		return false;
	Valid(true);
	return true;
}

void ColorBh1745::Reset()
{
	SoftReset();
}

bool ColorBh1745::Enable()
{
	uint8_t mode;

	if (!Valid() || !ReadReg(0x42, &mode, 1))
		return false;
	mode = (mode & 3) | 0x10;
	return WriteReg(0x42, &mode, 1);
}

void ColorBh1745::Disable()
{
	uint8_t mode;

	if (!Valid())
		return;
	if (!ReadReg(0x42, &mode, 1))
	{
		Valid(false);
		return;
	}
	mode &= 3;
	if (!WriteReg(0x42, &mode, 1))
		Valid(false);
}

bool ColorBh1745::Configure(uint8_t TimeCode, uint8_t Gain)
{
	uint8_t mode;
	uint8_t gain;
	uint8_t required = 2;

	if (!Valid() || TimeCode > 5 || (Gain != 1 && Gain != 2 && Gain != 16))
		return false;
	if (!ReadReg(0x42, &mode, 1))
		return false;
	gain = Gain == 16 ? 2 : Gain - 1;
	gain |= mode & 0x10;
	if (!WriteReg(0x41, &TimeCode, 1) || !WriteReg(0x42, &gain, 1) ||
		!WriteReg(0x44, &required, 1))
	{
		Valid(false);
		return false;
	}
	return true;
}

bool ColorBh1745::Read(Bh1745Data_t &Data)
{
	uint8_t mode;
	uint8_t raw[8];
	Bh1745Data_t sample;

	if (!Valid() || !ReadReg(0x42, &mode, 1) || (mode & 0x80) == 0 ||
		!ReadReg(0x50, raw, sizeof(raw)))
		return false;
	sample.Red = raw[0] | ((uint16_t)raw[1] << 8);
	sample.Green = raw[2] | ((uint16_t)raw[3] << 8);
	sample.Blue = raw[4] | ((uint16_t)raw[5] << 8);
	sample.Clear = raw[6] | ((uint16_t)raw[7] << 8);
	Data = sample;
	return true;
}

bool ColorBh1745::ConfigureInterrupt(bool Enable, uint8_t Source, uint8_t Persistence,
	uint16_t Lower, uint16_t Upper, bool Latched)
{
	uint8_t thresholds[4] = { (uint8_t)Upper, (uint8_t)(Upper >> 8),
		(uint8_t)Lower, (uint8_t)(Lower >> 8) };
	uint8_t control = 0;

	if (!Valid() || Source > 3 || Persistence > 3 || Lower > Upper)
		return false;
	if (!WriteReg(0x60, &control, 1) || !WriteReg(0x62, thresholds, 4) ||
		!WriteReg(0x61, &Persistence, 1))
		return false;
	control = (Source << 2) | (Latched ? 0 : 0x10) | (Enable ? 1 : 0);
	return WriteReg(0x60, &control, 1);
}

bool ColorBh1745::ClearInterrupt()
{
	uint8_t clear = 0x40;

	return Valid() && WriteReg(0x40, &clear, 1);
}

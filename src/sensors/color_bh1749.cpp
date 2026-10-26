/**-------------------------------------------------------------------------
@file color_bh1749.cpp

@brief ROHM BH1749NUC RGB/IR color sensor (I2C)

@license
MIT License

Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#include "sensors/color_bh1749.h"
#include "idelay.h"

#define BH1749_SYSTEM_CONTROL_REG			0x40U
#define BH1749_MODE_CONTROL1_REG				0x41U
#define BH1749_MODE_CONTROL2_REG				0x42U
#define BH1749_RED_DATA_REG					0x50U
#define BH1749_INTERRUPT_REG					0x60U
#define BH1749_PERSISTENCE_REG				0x61U
#define BH1749_TH_HIGH_REG					0x62U
#define BH1749_TH_LOW_REG					0x64U
#define BH1749_MANUFACTURER_REG				0x92U

#define BH1749_PART_ID_MASK					0x3FU
#define BH1749_MANUFACTURER_ID				0xE0U
#define BH1749_SOFT_RESET_MASK				0x80U
#define BH1749_INT_RESET_MASK				0x40U
#define BH1749_VALID_MASK					0x80U
#define BH1749_RGB_ENABLE					0x10U
#define BH1749_INT_STATUS_MASK				0x80U
#define BH1749_INT_SOURCE_SHIFT				2U
#define BH1749_INT_ENABLE_MASK				0x01U

ColorBh1749::ColorBh1749()
{
	DeviceAddress(BH1749_I2C_ADDR0);
	vEvtHandler = NULL;
	vIntId = 0;
	vByteOrder = DEV_BYTEORDER_LITTLE;
}

bool ColorBh1749::ReadReg(uint8_t Reg, uint8_t *pData, int Len)
{
	if (vpIntrf == NULL || pData == NULL || Len <= 0)
		return false;

	return Device::Read(&Reg, 1, pData, Len) == Len;
}

bool ColorBh1749::WriteReg(uint8_t Reg, uint8_t Value)
{
	if (vpIntrf == NULL)
		return false;

	return Device::Write(&Reg, 1, &Value, 1) == 1;
}

bool ColorBh1749::WriteReg16(uint8_t Reg, uint16_t Value)
{
	uint8_t data[2];

	if (vpIntrf == NULL)
		return false;

	data[0] = (uint8_t)Value;
	data[1] = (uint8_t)(Value >> 8);

	return Device::Write(&Reg, 1, data, sizeof(data)) == sizeof(data);
}

bool ColorBh1749::Init(DeviceIntrf * const pIntrf, uint32_t DevAddr)
{
	uint8_t id;
	uint8_t manufacturer;

	Valid(false);
	Interface(NULL);
	DeviceID(UINT64_MAX);

	if (pIntrf == NULL || pIntrf->Type() != DEVINTRF_TYPE_I2C ||
		(DevAddr != BH1749_I2C_ADDR0 && DevAddr != BH1749_I2C_ADDR1))
		return false;

	Interface(pIntrf);
	DeviceAddress(DevAddr);
	msDelay(2);

	if (!ReadReg(BH1749_SYSTEM_CONTROL_REG, &id, 1))
		return false;

	if ((id & BH1749_PART_ID_MASK) != BH1749_PART_ID)
		return false;

	if (!ReadReg(BH1749_MANUFACTURER_REG, &manufacturer, 1) ||
		manufacturer != BH1749_MANUFACTURER_ID)
		return false;

	DeviceID(id & BH1749_PART_ID_MASK);
	Valid(true);

	if (!Configure(COLOR_BH1749_GAIN_1X, COLOR_BH1749_GAIN_1X,
		COLOR_BH1749_MEASURE_120MS) || !Enable())
	{
		Valid(false);
		return false;
	}

	return true;
}

bool ColorBh1749::Enable()
{
	if (!Valid())
		return false;

	return WriteReg(BH1749_MODE_CONTROL2_REG, BH1749_RGB_ENABLE);
}

void ColorBh1749::Disable()
{
	if (!Valid())
		return;

	if (!WriteReg(BH1749_MODE_CONTROL2_REG, 0))
		Valid(false);
}

void ColorBh1749::Reset()
{
	// The reset measurement fields are not usable until Init configures them.
	if (Valid())
		WriteReg(BH1749_SYSTEM_CONTROL_REG, BH1749_SOFT_RESET_MASK);
	Valid(false);
}

bool ColorBh1749::Configure(ColorBh1749Gain_t RgbGain,
	ColorBh1749Gain_t IrGain, ColorBh1749MeasureTime_t MeasureTime)
{
	uint8_t value;

	if (!Valid())
		return false;

	if ((RgbGain != COLOR_BH1749_GAIN_1X &&
		RgbGain != COLOR_BH1749_GAIN_32X) ||
		(IrGain != COLOR_BH1749_GAIN_1X &&
		IrGain != COLOR_BH1749_GAIN_32X) ||
		(MeasureTime != COLOR_BH1749_MEASURE_120MS &&
		MeasureTime != COLOR_BH1749_MEASURE_240MS &&
		MeasureTime != COLOR_BH1749_MEASURE_35MS))
		return false;

	value = ((uint8_t)IrGain << 5) |
			((uint8_t)RgbGain << 3) |
			(uint8_t)MeasureTime;

	return WriteReg(BH1749_MODE_CONTROL1_REG, value);
}

bool ColorBh1749::ConfigureInterrupt(ColorBh1749IntSource_t Source,
	uint16_t LowThreshold, uint16_t HighThreshold,
	ColorBh1749IntPersistence_t Persistence)
{
	uint8_t value;

	if (!Valid() || LowThreshold > HighThreshold)
		return false;

	if ((Source != COLOR_BH1749_INT_RED &&
		Source != COLOR_BH1749_INT_GREEN &&
		Source != COLOR_BH1749_INT_BLUE) ||
		(Persistence != COLOR_BH1749_INT_EACH_MEASUREMENT &&
		Persistence != COLOR_BH1749_INT_UPDATE_EACH_MEASUREMENT &&
		Persistence != COLOR_BH1749_INT_4_CONSECUTIVE &&
		Persistence != COLOR_BH1749_INT_8_CONSECUTIVE))
		return false;

	if (!WriteReg16(BH1749_TH_LOW_REG, LowThreshold))
		return false;

	if (!WriteReg16(BH1749_TH_HIGH_REG, HighThreshold))
		return false;

	if (!WriteReg(BH1749_PERSISTENCE_REG, (uint8_t)Persistence))
		return false;

	value = ((uint8_t)Source << BH1749_INT_SOURCE_SHIFT) |
			BH1749_INT_ENABLE_MASK;

	return WriteReg(BH1749_INTERRUPT_REG, value);
}

bool ColorBh1749::DisableInterrupt()
{
	if (!Valid())
		return false;

	return WriteReg(BH1749_INTERRUPT_REG, 0);
}

bool ColorBh1749::ReadInterruptStatus(bool &Active)
{
	uint8_t value;

	if (!Valid() || !ReadReg(BH1749_INTERRUPT_REG, &value, 1))
		return false;

	Active = (value & BH1749_INT_STATUS_MASK) != 0;

	return true;
}

bool ColorBh1749::ClearInterrupt()
{
	if (!Valid())
		return false;

	return WriteReg(BH1749_SYSTEM_CONTROL_REG, BH1749_INT_RESET_MASK);
}

bool ColorBh1749::Read(ColorBh1749Data_t &Data)
{
	uint8_t status;
	uint8_t raw[12];
	ColorBh1749Data_t sample;

	if (!Valid())
		return false;

	/*
	 * Reading MODE_CONTROL2 clears VALID. The data registers still contain
	 * the completed sample, so read them only after observing VALID set.
	 */
	if (!ReadReg(BH1749_MODE_CONTROL2_REG, &status, 1))
		return false;

	if ((status & BH1749_VALID_MASK) == 0)
		return false;

	if (!ReadReg(BH1749_RED_DATA_REG, raw, sizeof(raw)))
		return false;

	sample.Red = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
	sample.Green = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
	sample.Blue = (uint16_t)raw[4] | ((uint16_t)raw[5] << 8);
	sample.Ir = (uint16_t)raw[8] | ((uint16_t)raw[9] << 8);
	sample.Green2 = (uint16_t)raw[10] | ((uint16_t)raw[11] << 8);

	Data = sample;

	return true;
}

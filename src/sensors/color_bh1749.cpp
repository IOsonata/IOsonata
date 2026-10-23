/**-------------------------------------------------------------------------
@file color_bh1749.cpp
@brief ROHM BH1749NUC RGB/IR color sensor (I2C)
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "sensors/color_bh1749.h"

#define BH1749_SYSTEM_CONTROL_REG  0x40
#define BH1749_MODE_CONTROL1_REG   0x41
#define BH1749_MODE_CONTROL2_REG   0x42
#define BH1749_RED_DATA_REG        0x50
#define BH1749_PART_ID_MASK        0x3F
#define BH1749_VALID_MASK          0x80
#define BH1749_RGB_ENABLE         0x10
#define BH1749_GAIN1_120MS        0x2A

bool ColorBh1749::ReadReg(uint8_t Reg, uint8_t *pData, int Len)
{
	if (vpIntrf == NULL || pData == NULL || Len <= 0)
		return false;

	return vpIntrf->Read(vDevAddr, &Reg, 1, pData, Len) == Len;
}

bool ColorBh1749::WriteReg(uint8_t Reg, uint8_t Value)
{
	if (vpIntrf == NULL)
		return false;

	return vpIntrf->Write(vDevAddr, &Reg, 1, &Value, 1) == 1;
}

bool ColorBh1749::Init(DeviceIntrf * const pIntrf, uint32_t DevAddr)
{
	uint8_t id;

	vbInitialized = false;
	vpIntrf = NULL;
	if (pIntrf == NULL || (DevAddr != BH1749_I2C_ADDR0 && DevAddr != BH1749_I2C_ADDR1))
		return false;

	vpIntrf = pIntrf;
	vDevAddr = DevAddr;
	if (!ReadReg(BH1749_SYSTEM_CONTROL_REG, &id, 1))
		return false;
	if ((id & BH1749_PART_ID_MASK) != BH1749_PART_ID)
		return false;
	if (!WriteReg(BH1749_MODE_CONTROL1_REG, BH1749_GAIN1_120MS))
		return false;
	vbInitialized = true;
	return Enable();
}

bool ColorBh1749::Enable()
{
	if (!vbInitialized)
		return false;

	return WriteReg(BH1749_MODE_CONTROL2_REG, BH1749_RGB_ENABLE);
}

bool ColorBh1749::Disable()
{
	if (!vbInitialized)
		return false;

	return WriteReg(BH1749_MODE_CONTROL2_REG, 0);
}

bool ColorBh1749::Read(ColorBh1749Data_t &Data)
{
	uint8_t status;
	uint8_t raw[12];
	ColorBh1749Data_t sample;

	if (!vbInitialized)
		return false;
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

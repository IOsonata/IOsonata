/**-------------------------------------------------------------------------
@file accel_adxl367.cpp
@brief ADXL367 14-bit acceleration sampling (I2C/SPI)
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "sensors/accel_adxl367.h"

#define REG_ID 0x00
#define REG_STATUS 0x0B
#define REG_XDATA_H 0x0E
#define REG_FILTER_CTL 0x2C
#define REG_POWER_CTL 0x2D
#define MEASURE_START 0x02

bool AccelAdxl367::ReadReg(uint8_t Reg, uint8_t *pData, int Len)
{
	DeviceIntrf *pIntrf;
	uint8_t cmd[2];

	pIntrf = Interface();
	if (pIntrf == NULL || pData == NULL || Len <= 0)
		return false;
	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		cmd[0] = 0x0B;
		cmd[1] = Reg;
		return pIntrf->Read(DeviceAddress(), cmd, 2, pData, Len) == Len;
	}
	return pIntrf->Read(DeviceAddress(), &Reg, 1, pData, Len) == Len;
}

bool AccelAdxl367::WriteReg(uint8_t Reg, uint8_t Value)
{
	DeviceIntrf *pIntrf;
	uint8_t cmd[3];

	pIntrf = Interface();
	if (pIntrf == NULL)
		return false;
	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		cmd[0] = 0x0A;
		cmd[1] = Reg;
		cmd[2] = Value;
		return pIntrf->Tx(DeviceAddress(), cmd, 3) == 3;
	}
	return pIntrf->Write(DeviceAddress(), &Reg, 1, &Value, 1) == 1;
}

bool AccelAdxl367::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint8_t id[3];
	uint8_t odr;
	uint32_t freq;
	uint16_t scale;

	vbInitialized = false;
	if (pIntrf == NULL)
		return false;
	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vpTimer = pTimer;
	if (!ReadReg(REG_ID, id, 3))
		return false;
	if (id[0] != 0xAD || id[1] != 0x1D || id[2] != ADXL367_PART_ID)
		return false;
	if (Cfg.Freq <= 12500) { freq = 12500; odr = 0; }
	else if (Cfg.Freq <= 25000) { freq = 25000; odr = 1; }
	else if (Cfg.Freq <= 50000) { freq = 50000; odr = 2; }
	else if (Cfg.Freq <= 100000) { freq = 100000; odr = 3; }
	else if (Cfg.Freq <= 200000) { freq = 200000; odr = 4; }
	else { freq = 400000; odr = 5; }
	scale = Cfg.Scale <= 2 ? 2 : Cfg.Scale <= 4 ? 4 : 8;
	// Preserve the default I2C high-speed bit; configure only ODR and range.
	vFilter = (uint8_t)(0x20 | odr | (scale == 2 ? 0 : scale == 4 ? 0x40 : 0x80));
	if (!WriteReg(REG_POWER_CTL, 0))
		return false;
	if (!WriteReg(REG_FILTER_CTL, vFilter))
		return false;
	if (!Mode(Cfg.OpMode, freq))
		return false;
	AccelSensor::Scale(scale);
	DeviceID(ADXL367_PART_ID);
	vbInitialized = true;
	return true;
}

bool AccelAdxl367::Enable()
{
	return vbInitialized && WriteReg(REG_POWER_CTL, MEASURE_START);
}

void AccelAdxl367::Disable()
{
	if (vbInitialized)
		WriteReg(REG_POWER_CTL, 0);
}

void AccelAdxl367::Reset()
{
	// Keep the device in standby. Full reset requires a subsequent init.
	if (WriteReg(REG_POWER_CTL, 0))
		vbInitialized = false;
}

bool AccelAdxl367::StartSampling()
{
	return Enable();
}

uint16_t AccelAdxl367::Scale(uint16_t Value)
{
	uint16_t scale;
	uint8_t filter;

	if (!vbInitialized)
		return AccelSensor::Scale();
	scale = Value <= 2 ? 2 : Value <= 4 ? 4 : 8;
	filter = (uint8_t)((vFilter & 0x3F) | (scale == 2 ? 0 : scale == 4 ? 0x40 : 0x80));
	if (!WriteReg(REG_FILTER_CTL, filter))
		return AccelSensor::Scale();
	vFilter = filter;
	return AccelSensor::Scale(scale);
}

bool AccelAdxl367::UpdateData()
{
	uint8_t status;
	uint8_t raw[6];
	AccelSensorRawData_t sample;
	uint16_t x;
	uint16_t y;
	uint16_t z;

	if (!vbInitialized || !ReadReg(REG_STATUS, &status, 1))
		return false;
	if ((status & 1) == 0 || !ReadReg(REG_XDATA_H, raw, sizeof(raw)))
		return false;
	x = ((uint16_t)raw[0] << 6) | (raw[1] >> 2);
	y = ((uint16_t)raw[2] << 6) | (raw[3] >> 2);
	z = ((uint16_t)raw[4] << 6) | (raw[5] >> 2);
	sample = vData;
	sample.X = (int16_t)((int16_t)(x << 2) >> 2);
	sample.Y = (int16_t)((int16_t)(y << 2) >> 2);
	sample.Z = (int16_t)((int16_t)(z << 2) >> 2);
	sample.Timestamp = vpTimer != NULL ? vpTimer->uSecond() : vData.Timestamp + 1;
	vData = sample;
	return true;
}

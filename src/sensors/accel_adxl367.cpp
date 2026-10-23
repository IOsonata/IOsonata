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
#define REG_THRESH_ACT_H 0x20
#define REG_TIME_ACT 0x22
#define REG_ACT_INACT 0x27
#define REG_FIFO_CTL 0x28
#define REG_FIFO_SAMPLES 0x29
#define REG_INT1_MAP 0x2A
#define REG_INT2_MAP 0x2B
#define REG_FIFO_ENTRIES 0x0C
#define REG_I2C_FIFO 0x18

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
	if (!WriteReg(REG_FIFO_CTL, 0) || !WriteReg(REG_INT1_MAP, 0) ||
		!WriteReg(REG_INT2_MAP, 0) || !WriteReg(REG_ACT_INACT, 0))
		return false;
	vInt1Map = vInt2Map = vFifoControl = 0;
	vbFifoEnabled = false;
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

bool AccelAdxl367::Status(uint8_t &Flags)
{
	return vbInitialized && ReadReg(REG_STATUS, &Flags, 1);
}

bool AccelAdxl367::ConfigureInterrupts(uint8_t Int1Map, uint8_t Int2Map)
{
	if (!vbInitialized)
		return false;
	if (!WriteReg(REG_INT1_MAP, Int1Map) || !WriteReg(REG_INT2_MAP, Int2Map))
		return false;
	vInt1Map = Int1Map;
	vInt2Map = Int2Map;
	return true;
}

bool AccelAdxl367::WakeOnEvent(bool bEnable, int Threshold)
{
	uint8_t high;
	uint8_t low;
	uint8_t ctl;

	if (!vbInitialized)
		return false;
	if (!bEnable)
	{
		if (!WriteReg(REG_ACT_INACT, 0))
			return false;
		return ConfigureInterrupts(vInt1Map & ~0x10, vInt2Map);
	}
	if (Threshold < 0 || Threshold > 8191)
		return false;
	high = (uint8_t)((unsigned int)Threshold >> 6);
	low = (uint8_t)((unsigned int)Threshold << 2);
	ctl = 1;
	if (!WriteReg(REG_THRESH_ACT_H, high) || !WriteReg(REG_THRESH_ACT_H + 1, low))
		return false;
	if (!WriteReg(REG_TIME_ACT, 1) || !WriteReg(REG_ACT_INACT, ctl))
		return false;
	return ConfigureInterrupts(vInt1Map | 0x10, vInt2Map);
}

bool AccelAdxl367::ConfigureFifo(uint16_t WatermarkSamples)
{
	uint16_t words;
	uint8_t ctl;

	if (!vbInitialized || WatermarkSamples > 170)
		return false;
	if (WatermarkSamples == 0)
	{
		if (!WriteReg(REG_FIFO_CTL, 0))
			return false;
		vbFifoEnabled = false;
		vFifoControl = 0;
		return true;
	}
	words = WatermarkSamples * 3;
	ctl = (uint8_t)(2 | ((words & 0x100) ? 4 : 0));
	if (!WriteReg(REG_FIFO_CTL, 0) ||
		!WriteReg(REG_FIFO_SAMPLES, (uint8_t)words) ||
		!WriteReg(REG_FIFO_CTL, ctl))
		return false;
	vFifoControl = ctl;
	vbFifoEnabled = true;
	return true;
}

bool AccelAdxl367::ReadFifo(AccelSensorRawData_t &Data)
{
	uint8_t entries[2];
	uint8_t raw[6];
	uint16_t count;
	uint16_t words[3];
	AccelSensorRawData_t sample;
	DeviceIntrf *pIntrf;
	int i;

	if (!vbInitialized || !vbFifoEnabled)
		return false;
	if (!ReadReg(REG_FIFO_ENTRIES, entries, 2))
		return false;
	count = (uint16_t)entries[0] | (((uint16_t)entries[1] & 3) << 8);
	if (count < 3)
		return false;
	pIntrf = Interface();
	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint8_t cmd = 0x0D;
		if (pIntrf->Read(DeviceAddress(), &cmd, 1, raw, 6) != 6)
			return false;
	}
	else if (!ReadReg(REG_I2C_FIFO, raw, 6))
		return false;
	for (i = 0; i < 3; i++)
	{
		words[i] = (uint16_t)raw[i * 2] | ((uint16_t)raw[i * 2 + 1] << 8);
		if ((words[i] >> 14) != (unsigned int)i)
			return false;
	}
	sample = vData;
	sample.X = (int16_t)((int16_t)(words[0] << 2) >> 2);
	sample.Y = (int16_t)((int16_t)(words[1] << 2) >> 2);
	sample.Z = (int16_t)((int16_t)(words[2] << 2) >> 2);
	sample.Timestamp = vpTimer ? vpTimer->uSecond() : vData.Timestamp + 1;
	vData = sample;
	Data = sample;
	return true;
}

/**-------------------------------------------------------------------------
@file	mag_bmm350.cpp
@brief	Bosch BMM350 magnetometer implementation
@license MIT License
Copyright (c) 2024-2026, I-SYST inc., all rights reserved
-------------------------------------------------------------------------*/

#include <math.h>
#include <string.h>

#include "idelay.h"
#include "sensors/mag_bmm350.h"

#define BMM350_STARTUP_DELAY_US		3000U
#define BMM350_SOFTRESET_DELAY_US	24000U
#define BMM350_GOTO_SUSPEND_DELAY_US	6000U
#define BMM350_SUSPEND_NORMAL_DELAY_US	38000U
#define BMM350_UPD_OAE_DELAY_US		1000U
#define BMM350_BR_DELAY_US		14000U
#define BMM350_FGR_DELAY_US		18000U
#define BMM350_OTP_POLL_DELAY_US	300U
#define BMM350_OTP_POLL_COUNT		100U
#define BMM350_OTP_POWER_OFF		0x80U
#define BMM350_PMU_CMD_VALUE_MASK	0xE0U
#define BMM350_PMU_CMD_VALUE_SHIFT	5U

#define BMM350_TEMP_OFF_SENS		0x0DU
#define BMM350_MAG_OFFSET_X		0x0EU
#define BMM350_MAG_OFFSET_Y		0x0FU
#define BMM350_MAG_OFFSET_Z		0x10U
#define BMM350_MAG_SENS_X		0x10U
#define BMM350_MAG_SENS_Y		0x11U
#define BMM350_MAG_SENS_Z		0x11U
#define BMM350_MAG_TCO_X		0x12U
#define BMM350_MAG_TCO_Y		0x13U
#define BMM350_MAG_TCO_Z		0x14U
#define BMM350_MAG_TCS_X		0x12U
#define BMM350_MAG_TCS_Y		0x13U
#define BMM350_MAG_TCS_Z		0x14U
#define BMM350_CROSS_X_Y		0x15U
#define BMM350_CROSS_Y_X		0x15U
#define BMM350_CROSS_Z_X		0x16U
#define BMM350_CROSS_Z_Y		0x16U
#define BMM350_MAG_DUT_T_0		0x18U

MagBmm350::MagBmm350()
{
	memset(vOtp, 0, sizeof(vOtp));
	memset(vOffset, 0, sizeof(vOffset));
	memset(vSens, 0, sizeof(vSens));
	memset(vTco, 0, sizeof(vTco));
	memset(vTcs, 0, sizeof(vTcs));
	memset(vCross, 0, sizeof(vCross));
	vT0 = 23.0f;
	vbInitialized = false;
}

bool MagBmm350::ReadReg(uint8_t Reg, uint8_t *pData, int Len)
{
	uint8_t reg;

	if (Interface() == NULL || pData == NULL || Len <= 0)
	{
		return false;
	}

	reg = Reg;
	return MagBmm350::Read(&reg, 1, pData, Len) == Len;
}

bool MagBmm350::WriteReg(uint8_t Reg, uint8_t Value)
{
	uint8_t reg;

	if (Interface() == NULL)
	{
		return false;
	}

	reg = Reg;
	return MagBmm350::Write(&reg, 1, &Value, 1) == 1;
}

int32_t MagBmm350::SignExtend(uint32_t Value, unsigned Bits)
{
	uint32_t sign;
	uint32_t mask;

	sign = 1UL << (Bits - 1U);
	mask = (1UL << Bits) - 1UL;
	Value &= mask;
	return (int32_t)((Value ^ sign) - sign);
}

bool MagBmm350::ReadOtpWord(uint8_t Addr, uint16_t &Word)
{
	uint8_t status;
	uint8_t msb;
	uint8_t lsb;
	uint8_t error;
	uint32_t poll;

	if (!WriteReg(BMM350_OTP_CMD_REG,
		(uint8_t)(BMM350_OTP_CMD_DIR_READ |
		(Addr & BMM350_OTP_CMD_WORD_ADDR_MASK))))
	{
		return false;
	}

	for (poll = 0; poll < BMM350_OTP_POLL_COUNT; poll++)
	{
		usDelay(BMM350_OTP_POLL_DELAY_US);
		if (!ReadReg(BMM350_OTP_STATUS_REG, &status, 1))
		{
			return false;
		}

		error = status & BMM350_OTP_STATUS_ERR_MASK;
		if (error != BMM350_OTP_STATUS_ERR_NONE)
		{
			return false;
		}
		if ((status & BMM350_OTP_STATUS_CMD_DONE) != 0)
		{
			if (!ReadReg(BMM350_OTP_DATA_MSB_REG, &msb, 1) ||
				!ReadReg(BMM350_OTP_DATA_LSB_REG, &lsb, 1))
			{
				return false;
			}
			Word = ((uint16_t)msb << 8) | lsb;
			return true;
		}
	}

	return false;
}

bool MagBmm350::LoadOtp()
{
	uint8_t i;

	for (i = 0; i < sizeof(vOtp) / sizeof(vOtp[0]); i++)
	{
		if (!ReadOtpWord(i, vOtp[i]))
		{
			return false;
		}
	}

	return WriteReg(BMM350_OTP_CMD_REG, BMM350_OTP_POWER_OFF);
}

void MagBmm350::ParseOtp()
{
	uint16_t value;
	uint8_t b;

	value = vOtp[BMM350_MAG_OFFSET_X] & 0x0FFFU;
	vOffset[0] = (float)SignExtend(value, 12);
	value = ((vOtp[BMM350_MAG_OFFSET_X] & 0xF000U) >> 4) |
		(vOtp[BMM350_MAG_OFFSET_Y] & 0x00FFU);
	vOffset[1] = (float)SignExtend(value, 12);
	value = (vOtp[BMM350_MAG_OFFSET_Y] & 0x0F00U) |
		(vOtp[BMM350_MAG_OFFSET_Z] & 0x00FFU);
	vOffset[2] = (float)SignExtend(value, 12);

	b = (uint8_t)(vOtp[BMM350_TEMP_OFF_SENS] & 0xFFU);
	vOffset[3] = (float)(int8_t)b / 5.0f;

	b = (uint8_t)(vOtp[BMM350_MAG_SENS_X] >> 8);
	vSens[0] = (float)(int8_t)b / 256.0f;
	b = (uint8_t)(vOtp[BMM350_MAG_SENS_Y] & 0xFFU);
	vSens[1] = (float)(int8_t)b / 256.0f;
	b = (uint8_t)(vOtp[BMM350_MAG_SENS_Z] >> 8);
	vSens[2] = (float)(int8_t)b / 256.0f;
	b = (uint8_t)(vOtp[BMM350_TEMP_OFF_SENS] >> 8);
	vSens[3] = (float)(int8_t)b / 512.0f;

	b = (uint8_t)(vOtp[BMM350_MAG_TCO_X] & 0xFFU);
	vTco[0] = (float)(int8_t)b / 32.0f;
	b = (uint8_t)(vOtp[BMM350_MAG_TCO_Y] & 0xFFU);
	vTco[1] = (float)(int8_t)b / 32.0f;
	b = (uint8_t)(vOtp[BMM350_MAG_TCO_Z] & 0xFFU);
	vTco[2] = (float)(int8_t)b / 32.0f;

	b = (uint8_t)(vOtp[BMM350_MAG_TCS_X] >> 8);
	vTcs[0] = (float)(int8_t)b / 16384.0f;
	b = (uint8_t)(vOtp[BMM350_MAG_TCS_Y] >> 8);
	vTcs[1] = (float)(int8_t)b / 16384.0f;
	b = (uint8_t)(vOtp[BMM350_MAG_TCS_Z] >> 8);
	vTcs[2] = (float)(int8_t)b / 16384.0f;

	vT0 = (float)(int16_t)vOtp[BMM350_MAG_DUT_T_0] / 512.0f + 23.0f;

	b = (uint8_t)(vOtp[BMM350_CROSS_X_Y] & 0xFFU);
	vCross[0] = (float)(int8_t)b / 800.0f;
	b = (uint8_t)(vOtp[BMM350_CROSS_Y_X] >> 8);
	vCross[1] = (float)(int8_t)b / 800.0f;
	b = (uint8_t)(vOtp[BMM350_CROSS_Z_X] & 0xFFU);
	vCross[2] = (float)(int8_t)b / 800.0f;
	b = (uint8_t)(vOtp[BMM350_CROSS_Z_Y] >> 8);
	vCross[3] = (float)(int8_t)b / 800.0f;
}

bool MagBmm350::MagneticReset()
{
	uint8_t status;
	uint8_t value;

	if (!WriteReg(BMM350_PMU_CMD_REG, BMM350_PMU_CMD_BR))
	{
		return false;
	}
	usDelay(BMM350_BR_DELAY_US);
	if (!ReadReg(BMM350_PMU_CMD_STATUS_0_REG, &status, 1))
	{
		return false;
	}
	value = (status & BMM350_PMU_CMD_VALUE_MASK) >>
		BMM350_PMU_CMD_VALUE_SHIFT;
	if (value != BMM350_PMU_CMD_BR)
	{
		return false;
	}

	if (!WriteReg(BMM350_PMU_CMD_REG, BMM350_PMU_CMD_FGR))
	{
		return false;
	}
	usDelay(BMM350_FGR_DELAY_US);
	if (!ReadReg(BMM350_PMU_CMD_STATUS_0_REG, &status, 1))
	{
		return false;
	}
	value = (status & BMM350_PMU_CMD_VALUE_MASK) >>
		BMM350_PMU_CMD_VALUE_SHIFT;

	return value == BMM350_PMU_CMD_FGR;
}

bool MagBmm350::Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf,
	Timer * const pTimer)
{
	uint8_t id;
	uint32_t address;

	vbInitialized = false;
	if (pIntrf == NULL)
	{
		return false;
	}

	MagSensor::Type(SENSOR_TYPE_MAG);
	Interface(pIntrf);
	vpTimer = pTimer;

	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		DeviceAddress(Cfg.DevAddr);
	}
	else if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		address = Cfg.DevAddr == BMM350_I2C_7BITS_DEVADDR2 ?
			BMM350_I2C_7BITS_DEVADDR2 : BMM350_I2C_7BITS_DEVADDR;
		DeviceAddress(address);
	}
	else
	{
		return false;
	}

	usDelay(BMM350_STARTUP_DELAY_US);
	if (!WriteReg(BMM350_CMD_REG, BMM350_CMD_SOFTRESET))
	{
		return false;
	}
	usDelay(BMM350_SOFTRESET_DELAY_US);

	if (!ReadReg(BMM350_CHIP_ID_REG, &id, 1) || id != BMM350_CHIP_ID)
	{
		if (pIntrf->Type() != DEVINTRF_TYPE_I2C ||
			DeviceAddress() == BMM350_I2C_7BITS_DEVADDR2)
		{
			return false;
		}

		DeviceAddress(BMM350_I2C_7BITS_DEVADDR2);
		if (!ReadReg(BMM350_CHIP_ID_REG, &id, 1) ||
			id != BMM350_CHIP_ID)
		{
			return false;
		}
	}

	if (!LoadOtp())
	{
		return false;
	}
	ParseOtp();

	if (!MagneticReset())
	{
		return false;
	}

	vPrecision = MAGSENSOR_PRECISION_HIGH;
	vSensitivity[0] = 1;
	vSensitivity[1] = 1;
	vSensitivity[2] = 1;
	vData.Sensitivity[0] = 1;
	vData.Sensitivity[1] = 1;
	vData.Sensitivity[2] = 1;
	vRange = BMM350_FLUX_DENSITY;
	ClearCalibration();
	DeviceID(BMM350_CHIP_ID);
	vbInitialized = true;

	if (SamplingFrequency(Cfg.Freq) == 0 || !Enable())
	{
		vbInitialized = false;
		return false;
	}

	return true;
}

uint32_t MagBmm350::SamplingFrequency(uint32_t Freq)
{
	uint8_t odr;
	uint32_t actual;

	if (!vbInitialized)
	{
		return 0;
	}

	if (Freq < 2000U) { odr = BMM350_PMU_CMD_AGGR_SET_ODR_1_5625HZ; actual = 1562U; }
	else if (Freq < 6000U) { odr = BMM350_PMU_CMD_AGGR_SET_ODR_3_125HZ; actual = 3125U; }
	else if (Freq < 10000U) { odr = BMM350_PMU_CMD_AGGR_SET_ODR_6_25HZ; actual = 6250U; }
	else if (Freq < 17000U) { odr = BMM350_PMU_CMD_AGGR_SET_ODR_12_5HZ; actual = 12500U; }
	else if (Freq < 35000U) { odr = BMM350_PMU_CMD_AGGR_SET_ODR_25HZ; actual = 25000U; }
	else if (Freq < 80000U) { odr = BMM350_PMU_CMD_AGGR_SET_ODR_50HZ; actual = 50000U; }
	else if (Freq < 150000U) { odr = BMM350_PMU_CMD_AGGR_SET_ODR_100HZ; actual = 100000U; }
	else if (Freq < 300000U) { odr = BMM350_PMU_CMD_AGGR_SET_ODR_200HZ; actual = 200000U; }
	else { odr = BMM350_PMU_CMD_AGGR_SET_ODR_400HZ; actual = 400000U; }

	if (!WriteReg(BMM350_PMU_CMD_AGGR_SET_REG, odr) ||
		!WriteReg(BMM350_PMU_CMD_REG, BMM350_PMU_CMD_UPD_OAE))
	{
		return 0;
	}

	usDelay(BMM350_UPD_OAE_DELAY_US);
	return Sensor::SamplingFrequency(actual);
}

bool MagBmm350::SetPowerMode(uint8_t Mode)
{
	uint8_t current;

	if (!vbInitialized || Mode > BMM350_PMU_CMD_FM_FAST ||
		!ReadReg(BMM350_PMU_CMD_REG, &current, 1))
	{
		return false;
	}

	current &= BMM350_PMU_CMD_MASK;
	if (current == BMM350_PMU_CMD_NM && Mode != BMM350_PMU_CMD_NM)
	{
		if (!WriteReg(BMM350_PMU_CMD_REG, BMM350_PMU_CMD_SUSP))
		{
			return false;
		}
		usDelay(BMM350_GOTO_SUSPEND_DELAY_US);
	}

	if (!WriteReg(BMM350_PMU_CMD_REG, Mode))
	{
		return false;
	}

	if (Mode == BMM350_PMU_CMD_NM)
	{
		usDelay(BMM350_SUSPEND_NORMAL_DELAY_US);
	}

	return true;
}

bool MagBmm350::Enable()
{
	if (!vbInitialized ||
		!WriteReg(BMM350_PMU_CMD_AXIS_EN_REG,
			BMM350_PMU_CMD_AXIS_EN_X | BMM350_PMU_CMD_AXIS_EN_Y |
			BMM350_PMU_CMD_AXIS_EN_Z))
	{
		return false;
	}

	return SetPowerMode(BMM350_PMU_CMD_NM);
}

void MagBmm350::Disable()
{
	if (!vbInitialized)
	{
		return;
	}

	WriteReg(BMM350_PMU_CMD_AXIS_EN_REG, 0);
	SetPowerMode(BMM350_PMU_CMD_SUSP);
}

void MagBmm350::Reset()
{
	if (Interface() != NULL)
	{
		WriteReg(BMM350_CMD_REG, BMM350_CMD_SOFTRESET);
		usDelay(BMM350_SOFTRESET_DELAY_US);
	}

	vbInitialized = false;
}

void MagBmm350::Compensate(int32_t X, int32_t Y, int32_t Z, int32_t Temp)
{
	float magnetic[3];
	float temperature;
	float denom;
	float x;
	float y;
	float z;
	int i;
	const float bxySens = 14.55f;
	const float bzSens = 9.0f;
	const float tempSens = 0.00204f;
	const float inaXyGain = 19.46f;
	const float inaZGain = 31.0f;
	const float adcGain = 1.0f / 1.5f;
	const float lutGain = 0.714607238769531f;
	const float power = 1000000.0f / 1048576.0f;

	magnetic[0] = (float)X * (power / (bxySens * inaXyGain * adcGain * lutGain));
	magnetic[1] = (float)Y * (power / (bxySens * inaXyGain * adcGain * lutGain));
	magnetic[2] = (float)Z * (power / (bzSens * inaZGain * adcGain * lutGain));
	temperature = (float)Temp /
		(tempSens * adcGain * lutGain * 1048576.0f) - 25.49f;
	temperature = (1.0f + vSens[3]) * temperature + vOffset[3];

	for (i = 0; i < 3; i++)
	{
		magnetic[i] *= 1.0f + vSens[i];
		magnetic[i] += vOffset[i];
		magnetic[i] += vTco[i] * (temperature - vT0);
		magnetic[i] /= 1.0f + vTcs[i] * (temperature - vT0);
	}

	denom = 1.0f - vCross[1] * vCross[0];
	if (fabsf(denom) < 0.000001f)
	{
		return;
	}

	x = (magnetic[0] - vCross[0] * magnetic[1]) / denom;
	y = (magnetic[1] - vCross[1] * magnetic[0]) / denom;
	z = magnetic[2] +
		(magnetic[0] * (vCross[1] * vCross[3] - vCross[2]) -
		magnetic[1] * (vCross[3] - vCross[0] * vCross[2])) / denom;

	vData.X = (int32_t)lroundf(x * 1000.0f);
	vData.Y = (int32_t)lroundf(y * 1000.0f);
	vData.Z = (int32_t)lroundf(z * 1000.0f);
	vData.Temp = (int16_t)lroundf(temperature * 100.0f);
}

bool MagBmm350::UpdateData()
{
	uint8_t status;
	uint8_t raw[12];
	uint32_t value;
	int32_t x;
	int32_t y;
	int32_t z;
	int32_t temp;

	if (!vbInitialized ||
		!ReadReg(BMM350_INT_STATUS_REG, &status, 1) ||
		(status & BMM350_INT_STATUS_DRDY) == 0 ||
		!ReadReg(BMM350_MAG_X_XLSB_REG, raw, sizeof(raw)))
	{
		return false;
	}

	value = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16);
	x = SignExtend(value, 24);
	value = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8) | ((uint32_t)raw[5] << 16);
	y = SignExtend(value, 24);
	value = (uint32_t)raw[6] | ((uint32_t)raw[7] << 8) | ((uint32_t)raw[8] << 16);
	z = SignExtend(value, 24);
	value = (uint32_t)raw[9] | ((uint32_t)raw[10] << 8) | ((uint32_t)raw[11] << 16);
	temp = SignExtend(value, 24);

	Compensate(x, y, z, temp);
	vData.Timestamp = vpTimer != NULL ? vpTimer->uSecond() : vData.Timestamp + 1U;
	return true;
}

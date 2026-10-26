/**-------------------------------------------------------------------------
@file	bmm350_test.cpp

@brief	Host test support: bmm350 test.

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

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensors/mag_bmm350.h"

uint64_t g_DelayUs;

class MockIntrf : public DeviceIntrf {
public:
	MockIntrf()
	{
		memset(vReg, 0, sizeof(vReg));
		memset(vOtp, 0, sizeof(vOtp));
		vReg[BMM350_CHIP_ID_REG] = BMM350_CHIP_ID;
		vType = DEVINTRF_TYPE_I2C;
		vRate = 100000;
		vOtpReads = 0;
		vFailOtp = false;
	}
	operator DevIntrf_t * () override
	{
		return NULL;
	}
	DEVINTRF_TYPE Type() override
	{
		return vType;
	}
	uint32_t Rate(uint32_t Rate) override
	{
		vRate = Rate;
		return Rate;
	}
	uint32_t Rate(void) override
	{
		return vRate;
	}
	void Disable(void) override
	{
	}
	void Enable(void) override
	{
	}
	int Rx(uint32_t, uint8_t *, int) override
	{
		return 0;
	}
	int Tx(uint32_t, const uint8_t *, int) override
	{
		return 0;
	}
	bool StartRx(uint32_t) override
	{
		return false;
	}
	int RxData(uint8_t *, int) override
	{
		return 0;
	}
	void StopRx(void) override
	{
	}
	bool StartTx(uint32_t) override
	{
		return false;
	}
	int TxData(const uint8_t *, int) override
	{
		return 0;
	}
	void StopTx(void) override
	{
	}
	bool RequestToSend(int) override
	{
		return true;
	}
	void Reset(void) override
	{
	}
	int Read(uint32_t, const uint8_t *pCmd, int CmdLen,
		uint8_t *pData, int Len) override
	{
		if (pCmd == NULL || CmdLen != 1 || pData == NULL || Len < 1)
			return 0;
		for (int i = 0; i < Len; i++)
			pData[i] = vReg[(uint8_t)(pCmd[0] + i)];
		return Len;
	}
	int Write(uint32_t, const uint8_t *pCmd, int CmdLen,
		const uint8_t *pData, int Len) override
	{
		uint8_t reg;
		uint8_t cmd;
		uint8_t addr;
		if (pCmd == NULL || CmdLen != 1 || pData == NULL || Len != 1)
			return 0;
		reg = pCmd[0];
		vReg[reg] = pData[0];
		if (reg == BMM350_OTP_CMD_REG)
		{
			cmd = pData[0] & BMM350_OTP_CMD_MASK;
			if (cmd == BMM350_OTP_CMD_DIR_READ)
			{
				addr = pData[0] & BMM350_OTP_CMD_WORD_ADDR_MASK;
				vOtpReads++;
				vReg[BMM350_OTP_STATUS_REG] = vFailOtp ?
					BMM350_OTP_STATUS_ERR_PAGE_RD : BMM350_OTP_STATUS_CMD_DONE;
				vReg[BMM350_OTP_DATA_MSB_REG] = (uint8_t)(vOtp[addr] >> 8);
				vReg[BMM350_OTP_DATA_LSB_REG] = (uint8_t)vOtp[addr];
			}
		}
		else if (reg == BMM350_PMU_CMD_REG)
		{
			vReg[BMM350_PMU_CMD_STATUS_0_REG] = (uint8_t)((pData[0] & 7U) << 5);
		}
		return Len;
	}

	uint8_t vReg[256];
	uint16_t vOtp[32];
	DEVINTRF_TYPE vType;
	uint32_t vRate;
	unsigned vOtpReads;
	bool vFailOtp;
};

static MagSensorCfg_t Config()
{
	MagSensorCfg_t cfg = {};
	cfg.DevAddr = BMM350_I2C_7BITS_DEVADDR;
	cfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	cfg.Freq = 100000;
	cfg.Precision = MAGSENSOR_PRECISION_HIGH;
	return cfg;
}

static void Put24(uint8_t *p, int32_t value)
{
	uint32_t v;
	v = (uint32_t)value & 0xFFFFFFU;
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
}

static void TestInitAndRate()
{
	MockIntrf bus;
	MagBmm350 mag;
	MagSensorCfg_t cfg = Config();

	assert(mag.Init(cfg, &bus, NULL));
	assert(bus.vOtpReads == 32);
	assert(bus.vReg[BMM350_OTP_CMD_REG] == BMM350_OTP_CMD_DIR_PWR_OFF_OTP);
	assert((bus.vReg[BMM350_PMU_CMD_AGGR_SET_REG] &
		BMM350_PMU_CMD_AGGR_SET_ODR_MASK) == BMM350_PMU_CMD_AGGR_SET_ODR_100HZ);
	assert(mag.SamplingFrequency(200000) == 200000);
	assert((bus.vReg[BMM350_PMU_CMD_AGGR_SET_REG] & 0x0F) == 3);
	mag.Disable();
	assert(bus.vReg[BMM350_PMU_CMD_AXIS_EN_REG] == 0);
	assert(bus.vReg[BMM350_PMU_CMD_REG] == BMM350_PMU_CMD_SUSP);
	assert(mag.Enable());
}

static void TestData()
{
	MockIntrf bus;
	MagBmm350 mag;
	MagSensorCfg_t cfg = Config();
	MagSensorRawData_t raw;
	MagSensorData_t data;
	uint8_t values[12];

	assert(mag.Init(cfg, &bus, NULL));
	memset(values, 0, sizeof(values));
	Put24(values + 0, 100000);
	Put24(values + 3, -200000);
	Put24(values + 6, 300000);
	Put24(values + 9, 0);
	memcpy(&bus.vReg[BMM350_MAG_X_XLSB_REG], values, sizeof(values));
	bus.vReg[BMM350_INT_STATUS_REG] = BMM350_INT_STATUS_DRDY;
	assert(mag.UpdateData());
	assert(mag.Read(raw));
	assert(raw.X > 0);
	assert(raw.Y < 0);
	assert(raw.Z > 0);
	assert(raw.Timestamp == 1);
	assert(mag.Read(data));
	assert(fabsf(data.X - (float)raw.X / 1000.0f) < 0.01f);
	assert(fabsf(data.Y - (float)raw.Y / 1000.0f) < 0.01f);
	bus.vReg[BMM350_INT_STATUS_REG] = 0;
	assert(!mag.UpdateData());
}

static void TestFailures()
{
	MockIntrf bus;
	MagBmm350 mag;
	MagSensorCfg_t cfg = Config();

	assert(!mag.Init(cfg, NULL, NULL));
	bus.vType = DEVINTRF_TYPE_UART;
	assert(!mag.Init(cfg, &bus, NULL));
	{
		MockIntrf badId;
		badId.vReg[BMM350_CHIP_ID_REG] = 0;
		assert(!mag.Init(cfg, &badId, NULL));
	}
	{
		MockIntrf badOtp;
		badOtp.vFailOtp = true;
		assert(!mag.Init(cfg, &badOtp, NULL));
	}
}

int main()
{
	static_assert(BMM350_PMU_CMD_AGGR_SET_ODR_400HZ == 2, "400 Hz code");
	static_assert(BMM350_PMU_CMD_AGGR_SET_ODR_200HZ == 3, "200 Hz code");
	static_assert(BMM350_PMU_CMD_AGGR_SET_ODR_100HZ == 4, "100 Hz code");
	static_assert(BMM350_PMU_CMD_AGGR_SET_ODR_1_5625HZ == 10, "1.5625 Hz code");
	TestInitAndRate();
	TestData();
	TestFailures();
	puts("BMM350 production-driver tests: PASS");
	return 0;
}

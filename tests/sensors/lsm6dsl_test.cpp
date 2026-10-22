/**-------------------------------------------------------------------------
@file	lsm6dsl_test.cpp

@brief	Host test of the LSM6DSL accel gyro driver.

Runs src/sensors/ag_lsm6dsl.cpp against a register model of the device on an
I2C or SPI interface: detection, reset, configuration registers, rate and
scale selection through the base classes, data conversion and the data ready
event.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

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
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "sensors/ag_lsm6dsl.h"

// Register model of the LSM6DSL behind an I2C or SPI interface
class Lsm6dslModel : public DeviceIntrf {
public:
	Lsm6dslModel(DEVINTRF_TYPE Type, uint32_t Addr) : vType(Type), vAddr(Addr), vDev{} {
		PowerOn();
	}
	void PowerOn() {
		memset(Reg, 0, sizeof(Reg));
		Reg[LSM6DSL_WHO_AM_I_REG] = LSM6DSL_WHO_AM_I_ID;
		Reg[LSM6DSL_CTRL3_C_REG] = LSM6DSL_CTRL3_C_IF_INC;
		ResetReads = 0;
	}
	operator DevIntrf_t * () { return &vDev; }
	DEVINTRF_TYPE Type() { return vType; }
	uint32_t Rate(uint32_t DataRate) { return DataRate; }
	uint32_t Rate(void) { return 400000; }

	int Read(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen, uint8_t *pBuff, int BuffLen) {
		assert(AdCmdLen == 1);
		if (DevAddr != vAddr)
		{
			return 0;
		}
		uint8_t addr = pAdCmd[0];
		if (vType == DEVINTRF_TYPE_SPI)
		{
			assert(addr & 0x80);
			addr &= 0x7F;
		}
		LastReadAddr = addr;
		LastReadLen = BuffLen;
		for (int i = 0; i < BuffLen; i++)
		{
			uint8_t a = (Reg[LSM6DSL_CTRL3_C_REG] & LSM6DSL_CTRL3_C_IF_INC) ? addr + i : addr;
			pBuff[i] = Reg[a];
			if (a == LSM6DSL_CTRL3_C_REG && ResetReads > 0)
			{
				pBuff[i] |= LSM6DSL_CTRL3_C_SW_RESET;
				if (ResetReads < 1000)
				{
					ResetReads--;
				}
			}
		}
		return BuffLen;
	}

	int Write(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen, const uint8_t *pData, int DataLen) {
		assert(AdCmdLen == 1);
		if (DevAddr != vAddr)
		{
			return 0;
		}
		uint8_t addr = pAdCmd[0];
		if (vType == DEVINTRF_TYPE_SPI)
		{
			assert((addr & 0x80) == 0);
		}
		for (int i = 0; i < DataLen; i++)
		{
			uint8_t a = (Reg[LSM6DSL_CTRL3_C_REG] & LSM6DSL_CTRL3_C_IF_INC) ? addr + i : addr;
			assert(a != LSM6DSL_WHO_AM_I_REG && a != LSM6DSL_STATUS_REG);
			if (a == LSM6DSL_CTRL3_C_REG && (pData[i] & LSM6DSL_CTRL3_C_SW_RESET))
			{
				int pending = StuckReset ? 1000 : 2;
				PowerOn();
				ResetReads = pending;
				Resets++;
				continue;
			}
			Reg[a] = pData[i];
			Writes++;
		}
		return DataLen;
	}

	void Set16(uint8_t Addr, int16_t Val) {
		Reg[Addr] = (uint8_t)Val;
		Reg[Addr + 1] = (uint8_t)((uint16_t)Val >> 8);
	}

	uint8_t Reg[128];
	int ResetReads;
	int Resets = 0;
	int Writes = 0;
	bool StuckReset = false;
	uint8_t LastReadAddr = 0;
	int LastReadLen = 0;

private:
	DEVINTRF_TYPE vType;
	uint32_t vAddr;
	DevIntrf_t vDev;
};

static int s_Fail = 0;
static int s_Checks = 0;

#define CHECK(c)	do { s_Checks++; if (!(c)) { s_Fail++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static bool Near(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

static int s_Events = 0;

static void EvtHandler(Device * const pDev, DEV_EVT Evt)
{
	(void)pDev;
	if (Evt == DEV_EVT_DATA_RDY)
	{
		s_Events++;
	}
}

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Scale = 4,
	.FltrFreq = 0,
	.Inter = 1,
	.IntPol = DEVINTR_POL_LOW,
	.IntHandler = NULL,
	.bFifoEn = false,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 100000,
	.Sensitivity = 0,
	.FltrFreq = 0,
	.Inter = 1,
	.IntPol = DEVINTR_POL_LOW,
	.bFifoEn = false,
};

static void TestI2c(void)
{
	// SA0 high, the driver probes 0x6A then 0x6B
	Lsm6dslModel m(DEVINTRF_TYPE_I2C, LSM6DSL_I2C_DEV_ADDR1);
	AgLsm6dsl ag;

	CHECK(ag.Init(s_AccelCfg, &m, NULL) == true);
	CHECK(ag.DeviceAddress() == LSM6DSL_I2C_DEV_ADDR1);
	CHECK(m.Resets == 1);
	CHECK(ag.Init(s_GyroCfg, &m, NULL) == true);
	CHECK(m.Resets == 1);

	// Configuration, the sensors stay powered down until Enable
	CHECK(m.Reg[LSM6DSL_CTRL3_C_REG] == (LSM6DSL_CTRL3_C_BDU | LSM6DSL_CTRL3_C_IF_INC | LSM6DSL_CTRL3_C_H_LACTIVE));
	CHECK(m.Reg[LSM6DSL_CTRL4_C_REG] == LSM6DSL_CTRL4_C_DRDY_MASK);
	CHECK(m.Reg[LSM6DSL_DRDY_PULSE_CFG_G_REG] == LSM6DSL_DRDY_PULSE_CFG_G_DRDY_PULSED);
	CHECK(m.Reg[LSM6DSL_INT1_CTRL_REG] == (LSM6DSL_INT1_CTRL_DRDY_XL | LSM6DSL_INT1_CTRL_DRDY_G));
	CHECK(m.Reg[LSM6DSL_INT2_CTRL_REG] == 0);
	CHECK(m.Reg[LSM6DSL_CTRL1_XL_REG] == LSM6DSL_CTRL1_XL_FS_XL_4G);
	CHECK(m.Reg[LSM6DSL_CTRL2_G_REG] == LSM6DSL_CTRL2_G_FS_125);

	AccelSensor *pa = &ag;
	GyroSensor *pg = &ag;

	CHECK(pa->SamplingFrequency() == 52000);
	CHECK(pa->Scale() == 4);
	CHECK(pg->SamplingFrequency() == 104000);
	CHECK(pg->Sensitivity() == 125);

	CHECK(ag.Enable() == true);
	CHECK(m.Reg[LSM6DSL_CTRL1_XL_REG] == ((LSM6DSL_ODR_52HZ << 4) | LSM6DSL_CTRL1_XL_FS_XL_4G));
	CHECK(m.Reg[LSM6DSL_CTRL2_G_REG] == ((LSM6DSL_ODR_104HZ << 4) | LSM6DSL_CTRL2_G_FS_125));

	// Data, 4 g and 125 dps
	ag.SetEvtHandler(EvtHandler);
	m.Set16(LSM6DSL_OUT_TEMP_L_REG, 256);
	m.Set16(LSM6DSL_OUTX_L_G_REG, 1000);
	m.Set16(LSM6DSL_OUTX_L_G_REG + 2, -1000);
	m.Set16(LSM6DSL_OUTX_L_G_REG + 4, 28571);
	m.Set16(LSM6DSL_OUTX_L_XL_REG, 8192);
	m.Set16(LSM6DSL_OUTX_L_XL_REG + 2, -16384);
	m.Set16(LSM6DSL_OUTX_L_XL_REG + 4, 32767);
	m.Reg[LSM6DSL_STATUS_REG] = LSM6DSL_STATUS_XLDA | LSM6DSL_STATUS_GDA | LSM6DSL_STATUS_TDA;

	ag.IntHandler();
	CHECK(s_Events == 1);
	CHECK(m.LastReadAddr == LSM6DSL_OUT_TEMP_L_REG && m.LastReadLen == 14);

	AccelSensorData_t acc;
	GyroSensorData_t gyro;
	AccelSensorRawData_t araw;
	TempSensorData_t temp;

	CHECK(ag.Read(acc) == true);
	CHECK(Near(acc.X, 1.0f, 1e-5f));
	CHECK(Near(acc.Y, -2.0f, 1e-5f));
	CHECK(Near(acc.Z, 4.0f * 32767 / 32768, 1e-5f));
	CHECK(Near(acc.Temp, 26.0f, 1e-5f));
	CHECK(ag.Read(gyro) == true);
	CHECK(Near(gyro.X, 4.375f, 1e-3f));
	CHECK(Near(gyro.Y, -4.375f, 1e-3f));
	CHECK(Near(gyro.Z, 125.0f, 1e-3f));
	ag.Read(temp);
	CHECK(temp.Temperature == 2600);

	// No new data
	m.Reg[LSM6DSL_STATUS_REG] = 0;
	m.Set16(LSM6DSL_OUTX_L_XL_REG, 0);
	CHECK(ag.UpdateData() == false);
	ag.IntHandler();
	CHECK(s_Events == 1);
	CHECK(ag.Read(araw) == true && araw.X == 8192);

	// Accelerometer only, below 0 C
	m.Set16(LSM6DSL_OUT_TEMP_L_REG, -256 * 30);
	m.Set16(LSM6DSL_OUTX_L_G_REG, 0);
	m.Set16(LSM6DSL_OUTX_L_XL_REG, -8192);
	m.Reg[LSM6DSL_STATUS_REG] = LSM6DSL_STATUS_XLDA;
	CHECK(ag.UpdateData() == true);
	CHECK(ag.Read(acc) == true && Near(acc.X, -1.0f, 1e-5f) && Near(acc.Temp, -5.0f, 1e-5f));
	CHECK(ag.Read(gyro) == true && Near(gyro.X, 4.375f, 1e-3f));
	ag.Read(temp);
	CHECK(temp.Temperature == 2600);

	// Rate and scale through the base classes while running
	CHECK(pa->Scale(16) == 16);
	CHECK((m.Reg[LSM6DSL_CTRL1_XL_REG] & LSM6DSL_CTRL1_XL_FS_XL_MASK) == LSM6DSL_CTRL1_XL_FS_XL_16G);
	CHECK(ag.Read(acc) == true && Near(acc.X, -4.0f, 1e-5f));
	CHECK(pa->Scale(3) == 4);
	CHECK((m.Reg[LSM6DSL_CTRL1_XL_REG] & LSM6DSL_CTRL1_XL_FS_XL_MASK) == LSM6DSL_CTRL1_XL_FS_XL_4G);
	CHECK(pa->SamplingFrequency(1000000) == 1660000);
	CHECK((m.Reg[LSM6DSL_CTRL1_XL_REG] >> 4) == LSM6DSL_ODR_1660HZ);
	CHECK(pa->SamplingFrequency(0) == 12500);
	CHECK((m.Reg[LSM6DSL_CTRL1_XL_REG] >> 4) == LSM6DSL_ODR_12_5HZ);
	CHECK(pa->SamplingFrequency(10000000) == 6660000);
	CHECK((m.Reg[LSM6DSL_CTRL1_XL_REG] >> 4) == LSM6DSL_ODR_6660HZ);
	CHECK(pg->Sensitivity(300) == 500);
	CHECK((m.Reg[LSM6DSL_CTRL2_G_REG] & 0x0E) == LSM6DSL_CTRL2_G_FS_G_500);
	CHECK(pg->Sensitivity(2000) == 2000);
	CHECK((m.Reg[LSM6DSL_CTRL2_G_REG] & 0x0E) == LSM6DSL_CTRL2_G_FS_G_2000);
	CHECK(pg->Sensitivity(250) == 250);
	CHECK((m.Reg[LSM6DSL_CTRL2_G_REG] & 0x0E) == LSM6DSL_CTRL2_G_FS_G_250);
	CHECK(pg->Sensitivity(1000) == 1000);
	CHECK((m.Reg[LSM6DSL_CTRL2_G_REG] & 0x0E) == LSM6DSL_CTRL2_G_FS_G_1000);
	m.Set16(LSM6DSL_OUTX_L_G_REG, 1000);
	m.Reg[LSM6DSL_STATUS_REG] = LSM6DSL_STATUS_GDA;
	CHECK(ag.UpdateData() == true);
	CHECK(ag.Read(gyro) == true && Near(gyro.X, 35.0f, 1e-2f));
	CHECK(pg->SamplingFrequency(30000) == 52000);
	CHECK((m.Reg[LSM6DSL_CTRL2_G_REG] >> 4) == LSM6DSL_ODR_52HZ);

	// Disable keeps the scales
	ag.Disable();
	CHECK(m.Reg[LSM6DSL_CTRL1_XL_REG] == LSM6DSL_CTRL1_XL_FS_XL_4G);
	CHECK(m.Reg[LSM6DSL_CTRL2_G_REG] == LSM6DSL_CTRL2_G_FS_G_1000);

	// A rate change while powered down waits for Enable
	CHECK(pa->SamplingFrequency(200000) == 208000);
	CHECK(m.Reg[LSM6DSL_CTRL1_XL_REG] == LSM6DSL_CTRL1_XL_FS_XL_4G);
	CHECK(ag.Enable() == true);
	CHECK((m.Reg[LSM6DSL_CTRL1_XL_REG] >> 4) == LSM6DSL_ODR_208HZ);
	CHECK((m.Reg[LSM6DSL_CTRL2_G_REG] >> 4) == LSM6DSL_ODR_52HZ);
}

static void TestNotFound(void)
{
	Lsm6dslModel m(DEVINTRF_TYPE_I2C, 0x6C);
	AgLsm6dsl ag;

	CHECK(ag.Init(s_AccelCfg, &m, NULL) == false);
	CHECK(ag.Valid() == false);

	// Wrong identification at the given address
	Lsm6dslModel w(DEVINTRF_TYPE_I2C, LSM6DSL_I2C_DEV_ADDR0);
	AccelSensorCfg_t cfg = s_AccelCfg;

	w.Reg[LSM6DSL_WHO_AM_I_REG] = 0x6C;
	cfg.DevAddr = LSM6DSL_I2C_DEV_ADDR0;
	CHECK(ag.Init(cfg, &w, NULL) == false);
	CHECK(w.Writes == 0 && w.Resets == 0);

	// Reset that does not end
	Lsm6dslModel s(DEVINTRF_TYPE_I2C, LSM6DSL_I2C_DEV_ADDR0);

	s.StuckReset = true;
	CHECK(ag.Init(cfg, &s, NULL) == false);
	CHECK(ag.Valid() == false);
}

static void TestSpiActiveHigh(void)
{
	Lsm6dslModel m(DEVINTRF_TYPE_SPI, 0);
	AgLsm6dsl ag;
	AccelSensorCfg_t cfg = s_AccelCfg;

	cfg.Inter = 2;
	cfg.IntPol = DEVINTR_POL_HIGH;
	cfg.Scale = 8;
	cfg.Freq = 400000;
	CHECK(ag.Init(cfg, &m, NULL) == true);
	CHECK(m.Reg[LSM6DSL_CTRL3_C_REG] == (LSM6DSL_CTRL3_C_BDU | LSM6DSL_CTRL3_C_IF_INC));
	CHECK(m.Reg[LSM6DSL_INT1_CTRL_REG] == 0);
	CHECK(m.Reg[LSM6DSL_INT2_CTRL_REG] == (LSM6DSL_INT2_CTRL_DRDY_XL | LSM6DSL_INT2_CTRL_DRDY_G));
	CHECK(((AccelSensor&)ag).SamplingFrequency() == 416000);
	CHECK(ag.Enable() == true);
	CHECK(m.Reg[LSM6DSL_CTRL1_XL_REG] == ((LSM6DSL_ODR_416HZ << 4) | LSM6DSL_CTRL1_XL_FS_XL_8G));
	// The gyroscope was not initialized, its rate stays at power down
	CHECK((m.Reg[LSM6DSL_CTRL2_G_REG] >> 4) == LSM6DSL_ODR_POWER_DOWN);

	m.Set16(LSM6DSL_OUTX_L_XL_REG, 4096);
	m.Reg[LSM6DSL_STATUS_REG] = LSM6DSL_STATUS_XLDA;
	AccelSensorData_t acc;

	CHECK(ag.UpdateData() == true);
	CHECK(ag.Read(acc) == true && Near(acc.X, 1.0f, 1e-5f));

	// No interrupt requested
	Lsm6dslModel n(DEVINTRF_TYPE_SPI, 0);
	AgLsm6dsl ag2;

	cfg.Inter = 0;
	CHECK(ag2.Init(cfg, &n, NULL) == true);
	CHECK(n.Reg[LSM6DSL_DRDY_PULSE_CFG_G_REG] == 0);
	CHECK(n.Reg[LSM6DSL_INT1_CTRL_REG] == 0 && n.Reg[LSM6DSL_INT2_CTRL_REG] == 0);
}

int main()
{
	TestI2c();
	TestNotFound();
	TestSpiActiveHigh();

	printf("LSM6DSL driver model: %d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}

/**-------------------------------------------------------------------------
@file	lis3mdl_test.cpp

@brief	Host test of the LIS3MDL magnetometer driver.

Runs src/sensors/mag_lis3mdl.cpp against a register model of the device on an
I2C or SPI interface: detection, software reset, operating mode, rate, full
scale, calibration, single and continuous conversion, data conversion.

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

#include "sensors/mag_lis3mdl.h"

#define LIS3MDL_STATUS_NEW		(LIS3MDL_STATUS_XDA | LIS3MDL_STATUS_YDA | LIS3MDL_STATUS_ZDA | LIS3MDL_STATUS_ZYXDA)

// Register model of the LIS3MDL behind an I2C or SPI interface
class Lis3mdlModel : public DeviceIntrf {
public:
	Lis3mdlModel(DEVINTRF_TYPE Type, uint8_t I2cAddr = LIS3MDL_I2C_DEV_ADDR1) :
		vType(Type), vI2cAddr(I2cAddr), vDev{} {
		memset(Reg, 0, sizeof(Reg));
		Reg[LIS3MDL_WHO_AM_I_REG] = LIS3MDL_WHO_AM_I_ID;
		Defaults();
	}
	void Defaults() {
		Reg[LIS3MDL_CTRL_REG1_REG] = 0x10;
		Reg[LIS3MDL_CTRL_REG2_REG] = 0;
		Reg[LIS3MDL_CTRL_REG3_REG] = LIS3MDL_CTRL_REG3_MD_POWER_DOWN;
		Reg[LIS3MDL_CTRL_REG4_REG] = 0;
		Reg[LIS3MDL_CTRL_REG5_REG] = 0;
		Reg[LIS3MDL_INT_CFG_REG] = 0xE8;
	}
	operator DevIntrf_t * () { return &vDev; }
	DEVINTRF_TYPE Type() { return vType; }
	uint32_t Rate(uint32_t DataRate) { return DataRate; }
	uint32_t Rate(void) { return 400000; }

	// Register address of an access
	uint8_t Decode(uint8_t Cmd, int Len, bool bRead) {
		bool inc;

		if (vType == DEVINTRF_TYPE_SPI)
		{
			assert(((Cmd & LIS3MDL_SPI_READ) != 0) == bRead);
			inc = (Cmd & LIS3MDL_SPI_ADDR_INC) != 0;
			Cmd &= 0x3F;
		}
		else
		{
			inc = (Cmd & LIS3MDL_I2C_ADDR_INC) != 0;
			Cmd &= 0x7F;
		}
		// A multiple byte access without the flag would repeat one register
		assert(Len == 1 || inc);
		return Cmd;
	}

	int Read(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen, uint8_t *pBuff, int BuffLen) {
		assert(AdCmdLen == 1);
		if (vType == DEVINTRF_TYPE_I2C && DevAddr != vI2cAddr)
		{
			return 0;
		}
		uint8_t addr = Decode(pAdCmd[0], BuffLen, true);
		PrevReadAddr = LastReadAddr;
		PrevReadLen = LastReadLen;
		LastReadAddr = addr;
		LastReadLen = BuffLen;
		Reads++;
		uint8_t a = addr;
		for (int i = 0; i < BuffLen; i++, a = a == LIS3MDL_OUT_X_L_REG + 5 ? LIS3MDL_OUT_X_L_REG : a + 1)
		{
			// The address returns to OUT_X_L after OUT_Z_H, as on the board
			pBuff[i] = Reg[a];
			if (a == LIS3MDL_CTRL_REG2_REG && ResetReads > 0)
			{
				pBuff[i] |= LIS3MDL_CTRL_REG2_SOFT_RST;
				if (ResetReads < 1000)
				{
					ResetReads--;
				}
			}
			if (a == LIS3MDL_OUT_X_L_REG + 5)
			{
				Reg[LIS3MDL_STATUS_REG] &= ~LIS3MDL_STATUS_NEW;
			}
		}
		return BuffLen;
	}

	int Write(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen, const uint8_t *pData, int DataLen) {
		assert(AdCmdLen == 1);
		if (vType == DEVINTRF_TYPE_I2C && DevAddr != vI2cAddr)
		{
			return 0;
		}
		uint8_t addr = Decode(pAdCmd[0], DataLen, false);
		for (int i = 0; i < DataLen; i++)
		{
			uint8_t a = addr + i;
			uint8_t v = pData[i];
			// Read only registers
			assert(a != LIS3MDL_WHO_AM_I_REG && a != LIS3MDL_INT_SRC_REG);
			assert(a < LIS3MDL_STATUS_REG || a > LIS3MDL_TEMP_OUT_L_REG + 1);
			// No self test, 4 wire SPI, little endian, both output bytes
			assert(a != LIS3MDL_CTRL_REG1_REG || (v & LIS3MDL_CTRL_REG1_ST) == 0);
			assert(a != LIS3MDL_CTRL_REG3_REG || (v & LIS3MDL_CTRL_REG3_SIM) == 0);
			assert(a != LIS3MDL_CTRL_REG4_REG || (v & LIS3MDL_CTRL_REG4_BLE) == 0);
			assert(a != LIS3MDL_CTRL_REG5_REG || (v & LIS3MDL_CTRL_REG5_FAST_READ) == 0);
			if (a == LIS3MDL_CTRL_REG2_REG)
			{
				assert((v & LIS3MDL_CTRL_REG2_REBOOT) == 0);
				if (v & LIS3MDL_CTRL_REG2_SOFT_RST)
				{
					Resets++;
					ResetReads = StuckReset ? 1000 : 2;
					Defaults();
					continue;
				}
			}
			Reg[a] = v;
			Writes++;
		}
		return DataLen;
	}

	void Set16(uint8_t Addr, int16_t Val) {
		Reg[Addr] = (uint8_t)Val;
		Reg[Addr + 1] = (uint8_t)((uint16_t)Val >> 8);
	}
	void Sample(int16_t X, int16_t Y, int16_t Z, int16_t T) {
		Set16(LIS3MDL_OUT_X_L_REG, X);
		Set16(LIS3MDL_OUT_X_L_REG + 2, Y);
		Set16(LIS3MDL_OUT_X_L_REG + 4, Z);
		Set16(LIS3MDL_TEMP_OUT_L_REG, T);
		Reg[LIS3MDL_STATUS_REG] = LIS3MDL_STATUS_NEW;
		if ((Reg[LIS3MDL_CTRL_REG3_REG] & LIS3MDL_CTRL_REG3_MD_MASK) == LIS3MDL_CTRL_REG3_MD_SINGLE)
		{
			// Power down after a single conversion
			Reg[LIS3MDL_CTRL_REG3_REG] |= LIS3MDL_CTRL_REG3_MD_POWER_DOWN;
		}
	}

	uint8_t Reg[64];
	int ResetReads = 0;
	int Resets = 0;
	int Writes = 0;
	int Reads = 0;
	int LastReadAddr = -1;
	int LastReadLen = 0;
	int PrevReadAddr = -1;
	int PrevReadLen = 0;
	bool StuckReset = false;

private:
	DEVINTRF_TYPE vType;
	uint8_t vI2cAddr;
	DevIntrf_t vDev;
};

// Interrupt state is protected in Device
class MagLis3mdlProbe : public MagLis3mdl {
public:
	bool IntEnabled() const { return InterruptEnabled(); }
	uint8_t IntId() const { return InterruptId(); }
};

static int s_Fail = 0;
static int s_Checks = 0;

#define CHECK(c)	do { s_Checks++; if (!(c)) { s_Fail++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static int s_DevEvt = 0;

static void DevEvt(Device * const pDev, DEV_EVT Evt)
{
	(void)pDev;
	if (Evt == DEV_EVT_DATA_RDY)
	{
		s_DevEvt++;
	}
}

static const MagSensorCfg_t s_MagCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 10000,
	.Precision = MAGSENSOR_PRECISION_HIGH,
	.Inter = 0,
	.IntPol = DEVINTR_POL_HIGH,
	.bFifoEn = false,
};

static bool Near(float A, double B)
{
	return fabs(A - B) <= 1e-3 + fabs(B) * 1e-6;
}

// Field in uT at LsbPerGauss, 100 uT per gauss, the 3 axes in one read,
// then the temperature
static void CheckSample(MagLis3mdl &Dev, Lis3mdlModel &M, int16_t X, int16_t Y, int16_t Z, int16_t T,
						int LsbPerGauss)
{
	MagSensorData_t d;
	MagSensorRawData_t r;
	int reads = M.Reads;

	M.Sample(X, Y, Z, T);
	CHECK(Dev.UpdateData() == true);
	CHECK(M.Reads == reads + 2);
	CHECK(M.PrevReadAddr == LIS3MDL_STATUS_REG && M.PrevReadLen == 7);
	CHECK(M.LastReadAddr == LIS3MDL_TEMP_OUT_L_REG && M.LastReadLen == 2);
	CHECK(M.Reg[LIS3MDL_STATUS_REG] == 0);
	Dev.Read(d);
	Dev.Read(r);
	CHECK(r.X == X && r.Y == Y && r.Z == Z);
	CHECK(Near(d.X, X * 100.0 / LsbPerGauss));
	CHECK(Near(d.Y, Y * 100.0 / LsbPerGauss));
	CHECK(Near(d.Z, Z * 100.0 / LsbPerGauss));
	CHECK(fabs(r.Temp - (T * 100.0 / 8 + 2500)) < 1.0);
	CHECK(fabs(d.Temp - (T / 8.0 + 25)) < 0.01);
}

static void TestI2c(void)
{
	Lis3mdlModel m(DEVINTRF_TYPE_I2C);
	MagLis3mdlProbe dev;

	// Address not given, found at the second one
	CHECK(dev.Init(s_MagCfg, &m, NULL) == true);
	CHECK(dev.DeviceAddress() == LIS3MDL_I2C_DEV_ADDR1);
	CHECK(m.Resets == 1);
	CHECK(m.Reg[LIS3MDL_CTRL_REG5_REG] == LIS3MDL_CTRL_REG5_BDU);
	CHECK(m.Reg[LIS3MDL_CTRL_REG2_REG] == LIS3MDL_CTRL_REG2_FS_4G);
	CHECK(m.Reg[LIS3MDL_CTRL_REG1_REG] == (LIS3MDL_CTRL_REG1_TEMP_EN | LIS3MDL_CTRL_REG1_OM_UHP |
										   (4 << LIS3MDL_CTRL_REG1_DO_POS)));
	CHECK(m.Reg[LIS3MDL_CTRL_REG4_REG] == LIS3MDL_CTRL_REG4_OMZ_UHP);
	// Powered down until Enable
	CHECK(m.Reg[LIS3MDL_CTRL_REG3_REG] == LIS3MDL_CTRL_REG3_MD_POWER_DOWN);
	CHECK(dev.SamplingFrequency() == 10000);
	CHECK(dev.Scale() == 4);
	CHECK(dev.Range() == LIS3MDL_ADC_RANGE);
	CHECK(dev.Precision() == MAGSENSOR_PRECISION_HIGH);
	CHECK(dev.IntEnabled() == false);

	CHECK(dev.Enable() == true);
	CHECK(m.Reg[LIS3MDL_CTRL_REG3_REG] == LIS3MDL_CTRL_REG3_MD_CONTINUOUS);
	CHECK(dev.StartSampling() == true);
	CHECK(m.Reg[LIS3MDL_CTRL_REG3_REG] == LIS3MDL_CTRL_REG3_MD_CONTINUOUS);

	// No new data
	CHECK(dev.UpdateData() == false);

	// 1 gauss, limits, temperature around 25 C
	CheckSample(dev, m, 6842, -6842, 3421, 80, LIS3MDL_SENSITIVITY_4G);
	CheckSample(dev, m, 32767, -32768, 0, -200, LIS3MDL_SENSITIVITY_4G);
	CheckSample(dev, m, -1, 1, 3000, -1, LIS3MDL_SENSITIVITY_4G);
	CheckSample(dev, m, 400, 0, -400, 480, LIS3MDL_SENSITIVITY_4G);

	MagSensorData_t d;
	MagSensorRawData_t r;

	m.Sample(6842, 0, 0, 80);
	CHECK(dev.UpdateData() == true);
	dev.Read(r);
	CHECK(r.Temp == 3500);
	CHECK(r.Sensitivity[0] == 15 && r.Sensitivity[1] == 15 && r.Sensitivity[2] == 15);

	// Full scale, register and conversion, sensitivity in nT rounded
	const struct {
		uint16_t Ask, Gauss, Lsb, Nt;
		uint8_t Fs;
	} fs[] = {
		{ 8, 8, LIS3MDL_SENSITIVITY_8G, 29, LIS3MDL_CTRL_REG2_FS_8G },
		{ 5, 8, LIS3MDL_SENSITIVITY_8G, 29, LIS3MDL_CTRL_REG2_FS_8G },
		{ 12, 12, LIS3MDL_SENSITIVITY_12G, 44, LIS3MDL_CTRL_REG2_FS_12G },
		{ 13, 16, LIS3MDL_SENSITIVITY_16G, 58, LIS3MDL_CTRL_REG2_FS_16G },
		{ 100, 16, LIS3MDL_SENSITIVITY_16G, 58, LIS3MDL_CTRL_REG2_FS_16G },
		{ 0, 4, LIS3MDL_SENSITIVITY_4G, 15, LIS3MDL_CTRL_REG2_FS_4G },
	};

	for (auto &f : fs)
	{
		CHECK(dev.Scale(f.Ask) == f.Gauss);
		CHECK(dev.Scale() == f.Gauss);
		CHECK(m.Reg[LIS3MDL_CTRL_REG2_REG] == f.Fs);
		CheckSample(dev, m, (int16_t)f.Lsb, (int16_t)-f.Lsb, 1000, 0, f.Lsb);
		dev.Read(r);
		CHECK(r.Sensitivity[0] == f.Nt && r.Sensitivity[2] == f.Nt);
	}

	// Calibration with the exact scale: offset in LSB, gain on X
	const float gain[3][3] = { { 2, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
	const float offset[3] = { 100, 0, -684.2f };

	dev.SetCalibration(gain, offset);
	m.Sample(6842, 6842, 0, 0);
	CHECK(dev.UpdateData() == true);
	dev.Read(d);
	CHECK(Near(d.X, 200.0 + 100 * 100.0 / 6842));
	CHECK(Near(d.Y, 100.0));
	CHECK(fabs(d.Z + 10.0) < 1e-3);

	// A new scale keeps the calibration, gain per LSB follows, offset in uT stays
	CHECK(dev.Scale(8) == 8);
	m.Sample(3421, 3421, 0, 0);
	CHECK(dev.UpdateData() == true);
	dev.Read(d);
	CHECK(Near(d.X, 200.0 + 100 * 100.0 / 6842));
	CHECK(Near(d.Y, 100.0));
	CHECK(fabs(d.Z + 10.0) < 1e-3);

	dev.ClearCalibration();
	CheckSample(dev, m, 3421, -3421, 0, 0, LIS3MDL_SENSITIVITY_8G);
	CHECK(dev.Scale(4) == 4);

	// Rate through the base class
	MagSensor *pm = &dev;
	const struct {
		uint32_t Ask, Freq;
		uint8_t Do;
	} rate[] = {
		{ 0, 625, 0 }, { 625, 625, 0 }, { 626, 1250, 1 }, { 2500, 2500, 2 }, { 5000, 5000, 3 },
		{ 19999, 20000, 5 }, { 40000, 40000, 6 }, { 40001, 80000, 7 }, { 80000, 80000, 7 },
	};

	for (auto &f : rate)
	{
		CHECK(pm->SamplingFrequency(f.Ask) == f.Freq);
		CHECK(m.Reg[LIS3MDL_CTRL_REG1_REG] == (LIS3MDL_CTRL_REG1_TEMP_EN | LIS3MDL_CTRL_REG1_OM_UHP |
											   (f.Do << LIS3MDL_CTRL_REG1_DO_POS)));
	}

	// Above 80 Hz the rate of the mode
	CHECK(pm->SamplingFrequency(80001) == 155000);
	CHECK(m.Reg[LIS3MDL_CTRL_REG1_REG] == (LIS3MDL_CTRL_REG1_TEMP_EN | LIS3MDL_CTRL_REG1_OM_UHP |
										   LIS3MDL_CTRL_REG1_FAST_ODR));
	CHECK(pm->Precision(MAGSENSOR_PRECISION_LOW) == MAGSENSOR_PRECISION_LOW);
	CHECK(m.Reg[LIS3MDL_CTRL_REG1_REG] == (LIS3MDL_CTRL_REG1_TEMP_EN | LIS3MDL_CTRL_REG1_OM_LP |
										   LIS3MDL_CTRL_REG1_FAST_ODR));
	CHECK(m.Reg[LIS3MDL_CTRL_REG4_REG] == LIS3MDL_CTRL_REG4_OMZ_LP);
	CHECK(pm->SamplingFrequency() == 1000000);
	CHECK(pm->Precision(MAGSENSOR_PRECISION_HIGH) == MAGSENSOR_PRECISION_HIGH);
	CHECK(m.Reg[LIS3MDL_CTRL_REG4_REG] == LIS3MDL_CTRL_REG4_OMZ_UHP);
	CHECK(pm->SamplingFrequency() == 155000);
	CHECK(pm->SamplingFrequency(20000) == 20000);
	CHECK(m.Reg[LIS3MDL_CTRL_REG1_REG] == (LIS3MDL_CTRL_REG1_TEMP_EN | LIS3MDL_CTRL_REG1_OM_UHP |
										   (5 << LIS3MDL_CTRL_REG1_DO_POS)));
	CHECK(pm->Precision(MAGSENSOR_PRECISION_LOW) == MAGSENSOR_PRECISION_LOW);
	CHECK(pm->SamplingFrequency() == 20000);
	CHECK(m.Reg[LIS3MDL_CTRL_REG1_REG] == (LIS3MDL_CTRL_REG1_TEMP_EN | LIS3MDL_CTRL_REG1_OM_LP |
										   (5 << LIS3MDL_CTRL_REG1_DO_POS)));
	CHECK(pm->SamplingFrequency(100000) == 1000000);
	CHECK(m.Reg[LIS3MDL_CTRL_REG1_REG] == (LIS3MDL_CTRL_REG1_TEMP_EN | LIS3MDL_CTRL_REG1_OM_LP |
										   LIS3MDL_CTRL_REG1_FAST_ODR));
	CHECK(pm->SamplingFrequency(20000) == 20000);

	// Event on new data only
	dev.SetEvtHandler(DevEvt);
	m.Sample(1, 2, 3, 0);
	dev.IntHandler();
	CHECK(s_DevEvt == 1);
	dev.IntHandler();
	CHECK(s_DevEvt == 1);

	dev.Disable();
	CHECK(m.Reg[LIS3MDL_CTRL_REG3_REG] == LIS3MDL_CTRL_REG3_MD_POWER_DOWN);
}

static void TestSingleSpi(void)
{
	Lis3mdlModel m(DEVINTRF_TYPE_SPI);
	MagLis3mdlProbe dev;
	MagSensorCfg_t cfg = s_MagCfg;

	cfg.DevAddr = 3;
	cfg.OpMode = SENSOR_OPMODE_SINGLE;
	cfg.Precision = MAGSENSOR_PRECISION_LOW;
	cfg.Freq = 100000;
	cfg.Inter = 1;
	CHECK(dev.Init(cfg, &m, NULL) == true);
	CHECK(dev.DeviceAddress() == 3);
	CHECK(dev.IntEnabled() == true && dev.IntId() == 1);
	// No fast rate in single conversion
	CHECK(dev.SamplingFrequency() == 80000);
	CHECK(m.Reg[LIS3MDL_CTRL_REG1_REG] == (LIS3MDL_CTRL_REG1_TEMP_EN | LIS3MDL_CTRL_REG1_OM_LP |
										   (7 << LIS3MDL_CTRL_REG1_DO_POS)));
	CHECK(m.Reg[LIS3MDL_CTRL_REG4_REG] == LIS3MDL_CTRL_REG4_OMZ_LP);

	CHECK(dev.Enable() == true);
	CHECK(m.Reg[LIS3MDL_CTRL_REG3_REG] == LIS3MDL_CTRL_REG3_MD_POWER_DOWN);
	CHECK(dev.StartSampling() == true);
	CHECK(m.Reg[LIS3MDL_CTRL_REG3_REG] == LIS3MDL_CTRL_REG3_MD_SINGLE);
	CheckSample(dev, m, 6842, -3421, 1711, 40, LIS3MDL_SENSITIVITY_4G);
	CHECK(m.Reg[LIS3MDL_CTRL_REG3_REG] == LIS3MDL_CTRL_REG3_MD_POWER_DOWN);
	CHECK(dev.StartSampling() == true);
	CHECK(m.Reg[LIS3MDL_CTRL_REG3_REG] == LIS3MDL_CTRL_REG3_MD_SINGLE);
	CheckSample(dev, m, -100, 200, -300, -40, LIS3MDL_SENSITIVITY_4G);

	CHECK(dev.Scale(12) == 12);
	CHECK(m.Reg[LIS3MDL_CTRL_REG2_REG] == LIS3MDL_CTRL_REG2_FS_12G);
}

static void TestFailures(void)
{
	MagLis3mdl dev;

	// Wrong identification, the LIS2MDL
	Lis3mdlModel w(DEVINTRF_TYPE_I2C);

	w.Reg[LIS3MDL_WHO_AM_I_REG] = 0x40;
	CHECK(dev.Init(s_MagCfg, &w, NULL) == false);
	CHECK(w.Writes == 0 && w.Resets == 0);

	// Address given, device at the other one
	Lis3mdlModel a(DEVINTRF_TYPE_I2C, LIS3MDL_I2C_DEV_ADDR0);
	MagSensorCfg_t cfg = s_MagCfg;

	cfg.DevAddr = LIS3MDL_I2C_DEV_ADDR1;
	CHECK(dev.Init(cfg, &a, NULL) == false);
	CHECK(a.Writes == 0 && a.Resets == 0);

	// Reset that does not end
	Lis3mdlModel s(DEVINTRF_TYPE_I2C);

	s.StuckReset = true;
	CHECK(dev.Init(s_MagCfg, &s, NULL) == false);
	CHECK(dev.Valid() == false);

	// A good device after the failures, address not given, found at the first one
	Lis3mdlModel g(DEVINTRF_TYPE_I2C, LIS3MDL_I2C_DEV_ADDR0);

	CHECK(dev.Init(s_MagCfg, &g, NULL) == true);
	CHECK(dev.Valid() == true);
	CHECK(dev.DeviceAddress() == LIS3MDL_I2C_DEV_ADDR0);
}

int main()
{
	TestI2c();
	TestSingleSpi();
	TestFailures();

	printf("LIS3MDL driver model: %d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}

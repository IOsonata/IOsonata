/**-------------------------------------------------------------------------
@file	lps22hb_test.cpp

@brief	Host test of the LPS22HB pressure and temperature driver.

Runs src/sensors/press_lps22hb.cpp against a register model of the device on
an I2C or SPI interface: detection, software reset, configuration registers,
one shot and continuous modes, low pass filter, data conversion.

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

#include "sensors/press_lps22hb.h"

#define LPS22HB_CTRL_REG2_DEFAULT		LPS22HB_CTRL_REG2_IF_ADD_INC

// Register model of the LPS22HB behind an I2C or SPI interface
class Lps22hbModel : public DeviceIntrf {
public:
	Lps22hbModel(DEVINTRF_TYPE Type, uint8_t I2cAddr = LPS22HB_I2C_DEV_ADDR1) :
		vType(Type), vI2cAddr(I2cAddr), vDev{} {
		memset(Reg, 0, sizeof(Reg));
		Reg[LPS22HB_WHO_AM_I_REG] = LPS22HB_WHO_AM_I_ID;
		Reg[LPS22HB_CTRL_REG2_REG] = LPS22HB_CTRL_REG2_DEFAULT;
	}
	operator DevIntrf_t * () { return &vDev; }
	DEVINTRF_TYPE Type() { return vType; }
	uint32_t Rate(uint32_t DataRate) { return DataRate; }
	uint32_t Rate(void) { return 400000; }

	// Register address of an access
	uint8_t Decode(uint8_t Cmd, int Len, bool bRead) {
		if (vType == DEVINTRF_TYPE_SPI)
		{
			assert(((Cmd & 0x80) != 0) == bRead);
		}
		else
		{
			assert((Cmd & 0x80) == 0);
		}
		// A multiple byte access without the increment would repeat one register
		assert(Len == 1 || (Reg[LPS22HB_CTRL_REG2_REG] & LPS22HB_CTRL_REG2_IF_ADD_INC));
		return Cmd & 0x7F;
	}

	int Read(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen, uint8_t *pBuff, int BuffLen) {
		assert(AdCmdLen == 1);
		if (vType == DEVINTRF_TYPE_I2C && DevAddr != vI2cAddr)
		{
			return 0;
		}
		uint8_t addr = Decode(pAdCmd[0], BuffLen, true);
		LastReadAddr = addr;
		LastReadLen = BuffLen;
		Reads++;
		for (int i = 0; i < BuffLen; i++)
		{
			uint8_t a = addr + i;
			pBuff[i] = Reg[a];
			if (a == LPS22HB_CTRL_REG2_REG && ResetReads > 0)
			{
				pBuff[i] |= LPS22HB_CTRL_REG2_SWRESET;
				if (ResetReads < 1000)
				{
					ResetReads--;
				}
			}
			if (a == LPS22HB_PRESS_OUT_XL_REG + 2)
			{
				Reg[LPS22HB_STATUS_REG] &= ~LPS22HB_STATUS_P_DA;
			}
			if (a == LPS22HB_TEMP_OUT_L_REG + 1)
			{
				Reg[LPS22HB_STATUS_REG] &= ~LPS22HB_STATUS_T_DA;
			}
			if (a == LPS22HB_LPFP_RES_REG)
			{
				FilterResets++;
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
			// Read only registers
			assert(a != LPS22HB_WHO_AM_I_REG && a < LPS22HB_INT_SOURCE_REG);
			if (a == LPS22HB_CTRL_REG2_REG)
			{
				// The I2C interface stays on, no FIFO, no trimming reload
				assert((pData[i] & (LPS22HB_CTRL_REG2_I2C_DIS | LPS22HB_CTRL_REG2_FIFO_EN |
									LPS22HB_CTRL_REG2_BOOT)) == 0);
				if (pData[i] & LPS22HB_CTRL_REG2_SWRESET)
				{
					// Control registers to their default values
					Resets++;
					ResetReads = StuckReset ? 1000 : 2;
					Reg[LPS22HB_INTERRUPT_CFG_REG] = 0;
					Reg[LPS22HB_CTRL_REG1_REG] = 0;
					Reg[LPS22HB_CTRL_REG2_REG] = LPS22HB_CTRL_REG2_DEFAULT;
					Reg[LPS22HB_CTRL_REG3_REG] = 0;
					Reg[LPS22HB_FIFO_CTRL_REG] = 0;
					Reg[LPS22HB_RES_CONF_REG] = 0;
					continue;
				}
			}
			Reg[a] = pData[i];
			Writes++;
		}
		return DataLen;
	}

	void Sample(int32_t P, int16_t T, uint8_t Status) {
		Reg[LPS22HB_PRESS_OUT_XL_REG] = (uint8_t)P;
		Reg[LPS22HB_PRESS_OUT_XL_REG + 1] = (uint8_t)((uint32_t)P >> 8);
		Reg[LPS22HB_PRESS_OUT_XL_REG + 2] = (uint8_t)((uint32_t)P >> 16);
		Reg[LPS22HB_TEMP_OUT_L_REG] = (uint8_t)T;
		Reg[LPS22HB_TEMP_OUT_L_REG + 1] = (uint8_t)((uint16_t)T >> 8);
		Reg[LPS22HB_STATUS_REG] = Status;
		Reg[LPS22HB_CTRL_REG2_REG] &= ~LPS22HB_CTRL_REG2_ONE_SHOT;
	}

	uint8_t Reg[64];
	int ResetReads = 0;
	int Resets = 0;
	int Writes = 0;
	int Reads = 0;
	int FilterResets = 0;
	int LastReadAddr = -1;
	int LastReadLen = 0;
	bool StuckReset = false;

private:
	DEVINTRF_TYPE vType;
	uint8_t vI2cAddr;
	DevIntrf_t vDev;
};

static int s_Fail = 0;
static int s_Checks = 0;

#define CHECK(c)	do { s_Checks++; if (!(c)) { s_Fail++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static int s_PressEvt = 0, s_TempEvt = 0, s_DevEvt = 0;

static void PressEvt(PressSensor * const pSensor, PressureSensorData_t *pData)
{
	(void)pSensor;
	(void)pData;
	s_PressEvt++;
}

static void TempEvt(TempSensor * const pSensor, TempSensorData_t *pData)
{
	(void)pSensor;
	(void)pData;
	s_TempEvt++;
}

static void DevEvt(Device * const pDev, DEV_EVT Evt)
{
	(void)pDev;
	if (Evt == DEV_EVT_DATA_RDY)
	{
		s_DevEvt++;
	}
}

static const PressureSensorCfg_t s_PressCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 20000,
	.PresOvrs = 0,
	.FilterCoeff = 9,
	.EvtHandler = PressEvt,
};

static const TempSensorCfg_t s_TempCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 60000,
	.IntPrio = 6,
	.Inter = 1,
	.IntPol = DEVINTR_POL_LOW,
	.TempOvrs = 0,
	.FilterCoeff = 0,
	.DataRdyCB = TempEvt,
};

// Pressure in Pa against the hPa x 4096 output, both outputs in one read
static void CheckSample(PressLps22hb &Dev, Lps22hbModel &M, int32_t P, int16_t T)
{
	PressureSensorData_t pd;
	TempSensorData_t td;
	int reads = M.Reads;

	M.Sample(P, T, LPS22HB_STATUS_P_DA | LPS22HB_STATUS_T_DA);
	CHECK(Dev.UpdateData() == true);
	CHECK(M.Reads == reads + 1);
	CHECK(M.LastReadAddr == LPS22HB_STATUS_REG && M.LastReadLen == 6);
	CHECK(M.Reg[LPS22HB_STATUS_REG] == 0);
	Dev.Read(pd);
	Dev.Read(td);

	double exp = P * 100.0 / LPS22HB_PRESS_SENSITIVITY;

	CHECK(fabs(pd.Pressure - exp) <= 0.5);
	CHECK(td.Temperature == T);
}

static void TestI2c(void)
{
	Lps22hbModel m(DEVINTRF_TYPE_I2C);
	PressLps22hb dev;

	// Address not given, found at the second one
	CHECK(dev.Init(s_PressCfg, &m, NULL) == true);
	CHECK(dev.DeviceAddress() == LPS22HB_I2C_DEV_ADDR1);
	CHECK(m.Resets == 1);
	// Powered down, filter ODR/9
	CHECK(m.Reg[LPS22HB_CTRL_REG1_REG] == (LPS22HB_CTRL_REG1_BDU | LPS22HB_CTRL_REG1_EN_LPFP));
	CHECK(m.Reg[LPS22HB_CTRL_REG2_REG] == LPS22HB_CTRL_REG2_IF_ADD_INC);
	CHECK(((PressSensor&)dev).SamplingFrequency() == 25000);

	CHECK(dev.Init(s_TempCfg, &m, NULL) == true);
	CHECK(m.Resets == 1);
	CHECK(m.Reg[LPS22HB_CTRL_REG3_REG] == (LPS22HB_CTRL_REG3_DRDY | LPS22HB_CTRL_REG3_INT_H_L));
	CHECK(((TempSensor&)dev).SamplingFrequency() == 75000);
	CHECK(((PressSensor&)dev).SamplingFrequency() == 75000);
	// Powered down until Enable
	CHECK((m.Reg[LPS22HB_CTRL_REG1_REG] & LPS22HB_CTRL_REG1_ODR_MASK) == 0);
	CHECK(m.FilterResets == 0);

	CHECK(dev.Enable() == true);
	CHECK(m.FilterResets == 1);
	CHECK(m.Reg[LPS22HB_CTRL_REG1_REG] == (LPS22HB_CTRL_REG1_ODR_75HZ | LPS22HB_CTRL_REG1_EN_LPFP |
										   LPS22HB_CTRL_REG1_BDU));
	CHECK(dev.StartSampling() == true);
	CHECK((m.Reg[LPS22HB_CTRL_REG2_REG] & LPS22HB_CTRL_REG2_ONE_SHOT) == 0);

	// No new data
	CHECK(dev.UpdateData() == false);

	// 1013.25 hPa, 260 and 1260 hPa, 24 bit limits, rounding to Pa
	CheckSample(dev, m, 4150272, 2534);
	CheckSample(dev, m, 260 * 4096, -4000);
	CheckSample(dev, m, 1260 * 4096 + 4095, 8500);
	CheckSample(dev, m, 0x7FFFFF, 0x7FFF);
	CheckSample(dev, m, 41, -1);
	CheckSample(dev, m, 3999877, -32768);

	PressureSensorData_t pd;
	TempSensorData_t td;

	m.Sample(4150272, 2534, LPS22HB_STATUS_P_DA | LPS22HB_STATUS_T_DA);
	CHECK(dev.UpdateData() == true);
	dev.Read(pd);
	dev.Read(td);
	CHECK(pd.Pressure == 101325);
	CHECK(td.Temperature == 2534);
	CHECK(fabs(((PressSensor&)dev).ReadPressure() - 101.325f) < 0.0001f);
	CHECK(fabs(((TempSensor&)dev).ReadTemperature() - 25.34f) < 0.0001f);

	// Half a Pa rounds up
	m.Sample(512, 0, LPS22HB_STATUS_P_DA);
	CHECK(dev.UpdateData() == true);
	dev.Read(pd);
	CHECK(pd.Pressure == 13);

	// Negative outputs, from the 24 bit sign, read as 0 Pa
	m.Sample(0x800000, 0, LPS22HB_STATUS_P_DA);
	CHECK(dev.UpdateData() == true);
	dev.Read(pd);
	CHECK(pd.Pressure == 0);
	m.Sample(0xFFFFFF, 0, LPS22HB_STATUS_P_DA);
	CHECK(dev.UpdateData() == true);
	dev.Read(pd);
	CHECK(pd.Pressure == 0);

	// Temperature only keeps the pressure, events per quantity
	m.Sample(4150272, 0, LPS22HB_STATUS_P_DA);
	CHECK(dev.UpdateData() == true);

	int pev = s_PressEvt, tev = s_TempEvt;

	m.Sample(0, -1234, LPS22HB_STATUS_T_DA);
	dev.SetEvtHandler(DevEvt);
	dev.IntHandler();
	CHECK(s_DevEvt == 1);
	CHECK(s_PressEvt == pev && s_TempEvt == tev + 1);
	dev.Read(pd);
	dev.Read(td);
	CHECK(pd.Pressure == 101325);
	CHECK(td.Temperature == -1234);
	dev.IntHandler();
	CHECK(s_DevEvt == 1);

	// Pressure only keeps the temperature
	m.Sample(4096 * 1000, 2000, LPS22HB_STATUS_P_DA);
	dev.IntHandler();
	CHECK(s_DevEvt == 2);
	CHECK(s_PressEvt == pev + 1 && s_TempEvt == tev + 1);
	dev.Read(pd);
	dev.Read(td);
	CHECK(pd.Pressure == 100000);
	CHECK(td.Temperature == -1234);

	// Rate through the base classes while running
	PressSensor *pp = &dev;

	CHECK(pp->SamplingFrequency(1000) == 1000);
	CHECK((m.Reg[LPS22HB_CTRL_REG1_REG] & LPS22HB_CTRL_REG1_ODR_MASK) == LPS22HB_CTRL_REG1_ODR_1HZ);
	CHECK(((TempSensor&)dev).SamplingFrequency() == 1000);
	CHECK(pp->SamplingFrequency(0) == 1000);
	CHECK(pp->SamplingFrequency(1001) == 10000);
	CHECK((m.Reg[LPS22HB_CTRL_REG1_REG] & LPS22HB_CTRL_REG1_ODR_MASK) == LPS22HB_CTRL_REG1_ODR_10HZ);
	CHECK(pp->SamplingFrequency(25000) == 25000);
	CHECK((m.Reg[LPS22HB_CTRL_REG1_REG] & LPS22HB_CTRL_REG1_ODR_MASK) == LPS22HB_CTRL_REG1_ODR_25HZ);
	CHECK(pp->SamplingFrequency(25001) == 50000);
	CHECK((m.Reg[LPS22HB_CTRL_REG1_REG] & LPS22HB_CTRL_REG1_ODR_MASK) == LPS22HB_CTRL_REG1_ODR_50HZ);
	CHECK(pp->SamplingFrequency(200000) == 75000);
	CHECK(m.Reg[LPS22HB_CTRL_REG1_REG] == (LPS22HB_CTRL_REG1_ODR_75HZ | LPS22HB_CTRL_REG1_EN_LPFP |
										   LPS22HB_CTRL_REG1_BDU));

	dev.Disable();
	CHECK(m.Reg[LPS22HB_CTRL_REG1_REG] == (LPS22HB_CTRL_REG1_EN_LPFP | LPS22HB_CTRL_REG1_BDU));

	// A rate change while powered down waits for Enable
	CHECK(pp->SamplingFrequency(10000) == 10000);
	CHECK(m.Reg[LPS22HB_CTRL_REG1_REG] == (LPS22HB_CTRL_REG1_EN_LPFP | LPS22HB_CTRL_REG1_BDU));
	CHECK(dev.Enable() == true);
	CHECK(m.FilterResets == 2);
	CHECK(m.Reg[LPS22HB_CTRL_REG1_REG] == (LPS22HB_CTRL_REG1_ODR_10HZ | LPS22HB_CTRL_REG1_EN_LPFP |
										   LPS22HB_CTRL_REG1_BDU));
}

static void TestOneShotSpi(void)
{
	Lps22hbModel m(DEVINTRF_TYPE_SPI);
	PressLps22hb dev;
	PressureSensorCfg_t pcfg = s_PressCfg;
	TempSensorCfg_t tcfg = s_TempCfg;

	pcfg.DevAddr = 2;
	pcfg.OpMode = SENSOR_OPMODE_SINGLE;
	pcfg.FilterCoeff = 20;
	tcfg.DevAddr = 2;
	tcfg.OpMode = SENSOR_OPMODE_SINGLE;
	tcfg.Inter = 1;
	tcfg.IntPol = DEVINTR_POL_HIGH;
	CHECK(dev.Init(pcfg, &m, NULL) == true);
	CHECK(dev.DeviceAddress() == 2);
	CHECK(dev.Init(tcfg, &m, NULL) == true);
	CHECK(m.Reg[LPS22HB_CTRL_REG3_REG] == LPS22HB_CTRL_REG3_DRDY);
	CHECK(((PressSensor&)dev).SamplingFrequency() == 0);

	CHECK(dev.Enable() == true);
	CHECK(m.FilterResets == 1);
	CHECK(m.Reg[LPS22HB_CTRL_REG1_REG] == (LPS22HB_CTRL_REG1_ODR_ONE_SHOT | LPS22HB_CTRL_REG1_EN_LPFP |
										   LPS22HB_CTRL_REG1_LPFP_CFG | LPS22HB_CTRL_REG1_BDU));
	CHECK(dev.StartSampling() == true);
	CHECK(m.Reg[LPS22HB_CTRL_REG2_REG] == (LPS22HB_CTRL_REG2_IF_ADD_INC | LPS22HB_CTRL_REG2_ONE_SHOT));
	// Measurement in progress
	CHECK(dev.StartSampling() == false);
	CheckSample(dev, m, 4150272, 2534);
	CHECK(dev.StartSampling() == true);
	CheckSample(dev, m, 3000000, -500);

	// Below 9 no filter, and no filter reset on Enable
	pcfg.FilterCoeff = 8;
	tcfg.Inter = 0;
	CHECK(dev.Init(pcfg, &m, NULL) == true);
	CHECK(dev.Init(tcfg, &m, NULL) == true);
	CHECK(m.Reg[LPS22HB_CTRL_REG1_REG] == LPS22HB_CTRL_REG1_BDU);
	CHECK(m.Reg[LPS22HB_CTRL_REG3_REG] == 0);
	CHECK(dev.Enable() == true);
	CHECK(m.FilterResets == 1);

	// 19 is still ODR/9
	pcfg.FilterCoeff = 19;
	CHECK(dev.Init(pcfg, &m, NULL) == true);
	CHECK(m.Reg[LPS22HB_CTRL_REG1_REG] == (LPS22HB_CTRL_REG1_EN_LPFP | LPS22HB_CTRL_REG1_BDU));
}

static void TestFailures(void)
{
	PressLps22hb dev;

	// Wrong identification, the LPS22HH
	Lps22hbModel w(DEVINTRF_TYPE_I2C);

	w.Reg[LPS22HB_WHO_AM_I_REG] = 0xB3;
	CHECK(dev.Init(s_PressCfg, &w, NULL) == false);
	CHECK(w.Writes == 0 && w.Resets == 0);

	// Address given, device at the other one
	Lps22hbModel a(DEVINTRF_TYPE_I2C, LPS22HB_I2C_DEV_ADDR0);
	PressureSensorCfg_t pcfg = s_PressCfg;

	pcfg.DevAddr = LPS22HB_I2C_DEV_ADDR1;
	CHECK(dev.Init(pcfg, &a, NULL) == false);
	CHECK(a.Writes == 0 && a.Resets == 0);

	// Reset that does not end
	Lps22hbModel s(DEVINTRF_TYPE_I2C);

	s.StuckReset = true;
	CHECK(dev.Init(s_PressCfg, &s, NULL) == false);
	CHECK(dev.Valid() == false);

	// A good device after the failures, address not given, found at the first one
	Lps22hbModel g(DEVINTRF_TYPE_I2C, LPS22HB_I2C_DEV_ADDR0);

	CHECK(dev.Init(s_TempCfg, &g, NULL) == true);
	CHECK(dev.Valid() == true);
	CHECK(dev.DeviceAddress() == LPS22HB_I2C_DEV_ADDR0);
}

int main()
{
	TestI2c();
	TestOneShotSpi();
	TestFailures();

	printf("LPS22HB driver model: %d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}

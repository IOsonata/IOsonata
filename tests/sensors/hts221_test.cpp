/**-------------------------------------------------------------------------
@file	hts221_test.cpp

@brief	Host test of the HTS221 humidity and temperature driver.

Runs src/sensors/humi_hts221.cpp against a register model of the device on an
I2C or SPI interface: detection, calibration read, configuration registers,
one shot and continuous modes, interpolation and clamping.

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

#include "sensors/humi_hts221.h"

// Calibration of the model: 33 % and 75 %, 20 C and 35 C (T1 uses its bit 8)
#define CAL_H0X2		66
#define CAL_H1X2		150
#define CAL_T0X8		160
#define CAL_T1X8		280
#define CAL_H0OUT		-4500
#define CAL_H1OUT		6000
#define CAL_T0OUT		300
#define CAL_T1OUT		1100

// Register model of the HTS221 behind an I2C or SPI interface
class Hts221Model : public DeviceIntrf {
public:
	Hts221Model(DEVINTRF_TYPE Type) : vType(Type), vDev{} {
		PowerOn();
		Calibrate(CAL_H0OUT, CAL_H1OUT, CAL_T0OUT, CAL_T1OUT);
	}
	void PowerOn() {
		memset(Reg, 0, sizeof(Reg));
		Reg[HTS221_WHO_AM_I_REG] = HTS221_WHO_AM_I_ID;
		Reg[HTS221_AV_CONF_REG] = HTS221_AV_CONF_DEFAULT;
	}
	void Calibrate(int16_t H0Out, int16_t H1Out, int16_t T0Out, int16_t T1Out) {
		uint8_t *c = &Reg[HTS221_CALIB_REG];
		c[0] = CAL_H0X2;
		c[1] = CAL_H1X2;
		c[2] = CAL_T0X8 & 0xFF;
		c[3] = CAL_T1X8 & 0xFF;
		c[4] = 0xA5;
		c[5] = ((CAL_T0X8 >> 8) & 3) | (((CAL_T1X8 >> 8) & 3) << 2) | 0xF0;
		Set16(HTS221_CALIB_REG + 6, H0Out);
		Set16(HTS221_CALIB_REG + 8, 0x5A5A);
		Set16(HTS221_CALIB_REG + 10, H1Out);
		Set16(HTS221_CALIB_REG + 12, T0Out);
		Set16(HTS221_CALIB_REG + 14, T1Out);
	}
	operator DevIntrf_t * () { return &vDev; }
	DEVINTRF_TYPE Type() { return vType; }
	uint32_t Rate(uint32_t DataRate) { return DataRate; }
	uint32_t Rate(void) { return 400000; }

	// Register address and increment flag of an access
	uint8_t Decode(uint8_t Cmd, int Len, bool bRead, bool &bInc) {
		if (vType == DEVINTRF_TYPE_SPI)
		{
			assert(((Cmd & HTS221_SPI_READ) != 0) == bRead);
			bInc = (Cmd & HTS221_SPI_ADDR_INC) != 0;
			Cmd &= 0x3F;
		}
		else
		{
			bInc = (Cmd & HTS221_I2C_ADDR_INC) != 0;
			Cmd &= 0x7F;
		}
		// A multiple byte access without the flag would repeat one register
		assert(Len == 1 || bInc);
		return Cmd;
	}

	int Read(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen, uint8_t *pBuff, int BuffLen) {
		assert(AdCmdLen == 1);
		if (vType == DEVINTRF_TYPE_I2C && DevAddr != HTS221_I2C_DEV_ADDR)
		{
			return 0;
		}
		bool inc;
		uint8_t addr = Decode(pAdCmd[0], BuffLen, true, inc);
		for (int i = 0; i < BuffLen; i++)
		{
			uint8_t a = inc ? addr + i : addr;
			pBuff[i] = Reg[a];
			if (a == HTS221_CTRL_REG2_REG && BootReads > 0)
			{
				pBuff[i] |= HTS221_CTRL_REG2_BOOT;
				if (BootReads < 1000)
				{
					BootReads--;
				}
			}
			if (a == HTS221_HUMIDITY_OUT_L_REG + 1)
			{
				Reg[HTS221_STATUS_REG] &= ~HTS221_STATUS_H_DA;
			}
			if (a == HTS221_TEMP_OUT_L_REG + 1)
			{
				Reg[HTS221_STATUS_REG] &= ~HTS221_STATUS_T_DA;
			}
		}
		return BuffLen;
	}

	int Write(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen, const uint8_t *pData, int DataLen) {
		assert(AdCmdLen == 1);
		if (vType == DEVINTRF_TYPE_I2C && DevAddr != HTS221_I2C_DEV_ADDR)
		{
			return 0;
		}
		bool inc;
		uint8_t addr = Decode(pAdCmd[0], DataLen, false, inc);
		for (int i = 0; i < DataLen; i++)
		{
			uint8_t a = inc ? addr + i : addr;
			// Read only registers
			assert(a != HTS221_WHO_AM_I_REG && a != HTS221_STATUS_REG && a < HTS221_HUMIDITY_OUT_L_REG);
			if (a == HTS221_CTRL_REG2_REG && (pData[i] & HTS221_CTRL_REG2_BOOT))
			{
				Boots++;
				BootReads = StuckBoot ? 1000 : 2;
				Reg[a] = pData[i] & ~HTS221_CTRL_REG2_BOOT;
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
	void Sample(int16_t H, int16_t T, uint8_t Status) {
		Set16(HTS221_HUMIDITY_OUT_L_REG, H);
		Set16(HTS221_TEMP_OUT_L_REG, T);
		Reg[HTS221_STATUS_REG] = Status;
		Reg[HTS221_CTRL_REG2_REG] &= ~HTS221_CTRL_REG2_ONE_SHOT;
	}

	uint8_t Reg[64];
	int BootReads = 0;
	int Boots = 0;
	int Writes = 0;
	bool StuckBoot = false;

private:
	DEVINTRF_TYPE vType;
	DevIntrf_t vDev;
};

static int s_Fail = 0;
static int s_Checks = 0;

#define CHECK(c)	do { s_Checks++; if (!(c)) { s_Fail++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

// Expected values by the TN1218 interpolation, in floating point
static double ExpHumi(int H)
{
	double h = CAL_H0X2 / 2.0 + (H - CAL_H0OUT) * (CAL_H1X2 - CAL_H0X2) / 2.0 / (CAL_H1OUT - CAL_H0OUT);
	return h < 0 ? 0 : (h > 100 ? 100 : h);
}

static double ExpTemp(int T)
{
	return CAL_T0X8 / 8.0 + (T - CAL_T0OUT) * (CAL_T1X8 - CAL_T0X8) / 8.0 / (CAL_T1OUT - CAL_T0OUT);
}

static int s_HumiEvt = 0, s_TempEvt = 0, s_DevEvt = 0;

static void HumiEvt(HumiSensor * const pSensor, HumiSensorData_t *pData)
{
	(void)pSensor;
	(void)pData;
	s_HumiEvt++;
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

static const HumiSensorCfg_t s_HumiCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 5000,
	.HumOvrs = 100,
	.FilterCoeff = 0,
	.EvtHandler = HumiEvt,
};

static const TempSensorCfg_t s_TempCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 12500,
	.IntPrio = 6,
	.Inter = 1,
	.IntPol = DEVINTR_POL_LOW,
	.TempOvrs = 2,
	.FilterCoeff = 0,
	.DataRdyCB = TempEvt,
};

static void CheckSample(HumiHts221 &Dev, Hts221Model &M, int16_t H, int16_t T)
{
	HumiSensorData_t hd;
	TempSensorData_t td;

	M.Sample(H, T, HTS221_STATUS_H_DA | HTS221_STATUS_T_DA);
	CHECK(Dev.UpdateData() == true);
	Dev.Read(hd);
	Dev.Read(td);
	CHECK(fabs(hd.Humidity / 100.0 - ExpHumi(H)) <= 0.012);
	CHECK(fabs(td.Temperature / 100.0 - ExpTemp(T)) <= 0.012);
}

static void TestI2c(void)
{
	Hts221Model m(DEVINTRF_TYPE_I2C);
	HumiHts221 dev;

	CHECK(dev.Init(s_HumiCfg, &m, NULL) == true);
	CHECK(dev.DeviceAddress() == HTS221_I2C_DEV_ADDR);
	CHECK(m.Boots == 1);
	CHECK(m.Reg[HTS221_CTRL_REG1_REG] == HTS221_CTRL_REG1_BDU);
	CHECK(m.Reg[HTS221_AV_CONF_REG] == ((3 << HTS221_AV_CONF_AVGT_POS) | (5 << HTS221_AV_CONF_AVGH_POS)));
	CHECK(((HumiSensor&)dev).SamplingFrequency() == 7000);

	CHECK(dev.Init(s_TempCfg, &m, NULL) == true);
	CHECK(m.Boots == 1);
	CHECK(m.Reg[HTS221_AV_CONF_REG] == (5 << HTS221_AV_CONF_AVGH_POS));
	CHECK(m.Reg[HTS221_CTRL_REG3_REG] == (HTS221_CTRL_REG3_DRDY | HTS221_CTRL_REG3_DRDY_H_L));
	CHECK(((TempSensor&)dev).SamplingFrequency() == 12500);
	CHECK(((HumiSensor&)dev).SamplingFrequency() == 12500);
	// Powered down until Enable
	CHECK(m.Reg[HTS221_CTRL_REG1_REG] == HTS221_CTRL_REG1_BDU);

	CHECK(dev.Enable() == true);
	CHECK(m.Reg[HTS221_CTRL_REG1_REG] == (HTS221_CTRL_REG1_PD | HTS221_CTRL_REG1_BDU | HTS221_CTRL_REG1_ODR_12_5HZ));
	CHECK(dev.StartSampling() == true);
	CHECK((m.Reg[HTS221_CTRL_REG2_REG] & HTS221_CTRL_REG2_ONE_SHOT) == 0);

	// No new data
	CHECK(dev.UpdateData() == false);

	// Interpolation at the calibration points, between and outside them
	CheckSample(dev, m, 750, 700);
	CheckSample(dev, m, CAL_H0OUT, CAL_T0OUT);
	CheckSample(dev, m, CAL_H1OUT, CAL_T1OUT);
	CheckSample(dev, m, -20000, -2000);
	CheckSample(dev, m, 30000, 4000);
	CheckSample(dev, m, 1, -500);

	HumiSensorData_t hd;
	TempSensorData_t td;

	m.Sample(750, 700, HTS221_STATUS_H_DA | HTS221_STATUS_T_DA);
	CHECK(dev.UpdateData() == true);
	dev.Read(hd);
	dev.Read(td);
	CHECK(hd.Humidity == 5400);
	CHECK(td.Temperature == 2750);
	CHECK(fabs(((HumiSensor&)dev).ReadHumidity() - 54.0f) < 0.001f);
	CHECK(fabs(((TempSensor&)dev).ReadTemperature() - 27.5f) < 0.001f);

	// Clamped humidity
	m.Sample(-20000, 700, HTS221_STATUS_H_DA);
	CHECK(dev.UpdateData() == true);
	dev.Read(hd);
	CHECK(hd.Humidity == 0);
	m.Sample(30000, 700, HTS221_STATUS_H_DA);
	CHECK(dev.UpdateData() == true);
	dev.Read(hd);
	CHECK(hd.Humidity == 10000);

	// Temperature only keeps the humidity, events per quantity
	int hev = s_HumiEvt, tev = s_TempEvt;

	m.Sample(750, -2000, HTS221_STATUS_T_DA);
	dev.SetEvtHandler(DevEvt);
	dev.IntHandler();
	CHECK(s_DevEvt == 1);
	CHECK(s_HumiEvt == hev && s_TempEvt == tev + 1);
	dev.Read(hd);
	dev.Read(td);
	CHECK(hd.Humidity == 10000);
	CHECK(td.Temperature == -2312 || td.Temperature == -2313);
	dev.IntHandler();
	CHECK(s_DevEvt == 1);

	// Rate through the base classes while running
	HumiSensor *ph = &dev;

	CHECK(ph->SamplingFrequency(1000) == 1000);
	CHECK((m.Reg[HTS221_CTRL_REG1_REG] & HTS221_CTRL_REG1_ODR_MASK) == HTS221_CTRL_REG1_ODR_1HZ);
	CHECK(((TempSensor&)dev).SamplingFrequency() == 1000);
	CHECK(ph->SamplingFrequency(0) == 1000);
	CHECK(ph->SamplingFrequency(7000) == 7000);
	CHECK((m.Reg[HTS221_CTRL_REG1_REG] & HTS221_CTRL_REG1_ODR_MASK) == HTS221_CTRL_REG1_ODR_7HZ);
	CHECK(ph->SamplingFrequency(20000) == 12500);
	CHECK((m.Reg[HTS221_CTRL_REG1_REG] & HTS221_CTRL_REG1_ODR_MASK) == HTS221_CTRL_REG1_ODR_12_5HZ);

	dev.Heater(true);
	CHECK(m.Reg[HTS221_CTRL_REG2_REG] == HTS221_CTRL_REG2_HEATER);
	dev.Heater(false);
	CHECK(m.Reg[HTS221_CTRL_REG2_REG] == 0);

	dev.Disable();
	CHECK(m.Reg[HTS221_CTRL_REG1_REG] == (HTS221_CTRL_REG1_BDU | HTS221_CTRL_REG1_ODR_12_5HZ));

	// A rate change while powered down waits for Enable
	CHECK(ph->SamplingFrequency(1000) == 1000);
	CHECK(m.Reg[HTS221_CTRL_REG1_REG] == (HTS221_CTRL_REG1_BDU | HTS221_CTRL_REG1_ODR_12_5HZ));
	CHECK(dev.Enable() == true);
	CHECK(m.Reg[HTS221_CTRL_REG1_REG] == (HTS221_CTRL_REG1_PD | HTS221_CTRL_REG1_BDU | HTS221_CTRL_REG1_ODR_1HZ));
}

static void TestOneShotSpi(void)
{
	Hts221Model m(DEVINTRF_TYPE_SPI);
	HumiHts221 dev;
	HumiSensorCfg_t hcfg = s_HumiCfg;
	TempSensorCfg_t tcfg = s_TempCfg;

	hcfg.OpMode = SENSOR_OPMODE_SINGLE;
	hcfg.HumOvrs = 0;
	tcfg.OpMode = SENSOR_OPMODE_SINGLE;
	tcfg.TempOvrs = 1000;
	tcfg.Inter = 0;
	CHECK(dev.Init(hcfg, &m, NULL) == true);
	CHECK(dev.Init(tcfg, &m, NULL) == true);
	CHECK(m.Reg[HTS221_AV_CONF_REG] == ((7 << HTS221_AV_CONF_AVGT_POS) | (3 << HTS221_AV_CONF_AVGH_POS)));
	CHECK(m.Reg[HTS221_CTRL_REG3_REG] == 0);

	CHECK(dev.Enable() == true);
	CHECK(m.Reg[HTS221_CTRL_REG1_REG] == (HTS221_CTRL_REG1_PD | HTS221_CTRL_REG1_BDU | HTS221_CTRL_REG1_ODR_ONE_SHOT));
	CHECK(dev.StartSampling() == true);
	CHECK(m.Reg[HTS221_CTRL_REG2_REG] == HTS221_CTRL_REG2_ONE_SHOT);
	// Measurement in progress
	CHECK(dev.StartSampling() == false);
	CheckSample(dev, m, 750, 700);
	CHECK(dev.StartSampling() == true);
	CheckSample(dev, m, -3000, 1500);

	// Averaging counts at a power of 2 select that count
	hcfg.HumOvrs = 64;
	tcfg.TempOvrs = 16;
	CHECK(dev.Init(hcfg, &m, NULL) == true);
	CHECK(dev.Init(tcfg, &m, NULL) == true);
	CHECK(m.Reg[HTS221_AV_CONF_REG] == ((3 << HTS221_AV_CONF_AVGT_POS) | (4 << HTS221_AV_CONF_AVGH_POS)));
}

static void TestFailures(void)
{
	HumiHts221 dev;

	// Wrong identification
	Hts221Model w(DEVINTRF_TYPE_I2C);

	w.Reg[HTS221_WHO_AM_I_REG] = 0xBD;
	CHECK(dev.Init(s_HumiCfg, &w, NULL) == false);
	CHECK(w.Writes == 0 && w.Boots == 0);

	// Calibration reload that does not end
	Hts221Model s(DEVINTRF_TYPE_I2C);

	s.StuckBoot = true;
	CHECK(dev.Init(s_HumiCfg, &s, NULL) == false);
	CHECK(dev.Valid() == false);

	// Two equal humidity points
	Hts221Model e(DEVINTRF_TYPE_I2C);

	e.Calibrate(100, 100, CAL_T0OUT, CAL_T1OUT);
	CHECK(dev.Init(s_HumiCfg, &e, NULL) == false);

	// Two equal temperature points
	Hts221Model t(DEVINTRF_TYPE_I2C);

	t.Calibrate(CAL_H0OUT, CAL_H1OUT, 500, 500);
	CHECK(dev.Init(s_TempCfg, &t, NULL) == false);
	CHECK(dev.Valid() == false);

	// A good device after the failures
	Hts221Model g(DEVINTRF_TYPE_I2C);

	CHECK(dev.Init(s_TempCfg, &g, NULL) == true);
	CHECK(dev.Valid() == true);
}

int main()
{
	TestI2c();
	TestOneShotSpi();
	TestFailures();

	printf("HTS221 driver model: %d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}

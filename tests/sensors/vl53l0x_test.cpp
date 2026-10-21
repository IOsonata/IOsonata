/**-------------------------------------------------------------------------
@file	vl53l0x_test.cpp

@brief	Host test of the VL53L0X distance sensor driver.

Runs src/sensors/dist_vl53l0x.cpp against a register model of the device on
I2C: detection, address change, soft reset, internal stop value, reference
SPAD selection, tuning settings, timing budget, reference calibrations,
continuous and single measurements, result conversion.

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
#include <cstdio>
#include <cstring>

#include "sensors/dist_vl53l0x.h"

#define MODEL_STOP_VARIABLE		0x3C
#define MODEL_SPAD_INFO			(0x80 | 5)		// Aperture type, 5 SPADs
#define MODEL_OSC_CALIBRATE		0x0420
#define MODEL_LOG_SIZE			1024

// Default tuning settings of the ST API, the expected write sequence
static const uint8_t s_Tuning[][2] = {
	{0xFF, 0x01}, {0x00, 0x00},
	{0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
	{0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00},
	{0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20},
	{0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
	{0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00}, {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30},
	{0x61, 0x00}, {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
	{0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF}, {0x4A, 0x00},
	{0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21},
	{0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
	{0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40},
	{0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},
	{0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04},
	{0xFF, 0x00}, {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04},
	{0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00},
	{0xFF, 0x01}, {0x0D, 0x01},
	{0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8},
	{0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01},
	{0xFF, 0x00}, {0x80, 0x00},
};

// Register model of the VL53L0X: 8 pages selected by register 0xFF
class Vl53l0xModel : public DeviceIntrf {
public:
	Vl53l0xModel(uint8_t I2cAddr = VL53L0X_I2C_DEV_ADDR) : vDev{} {
		Addr = I2cAddr;
		PowerOn();
	}
	void PowerOn() {
		memset(Reg, 0, sizeof(Reg));
		Page = 0;
		Reg[0][VL53L0X_IDENTIFICATION_MODEL_ID_REG] = VL53L0X_IDENTIFICATION_MODEL_ID;
		Reg[0][VL53L0X_IDENTIFICATION_REVISION_ID_REG] = 0x10;
		Reg[0][VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REG] = 0x11;
		Reg[0][VL53L0X_SOFT_RESET_GO2_SOFT_RESET_N_REG] = 1;
		Reg[0][VL53L0X_OSC_CALIBRATE_VAL_REG] = MODEL_OSC_CALIBRATE >> 8;
		Reg[0][VL53L0X_OSC_CALIBRATE_VAL_REG + 1] = MODEL_OSC_CALIBRATE & 0xFF;
		memset(&Reg[0][VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG], 0xFF, VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_SIZE);
		Reg[1][0x91] = MODEL_STOP_VARIABLE;
		Reg[7][0x92] = SpadInfo;
	}
	operator DevIntrf_t * () { return &vDev; }
	DEVINTRF_TYPE Type() { return DEVINTRF_TYPE_I2C; }
	uint32_t Rate(uint32_t DataRate) { return DataRate; }
	uint32_t Rate(void) { return 400000; }

	int Read(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen, uint8_t *pBuff, int BuffLen) {
		assert(AdCmdLen == 1);
		if (DevAddr != Addr)
		{
			return 0;
		}
		uint8_t a = pAdCmd[0];
		LastReadPage = Page;
		LastReadAddr = a;
		LastReadLen = BuffLen;
		Reads++;
		for (int i = 0; i < BuffLen; i++, a++)
		{
			if (bInReset)
			{
				pBuff[i] = 0;
				continue;
			}
			pBuff[i] = Reg[Page][a];
			if (Page == 7 && a == 0x83 && SpadReads > 0)
			{
				// Request in progress
				pBuff[i] = 0;
				if (SpadReads < 1000)
				{
					SpadReads--;
				}
				if (SpadReads == 0)
				{
					Reg[7][0x83] = 0x10;
				}
			}
			if (Page == 0 && a == VL53L0X_RESULT_INTERRUPT_STATUS_REG && CalReads > 0)
			{
				pBuff[i] = 0;
				if (CalReads < 1000)
				{
					CalReads--;
				}
				if (CalReads == 0)
				{
					Reg[0][a] = 7;
				}
			}
		}
		return BuffLen;
	}

	int Write(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen, const uint8_t *pData, int DataLen) {
		assert(AdCmdLen == 1);
		if (DevAddr != Addr)
		{
			return 0;
		}
		uint8_t a = pAdCmd[0];
		for (int i = 0; i < DataLen; i++, a++)
		{
			uint8_t v = pData[i];

			if (bInReset)
			{
				if (Page == 0 && a == VL53L0X_SOFT_RESET_GO2_SOFT_RESET_N_REG && v == 1)
				{
					uint8_t addr = Addr;

					bInReset = false;
					PowerOn();
					Addr = ResetToDefaultAddr ? VL53L0X_I2C_DEV_ADDR : addr;
				}
				continue;
			}
			if (LogLen < MODEL_LOG_SIZE)
			{
				Log[LogLen][0] = Page;
				Log[LogLen][1] = a;
				Log[LogLen][2] = v;
				LogLen++;
			}
			Writes++;
			if (a == VL53L0X_PAGE_SELECT_REG)
			{
				Page = v;
				assert(Page < 8);
				continue;
			}
			Reg[Page][a] = v;
			if (Page != 0)
			{
				if (Page == 7 && a == 0x83 && v == 0 && Reg[7][0x94] == 0x6B)
				{
					SpadReads = StuckSpad ? 1000 : 3;
				}
				continue;
			}
			switch (a)
			{
				case VL53L0X_SOFT_RESET_GO2_SOFT_RESET_N_REG:
					if (v == 0)
					{
						Resets++;
						bInReset = true;
						if (ResetToDefaultAddr)
						{
							Addr = VL53L0X_I2C_DEV_ADDR;
						}
					}
					break;
				case VL53L0X_I2C_SLAVE_DEVICE_ADDRESS_REG:
					Addr = v & 0x7F;
					Moves++;
					break;
				case VL53L0X_SYSTEM_INTERRUPT_CLEAR_REG:
					if (v & 1)
					{
						Reg[0][VL53L0X_RESULT_INTERRUPT_STATUS_REG] = 0;
						Clears++;
					}
					break;
				case VL53L0X_SYSRANGE_START_REG:
					if (v & VL53L0X_SYSRANGE_START_STARTSTOP)
					{
						uint8_t seq = Reg[0][VL53L0X_SYSTEM_SEQUENCE_CONFIG_REG];

						if (seq == VL53L0X_SYSTEM_SEQUENCE_CONFIG_VHV || seq == VL53L0X_SYSTEM_SEQUENCE_CONFIG_PHASE)
						{
							assert(NbCal < 4);
							Cal[NbCal][0] = seq;
							Cal[NbCal][1] = v;
							NbCal++;
							CalReads = StuckCal ? 1000 : 2;
						}
						else
						{
							Starts++;
							Running = 0;
						}
					}
					else if (v & (VL53L0X_SYSRANGE_START_BACKTOBACK | VL53L0X_SYSRANGE_START_TIMED))
					{
						Running = v;
					}
					break;
			}
		}
		return DataLen;
	}

	void Sample(uint16_t Distance, uint8_t DevStatus, uint16_t Signal, uint16_t Ambient) {
		uint8_t *r = &Reg[0][VL53L0X_RESULT_RANGE_STATUS_REG];

		memset(r, 0, VL53L0X_RESULT_SIZE);
		r[0] = (uint8_t)(DevStatus << VL53L0X_RANGE_STATUS_DEVICE_POS) | 0x01;
		r[VL53L0X_RESULT_SIGNAL_RATE] = Signal >> 8;
		r[VL53L0X_RESULT_SIGNAL_RATE + 1] = Signal & 0xFF;
		r[VL53L0X_RESULT_AMBIENT_RATE] = Ambient >> 8;
		r[VL53L0X_RESULT_AMBIENT_RATE + 1] = Ambient & 0xFF;
		r[VL53L0X_RESULT_DISTANCE] = Distance >> 8;
		r[VL53L0X_RESULT_DISTANCE + 1] = Distance & 0xFF;
		Reg[0][VL53L0X_RESULT_INTERRUPT_STATUS_REG] = VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_NEW_SAMPLE;
	}
	uint32_t Reg32(uint8_t A) {
		return ((uint32_t)Reg[0][A] << 24) | ((uint32_t)Reg[0][A + 1] << 16) | ((uint32_t)Reg[0][A + 2] << 8) | Reg[0][A + 3];
	}
	uint16_t Reg16(uint8_t A) {
		return (uint16_t)((Reg[0][A] << 8) | Reg[0][A + 1]);
	}
	// Index of Seq as consecutive writes in the log, -1 when absent
	int FindWrites(const uint8_t (*pSeq)[2], int Len) {
		for (int s = 0; s + Len <= LogLen; s++)
		{
			int i = 0;

			while (i < Len && Log[s + i][1] == pSeq[i][0] && Log[s + i][2] == pSeq[i][1])
			{
				i++;
			}
			if (i == Len)
			{
				return s;
			}
		}
		return -1;
	}

	uint8_t Reg[8][256];
	uint8_t SpadInfo = MODEL_SPAD_INFO;
	uint8_t Page = 0;
	uint8_t Addr;
	bool bInReset = false;
	bool ResetToDefaultAddr = true;
	bool StuckSpad = false;
	bool StuckCal = false;
	int SpadReads = 0;
	int CalReads = 0;
	int Resets = 0;
	int Moves = 0;
	int Clears = 0;
	int Writes = 0;
	int Reads = 0;
	int Starts = 0;
	uint8_t Running = 0;
	uint8_t Cal[4][2];
	int NbCal = 0;
	uint8_t Log[MODEL_LOG_SIZE][3];
	int LogLen = 0;
	int LastReadPage = -1;
	int LastReadAddr = -1;
	int LastReadLen = 0;

private:
	DevIntrf_t vDev;
};

// Interrupt state is protected in Device
class DistVl53l0xProbe : public DistVl53l0x {
public:
	bool IntEnabled() const { return InterruptEnabled(); }
};

static int s_Fail = 0;
static int s_Checks = 0;

#define CHECK(c)	do { s_Checks++; if (!(c)) { s_Fail++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static int s_DataEvt = 0, s_DevEvt = 0;

static void DataEvt(DistSensor * const pSensor, DistSensorData_t *pData)
{
	(void)pSensor;
	(void)pData;
	s_DataEvt++;
}

static void DevEvt(Device * const pDev, DEV_EVT Evt)
{
	(void)pDev;
	if (Evt == DEV_EVT_DATA_RDY)
	{
		s_DevEvt++;
	}
}

static const DistSensorCfg_t s_DistCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 10000,
	.TimingBudget = 0,
	.IntPrio = 6,
	.Inter = 0,
	.IntPol = DEVINTR_POL_LOW,
	.DataRdyCB = DataEvt,
};

// Budget of the ST default timeouts after the tuning, with the full sequence,
// and the final range timeouts for the ranging sequence (DSS, pre and final)
#define BUDGET_DEFAULT			33849
#define FINAL_TIMEOUT_DEFAULT	0x0290
#define FINAL_TIMEOUT_20MS		0x00D5
#define FINAL_TIMEOUT_50MS		0x02FA
#define FINAL_TIMEOUT_200MS		0x059A
#define BUDGET_MIN				17579

static void CheckSample(DistVl53l0x &Dev, Vl53l0xModel &M, uint16_t Dist, uint8_t DevStatus, uint16_t Signal,
						uint16_t Ambient, uint8_t Status)
{
	DistSensorData_t d;
	int clears = M.Clears;
	int evt = s_DataEvt;

	M.Sample(Dist, DevStatus, Signal, Ambient);
	CHECK(Dev.UpdateData() == true);
	CHECK(M.LastReadPage == 0 && M.LastReadAddr == VL53L0X_RESULT_RANGE_STATUS_REG &&
		  M.LastReadLen == VL53L0X_RESULT_SIZE);
	CHECK(M.Clears == clears + 1 && M.Reg[0][VL53L0X_RESULT_INTERRUPT_STATUS_REG] == 0);
	CHECK(s_DataEvt == evt + 1);
	Dev.Read(d);
	CHECK(d.Distance == Dist);
	CHECK(Dev.ReadDistance() == Dist);
	CHECK(d.Status == Status);
	CHECK(d.SignalRate == (uint32_t)Signal * 1000 / 128);
	CHECK(d.AmbientRate == (uint32_t)Ambient * 1000 / 128);
}

static void TestContinuous(void)
{
	Vl53l0xModel m;
	DistVl53l0xProbe dev;

	CHECK(dev.Init(s_DistCfg, &m, NULL) == true);
	CHECK(dev.DeviceAddress() == VL53L0X_I2C_DEV_ADDR);
	CHECK(dev.Type() == SENSOR_TYPE_DISTANCE);
	CHECK(m.Resets == 1 && m.Moves == 0);
	CHECK(m.Page == 0);
	CHECK(m.Reg[0][VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REG] & VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_2V8);
	CHECK(m.Reg[0][0x88] == 0);
	CHECK(m.Reg[0][0x80] == 0);
	// Final range rate limit 0.25 Mcps, written by the data init and the tuning
	CHECK(m.Reg[0][VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG] == 0 &&
		  m.Reg[0][VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG + 1] == 0x20);

	// Data init: limit checks off and 0.25 Mcps, full sequence, before the tuning
	const uint8_t datainit[][2] = {
		{VL53L0X_MSRC_CONFIG_CONTROL_REG, VL53L0X_MSRC_CONFIG_CONTROL_LIMIT_CHECKS_OFF},
		{VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG, 0},
		{VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG + 1, VL53L0X_SIGNAL_RATE_LIMIT_DEFAULT},
		{VL53L0X_SYSTEM_SEQUENCE_CONFIG_REG, 0xFF},
	};
	int di = m.FindWrites(datainit, 4);
	int tu = m.FindWrites(s_Tuning, sizeof(s_Tuning) / sizeof(s_Tuning[0]));

	CHECK(di >= 0);
	CHECK(tu > di);

	// Reference SPADs: 5 of the aperture type from SPAD 12
	CHECK(m.Reg[1][0x4F] == 0 && m.Reg[1][0x4E] == 0x2C);
	CHECK(m.Reg[0][VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT_REG] == 0xB4);
	const uint8_t spad[6] = { 0, 0xF0, 0x01, 0, 0, 0 };
	CHECK(memcmp(&m.Reg[0][VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG], spad, 6) == 0);
	CHECK(m.Reg[6][0x83] == 0 && m.Reg[7][0x81] == 0);

	// Interrupt on new sample, GPIO1 active low
	CHECK(m.Reg[0][VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_REG] == VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_NEW_SAMPLE);
	CHECK((m.Reg[0][VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REG] & VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_POL) == 0);
	CHECK(dev.IntEnabled() == false);

	// VHV then phase calibration, ranging sequence after
	CHECK(m.NbCal == 2);
	CHECK(m.Cal[0][0] == VL53L0X_SYSTEM_SEQUENCE_CONFIG_VHV && m.Cal[0][1] == 0x41);
	CHECK(m.Cal[1][0] == VL53L0X_SYSTEM_SEQUENCE_CONFIG_PHASE && m.Cal[1][1] == 0x01);
	CHECK(m.Reg[0][VL53L0X_SYSTEM_SEQUENCE_CONFIG_REG] == VL53L0X_SYSTEM_SEQUENCE_CONFIG_RANGING);
	CHECK(m.Reg[0][VL53L0X_SYSRANGE_START_REG] == 0);

	// Default budget kept with the ranging sequence
	CHECK(dev.TimingBudget() == BUDGET_DEFAULT);
	CHECK(m.Reg16(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG) == FINAL_TIMEOUT_DEFAULT);
	CHECK(dev.SamplingFrequency() == 10000);

	// Idle until Enable
	CHECK(m.Running == 0);
	CHECK(dev.Enable() == true);
	CHECK(m.Reg[1][0x91] == MODEL_STOP_VARIABLE);
	CHECK(m.Reg32(VL53L0X_SYSTEM_INTERMEASUREMENT_PERIOD_REG) == 100U * MODEL_OSC_CALIBRATE);
	CHECK(m.Running == VL53L0X_SYSRANGE_START_TIMED);
	CHECK(m.Page == 0 && m.Reg[0][0x80] == 0);

	CHECK(dev.UpdateData() == false);

	CheckSample(dev, m, 1234, VL53L0X_DEVICE_STATUS_RANGE_COMPLETE, 0x0A00, 0x0040, DISTSENSOR_STATUS_VALID);
	CheckSample(dev, m, 8190, 4, 0x0003, 0x0123, 4);
	CheckSample(dev, m, 0, 0, 0xFFFF, 0, VL53L0X_DEVICE_STATUS_NONE_CODE);
	CheckSample(dev, m, 65535, 14, 1, 0xFFFF, 14);

	dev.SetEvtHandler(DevEvt);
	m.Sample(500, VL53L0X_DEVICE_STATUS_RANGE_COMPLETE, 0x100, 0);
	dev.IntHandler();
	CHECK(s_DevEvt == 1);
	dev.IntHandler();
	CHECK(s_DevEvt == 1);

	// Budget: too short is refused
	CHECK(dev.TimingBudget(20000) == 20000);
	CHECK(m.Reg16(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG) == FINAL_TIMEOUT_20MS);
	CHECK(dev.TimingBudget(BUDGET_MIN - 1) == 20000);
	CHECK(m.Reg16(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG) == FINAL_TIMEOUT_20MS);
	CHECK(dev.TimingBudget(200000) == 200000);
	CHECK(m.Reg16(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG) == FINAL_TIMEOUT_200MS);

	// Stop: internal stop value cleared
	int starts = m.Starts;

	dev.Disable();
	CHECK(m.Starts == starts + 1 && m.Running == 0);
	CHECK(m.Reg[1][0x91] == 0);
	CHECK(m.Page == 0);

	// Rates, period in whole ms, back to back at 0
	CHECK(dev.SamplingFrequency(15000) == 14925);
	CHECK(dev.SamplingFrequency(30000) == 30303);
	CHECK(dev.Enable() == true);
	CHECK(m.Reg32(VL53L0X_SYSTEM_INTERMEASUREMENT_PERIOD_REG) == 33U * MODEL_OSC_CALIBRATE);
	CHECK(m.Reg[1][0x91] == MODEL_STOP_VARIABLE);
	CHECK(dev.SamplingFrequency(2000000) == 1000000);
	CHECK(dev.SamplingFrequency(1) == 1);
	CHECK(dev.SamplingFrequency(0) == 0);
	dev.Disable();
	CHECK(dev.Enable() == true);
	CHECK(m.Running == VL53L0X_SYSRANGE_START_BACKTOBACK);

	// A device without the oscillator value takes the period in ms
	m.Reg[0][VL53L0X_OSC_CALIBRATE_VAL_REG] = 0;
	m.Reg[0][VL53L0X_OSC_CALIBRATE_VAL_REG + 1] = 0;
	dev.Disable();
	CHECK(dev.SamplingFrequency(5000) == 5000);
	CHECK(dev.Enable() == true);
	CHECK(m.Reg32(VL53L0X_SYSTEM_INTERMEASUREMENT_PERIOD_REG) == 200);
}

static void TestSingle(void)
{
	Vl53l0xModel m;
	DistVl53l0xProbe dev;
	DistSensorCfg_t cfg = s_DistCfg;

	cfg.OpMode = SENSOR_OPMODE_SINGLE;
	cfg.TimingBudget = 50000;
	cfg.Inter = 1;
	cfg.IntPol = DEVINTR_POL_HIGH;
	dev.Io2v8(false);
	// 3 SPADs of the other type, from SPAD 0
	m.SpadInfo = 3;
	m.Reg[7][0x92] = 3;
	CHECK(dev.Init(cfg, &m, NULL) == true);
	const uint8_t spad[6] = { 0x07, 0, 0, 0, 0, 0 };
	CHECK(memcmp(&m.Reg[0][VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG], spad, 6) == 0);
	CHECK((m.Reg[0][VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REG] & VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_2V8) == 0);
	CHECK(m.Reg[0][VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REG] & VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_POL);
	CHECK(dev.IntEnabled() == true);
	CHECK(dev.TimingBudget() == 50000);
	CHECK(m.Reg16(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG) == FINAL_TIMEOUT_50MS);
	CHECK(dev.SamplingFrequency() == 0);

	int writes = m.Writes;

	CHECK(dev.Enable() == true);
	CHECK(m.Writes == writes && m.Running == 0);
	CHECK(dev.StartSampling() == true);
	CHECK(m.Starts == 1);
	CHECK(m.Reg[1][0x91] == MODEL_STOP_VARIABLE);
	CHECK(dev.UpdateData() == false);
	CheckSample(dev, m, 120, VL53L0X_DEVICE_STATUS_RANGE_COMPLETE, 0x2000, 0x10, DISTSENSOR_STATUS_VALID);
	m.Reg[1][0x91] = 0;
	CHECK(dev.StartSampling() == true);
	CHECK(m.Starts == 2);
	CHECK(m.Reg[1][0x91] == MODEL_STOP_VARIABLE);

	writes = m.Writes;
	dev.Disable();
	CHECK(m.Writes == writes);
}

static void TestAddress(void)
{
	DistSensorCfg_t cfg = s_DistCfg;

	cfg.DevAddr = 0x30;

	// Device at the power up address moved to the address asked
	Vl53l0xModel a;
	DistVl53l0x da;

	CHECK(da.Init(cfg, &a, NULL) == true);
	CHECK(a.Addr == 0x30 && da.DeviceAddress() == 0x30 && a.Moves == 1);

	// Already moved, the reset returns it to the power up address
	Vl53l0xModel b(0x30);
	DistVl53l0x db;

	CHECK(db.Init(cfg, &b, NULL) == true);
	CHECK(b.Resets == 1 && b.Moves == 1 && b.Addr == 0x30 && db.DeviceAddress() == 0x30);

	// Already moved, the reset keeps the address
	Vl53l0xModel c(0x30);
	DistVl53l0x dc;

	c.ResetToDefaultAddr = false;
	CHECK(dc.Init(cfg, &c, NULL) == true);
	CHECK(c.Resets == 1 && c.Moves == 0 && c.Addr == 0x30);

	// Power up address asked, no device there
	Vl53l0xModel d(0x31);
	DistVl53l0x dd;

	cfg.DevAddr = VL53L0X_I2C_DEV_ADDR;
	CHECK(dd.Init(cfg, &d, NULL) == false);
	CHECK(d.Writes == 0);
}

static void TestFailures(void)
{
	DistVl53l0x dev;

	// Wrong identification
	Vl53l0xModel w;

	w.Reg[0][VL53L0X_IDENTIFICATION_MODEL_ID_REG] = 0xEA;
	CHECK(dev.Init(s_DistCfg, &w, NULL) == false);
	CHECK(w.Writes == 0);

	// SPAD information that does not come
	Vl53l0xModel s;

	s.StuckSpad = true;
	CHECK(dev.Init(s_DistCfg, &s, NULL) == false);
	CHECK(dev.Valid() == false);

	// Calibration that does not end
	Vl53l0xModel c;

	c.StuckCal = true;
	CHECK(dev.Init(s_DistCfg, &c, NULL) == false);
	CHECK(dev.Valid() == false);

	// A good device after the failures
	Vl53l0xModel g;

	CHECK(dev.Init(s_DistCfg, &g, NULL) == true);
	CHECK(dev.Valid() == true);
}

// I2C only device
class SpiIntrf : public Vl53l0xModel {
public:
	DEVINTRF_TYPE Type() { return DEVINTRF_TYPE_SPI; }
};

static void TestSpi(void)
{
	SpiIntrf m;
	DistVl53l0x dev;

	CHECK(dev.Init(s_DistCfg, &m, NULL) == false);
	CHECK(m.Writes == 0 && m.Reads == 0);
}

int main()
{
	TestContinuous();
	TestSingle();
	TestAddress();
	TestFailures();
	TestSpi();

	printf("VL53L0X driver model: %d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}

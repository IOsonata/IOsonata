/**-------------------------------------------------------------------------
@file	device_test.cpp
@brief	Host tests for nRF5340 Audio DK portable device drivers.

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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "audio/cs47l63.h"
#include "pwrmgnt/pm_npm1100.h"
#include "sensors/ina230.h"

static int s_Pin[2][32];
static uint32_t s_DelayMs;

extern "C" void IOPinConfig(int PortNo, int PinNo, int PinOp,
							 IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	(void)PortNo;
	(void)PinNo;
	(void)PinOp;
	(void)Dir;
	(void)Resistor;
	(void)Type;
}

extern "C" int IOPinRead(int PortNo, int PinNo)
{
	return s_Pin[PortNo][PinNo];
}

extern "C" void IOPinSet(int PortNo, int PinNo)
{
	s_Pin[PortNo][PinNo] = 1;
}

extern "C" void IOPinClear(int PortNo, int PinNo)
{
	s_Pin[PortNo][PinNo] = 0;
}

extern "C" void IOPinDisable(int PortNo, int PinNo)
{
	(void)PortNo;
	(void)PinNo;
}

extern "C" void msDelay(uint32_t Delay)
{
	s_DelayMs += Delay;
}

extern "C" void usDelay(uint32_t Delay)
{
	(void)Delay;
}

class InaBus : public DeviceIntrf
{
public:
	uint16_t Reg[8]{};

	operator DevIntrf_t *()
	{
		return nullptr;
	}

	DEVINTRF_TYPE Type()
	{
		return DEVINTRF_TYPE_I2C;
	}

	uint32_t Rate(uint32_t DataRate)
	{
		return DataRate;
	}

	uint32_t Rate()
	{
		return 400000;
	}

	int Read(uint32_t Addr, const uint8_t *pReg, int RegLen,
			 uint8_t *pData, int DataLen)
	{
		uint16_t value;

		assert(Addr == 0x44);
		assert(RegLen == 1);
		assert(DataLen == 2);
		value = Reg[*pReg];
		if (*pReg == INA230_REG_CONFIG)
		{
			value |= 0x4000U;	// Reserved D14 reads as one on INA230.
		}
		pData[0] = uint8_t(value >> 8);
		pData[1] = uint8_t(value);

		return 2;
	}

	int Write(uint32_t Addr, const uint8_t *pReg, int RegLen,
			  const uint8_t *pData, int DataLen)
	{
		uint16_t value;

		assert(Addr == 0x44);
		assert(RegLen == 1);
		assert(DataLen == 2);
		value = (uint16_t(pData[0]) << 8) | pData[1];
		if (*pReg == INA230_REG_CONFIG && (value & INA230_CONFIG_RESET) != 0)
		{
			memset(Reg, 0, sizeof(Reg));
		}
		else
		{
			Reg[*pReg] = value;
		}

		return 2;
	}
};

class CodecBus : public DeviceIntrf
{
public:
	uint32_t Reg[30000]{};

	CodecBus()
	{
		Reg[CS47L63_REG_DEVID / 4] = CS47L63_DEVICE_ID;
		Reg[CS47L63_REG_REVID / 4] = 0x12;
		Reg[0x18014U / 4] = (1U << 3);
	}

	operator DevIntrf_t *()
	{
		return nullptr;
	}

	DEVINTRF_TYPE Type()
	{
		return DEVINTRF_TYPE_SPI;
	}

	uint32_t Rate(uint32_t DataRate)
	{
		return DataRate;
	}

	uint32_t Rate()
	{
		return 8000000;
	}

	int Read(uint32_t Addr, const uint8_t *pCmd, int CmdLen,
			 uint8_t *pData, int DataLen)
	{
		uint32_t address;
		int offset;
		uint32_t value;

		assert(Addr == 2);
		assert(CmdLen == 8);
		assert((DataLen & 3) == 0);
		assert(pCmd[4] == 0 && pCmd[5] == 0 && pCmd[6] == 0 && pCmd[7] == 0);
		address = (uint32_t(pCmd[0]) << 24) | (uint32_t(pCmd[1]) << 16) |
				  (uint32_t(pCmd[2]) << 8) | pCmd[3];
		assert((address & CS47L63_SPI_READ) != 0);
		address &= CS47L63_SPI_ADDR_MASK;
		for (offset = 0; offset < DataLen; offset += 4)
		{
			value = Reg[(address + uint32_t(offset)) / 4];
			pData[offset] = uint8_t(value >> 24);
			pData[offset + 1] = uint8_t(value >> 16);
			pData[offset + 2] = uint8_t(value >> 8);
			pData[offset + 3] = uint8_t(value);
		}

		return DataLen;
	}

	int Write(uint32_t Addr, const uint8_t *pCmd, int CmdLen,
			  const uint8_t *pData, int DataLen)
	{
		uint32_t address;
		int offset;
		uint32_t value;

		assert(Addr == 2);
		assert(CmdLen == 8);
		assert((DataLen & 3) == 0);
		assert(pCmd[4] == 0 && pCmd[5] == 0 && pCmd[6] == 0 && pCmd[7] == 0);
		address = (uint32_t(pCmd[0]) << 24) | (uint32_t(pCmd[1]) << 16) |
				  (uint32_t(pCmd[2]) << 8) | pCmd[3];
		assert((address & CS47L63_SPI_READ) == 0);
		for (offset = 0; offset < DataLen; offset += 4)
		{
			value = (uint32_t(pData[offset]) << 24) |
					(uint32_t(pData[offset + 1]) << 16) |
					(uint32_t(pData[offset + 2]) << 8) |
					pData[offset + 3];
			Reg[(address + uint32_t(offset)) / 4] = value;
		}

		return DataLen;
	}
};

static void TestIna230()
{
	InaBus bus;
	Ina230 monitor;
	Ina230Cfg_t cfg;
	uint16_t config;

	config = INA230_CONFIG_AVG_1024 |
			 INA230_CONFIG_VBUSCT_4156US |
			 INA230_CONFIG_VSHCT_4156US |
			 INA230_CONFIG_MODE_SHUNT_BUS_CONT;
	cfg.DevAddr = 0x44;
	cfg.RShuntMicroOhm = 510000;
	cfg.CurrentLsbMicroAmp = 1;
	cfg.Config = config;

	assert(monitor.Init(cfg, &bus));
	assert(monitor.Valid());
	assert(monitor.Calibration() == 10039);
	assert(bus.Reg[INA230_REG_CALIBRATION] == 10039);
	assert(bus.Reg[INA230_REG_CONFIG] == config);

	bus.Reg[INA230_REG_SHUNT_VOLTAGE] = 0xFFFE;
	bus.Reg[INA230_REG_BUS_VOLTAGE] = 1000;
	bus.Reg[INA230_REG_CURRENT] = uint16_t(int16_t(-100));
	bus.Reg[INA230_REG_POWER] = 10;
	assert(monitor.ShuntVoltageNanoVolt() == -5000);
	assert(monitor.BusVoltageMicroVolt() == 1250000);
	assert(monitor.CurrentMicroAmp() == -100);
	assert(monitor.PowerMicroWatt() == 250);

	monitor.Disable();
	assert((bus.Reg[INA230_REG_CONFIG] & 7) == 0);
	assert(monitor.Enable());
	assert(bus.Reg[INA230_REG_CONFIG] == config);
	monitor.Reset();
	assert(monitor.Valid());
	assert(bus.Reg[INA230_REG_CALIBRATION] == 10039);
	assert(bus.Reg[INA230_REG_CONFIG] == config);
	monitor.PowerOff();
	assert(!monitor.Valid());
}

static void TestNpm1100()
{
	PmnPM1100 pmic;
	PmnPM1100Cfg_t cfg;

	cfg.IsetPin = { 0, 23, IOPINOP_GPIO, IOPINDIR_OUTPUT,
					IOPINRES_NONE, IOPINTYPE_NORMAL };
	s_Pin[0][23] = 1;
	assert(pmic.Init(cfg));
	assert(pmic.Valid());
	assert(s_Pin[0][23] == 0);
	assert(pmic.InputCurrentLimit() == 100);
	assert(pmic.InputCurrentLimit(200) == 100);
	assert(s_Pin[0][23] == 0);
	assert(pmic.InputCurrentLimit(500) == 500);
	assert(s_Pin[0][23] == 1);
	assert(pmic.InputCurrentLimit(100) == 100);
	assert(s_Pin[0][23] == 0);
	pmic.Disable();
	assert(s_Pin[0][23] == 0);
	assert(pmic.Enable());
	assert(s_Pin[0][23] == 0);
	pmic.PowerOff();
	assert(!pmic.Valid());
}

static void TestCs47l63()
{
	CodecBus bus;
	Cs47l63 codec;
	Cs47l63Cfg_t cfg;
	Cs47l63RegVal_t sequence[2];
	uint32_t value;
	uint8_t block[8];

	memset(s_Pin, 0, sizeof(s_Pin));
	s_Pin[0][19] = 1; // IRQ pin can remain masked while BOOT_DONE latches.
	cfg.DevAddr = 2;
	cfg.ResetPin = { 0, 18, IOPINOP_GPIO, IOPINDIR_OUTPUT,
					 IOPINRES_NONE, IOPINTYPE_NORMAL };
	cfg.IrqPin = { 0, 19, IOPINOP_GPIO, IOPINDIR_INPUT,
				   IOPINRES_PULLUP, IOPINTYPE_NORMAL };
	cfg.BootTimeoutMs = 20;

	assert(codec.Init(cfg, &bus));
	assert(codec.Valid());
	assert(codec.DeviceID() == ((uint64_t(CS47L63_DEVICE_ID) << 8) | 0x12));
	assert(codec.Revision() == 0x12);
	assert(s_Pin[0][18] == 1);
	assert(s_DelayMs >= 2);

	assert(codec.WriteReg(0x20, 0x12345678));
	assert(codec.ReadReg(0x20, value));
	assert(value == 0x12345678);
	assert(codec.UpdateReg(0x20, 0x00FF0000, 0x005A0000));
	assert(codec.ReadReg(0x20, value));
	assert(value == 0x125A5678);

	block[0] = 0x01;
	block[1] = 0x02;
	block[2] = 0x03;
	block[3] = 0x04;
	block[4] = 0x11;
	block[5] = 0x12;
	block[6] = 0x13;
	block[7] = 0x14;
	assert(codec.WriteBlock(0x40, block, sizeof(block)));
	memset(block, 0, sizeof(block));
	assert(codec.ReadBlock(0x40, block, sizeof(block)));
	assert(block[0] == 0x01 && block[3] == 0x04);
	assert(block[4] == 0x11 && block[7] == 0x14);

	sequence[0] = { 0x30, 0xA5A50001, 0 };
	sequence[1] = { 0x34, 0x5A5A0002, 1 };
	assert(codec.Apply(sequence, 2));
	assert(bus.Reg[0x30 / 4] == 0xA5A50001);
	assert(bus.Reg[0x34 / 4] == 0x5A5A0002);

	Cs47l63AudioCfg_t audio = { 48000, CS47L63_INPUT_PDM, true, -12000 };
	assert(codec.ConfigureAudio(audio));
	assert(codec.AudioConfigured());
	assert(bus.Reg[0x1404 / 4] == 0x034C);
	assert(bus.Reg[0x6008 / 4] == 0x10100200);
	assert(bus.Reg[0x6000 / 4] == 0x30003);
	assert(bus.Reg[0x4020 / 4] == 0x50021);
	assert(bus.Reg[0x8200 / 4] == 0x800010);
	assert(bus.Reg[0x4818 / 4] == (0x200U | 104U));
	assert(codec.HeadphoneMute(true));
	assert((bus.Reg[0x4818 / 4] & 0x100U) != 0);
	assert(codec.HeadphoneMute(false));
	assert(codec.HeadphoneGain(-18000));
	assert((bus.Reg[0x4818 / 4] & 0xFFU) == 92U);
	assert(codec.SelectInput(CS47L63_INPUT_LINE));
	assert(bus.Reg[0x4060 / 4] == 0x50020);
	assert(bus.Reg[0x8200 / 4] == 0x800012);
	assert(codec.SelectInput(CS47L63_INPUT_OFF));
	assert(bus.Reg[0x4000 / 4] == 0);
	assert(codec.SelectInput(CS47L63_INPUT_PDM));

	codec.Disable();
	assert(s_Pin[0][18] == 0);
	assert(codec.Enable());
	assert(s_Pin[0][18] == 1);
	assert(codec.AudioConfigured());
	assert(bus.Reg[0x6000 / 4] == 0x30003);
	codec.Reset();
	assert(codec.Valid());
	codec.PowerOff();
	assert(!codec.Valid());
	assert(s_Pin[0][18] == 0);
}

int main()
{
	TestIna230();
	TestNpm1100();
	TestCs47l63();
	printf("nRF5340 Audio DK device drivers: PASS\n");
	return 0;
}

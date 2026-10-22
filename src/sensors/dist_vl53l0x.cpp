/**-------------------------------------------------------------------------
@file	dist_vl53l0x.cpp

@brief	ST VL53L0X time of flight distance sensor implementation

Init follows the ST VL53L0X API (STSW-IMG005): data init, reference SPAD
selection, default tuning settings, timing budget, then the VHV and phase
reference calibrations. The calibration of the device for a cover glass
(offset and cross talk) is not done.

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
#include "sensors/dist_vl53l0x.h"

// Bound of the reads that wait for the device
#define VL53L0X_POLL_MAX				2000

// Registers of the internal pages, selected by VL53L0X_PAGE_SELECT_REG
#define VL53L0X_INTERNAL_POWER_REG		0x80
#define VL53L0X_INTERNAL_STOP_REG		0x91
#define VL53L0X_INTERNAL_I2C_MODE_REG	0x88
#define VL53L0X_SPAD_INFO_REQ_REG		0x83
#define VL53L0X_SPAD_INFO_REG			0x92
#define VL53L0X_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REG	0x4E
#define VL53L0X_DYNAMIC_SPAD_REF_EN_START_OFFSET_REG	0x4F
#define VL53L0X_SPAD_REF_REQUESTED		0x2C
#define VL53L0X_SPAD_REF_START_SELECT	0xB4
#define VL53L0X_SPAD_APERTURE_FIRST		12		//!< First reference SPAD of the aperture type
#define VL53L0X_SPAD_COUNT				48

// Overheads of the sequence steps, usec
#define VL53L0X_START_OVERHEAD			1910
#define VL53L0X_END_OVERHEAD			960
#define VL53L0X_MSRC_OVERHEAD			660
#define VL53L0X_TCC_OVERHEAD			590
#define VL53L0X_DSS_OVERHEAD			690
#define VL53L0X_PRE_RANGE_OVERHEAD		660
#define VL53L0X_FINAL_RANGE_OVERHEAD	550

// Default tuning settings of the ST API, register and value, in order. 0xFF
// selects the page of the registers that follow.
static const uint8_t s_Vl53l0xTuning[][2] = {
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

// VCSEL period in PCLKs from its register
static inline uint32_t Vl53l0xPclks(uint8_t Reg)
{
	return ((uint32_t)Reg + 1) << 1;
}

// Macro period in ns for a VCSEL period in PCLKs
static inline uint32_t Vl53l0xMacroNs(uint32_t Pclks)
{
	return (2304U * Pclks * 1655U + 500U) / 1000U;
}

static inline uint32_t Vl53l0xMclksToUs(uint32_t Mclks, uint32_t Pclks)
{
	return (Mclks * Vl53l0xMacroNs(Pclks) + 500U) / 1000U;
}

static inline uint32_t Vl53l0xUsToMclks(uint32_t Us, uint32_t Pclks)
{
	uint32_t ns = Vl53l0xMacroNs(Pclks);

	return (Us * 1000U + ns / 2U) / ns;
}

// Timeout registers hold (LSB << MSB) + 1 macro periods
static inline uint32_t Vl53l0xDecode(uint16_t Reg)
{
	return ((uint32_t)(Reg & 0xFF) << (Reg >> 8)) + 1;
}

static inline uint16_t Vl53l0xEncode(uint32_t Mclks)
{
	if (Mclks == 0)
	{
		return 0;
	}

	uint32_t ls = Mclks - 1;
	uint16_t ms = 0;

	while (ls & 0xFFFFFF00U)
	{
		ls >>= 1;
		ms++;
	}

	return (uint16_t)((ms << 8) | (ls & 0xFF));
}

DistVl53l0x::DistVl53l0x()
{
	vStopVariable = 0;
	vbIo2v8 = true;
	vTimingBudget = VL53L0X_TIMING_BUDGET_DEFAULT;
	vPeriodMs = 0;
	memset(&vData, 0, sizeof(vData));
	vDataRdyHandler = NULL;
}

uint8_t DistVl53l0x::ModelId()
{
	uint8_t regaddr = VL53L0X_IDENTIFICATION_MODEL_ID_REG;

	return Read8(&regaddr, 1);
}

bool DistVl53l0x::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Valid())
	{
		return true;
	}

	if (pIntrf == NULL || pIntrf->Type() != DEVINTRF_TYPE_I2C)
	{
		// I2C only device
		return false;
	}

	Interface(pIntrf);

	uint32_t addr = DevAddr == 0 ? VL53L0X_I2C_DEV_ADDR : DevAddr;

	// At the address asked, else at the power up address
	DeviceAddress(addr);
	if (ModelId() != VL53L0X_IDENTIFICATION_MODEL_ID)
	{
		if (addr == VL53L0X_I2C_DEV_ADDR)
		{
			return false;
		}
		DeviceAddress(VL53L0X_I2C_DEV_ADDR);
		if (ModelId() != VL53L0X_IDENTIFICATION_MODEL_ID)
		{
			return false;
		}
	}

	Reset();

	// The reset may return the device to its power up address
	if (ModelId() != VL53L0X_IDENTIFICATION_MODEL_ID)
	{
		DeviceAddress(VL53L0X_I2C_DEV_ADDR);
		if (ModelId() != VL53L0X_IDENTIFICATION_MODEL_ID)
		{
			return false;
		}
	}

	if (DeviceAddress() != addr)
	{
		uint8_t regaddr = VL53L0X_I2C_SLAVE_DEVICE_ADDRESS_REG;

		Write8(&regaddr, 1, (uint8_t)(addr & 0x7F));
		DeviceAddress(addr);
		if (ModelId() != VL53L0X_IDENTIFICATION_MODEL_ID)
		{
			return false;
		}
	}

	DeviceID(VL53L0X_IDENTIFICATION_MODEL_ID);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	if (DataInit() == false || SpadInit() == false)
	{
		return false;
	}

	for (size_t i = 0; i < sizeof(s_Vl53l0xTuning) / sizeof(s_Vl53l0xTuning[0]); i++)
	{
		uint8_t regaddr = s_Vl53l0xTuning[i][0];

		Write8(&regaddr, 1, s_Vl53l0xTuning[i][1]);
	}

	// Interrupt on each new sample, GPIO1 active low
	uint8_t regaddr = VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_REG;
	Write8(&regaddr, 1, VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_NEW_SAMPLE);

	regaddr = VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REG;
	uint8_t d = Read8(&regaddr, 1);
	Write8(&regaddr, 1, d & ~VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_POL);

	regaddr = VL53L0X_SYSTEM_INTERRUPT_CLEAR_REG;
	Write8(&regaddr, 1, 1);

	// Budget of the full sequence, kept with the ranging sequence
	uint32_t premclks, finalus;
	uint32_t used = SequenceTime(0xFF, premclks, finalus);

	vTimingBudget = used + VL53L0X_FINAL_RANGE_OVERHEAD + finalus;

	regaddr = VL53L0X_SYSTEM_SEQUENCE_CONFIG_REG;
	Write8(&regaddr, 1, VL53L0X_SYSTEM_SEQUENCE_CONFIG_RANGING);

	TimingBudget(vTimingBudget);

	if (RefCalibration(VL53L0X_SYSTEM_SEQUENCE_CONFIG_VHV, 0x40) == false ||
		RefCalibration(VL53L0X_SYSTEM_SEQUENCE_CONFIG_PHASE, 0) == false)
	{
		return false;
	}

	regaddr = VL53L0X_SYSTEM_SEQUENCE_CONFIG_REG;
	Write8(&regaddr, 1, VL53L0X_SYSTEM_SEQUENCE_CONFIG_RANGING);

	Valid(true);

	return true;
}

bool DistVl53l0x::Init(const DistSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
	{
		return false;
	}

	vOpMode = Cfg.OpMode;
	vDataRdyHandler = Cfg.DataRdyCB;

	if (Cfg.TimingBudget != 0)
	{
		TimingBudget(Cfg.TimingBudget);
	}

	uint8_t regaddr = VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REG;
	uint8_t d = Read8(&regaddr, 1) & ~VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_POL;

	if (Cfg.IntPol == DEVINTR_POL_HIGH)
	{
		d |= VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_POL;
	}
	Write8(&regaddr, 1, d);

	if (Cfg.Inter)
	{
		InterruptId(Cfg.Inter);
		InterruptEnabled(true);
	}

	SamplingFrequency(Cfg.OpMode == SENSOR_OPMODE_SINGLE ? 0 : Cfg.Freq);

	return true;
}

bool DistVl53l0x::DataInit()
{
	uint8_t regaddr;
	uint8_t d;

	if (vbIo2v8)
	{
		regaddr = VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REG;
		d = Read8(&regaddr, 1);
		Write8(&regaddr, 1, d | VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_2V8);
	}

	// I2C standard mode
	regaddr = VL53L0X_INTERNAL_I2C_MODE_REG;
	Write8(&regaddr, 1, 0);

	// Internal stop value, written back before each start
	const uint8_t open[][2] = {
		{VL53L0X_INTERNAL_POWER_REG, 1}, {VL53L0X_PAGE_SELECT_REG, 1}, {0x00, 0},
	};
	const uint8_t close[][2] = {
		{0x00, 1}, {VL53L0X_PAGE_SELECT_REG, 0}, {VL53L0X_INTERNAL_POWER_REG, 0},
	};

	for (auto &w : open)
	{
		regaddr = w[0];
		Write8(&regaddr, 1, w[1]);
	}
	regaddr = VL53L0X_INTERNAL_STOP_REG;
	vStopVariable = Read8(&regaddr, 1);
	for (auto &w : close)
	{
		regaddr = w[0];
		Write8(&regaddr, 1, w[1]);
	}

	regaddr = VL53L0X_MSRC_CONFIG_CONTROL_REG;
	d = Read8(&regaddr, 1);
	Write8(&regaddr, 1, d | VL53L0X_MSRC_CONFIG_CONTROL_LIMIT_CHECKS_OFF);

	Reg16(VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG, VL53L0X_SIGNAL_RATE_LIMIT_DEFAULT);

	regaddr = VL53L0X_SYSTEM_SEQUENCE_CONFIG_REG;

	return Write8(&regaddr, 1, 0xFF);
}

bool DistVl53l0x::SpadInit()
{
	uint8_t regaddr;
	uint8_t d;

	// Count and type of the reference SPADs, from the internal pages
	const uint8_t req[][2] = {
		{VL53L0X_INTERNAL_POWER_REG, 1}, {VL53L0X_PAGE_SELECT_REG, 1}, {0x00, 0}, {VL53L0X_PAGE_SELECT_REG, 6},
	};

	for (auto &w : req)
	{
		regaddr = w[0];
		Write8(&regaddr, 1, w[1]);
	}

	regaddr = VL53L0X_SPAD_INFO_REQ_REG;
	d = Read8(&regaddr, 1);
	Write8(&regaddr, 1, d | 4);

	const uint8_t req2[][2] = {
		{VL53L0X_PAGE_SELECT_REG, 7}, {0x81, 1}, {VL53L0X_INTERNAL_POWER_REG, 1}, {0x94, 0x6B},
		{VL53L0X_SPAD_INFO_REQ_REG, 0},
	};

	for (auto &w : req2)
	{
		regaddr = w[0];
		Write8(&regaddr, 1, w[1]);
	}

	bool ready = false;

	regaddr = VL53L0X_SPAD_INFO_REQ_REG;
	for (int i = 0; i < VL53L0X_POLL_MAX && ready == false; i++)
	{
		ready = Read8(&regaddr, 1) != 0;
	}

	if (ready == false)
	{
		return false;
	}

	Write8(&regaddr, 1, 1);

	regaddr = VL53L0X_SPAD_INFO_REG;
	uint8_t info = Read8(&regaddr, 1);
	int count = info & 0x7F;
	bool aperture = (info & 0x80) != 0;

	const uint8_t rel[][2] = {
		{0x81, 0}, {VL53L0X_PAGE_SELECT_REG, 6},
	};

	for (auto &w : rel)
	{
		regaddr = w[0];
		Write8(&regaddr, 1, w[1]);
	}

	regaddr = VL53L0X_SPAD_INFO_REQ_REG;
	d = Read8(&regaddr, 1);
	Write8(&regaddr, 1, d & ~4);

	const uint8_t rel2[][2] = {
		{VL53L0X_PAGE_SELECT_REG, 1}, {0x00, 1}, {VL53L0X_PAGE_SELECT_REG, 0}, {VL53L0X_INTERNAL_POWER_REG, 0},
	};

	for (auto &w : rel2)
	{
		regaddr = w[0];
		Write8(&regaddr, 1, w[1]);
	}

	// Enable count reference SPADs of the type, from the first available
	uint8_t map[VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_SIZE];

	regaddr = VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG;
	if (Device::Read(&regaddr, 1, map, sizeof(map)) != (int)sizeof(map))
	{
		return false;
	}

	const uint8_t ref[][2] = {
		{VL53L0X_PAGE_SELECT_REG, 1}, {VL53L0X_DYNAMIC_SPAD_REF_EN_START_OFFSET_REG, 0},
		{VL53L0X_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REG, VL53L0X_SPAD_REF_REQUESTED},
		{VL53L0X_PAGE_SELECT_REG, 0}, {VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT_REG, VL53L0X_SPAD_REF_START_SELECT},
	};

	for (auto &w : ref)
	{
		regaddr = w[0];
		Write8(&regaddr, 1, w[1]);
	}

	int first = aperture ? VL53L0X_SPAD_APERTURE_FIRST : 0;
	int enabled = 0;

	for (int i = 0; i < VL53L0X_SPAD_COUNT; i++)
	{
		uint8_t bit = 1 << (i & 7);

		if (i < first || enabled == count)
		{
			map[i >> 3] &= ~bit;
		}
		else if (map[i >> 3] & bit)
		{
			enabled++;
		}
	}

	regaddr = VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG;

	return Device::Write(&regaddr, 1, map, sizeof(map)) == (int)sizeof(map);
}

bool DistVl53l0x::RefCalibration(uint8_t SeqConfig, uint8_t VhvInit)
{
	uint8_t regaddr = VL53L0X_SYSTEM_SEQUENCE_CONFIG_REG;

	Write8(&regaddr, 1, SeqConfig);

	regaddr = VL53L0X_SYSRANGE_START_REG;
	Write8(&regaddr, 1, VL53L0X_SYSRANGE_START_STARTSTOP | VhvInit);

	bool done = false;

	regaddr = VL53L0X_RESULT_INTERRUPT_STATUS_REG;
	for (int i = 0; i < VL53L0X_POLL_MAX && done == false; i++)
	{
		done = (Read8(&regaddr, 1) & VL53L0X_RESULT_INTERRUPT_STATUS_MASK) != 0;
	}

	regaddr = VL53L0X_SYSTEM_INTERRUPT_CLEAR_REG;
	Write8(&regaddr, 1, 1);

	regaddr = VL53L0X_SYSRANGE_START_REG;
	Write8(&regaddr, 1, 0);

	return done;
}

void DistVl53l0x::StopVariable(uint8_t Val)
{
	const uint8_t seq[][2] = {
		{VL53L0X_INTERNAL_POWER_REG, 1}, {VL53L0X_PAGE_SELECT_REG, 1}, {0x00, 0},
		{VL53L0X_INTERNAL_STOP_REG, Val},
		{0x00, 1}, {VL53L0X_PAGE_SELECT_REG, 0}, {VL53L0X_INTERNAL_POWER_REG, 0},
	};

	for (auto &w : seq)
	{
		uint8_t regaddr = w[0];

		Write8(&regaddr, 1, w[1]);
	}
}

uint32_t DistVl53l0x::SequenceTime(uint8_t SeqConfig, uint32_t &PreMclks, uint32_t &FinalUs)
{
	uint8_t regaddr = VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD_REG;
	uint32_t prepclks = Vl53l0xPclks(Read8(&regaddr, 1));

	regaddr = VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP_REG;
	uint32_t msrcus = Vl53l0xMclksToUs((uint32_t)Read8(&regaddr, 1) + 1, prepclks);

	PreMclks = Vl53l0xDecode(Reg16(VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG));

	uint32_t preus = Vl53l0xMclksToUs(PreMclks, prepclks);

	regaddr = VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD_REG;
	uint32_t finalpclks = Vl53l0xPclks(Read8(&regaddr, 1));
	uint32_t finalmclks = Vl53l0xDecode(Reg16(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG));

	if (SeqConfig & VL53L0X_SYSTEM_SEQUENCE_CONFIG_PRE_RANGE)
	{
		// The final range timeout includes the pre range
		finalmclks -= PreMclks;
	}

	FinalUs = Vl53l0xMclksToUs(finalmclks, finalpclks);

	uint32_t used = VL53L0X_START_OVERHEAD + VL53L0X_END_OVERHEAD;

	if (SeqConfig & VL53L0X_SYSTEM_SEQUENCE_CONFIG_TCC)
	{
		used += msrcus + VL53L0X_TCC_OVERHEAD;
	}

	if (SeqConfig & VL53L0X_SYSTEM_SEQUENCE_CONFIG_DSS)
	{
		used += 2 * (msrcus + VL53L0X_DSS_OVERHEAD);
	}
	else if (SeqConfig & VL53L0X_SYSTEM_SEQUENCE_CONFIG_MSRC)
	{
		used += msrcus + VL53L0X_MSRC_OVERHEAD;
	}

	if (SeqConfig & VL53L0X_SYSTEM_SEQUENCE_CONFIG_PRE_RANGE)
	{
		used += preus + VL53L0X_PRE_RANGE_OVERHEAD;
	}

	return used;
}

uint32_t DistVl53l0x::TimingBudget(uint32_t Us)
{
	uint8_t regaddr = VL53L0X_SYSTEM_SEQUENCE_CONFIG_REG;
	uint8_t seq = Read8(&regaddr, 1);
	uint32_t premclks, finalus;
	uint32_t used = SequenceTime(seq, premclks, finalus);

	if ((seq & VL53L0X_SYSTEM_SEQUENCE_CONFIG_FINAL_RANGE) == 0)
	{
		return vTimingBudget;
	}

	used += VL53L0X_FINAL_RANGE_OVERHEAD;
	if (used > Us)
	{
		// Too short for the steps of the sequence
		return vTimingBudget;
	}

	regaddr = VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD_REG;
	uint32_t finalpclks = Vl53l0xPclks(Read8(&regaddr, 1));
	uint32_t finalmclks = Vl53l0xUsToMclks(Us - used, finalpclks);

	if (seq & VL53L0X_SYSTEM_SEQUENCE_CONFIG_PRE_RANGE)
	{
		finalmclks += premclks;
	}

	Reg16(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG, Vl53l0xEncode(finalmclks));
	vTimingBudget = Us;

	return vTimingBudget;
}

uint32_t DistVl53l0x::SamplingFrequency(uint32_t Freq)
{
	if (Freq == 0)
	{
		// Back to back
		vPeriodMs = 0;

		return DistSensor::SamplingFrequency(0);
	}

	// Period in ms rounded, at least 1
	vPeriodMs = (1000000U + Freq / 2U) / Freq;
	if (vPeriodMs == 0)
	{
		vPeriodMs = 1;
	}

	return DistSensor::SamplingFrequency(1000000U / vPeriodMs);
}

bool DistVl53l0x::Enable()
{
	if (vOpMode == SENSOR_OPMODE_SINGLE)
	{
		// StartSampling starts each measurement
		return true;
	}

	StopVariable(vStopVariable);

	uint8_t regaddr = VL53L0X_SYSRANGE_START_REG;

	if (vPeriodMs == 0)
	{
		return Write8(&regaddr, 1, VL53L0X_SYSRANGE_START_BACKTOBACK);
	}

	// Period in oscillator periods when the device gives their number per ms
	uint32_t period = vPeriodMs;
	uint16_t osc = Reg16(VL53L0X_OSC_CALIBRATE_VAL_REG);

	if (osc != 0)
	{
		period *= osc;
	}
	Reg32(VL53L0X_SYSTEM_INTERMEASUREMENT_PERIOD_REG, period);

	return Write8(&regaddr, 1, VL53L0X_SYSRANGE_START_TIMED);
}

void DistVl53l0x::Disable()
{
	if (vOpMode == SENSOR_OPMODE_SINGLE)
	{
		// Nothing runs between measurements
		return;
	}

	const uint8_t seq[][2] = {
		{VL53L0X_SYSRANGE_START_REG, VL53L0X_SYSRANGE_START_STARTSTOP},
		{VL53L0X_PAGE_SELECT_REG, 1}, {0x00, 0}, {VL53L0X_INTERNAL_STOP_REG, 0},
		{0x00, 1}, {VL53L0X_PAGE_SELECT_REG, 0},
	};

	for (auto &w : seq)
	{
		uint8_t regaddr = w[0];

		Write8(&regaddr, 1, w[1]);
	}
}

void DistVl53l0x::Reset()
{
	uint8_t regaddr = VL53L0X_SOFT_RESET_GO2_SOFT_RESET_N_REG;
	uint32_t addr = DeviceAddress();

	Write8(&regaddr, 1, 0);

	// The revision reads 0 while in reset
	regaddr = VL53L0X_IDENTIFICATION_REVISION_ID_REG;
	for (int i = 0; i < VL53L0X_POLL_MAX; i++)
	{
		if (Read8(&regaddr, 1) == 0)
		{
			break;
		}
	}

	regaddr = VL53L0X_SOFT_RESET_GO2_SOFT_RESET_N_REG;
	Write8(&regaddr, 1, 1);

	if (addr != VL53L0X_I2C_DEV_ADDR)
	{
		// Also at the power up address, in case the reset returned the device
		// there. 1 is the running value of a device already out of reset.
		DeviceAddress(VL53L0X_I2C_DEV_ADDR);
		Write8(&regaddr, 1, 1);
		DeviceAddress(addr);
	}

	regaddr = VL53L0X_IDENTIFICATION_REVISION_ID_REG;
	for (int i = 0; i < VL53L0X_POLL_MAX; i++)
	{
		if (Read8(&regaddr, 1) != 0)
		{
			break;
		}
	}
}

bool DistVl53l0x::StartSampling()
{
	if (vOpMode != SENSOR_OPMODE_SINGLE)
	{
		// Measures continuously
		return true;
	}

	StopVariable(vStopVariable);

	uint8_t regaddr = VL53L0X_SYSRANGE_START_REG;

	return Write8(&regaddr, 1, VL53L0X_SYSRANGE_START_STARTSTOP);
}

bool DistVl53l0x::UpdateData()
{
	uint8_t regaddr = VL53L0X_RESULT_INTERRUPT_STATUS_REG;

	if ((Read8(&regaddr, 1) & VL53L0X_RESULT_INTERRUPT_STATUS_MASK) == 0)
	{
		return false;
	}

	uint8_t r[VL53L0X_RESULT_SIZE];

	regaddr = VL53L0X_RESULT_RANGE_STATUS_REG;
	if (Device::Read(&regaddr, 1, r, sizeof(r)) != (int)sizeof(r))
	{
		return false;
	}

	regaddr = VL53L0X_SYSTEM_INTERRUPT_CLEAR_REG;
	Write8(&regaddr, 1, 1);

	uint8_t status = (r[0] & VL53L0X_RANGE_STATUS_DEVICE_MASK) >> VL53L0X_RANGE_STATUS_DEVICE_POS;
	uint32_t distance = ((uint32_t)r[VL53L0X_RESULT_DISTANCE] << 8) | r[VL53L0X_RESULT_DISTANCE + 1];

	if (status == VL53L0X_DEVICE_STATUS_RANGE_COMPLETE)
	{
		// On the B-L475E-IOT01A2, a few measurements without a target end
		// complete with 0 mm and the signal of no target
		status = distance == 0 ? VL53L0X_DEVICE_STATUS_ZERO_CODE : DISTSENSOR_STATUS_VALID;
	}
	else if (status == 0)
	{
		status = VL53L0X_DEVICE_STATUS_NONE_CODE;
	}

	// kcps from Mcps 9.7 fixed point: x 1000 / 128
	vData.Timestamp = vpTimer != NULL ? vpTimer->uSecond() : 0;
	vData.Distance = distance;
	vData.SignalRate = ((((uint32_t)r[VL53L0X_RESULT_SIGNAL_RATE] << 8) | r[VL53L0X_RESULT_SIGNAL_RATE + 1]) * 125U) / 16U;
	vData.AmbientRate = ((((uint32_t)r[VL53L0X_RESULT_AMBIENT_RATE] << 8) | r[VL53L0X_RESULT_AMBIENT_RATE + 1]) * 125U) / 16U;
	vData.Status = status;
	vSampleCnt++;

	if (vDataRdyHandler != NULL)
	{
		vDataRdyHandler(this, &vData);
	}

	return true;
}

void DistVl53l0x::IntHandler()
{
	if (UpdateData() == true && Device::vEvtHandler != NULL)
	{
		Device::vEvtHandler(this, DEV_EVT_DATA_RDY);
	}
}

uint16_t DistVl53l0x::Reg16(uint8_t Reg)
{
	uint8_t d[2] = {0, 0};

	Device::Read(&Reg, 1, d, 2);

	return (uint16_t)((d[0] << 8) | d[1]);
}

bool DistVl53l0x::Reg16(uint8_t Reg, uint16_t Val)
{
	uint8_t d[2] = { (uint8_t)(Val >> 8), (uint8_t)Val };

	return Device::Write(&Reg, 1, d, 2) == 2;
}

bool DistVl53l0x::Reg32(uint8_t Reg, uint32_t Val)
{
	uint8_t d[4] = { (uint8_t)(Val >> 24), (uint8_t)(Val >> 16), (uint8_t)(Val >> 8), (uint8_t)Val };

	return Device::Write(&Reg, 1, d, 4) == 4;
}

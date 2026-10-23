/**-------------------------------------------------------------------------
@file	accel_adxl362.cpp

@brief	Implementation of Analog ADXL362 accelerometer

@author	Hoang Nguyen Hoan
@date	July 13, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#include "sensors/accel_adxl362.h"

bool AccelAdxl362::ReadRegisters(uint8_t Reg, uint8_t *pData, int Len)
{
	uint8_t cmd[2] = { ADXL362_CMD_READ, Reg };

	return Interface() != NULL && Interface()->Read(DeviceAddress(), cmd, 2, pData, Len) == Len;
}

bool AccelAdxl362::WriteRegister(uint8_t Reg, uint8_t Value)
{
	uint8_t cmd[3] = { ADXL362_CMD_WRITE, Reg, Value };

	return Interface() != NULL && Interface()->Tx(DeviceAddress(), cmd, 3) == 3;
}

bool AccelAdxl362::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint8_t id[3];
	uint8_t filter;
	uint8_t range;
	uint32_t freq;

	if (vbInitialized)
		return true;
	if (pIntrf == NULL)
		return false;

	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vpTimer = pTimer;
	if (!ReadRegisters(ADXL362_DEVID_AD_REG, id, sizeof(id)))
		return false;
	if (id[0] != ADXL362_DEVID_AD || id[1] != ADXL362_DEVID_MST || id[2] != ADXL362_PARTID)
		return false;

	// Freq is in mHz. Select the lowest supported rate >= requested rate.
	if (Cfg.Freq <= 12500)
	{
		freq = 12500;
		filter = ADXL362_FILTER_CTL_ODR_12_5HZ;
	}
	else if (Cfg.Freq <= 25000)
	{
		freq = 25000;
		filter = ADXL362_FILTER_CTL_ODR_25HZ;
	}
	else if (Cfg.Freq <= 50000)
	{
		freq = 50000;
		filter = ADXL362_FILTER_CTL_ODR_50HZ;
	}
	else if (Cfg.Freq <= 100000)
	{
		freq = 100000;
		filter = ADXL362_FILTER_CTL_ODR_100HZ;
	}
	else if (Cfg.Freq <= 200000)
	{
		freq = 200000;
		filter = ADXL362_FILTER_CTL_ODR_200HZ;
	}
	else
	{
		freq = 400000;
		filter = ADXL362_FILTER_CTL_ODR_400HZ;
	}

	if (Cfg.Scale <= 2)
	{
		range = ADXL362_FILTER_CTL_RANGE_2G;
	}
	else if (Cfg.Scale <= 4)
	{
		range = ADXL362_FILTER_CTL_RANGE_4G;
	}
	else
	{
		range = ADXL362_FILTER_CTL_RANGE_8G;
	}
	filter |= ADXL362_FILTER_CTL_HALF_BW | range;

	if (!WriteRegister(ADXL362_POWER_CTL_REG, ADXL362_POWER_CTL_MEASURE_STANDBY))
		return false;
	if (!WriteRegister(ADXL362_FILTER_CTL_REG, filter))
		return false;
	if (!WriteRegister(ADXL362_INTMAP1_REG, 0) || !WriteRegister(ADXL362_INTMAP2_REG, 0))
		return false;
	if (!WriteRegister(ADXL362_FIFO_CONTROL_REG, 0))
		return false;
	vIntMap1 = vIntMap2 = vFifoCtl = 0;
	vFifoWatermark = 0;
	vFifoOverruns = 0;
	vbFifoEnabled = false;

	DeviceID(ADXL362_DEVID);
	AccelSensor::Scale(Cfg.Scale <= 2 ? 2 : Cfg.Scale <= 4 ? 4 : 8);
	Mode(Cfg.OpMode, freq);
	vbInitialized = true;
	return true;
}

bool AccelAdxl362::Enable()
{
	if (!vbInitialized)
		return false;

	return WriteRegister(ADXL362_POWER_CTL_REG, ADXL362_POWER_CTL_MEASURE_START);
}

void AccelAdxl362::Disable()
{
	if (vbInitialized)
		WriteRegister(ADXL362_POWER_CTL_REG, ADXL362_POWER_CTL_MEASURE_STANDBY);
}

void AccelAdxl362::Reset()
{
	if (WriteRegister(ADXL362_SOFT_RESET_REG, ADXL362_SOFT_RESET))
		vbInitialized = false;
}

bool AccelAdxl362::StartSampling()
{
	return Enable();
}

uint16_t AccelAdxl362::Scale(uint16_t Value)
{
	uint8_t regval;
	uint16_t scale;

	if (!vbInitialized || !ReadRegisters(ADXL362_FILTER_CTL_REG, &regval, 1))
		return AccelSensor::Scale();

	regval &= ~ADXL362_FILTER_CTL_RANGE_MASK;
	if (Value <= 2)
	{
		scale = 2;
		regval |= ADXL362_FILTER_CTL_RANGE_2G;
	}
	else if (Value <= 4)
	{
		scale = 4;
		regval |= ADXL362_FILTER_CTL_RANGE_4G;
	}
	else
	{
		scale = 8;
		regval |= ADXL362_FILTER_CTL_RANGE_8G;
	}
	if (!WriteRegister(ADXL362_FILTER_CTL_REG, regval))
		return AccelSensor::Scale();

	return AccelSensor::Scale(scale);
}

bool AccelAdxl362::UpdateData()
{
	uint8_t status;
	uint8_t raw[8];
	AccelSensorRawData_t sample;

	if (!vbInitialized || !ReadRegisters(ADXL362_STATUS_REG, &status, 1))
		return false;
	if ((status & ADXL362_STATUS_DATA_READY) == 0)
		return false;
	if (!ReadRegisters(ADXL362_XDATA_L_REG, raw, sizeof(raw)))
		return false;

	sample = vData;
	sample.Timestamp = vpTimer ? vpTimer->uSecond() : vData.Timestamp + 1;
	sample.X = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
	sample.Y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
	sample.Z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
	sample.Temp = (int16_t)((uint16_t)raw[6] | ((uint16_t)raw[7] << 8));
	vData = sample;
	return true;
}

bool AccelAdxl362::WakeOnEvent(bool bEnable, int Threshold)
{
	uint8_t threshold[2];
	uint8_t ctl;

	if (!vbInitialized)
		return false;
	if (!bEnable)
	{
		if (!WriteRegister(ADXL362_ACT_INACT_CTL_REG, 0))
			return false;
		vbMotionEnabled = false;
		return ConfigureInterrupts(vIntMap1 & ~ADXL362_INTMAP1_ACT, vIntMap2);
	}
	if (Threshold < 0 || Threshold > 2047)
		return false;

	threshold[0] = (uint8_t)Threshold;
	threshold[1] = (uint8_t)((unsigned int)Threshold >> 8);
	if (!WriteRegister(ADXL362_THRESH_ACT_L_REG, threshold[0]) ||
		!WriteRegister(ADXL362_THRESH_ACT_H_REG, threshold[1] & ADXL362_THRESH_ACT_H_MASK))
		return false;
	if (!WriteRegister(ADXL362_TIME_ACT_REG, 1))
		return false;
	ctl = ADXL362_ACT_INACT_CTL_ACT_EN;
	if (!WriteRegister(ADXL362_ACT_INACT_CTL_REG, ctl))
		return false;
	if (!ConfigureInterrupts(vIntMap1 | ADXL362_INTMAP1_ACT, vIntMap2))
		return false;
	vbMotionEnabled = true;
	return true;
}

bool AccelAdxl362::Status(uint8_t &Flags)
{
	if (!vbInitialized)
		return false;

	return ReadRegisters(ADXL362_STATUS_REG, &Flags, 1);
}

bool AccelAdxl362::ConfigureFifo(uint16_t WatermarkSamples)
{
	uint8_t ctl;
	uint8_t samples;

	if (!vbInitialized || WatermarkSamples > 170)
		return false;
	if (WatermarkSamples == 0)
	{
			if (!WriteRegister(ADXL362_FIFO_CONTROL_REG, 0))
			return false;
		vFifoCtl = 0;
		vFifoWatermark = 0;
		vbFifoEnabled = false;
		return true;
	}

	// FIFO sample count is measured in individual XYZ words, not frames.
	// At most 512 entries are supported; use a full XYZ frame per read.
	// The 9th watermark bit resides in FIFO_CONTROL.AH.
	uint16_t entries = WatermarkSamples * 3;
	ctl = 2 | ((entries & 0x100) ? ADXL362_FIFO_CONTROL_AH : 0);
	samples = (uint8_t)entries;
	if (!WriteRegister(ADXL362_FIFO_CONTROL_REG, 0))
		return false;
	if (!WriteRegister(ADXL362_FIFO_SAMPLES_REG, samples))
		return false;
	if (!WriteRegister(ADXL362_FIFO_CONTROL_REG, ctl))
		return false;
	vFifoCtl = ctl;
	vFifoWatermark = entries;
	vbFifoEnabled = true;
	return true;
}

bool AccelAdxl362::ReadFifo(AccelSensorRawData_t &Data)
{
	uint8_t cmd;
	uint8_t n[2];
	uint8_t raw[6];
	uint16_t entries;
	AccelSensorRawData_t sample;

	if (!vbInitialized || !vbFifoEnabled || Interface() == NULL)
		return false;
	if (!ReadRegisters(ADXL362_FIFO_ENTRIES_L_REG, n, sizeof(n)))
		return false;
	entries = (uint16_t)n[0] | (((uint16_t)n[1] & 3) << 8);
	if (entries < 3)
		return false;

	cmd = ADXL362_CMD_READFIFO;
	if (Interface()->Read(DeviceAddress(), &cmd, 1, raw, sizeof(raw)) != sizeof(raw))
		return false;
	// FIFO words use bits 15:14 to encode the axis and bits 11:0 for data.
	if ((raw[1] & 0xC0) != 0 || (raw[3] & 0xC0) != 0x40 ||
		(raw[5] & 0xC0) != 0x80)
	{
		FlushFifo();
		return false;
	}

	sample = vData;
	sample.Timestamp = vpTimer ? vpTimer->uSecond() : vData.Timestamp + 1;
	sample.X = (int16_t)(((uint16_t)raw[0] | ((uint16_t)raw[1] << 8)) << 4) >> 4;
	sample.Y = (int16_t)(((uint16_t)raw[2] | ((uint16_t)raw[3] << 8)) << 4) >> 4;
	sample.Z = (int16_t)(((uint16_t)raw[4] | ((uint16_t)raw[5] << 8)) << 4) >> 4;
	vData = sample;
	Data = sample;
	return true;
}

bool AccelAdxl362::ConfigureInterrupts(uint8_t Int1Map, uint8_t Int2Map)
{
	if (!vbInitialized)
		return false;
	if (!WriteRegister(ADXL362_INTMAP1_REG, Int1Map))
		return false;
	if (!WriteRegister(ADXL362_INTMAP2_REG, Int2Map))
		return false;
	vIntMap1 = Int1Map;
	vIntMap2 = Int2Map;
	return true;
}

bool AccelAdxl362::FlushFifo()
{
	if (!vbInitialized)
		return false;
	if (!WriteRegister(ADXL362_FIFO_CONTROL_REG, 0))
		return false;
	if (vbFifoEnabled && !WriteRegister(ADXL362_FIFO_CONTROL_REG, vFifoCtl))
		return false;
	return true;
}

void AccelAdxl362::IntHandler()
{
	uint8_t status;

	// Reading STATUS clears activity and inactivity interrupt latches.
	if (!Status(status))
		return;
	if ((status & ADXL362_STATUS_FIFO_OVER_RUN) != 0 && vbFifoEnabled)
	{
		vFifoOverruns++;
		FlushFifo();
		return;
	}
	if ((status & ADXL362_STATUS_DATA_READY) != 0 && !vbFifoEnabled)
		UpdateData();
}

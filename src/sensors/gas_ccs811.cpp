/**-------------------------------------------------------------------------
@file	gas_ccs811.cpp

@brief	CCS811 gas sensor.

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

/** CCS811 gas sensor. SPDX-License-Identifier: MIT */
#include "sensors/gas_ccs811.h"
#include "idelay.h"
#include <math.h>

static uint32_t Ccs811Frequency(uint8_t Mode)
{
	static const uint32_t rates[] = { 0, 1000, 100, 17, 4000 };

	return Mode <= 4 ? rates[Mode] : 0;
}

static uint8_t Ccs811ModeForRate(uint32_t Freq)
{
	static const uint8_t modes[] = { 3, 2, 1, 4 };
	uint8_t best = 3;
	uint64_t error = UINT64_MAX;
	uint64_t delta;
	uint32_t rate;
	int i;

	for (i = 0; i < 4; ++i)
	{
		rate = Ccs811Frequency(modes[i]);
		delta = Freq > rate ? (uint64_t)Freq - rate : (uint64_t)rate - Freq;
		if (delta < error)
		{
			error = delta;
			best = modes[i];
		}
	}
	return best;
}

GasCcs811::GasCcs811() : vWake(nullptr), vpWakeContext(nullptr), vDrive(0),
	vLastDrive(0), vResumeDrive(1), vData{}
{
	vEvtHandler = nullptr;
	vIntId = 0;
	vByteOrder = DEV_BYTEORDER_BIG;
	vType = (SENSOR_TYPE)0;
	vState = SENSOR_STATE_SLEEP;
	vOpMode = SENSOR_OPMODE_CONTINUOUS;
	vSampFreq = 0;
	vSampPeriod = 0;
	vbSampling = false;
	vSampleCnt = 0;
	vSampleTime = 0;
	vDropCnt = 0;
	vFilterrFreq = 0;
	vTimerTrigId = -1;
	vRange = 1023;
	DataReadyClear();
	vbFifoEn = false;
	memset(&vGasData, 0, sizeof(vGasData));
	vGasData.MeasIdx = -1;
	vGasData.AirQualIdx = NAN;
}

void GasCcs811::WakeControl(Ccs811WakeFn_t Callback, void *pContext)
{
	vWake = Callback;
	vpWakeContext = pContext;
}

bool GasCcs811::Transfer(uint8_t Reg, uint8_t *pData, int Len, bool Write)
{
	bool ok;
	bool released = true;

	if (vpIntrf == nullptr)
		return false;
	if (vWake != nullptr && !vWake(true, vpWakeContext))
	{
		vWake(false, vpWakeContext);
		usDelay(20);
		return false;
	}
	usDelay(50);
	if (Len == 0)
		ok = vpIntrf->Tx(vDevAddr, &Reg, 1) == 1;
	else if (Write)
		ok = Device::Write(&Reg, 1, pData, Len) == Len;
	else
		ok = Device::Read(&Reg, 1, pData, Len) == Len;
	if (vWake != nullptr)
		released = vWake(false, vpWakeContext);
	usDelay(20);
	return ok && released;
}

bool GasCcs811::ReadReg(uint8_t Reg, uint8_t *pData, int Len)
{
	return Transfer(Reg, pData, Len, false);
}

bool GasCcs811::WriteReg(uint8_t Reg, const uint8_t *pData, int Len)
{
	return Transfer(Reg, const_cast<uint8_t *>(pData), Len, true);
}

bool GasCcs811::AppStart()
{
	uint8_t status;

	if (!ReadReg(0, &status, 1) || (status & 0x11) != 0x10)
		return false;
	if ((status & 0x80) == 0)
	{
		if (!Transfer(0xF4, nullptr, 0, true))
			return false;
		msDelay(1);
	}
	return ReadReg(0, &status, 1) && (status & 0x91) == 0x90;
}

bool GasCcs811::Init(const GasSensorCfg_t &Cfg, DeviceIntrf * const pIntrf,
	Timer * const pTimer)
{
	uint8_t id;
	uint8_t version;
	uint8_t mode;

	Valid(false);
	if (pIntrf == nullptr || pIntrf->Type() != DEVINTRF_TYPE_I2C ||
		(Cfg.DevAddr != 0x5A && Cfg.DevAddr != 0x5B) ||
		Cfg.OpMode != SENSOR_OPMODE_CONTINUOUS || Cfg.NbHeatPoint != 0)
		return false;
	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vpTimer = pTimer;
	msDelay(2);
	if (!ReadReg(0x20, &id, 1) || id != 0x81 ||
		!ReadReg(0x21, &version, 1) || (version >> 4) != 1 || !AppStart() ||
		!ReadReg(1, &mode, 1) || ((mode >> 4) & 7) > 4)
		return false;
	DeviceID(id);
	vDrive = (mode >> 4) & 7;
	// Retain prior sampling history when reattaching to this device in idle.
	if (vDrive != 0)
		vLastDrive = vDrive;
	Valid(true);
	if (!Mode(Cfg.OpMode, Cfg.Freq))
	{
		Valid(false);
		return false;
	}
	memset(&vData, 0, sizeof(vData));
	memset(&vGasData, 0, sizeof(vGasData));
	vGasData.MeasIdx = -1;
	vGasData.AirQualIdx = NAN;
	DataReadyClear();
	return true;
}

bool GasCcs811::SetDriveMode(uint8_t Mode, uint32_t IdleMs)
{
	uint8_t control;

	if (!Valid() || Mode > 4)
		return false;
	if (Mode != 0 && vLastDrive != 0 &&
		Ccs811Frequency(Mode) < Ccs811Frequency(vLastDrive) &&
		(vDrive != 0 || IdleMs < 600000))
		return false;
	if (!ReadReg(1, &control, 1))
		return false;
	control = (control & 0x0C) | (Mode << 4);
	if (!WriteReg(1, &control, 1))
	{
		Valid(false); // Ambiguous transfer: software cannot assume the old mode.
		return false;
	}
	vDrive = Mode;
	if (Mode != 0)
	{
		vLastDrive = Mode;
		vResumeDrive = Mode;
	}
	vSampFreq = Ccs811Frequency(Mode);
	vSampPeriod = Mode == 3 ? 60000000000ULL :
		(vSampFreq == 0 ? 0 : 1000000000000ULL / vSampFreq);
	vbSampling = Mode != 0;
	vState = Mode == 0 ? SENSOR_STATE_IDLE : SENSOR_STATE_SAMPLING;
	return true;
}

bool GasCcs811::Mode(SENSOR_OPMODE Mode, uint32_t Freq)
{
	uint8_t drive;

	if (Mode != SENSOR_OPMODE_CONTINUOUS)
		return false;
	drive = Ccs811ModeForRate(Freq);
	return SetDriveMode(drive);
}

uint32_t GasCcs811::SamplingFrequency(uint32_t Freq)
{
	Mode(SENSOR_OPMODE_CONTINUOUS, Freq);
	return vSampFreq;
}

bool GasCcs811::Enable()
{
	return SetDriveMode(vResumeDrive);
}

void GasCcs811::Disable()
{
	if (Valid() && !SetDriveMode(0))
		Valid(false);
}

bool GasCcs811::SoftReset()
{
	static const uint8_t key[] = { 0x11, 0xE5, 0x72, 0x8A };
	bool ok;

	ok = WriteReg(0xFF, key, sizeof(key));
	Valid(false);
	vDrive = 0;
	vbSampling = false;
	vState = SENSOR_STATE_SLEEP;
	vSampFreq = 0;
	vSampPeriod = 0;
	DataReadyClear();
	msDelay(2);
	return ok;
}

void GasCcs811::Reset()
{
	SoftReset();
}

bool GasCcs811::Read(Ccs811Data_t &Data)
{
	uint8_t status;
	uint8_t raw[8];
	uint16_t adc;
	Ccs811Data_t sample = {};

	if (!Valid() || vDrive == 0 || !ReadReg(0, &status, 1) ||
		(status & 0x89) != 0x88)
		return false;
	if (vDrive == 4)
	{
		// Only RAW_DATA is refreshed in mode 4. Read ALG_RESULT to acknowledge.
		if (!ReadReg(3, raw + 6, 2) || !ReadReg(2, raw, 2))
			return false;
	}
	else if (!ReadReg(2, raw, sizeof(raw)) || (raw[4] & 1) != 0)
		return false;
	sample.Timestamp = vpTimer != nullptr ? vpTimer->uSecond() : 0;
	sample.AlgorithmValid = vDrive != 4;
	if (sample.AlgorithmValid)
	{
		sample.Eco2 = ((uint16_t)raw[0] << 8) | raw[1];
		sample.Etvoc = ((uint16_t)raw[2] << 8) | raw[3];
	}
	adc = ((uint16_t)raw[6] << 8) | raw[7];
	sample.RawAdc = adc & 0x3FF;
	sample.Current = adc >> 10;
	vData = sample;
	Data = sample;
	vGasData.Timestamp = sample.Timestamp;
	vGasData.MeasIdx = sample.Current == 0 ? -1 : 0;
	vGasData.GasRes[0] = sample.Current == 0 ? 0 :
		(uint32_t)(((uint64_t)sample.RawAdc * 1650000) / (1023U * sample.Current));
	vGasData.AirQualIdx = NAN;
	++vSampleCnt;
	DataReadyClear();
	return true;
}

bool GasCcs811::Read(GasSensorData_t &Data)
{
	Ccs811Data_t sample;
	bool fresh;

	fresh = Read(sample);
	Data = vGasData;
	return fresh;
}

bool GasCcs811::StartSampling()
{
	return Valid() && (vDrive != 0 || Enable());
}

bool GasCcs811::UpdateData()
{
	Ccs811Data_t sample;

	return Read(sample);
}

bool GasCcs811::SetHeatingProfile(int Count, const GasSensorHeater_t * const pProfile)
{
	(void)pProfile;
	return Count == 0; // Heater is controlled by the sensor firmware.
}

bool GasCcs811::SetEnvironment(int16_t TemperatureCentiC, uint16_t HumidityCentiPercent)
{
	uint16_t humidity;
	uint16_t temperature;
	uint8_t data[4];

	if (!Valid() || HumidityCentiPercent > 10000 || TemperatureCentiC > 8500 ||
		TemperatureCentiC < -4000)
		return false;
	humidity = ((uint32_t)HumidityCentiPercent * 512 + 50) / 100;
	temperature = TemperatureCentiC < -2500 ? 0 :
		((uint32_t)(TemperatureCentiC + 2500) * 512 + 50) / 100;
	data[0] = humidity >> 8;
	data[1] = humidity;
	data[2] = temperature >> 8;
	data[3] = temperature;
	return WriteReg(5, data, sizeof(data));
}

bool GasCcs811::ConfigureInterrupt(bool Enable, bool ThresholdOnly)
{
	uint8_t mode;

	if (!Valid() || (!Enable && ThresholdOnly) || !ReadReg(1, &mode, 1))
		return false;
	mode = (mode & 0x70) | (Enable ? 8 : 0) | (ThresholdOnly ? 4 : 0);
	return WriteReg(1, &mode, 1);
}

bool GasCcs811::SetThresholds(uint16_t LowPpm, uint16_t HighPpm)
{
	uint8_t data[4] = { (uint8_t)(LowPpm >> 8), (uint8_t)LowPpm,
		(uint8_t)(HighPpm >> 8), (uint8_t)HighPpm };

	return Valid() && LowPpm >= 400 && HighPpm <= 29206 && LowPpm < HighPpm &&
		WriteReg(0x10, data, sizeof(data));
}

bool GasCcs811::ReadBaseline(uint16_t &Baseline)
{
	uint8_t data[2];

	if (!Valid() || !ReadReg(0x11, data, 2))
		return false;
	Baseline = ((uint16_t)data[0] << 8) | data[1];
	return true;
}

bool GasCcs811::WriteBaseline(uint16_t Baseline)
{
	uint8_t data[2] = { (uint8_t)(Baseline >> 8), (uint8_t)Baseline };

	return Valid() && WriteReg(0x11, data, 2);
}

bool GasCcs811::ReadError(uint8_t &Error)
{
	uint8_t error;

	if (!ReadReg(0xE0, &error, 1))
		return false;
	Error = error;
	return true;
}

bool GasCcs811::FirmwareVersion(uint16_t &Boot, uint16_t &Application)
{
	uint8_t boot[2];
	uint8_t app[2];

	if (!Valid() || !ReadReg(0x23, boot, 2) || !ReadReg(0x24, app, 2))
		return false;
	Boot = ((uint16_t)boot[0] << 8) | boot[1];
	Application = ((uint16_t)app[0] << 8) | app[1];
	return true;
}

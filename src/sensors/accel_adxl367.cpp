/**-------------------------------------------------------------------------
@file accel_adxl367.cpp
@brief ADXL367 device driver
@license MIT License
Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#include "sensors/accel_adxl367.h"

#define REG_ID               0x00
#define REG_STATUS           0x0B
#define REG_FIFO_ENTRIES     0x0C
#define REG_XDATA_H          0x0E
#define REG_TEMP_H           0x14
#define REG_ADC_H            0x16
#define REG_FIFO_I2C         0x18
#define REG_RESET            0x1F
#define REG_ACT_H            0x20
#define REG_INACT_H          0x23
#define REG_ACT_INACT_CTL    0x27
#define REG_FIFO_CTL         0x28
#define REG_FIFO_SAMPLES     0x29
#define REG_INT1            0x2A
#define REG_INT2            0x2B
#define REG_FILTER          0x2C
#define REG_POWER           0x2D
#define REG_TAP_THRESH      0x2F
#define REG_INT1_UPPER      0x3A
#define REG_INT2_UPPER      0x3B
#define REG_ADC_CTL         0x3C
#define REG_TEMP_CTL        0x3D
#define REG_PEDOMETER_CTL   0x49
#define REG_STEP_H          0x47

AccelAdxl367::AccelAdxl367() :
	vbInitialized(false), vbFifoEnabled(false), vFilter(0), vPower(0),
	vFifoCtl(0), vFifoReadMode(0xC0), vFifoSamples(0), vInt1Map(0),
	vInt2Map(0), vFifoOverruns(0) {}

int16_t AccelAdxl367::Sign14(uint16_t Value)
{
	Value &= 0x3FFF;
	return (int16_t)((Value ^ 0x2000) - 0x2000);
}

bool AccelAdxl367::ReadReg(uint8_t Reg, uint8_t *pData, int Len)
{
	uint8_t cmd[2];
	DeviceIntrf *pIntrf;

	pIntrf = Interface();
	if (pIntrf == NULL || pData == NULL || Len <= 0)
		return false;
	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		cmd[0] = 0x0B;
		cmd[1] = Reg;
		return pIntrf->Read(DeviceAddress(), cmd, 2, pData, Len) == Len;
	}
	return pIntrf->Read(DeviceAddress(), &Reg, 1, pData, Len) == Len;
}

bool AccelAdxl367::WriteReg(uint8_t Reg, uint8_t Value)
{
	uint8_t cmd[3];
	DeviceIntrf *pIntrf;

	pIntrf = Interface();
	if (pIntrf == NULL)
		return false;
	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		cmd[0] = 0x0A;
		cmd[1] = Reg;
		cmd[2] = Value;
		return pIntrf->Tx(DeviceAddress(), cmd, 3) == 3;
	}
	return pIntrf->Write(DeviceAddress(), &Reg, 1, &Value, 1) == 1;
}

bool AccelAdxl367::WriteField(uint8_t Reg, uint8_t Mask, uint8_t Value)
{
	uint8_t oldval;

	if (!ReadReg(Reg, &oldval, 1))
		return false;
	return WriteReg(Reg, (oldval & ~Mask) | (Value & Mask));
}

bool AccelAdxl367::Read14(uint8_t Reg, int16_t &Value)
{
	uint8_t raw[2];

	if (!ReadReg(Reg, raw, 2))
		return false;
	Value = Sign14(((uint16_t)raw[0] << 6) | (raw[1] >> 2));
	return true;
}

bool AccelAdxl367::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint8_t id[3];
	uint16_t scale;

	vbInitialized = false;
	if (pIntrf == NULL)
		return false;
	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vpTimer = pTimer;
	if (!ReadReg(REG_ID, id, 3))
		return false;
	if (id[0] != 0xAD || id[1] != 0x1D || id[2] != ADXL367_PART_ID)
		return false;
	if (!WriteReg(REG_POWER, 0) || !WriteReg(REG_FIFO_CTL, 0))
		return false;
	if (!WriteReg(REG_INT1, 0) || !WriteReg(REG_INT2, 0) ||
		!WriteReg(REG_INT1_UPPER, 0) || !WriteReg(REG_INT2_UPPER, 0))
		return false;
	if (!WriteReg(REG_ACT_INACT_CTL, 0))
		return false;
	vFilter = 0x20;
	vPower = 0;
	vInt1Map = vInt2Map = 0;
	vFifoSamples = 0;
	vbFifoEnabled = false;
	vFifoOverruns = 0;
	vbInitialized = true;
	if (!SetSamplingFrequency(Cfg.Freq))
		goto fail;
	scale = Cfg.Scale <= 2 ? 2 : Cfg.Scale <= 4 ? 4 : 8;
	if (Scale(scale) != scale)
		goto fail;
	Mode(Cfg.OpMode, SamplingFrequency());
	DeviceID(ADXL367_PART_ID);
	return true;
fail:
	vbInitialized = false;
	return false;
}

bool AccelAdxl367::SetSamplingFrequency(uint32_t FreqMilliHz)
{
	uint8_t odr;
	uint32_t actual;

	if (!vbInitialized)
		return false;
	if (FreqMilliHz <= 12500)
	{
		odr = 0;
		actual = 12500;
	}
	else if (FreqMilliHz <= 25000)
	{
		odr = 1;
		actual = 25000;
	}
	else if (FreqMilliHz <= 50000)
	{
		odr = 2;
		actual = 50000;
	}
	else if (FreqMilliHz <= 100000)
	{
		odr = 3;
		actual = 100000;
	}
	else if (FreqMilliHz <= 200000)
	{
		odr = 4;
		actual = 200000;
	}
	else
	{
		odr = 5;
		actual = 400000;
	}
	if (!WriteReg(REG_FILTER, (vFilter & ~7) | odr))
		return false;
	vFilter = (vFilter & ~7) | odr;
	Sensor::SamplingFrequency(actual);
	return true;
}

uint16_t AccelAdxl367::Scale(uint16_t Value)
{
	uint16_t scale;
	uint8_t filter;

	if (!vbInitialized)
		return AccelSensor::Scale();
	scale = Value <= 2 ? 2 : Value <= 4 ? 4 : 8;
	filter = (vFilter & ~0xC0) | (scale == 2 ? 0 : scale == 4 ? 0x40 : 0x80);
	if (!WriteReg(REG_FILTER, filter))
		return AccelSensor::Scale();
	vFilter = filter;
	return AccelSensor::Scale(scale);
}

bool AccelAdxl367::Enable()
{
	if (!vbInitialized || !WriteReg(REG_POWER, (vPower & ~3) | 2))
		return false;
	vPower = (vPower & ~3) | 2;
	return true;
}

void AccelAdxl367::Disable()
{
	if (vbInitialized && WriteReg(REG_POWER, vPower & ~3))
		vPower &= ~3;
}

void AccelAdxl367::Reset()
{
	if (WriteReg(REG_RESET, 0x52))
		vbInitialized = false;
}

bool AccelAdxl367::StartSampling()
{
	return Enable();
}

bool AccelAdxl367::UpdateData()
{
	uint8_t status;
	uint8_t raw[6];
	AccelSensorRawData_t sample;

	if (!vbInitialized || !ReadReg(REG_STATUS, &status, 1) || !(status & 1))
		return false;
	if (!ReadReg(REG_XDATA_H, raw, 6))
		return false;
	sample = vData;
	sample.X = Sign14(((uint16_t)raw[0] << 6) | (raw[1] >> 2));
	sample.Y = Sign14(((uint16_t)raw[2] << 6) | (raw[3] >> 2));
	sample.Z = Sign14(((uint16_t)raw[4] << 6) | (raw[5] >> 2));
	sample.Timestamp = vpTimer ? vpTimer->uSecond() : vData.Timestamp + 1;
	vData = sample;
	return true;
}

bool AccelAdxl367::Status(uint16_t &Events)
{
	uint8_t status[2];

	if (!vbInitialized || !ReadReg(REG_STATUS, status, 1) ||
		!ReadReg(0x45, status + 1, 1))
		return false;
	Events = (uint16_t)status[0] | ((uint16_t)status[1] << 8);
	return true;
}

bool AccelAdxl367::ConfigureInterrupts(uint16_t Int1Map, uint16_t Int2Map)
{
	if (!vbInitialized)
		return false;
	if (!WriteReg(REG_INT1, (uint8_t)Int1Map) ||
		!WriteReg(REG_INT1_UPPER, (uint8_t)(Int1Map >> 8) & 0xDF) ||
		!WriteReg(REG_INT2, (uint8_t)Int2Map) ||
		!WriteReg(REG_INT2_UPPER, (uint8_t)(Int2Map >> 8) & 0xDF))
		return false;
	vInt1Map = Int1Map;
	vInt2Map = Int2Map;
	return true;
}

bool AccelAdxl367::ConfigureActivity(uint16_t Threshold, uint8_t Time, bool Referenced)
{
	uint8_t ctl;

	if (!vbInitialized || Threshold > 0x1FFF)
		return false;
	if (!ReadReg(REG_ACT_INACT_CTL, &ctl, 1))
		return false;
	if (!WriteReg(REG_ACT_H, (uint8_t)(Threshold >> 6) & 0x7F) ||
		!WriteReg(REG_ACT_H + 1, (uint8_t)(Threshold << 2)) ||
		!WriteReg(REG_ACT_H + 2, Time))
		return false;
	return WriteReg(REG_ACT_INACT_CTL, (ctl & ~3) | (Referenced ? 3 : 1));
}

bool AccelAdxl367::ConfigureInactivity(uint16_t Threshold, uint16_t Time, bool Referenced)
{
	uint8_t ctl;

	if (!vbInitialized || Threshold > 0x3FFF)
		return false;
	if (!ReadReg(REG_ACT_INACT_CTL, &ctl, 1))
		return false;
	if (!WriteReg(REG_INACT_H, (uint8_t)(Threshold >> 6) & 0x7F) ||
		!WriteReg(REG_INACT_H + 1, (uint8_t)(Threshold << 2)) ||
		!WriteReg(REG_INACT_H + 2, (uint8_t)Time) ||
		!WriteReg(REG_INACT_H + 3, (uint8_t)(Time >> 8)))
		return false;
	return WriteReg(REG_ACT_INACT_CTL, (ctl & ~0x0C) | (Referenced ? 0x0C : 0x04));
}

bool AccelAdxl367::WakeOnEvent(bool bEnable, int Threshold)
{
	if (!vbInitialized)
		return false;
	if (!bEnable)
	{
		if (!WriteField(REG_ACT_INACT_CTL, 3, 0))
			return false;
		return ConfigureInterrupts(vInt1Map & ~ADXL367_EVT_ACTIVITY, vInt2Map);
	}
	if (Threshold < 0 || Threshold > 0x3FFF)
		return false;
	return ConfigureActivity((uint16_t)Threshold, 1, false) &&
		ConfigureInterrupts(vInt1Map | ADXL367_EVT_ACTIVITY, vInt2Map);
}

bool AccelAdxl367::ConfigureTap(uint8_t Threshold, uint8_t Duration,
	uint8_t Latency, uint8_t Window, bool DoubleTap)
{
	uint16_t bits;

	if (!vbInitialized)
		return false;
	if (!WriteReg(REG_TAP_THRESH, Threshold) ||
		!WriteReg(REG_TAP_THRESH + 1, Duration) ||
		!WriteReg(REG_TAP_THRESH + 2, Latency) ||
		!WriteReg(REG_TAP_THRESH + 3, Window))
		return false;
	bits = DoubleTap ? 0x0300 : 0x0100;
	return ConfigureInterrupts(vInt1Map | bits, vInt2Map);
}

bool AccelAdxl367::EnableTemperature(bool Enable)
{
	if (!vbInitialized)
		return false;
	if (Enable && !WriteField(REG_ADC_CTL, 1, 0))
		return false;
	return WriteField(REG_TEMP_CTL, 1, Enable ? 1 : 0);
}

bool AccelAdxl367::EnableAdc(bool Enable)
{
	if (!vbInitialized)
		return false;
	if (Enable && !WriteField(REG_TEMP_CTL, 1, 0))
		return false;
	return WriteField(REG_ADC_CTL, 1, Enable ? 1 : 0);
}

bool AccelAdxl367::ReadTemperature(int16_t &Raw, int32_t &CentiCelsius)
{
	int16_t temp;

	if (!vbInitialized || !Read14(REG_TEMP_H, temp))
		return false;
	Raw = temp;
	CentiCelsius = 2500 + ((int32_t)temp - 165) * 100 / 54;
	return true;
}

bool AccelAdxl367::ReadAdc(int16_t &Raw)
{
	return vbInitialized && Read14(REG_ADC_H, Raw);
}

bool AccelAdxl367::EnablePedometer(bool Enable)
{
	return vbInitialized && WriteField(REG_PEDOMETER_CTL, 1, Enable ? 1 : 0);
}

bool AccelAdxl367::ReadSteps(uint16_t &Steps)
{
	uint8_t raw[2];

	if (!vbInitialized || !ReadReg(REG_STEP_H, raw, 2))
		return false;
	Steps = ((uint16_t)raw[0] << 8) | raw[1];
	return true;
}

bool AccelAdxl367::ResetSteps()
{
	if (!vbInitialized || !WriteField(REG_PEDOMETER_CTL, 4, 4))
		return false;
	return WriteField(REG_PEDOMETER_CTL, 4, 0);
}

bool AccelAdxl367::SetLowPower(bool AutoSleep, bool Wakeup, uint8_t NoiseMode)
{
	uint8_t value;

	if (!vbInitialized || NoiseMode > 2)
		return false;
	value = (vPower & ~0x3C) | (NoiseMode << 4) |
		(AutoSleep ? 4 : 0) | (Wakeup ? 8 : 0);
	if (!WriteReg(REG_POWER, value))
		return false;
	vPower = value;
	return true;
}

bool AccelAdxl367::ConfigureFifo(ADXL367_FIFO_MODE Mode,
	ADXL367_FIFO_FORMAT Format, uint16_t SampleSets)
{
	uint8_t ctl;
	uint8_t dataCtl;

	if (!vbInitialized || Mode > ADXL367_FIFO_TRIGGERED ||
		!(Format == ADXL367_FIFO_XYZ || Format == ADXL367_FIFO_XYZ_TEMP ||
		Format == ADXL367_FIFO_XYZ_ADC) || SampleSets > 511)
		return false;
	if (Mode != ADXL367_FIFO_DISABLED && (SampleSets == 0 || SampleSets > (Format == ADXL367_FIFO_XYZ ? 170 : 128)))
		return false;
	ctl = (uint8_t)(((uint8_t)Format << 3) | (uint8_t)Mode |
		((SampleSets >> 8) & 1 ? 4 : 0));
	if (!WriteReg(REG_FIFO_CTL, 0))
		return false;
	// FIFO format 11 = 14 bits with channel IDs.
	if (!ReadReg(REG_ADC_CTL, &dataCtl, 1))
		return false;
	if (!WriteReg(REG_ADC_CTL, (dataCtl & 0x3F) | 0xC0) ||
		!WriteReg(REG_FIFO_SAMPLES, (uint8_t)SampleSets) ||
		!WriteReg(REG_FIFO_CTL, ctl))
		return false;
	vFifoCtl = ctl;
	vFifoSamples = SampleSets;
	vbFifoEnabled = Mode != ADXL367_FIFO_DISABLED;
	return true;
}

bool AccelAdxl367::ResetFifo()
{
	if (!WriteReg(REG_FIFO_CTL, 0))
		return false;
	return WriteReg(REG_FIFO_CTL, vFifoCtl);
}

bool AccelAdxl367::FlushFifo()
{
	return vbInitialized && ResetFifo();
}

bool AccelAdxl367::ReadFifo(AccelSensorRawData_t &Data)
{
	int16_t aux;

	return ReadFifo(Data, aux);
}

bool AccelAdxl367::ReadFifo(AccelSensorRawData_t &Data, int16_t &Auxiliary)
{
	uint8_t entries[2];
	uint8_t raw[8];
	int bytes;
	uint16_t count;
	uint16_t word;
	uint8_t cmd;
	int16_t axes[3];
	int i;
	AccelSensorRawData_t sample;
	DeviceIntrf *pIntrf;

	if (!vbInitialized || !vbFifoEnabled || Interface() == NULL)
		return false;
	if (!ReadReg(REG_FIFO_ENTRIES, entries, 2))
		return false;
	count = (uint16_t)entries[0] | (((uint16_t)entries[1] & 3) << 8);
	bytes = ((vFifoCtl & 0x78) == 0) ? 6 : 8;
	if (count < (uint16_t)(bytes / 2))
		return false;
	pIntrf = Interface();
	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		cmd = 0x0D;
		if (pIntrf->Read(DeviceAddress(), &cmd, 1, raw, bytes) != bytes)
			return false;
	}
	else if (!ReadReg(REG_FIFO_I2C, raw, bytes))
		return false;
	for (i = 0; i < 3; i++)
	{
		word = ((uint16_t)raw[2 * i] << 8) | raw[2 * i + 1];
		if ((word >> 14) != (uint16_t)i)
		{
			ResetFifo();
			return false;
		}
		axes[i] = Sign14(word);
	}
	if (bytes == 8)
	{
		word = ((uint16_t)raw[6] << 8) | raw[7];
		if ((word >> 14) != 3)
		{
			ResetFifo();
			return false;
		}
		Auxiliary = Sign14(word);
	}
	else
	{
		Auxiliary = 0;
	}
	sample = vData;
	sample.X = axes[0];
	sample.Y = axes[1];
	sample.Z = axes[2];
	sample.Timestamp = vpTimer ? vpTimer->uSecond() : vData.Timestamp + 1;
	vData = sample;
	Data = sample;
	return true;
}

void AccelAdxl367::IntHandler()
{
	uint16_t flags;

	if (!Status(flags))
		return;
	if (vbFifoEnabled && (flags & ADXL367_EVT_FIFO_OVERRUN))
	{
		vFifoOverruns++;
		ResetFifo();
		return;
	}
	if (!vbFifoEnabled && (flags & ADXL367_EVT_DATA_READY))
		UpdateData();
}

bool AccelAdxl367::ReadInterruptEvents(uint16_t &Events)
{
	return Status(Events);
}

bool AccelAdxl367::SelfTest(void (*DelayMs)(uint32_t))
{
	uint8_t previous;
	uint32_t delay;
	int16_t before;
	int16_t after;
	int32_t difference;
	uint16_t scale;
	bool ok;

	if (!vbInitialized || DelayMs == NULL)
		return false;
	previous = vPower;
	scale = AccelSensor::Scale();
	delay = SamplingFrequency() ? 4000000 / SamplingFrequency() : 320;
	if (delay < 10)
		delay = 10;
	ok = false;
	if (!Enable() || !WriteReg(0x2E, 1))
		goto cleanup;
	DelayMs(delay);
	if (!Read14(REG_XDATA_H, before))
		goto cleanup;
	if (!WriteReg(0x2E, 3))
		goto cleanup;
	DelayMs(delay);
	if (!Read14(REG_XDATA_H, after))
		goto cleanup;
	difference = (int32_t)after - before;
	ok = difference >= (360 * (scale / 2)) && difference <= (1080 * (scale / 2));
cleanup:
	WriteReg(0x2E, 0);
	WriteReg(REG_POWER, previous);
	vPower = previous;
	return ok;
}

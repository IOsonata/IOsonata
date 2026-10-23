/**-------------------------------------------------------------------------
@file accel_adxl372.cpp
@brief ADXL372 high-g accelerometer, SPI/I2C
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "sensors/accel_adxl372.h"

#define REG_ID                 0x00
#define REG_STATUS            0x04
#define REG_FIFO_ENTRIES      0x06
#define REG_XDATA             0x08
#define REG_PEAK              0x15
#define REG_THRESH_ACT        0x23
#define REG_THRESH_INACT      0x2A
#define REG_THRESH_ACT2       0x32
#define REG_TIME_ACT          0x29
#define REG_TIME_INACT        0x30
#define REG_FIFO_SAMPLES      0x39
#define REG_FIFO_CTL          0x3A
#define REG_INT1              0x3B
#define REG_INT2              0x3C
#define REG_TIMING            0x3D
#define REG_MEASURE           0x3E
#define REG_POWER             0x3F
#define REG_RESET             0x41
#define REG_FIFO_DATA         0x42

AccelAdxl372::AccelAdxl372() :
	vbInitialized(false), vPower(0), vMeasure(0), vInt1(0), vInt2(0),
	vFifoControl(0), vFifoEntries(0), vFifoOverruns(0) {
	vLastInterrupt.Status1 = 0;
	vLastInterrupt.Status2 = 0;
	vLastInterrupt.FifoEntries = 0;
}

bool AccelAdxl372::ReadReg(uint8_t Reg, uint8_t *pData, int Len)
{
	DeviceIntrf *pIntrf;
	uint8_t cmd;

	pIntrf = Interface();
	if (pIntrf == NULL || pData == NULL || Len < 1)
		return false;
	cmd = pIntrf->Type() == DEVINTRF_TYPE_SPI ? (uint8_t)((Reg << 1) | 1) : Reg;
	return pIntrf->Read(DeviceAddress(), &cmd, 1, pData, Len) == Len;
}

bool AccelAdxl372::WriteReg(uint8_t Reg, uint8_t Value)
{
	DeviceIntrf *pIntrf;
	uint8_t cmd[2];

	pIntrf = Interface();
	if (pIntrf == NULL)
		return false;
	cmd[0] = pIntrf->Type() == DEVINTRF_TYPE_SPI ? (uint8_t)(Reg << 1) : Reg;
	cmd[1] = Value;
	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
		return pIntrf->Tx(DeviceAddress(), cmd, 2) == 2;
	return pIntrf->Write(DeviceAddress(), cmd, 1, cmd + 1, 1) == 1;
}

bool AccelAdxl372::UpdateField(uint8_t Reg, uint8_t Mask, uint8_t Value)
{
	uint8_t data;

	if (!ReadReg(Reg, &data, 1))
		return false;
	return WriteReg(Reg, (data & ~Mask) | (Value & Mask));
}

int16_t AccelAdxl372::Decode12(uint8_t High, uint8_t Low)
{
	uint16_t raw;

	raw = ((uint16_t)High << 4) | (Low >> 4);
	return (int16_t)((raw ^ 0x800) - 0x800);
}

bool AccelAdxl372::DecodeXYZ(uint8_t Reg, AccelSensorRawData_t &Data)
{
	uint8_t raw[6];
	AccelSensorRawData_t sample;

	if (!ReadReg(Reg, raw, 6))
		return false;
	sample = vData;
	sample.X = Decode12(raw[0], raw[1]);
	sample.Y = Decode12(raw[2], raw[3]);
	sample.Z = Decode12(raw[4], raw[5]);
	sample.Timestamp = vpTimer ? vpTimer->uSecond() : vData.Timestamp + 1;
	Data = sample;
	return true;
}

bool AccelAdxl372::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint8_t id[3];

	vbInitialized = false;
	if (pIntrf == NULL)
		return false;
	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vpTimer = pTimer;
	if (!ReadReg(REG_ID, id, 3))
		return false;
	if (id[0] != 0xAD || id[1] != 0x1D || id[2] != ADXL372_PART_ID)
		return false;
	if (!WriteReg(REG_POWER, 0))
		return false;
	if (!WriteReg(REG_INT1, 0) || !WriteReg(REG_INT2, 0) ||
		!WriteReg(REG_FIFO_CTL, 0) || !WriteReg(REG_MEASURE, 0))
		return false;
	vPower = vMeasure = vInt1 = vInt2 = vFifoControl = 0;
	vFifoEntries = 0;
	vFifoOverruns = 0;
	vLastInterrupt.Status1 = 0;
	vLastInterrupt.Status2 = 0;
	vLastInterrupt.FifoEntries = 0;
	vbInitialized = true;
	if (!SetOutputRate(Cfg.Freq))
		goto fail;
	if (!SetBandwidth(2))
		goto fail;
	AccelSensor::Scale(200);
	if (!Mode(Cfg.OpMode, SamplingFrequency()))
		goto fail;
	DeviceID(ADXL372_PART_ID);
	return true;
fail:
	vbInitialized = false;
	return false;
}

uint16_t AccelAdxl372::Scale(uint16_t Value)
{
	(void)Value;
	return AccelSensor::Scale(200);
}

bool AccelAdxl372::SetOutputRate(uint32_t MilliHz)
{
	uint8_t odr;
	uint32_t actual;

	if (!vbInitialized)
		return false;
	if (MilliHz <= 400000) { odr = 0; actual = 400000; }
	else if (MilliHz <= 800000) { odr = 1; actual = 800000; }
	else if (MilliHz <= 1600000) { odr = 2; actual = 1600000; }
	else if (MilliHz <= 3200000) { odr = 3; actual = 3200000; }
	else { odr = 4; actual = 6400000; }
	if (!UpdateField(REG_TIMING, 0xE0, odr << 5))
		return false;
	Sensor::SamplingFrequency(actual);
	return true;
}

bool AccelAdxl372::SetBandwidth(uint8_t Value)
{
	if (!vbInitialized || Value > 4 || !UpdateField(REG_MEASURE, 7, Value))
		return false;
	vMeasure = (vMeasure & ~7) | Value;
	return true;
}

bool AccelAdxl372::SetWakeupRate(uint8_t Value)
{
	return vbInitialized && Value <= 7 && UpdateField(REG_TIMING, 0x1C, Value << 2);
}

bool AccelAdxl372::SetPowerMode(Adxl372PowerMode Mode)
{
	uint8_t value;

	if (!vbInitialized || Mode > ADXL372_MEASUREMENT)
		return false;
	value = (vPower & ~3) | (uint8_t)Mode;
	if (!WriteReg(REG_POWER, value))
		return false;
	vPower = value;
	return true;
}

bool AccelAdxl372::Enable() { return SetPowerMode(ADXL372_MEASUREMENT); }
void AccelAdxl372::Disable() { SetPowerMode(ADXL372_STANDBY); }
bool AccelAdxl372::StartSampling() { return Enable(); }

void AccelAdxl372::Reset()
{
	if (!vbInitialized)
		return;
	if (SetPowerMode(ADXL372_STANDBY) && WriteReg(REG_RESET, 0x52))
		vbInitialized = false;
}

bool AccelAdxl372::UpdateData()
{
	uint8_t status;
	AccelSensorRawData_t sample;

	if (!vbInitialized || !ReadReg(REG_STATUS, &status, 1) || !(status & 1))
		return false;
	if (!DecodeXYZ(REG_XDATA, sample))
		return false;
	vData = sample;
	return true;
}

bool AccelAdxl372::ReadPeak(AccelSensorRawData_t &Data)
{
	return vbInitialized && DecodeXYZ(REG_PEAK, Data);
}

bool AccelAdxl372::SetAutosleep(bool Enable, uint8_t LinkLoop)
{
	uint8_t value;

	if (!vbInitialized || LinkLoop > 2)
		return false;
	value = (vMeasure & ~0x70) | ((LinkLoop & 3) << 4) |
		(Enable ? 0x40 : 0);
	if (!WriteReg(REG_MEASURE, value))
		return false;
	vMeasure = value;
	return true;
}

bool AccelAdxl372::SetDetection(Adxl372Detection Type,
	uint16_t Threshold, bool Referenced, bool Enable)
{
	uint8_t reg;
	uint8_t low;
	int i;

	if (!vbInitialized || Type > ADXL372_DETECT_INACTIVITY || Threshold > 2047)
		return false;
	reg = Type == ADXL372_DETECT_ACTIVITY ? REG_THRESH_ACT :
		Type == ADXL372_DETECT_ACTIVITY2 ? REG_THRESH_ACT2 : REG_THRESH_INACT;
	low = (uint8_t)((Threshold << 5) | (Referenced ? 2 : 0) | (Enable ? 1 : 0));
	for (i = 0; i < 3; i++)
	{
		if (!WriteReg(reg + i * 2, (uint8_t)(Threshold >> 3)) ||
			!WriteReg(reg + i * 2 + 1, low))
			return false;
	}
	return true;
}

bool AccelAdxl372::SetActivityTime(uint8_t Samples)
{
	return vbInitialized && WriteReg(REG_TIME_ACT, Samples);
}

bool AccelAdxl372::SetInactivityTime(uint16_t Samples)
{
	return vbInitialized && WriteReg(REG_TIME_INACT, (uint8_t)(Samples >> 8)) &&
		WriteReg(REG_TIME_INACT + 1, (uint8_t)Samples);
}

bool AccelAdxl372::ConfigureInterrupts(uint8_t Int1, uint8_t Int2)
{
	if (!vbInitialized || !WriteReg(REG_INT1, Int1) || !WriteReg(REG_INT2, Int2))
		return false;
	vInt1 = Int1;
	vInt2 = Int2;
	return true;
}

bool AccelAdxl372::WakeOnEvent(bool Enable, int Threshold)
{
	if (!vbInitialized)
		return false;
	if (!Enable)
		return SetDetection(ADXL372_DETECT_ACTIVITY, 0, false, false) &&
			ConfigureInterrupts(vInt1 & ~0x20, vInt2);
	if (Threshold < 0 || Threshold > 2047)
		return false;
	return SetDetection(ADXL372_DETECT_ACTIVITY, (uint16_t)Threshold, false, true) &&
		SetActivityTime(1) &&
		ConfigureInterrupts(vInt1 | 0x20, vInt2);
}

bool AccelAdxl372::ConfigureFifo(Adxl372FifoMode Mode,
	Adxl372FifoFormat Format, uint16_t Entries)
{
	uint8_t ctl;
	uint8_t prior;

	if (!vbInitialized || Mode > ADXL372_FIFO_OLDEST ||
		Format > ADXL372_FIFO_XYZ_PEAK || Entries > 512)
		return false;
	prior = vPower;
	if (!SetPowerMode(ADXL372_STANDBY))
		return false;
	ctl = ((uint8_t)Format << 3) | ((uint8_t)Mode << 1) | (Entries > 255 ? 1 : 0);
	if (!WriteReg(REG_FIFO_CTL, 0) ||
		!WriteReg(REG_FIFO_SAMPLES, (uint8_t)Entries) ||
		!WriteReg(REG_FIFO_CTL, ctl))
	{
		SetPowerMode((Adxl372PowerMode)(prior & 3));
		return false;
	}
	vFifoControl = ctl;
	vFifoEntries = Entries;
	return SetPowerMode((Adxl372PowerMode)(prior & 3));
}

bool AccelAdxl372::FlushFifo()
{
	uint8_t control;

	if (!vbInitialized)
		return false;
	control = vFifoControl;
	return WriteReg(REG_FIFO_CTL, 0) && WriteReg(REG_FIFO_CTL, control);
}

bool AccelAdxl372::Status(Adxl372Status &Result)
{
	uint8_t raw[4];

	if (!vbInitialized || !ReadReg(REG_STATUS, raw, 4))
		return false;
	Result.Status1 = raw[0];
	Result.Status2 = raw[1];
	Result.FifoEntries = (((uint16_t)raw[2] & 3) << 8) | raw[3];
	return true;
}

bool AccelAdxl372::ReadFifo(AccelSensorRawData_t &Data)
{
	Adxl372FifoSample sample;

	if (!ReadFifo(sample) || sample.AxisMask != 7)
		return false;
	Data = sample.Accel;
	return true;
}

bool AccelAdxl372::ReadFifo(Adxl372FifoSample &Sample)
{
	Adxl372Status status;
	uint8_t raw[6];
	uint8_t format;
	uint8_t mask;
	int count;
	int offset;
	AccelSensorRawData_t sample;

	if (!vbInitialized || !(vFifoControl & 6))
		return false;
	if (!Status(status))
		return false;
	if (status.Status1 & 8)
	{
		vFifoOverruns++;
		FlushFifo();
		return false;
	}

	format = (vFifoControl >> 3) & 7;
	mask = 0;
	count = 0;
	switch (format)
	{
	case ADXL372_FIFO_XYZ:
	case ADXL372_FIFO_XYZ_PEAK:
		mask = 7;
		count = 3;
		break;
	case ADXL372_FIFO_X:
		mask = 1;
		count = 1;
		break;
	case ADXL372_FIFO_Y:
		mask = 2;
		count = 1;
		break;
	case ADXL372_FIFO_XY:
		mask = 3;
		count = 2;
		break;
	case ADXL372_FIFO_Z:
		mask = 4;
		count = 1;
		break;
	case ADXL372_FIFO_XZ:
		mask = 5;
		count = 2;
		break;
	case ADXL372_FIFO_YZ:
		mask = 6;
		count = 2;
		break;
	default:
		return false;
	}

	// Leave a complete sample set in the FIFO to avoid a concurrent stream
	// update overwriting a partially retrieved multi-axis sample.
	if (status.FifoEntries < (uint16_t)(count * 2))
		return false;
	if (!ReadReg(REG_FIFO_DATA, raw, count * 2))
		return false;

	sample = vData;
	offset = 0;
	if (mask & 1)
	{
		sample.X = Decode12(raw[offset], raw[offset + 1]);
		offset += 2;
	}
	if (mask & 2)
	{
		sample.Y = Decode12(raw[offset], raw[offset + 1]);
		offset += 2;
	}
	if (mask & 4)
		sample.Z = Decode12(raw[offset], raw[offset + 1]);

	sample.Timestamp = vpTimer ? vpTimer->uSecond() : vData.Timestamp + 1;
	vData = sample;
	Sample.Accel = sample;
	Sample.AxisMask = mask;
	Sample.Peak = format == ADXL372_FIFO_XYZ_PEAK;
	return true;
}

bool AccelAdxl372::InterruptEvents(Adxl372Status &Events)
{
	Events = vLastInterrupt;
	vLastInterrupt.Status1 = 0;
	vLastInterrupt.Status2 = 0;
	vLastInterrupt.FifoEntries = 0;
	return true;
}

void AccelAdxl372::IntHandler()
{
	Adxl372Status status;

	if (!Status(status))
		return;
	vLastInterrupt.Status1 |= status.Status1;
	vLastInterrupt.Status2 |= status.Status2;
	vLastInterrupt.FifoEntries = status.FifoEntries;
	if (status.Status1 & 8)
	{
		vFifoOverruns++;
		FlushFifo();
		return;
	}
	if ((status.Status1 & 1) && (vFifoControl & 6) == 0)
		UpdateData();
}

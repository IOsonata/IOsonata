/**-------------------------------------------------------------------------
@file accel_adxl372.h
@brief ADXL372 high-g accelerometer SPI/I2C driver
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
#ifndef __ACCEL_ADXL372_H__
#define __ACCEL_ADXL372_H__

#include "sensors/accel_sensor.h"

#define ADXL372_PART_ID 0xFA
#define ADXL372_I2C_ADDR 0x53

enum Adxl372PowerMode
{
	ADXL372_STANDBY = 0,
	ADXL372_WAKEUP = 1,
	ADXL372_INSTANT_ON = 2,
	ADXL372_MEASUREMENT = 3
};
enum Adxl372FifoMode
{
	ADXL372_FIFO_BYPASS = 0,
	ADXL372_FIFO_STREAM = 1,
	ADXL372_FIFO_TRIGGER = 2,
	ADXL372_FIFO_OLDEST = 3
};
enum Adxl372FifoFormat
{
	ADXL372_FIFO_XYZ = 0,
	ADXL372_FIFO_X = 1,
	ADXL372_FIFO_Y = 2,
	ADXL372_FIFO_XY = 3,
	ADXL372_FIFO_Z = 4,
	ADXL372_FIFO_XZ = 5,
	ADXL372_FIFO_YZ = 6,
	ADXL372_FIFO_XYZ_PEAK = 7
};
enum Adxl372Detection
{
	ADXL372_DETECT_ACTIVITY = 0,
	ADXL372_DETECT_ACTIVITY2 = 1,
	ADXL372_DETECT_INACTIVITY = 2
};
struct Adxl372FifoSample
{
	AccelSensorRawData_t Accel;
	uint8_t AxisMask;
	bool Peak;
};

struct Adxl372Status
{
	uint8_t Status1;
	uint8_t Status2;
	uint16_t FifoEntries;
};

class AccelAdxl372 : public AccelSensor
{
public:
	AccelAdxl372();
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool StartSampling();
	virtual bool UpdateData();
	virtual uint16_t Scale(uint16_t Value);
	virtual bool WakeOnEvent(bool Enable, int Threshold);
	virtual void IntHandler();

	/** Select the closest rate in mHz (ties lower); read SamplingFrequency()
	 * for the applied rate. Returns false if uninitialized or transfer fails. */
	bool SetOutputRate(uint32_t MilliHz);
	bool SetBandwidth(uint8_t Value);
	bool SetWakeupRate(uint8_t Value);
	bool SetPowerMode(Adxl372PowerMode Mode);
	bool SetAutosleep(bool Enable, uint8_t LinkLoop = 1);
	bool SetDetection(Adxl372Detection Type, uint16_t Threshold, bool Referenced, bool Enable);
	bool SetActivityTime(uint8_t Samples);
	bool SetInactivityTime(uint16_t Samples);
	bool ConfigureInterrupts(uint8_t Int1, uint8_t Int2);
	bool ConfigureFifo(Adxl372FifoMode Mode, Adxl372FifoFormat Format, uint16_t Entries);
	bool FlushFifo();
	bool ReadFifo(AccelSensorRawData_t &Data);
	bool ReadFifo(Adxl372FifoSample &Sample);
	bool InterruptEvents(Adxl372Status &Events);
	bool ReadPeak(AccelSensorRawData_t &Data);
	bool Status(Adxl372Status &Result);
	uint32_t FifoOverruns() const
	{
		return vFifoOverruns;
	}

private:
	bool ReadReg(uint8_t Reg, uint8_t *pData, int Len);
	bool WriteReg(uint8_t Reg, uint8_t Value);
	bool UpdateField(uint8_t Reg, uint8_t Mask, uint8_t Value);
	bool DecodeXYZ(uint8_t Reg, AccelSensorRawData_t &Data);
	static int16_t Decode12(uint8_t High, uint8_t Low);
	bool vbInitialized;
	uint8_t vPower;
	uint8_t vMeasure;
	uint8_t vInt1;
	uint8_t vInt2;
	uint8_t vFifoControl;
	uint16_t vFifoEntries;
	uint32_t vFifoOverruns;
	Adxl372Status vLastInterrupt;
};
#endif

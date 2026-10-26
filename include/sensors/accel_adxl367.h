/**-------------------------------------------------------------------------
@file accel_adxl367.h
@brief Analog Devices ADXL367 three-axis accelerometer, SPI/I2C
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
#ifndef __ACCEL_ADXL367_H__
#define __ACCEL_ADXL367_H__

#include "sensors/accel_sensor.h"

#define ADXL367_I2C_ADDR0 0x1D
#define ADXL367_I2C_ADDR1 0x53
#define ADXL367_PART_ID 0xF7

#define ADXL367_EVT_DATA_READY   0x0001
#define ADXL367_EVT_FIFO_READY   0x0002
#define ADXL367_EVT_FIFO_WMARK   0x0004
#define ADXL367_EVT_FIFO_OVERRUN 0x0008
#define ADXL367_EVT_ACTIVITY     0x0010
#define ADXL367_EVT_INACTIVITY   0x0020
#define ADXL367_EVT_AWAKE        0x0040
#define ADXL367_EVT_TAP_ONE      0x0100
#define ADXL367_EVT_TAP_TWO      0x0200

typedef enum
{
	ADXL367_FIFO_DISABLED = 0,
	ADXL367_FIFO_OLDEST = 1,
	ADXL367_FIFO_STREAM = 2,
	ADXL367_FIFO_TRIGGERED = 3
} ADXL367_FIFO_MODE;

typedef enum
{
	ADXL367_FIFO_XYZ = 0,
	ADXL367_FIFO_XYZ_TEMP = 4,
	ADXL367_FIFO_XYZ_ADC = 8
} ADXL367_FIFO_FORMAT;

class AccelAdxl367 : public AccelSensor
{
public:
	AccelAdxl367();
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool StartSampling();
	virtual bool UpdateData();
	virtual uint16_t Scale(uint16_t Value);
	virtual bool WakeOnEvent(bool bEnable, int Threshold);
	virtual void IntHandler();

	/** Select the closest rate in mHz (ties lower); read SamplingFrequency()
	 * for the applied rate. Returns false if uninitialized or transfer fails. */
	bool SetSamplingFrequency(uint32_t FreqMilliHz);
	bool ConfigureInterrupts(uint16_t Int1Map, uint16_t Int2Map);
	bool Status(uint16_t &Events);
	bool ConfigureActivity(uint16_t Threshold, uint8_t Time, bool Referenced);
	bool ConfigureInactivity(uint16_t Threshold, uint16_t Time, bool Referenced);
	bool ConfigureTap(uint8_t Threshold, uint8_t Duration, uint8_t Latency, uint8_t Window, bool DoubleTap);
	bool ConfigureFifo(ADXL367_FIFO_MODE Mode, ADXL367_FIFO_FORMAT Format, uint16_t SampleSets);
	bool ReadFifo(AccelSensorRawData_t &Data);
	/** Read configured XYZ plus temperature/ADC FIFO sample set. */
	bool ReadFifo(AccelSensorRawData_t &Data, int16_t &Auxiliary);
	/** Run the manufacturer's two-phase self-test, using a caller delay in ms. */
	bool SelfTest(void (*DelayMs)(uint32_t));
	bool ReadInterruptEvents(uint16_t &Events);
	bool FlushFifo();
	bool EnableTemperature(bool Enable);
	bool EnableAdc(bool Enable);
	bool ReadTemperature(int16_t &Raw, int32_t &CentiCelsius);
	bool ReadAdc(int16_t &Raw);
	bool EnablePedometer(bool Enable);
	bool ReadSteps(uint16_t &Steps);
	bool ResetSteps();
	bool SetLowPower(bool AutoSleep, bool Wakeup, uint8_t NoiseMode);
	uint32_t FifoOverruns() const
	{
		return vFifoOverruns;
	}

private:
	bool ReadReg(uint8_t Reg, uint8_t *pData, int Len);
	bool WriteReg(uint8_t Reg, uint8_t Value);
	bool WriteField(uint8_t Reg, uint8_t Mask, uint8_t Value);
	bool Read14(uint8_t Reg, int16_t &Value);
	bool ResetFifo();
	static int16_t Sign14(uint16_t Value);

	bool vbInitialized;
	bool vbFifoEnabled;
	uint8_t vFilter;
	uint8_t vPower;
	uint8_t vFifoCtl;
	uint8_t vFifoReadMode;
	uint16_t vFifoSamples;
	uint16_t vInt1Map;
	uint16_t vInt2Map;
	uint32_t vFifoOverruns;
};
#endif

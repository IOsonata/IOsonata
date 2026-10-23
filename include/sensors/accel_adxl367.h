/**-------------------------------------------------------------------------
@file accel_adxl367.h
@brief ADXL367 14-bit accelerometer interface (I2C/SPI)
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __ACCEL_ADXL367_H__
#define __ACCEL_ADXL367_H__

#include "sensors/accel_sensor.h"

#define ADXL367_I2C_ADDR0 0x1D
#define ADXL367_I2C_ADDR1 0x53
#define ADXL367_PART_ID 0xF7

class AccelAdxl367 : public AccelSensor {
public:
	AccelAdxl367() : vbInitialized(false), vFilter(0x23) {}
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool StartSampling();
	virtual bool UpdateData();
	virtual uint16_t Scale(uint16_t Value);
	virtual bool WakeOnEvent(bool bEnable, int Threshold);
	bool ConfigureInterrupts(uint8_t Int1Map, uint8_t Int2Map);
	bool ConfigureFifo(uint16_t WatermarkSamples);
	bool Status(uint8_t &Flags);
	bool ReadFifo(AccelSensorRawData_t &Data);

private:
	bool ReadReg(uint8_t Reg, uint8_t *pData, int Len);
	bool WriteReg(uint8_t Reg, uint8_t Value);
	bool vbInitialized;
	uint8_t vFilter;
	uint8_t vInt1Map = 0;
	uint8_t vInt2Map = 0;
	uint8_t vFifoControl = 0;
	bool vbFifoEnabled = false;
};

#endif

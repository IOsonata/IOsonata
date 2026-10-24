/**-------------------------------------------------------------------------
@file color_bh1749.h

@brief ROHM BH1749NUC RGB/IR color sensor (I2C)

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __COLOR_BH1749_H__
#define __COLOR_BH1749_H__

#include <stddef.h>
#include <stdint.h>
#include "device_intrf.h"

#define BH1749_I2C_ADDR0					0x38U
#define BH1749_I2C_ADDR1					0x39U
#define BH1749_PART_ID						0x0DU

typedef enum __ColorBh1749Gain {
	COLOR_BH1749_GAIN_1X = 1,
	COLOR_BH1749_GAIN_32X = 3,
} ColorBh1749Gain_t;

typedef enum __ColorBh1749MeasureTime {
	COLOR_BH1749_MEASURE_120MS = 2,
	COLOR_BH1749_MEASURE_240MS = 3,
	COLOR_BH1749_MEASURE_35MS = 5,
} ColorBh1749MeasureTime_t;

typedef enum __ColorBh1749IntSource {
	COLOR_BH1749_INT_RED = 0,
	COLOR_BH1749_INT_GREEN = 1,
	COLOR_BH1749_INT_BLUE = 2,
} ColorBh1749IntSource_t;

typedef enum __ColorBh1749IntPersistence {
	COLOR_BH1749_INT_EACH_MEASUREMENT = 0,
	COLOR_BH1749_INT_UPDATE_EACH_MEASUREMENT = 1,
	COLOR_BH1749_INT_4_CONSECUTIVE = 2,
	COLOR_BH1749_INT_8_CONSECUTIVE = 3,
} ColorBh1749IntPersistence_t;

/** Raw channel counts; BH1749 does not provide a calibrated lux/CCT conversion. */
typedef struct __ColorBh1749_Data {
	uint16_t Red;
	uint16_t Green;
	uint16_t Blue;
	uint16_t Ir;
	uint16_t Green2;
} ColorBh1749Data_t;

class ColorBh1749 {
public:
	ColorBh1749() : vpIntrf(NULL), vDevAddr(BH1749_I2C_ADDR0), vbInitialized(false) {}

	/** Detect device, select 120-ms measurement and 1x RGB/IR gains, and enable it. */
	bool Init(DeviceIntrf * const pIntrf, uint32_t DevAddr = BH1749_I2C_ADDR0);
	bool Enable();
	bool Disable();

	/** Configure RGB/IR gains and measurement time. */
	bool Configure(ColorBh1749Gain_t RgbGain, ColorBh1749Gain_t IrGain,
		ColorBh1749MeasureTime_t MeasureTime);

	/**
	 * Configure the threshold interrupt. Thresholds are raw channel counts.
	 * The interrupt output is active when the selected channel is outside
	 * the inclusive LowThreshold..HighThreshold window.
	 */
	bool ConfigureInterrupt(ColorBh1749IntSource_t Source,
		uint16_t LowThreshold, uint16_t HighThreshold,
		ColorBh1749IntPersistence_t Persistence);
	bool DisableInterrupt();

	/**
	 * Read the interrupt status. Reading the interrupt register clears the
	 * current interrupt condition in the device.
	 */
	bool ReadInterruptStatus(bool &Active);
	bool ClearInterrupt();

	/** Returns false until VALID is set or if a bus transfer fails. */
	bool Read(ColorBh1749Data_t &Data);

private:
	bool ReadReg(uint8_t Reg, uint8_t *pData, int Len);
	bool WriteReg(uint8_t Reg, uint8_t Value);
	bool WriteReg16(uint8_t Reg, uint16_t Value);

	DeviceIntrf *vpIntrf;
	uint32_t vDevAddr;
	bool vbInitialized;
};

#endif

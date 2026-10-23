/**-------------------------------------------------------------------------
@file color_bh1749.h
@brief ROHM BH1749NUC RGB/IR color sensor (I2C)
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __COLOR_BH1749_H__
#define __COLOR_BH1749_H__

#include <stdint.h>
#include "device_intrf.h"

#define BH1749_I2C_ADDR0          0x38
#define BH1749_I2C_ADDR1          0x39
#define BH1749_PART_ID            0x0D

/** Raw channel counts; not calibrated illuminance or color temperature. */
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

	/** Detect device, select 120-ms measurement and 1x channel gains. */
	bool Init(DeviceIntrf * const pIntrf, uint32_t DevAddr = BH1749_I2C_ADDR0);
	bool Enable();
	bool Disable();
	/** Returns false until VALID is set or if a bus transfer fails. */
	bool Read(ColorBh1749Data_t &Data);

private:
	bool ReadReg(uint8_t Reg, uint8_t *pData, int Len);
	bool WriteReg(uint8_t Reg, uint8_t Value);

	DeviceIntrf *vpIntrf;
	uint32_t vDevAddr;
	bool vbInitialized;
};

#endif

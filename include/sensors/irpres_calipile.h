/**-------------------------------------------------------------------------
@file	irpres_calipile.h

@brief	Implementation of Excelitas IR presence sensor Calipile

@author	Hoang Nguyen Hoan
@date	Apr. 14, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#ifndef __IRPRES_CALIPILE_H__
#define __IRPRES_CALIPILE_H__

#include <stdint.h>

#include "sensor.h"

#define CALIPILE_I2C_DEVADDR			0xC

#define CALIPILE_TP_REG					1	// Combines the TPobject & TPambient 5 bytes
#define CALIPILE_TPLP_REG				5	// Combines TPobjectLP1 & TPobjectLP2 5 bytes
#define CALIPILE_TP_AMBLP3_REG			10	// TP ambLP3 2 bytes
#define CALIPILE_TP_OBJLP2_FROZEN_REG	12	// TP objectLP2 frozen 3 bytes
#define CALIPILE_TP_PRESENCE_REG		15
#define CALIPILE_TP_MOTION_REG			16
#define CALIPILE_TP_AMB_SHOCK_REG		17
#define CALIPILE_INT_STATUS_REG			18	// Interrupt status
#define CALIPILE_INT_STATUS_TIMER				(1<<0)
#define CALIPILE_INT_STATUS_FLAG_TP_AMBSHOCK	(1<<1)
#define CALIPILE_INT_STATUS_FLAG_TP_MOTION		(1<<2)
#define CALIPILE_INT_STATUS_FLAG_TP_PRESENCE	(1<<3)
#define CALIPILE_INT_STATUS_FLAG_TP_OT			(1<<4)
#define CALIPILE_INT_STATUS_SIGN_TP_AMBSHOCK	(1<<5)
#define CALIPILE_INT_STATUS_SIGN_TP_MOTION		(1<<6)
#define CALIPILE_INT_STATUS_SIGN_TP_PRESENCEH	(1<<7)

#define CALIPILE_CHIP_STATUS_REG		19	// Chip status
#define CALIPILE_CHIP_STATUS_TIMER				(1<<0)
#define CALIPILE_CHIP_STATUS_FLAG_TP_AMBSHOCK	(1<<1)
#define CALIPILE_CHIP_STATUS_FLAG_TP_MOTION		(1<<2)
#define CALIPILE_CHIP_STATUS_FLAG_TP_PRESENCE	(1<<3)
#define CALIPILE_CHIP_STATUS_FLAG_TP_OT			(1<<4)
#define CALIPILE_CHIP_STATUS_SIGN_TP_AMBSHOCK	(1<<5)
#define CALIPILE_CHIP_STATUS_SIGN_TP_MOTION		(1<<6)
#define CALIPILE_CHIP_STATUS_SIGN_TP_PRESENCEH	(1<<7)

#define CALIPILE_LOWPASS_TCONST_REG		20	// Low pass constant Slp, 2 bytes
#define CALIPILE_LOWPASS_TCONST_LP1_MASK		(0xf<<0)
#define CALIPILE_LOWPASS_TCONST_LP2_MASK		(0xf<<4)
#define CALIPILE_LOWPASS_TCONST_LP3_MASK		(7<<8)

#define CALIPILE_TP_PRESENCE_THRS_REG	22	// TP presence threshold
#define CALIPILE_TP_MOTION_THRS_REG		23	// TP motion threshold
#define CALIPILE_TP_AMB_SHOCK_THRS_REG	24	// TP amb shock threshold
#define CALIPILE_INT_MASK_REG			25	// Interrupt mask
#define CALIPILE_INT_MASK_TIMER					(1<<0)
#define CALIPILE_INT_MASK_TP_AMBSHOCK			(1<<1)
#define CALIPILE_INT_MASK_TP_MOTION				(1<<2)
#define CALIPILE_INT_MASK_TP_PRESENCE			(1<<3)
#define CALIPILE_INT_MASK_TP_OT					(1<<4)

#define CALIPILE_INT_MASK2_REG			26
#define CALIPILE_INT_MASK2_CYCLE_TIME_MASK		(3<<0)
#define CALIPILE_INT_MASK2_SRC_SEL_MASK			(3>>2)
#define CALIPILE_INT_MASK2_TP_OT_DIR			(1<<4)

#define CALIPILE_TIMER_INT_REG			27	// Timer interrupt
#define CALIPILE_TP_OT_THRS_REG			28	// TP ot threshold, 2 bytes

#define CALIPILE_EEPROM_CTRL_REG		31	// EEPROM control register
#define CALIPILE_EEPROM_CTRL_ACCESS_DIS			0	// EEPROM access disable
#define CALIPILE_EEPROM_CTRL_ACCESS_EN			0x80// EEPROM access enable

#define CALIPILE_EEPROM_PROTOCL_REG		32
#define CALIPILE_EEPROM_CHECKSUM_REG	33	// 2 bytes
#define CALIPILE_EEPROM_LOOKUP_IDX_REG	41
#define CALIPILE_EEPROM_PTAT25_REG		42	// 2 bytes
#define CALIPILE_EEPROM_M_REG			44
#define CALIPILE_EEPROM_U0_REG			46
#define CALIPILE_EEPROM_UOUT1_REG		48
#define CALIPILE_EEPROM_TOBJ1_REG		50
#define CALIPILE_EEPROM_DEVADDR_REG		63

typedef struct __Calipile_TPData {
	uint32_t TPobject;
	uint16_t TPambient;
	uint32_t TPobjectLP1;
	uint32_t TPobjectLP2;
	uint16_t TPambLP3;
	uint32_t TPobjectLP2Frozen;
	uint8_t TPpresence;
	uint8_t TPmotion;
	uint8_t TPambshock;
} CalipileTP_t;

typedef struct __Calipile_Cfg {
	uint8_t DevAddr;	// I2C device address
	DevEvtHandler_t EvHandler;
} CalipileCfg_t;

#ifdef __cplusplus

class Calipile : public Sensor {
public:
	bool Init(const CalipileCfg_t &Cfg, DeviceIntrf * const pIntrf);

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable() { return true; }

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * This function is used to put the device in lowest power mode
	 * possible so that the Enable function can wake up without full
	 * initialization.
	 */
	virtual void Disable() {}

	/**
	 * @brief	Reset device to it initial default state
	 */
	virtual void Reset() {}

	/**
	 * @brief	Start sampling data
	 *
	 * This is a require implementation by sensor implementer.\n
	 * This function initiates sensor to do actual measurement.
	 *
	 * @return	true - success
	 * 			false - in case of error or sensor busy measuring
	 */
	virtual bool StartSampling() { return true; }

	/**
	 * @brief	Read sensor and update internal data with new readings
	 *
	 * This function should be called by a periodic timer to update
	 * sensor data in SENSOR_OPMODE_CONTINUOUS or interrupt or when Read is called
	 * in SENSOR_OPMODE_SINGLE
	 *
	 * @return	true - New data is updated
	 */
	virtual bool UpdateData();

	/**
	 * @brief	Interrupt handler (optional)
	 *
	 * Sensor that supports interrupt can implement this to handle interrupt.
	 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
	 */
	virtual void IntHandler();

private:
	void ReadTP();

	CalipileTP_t vTPData;
};

extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif // __IRPRES_CALIPILE_H__

/**-------------------------------------------------------------------------
@file	tir_calipile.cpp

@brief	Implementation of Excelitas Thermal IR presence sensor Calipile

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
#include <math.h>

#include "idelay.h"
#include "convutil.h"
#include "coredev/uart.h"
#include "sensors/tir_calipile.h"

bool Calipile::Init(const CalipileCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	vpIntrf = pIntrf;
	vpTimer = pTimer;

	// Calipile require a I2C general call re-load cmd (4) to load the device
	// address from EEMPROM into its I2C interface
	uint8_t cmd = 4;

	vpIntrf->Tx(0, &cmd, 1);

	msDelay(10);

	vDevAddr = Cfg.DevAddr;

	uint8_t reg = CALIPILE_EEPROM_CTRL_REG;
	uint8_t d = CALIPILE_EEPROM_CTRL_ACCESS_EN;
	Write(&reg, 1, &d, 1);

	// Read eeprom data
	uint8_t buff[32];
	reg = CALIPILE_EEPROM_PROTOCL_REG;
	Sensor::Read(&reg, 1, buff, 32);

	uint16_t cs = buff[0];
	uint16_t ecs = (uint16_t)(buff[1] << 8UL) | buff[2];

	for (int i = 3; i < 32; i++)
	{
		cs += buff[i];
	}

	if (cs != ecs)
	{
		return false;
	}

	if ((buff[31] & CALIPILE_I2C_DEVADDR) != (vDevAddr & CALIPILE_I2C_DEVADDR))
	{
		printf("wrong addr\r\n");
		return false;
	}

	reg = CALIPILE_EEPROM_CTRL_REG;
	d = 0;
	Write(&reg, 1, &d, 1);

	memset(&vData, 0, sizeof(CalipileData_t));

	SetEvtHandler(Cfg.EvHandler);

	// Read calibration data
	vLookup = buff[9];
	vPTat25 = ((buff[10] & 0x7fULL) << 8UL) | buff[11];
	vM = (float)((uint16_t)(buff[12] << 8UL) | buff[13]) / 100.0;
	vU0 = ((uint32_t)(buff[14] << 8UL) | buff[15]) + 32768;
	vUout1 = ((uint32_t)(buff[16] << 8UL) | buff[17]) << 1UL;
	vTobj1 = (float)buff[18] + 273.15;

	float f = pow(vTobj1, 3.8) - pow(25.0 + 273.15, 3.8);
	vkFactor = (float)(vUout1 - vU0) / f;

	// TODO: other require initializations

	reg = 0;
	Sensor::Read(&reg, 1, buff, 32);

	uint8_t reg_highsens[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0x8D,0x0D,15,30,30,0x09,0x04,20,0xFF,0x00,0,0}; // high sensitivity
	uint8_t reg_lowsens[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
							  0x8B,0x0B,255,255,30,0x1c,0x13,20, 0x7f, 0xFF,0,0};

	reg = 0;
	Write(&reg,1, reg_lowsens, 32);

	reg = CALIPILE_TIMER_INT_REG;
	d = 1000 / 30 - 1;
	Write(&reg,1, &d, 1);

	reg = CALIPILE_INT_STATUS_REG;
	Sensor::Read(&reg, 1, &d, 1);

	if (Cfg.IntEn)
	{
/*		reg = CALIPILE_TP_PRESENCE_THRS_REG;
		d = 250;
		Write(&reg, 1, &d, 1);
		reg = CALIPILE_TP_MOTION_THRS_REG;
		Write(&reg, 1, &d, 1);
		reg = CALIPILE_TP_AMB_SHOCK_THRS_REG;
		Write(&reg, 1, &d, 1);

		reg = CALIPILE_INT_MASK_REG;
		d = 0x1f;
		Write(&reg, 1, &d, 1);
*/
	}

	reg = CALIPILE_INT_STATUS_REG;
//	Sensor::Read(&reg, 1, &d, 1);

	reg = CALIPILE_CHIP_STATUS_REG;
//	Sensor::Read(&reg, 1, &d, 1);

	return true;
}

/**
 * @brief	Read sensor and update internal data with new readings
 *
 * This function should be called by a periodic timer to update
 * sensor data in SENSOR_OPMODE_CONTINUOUS or interrupt or when Read is called
 * in SENSOR_OPMODE_SINGLE
 *
 * @return	true - New data is updated
 */
bool Calipile::UpdateData()
{
	uint8_t buff[30];
	uint8_t reg = 0;
	uint64_t d = 0;

	if (vpTimer)
	{
		vData.Timestamp = vpTimer->mSecond();
	}
	else
	{
		vData.Timestamp++;
	}

	Sensor::Read(&reg, 1, buff, 30);

	vData.TPobject = (buff[1] << 16UL) | (buff[2] << 8UL) | buff[3];
	vData.TPobject >>= 7;
	vData.TPambient = ((buff[3] & 0x7f) << 8UL) | buff[4];

	vData.TPobjectLP1 = (buff[5] << 16UL) | (buff[6] << 8UL) | buff[7];
	vData.TPobjectLP1 >>= 4;

	vData.TPobjectLP2 = ((buff[7] & 0xF) << 16UL) | (buff[8] << 8UL) | buff[9];
	vData.TPambLP3  = (buff[10]<<8UL) | buff[11];
	vData.TPobjectLP2Frozen = (buff[12] << 16UL) | (buff[13] << 8UL) | buff[14];
	vData.TPpresence = buff[15];
	vData.TPmotion = buff[16];
	vData.TPambshock = buff[17];

	uint16_t cstatus = buff[19];
	//printf("cstatus: %x, %d %d\r\n", cstatus, vTPData.TPpresence, vTPData.TPambshock);

	//printf("Obj:%d, amb:%d, objLP1:%d, objLP2:%d, ambLP3:%d, Froz:%d, pres:%d, m:%d, s:%d\r\n", vTPData.TPobject, vTPData.TPambient, vTPData.TPobjectLP1, vTPData.TPobjectLP2, vTPData.TPambLP3, vTPData.TPobjectLP2Frozen, vTPData.TPpresence, vTPData.TPmotion, vTPData.TPambshock);
	float tobj = CalcObjTemp(vData.TPobject, vData.TPambient);

	vTempData.Timestamp = vData.Timestamp;
	vTempData.Temperature = (tobj - 273.15) * 100.0;
//extern UART g_Uart;

//	g_Uart.printf("tobj = %.4f %d %d %d\r\n", tobj - 273.15, vData.TPpresence, vData.TPmotion, vData.TPambshock);

	return true;
}

/**
 * @brief	Initialize sensor (require implementation).
 *
 * @param 	CfgData : Reference to configuration data
 * @param	pIntrf 	: Pointer to interface to the sensor.
 * 					  This pointer will be kept internally
 * 					  for all access to device.
 * 					  DONOT delete this object externally
 * @param	pTimer	: Pointer to timer for retrieval of time stamp
 * 					  This pointer will be kept internally
 * 					  for all access to device.
 * 					  DONOT delete this object externally
 *
 * @return
 * 			- true	: Success
 * 			- false	: Failed
 */
bool Calipile::Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	(void)CfgData;
	(void)pIntrf;
	(void)pTimer;

	return true;
}

/**
 * @brief	Interrupt handler (optional)
 *
 * Sensor that supports interrupt can implement this to handle interrupt.
 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
 */
void Calipile::IntHandler()
{
	uint8_t reg = CALIPILE_INT_STATUS_REG;
	uint8_t status[2];

	Sensor::Read(&reg, 1, status, 2);

	if (status[0] & 0x1f)
	{
		UpdateData();
	}

	Sensor::Read(&reg, 1, status, 2);
}

#if 0
void Calipile::ReadTP()
{
	uint8_t buff[30];
	uint8_t reg = 0;
	uint64_t d = 0;

	Read(&reg, 1, buff, 30);

	vTPData.TPobject = (buff[1] << 16UL) | (buff[2] << 8UL) | buff[3];
	vTPData.TPobject >>= 7;
	vTPData.TPambient = ((buff[3] & 0x7f) << 8UL) | buff[4];

	vTPData.TPobjectLP1 = (buff[5] << 16UL) | (buff[6] << 8UL) | buff[7];
	vTPData.TPobjectLP1 >>= 4;

	vTPData.TPobjectLP2 = ((buff[7] & 0xF) << 16UL) | (buff[8] << 8UL) | buff[9];
	vTPData.TPambLP3  = (buff[10]<<8UL) | buff[11];
	vTPData.TPobjectLP2Frozen = (buff[12] << 16UL) | (buff[13] << 8UL) | buff[14];
	vTPData.TPpresence = buff[15];
	vTPData.TPmotion = buff[16];
	vTPData.TPambshock = buff[17];

	uint16_t cstatus = buff[19];
	//printf("cstatus: %x, %d %d\r\n", cstatus, vTPData.TPpresence, vTPData.TPambshock);

	//printf("Obj:%d, amb:%d, objLP1:%d, objLP2:%d, ambLP3:%d, Froz:%d, pres:%d, m:%d, s:%d\r\n", vTPData.TPobject, vTPData.TPambient, vTPData.TPobjectLP1, vTPData.TPobjectLP2, vTPData.TPambLP3, vTPData.TPobjectLP2Frozen, vTPData.TPpresence, vTPData.TPmotion, vTPData.TPambshock);
	float tobj = CalcObjTemp(vTPData.TPobject, vTPData.TPambient);

	printf("tobj = %.4f %.4f\r\n", tobj, tobj - 273.15);
}
#endif

float Calipile::CalcObjTemp(int32_t TPobj, int16_t TPamb)
{
	float tamb = CalcAmbientTemp(TPamb);
	float f = pow(tamb, 3.8);
	//printf("tamb: %.4f %.4f\r\n", tamb - 273.15, f);
	//printf("Tamb : %d %d\r\n", tak, (int32_t)((f - 273.15)*100.0));
	return pow((TPobj - vU0) / vkFactor + f, 1.0 / 3.8);
}


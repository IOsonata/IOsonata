/**-------------------------------------------------------------------------
@file	ble_require_srvc.cpp

@brief	Implement minimum require Bluetooth services


@author	Hoang Nguyen Hoan
@date	Oct. 29, 2022

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
#include "bluetooth/ble_srvc.h"

static uint16_t s_GenAccCharApperance = 0;
static uint8_t s_GenAccCharDevNameBuffer[255] = "TestName";

static BleSrvcChar_t s_GenericAccessChar[] = {
	{
		// Read characteristic
		.Uuid = BT_UUID_GATT_DEVICE_NAME,
		.MaxDataLen = 255,
		.Property =	BLESRVC_CHAR_PROP_READ | BLESRVC_CHAR_PROP_VARLEN,
		.pDesc = NULL,						// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pValue = s_GenAccCharDevNameBuffer,
		.ValueLen = 10,
		//.RdHandler = NULL,
		//.WrHandler = NULL,
		//.CharVal = {255, 10, s_GenAccCharDevNameBuffer},					// pointer to char default values
	},
	{
		// Read characteristic
		.Uuid = BT_UUID_GATT_APPEARANCE,
		.MaxDataLen = 2,
		.Property =	BLESRVC_CHAR_PROP_READ,
		.pDesc = NULL,						// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pValue = (uint8_t*)&s_GenAccCharApperance,
		.ValueLen = 2,
		//.RdHandler = NULL,
		//.WrHandler = NULL,
		//.CharVal = {2, 2, (uint8_t*)&s_GenAccCharApperance},					// pointer to char default values
	},
};

static const BleSrvcCfg_t s_GenericAccessSrvcCfg = {
	.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.bCustom = false,
	.UuidBase = {0,},		// Base UUID
	.UuidSrvc = BT_UUID_GATT_SERVICE_GENERIC_ACCESS,		// Service UUID
	.NbChar = sizeof(s_GenericAccessChar) / sizeof(BleSrvcChar_t),				// Total number of characteristics for the service
	.pCharArray = s_GenericAccessChar,				// Pointer a an array of characteristic
};

static BleSrvc_t s_GenericAccessSrvc;

static BtGattCharSrvcChanged_t s_GenAttCharSrvcChanged = {0,};

static BleSrvcChar_t s_GenericAttributeChar[] = {
	{
		// Read characteristic
		.Uuid = BT_UUID_GATT_CHAR_SERVICE_CHANGED,
		.MaxDataLen = sizeof(BtGattCharSrvcChanged_t),
		.Property =	BLESRVC_CHAR_PROP_INDICATE,
		.pDesc = NULL,						// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pValue = &s_GenAttCharSrvcChanged,
		.ValueLen = 0,
		//.RdHandler = NULL,
		//.WrHandler = NULL,
		//.CharVal = {PACKET_SIZE, 0, s_RxCharValMem},					// pointer to char default values
	},
};

static BleSrvcCfg_t s_GenericAttributeSrvcCfg = {
	.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.bCustom = false,
	.UuidBase = {0,},		// Base UUID
	.UuidSrvc = BT_UUID_GATT_SERVICE_GENERIC_ATTRIBUTE,		// Service UUID
	.NbChar = sizeof(s_GenericAttributeChar) / sizeof(BleSrvcChar_t),				// Total number of characteristics for the service
	.pCharArray = s_GenericAttributeChar,				// Pointer a an array of characteristic
};

static BleSrvc_t s_GenericAttributeSrvc;

extern "C" void BleAppInitGenericServices()
{
    BleSrvcInit(&s_GenericAttributeSrvc, &s_GenericAttributeSrvcCfg);
    BleSrvcInit(&s_GenericAccessSrvc, &s_GenericAccessSrvcCfg);
}





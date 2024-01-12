/**-------------------------------------------------------------------------
@file	bt_gap.cpp

@brief	Implement Bluetooth Generic Access Profile (GAP)

Core Bluetooth Vol.1, Part A, 6.2

The Bluetooth system defines a base profile which all Bluetooth devices implement.
This profile is the Generic Access Profile (GAP), which defines the basic
requirements of a Bluetooth device. For instance, for BR/EDR, it defines a
Bluetooth device to include the Radio, Baseband, Link Manager, L2CAP, and the
Service Discovery protocol functionality; for LE, it defines the Physical Layer,
Link Layer, L2CAP, Security Manager, Attribute Protocol and Generic Attribute Profile.
This ties all the various layers together to form the basic requirements for a
Bluetooth device. It also describes the behaviors and methods for device discovery,
connection establishment, security, authentication, association models and
service discovery.

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
#include <memory.h>

#include "convutil.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_adv.h"

#ifndef BT_GAP_DEVNAME_MAX_LEN
#define BT_GAP_DEVNAME_MAX_LEN			64
#endif

#ifndef BT_GAP_CONN_MAX_COUNT
#define BT_GAP_CONN_MAX_COUNT			10
#endif

alignas(4) static BtGapConnection_t s_BtGapConnection[BT_GAP_CONN_MAX_COUNT] = {
	{.Hdl = BT_GATT_HANDLE_INVALID,},
};

#if 1
//static uint16_t s_BtGapCharApperance = 0;
//static char s_BtGapCharDevName[BT_GAP_DEVNAME_MAX_LEN];
//static BtGattPreferedConnParams_t s_BtGapCharPerferedConnParams = {
//	USEC_TO_1250(7500), MSEC_TO_1_25(40), 0, 400
//};

static BtGattChar_t s_BtGapChar[] = {
	{
		// Device name characteristic
		.Uuid = BT_UUID_GATT_DEVICE_NAME,
		.MaxDataLen = BT_GAP_DEVNAME_MAX_LEN,
		.Property =	BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_VALEN,
		.pDesc = NULL,						// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		//.pValue = s_BtGapCharDevName,
		//.ValueLen = 0,
	},
	{
		// Appearance characteristic
		.Uuid = BT_UUID_GATT_APPEARANCE,
		.MaxDataLen = 2,
		.Property =	BT_GATT_CHAR_PROP_READ,
		.pDesc = NULL,						// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		//.pValue = &s_BtGapCharApperance,
		//.ValueLen = 2,
	},
	{
		// Prefered connection parameter characteristic
		.Uuid = BT_UUID_GATT_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
		.MaxDataLen = sizeof(BtGattPreferedConnParams_t),
		.Property =	BT_GATT_CHAR_PROP_READ,
		.pDesc = NULL,						// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
//		.pValue = &s_BtGapCharPerferedConnParams,
//		.ValueLen = sizeof(BtGattPreferedConnParams_t),
	},
};

static const BtGattSrvcCfg_t s_BtGapSrvcCfg = {
	//.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.bCustom = false,
	.UuidBase = {0,},						// Base UUID
	.UuidSrvc = BT_UUID_GATT_SERVICE_GENERIC_ACCESS,	// Service UUID
	.NbChar = sizeof(s_BtGapChar) / sizeof(BtGattChar_t),// Total number of characteristics for the service
	.pCharArray = s_BtGapChar,				// Pointer a an array of characteristic
};

static BtGattSrvc_t s_BtGapSrvc;

static BtGattCharSrvcChanged_t s_BtGattCharSrvcChanged = {0,};

static BtGattChar_t s_BtGattChar[] = {
	{
		// Read characteristic
		.Uuid = BT_UUID_GATT_CHAR_SERVICE_CHANGED,
		.MaxDataLen = sizeof(BtGattCharSrvcChanged_t),
		.Property =	BT_GATT_CHAR_PROP_INDICATE,
		.pDesc = NULL,						// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		//.pValue = &s_BtGattCharSrvcChanged,
		//.ValueLen = 0,
	},
};

static BtGattSrvcCfg_t s_BtGattSrvcCfg = {
	//.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.bCustom = false,
	.UuidBase = {0,},		// Base UUID
	.UuidSrvc = BT_UUID_GATT_SERVICE_GENERIC_ATTRIBUTE,		// Service UUID
	.NbChar = sizeof(s_BtGattChar) / sizeof(BtGattChar_t),				// Total number of characteristics for the service
	.pCharArray = s_BtGattChar,				// Pointer a an array of characteristic
};

static BtGattSrvc_t s_BtGattSrvc;
#if 0
alignas(4) static uint8_t s_BleAppAdvBuff[260];
alignas(4) static sdc_hci_cmd_le_set_adv_data_t &s_BleAppAdvData = *(sdc_hci_cmd_le_set_adv_data_t*)s_BleAppAdvBuff;
alignas(4) static BleAdvPacket_t s_BleAppAdvPkt = { sizeof(s_BleAppAdvData.adv_data), 0, s_BleAppAdvData.adv_data};

alignas(4) static sdc_hci_cmd_le_set_ext_adv_data_t &s_BleAppExtAdvData = *(sdc_hci_cmd_le_set_ext_adv_data_t*)s_BleAppAdvBuff;
alignas(4) static BleAdvPacket_t s_BleAppExtAdvPkt = { 255, 0, s_BleAppExtAdvData.adv_data};

alignas(4) static uint8_t s_BleAppSrBuff[260];

alignas(4) static sdc_hci_cmd_le_set_scan_response_data_t &s_BleAppSrData = *(sdc_hci_cmd_le_set_scan_response_data_t*)s_BleAppSrBuff;
alignas(4) static BleAdvPacket_t s_BleAppSrPkt = { sizeof(s_BleAppSrData.scan_response_data), 0, s_BleAppSrData.scan_response_data};

alignas(4) static sdc_hci_cmd_le_set_ext_scan_response_data_t &s_BleAppExtSrData = *(sdc_hci_cmd_le_set_ext_scan_response_data_t*)s_BleAppSrBuff;
alignas(4) static BleAdvPacket_t s_BleAppExtSrPkt = { 255, 0, s_BleAppExtSrData.scan_response_data};
#endif

__attribute__((weak)) void BtGapSetDevName(const char *pName)
{
	if (pName == nullptr)
	{
		return;
	}

	// Update name in GAP characteristic
	BtGattCharSetValue(&s_BtGapChar[0], (void*)pName, strlen(pName));
}

__attribute__((weak)) char * const BtGapGetDevName()
{
	return (char*)s_BtGapChar[0].pValue;
}

__attribute__((weak)) void BtGapSetAppearance(uint16_t Val)
{
	BtGattCharSetValue(&s_BtGapChar[1], &Val, 2);
}

__attribute__((weak)) void BtGapSetPreferedConnParam(BtGattPreferedConnParams_t *pVal)
{
	BtGattCharSetValue(&s_BtGapChar[2], pVal, sizeof(BtGattPreferedConnParams_t));
}

__attribute__((weak)) void BtGapServiceInit()//BtGattSrvc_t * const pSrvc)
{
	BtGattSrvcAdd(&s_BtGattSrvc, &s_BtGattSrvcCfg);
	BtGattSrvcAdd(&s_BtGapSrvc, &s_BtGapSrvcCfg);
//	BtGattSrvcAdd(pSrvc, &s_BtGapSrvcCfg);

	//BtGattPreferedConnParams_t connparm = {
//		USEC_TO_1250(7500), MSEC_TO_1_25(40), 0, 400
//	};

//	BtGattCharSetValue(&s_BtGapChar[2], &connparm, sizeof(BtGattPreferedConnParams_t));
}
#endif

__attribute__((weak)) void BtGapParamInit(const BtGapCfg_t *pCfg)
{
}

void BtGapInit(const BtGapCfg_t *pCfg)
{
	memset(s_BtGapConnection, 0xFF, sizeof(s_BtGapConnection));

	if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
	{
		BtGattSrvcAdd(&s_BtGattSrvc, &s_BtGattSrvcCfg);
		BtGattSrvcAdd(&s_BtGapSrvc, &s_BtGapSrvcCfg);
	}

	if (pCfg->Role & (BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL))
	{
		BtGapParamInit(pCfg);
	}
}

bool isBtGapConnected()
{
	for (int i = 0; i < BT_GAP_CONN_MAX_COUNT; i++)
	{
		if (s_BtGapConnection[i].Hdl != BT_GATT_HANDLE_INVALID)
		{
			return true;
		}
	}

	return false;
}

uint16_t BtGapGetConnection()
{
	for (int i = 0; i < BT_GAP_CONN_MAX_COUNT; i++)
	{
		if (s_BtGapConnection[i].Hdl != BT_GATT_HANDLE_INVALID)
		{
			return s_BtGapConnection[i].Hdl;
		}
	}

	return BT_GATT_HANDLE_INVALID;
}

size_t BtGapGetConnectedHandles(uint16_t *pHdl, size_t MaxCount)
{
	size_t count = 0;

	for (int i = 0; i < BT_GAP_CONN_MAX_COUNT; i++)
	{
		if (s_BtGapConnection[i].Hdl != BT_GATT_HANDLE_INVALID)
		{
			pHdl[count++] = s_BtGapConnection[i].Hdl;
		}
	}

	return count;
}

bool BtGapAddConnection(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6])
{
	for (int i = 0; i < BT_GAP_CONN_MAX_COUNT; i++)
	{
		if (s_BtGapConnection[i].Hdl == BT_GATT_HANDLE_INVALID)
		{
			s_BtGapConnection[i].Hdl = ConnHdl;
			s_BtGapConnection[i].Role = Role;
			s_BtGapConnection[i].PeerAddrType = AddrType;
			memcpy(s_BtGapConnection[i].PeerAddr, PeerAddr, 6);

			return true;
		}
	}

	return false;
}

void BtGapDeleteConnection(uint16_t Hdl)
{
	for (int i = 0; i < BT_GAP_CONN_MAX_COUNT; i++)
	{
		if (s_BtGapConnection[i].Hdl == Hdl)
		{
			memset(&s_BtGapConnection[i], 0xFF, sizeof(BtGapConnection_t));
		}
	}

	if (isBtGapConnected() == false)
	{
		BtGattSrvcDisconnected(&s_BtGapSrvc);
		BtGattSrvcDisconnected(&s_BtGattSrvc);
	}
}

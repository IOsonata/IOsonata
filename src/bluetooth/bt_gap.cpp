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

#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"

#ifndef BT_GAP_DEVNAME_MAX_LEN
#define BT_GAP_DEVNAME_MAX_LEN			64
#endif

static BtGattChar_t s_BtGapChar[] = {
	BT_CHAR(BT_UUID_CHARACTERISTIC_DEVICE_NAME,
	        BT_GAP_DEVNAME_MAX_LEN,
	        BT_GATT_CHAR_PROP_READ,
	        NULL),
	BT_CHAR(BT_UUID_CHARACTERISTIC_APPEARANCE,
	        2,
	        BT_GATT_CHAR_PROP_READ,
	        NULL),
	BT_CHAR(BT_UUID_CHARACTERISTIC_PERIPH_PREFERRED_CONN_PARAM,
	        sizeof(BtGattPreferedConnParams_t),
	        BT_GATT_CHAR_PROP_READ,
	        NULL),
};

static BtGattSrvc_t s_BtGapSrvc = BT_SRVC_STD(BT_UUID_GATT_SERVICE_GENERIC_ACCESS, s_BtGapChar);

static BtGattChar_t s_BtGattChar[] = {
	BT_CHAR(BT_UUID_CHARACTERISTIC_SERVICE_CHANGED,
	        sizeof(BtGattCharSrvcChanged_t),
	        BT_GATT_CHAR_PROP_INDICATE,
	        NULL),
};

static BtGattSrvc_t s_BtGattSrvc = BT_SRVC_STD(BT_UUID_GATT_SERVICE_GENERIC_ATTRIBUTE, s_BtGattChar);

__attribute__((weak)) void BtGapSetDevName(const char *pName)
{
	if (pName == nullptr)
	{
		return;
	}

	BtGattChar_t *p = &s_BtGapChar[0];
	size_t l = strlen(pName);
	if (p->MaxDataLen > 0 && l > (size_t)(p->MaxDataLen - 1))
	{
		l = p->MaxDataLen - 1;
	}

	BtGattCharSetValue(p, (void*)pName, l);
	if (p->pValue != nullptr)
	{
		((uint8_t*)p->pValue)[p->ValueLen] = '\0';
	}
}

__attribute__((weak)) const char *BtGapGetDevName()
{
	return (const char*)s_BtGapChar[0].pValue;
}

__attribute__((weak)) void BtGapSetAppearance(uint16_t Val)
{
	uint8_t buf[2];
	buf[0] = (uint8_t)(Val & 0xFF);
	buf[1] = (uint8_t)(Val >> 8);
	BtGattCharSetValue(&s_BtGapChar[1], buf, 2);
}

__attribute__((weak)) void BtGapSetPreferedConnParam(BtGattPreferedConnParams_t *pVal)
{
	if (pVal == nullptr)
	{
		return;
	}
	BtGattCharSetValue(&s_BtGapChar[2], pVal, sizeof(BtGattPreferedConnParams_t));
}

__attribute__((weak)) void BtGapParamInit(const BtGapCfg_t *pCfg)
{
	(void)pCfg;
}

void BtGapInit(const BtGapCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return;
	}

	if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
	{
		BtGattSrvcAdd(&s_BtGattSrvc);
		BtGattSrvcAdd(&s_BtGapSrvc);
	}

	if (pCfg->Role & (BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL))
	{
		BtGapParamInit(pCfg);
	}
}

__attribute__((weak)) bool BtGapConnSecGet(uint16_t ConnHdl, BtConnSec_t *pSec)
{
	if (pSec == nullptr)
	{
		return false;
	}

	BtDevice_t *p = BtPeerFindByHdl(ConnHdl);
	if (p == nullptr)
	{
		return false;
	}

	*pSec = p->Conn.Sec;
	if (BtSmpBonded(ConnHdl))
	{
		pSec->Flags |= BT_GAP_SEC_FLAG_BONDED;
	}
	return true;
}

__attribute__((weak)) void BtGapConnSecSet(uint16_t ConnHdl, const BtConnSec_t *pSec)
{
	if (pSec == nullptr)
	{
		return;
	}

	BtDevice_t *p = BtPeerFindByHdl(ConnHdl);
	if (p != nullptr)
	{
		p->Conn.Sec = *pSec;
	}
}

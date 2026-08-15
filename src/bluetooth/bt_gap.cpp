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
This ties all the various layers together to form the basic requirements of a
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
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include <atomic>
#include <memory.h>

#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gatt_init.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"

#ifndef BT_GAP_DEVNAME_MAX_LEN
#define BT_GAP_DEVNAME_MAX_LEN			64
#endif

static std::atomic<bool> s_BtGattInitActive;
static std::atomic<uint8_t> s_BtGattInitError;

void BtGattInitStatusReset(void)
{
	s_BtGattInitError.store(BT_GATT_INIT_ERROR_NONE,
			std::memory_order_relaxed);
	s_BtGattInitActive.store(true, std::memory_order_release);
}

void BtGattInitStatusFail(BtGattInitError_t Error)
{
	if (Error == BT_GATT_INIT_ERROR_NONE ||
		!s_BtGattInitActive.load(std::memory_order_acquire))
	{
		return;
	}

	uint8_t expected = BT_GATT_INIT_ERROR_NONE;
	s_BtGattInitError.compare_exchange_strong(expected, (uint8_t)Error,
			std::memory_order_acq_rel, std::memory_order_acquire);
}

bool BtGattInitStatusActive(void)
{
	return s_BtGattInitActive.load(std::memory_order_acquire);
}

bool BtGattInitStatusOk(void)
{
	return s_BtGattInitError.load(std::memory_order_acquire) ==
		BT_GATT_INIT_ERROR_NONE;
}

bool BtGattInitStatusComplete(void)
{
	bool ok = BtGattInitStatusOk();
	s_BtGattInitActive.store(false, std::memory_order_release);
	return ok;
}

BtGattInitError_t BtGattInitStatusErrorGet(void)
{
	return (BtGattInitError_t)s_BtGattInitError.load(
			std::memory_order_acquire);
}

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
	// LE GATT Security Levels, Core 5.4 Vol 3 Part C 12.7. Read only with no
	// encryption, no authentication and no authorization, because a client
	// reads it to find out what security to establish and cannot have
	// established it yet.
	BT_CHAR(BT_UUID_CHARACTERISTIC_LE_GATT_SECURITY_LEVELS,
	        BT_GAP_SEC_LEVELS_MAX_LEN,
	        BT_GATT_CHAR_PROP_READ,
	        NULL),
};

// Index of the security levels characteristic in the array above.
#define BT_GAP_SEC_LEVELS_CHAR_IDX		3

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

	if (!BtGattCharSetValue(p, (void*)pName, l))
	{
		BtGattInitStatusFail(BT_GATT_INIT_ERROR_VALUE_SET);
		return;
	}
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
	if (!BtGattCharSetValue(&s_BtGapChar[1], buf, 2))
	{
		BtGattInitStatusFail(BT_GATT_INIT_ERROR_VALUE_SET);
	}
}

__attribute__((weak)) void BtGapSetPreferedConnParam(BtGattPreferedConnParams_t *pVal)
{
	if (pVal == nullptr)
	{
		return;
	}
	if (!BtGattCharSetValue(&s_BtGapChar[2], pVal,
			sizeof(BtGattPreferedConnParams_t)))
	{
		BtGattInitStatusFail(BT_GATT_INIT_ERROR_VALUE_SET);
	}
}

__attribute__((weak)) void BtGapParamInit(const BtGapCfg_t *pCfg)
{
	(void)pCfg;
}

// The LE GATT Security Levels value, Core 5.4 Vol 3 Part C 12.7: a sequence of
// Security Level Requirements, each a Security Mode octet followed by a
// Security Level octet, both written as the number used in their definitions.
//
// The spec makes meeting any one of the requirements listed for a mode
// sufficient, so a longer sequence is a weaker statement, not a stronger one.
// One requirement is what a server with a single policy has to say.
bool BtGapSecLevelsSet(const uint8_t *pReq, size_t Len)
{
	if (pReq == nullptr || Len < 2 || (Len & 1) != 0 ||
		Len > BT_GAP_SEC_LEVELS_MAX_LEN)
	{
		return false;
	}

	if (!BtGattCharSetValue(&s_BtGapChar[BT_GAP_SEC_LEVELS_CHAR_IDX],
							(void*)pReq, Len))
	{
		BtGattInitStatusFail(BT_GATT_INIT_ERROR_VALUE_SET);
		return false;
	}

	return true;
}

// Default value from the configured security type. This says what the
// configuration names and nothing more.
//
// A build that refuses legacy pairing requires mode 1 level 4 wherever this
// would answer level 3, because level 3 tells a client that authenticated
// legacy pairing is enough when it is not. The generic bt_smp.cpp path is one
// such build: BtSmpAuthConfig forces the Secure Connections bit. On the nRF54
// port it depends on BtSecBmScOnlySet, which is a runtime choice. Neither is
// visible from here, so an application whose stack refuses legacy calls
// BtGapSecLevelsSet with mode 1 level 4 instead.
static void BtGapSecLevelsDefault(uint32_t SecType)
{
	uint8_t req[2];

	switch (SecType)
	{
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
			req[0] = 1; req[1] = 2;		// encrypted, unauthenticated
			break;
		case BTGAP_SECTYPE_STATICKEY_MITM:
			req[0] = 1; req[1] = 3;		// encrypted, authenticated
			break;
		case BTGAP_SECTYPE_LESC_MITM:
			req[0] = 1; req[1] = 4;		// authenticated Secure Connections
			break;
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
			req[0] = 2; req[1] = 1;		// data signing, unauthenticated
			break;
		case BTGAP_SECTYPE_SIGNED_MITM:
			req[0] = 2; req[1] = 2;		// data signing, authenticated
			break;
		case BTGAP_SECTYPE_NONE:
		default:
			req[0] = 1; req[1] = 1;		// no security
			break;
	}

	(void)BtGapSecLevelsSet(req, sizeof(req));
}

void BtGapInit(const BtGapCfg_t *pCfg)
{
	BtGattInitStatusReset();
	if (pCfg == nullptr)
	{
		BtGattInitStatusFail(BT_GATT_INIT_ERROR_INVALID_CFG);
		return;
	}

	if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
	{
		// The value has to be in place before the service is registered: a
		// port that copies the characteristic into a vendor database at
		// registration would otherwise take an empty one, and 12.7 requires
		// the value to be static during a connection.
		BtGapSecLevelsDefault(pCfg->SecType);

		if (!BtGattSrvcAdd(&s_BtGattSrvc) ||
			!BtGattSrvcAdd(&s_BtGapSrvc))
		{
			BtGattInitStatusFail(BT_GATT_INIT_ERROR_SERVICE_ADD);
		}
	}

	if (pCfg->Role & (BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL))
	{
		BtGapParamInit(pCfg);
	}

	if ((pCfg->Role & BT_GAP_ROLE_PERIPHERAL) == 0)
	{
		(void)BtGattInitStatusComplete();
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

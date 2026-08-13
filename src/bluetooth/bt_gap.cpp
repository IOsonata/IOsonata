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

#include "bluetooth/bt_app.h"
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

// These name OPEN rather than inheriting the application's security type
// because the spec fixes their permissions, per characteristic:
//
//   Device Name  Vol 3 Part C 12.1, Table 12.3. "When the device is
//                discoverable, the Device Name characteristic value shall be
//                readable without authentication or authorization." Not
//                readable that way once undiscoverable, which this
//                declaration does not yet distinguish.
//   Appearance   Vol 3 Part C 12.2, Table 12.4. "The Appearance
//                characteristic value shall be readable without
//                authentication or authorization."
//   PPCP         Vol 3 Part C 12.3, Table 12.5. "shall be readable.
//                Authentication and authorization may be defined by a higher
//                layer specification or be implementation specific." So OPEN
//                is a choice here, not a requirement, and it is the one that
//                lets a peer read the preferred parameters before pairing.
//   Sec Levels   Vol 3 Part C 12.7, Table 12.11. "Read Only, No Encryption
//                required, No Authentication, No Authorization." A client
//                reads this to learn what the server will require, so it has
//                to work before the client has met any of it.
//
// A platform constraint points the same way on one port: the Nordic
// SoftDevice takes only a write permission in sd_ble_gap_device_name_set, so
// the read cannot be secured there at all.
static BtGattChar_t s_BtGapChar[] = {
	BT_CHAR(BT_UUID_CHARACTERISTIC_DEVICE_NAME,
	        BT_GAP_DEVNAME_MAX_LEN,
	        BT_GATT_CHAR_PROP_READ,
	        NULL, .SecType = BT_GAP_SECTYPE_OPEN),
	BT_CHAR(BT_UUID_CHARACTERISTIC_APPEARANCE,
	        2,
	        BT_GATT_CHAR_PROP_READ,
	        NULL, .SecType = BT_GAP_SECTYPE_OPEN),
	BT_CHAR(BT_UUID_CHARACTERISTIC_PERIPH_PREFERRED_CONN_PARAM,
	        sizeof(BtGattPreferedConnParams_t),
	        BT_GATT_CHAR_PROP_READ,
	        NULL, .SecType = BT_GAP_SECTYPE_OPEN),
	BT_CHAR(BT_UUID_CHARACTERISTIC_LE_GATT_SECURITY_LEVELS,
	        BT_GAP_SEC_LEVEL_VALUE_MAX,
	        BT_GATT_CHAR_PROP_READ,
	        NULL, .SecType = BT_GAP_SECTYPE_OPEN),
};

// Index of the LE GATT Security Levels characteristic above. The setters
// address the array by position, so keep this in step with the order.
#define BT_GAP_CHAR_IDX_SEC_LEVELS		3

static BtGattSrvc_t s_BtGapSrvc = BT_SRVC_STD(BT_UUID_GATT_SERVICE_GENERIC_ACCESS, s_BtGapChar);

// Service Changed names OPEN because the spec fixes its permissions too:
// Vol 3 Part G 7.1, Table 7.2 gives the declaration "No Authentication, No
// Authorization", and Table 7.3 gives the value "No Authentication, No
// Authorization, Not Readable, Not Writable". Indication is the only
// permitted operation, which is what the property below says.
//
// Naming OPEN rather than leaving NONE also keeps it out of the application
// inheritance that BtGapInit publishes below.
static BtGattChar_t s_BtGattChar[] = {
	BT_CHAR(BT_UUID_CHARACTERISTIC_SERVICE_CHANGED,
	        sizeof(BtGattCharSrvcChanged_t),
	        BT_GATT_CHAR_PROP_INDICATE,
	        NULL, .SecType = BT_GAP_SECTYPE_OPEN),
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

static bool BtGapSecurityLevelValid(uint8_t Mode, uint8_t Level)
{
	// Core Vol 3 Part C 10.2. Mode 1 has four levels, mode 2 has two. Mode 3
	// is BR/EDR only and has no place in this characteristic, which reports
	// the requirements of a GATT server on an LE connection.
	if (Mode == BT_GAP_SEC_MODE_1)
	{
		return Level >= BT_GAP_SEC_MODE1_LEVEL_NONE &&
			Level <= BT_GAP_SEC_MODE1_LEVEL_LESC;
	}

	if (Mode == BT_GAP_SEC_MODE_2)
	{
		return Level >= BT_GAP_SEC_MODE2_LEVEL_SIGN_NO_AUTH &&
			Level <= BT_GAP_SEC_MODE2_LEVEL_SIGN_AUTH;
	}

	return false;
}

// True when a port actually registered this characteristic, so it has a handle
// and somewhere to hold a value.
//
// BT_CHAR names neither field, so a static declaration starts with ValHdl 0
// and pValue null, and BtGattSrvcAdd fills them in. A port whose stack owns
// the GAP service declines to register it and leaves both as they were. Note
// the handle is 0 there and not BT_ATT_HANDLE_INVALID, which is 0xFFFF: a
// check for the invalid handle alone does not catch this.
static bool BtGapCharRegistered(const BtGattChar_t *pChar)
{
	return pChar != nullptr && pChar->pValue != nullptr &&
		   pChar->ValHdl != 0 && pChar->ValHdl != BT_ATT_HANDLE_INVALID;
}

bool BtGapSetSecurityLevels(const uint8_t *pRequirements, size_t Count)
{
	if (pRequirements == nullptr || Count == 0 ||
		Count > BT_GAP_SEC_LEVEL_REQ_MAX)
	{
		return false;
	}

	// Nothing to write to. Answering here keeps a value set off a
	// characteristic with no handle, which on the SoftDevice ports means
	// sd_ble_gatts_value_set on handle 0.
	if (!BtGapCharRegistered(&s_BtGapChar[BT_GAP_CHAR_IDX_SEC_LEVELS]))
	{
		return false;
	}

	// Validate the whole set before writing any of it, so a bad entry late in
	// the list cannot leave the characteristic holding half an update.
	for (size_t i = 0; i < Count; i++)
	{
		if (BtGapSecurityLevelValid(pRequirements[i * 2],
			pRequirements[i * 2 + 1]) == false)
		{
			return false;
		}
	}

	return BtGattCharSetValue(&s_BtGapChar[BT_GAP_CHAR_IDX_SEC_LEVELS],
		(void*)pRequirements, Count * 2);
}

size_t BtGapGetSecurityLevels(uint8_t *pBuff, size_t BuffLen)
{
	const BtGattChar_t *p = &s_BtGapChar[BT_GAP_CHAR_IDX_SEC_LEVELS];
	size_t len = p->ValueLen;

	if (pBuff != nullptr && p->pValue != nullptr && BuffLen >= len)
	{
		memcpy(pBuff, p->pValue, len);
	}

	return len;
}

// The security type the application asked for, expressed as the single
// Security Level Requirement it amounts to. Core Vol 3 Part C 10.2.1 and
// 10.2.2 name the levels; the mapping is one to one because each security type
// names an authentication strength and whether the link is encrypted or
// signed.
static void BtGapSecurityLevelsFromSecType(uint8_t SecType, uint8_t Req[2])
{
	switch (SecType)
	{
		case BT_GAP_SECTYPE_STATICKEY_NO_MITM:
			Req[0] = BT_GAP_SEC_MODE_1;
			Req[1] = BT_GAP_SEC_MODE1_LEVEL_ENC_NO_AUTH;
			break;
		case BT_GAP_SECTYPE_STATICKEY_MITM:
			Req[0] = BT_GAP_SEC_MODE_1;
			Req[1] = BT_GAP_SEC_MODE1_LEVEL_ENC_AUTH;
			break;
		case BT_GAP_SECTYPE_LESC_MITM:
			Req[0] = BT_GAP_SEC_MODE_1;
			Req[1] = BT_GAP_SEC_MODE1_LEVEL_LESC;
			break;
		case BT_GAP_SECTYPE_SIGNED_NO_MITM:
			Req[0] = BT_GAP_SEC_MODE_2;
			Req[1] = BT_GAP_SEC_MODE2_LEVEL_SIGN_NO_AUTH;
			break;
		case BT_GAP_SECTYPE_SIGNED_MITM:
			Req[0] = BT_GAP_SEC_MODE_2;
			Req[1] = BT_GAP_SEC_MODE2_LEVEL_SIGN_AUTH;
			break;
		case BT_GAP_SECTYPE_NONE:
		default:
			Req[0] = BT_GAP_SEC_MODE_1;
			Req[1] = BT_GAP_SEC_MODE1_LEVEL_NONE;
			break;
	}
}

// --- Link procedures on an established connection ---
//
// Weak and refusing, so every port answers these whether or not it implements
// them, and a port that does overrides. bt_gap_hci runs them over standard
// HCI. The vendor host ports answer a PHY update the peer starts, in their
// BLE_GAP_EVT_PHY_UPDATE_REQUEST handler, but none of them can start one, so
// on those ports these refuse until a port implementation is written against
// the vendor call, sd_ble_gap_phy_update on the Nordic ports.
//
// Refusing is what an application can act on. The alternative is a link that
// silently stays on the PHY it had while the caller believes it asked.

__attribute__((weak)) bool BtGapSetPhy(uint16_t ConnHdl, uint8_t TxPhys,
	uint8_t RxPhys, uint16_t PhyOptions)
{
	(void)ConnHdl;
	(void)TxPhys;
	(void)RxPhys;
	(void)PhyOptions;

	return false;
}

__attribute__((weak)) bool BtGapReadPhy(uint16_t ConnHdl, uint8_t *pTxPhy,
	uint8_t *pRxPhy)
{
	(void)ConnHdl;
	(void)pTxPhy;
	(void)pRxPhy;

	return false;
}

__attribute__((weak)) bool BtGapSetDefaultPhy(uint8_t TxPhys, uint8_t RxPhys)
{
	(void)TxPhys;
	(void)RxPhys;

	return false;
}

__attribute__((weak)) bool BtGapSetDataLength(uint16_t ConnHdl,
	uint16_t TxOctets, uint16_t TxTime)
{
	(void)ConnHdl;
	(void)TxOctets;
	(void)TxTime;

	return false;
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

void BtGapInit(const BtGapCfg_t *pCfg)
{
	BtGattInitStatusReset();

	// Clear before validating so a refused configuration cannot leave the
	// requirement from an earlier one in force.
	g_BtAppData.SecType = BTGAP_SECTYPE_NONE;

	if (pCfg == nullptr)
	{
		BtGattInitStatusFail(BT_GATT_INIT_ERROR_INVALID_CFG);
		return;
	}

	// Publish the configured security type for BtGattSecTypeGet to resolve
	// against. It is the last step of the characteristic to service to
	// application walk, so a service written to inherit the application policy
	// reads it and nothing else. This belongs here rather than in each port's
	// BtAppInit: every port routes its BtAppCfg_t::SecType through this one
	// call, and every port registers its services after it, so one assignment
	// covers all four. It is set for every role, not only peripheral, because
	// a central applies the same requirement to the server it talks to.
	g_BtAppData.SecType = (BTGAP_SECTYPE)pCfg->SecType;

	if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
	{
		if (!BtGattSrvcAdd(&s_BtGattSrvc) ||
			!BtGattSrvcAdd(&s_BtGapSrvc))
		{
			BtGattInitStatusFail(BT_GATT_INIT_ERROR_SERVICE_ADD);
		}
	}

	if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
	{
		// The value is required to be static during a connection, so it is set
		// once here rather than tracked as security is negotiated. An
		// application whose real requirement is not the configured one calls
		// BtGapSetSecurityLevels afterwards.
		//
		// Only a port that registered the characteristic can be held to
		// writing it. Core Vol 3 Part C Table 12.2 lists LE GATT Security
		// Levels as Optional for both the LE Peripheral and the LE Central
		// role, so a stack that owns the GAP service and offers no 0x2BF5
		// leaves a device that conforms without it. The SoftDevice, sdk-nrf-bm
		// and CubeWBA are all in that position: they create 0x1800 themselves
		// and expose no way to add a characteristic to it. Treating that as a
		// failed value set stopped BtAppInit on all three.
		if (BtGapCharRegistered(&s_BtGapChar[BT_GAP_CHAR_IDX_SEC_LEVELS]))
		{
			uint8_t req[2];
			BtGapSecurityLevelsFromSecType(pCfg->SecType, req);
			if (BtGapSetSecurityLevels(req, 1) == false)
			{
				BtGattInitStatusFail(BT_GATT_INIT_ERROR_VALUE_SET);
			}
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

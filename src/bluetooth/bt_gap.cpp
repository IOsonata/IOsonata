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

#ifndef BT_GAP_DEVNAME_MAX_LEN
#define BT_GAP_DEVNAME_MAX_LEN			64
#endif

// Connection pool: app-sized via BtGapConnPoolInit. Layout in the buffer:
//
//   [ BtGapConnPoolHdr_t | BtGapConnection_t[0] | ... | BtGapConnection_t[N-1] ]
//
// SlotSize is stamped with the library's sizeof(BtGapConnection_t); init
// refuses buffers whose payload doesn't divide evenly by that size, which
// catches lib/app ABI drift at runtime. Free slot has Hdl == BT_ATT_HANDLE_INVALID.
#ifndef BT_GAP_CONN_POOL_DEFAULT_COUNT
#define BT_GAP_CONN_POOL_DEFAULT_COUNT	4
#endif

alignas(4) static uint8_t s_DefaultGapConnPoolMem[BT_GAP_CONN_POOL_MEMSIZE(BT_GAP_CONN_POOL_DEFAULT_COUNT)];
static BtGapConnPoolHdr_t *s_pGapConnPool = nullptr;

static inline BtGapConnection_t * GapConnSlots(void)
{
	return s_pGapConnPool ? (BtGapConnection_t*)(s_pGapConnPool + 1) : nullptr;
}

bool BtGapConnPoolInit(uint8_t *pMem, size_t MemSize)
{
	uint8_t *mem;
	size_t  size;

	if (pMem == nullptr || MemSize == 0)
	{
		mem  = s_DefaultGapConnPoolMem;
		size = sizeof(s_DefaultGapConnPoolMem);
	}
	else
	{
		mem  = pMem;
		size = MemSize;
	}

	if (size < BT_GAP_CONN_POOL_MEMSIZE(1))
	{
		return false;
	}

	size_t payload = size - sizeof(BtGapConnPoolHdr_t);
	if (payload % sizeof(BtGapConnection_t) != 0)
	{
		return false;
	}

	uint16_t count = (uint16_t)(payload / sizeof(BtGapConnection_t));
	if (count == 0)
	{
		return false;
	}

	BtGapConnPoolHdr_t *hdr = (BtGapConnPoolHdr_t*)mem;
	hdr->SlotSize = (uint16_t)sizeof(BtGapConnection_t);
	hdr->Count    = count;

	BtGapConnection_t *slots = (BtGapConnection_t*)(hdr + 1);
	for (uint16_t i = 0; i < count; i++)
	{
		memset(&slots[i], 0, sizeof(BtGapConnection_t));
		slots[i].Hdl = BT_ATT_HANDLE_INVALID;
	}

	s_pGapConnPool = hdr;
	return true;
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

	BtGattCharSetValue(&s_BtGapChar[0], (void*)pName, strlen(pName));
}

__attribute__((weak)) const char *BtGapGetDevName()
{
	return (const char*)s_BtGapChar[0].pValue;
}

__attribute__((weak)) void BtGapSetAppearance(uint16_t Val)
{
	// BLE wire format is little-endian; assemble byte-by-byte instead of
	// passing &Val so the value is not host-byte-order dependent.
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
}

void BtGapInit(const BtGapCfg_t *pCfg)
{
	// Slot reset is BtGapConnPoolInit's job; the port calls it from
	// BtAppInit before reaching here. If a port omits that call, the
	// lazy fallback below kicks in on the first access using the small
	// library default buffer.

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

bool isBtGapConnected()
{
	if (s_pGapConnPool == nullptr)
	{
		return false;
	}

	BtGapConnection_t *slots = GapConnSlots();
	for (uint16_t i = 0; i < s_pGapConnPool->Count; i++)
	{
		if (slots[i].Hdl != BT_ATT_HANDLE_INVALID)
		{
			return true;
		}
	}

	return false;
}

uint16_t BtGapGetConnection()
{
	if (s_pGapConnPool == nullptr)
	{
		return BT_ATT_HANDLE_INVALID;
	}

	BtGapConnection_t *slots = GapConnSlots();
	for (uint16_t i = 0; i < s_pGapConnPool->Count; i++)
	{
		if (slots[i].Hdl != BT_ATT_HANDLE_INVALID)
		{
			return slots[i].Hdl;
		}
	}

	return BT_ATT_HANDLE_INVALID;
}

size_t BtGapGetConnectedHandles(uint16_t *pHdl, size_t MaxCount)
{
	size_t count = 0;

	if (pHdl == nullptr || MaxCount == 0 || s_pGapConnPool == nullptr)
	{
		return 0;
	}

	BtGapConnection_t *slots = GapConnSlots();
	for (uint16_t i = 0; i < s_pGapConnPool->Count && count < MaxCount; i++)
	{
		if (slots[i].Hdl != BT_ATT_HANDLE_INVALID)
		{
			pHdl[count++] = slots[i].Hdl;
		}
	}

	return count;
}

bool BtGapAddConnection(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6])
{
	if (ConnHdl == BT_ATT_HANDLE_INVALID || s_pGapConnPool == nullptr)
	{
		return false;
	}

	BtGapConnection_t *slots = GapConnSlots();

	// Idempotent: if the handle is already tracked, refresh its fields.
	for (uint16_t i = 0; i < s_pGapConnPool->Count; i++)
	{
		if (slots[i].Hdl == ConnHdl)
		{
			slots[i].Role         = Role;
			slots[i].PeerAddrType = AddrType;

			if (PeerAddr != nullptr)
			{
				memcpy(slots[i].PeerAddr, PeerAddr, 6);
			}
			else
			{
				memset(slots[i].PeerAddr, 0, sizeof(slots[i].PeerAddr));
			}

			return true;
		}
	}

	// Otherwise, take the first free slot.
	for (uint16_t i = 0; i < s_pGapConnPool->Count; i++)
	{
		if (slots[i].Hdl == BT_ATT_HANDLE_INVALID)
		{
			slots[i].Hdl          = ConnHdl;
			slots[i].Role         = Role;
			slots[i].PeerAddrType = AddrType;

			if (PeerAddr != nullptr)
			{
				memcpy(slots[i].PeerAddr, PeerAddr, 6);
			}
			else
			{
				memset(slots[i].PeerAddr, 0, sizeof(slots[i].PeerAddr));
			}

			return true;
		}
	}

	return false;
}

void BtGapDeleteConnection(uint16_t Hdl)
{
	bool bRemoved = false;

	if (Hdl == BT_ATT_HANDLE_INVALID || s_pGapConnPool == nullptr)
	{
		return;
	}

	BtGapConnection_t *slots = GapConnSlots();
	for (uint16_t i = 0; i < s_pGapConnPool->Count; i++)
	{
		if (slots[i].Hdl == Hdl)
		{
			slots[i].Hdl = BT_ATT_HANDLE_INVALID;
			bRemoved = true;
		}
	}

	if (bRemoved && isBtGapConnected() == false)
	{
		BtGattSrvcDisconnected(&s_BtGapSrvc);
		BtGattSrvcDisconnected(&s_BtGattSrvc);
	}
}


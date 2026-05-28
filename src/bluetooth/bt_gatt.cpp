/**-------------------------------------------------------------------------
@file	bt_gatt.cpp

@brief	Bluetooth GATT  

Generic implementation & definitions of Bluetooth Generic Attribute Profile

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2022

@license

MIT License

Copyright (c) 2022, I-SYST, all rights reserved

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
#include <stdio.h>

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_gatt.h"

static BtGattSrvc_t *s_pBtGattSrvcList = nullptr;

BtGattSrvc_t * const BtGattGetSrvcList()
{
	return s_pBtGattSrvcList;
}

void BtGattInsertSrvcList(BtGattSrvc_t * const pSrvc)
{
	pSrvc->pPrev = nullptr;
	pSrvc->pNext = s_pBtGattSrvcList;

	if (s_pBtGattSrvcList)
	{
		s_pBtGattSrvcList->pPrev = pSrvc;
	}
	s_pBtGattSrvcList = pSrvc;
}

//size_t BtGattGetValue(BtAttDBEntry_t *pEntry, uint16_t Offset, uint8_t *pBuff)

__attribute__((weak)) bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar->Runtime.ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	// Static-backed characteristics serve reads directly from user .rodata
	// and reject runtime updates. Callers that need to push values should
	// use a DB-backed characteristic (pStaticVal left NULL at declaration).
	if (pChar->pStaticVal != nullptr)
	{
		return false;
	}

	int l = min((uint16_t)Len, pChar->MaxDataLen);

	memcpy(pChar->Runtime.pValue, pVal, l);
	pChar->Runtime.ValueLen = l;

	return true;
}

bool isBtGattCharNotifyEnabled(BtGattChar_t *pChar)
{
	if (pChar->Runtime.CccdHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	return pChar->Runtime.bNotify;
}

__attribute__((weak)) bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	uint8_t baseidx = 0;

	if (pSrvc == nullptr || pSrvc->pCharArray == nullptr || pSrvc->NbChar <= 0)
	{
		return false;
	}

	// Add base UUID to internal list for custom 128-bit services.
	if (pSrvc->bCustom)
	{
		baseidx = BtUuidAddBase(pSrvc->UuidBase);
	}

	pSrvc->Uuid = { baseidx, BT_UUID_TYPE_16, pSrvc->UuidSrvc };
	pSrvc->Hdl  = BT_ATT_HANDLE_INVALID;

	// Validate every static-backed characteristic up front. A pStaticVal
	// points to read-only user memory, so mixing it with WRITE / NOTIFY /
	// INDICATE makes no sense; fail loudly before allocating any DB entry.
	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		BtGattChar_t *c = &pSrvc->pCharArray[i];
		if (c->pStaticVal == nullptr)
		{
			continue;
		}
		if (c->Property & (BT_GATT_CHAR_PROP_WRITE
		                  | BT_GATT_CHAR_PROP_WRITE_WORESP
		                  | BT_GATT_CHAR_PROP_NOTIFY
		                  | BT_GATT_CHAR_PROP_INDICATE
		                  | BT_GATT_CHAR_PROP_AUTH_SIGNED))
		{
			return false;
		}
	}

	BtUuid16_t typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DECLARATIONS_PRIMARY_SERVICE };

	int l = sizeof(BtAttSrvcDeclar_t);

	BtAttDBEntry_t *srvcentry = BtAttDBAddEntry(&typeuuid, l);

	// Note: BtAttDBAddEntry has no rollback. Once any sub-entry succeeds
	// the bump-pointer and handle counter have advanced; on a later failure
	// inside this function the partial entries are leaked from the DB. The
	// service object itself is left with Hdl = BT_ATT_HANDLE_INVALID and
	// NOT inserted into s_pBtGattSrvcList, so callers walking the list see
	// only fully-registered services. A proper fix needs a BtAttDBUnwind
	// API; tracked separately.
	if (srvcentry == nullptr)
	{
		return false;
	}

	BtAttSrvcDeclar_t *srvcdec = (BtAttSrvcDeclar_t*) srvcentry->Data;

	srvcdec->Uuid = pSrvc->Uuid;
	srvcdec->pSrvc = pSrvc;

	pSrvc->Hdl = srvcentry->Hdl;

	BtGattChar_t *c = pSrvc->pCharArray;

	BtAttDBEntry_t *entry = nullptr;

	for (int i = 0; i < pSrvc->NbChar; i++, c++)
	{
		typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DECLARATIONS_CHARACTERISTIC };
		l = sizeof(BtAttCharDeclar_t);

		entry = BtAttDBAddEntry(&typeuuid, l);
		if (entry == nullptr)
		{
			pSrvc->Hdl = BT_ATT_HANDLE_INVALID;
			return false;
		}

		BtAttCharDeclar_t *chardec = (BtAttCharDeclar_t*)entry->Data;

		chardec->Uuid  = {baseidx, BT_UUID_TYPE_16, c->Uuid};
		chardec->pChar = c;

		c->Runtime.ValHdl      = BT_ATT_HANDLE_INVALID;
		c->Runtime.DescHdl     = BT_ATT_HANDLE_INVALID;
		c->Runtime.CccdHdl     = BT_ATT_HANDLE_INVALID;
		c->Runtime.SccdHdl     = BT_ATT_HANDLE_INVALID;
		c->Runtime.pSrvc       = pSrvc;
		c->Runtime.BaseUuidIdx = pSrvc->Uuid.BaseIdx;

		// Characteristic value. For static-backed chars allocate only the
		// header (handle slot + back-pointer); the actual bytes stay in the
		// user-supplied .rodata pointed to by pStaticVal. For DB-backed
		// chars allocate header + MaxDataLen bytes as before.
		typeuuid = {baseidx, BT_UUID_TYPE_16, c->Uuid };
		size_t valsize = sizeof(BtAttCharValue_t);
		if (c->pStaticVal == nullptr)
		{
			valsize += c->MaxDataLen;
		}
		entry = BtAttDBAddEntry(&typeuuid, valsize);
		if (entry == nullptr)
		{
			pSrvc->Hdl = BT_ATT_HANDLE_INVALID;
			return false;
		}
		BtAttCharValue_t *charval = (BtAttCharValue_t*)entry->Data;

		charval->pChar = c;
		c->Runtime.ValHdl = entry->Hdl;

		if (c->pStaticVal != nullptr)
		{
			c->Runtime.pValue   = (void*)c->pStaticVal;
			c->Runtime.ValueLen = c->MaxDataLen;
		}
		else
		{
			c->Runtime.pValue   = charval->Data;
			c->Runtime.ValueLen = 0;
		}

		c->Runtime.bNotify = false;
		c->Runtime.bIndic  = false;
		if (c->Property & (BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
		{
			// Characteristic Descriptor CCC
			typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION };
			l = sizeof(BtDescClientCharConfig_t);
			entry = BtAttDBAddEntry(&typeuuid, l);
			if (entry == nullptr)
			{
				pSrvc->Hdl = BT_ATT_HANDLE_INVALID;
				return false;
			}

			BtDescClientCharConfig_t *ccc = (BtDescClientCharConfig_t*)entry->Data;

			ccc->pChar  = c;
			ccc->CccVal = 0;
			c->Runtime.CccdHdl = entry->Hdl;
		}

		if (c->pDesc)
		{
			// Characteristic Description
			typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION };
			size_t dl = sizeof(BtDescCharUserDesc_t);
			entry = BtAttDBAddEntry(&typeuuid, dl);
			if (entry == nullptr)
			{
				pSrvc->Hdl = BT_ATT_HANDLE_INVALID;
				return false;
			}

			BtDescCharUserDesc_t *dcud = (BtDescCharUserDesc_t*)entry->Data;

			dcud->pChar = c;
			c->Runtime.DescHdl = entry->Hdl;
		}
	}

	BtGattInsertSrvcList(pSrvc);

	return true;
}

__attribute__((weak)) void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc)
{
	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		if (pSrvc->pCharArray[i].Runtime.CccdHdl != BT_ATT_HANDLE_INVALID)
		{
			pSrvc->pCharArray[i].Runtime.bNotify = false;
			pSrvc->pCharArray[i].Runtime.bIndic  = false;

			BtAttDBEntry_t *entry = BtAttDBFindHandle(pSrvc->pCharArray[i].Runtime.CccdHdl);
			if (entry)
			{
				BtDescClientCharConfig_t *p = (BtDescClientCharConfig_t*)entry->Data;
				p->CccVal = 0;
			}
		}
	}
}

__attribute__((weak)) void BtGattEvtHandler(uint32_t Evt, void * const pCtx)
{
	if (s_pBtGattSrvcList)
	{
		BtGattSrvc_t *p = s_pBtGattSrvcList;

		while (p)
		{
			BtGattSrvcEvtHandler(p, Evt, pCtx);

			p = p->pNext;
		}
	}
}

// HCI Number-Of-Completed-Packets event hook. The HCI controller reports
// per-connection packet completions but does not say which characteristic
// produced them. Without per-connection bookkeeping of the last enqueued
// char, the best we can do here is fire TxCompleteCB only on chars where
// a notification or indication could be in flight (i.e. the peer has
// subscribed). Chars with neither bNotify nor bIndic cannot have produced
// a notify packet and are skipped.
//
// TODO(per-conn-tx-complete): with the unified peer pool now in place,
// the precise fix is to add a small per-peer TX-pending tracking ring
// (BtDevice_t.TxPendingChar[]) populated by BtGattCharNotify on each
// outgoing notification/indication, then dequeue by ConnHdl here and
// call TxCompleteCB on the actual originating char. Cross-port work -
// scheduled as a separate round.
void BtGattSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent)
{
	if (NbPktSent == 0)
	{
		return;
	}

	for (BtGattSrvc_t *p = s_pBtGattSrvcList; p != nullptr; p = p->pNext)
	{
		for (int i = 0; i < p->NbChar; i++)
		{
			BtGattChar_t *c = &p->pCharArray[i];
			if (c->TxCompleteCB == nullptr)
			{
				continue;
			}
			if (c->Runtime.bNotify == false && c->Runtime.bIndic == false)
			{
				continue;
			}
			c->TxCompleteCB(c, i);
		}
	}
}



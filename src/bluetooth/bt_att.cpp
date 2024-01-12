/**-------------------------------------------------------------------------
@file	bt_att.cpp

@brief	Generic Bluetooth ATT protocol

Generic definitions for Bluetooth Attribute Protocol implementation

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2022

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
#include <inttypes.h>
#include <stddef.h>
#include <memory.h>

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
//#include "coredev/uart.h"

//extern UART g_Uart;

/// Default Attribute database mem size
/// User App can re-define a bigger or smaller value to fit use case
#ifndef BT_ATT_DB_MEMSIZE
#define BT_ATT_DB_MEMSIZE			2048	//!< Mem size in byte
#endif

static uint16_t s_AttMaxMtu = 515;

alignas(4) __attribute__((weak)) uint8_t s_BtAttDBMem[BT_ATT_DB_MEMSIZE];
static size_t s_BtAttDBMemSize = sizeof(s_BtAttDBMem);
static const uint32_t s_BtAttDBMemEnd = (uint32_t)s_BtAttDBMem + s_BtAttDBMemSize;
static BtAttDBEntry_t * const s_pBtAttDbEntryFirst = (BtAttDBEntry_t *)s_BtAttDBMem;
static BtAttDBEntry_t *s_pBtAttDbEntryEnd = (BtAttDBEntry_t*)s_BtAttDBMem;
static uint16_t s_LastHdl = 0;

uint16_t BtAttSetMaxMtu(uint16_t MaxMtu)
{
	if (MaxMtu >= 23)
	{
		s_AttMaxMtu = MaxMtu;
	}

	return s_AttMaxMtu;
}

void BtAttDBInit(size_t MemSize)
{
	s_BtAttDBMemSize = MemSize;
	memset(s_BtAttDBMem, 0, s_BtAttDBMemSize);
	s_pBtAttDbEntryEnd = (BtAttDBEntry_t*)(s_BtAttDBMem + s_BtAttDBMemSize);
}

BtAttDBEntry_t * const BtAttDBAddEntry(BtUuid16_t *pUuid, int MaxDataLen)//, void *pData, int DataLen)
{
	BtAttDBEntry_t *entry = s_pBtAttDbEntryEnd;

	uint32_t l = sizeof(BtAttDBEntry_t) + MaxDataLen;

	l = (l + 3) & 0xFFFFFFFC;
	if ((uint32_t)entry + l > s_BtAttDBMemEnd)
	{
		return nullptr;
	}

	entry->TypeUuid = *pUuid;
	entry->Hdl = ++s_LastHdl;
	entry->DataLen = MaxDataLen;

	s_pBtAttDbEntryEnd = (BtAttDBEntry_t*)((uint8_t*)entry + l);
	s_pBtAttDbEntryEnd->pNext = nullptr;
	s_pBtAttDbEntryEnd->pPrev = entry;
	entry->pNext = s_pBtAttDbEntryEnd;

	return entry;
}

BtAttDBEntry_t * const BtAttDBFindHandle(uint16_t Hdl)
{
	if (Hdl >= BT_ATT_DB_MEMSIZE)
	{
		return nullptr;
	}

	BtAttDBEntry_t *p = (BtAttDBEntry_t *)s_pBtAttDbEntryFirst;

	do {
		if (p->Hdl == Hdl)
		{
			return p;
		}

		p = p->pNext;
	} while (p != s_pBtAttDbEntryEnd);

	return nullptr;
}

BtAttDBEntry_t * const BtAttDBFindUuid(BtAttDBEntry_t *pStart, BtUuid16_t *pUuid)
{
	BtAttDBEntry_t *p = pStart;
	if (p == nullptr)
	{
		p = (BtAttDBEntry_t*)s_BtAttDBMem;
	}
	do {
		if (memcmp(&p->TypeUuid, pUuid, sizeof(BtUuid16_t)) == 0)
		{
			return p;
		}

		p = p->pNext;
	} while (p != s_pBtAttDbEntryEnd);

	return nullptr;
}

BtAttDBEntry_t * const BtAttDBFindUuidRange(BtUuid16_t *pUuid, uint16_t HdlStart, uint16_t HdlEnd)
{
	BtAttDBEntry_t *p = s_pBtAttDbEntryFirst;

	do {
		if (memcmp(&p->TypeUuid, pUuid, sizeof(BtUuid16_t)) == 0)
		{
			if (p->Hdl >= HdlStart && p->Hdl <= HdlEnd)
			{
				return p;
			}
		}

		p = p->pNext;
	} while (p != s_pBtAttDbEntryEnd);

	return nullptr;
}

BtAttDBEntry_t * const BtAttDBFindHdlRange(BtUuid16_t *pUuid, uint16_t *pHdlStart, uint16_t *pHdlEnd)
{
	BtAttDBEntry_t *first = BtAttDBFindUuidRange(pUuid, *pHdlStart, *pHdlEnd);

	if (first)
	{
		*pHdlStart = first->Hdl;
		BtAttDBEntry_t *p = BtAttDBFindUuid(first->pNext, pUuid);
		if (p)
		{
			*pHdlEnd = p->pPrev->Hdl;
		}
		else
		{
			*pHdlEnd = s_pBtAttDbEntryEnd->pPrev->Hdl;//0xFFFF;
		}
	}

	return first;
}


uint32_t BtAttError(BtAttReqRsp_t * const pRspAtt, uint16_t Hdl, uint8_t OpCode, uint8_t ErrCode)
{
	pRspAtt->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	pRspAtt->ErrorRsp.ReqOpCode = OpCode;
	pRspAtt->ErrorRsp.Hdl = Hdl;
	pRspAtt->ErrorRsp.Error = ErrCode;

	return sizeof(BtAttErrorRsp_t) + 1;
}




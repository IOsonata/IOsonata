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
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the app configured (UART, USB, RTT, BLE, or any other
// DeviceIntrf); the trace does not assume a transport. A release build
// defines NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

/// Default Attribute database mem size
/// User App can re-define a bigger or smaller value to fit use case
#ifndef BT_ATT_DB_MEMSIZE
#define BT_ATT_DB_MEMSIZE			2048	//!< Mem size in byte
#endif

static uint16_t s_AttMtu = BT_ATT_MTU_MIN;

// The pool must hold BtAttDBEntry_t objects at rounded offsets; align it to
// the entry type so that holds on any pointer width (a strong override of
// this array must be aligned at least as strictly).
alignas(BtAttDBEntry_t) __attribute__((weak)) uint8_t s_BtAttDBMem[BT_ATT_DB_MEMSIZE];
static size_t s_BtAttDBMemSize = sizeof(s_BtAttDBMem);
static size_t s_BtAttDBMemUsed = 0;
static BtAttDBEntry_t * const s_pBtAttDbEntryFirst = (BtAttDBEntry_t *)s_BtAttDBMem;
static BtAttDBEntry_t *s_pBtAttDbEntryEnd = (BtAttDBEntry_t*)s_BtAttDBMem;
static uint16_t s_LastHdl = 0;

// Set the negotiated ATT MTU from a peer's offered Rx MTU. The negotiated
// value is min(peer_offered, BT_ATT_MTU_MAX). Offers below BT_ATT_MTU_MIN
// are rejected (BT Core spec mandates 23 as the absolute floor).
//
// NOTE: s_AttMtu is the stack-wide default/fallback MTU. Per-connection
// response sizing resolves the link MTU via BtAttGetMtuForConn(), which falls
// back to this global when a peer has not (yet) negotiated one.
uint16_t BtAttSetMtu(uint16_t Mtu)
{
	if (Mtu < BT_ATT_MTU_MIN)
	{
		return s_AttMtu;
	}
	if (Mtu > BT_ATT_MTU_MAX)
	{
		Mtu = BT_ATT_MTU_MAX;
	}
	s_AttMtu = Mtu;
	return s_AttMtu;
}

uint16_t BtAttGetMtu()
{
	return s_AttMtu;
}

static uint16_t BtAttGetMtuForConn(uint16_t ConnHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);

	if (pConn != nullptr && pConn->Conn.MaxMtu >= BT_ATT_MTU_MIN)
	{
		return pConn->Conn.MaxMtu;
	}

	return s_AttMtu;
}
/*
void BtAttSetHandler(AttReadValFct_t ReadFct, AttWriteValFct_t WriteFct)
{
	s_AttReadValue = ReadFct;
	s_AttWriteValue = WriteFct;
}
*/
void BtAttDBInit(size_t MemSize)
{
	// The database lives in s_BtAttDBMem, a weak array of BT_ATT_DB_MEMSIZE
	// bytes. MemSize is the caller's claim about how much of it to use, and
	// this file cannot prove a claim larger than the array it was compiled
	// against, so clamp to that: a larger value would run the memset below
	// off the end of the pool. A bigger database comes from raising
	// BT_ATT_DB_MEMSIZE for the whole build, which resizes the array and this
	// sizeof together, not from defining a different sized array elsewhere,
	// which the sizeof here would not see.
	if (MemSize > sizeof(s_BtAttDBMem))
	{
		MemSize = sizeof(s_BtAttDBMem);
	}

	// The tail sentinel is written immediately below, so the pool has to hold
	// at least one entry. The clamp above already bounded MemSize by the
	// array, which is far larger than one entry.
	if (MemSize < sizeof(BtAttDBEntry_t))
	{
		MemSize = sizeof(BtAttDBEntry_t);
	}

	s_BtAttDBMemSize = MemSize;
	s_BtAttDBMemUsed = 0;
	memset(s_BtAttDBMem, 0, s_BtAttDBMemSize);

	s_pBtAttDbEntryEnd = (BtAttDBEntry_t*)s_BtAttDBMem;
	s_LastHdl = 0;
	s_pBtAttDbEntryEnd->pNext = nullptr;
	s_pBtAttDbEntryEnd->pPrev = nullptr;
}

BtAttDBEntry_t *BtAttDBAddEntry(BtUuid16_t *pUuid, int MaxDataLen)//, void *pData, int DataLen)
{
	if (pUuid == nullptr || MaxDataLen < 0 || MaxDataLen > 0xFFFF)
	{
		return nullptr;
	}

	size_t entrySize = sizeof(BtAttDBEntry_t) + (size_t)MaxDataLen;
	entrySize = (entrySize + alignof(BtAttDBEntry_t) - 1U) &
				~(size_t)(alignof(BtAttDBEntry_t) - 1U);

	if (s_BtAttDBMemUsed > s_BtAttDBMemSize)
	{
		return nullptr;
	}

	size_t remaining = s_BtAttDBMemSize - s_BtAttDBMemUsed;

	// We will write pNext/pPrev into the slot after this entry to seed the
	// next tail entry. That seed write touches the first few bytes of a
	// BtAttDBEntry_t, so the bound check must reserve a complete tail entry.
	// Without the pad, the seed writes spill past the pool when it is full.
	if (entrySize > remaining ||
		sizeof(BtAttDBEntry_t) > remaining - entrySize)
	{
		return nullptr;
	}

	BtAttDBEntry_t *entry =
		(BtAttDBEntry_t*)(s_BtAttDBMem + s_BtAttDBMemUsed);

	entry->TypeUuid = *pUuid;
	entry->Hdl = ++s_LastHdl;
	entry->DataLen = (uint16_t)MaxDataLen;

	s_BtAttDBMemUsed += entrySize;

	s_pBtAttDbEntryEnd =
		(BtAttDBEntry_t*)(s_BtAttDBMem + s_BtAttDBMemUsed);
	s_pBtAttDbEntryEnd->pNext = nullptr;
	s_pBtAttDbEntryEnd->pPrev = entry;
	entry->pNext = s_pBtAttDbEntryEnd;

	return entry;
}

void BtAttDBMark(BtAttDBMark_t *pMark)
{
	if (pMark == nullptr)
	{
		return;
	}

	pMark->MemUsed = s_BtAttDBMemUsed;
	pMark->LastHdl = s_LastHdl;
}

void BtAttDBUnwind(const BtAttDBMark_t *pMark)
{
	// Only a rewind is allowed. A mark from a database that has since been
	// reinitialised, or one recorded after the current position, would move
	// the allocator forward over memory no entry owns.
	if (pMark == nullptr || pMark->MemUsed > s_BtAttDBMemUsed ||
		pMark->LastHdl > s_LastHdl)
	{
		return;
	}

	s_BtAttDBMemUsed = pMark->MemUsed;
	s_LastHdl = pMark->LastHdl;

	// The slot at the mark becomes the tail sentinel again. Its pPrev still
	// points at the entry before it: BtAttDBAddEntry writes pPrev only when a
	// slot is seeded as the sentinel, never when it is turned into an entry,
	// so the link back is the one this position had before the dropped
	// entries were added.
	s_pBtAttDbEntryEnd = (BtAttDBEntry_t*)(s_BtAttDBMem + s_BtAttDBMemUsed);
	s_pBtAttDbEntryEnd->pNext = nullptr;
}

BtAttDBEntry_t *BtAttDBFindHandle(uint16_t Hdl)
{
	// Handle 0x0000 is reserved and never assigned to an attribute (Vol 3
	// Part F 3.2.2). Rejecting it here matters because the tail sentinel is
	// zero initialised, so a search for 0 would otherwise match it and hand
	// back the sentinel as if it were an attribute.
	if (Hdl == 0 || Hdl > s_LastHdl)
	{
		return nullptr;
	}

	// Walk to the tail sentinel, guarding against null: on an empty database
	// the first entry is the sentinel and its pNext is null, so a do/while
	// that only tests the sentinel dereferences null on the second pass.
	for (BtAttDBEntry_t *p = (BtAttDBEntry_t *)s_pBtAttDbEntryFirst;
		 p != nullptr && p != s_pBtAttDbEntryEnd; p = p->pNext)
	{
		if (p->Hdl == Hdl)
		{
			return p;
		}
	}

	return nullptr;
}

void BtAttDBEntrySetPermission(BtAttDBEntry_t *pEntry, uint32_t Permission)
{
	if (pEntry != nullptr)
	{
		pEntry->Permission = Permission;
	}
}

uint32_t BtAttDBEntryGetPermission(BtAttDBEntry_t *pEntry)
{
	return pEntry != nullptr ? pEntry->Permission : 0;
}

// Strong override of the weak default in bt_gatt.cpp. On the native host the
// ATT DB exists, so mirror the aggregate CCCD value into the descriptor entry
// for a local CCCD read.
void BtGattCccdDbSync(uint16_t CccdHdl, uint16_t CccVal)
{
	BtAttDBEntry_t *entry = BtAttDBFindHandle(CccdHdl);
	if (entry != nullptr)
	{
		BtDescClientCharConfig_t *pCccd = (BtDescClientCharConfig_t*)entry->Data;
		pCccd->CccVal = CccVal;
	}
}

BtAttDBEntry_t *BtAttDBFindUuid(BtAttDBEntry_t *pStart, BtUuid16_t *pUuid)
{
	BtAttDBEntry_t *p = pStart;
	if (p == nullptr)
	{
		p = (BtAttDBEntry_t*)s_BtAttDBMem;
	}
	// Stop at the tail sentinel and also guard against null: pStart can be the
	// sentinel itself (a caller passing lastEntry->pNext), and the sentinel's
	// pNext is null, so a do/while that only tests the sentinel walks off the
	// end.
	while (p != nullptr && p != s_pBtAttDbEntryEnd)
	{
		if (memcmp(&p->TypeUuid, pUuid, sizeof(BtUuid16_t)) == 0)
		{
			return p;
		}

		p = p->pNext;
	}

	return nullptr;
}

BtAttDBEntry_t *BtAttDBFindUuidRange(BtUuid16_t *pUuid, uint16_t HdlStart, uint16_t HdlEnd)
{
	// Same guard as BtAttDBFindUuid: stop at the tail sentinel and at null.
	// On an empty database the first entry is the sentinel and its pNext is
	// null, so a do/while that only tests the sentinel walks off the end.
	for (BtAttDBEntry_t *p = s_pBtAttDbEntryFirst;
		 p != nullptr && p != s_pBtAttDbEntryEnd; p = p->pNext)
	{
		if (memcmp(&p->TypeUuid, pUuid, sizeof(BtUuid16_t)) == 0)
		{
			if (p->Hdl >= HdlStart && p->Hdl <= HdlEnd)
			{
				return p;
			}
		}
	}

	return nullptr;
}

BtAttDBEntry_t *BtAttDBFindHdlRange(BtUuid16_t *pUuid, uint16_t *pHdlStart, uint16_t *pHdlEnd)
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

size_t BtAttReadValue(BtAttDBEntry_t *pEntry, uint16_t Offset, uint8_t *pBuff, uint16_t Len)
{
	if (pBuff == nullptr)
	{
		return 0;
	}

	uint16_t len = 0;

	if (pEntry->TypeUuid.BaseIdx == 0)
	{
		//DEBUG_PRINTF("BtAttReadValue : %x, %d\r\n", pEntry->TypeUuid.Uuid, Len);

		switch (pEntry->TypeUuid.Uuid)
		{
		case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
		case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
		{
			DEBUG_PRINTF("BT_UUID_DECLARATIONS_PRIMARY_SERVICE (0x2800)/ SECONDARY_SERVICE (0x2801)\r\n");
			BtAttSrvcDeclar_t *p = (BtAttSrvcDeclar_t*) pEntry->Data;

			// Honour the caller's buffer/MTU cap (Len) before writing the fixed
			// declaration value, so this helper cannot overrun the response
			// buffer if it is ever handed less room than the value size.
			if (p->Uuid.BaseIdx > 0)
			{
				if (Len < 16)
				{
					break;
				}
				BtUuidGetBase(p->Uuid.BaseIdx, pBuff);

				pBuff[12] = p->Uuid.Uuid16 & 0xFF;
				pBuff[13] = p->Uuid.Uuid16 >> 8;

				len = 16;
			}
			else
			{
				if (Len < 2)
				{
					break;
				}
				pBuff[0] = p->Uuid.Uuid16 & 0xFF;
				pBuff[1] = p->Uuid.Uuid16 >> 8;

				len = 2;
			}
		}
			break;
		case BT_UUID_DECLARATIONS_INCLUDE:
		{
			DEBUG_PRINTF("BT_UUID_DECLARATIONS_INCLUDE (0x2802)\r\n");
			BtAttSrvcInclude_t *p = (BtAttSrvcInclude_t*) pEntry->Data;

			// Include Definition value (Core Vol 3 Part G 3.2): Included Service
			// Attribute Handle, End Group Handle, then the Service UUID only when
			// it is a 16-bit Bluetooth UUID. For a 128-bit included service UUID
			// the UUID field is omitted and the client reads the included
			// service's own declaration. The value MUST be written here: the
			// previous code set the length but left pBuff untouched, returning
			// stale response-buffer contents to the peer.
			uint16_t need = p->SrvcUuid.BaseIdx > 0 ? 4 : 6;
			if (Len < need)
			{
				break;
			}
			pBuff[0] = p->SrvcHdl & 0xFF;
			pBuff[1] = (p->SrvcHdl >> 8) & 0xFF;
			pBuff[2] = p->EndGrpHdl & 0xFF;
			pBuff[3] = (p->EndGrpHdl >> 8) & 0xFF;
			if (p->SrvcUuid.BaseIdx == 0)
			{
				pBuff[4] = p->SrvcUuid.Uuid16 & 0xFF;
				pBuff[5] = p->SrvcUuid.Uuid16 >> 8;
			}
			len = need;
		}

			break;
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		{
			DEBUG_PRINTF("BT_UUID_DECLARATIONS_CHARACTERISTIC (0x2803)\r\n");
			BtAttCharDeclar_t *p = (BtAttCharDeclar_t*) pEntry->Data;
			//len = pEntry->DataLen;
			//memcpy(pBuff, pEntry->Data, len);

			// Honour the caller's buffer/MTU cap (Len): a 128-bit characteristic
			// declaration is 19 octets, a 16-bit one 5. Bail before writing if
			// the buffer cannot hold the value, so this cannot overrun.
			if (Len < (p->Uuid.BaseIdx > 0 ? 19 : 5))
			{
				break;
			}
			pBuff[0] = p->pChar->Property;
			pBuff[1] = p->pChar->ValHdl & 0xFF;
			pBuff[2] = (p->pChar->ValHdl >> 8) & 0xFF;
			BtUuidVal_t *u = (BtUuidVal_t*) &pBuff[3];
			if (p->Uuid.BaseIdx > 0)
			{
				BtUuidGetBase(p->Uuid.BaseIdx, u->Uuid128);

				u->Uuid128[12] = p->Uuid.Uuid16 & 0xFF;
				u->Uuid128[13] = p->Uuid.Uuid16 >> 8;
				len = 19;
				DEBUG_PRINTF("UUID16 0x%X ,", p->Uuid.Uuid16);
				DEBUG_PRINTF("Base UUID128 (hex) = ");
				for (int i = 0; i < 16; i++)
					DEBUG_PRINTF("%X ", u->Uuid128[i]);
				DEBUG_PRINTF("\r\n");
			}
			else
			{
				u->Uuid16 = p->Uuid.Uuid16;
				len = 5;
				DEBUG_PRINTF("UUID16 = 0x%X \r\n", u->Uuid16);
			}
		}
			break;
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
			DEBUG_PRINTF("BT_UUID_DECLARATIONS_CHARACTERISTIC (0x2900)\r\n");
			break;
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
//				if (pEntry->pVal)
		{
			DEBUG_PRINTF("BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION (0x2901)\r\n");
			BtDescCharUserDesc_t *p = (BtDescCharUserDesc_t*) pEntry->Data;
			const char *desc = (p != nullptr && p->pChar != nullptr) ? p->pChar->pDesc : nullptr;

			if (desc != nullptr && Len > 0)
			{
				size_t dlen = strlen(desc);

				if (Offset < dlen)
				{
					size_t l = dlen - Offset;
					if (l > Len)
					{
						l = Len;
					}
					memcpy(pBuff, desc + Offset, l);
					len = l;
				}
			}
		}
			break;
		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
		{
			DEBUG_PRINTF("BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION (0x2902)\r\n");
			BtDescClientCharConfig_t *d =
					(BtDescClientCharConfig_t*) pEntry->Data;

			*(uint16_t*) pBuff = d->CccVal;
			DEBUG_PRINTF("Hdl = %d, CCC val = %d\r\n", pEntry->Hdl, d->CccVal);
			len = 2;
		}
			break;
		case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
			break;
		default:
		{
			BtAttCharValue_t *p = (BtAttCharValue_t*) pEntry->Data;
			DEBUG_PRINTF("UUID unknown : Type 0x%x, Len %d\r\n", pEntry->TypeUuid.Uuid, p->pChar->ValueLen);

			// Clamp to the bytes available from Offset and to the caller's
			// buffer/MTU cap (Len). Offset is attacker-controlled on a blob
			// read; without the bound an oversized Offset/ValueLen reads past
			// the value and overflows the response buffer.
			uint16_t vlen = p->pChar->ValueLen;
			if (Offset <= vlen)
			{
				size_t l = min((uint16_t)(vlen - Offset), Len);
				memcpy(pBuff, (uint8_t*)p->pChar->pValue + Offset, l);
				len = l;
			}
		}

		}
	}
	else
	{
		BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;
		DEBUG_PRINTF("Read Req UUID custom: %d: %x, %d\r\n", pEntry->TypeUuid.BaseIdx, pEntry->TypeUuid.Uuid, p->pChar->ValueLen);

		// Clamp to bytes available from Offset and to the caller's buffer/MTU
		// cap (Len) to avoid an out-of-bounds read / response overflow.
		uint16_t vlen = p->pChar->ValueLen;
		if (Offset <= vlen)
		{
			size_t l = min((uint16_t)(vlen - Offset), Len);
			memcpy(pBuff, (uint8_t*)p->pChar->pValue + Offset, l);
			len = l;
		}
	}

	return len;
}


static size_t BtAttReadValueForConn(uint16_t ConnHdl, BtAttDBEntry_t *pEntry,
									uint16_t Offset, uint8_t *pBuff, uint16_t Len)
{
	if (pEntry == nullptr || pBuff == nullptr)
	{
		return 0;
	}

	if (pEntry->TypeUuid.BaseIdx == 0 &&
		pEntry->TypeUuid.Uuid == BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION)
	{
		if (Offset >= 2 || Len == 0)
		{
			return 0;
		}

		uint16_t cccd = BtGattCccdGet(ConnHdl, pEntry->Hdl);
		uint8_t tmp[2] = { (uint8_t)(cccd & 0xFF), (uint8_t)(cccd >> 8) };
		uint16_t n = min((uint16_t)(2 - Offset), Len);

		memcpy(pBuff, &tmp[Offset], n);
		return n;
	}

	return BtAttReadValue(pEntry, Offset, pBuff, Len);
}

static bool BtAttEntryIsCharValue(BtAttDBEntry_t *pEntry)
{
	if (pEntry == nullptr)
	{
		return false;
	}

	if (pEntry->TypeUuid.BaseIdx != 0)
	{
		return true;
	}

	switch (pEntry->TypeUuid.Uuid)
	{
		case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
		case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
		case BT_UUID_DECLARATIONS_INCLUDE:
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
		case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
			return false;
		default:
			return true;
	}
}

static bool BtAttEntryIsCccd(BtAttDBEntry_t *pEntry)
{
	return pEntry != nullptr && pEntry->TypeUuid.BaseIdx == 0 &&
		   pEntry->TypeUuid.Uuid == BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION;
}

static BtGattChar_t *BtAttEntryChar(BtAttDBEntry_t *pEntry)
{
	if (pEntry == nullptr)
	{
		return nullptr;
	}

	if (pEntry->TypeUuid.BaseIdx == 0)
	{
		switch (pEntry->TypeUuid.Uuid)
		{
			case BT_UUID_DECLARATIONS_CHARACTERISTIC:
				return ((BtAttCharDeclar_t*)pEntry->Data)->pChar;

			case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				return ((BtDescClientCharConfig_t*)pEntry->Data)->pChar;

			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				return ((BtDescCharUserDesc_t*)pEntry->Data)->pChar;

			case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
			case BT_UUID_DECLARATIONS_INCLUDE:
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
			case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				return nullptr;

			default:
				break;
		}
	}

	if (BtAttEntryIsCharValue(pEntry))
	{
		return ((BtAttCharValue_t*)pEntry->Data)->pChar;
	}

	return nullptr;
}

// Link security (encryption / authentication / key size) is read from the
// generic BtGapConnSecGet snapshot, not from SMP internals. Authorization stays
// a per-attribute hook here because it is app policy, not link state: ports or
// apps override it. The default authorizes everything; gating happens only when
// an attribute sets the AUTHOR Permission bit.
__attribute__((weak)) bool BtAttLinkAuthorized(uint16_t ConnHdl,
											  BtAttDBEntry_t *pEntry,
											  bool bRead)
{
	(void)ConnHdl;
	(void)pEntry;
	(void)bRead;
	return true;
}

// Compatibility hook for ports/apps that already enforce security externally.
// Return 0 to allow access, or an ATT error code such as INSUF_ENCRYPT.
__attribute__((weak)) uint8_t BtAttAccessSecurityError(uint16_t ConnHdl,
													  BtAttDBEntry_t *pEntry,
													  bool bRead)
{
	(void)ConnHdl;
	(void)pEntry;
	(void)bRead;
	return 0;
}

__attribute__((weak)) bool BtAttSignedWriteVerify(uint16_t ConnHdl,
												 const BtAttSignedWriteCmd_t *pCmd,
												 uint16_t ValueLen,
												 const uint8_t *pSignature)
{
	// Rebuild the signed message in wire order: opcode || handle || value ||
	// SignCounter. The SignCounter is the first 4 bytes of the 12-byte signature
	// and is part of the data covered by the MAC (Core spec Vol 3 Part H 2.4.5).
	uint8_t m[BT_ATT_MTU_MAX];
	size_t n = 0;

	if ((size_t)ValueLen + 7 > sizeof(m))
	{
		return false;
	}

	m[n++] = BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD;
	m[n++] = (uint8_t)(pCmd->Hdl & 0xFF);
	m[n++] = (uint8_t)(pCmd->Hdl >> 8);
	memcpy(&m[n], pCmd->Data, ValueLen);
	n += ValueLen;
	memcpy(&m[n], pSignature, 4);
	n += 4;

	return BtSmpSignVerify(ConnHdl, m, n, pSignature);
}

static uint8_t BtAttAccessPolicyError(uint16_t ConnHdl,
									  BtAttDBEntry_t *pEntry,
									  bool bRead)
{
	if (pEntry == nullptr)
	{
		return BT_ATT_ERROR_INVALID_HANDLE;
	}

	uint32_t perm = pEntry->Permission;

	uint32_t encFlag = bRead ? BT_ATT_PERMISSION_READ_ENCRYPT :
							   BT_ATT_PERMISSION_WRITE_ENCRYPT;
	uint32_t authFlag = bRead ? BT_ATT_PERMISSION_READ_AUTHEN :
								BT_ATT_PERMISSION_WRITE_AUTHEN;
	uint32_t authorFlag = bRead ? BT_ATT_PERMISSION_READ_AUTHOR :
								  BT_ATT_PERMISSION_WRITE_AUTHOR;
	uint32_t keyFlag = bRead ? BT_ATT_PERMISSION_READ_KEY_SIZE :
							   BT_ATT_PERMISSION_WRITE_KEY_SIZE;

	BtConnSec_t sec;
	if (BtGapConnSecGet(ConnHdl, &sec) == false)
	{
		sec.Level = BT_GAP_SEC_LEVEL_NONE;
		sec.KeySize = 0;
		sec.Flags = 0;
	}

	if ((perm & encFlag) != 0 && sec.Level < BT_GAP_SEC_LEVEL_ENC_UNAUTH)
	{
		// Encrypted link required but absent. A bonded peer can re-encrypt from
		// the stored key (Insufficient Encryption); an unbonded peer has to pair
		// first (Insufficient Authentication). Core spec Vol 3 Part C 10.3.
		return (sec.Flags & BT_GAP_SEC_FLAG_BONDED) ? BT_ATT_ERROR_INSUF_ENCRYPT
													: BT_ATT_ERROR_INSUF_AUTHEN;
	}

	if ((perm & authFlag) != 0 && sec.Level < BT_GAP_SEC_LEVEL_ENC_AUTH)
	{
		return BT_ATT_ERROR_INSUF_AUTHEN;
	}

	if ((perm & authorFlag) != 0 &&
		BtAttLinkAuthorized(ConnHdl, pEntry, bRead) == false)
	{
		return BT_ATT_ERROR_INSUF_AUTHOR;
	}

	if ((perm & keyFlag) != 0 && sec.KeySize < 16)
	{
		return BT_ATT_ERROR_ENCRYPT_KEY_TOO_SHORT;
	}

	return 0;
}

static uint8_t BtAttReadPermError(uint16_t ConnHdl, BtAttDBEntry_t *pEntry)
{
	if (pEntry == nullptr)
	{
		return BT_ATT_ERROR_INVALID_HANDLE;
	}

	if (BtAttEntryIsCharValue(pEntry))
	{
		BtGattChar_t *pChar = BtAttEntryChar(pEntry);
		if (pChar == nullptr || (pChar->Property & BT_GATT_CHAR_PROP_READ) == 0)
		{
			return BT_ATT_ERROR_READ_NOT_PERMITTED;
		}
	}

	uint8_t err = BtAttAccessPolicyError(ConnHdl, pEntry, true);
	if (err != 0)
	{
		return err;
	}

	return BtAttAccessSecurityError(ConnHdl, pEntry, true);
}

static uint8_t BtAttWritePermError(uint16_t ConnHdl, BtAttDBEntry_t *pEntry,
								   uint8_t OpCode, const uint8_t *pData,
								   uint16_t Len)
{
	if (pEntry == nullptr)
	{
		return BT_ATT_ERROR_INVALID_HANDLE;
	}

	if (BtAttEntryIsCharValue(pEntry))
	{
		BtGattChar_t *pChar = BtAttEntryChar(pEntry);
		if (pChar == nullptr)
		{
			return BT_ATT_ERROR_WRITE_NOT_PERMITTED;
		}

		uint32_t required = BT_GATT_CHAR_PROP_WRITE;
		if (OpCode == BT_ATT_OPCODE_ATT_CMD)
		{
			required = BT_GATT_CHAR_PROP_WRITE_WORESP;
		}
		else if (OpCode == BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD)
		{
			required = BT_GATT_CHAR_PROP_AUTH_SIGNED;
		}

		if ((pChar->Property & required) == 0)
		{
			return BT_ATT_ERROR_WRITE_NOT_PERMITTED;
		}

		uint8_t err = BtAttAccessPolicyError(ConnHdl, pEntry, false);
		if (err != 0)
		{
			return err;
		}

		return BtAttAccessSecurityError(ConnHdl, pEntry, false);
	}

	if (BtAttEntryIsCccd(pEntry))
	{
		BtGattChar_t *pChar = BtAttEntryChar(pEntry);
		if (pChar == nullptr)
		{
			return BT_ATT_ERROR_WRITE_NOT_PERMITTED;
		}
		if (Len >= 2 && pData != nullptr)
		{
			// One value check for the whole stack, in bt_gatt.cpp, so the ATT
			// server and direct GATT callers agree on what a characteristic
			// accepts. A rejected value is Client Characteristic Configuration
			// Descriptor Improperly Configured, the code GATT clients expect
			// here (Core Specification Supplement Part B 1.2).
			uint8_t cerr = BtGattCccdValueError(pChar,
									(uint16_t)(pData[0] | (pData[1] << 8)));
			if (cerr != 0)
			{
				return cerr;
			}
		}
		uint8_t err = BtAttAccessPolicyError(ConnHdl, pEntry, false);
		if (err != 0)
		{
			return err;
		}

		return BtAttAccessSecurityError(ConnHdl, pEntry, false);
	}

	return BT_ATT_ERROR_WRITE_NOT_PERMITTED;
}

//size_t BtGattWriteValue(uint16_t Hdl, uint8_t *pBuff, size_t Len)
size_t BtAttWriteValue(BtAttDBEntry_t *pEntry, uint16_t Offset, uint8_t *pData, uint16_t Len)
{
	size_t len = 0;
	//BtAttDBEntry_t *entry = BtAttDBFindHandle(Hdl);//&s_BtGattEntryTbl[Hdl - 1];
//DEBUG_PRINTF("BtAttWriteValue : uuid %x, %x\r\n", pEntry->TypeUuid.Uuid, pEntry->Hdl);

	if (pEntry->TypeUuid.BaseIdx == 0)
	{
		switch (pEntry->TypeUuid.Uuid)
		{
			case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
				break;
			case BT_UUID_DECLARATIONS_INCLUDE:
				break;
			case BT_UUID_DECLARATIONS_CHARACTERISTIC:
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				break;
			case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				{
					//DEBUG_PRINTF("BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION\r\n");

					BtDescClientCharConfig_t *p = (BtDescClientCharConfig_t*)pEntry->Data;

					// The CCCD value is a 2-byte field. A write shorter than 2
					// bytes would read past the received PDU; reject it instead
					// of dereferencing out of bounds.
					if (Len < 2)
					{
						break;
					}

					p->CccVal = *(uint16_t*)pData;
					p->pChar->bNotify = p->CccVal & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION ? true: false;
					if (p->pChar->SetNotifCB)
					{
						p->pChar->SetNotifCB(p->pChar, p->pChar->bNotify, BT_CONN_HDL_INVALID);
					}
					p->pChar->bIndic = p->CccVal & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION ? true: false;
					if (p->pChar->SetIndCB)
					{
						p->pChar->SetIndCB(p->pChar, p->pChar->bIndic, BT_CONN_HDL_INVALID);
					}
					len = 2;
//					DEBUG_PRINTF("CccdHdl : %x\r\n", p->pChar->CccdHdl);
				}
				break;
			case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				{
					BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;

					if (p != nullptr && p->pChar != nullptr &&
						(pData != nullptr || Len == 0) &&
						Offset <= p->pChar->MaxDataLen)
					{
						size_t l = Len;
						size_t avail = (size_t)p->pChar->MaxDataLen - Offset;

						if (l > avail)
						{
							l = avail;
						}

						if (l > 0 && p->pChar->pValue != nullptr)
						{
							memcpy((uint8_t*)p->pChar->pValue + Offset, pData, l);
						}

						if ((uint32_t)Offset + l > p->pChar->ValueLen)
						{
							p->pChar->ValueLen = (uint16_t)(Offset + l);
						}
						len = l;

						if (p->pChar->WrCB)
						{
							p->pChar->WrCB(p->pChar, pData, (int)Offset, (int)l);
						}
					}
				}

		}
	}
	else
	{
		BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;

		if (p != nullptr && p->pChar != nullptr &&
			(pData != nullptr || Len == 0) &&
			Offset <= p->pChar->MaxDataLen)
		{
			size_t l = Len;
			size_t avail = (size_t)p->pChar->MaxDataLen - Offset;

			if (l > avail)
			{
				l = avail;
			}

			if (l > 0 && p->pChar->pValue != nullptr)
			{
				memcpy((uint8_t*)p->pChar->pValue + Offset, pData, l);
			}

			if ((uint32_t)Offset + l > p->pChar->ValueLen)
			{
				p->pChar->ValueLen = (uint16_t)(Offset + l);
			}
			len = l;

			if (p->pChar->WrCB)
			{
				p->pChar->WrCB(p->pChar, pData, (int)Offset, (int)l);
			}
		}
	}

	return len;
}


static size_t BtAttWriteValueForConn(uint16_t ConnHdl, BtAttDBEntry_t *pEntry,
									 uint16_t Offset, uint8_t *pData, uint16_t Len)
{
	if (pEntry == nullptr)
	{
		return 0;
	}

	if (BtAttEntryIsCccd(pEntry))
	{
		if (Offset != 0 || Len != 2 || pData == nullptr)
		{
			return 0;
		}

		uint16_t cccd = (uint16_t)(pData[0] | (pData[1] << 8));
		return BtGattCccdSet(ConnHdl, pEntry->Hdl, cccd) ? 2 : 0;
	}

	return BtAttWriteValue(pEntry, Offset, pData, Len);
}

uint32_t BtAttError(BtAttReqRsp_t * const pRspAtt, uint16_t Hdl, uint8_t OpCode, uint8_t ErrCode)
{
	pRspAtt->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	pRspAtt->ErrorRsp.ReqOpCode = OpCode;
	pRspAtt->ErrorRsp.Hdl = Hdl;
	pRspAtt->ErrorRsp.Error = ErrCode;

	return sizeof(BtAttErrorRsp_t) + 1;
}

// Execute a queued long write for one connection: walk the prepare queue,
// compact consecutive chunks that target the same handle into one contiguous
// value, then apply it through BtAttWriteValue (which fires the char WrCB once
// with the full value, same as a normal write). Queue records are laid out as
// { uint16 Hdl, uint16 Offset, uint16 Len, uint8 Data[Len] } in arrival order;
// clients send chunks in ascending offset, so concatenation rebuilds the value.
// Current value length of an attribute, for offset/length validation. Char
// values report their live ValueLen; the CCCD is 2 bytes. Other attribute types
// are not offset-validated here (returns 0xFFFF so the caller skips the check).
static uint16_t BtAttCurValueLen(BtAttDBEntry_t *pEntry)
{
	if (pEntry == nullptr)
	{
		return 0;
	}
	if (BtAttEntryIsCharValue(pEntry))
	{
		BtGattChar_t *pChar = BtAttEntryChar(pEntry);
		return pChar != nullptr ? (uint16_t)pChar->ValueLen : 0;
	}
	if (pEntry->TypeUuid.BaseIdx == 0 &&
		pEntry->TypeUuid.Uuid == BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION)
	{
		return 2;
	}
	return 0xFFFF;
}

// Complete on-air value length of an attribute, without copying it. Read By
// Type uses this to size and compare handle-value pairs so it does not need a
// full-value scratch buffer. The lengths MUST match what BtAttReadValue emits
// for each type (declarations are fixed-format; char values and the user
// description come from the characteristic).
static uint16_t BtAttFullValueLen(BtAttDBEntry_t *pEntry)
{
	if (pEntry == nullptr)
	{
		return 0;
	}

	if (pEntry->TypeUuid.BaseIdx != 0)
	{
		BtGattChar_t *pChar = BtAttEntryChar(pEntry);
		return pChar != nullptr ? (uint16_t)pChar->ValueLen : 0;
	}

	switch (pEntry->TypeUuid.Uuid)
	{
		case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
		case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
			return ((BtAttSrvcDeclar_t*)pEntry->Data)->Uuid.BaseIdx > 0 ? 16 : 2;

		case BT_UUID_DECLARATIONS_INCLUDE:
			return ((BtAttSrvcInclude_t*)pEntry->Data)->SrvcUuid.BaseIdx > 0 ? 4 : 6;

		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
			return ((BtAttCharDeclar_t*)pEntry->Data)->Uuid.BaseIdx > 0 ? 19 : 5;

		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
		{
			BtGattChar_t *pChar = BtAttEntryChar(pEntry);
			const char *desc = pChar != nullptr ? pChar->pDesc : nullptr;
			return desc != nullptr ? (uint16_t)strlen(desc) : 0;
		}

		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
			return 2;

		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
		case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
			return 0;

		default:
		{
			BtGattChar_t *pChar = BtAttEntryChar(pEntry);
			return pChar != nullptr ? (uint16_t)pChar->ValueLen : 0;
		}
	}
}

// True only when the attribute's complete value equals the Len expected bytes.
// Used by Find By Type Value so it does not need a full-MTU scratch copy: the
// length is checked first (a mismatch is not a match and needs no read), then
// the value is compared in small chunks. Declaration values are at most 19
// octets and are read in a single offset-0 chunk, which matters because
// BtAttReadValue ignores the offset for declarations; characteristic values
// support offset reads, so longer ones compare correctly across chunks.
static bool BtAttValueEquals(uint16_t ConnHdl, BtAttDBEntry_t *pEntry,
							 const uint8_t *pExpected, uint16_t Len)
{
	if (BtAttFullValueLen(pEntry) != Len)
	{
		return false;
	}

	uint8_t chunk[64];		// >= the largest declaration value (19 octets)
	uint16_t off = 0;

	while (off < Len)
	{
		uint16_t want = (uint16_t)(Len - off);
		if (want > sizeof(chunk))
		{
			want = sizeof(chunk);
		}

		size_t got = BtAttReadValueForConn(ConnHdl, pEntry, off, chunk, want);
		if (got != want || memcmp(chunk, pExpected + off, want) != 0)
		{
			return false;
		}
		off = (uint16_t)(off + want);
	}

	return true;
}

// Measure one run of prepared-write chunks starting at Pos: the first chunk
// plus every following chunk for the same handle whose offset continues it
// exactly. Fills the run description and returns an ATT error code, 0 on
// success. bCompact moves the following chunks' data up against the first
// chunk so the run forms one contiguous value; pass false to measure the run
// without touching the buffer.
static uint8_t BtAttLongWrRun(uint8_t *pBuf, uint16_t Total, uint16_t Pos, bool bCompact,
							  uint16_t *pHdl, uint16_t *pOff, uint16_t *pLen, uint16_t *pNext)
{
	uint16_t hdl, off, len;
	memcpy(&hdl, pBuf + Pos, 2);
	memcpy(&off, pBuf + Pos + 2, 2);
	memcpy(&len, pBuf + Pos + 4, 2);

	uint8_t  *dst    = pBuf + Pos + 6 + len;
	uint16_t  totLen = len;
	uint16_t  next   = Pos + 6 + len;
	uint8_t   err    = 0;

	while (next + 6 <= Total)
	{
		uint16_t nhdl, noff, nlen;
		memcpy(&nhdl, pBuf + next, 2);
		memcpy(&noff, pBuf + next + 2, 2);
		memcpy(&nlen, pBuf + next + 4, 2);

		if (nhdl != hdl)
		{
			break;
		}

		// A gap or overlap ends the current contiguous run; it is not an
		// error. Queued prepared writes are applied in order and need not
		// be contiguous (Vol 3 Part F 3.4.6), so a Reliable Write may patch
		// disjoint regions of one attribute. The discontiguous chunk starts
		// its own run.
		if (noff != (uint16_t)(off + totLen))
		{
			break;
		}

		if ((uint32_t)next + 6UL + nlen > Total)
		{
			err = BT_ATT_ERROR_INVALID_PDU;
			break;
		}

		if (bCompact)
		{
			memmove(dst, pBuf + next + 6, nlen);
		}
		dst    += nlen;
		totLen += nlen;
		next   += 6 + nlen;
	}

	*pHdl  = hdl;
	*pOff  = off;
	*pLen  = totLen;
	*pNext = next;

	return err;
}

// Value length the attribute will have by the time the run at StopPos is
// applied: its live length extended by every earlier run for the same handle.
// A write only ever extends a value, so validating an offset against this
// projection accepts a queue that extends an attribute and then patches inside
// the extended region, which applying the queue in order does produce.
static uint16_t BtAttLongWrProjLen(uint8_t *pBuf, uint16_t Total, uint16_t StopPos,
								   uint16_t Hdl, uint16_t CurLen)
{
	uint16_t projected = CurLen;
	uint16_t pos = 0;

	while (pos + 6 <= Total && pos < StopPos)
	{
		uint16_t hdl, off, len, next;

		// Measure only; the commit pass does the compaction.
		if (BtAttLongWrRun(pBuf, Total, pos, false, &hdl, &off, &len, &next) != 0)
		{
			break;
		}

		if (hdl == Hdl && (uint32_t)off + len > projected)
		{
			projected = (uint16_t)(off + len);
		}

		pos = next;
	}

	return projected;
}

// Apply the queued prepared writes. Returns 0 on success, or an ATT error code
// with *pFailHdl set to the offending handle (Vol 3 Part F 3.4.6.3). The queue
// is consumed either way.
//
// Two passes, because Execute Write is atomic (Vol 3 Part F 3.4.6): the whole
// queue is validated before any attribute is touched, so a bad record late in
// the queue cannot leave earlier records already applied.
static uint8_t BtAttExecLongWrite(BtDevice_t *pConn, uint16_t *pFailHdl)
{
	if (pConn == nullptr || pConn->Conn.pLongWrBuff == nullptr)
	{
		return 0;
	}

	uint8_t  *buf   = pConn->Conn.pLongWrBuff;
	uint16_t  total = pConn->Conn.LongWrLen;
	uint16_t  pos   = 0;
	uint8_t   err   = 0;

	// Pass 1: validate every queued write. Nothing is modified here, not the
	// buffer and not any attribute.
	while (pos + 6 <= total)
	{
		uint16_t hdl, off, totLen, next;

		err = BtAttLongWrRun(buf, total, pos, false, &hdl, &off, &totLen, &next);
		if (err != 0)
		{
			*pFailHdl = hdl;
			break;
		}

		BtAttDBEntry_t *entry = BtAttDBFindHandle(hdl);
		if (entry == nullptr)
		{
			*pFailHdl = hdl;
			err = BT_ATT_ERROR_INVALID_HANDLE;
			break;
		}

		// Offset past the value this write will land on is INVALID_OFFSET; a
		// reassembled value beyond the characteristic maximum is
		// INVALID_ATTRIBUTE_VALUE_LENGTH.
		if (off > BtAttLongWrProjLen(buf, total, pos, hdl, BtAttCurValueLen(entry)))
		{
			*pFailHdl = hdl;
			err = BT_ATT_ERROR_INVALID_OFFSET;
			break;
		}

		// MaxDataLen bounds the characteristic value only. BtAttEntryChar also
		// resolves the owner of a CCCD or user description entry, so ask it for
		// a value attribute; otherwise a descriptor write would be measured
		// against the value maximum of the characteristic it describes.
		BtGattChar_t *pChar = BtAttEntryIsCharValue(entry) ? BtAttEntryChar(entry) : nullptr;
		if (pChar != nullptr && (uint32_t)off + totLen > pChar->MaxDataLen)
		{
			*pFailHdl = hdl;
			err = BT_ATT_ERROR_INVALID_ATT_VALUE;
			break;
		}

		// A CCCD reassembled from prepared writes has to satisfy the same rules
		// as a direct Write Request: exactly two octets at offset 0, holding a
		// value the characteristic supports. The run is measured here, not
		// compacted, so the two octets are only laid out contiguously when the
		// run is a single queued record; a CCCD split across records is
		// rejected rather than reassembled for this check.
		if (BtAttEntryIsCccd(entry))
		{
			uint16_t reclen;
			memcpy(&reclen, buf + pos + 4, 2);

			if (off != 0 || totLen != 2 || reclen != 2 ||
				(uint32_t)pos + 8UL > total)
			{
				*pFailHdl = hdl;
				err = BT_ATT_ERROR_INVALID_ATT_VALUE;
				break;
			}

			const uint8_t *val = buf + pos + 6;
			uint8_t cerr = BtGattCccdValueError(BtAttEntryChar(entry),
									(uint16_t)(val[0] | (val[1] << 8)));

			if (cerr != 0)
			{
				*pFailHdl = hdl;
				err = cerr;
				break;
			}
		}

		pos = next;
	}

	if (err != 0)
	{
		pConn->Conn.LongWrLen = 0;
		return err;
	}

	// Pass 2: commit. Every record validated above, so no attribute write in
	// this loop can fail on a check that pass 1 already made.
	pos = 0;
	while (pos + 6 <= total)
	{
		uint16_t hdl, off, totLen, next;

		BtAttLongWrRun(buf, total, pos, true, &hdl, &off, &totLen, &next);

		BtAttDBEntry_t *entry = BtAttDBFindHandle(hdl);
		if (entry != nullptr)
		{
			BtAttWriteValueForConn(pConn->Conn.Hdl, entry, off, buf + pos + 6, totLen);
		}

		pos = next;
	}

	pConn->Conn.LongWrLen = 0;
	return 0;
}

// Resolve the attribute type of a Read By Type / Read By Group Type request
// into the packed DB UUID form. The type is 2 or 16 octets and both are
// mandatory (Vol 3 Part F 3.4.4.1): TypeLen 2 gives a 16-bit type; TypeLen 16
// gives a 128-bit type resolved to a registered base (bytes 12-13 are the
// short id, the rest must match a known base). Returns:
//   1  = resolved into *pOut
//   0  = a valid 128-bit type whose base is not registered - nothing can match
//  -1  = malformed type length
static int BtAttResolveReqType(const BtUuidVal_t *pReqUuid, int TypeLen,
							   BtUuid16_t *pOut)
{
	if (TypeLen == 2)
	{
		pOut->BaseIdx = 0;
		pOut->Type = BT_UUID_TYPE_16;
		pOut->Uuid = pReqUuid->Uuid16;
		return 1;
	}

	if (TypeLen == 16)
	{
		uint8_t base[16];
		memcpy(base, pReqUuid->Uuid128, 16);
		uint16_t shortUuid = (uint16_t)base[12] | ((uint16_t)base[13] << 8);
		base[12] = 0;
		base[13] = 0;

		int idx = BtUuidFindBase(base);
		if (idx < 0)
		{
			return 0;		// unknown vendor base: no attribute can match
		}

		pOut->BaseIdx = (uint8_t)idx;
		pOut->Type = BT_UUID_TYPE_16;
		pOut->Uuid = shortUuid;
		return 1;
	}

	return -1;
}

uint32_t BtAttProcessReq(uint16_t ConnHdl, BtAttReqRsp_t * const pReqAtt, int ReqLen, BtAttReqRsp_t * const pRspAtt)
{
	if (pReqAtt == nullptr || pRspAtt == nullptr || ReqLen < 1)
	{
		return 0;
	}

	uint32_t retval = 0;

	DEBUG_PRINTF("ATT OpCode %x, L2Cap len %d\n", pReqAtt->OpCode, ReqLen);

	// Reject truncated request PDUs before any field is parsed. ReqLen counts
	// the opcode byte plus all parameters; a PDU shorter than the fixed size
	// required by its opcode would read handles/lengths past the end of the
	// received L2CAP buffer. PDUs arrive over the air from an untrusted peer.
	{
		int minLen = 1;
		switch (pReqAtt->OpCode)
		{
			case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:		minLen = 3; break;	// op + RxMtu(2)
			case BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ:	minLen = 5; break;	// op + start(2) + end(2)
			case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:	minLen = 7; break;	// op + start(2) + end(2) + type(2)
			case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:		minLen = 7; break;	// op + start(2) + end(2) + uuid16(2)
			case BT_ATT_OPCODE_ATT_READ_REQ:				minLen = 3; break;	// op + hdl(2)
			case BT_ATT_OPCODE_ATT_READ_BLOB_REQ:			minLen = 5; break;	// op + hdl(2) + offset(2)
			case BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ:		minLen = 5; break;	// op + at least 2 handles
			case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ: minLen = 3; break;	// op + at least 1 handle
			case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:	minLen = 7; break;	// op + start(2) + end(2) + uuid16(2)
			case BT_ATT_OPCODE_ATT_WRITE_REQ:				minLen = 3; break;	// op + hdl(2) + value(>=0)
			case BT_ATT_OPCODE_ATT_CMD:						minLen = 3; break;	// op + hdl(2) + value(>=0)
			case BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD:			minLen = 15; break;	// op + hdl(2) + signature(12)
			case BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ:		minLen = 5; break;	// op + hdl(2) + offset(2)
			case BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ:		minLen = 2; break;	// op + flags(1)
			case BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF:		minLen = 3; break;	// op + hdl(2) + value(>=0)
			case BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND:		minLen = 3; break;	// op + hdl(2) + value(>=0)
			case BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:		minLen = 1; break;	// op only
			default:										minLen = 1; break;
		}

		if (ReqLen < minLen)
		{
			return BtAttError(pRspAtt, 0, pReqAtt->OpCode, BT_ATT_ERROR_INVALID_PDU);
		}
	}

	uint16_t rspMtu = BtAttGetMtuForConn(ConnHdl);

	switch (pReqAtt->OpCode)
	{
		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:
			{

				DEBUG_PRINTF("ATT_EXCHANGE_MTU_REQ (0x02) \r\n");
				DEBUG_PRINTF("RxMtu %d %d\r\n", pReqAtt->ExchgMtuReqRsp.RxMtu, s_AttMtu);

				if (pReqAtt->ExchgMtuReqRsp.RxMtu < BT_ATT_MTU_MIN)
				{
					retval = BtAttError(pRspAtt, 0x0000, BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ, BT_ATT_ERROR_INVALID_ATT_VALUE);
					break;
				}
				retval = sizeof(BtAttExchgMtuReqRsp_t) + 1;
				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP;

				// Core Vol 3 Part F 3.4.2.2: the response holds the server's
				// own Rx MTU (a fixed local capability), and the ATT_MTU for the
				// link is min(client Rx MTU, server Rx MTU). Advertise the
				// largest ATT PDU this server can receive (BT_ATT_MTU_MAX, bounded
				// by the HCI buffer). Do NOT overwrite the stack-wide s_AttMtu
				// with the peer's offer: that global is the client-role/local
				// setting and one peer's value must not leak into other links.
				uint16_t serverRxMtu = BT_ATT_MTU_MAX;
				uint16_t peerRxMtu   = pReqAtt->ExchgMtuReqRsp.RxMtu;
				uint16_t linkMtu     = peerRxMtu < serverRxMtu ? peerRxMtu : serverRxMtu;

				pRspAtt->ExchgMtuReqRsp.RxMtu = serverRxMtu;

				// Record the negotiated per-link MTU so later requests size their
				// responses to it (via BtAttGetMtuForConn) and server-initiated
				// PDUs (notifications/indications) use it too.
				BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
				if (pConn != nullptr)
				{
					pConn->Conn.MaxMtu = linkMtu;
				}
				rspMtu = linkMtu;

				//DEBUG_PRINTF("MTU : %d\r\n", s_AttMtu);
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ:
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ (0x04) \r\n");
				BtAttFindInfoReq_t *req = (BtAttFindInfoReq_t*)&pReqAtt->FindInfoReq;

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP;

				int l = 0;
				uint8_t fmt = 0;
				uint8_t *p = (uint8_t*)pRspAtt->FindInfoRsp.HdlUuid16;

				for (uint16_t hdl = req->StartHdl; hdl <= req->EndHdl; hdl++)
				{
					BtAttDBEntry_t *entry = BtAttDBFindHandle(hdl);
					if (entry == nullptr)
					{
						continue;
					}

					uint8_t entryFmt = entry->TypeUuid.BaseIdx > 0 ?
							BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128 :
							BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;
					uint16_t entryLen = entryFmt == BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128 ?
							sizeof(BtAttHdlUuid128_t) :
							sizeof(BtAttHdlUuid16_t);

					if (fmt == 0)
					{
						fmt = entryFmt;
						pRspAtt->FindInfoRsp.Fmt = fmt;
					}
					else if (entryFmt != fmt)
					{
						break;
					}

					if ((uint16_t)(2 + l + entryLen) > rspMtu)
					{
						break;
					}

					if (fmt == BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128)
					{
						BtAttHdlUuid128_t *hu = (BtAttHdlUuid128_t*)p;
						uint8_t uuid128[16];

						BtUuidGetBase(entry->TypeUuid.BaseIdx, uuid128);
						uuid128[12] = entry->TypeUuid.Uuid & 0xff;
						uuid128[13] = entry->TypeUuid.Uuid >> 8;

						hu->Hdl = entry->Hdl;
						memcpy(hu->Uuid, uuid128, sizeof(hu->Uuid));
					}
					else
					{
						BtAttHdlUuid16_t *hu = (BtAttHdlUuid16_t*)p;
						hu->Hdl = entry->Hdl;
						hu->Uuid = entry->TypeUuid.Uuid;
					}

					p += entryLen;
					l += entryLen;

					if (hdl == 0xFFFF)
					{
						break;
					}
				}

				if (l > 0)
				{
					retval = 2 + l;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:
			{
				BtAttFindByTypeValueReq_t *req =
						(BtAttFindByTypeValueReq_t*)&pReqAtt->FindByTypeValueReq;
				int valLen = ReqLen - 7; // opcode + start + end + type

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl,
										BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ,
										BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Type };
				BtAttHdlRange_t *pOut = pRspAtt->FindByTypeValueRsp.Hdl;
				uint16_t start = req->StartHdl;
				int l = 0;

				// Group End Handle is the group end only for a grouping type
				// (Primary/Secondary Service); for any other type it equals the
				// found handle (Vol 3 Part F 3.4.3.1).
				bool grouping =
					req->Type == BT_UUID_DECLARATIONS_PRIMARY_SERVICE ||
					req->Type == BT_UUID_DECLARATIONS_SECONDARY_SERVICE;

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP;

				while (start <= req->EndHdl &&
					   (size_t)(1 + l) + sizeof(BtAttHdlRange_t) <= rspMtu)
				{
					uint16_t hdlStart = start;
					uint16_t hdlEnd   = req->EndHdl;
					BtAttDBEntry_t *entry = BtAttDBFindHdlRange(&uid16, &hdlStart, &hdlEnd);

					if (entry == nullptr)
						break;

					if (!grouping)
						hdlEnd = entry->Hdl;

					if (hdlEnd > req->EndHdl)
						hdlEnd = req->EndHdl;

					// An attribute whose value cannot be read due to permissions
					// must not be matched: reading and comparing it anyway lets a
					// peer confirm a protected value by guessing (a comparison
					// oracle). Treat it as not matching and move on.
					if (BtAttReadPermError(ConnHdl, entry) == 0 &&
						BtAttValueEquals(ConnHdl, entry, req->Val, (uint16_t)valLen))
					{
						pOut->StartHdl = entry->Hdl;
						pOut->EndHdl   = hdlEnd;
						pOut++;
						l += sizeof(BtAttHdlRange_t);
					}

					if (hdlEnd >= req->EndHdl || hdlEnd == 0xFFFF)
						break;

					start = hdlEnd + 1;
				}

				if (l > 0)
					retval = 1 + l;
				else
					retval = BtAttError(pRspAtt, req->StartHdl,
										BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ,
										BT_ATT_ERROR_ATT_NOT_FOUND);
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ: // Parse UUID Type of characteristic inside a BLE service
			{
				// Only the attributes with attribute handles between and including
				// the Starting Handle and the Ending Handle with the attribute type
				// that is the same as the Attribute Type given will be returned. To
				// search through all attributes, the starting handle shall be set to
				// 0x0001 and the ending handle shall be set to 0xFFFF.
				BtAttReadByTypeReq_t *req = (BtAttReadByTypeReq_t*)&pReqAtt->ReadByTypeReq;

				DEBUG_PRINTF(
						"BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ (0x08): StartHdl %d, EndHdl %d, Uuid16Type 0x%X \r\n",
						pReqAtt->ReadByTypeReq.StartHdl, pReqAtt->ReadByTypeReq.EndHdl,
						req->Uuid.Uuid16);

				// Starting handle 0x0000 is invalid (Vol 3 Part F 3.4.4.1), as
				// is a start beyond the end.
				if (req->StartHdl == 0 || req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				// The attribute type is 2 or 16 octets (op + start(2) + end(2) +
				// type). A 128-bit type is resolved to its registered base so
				// that "Discover Characteristics by UUID" / "Read using
				// Characteristic UUID" with a vendor 128-bit UUID works.
				BtUuid16_t uid16;
				int typeRes = BtAttResolveReqType(&req->Uuid, ReqLen - 5, &uid16);
				if (typeRes < 0)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_INVALID_PDU);
					break;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP;

				uint8_t *p = (uint8_t*)pRspAtt->ReadByTypeRsp.Data;

				// A well-formed 128-bit type with no registered base matches no
				// attribute: return Attribute Not Found rather than searching.
				BtAttDBEntry_t *entry = typeRes == 1 ?
					BtAttDBFindUuidRange(&uid16, req->StartHdl, req->EndHdl) : nullptr;
				if (entry)
				{
					DEBUG_PRINTF("Entry found\r\n");
				}
				else
				{
					DEBUG_PRINTF("Entry not found\r\n");
				}
				int l = 0;
				uint8_t permErr = 0;
				uint16_t permErrHdl = 0;
				uint16_t firstFull = 0;		// complete value length of the first matched attribute
				uint16_t pairValLen = 0;	// value bytes emitted per pair (first may be truncated)
				bool havePair = false;
				pRspAtt->ReadByTypeRsp.Len = 0;

				while (entry)
				{
					if (entry->Hdl < req->StartHdl || entry->Hdl > req->EndHdl)
					{
						break;
					}

					// Check permission before writing this attribute. Per Vol 3
					// Part F 3.4.4.1 a permission failure aborts with an Error
					// Response only when it is the first attribute; if any
					// attribute was already collected, return those instead. So
					// record the error and stop rather than emitting it here.
					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						permErr = err;
						permErrHdl = entry->Hdl;
						break;
					}

					// Compare complete value lengths without copying, so a later
					// value longer than the first is dropped rather than
					// truncated to masquerade as the common length (Vol 3 Part F
					// 3.4.4.1). The first attribute fixes that length; its value
					// is itself capped to (ATT_MTU - 4) per 3.4.4.2.
					uint16_t full = BtAttFullValueLen(entry);

					if (!havePair)
					{
						firstFull = full;
						uint16_t cap = (uint16_t)(rspMtu - 4);
						pairValLen = full < cap ? full : cap;
					}
					else if (full != firstFull)
					{
						break;
					}

					// Fit check on the actual pair size, not the MTU minimum, so
					// every pair that fits within ATT_MTU is included.
					if ((size_t)(2 + l) + 2 + pairValLen > rspMtu)
					{
						break;
					}

					// Copy handle + value straight into the response. The value
					// read is bounded by pairValLen, so no scratch is needed.
					p[0] = entry->Hdl & 0xFF;
					p[1] = entry->Hdl >> 8;
					BtAttReadValueForConn(ConnHdl, entry, 0, p + 2, pairValLen);
					p += 2 + pairValLen;
					l += 2 + pairValLen;

					if (!havePair)
					{
						havePair = true;
						pRspAtt->ReadByTypeRsp.Len = (uint8_t)(2 + pairValLen);
					}

					// Advance strictly past the handle just emitted. Continuing
					// from entry->pNext->Hdl is wrong when pNext is the list-end
					// sentinel (handle 0): the search would restart at handle 0
					// and re-match this same entry until the response fills.
					if (entry->Hdl >= req->EndHdl || entry->Hdl == 0xFFFF)
					{
						break;
					}
					req->StartHdl = entry->Hdl + 1;
					entry = BtAttDBFindUuidRange(&uid16, req->StartHdl, req->EndHdl);
				}

				if (l > 0)
				{
					retval = l + 2;
				}
				else if (permErr != 0)
				{
					// First attribute failed the permission check: return the
					// permission error, not Attribute Not Found.
					retval = BtAttError(pRspAtt, permErrHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, permErr);
				}
				else
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
				}
				//DEBUG_PRINTF("retval : %d\r\n", retval);
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ:
			{
				// The ATT_READ_REQ PDU is used to request the server to read the value
				// of an attribute and return its value in an ATT_READ_RSP PDU.
				BtAttReadReq_t *req = (BtAttReadReq_t*)&pReqAtt->ReadReq;

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_RSP;

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
					DEBUG_PRINTF("Entry with Hdl = %d found\r\n", req->Hdl);
					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, req->Hdl,
											BT_ATT_OPCODE_ATT_READ_REQ, err);
					}
					else
					{
						retval = BtAttReadValueForConn(ConnHdl, entry, 0, pRspAtt->ReadRsp.Data,
												rspMtu - 1)  + 1;
					}
				}
				else
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_READ_REQ, BT_ATT_ERROR_INVALID_HANDLE);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BLOB_REQ:
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ (0x0C):\r\n");

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BLOB_RSP;
				BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->ReadBlobReq.Hdl);

				if (entry)
				{
					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadBlobReq.Hdl,
											BT_ATT_OPCODE_ATT_READ_BLOB_REQ, err);
					}
					else if (pReqAtt->ReadBlobReq.Offset > BtAttCurValueLen(entry))
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadBlobReq.Hdl,
											BT_ATT_OPCODE_ATT_READ_BLOB_REQ, BT_ATT_ERROR_INVALID_OFFSET);
					}
					else
					{
						retval = BtAttReadValueForConn(ConnHdl, entry, pReqAtt->ReadBlobReq.Offset,
												pRspAtt->ReadBlobRsp.Data, rspMtu - 1) + 1;
					}
				}
				else
				{
					retval = BtAttError(pRspAtt, pReqAtt->ReadBlobReq.Hdl, BT_ATT_OPCODE_ATT_READ_BLOB_REQ, BT_ATT_ERROR_INVALID_HANDLE);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ:
			{
				// The ATT_READ_MULTIPLE_REQ PDU is used to request the server to read
				// two or more values of a set of attributes and return their values in
				// an ATT_READ_MULTIPLE_RSP PDU. Only values that have a known fixed size
				// can be read, with the exception of the last value that can have a variable
				// length. The knowledge of whether attributes have a known fixed size is
				// defined in a higher layer specification.

				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ (0x0E)\r\n");
				int nhdl = (ReqLen - 1) >> 1;
				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP;
				uint8_t *p = pRspAtt->ReadMultipleRsp.Data;
				retval = 1;
				// Concatenate values until the response reaches ATT_MTU or the
				// handles run out (Vol 3 Part F 3.4.4.7). Each read is capped at
				// the space remaining (rspMtu - retval), so the last value is
				// truncated to fit; the previous guard demanded a whole ATT_MTU
				// of free space and so returned an empty response at MTU 23.
				for (int i = 0; i < nhdl && retval < rspMtu; i++)
				{
					BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->ReadMultipleReq.Hdl[i]);
					if (entry == nullptr)
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadMultipleReq.Hdl[i], BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
						break;
					}
					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadMultipleReq.Hdl[i],
											BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ, err);
						break;
					}

					// Cap each value at the space remaining in the response
					// buffer (MTU - bytes already written), not the full MTU,
					// so cumulative values cannot overrun the response.
					int l = BtAttReadValueForConn(ConnHdl, entry, 0, p, rspMtu - retval);
					p += l;
					retval += l;
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ: // Parse the info of primary/secondary service
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ (0x10) \r\n");
				BtAttReadByGroupTypeReq_t *req = (BtAttReadByGroupTypeReq_t*)&pReqAtt->ReadByGroupTypeReq;
				DEBUG_PRINTF("Requested StartHdl = %d, EndHdl = %d\r\n", req->StartHdl, req->EndHdl);
				DEBUG_PRINTF("uuid16 = 0x%02x \r\n", req->Uuid.Uuid16);
				DEBUG_PRINTF("uuid32 = 0x%02x \r\n", req->Uuid.Uuid32);
				DEBUG_PRINTF("uuid128 = (0x) ");
				for (int i = 0; i < 16; i++)
					DEBUG_PRINTF("%x ", req->Uuid.Uuid128[i]);
				DEBUG_PRINTF("\r\n");

				// Starting handle 0x0000 is invalid (Vol 3 Part F 3.4.4.9), as
				// is a start beyond the end.
				if (req->StartHdl == 0 || req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				// Read By Group Type is defined only for the Primary Service
				// (0x2800) and Secondary Service (0x2801) group types (Core Vol 3
				// Part F 3.4.4.9). Accept either the 16-bit form or its 128-bit
				// expansion; a malformed type length is Invalid PDU, and any
				// other type (including an unknown 128-bit base) is Unsupported
				// Group Type.
				BtUuid16_t uid16;
				int gTypeRes = BtAttResolveReqType(&req->Uuid, ReqLen - 5, &uid16);
				if (gTypeRes < 0)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_INVALID_PDU);
					break;
				}
				if (gTypeRes != 1 || uid16.BaseIdx != 0 ||
					(uid16.Uuid != BT_UUID_DECLARATIONS_PRIMARY_SERVICE &&
					 uid16.Uuid != BT_UUID_DECLARATIONS_SECONDARY_SERVICE))
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_UNSUPP_GROUP_TYPE);
					break;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP;

				uint8_t *p = (uint8_t*)pRspAtt->ReadByGroupTypeRsp.Data;
				BtAttHdlRange_t *hu = (BtAttHdlRange_t*)p;
				int l = 0;

				pRspAtt->ReadByGroupTypeRsp.Len = 0;

				hu->StartHdl = req->StartHdl;
				hu->EndHdl = req->EndHdl;

				BtAttDBEntry_t *entry = BtAttDBFindHdlRange(&uid16, &hu->StartHdl, &hu->EndHdl);

				int baseidx = 0;// = entry->TypeUuid.BaseIdx;

				if (entry)
				{
					baseidx = entry->TypeUuid.BaseIdx;
				}

				pRspAtt->ReadByGroupTypeRsp.Len = 0;

				while (entry && (rspMtu - l) >= BT_ATT_MTU_MIN)
				{
					DEBUG_PRINTF("BaseIdx = %d, entry->Hdl = %d, entry->pNext->Hdl = %d, UuidType = %d, uuid16 = 0x%x\r\n",
							entry->TypeUuid.BaseIdx, entry->Hdl, (entry->pNext ? entry->pNext->Hdl : 0), entry->TypeUuid.Type, entry->TypeUuid.Uuid);

					if (entry->Hdl >= req->StartHdl && entry->Hdl <= req->EndHdl && baseidx == entry->TypeUuid.BaseIdx)
					{
						p += sizeof(BtAttHdlRange_t);

						int cnt = BtAttReadValueForConn(ConnHdl, entry, 0, p, rspMtu - l - sizeof(BtAttHdlRange_t));

						DEBUG_PRINTF("Ble Service UUID16 = 0x%X \r\n",
								((BtAttSrvcDeclar_t *)entry->Data)->Uuid.Uuid16);

						if (pRspAtt->ReadByGroupTypeRsp.Len == 0)
						{
							pRspAtt->ReadByGroupTypeRsp.Len = cnt;
						}
						else if (cnt != pRspAtt->ReadByGroupTypeRsp.Len)
						{
							break;
						}

						p += pRspAtt->ReadByGroupTypeRsp.Len;
						l += 4 + pRspAtt->ReadByGroupTypeRsp.Len;
						((BtAttHdlRange_t*)p)->StartHdl = hu->EndHdl;
						hu = (BtAttHdlRange_t*)p;// jump to the next entry
						hu->EndHdl = req->EndHdl;
					}
					else
					{
						break;
					}
					DEBUG_PRINTF("Next Find Range StartHdl = %d, EndHdl = %d, Uuid16Type = 0x%X\r\n",
							hu->StartHdl, hu->EndHdl, uid16.Uuid);
					entry = BtAttDBFindHdlRange(&uid16, &hu->StartHdl, &hu->EndHdl);
				}

				if (l > 0)
				{
					pRspAtt->ReadByGroupTypeRsp.Len += 4;
					retval = l + 2;
					break;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
				}

			}
			break;

		case BT_ATT_OPCODE_ATT_WRITE_REQ:	// Write With Response
			{
				BtAttWriteReq_t *req = (BtAttWriteReq_t*)&pReqAtt->WriteReq;

				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_WRITE_REQ (0x12):\r\n");

				if (req->Hdl < 1)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
					// ReqLen counts opcode(1) + handle(2) + data, so the data
					// length is ReqLen - 3. Passing ReqLen directly overruns the
					// value buffer by 3 bytes (handle + opcode worth of trailing
					// memory), corrupting the stored value.
					int dlen = ReqLen - 3;
					if (dlen < 0)
					{
						dlen = 0;
					}

					// A Client Characteristic Configuration descriptor is a fixed
					// 2-octet field. A write of any other length is rejected with
					// Invalid Attribute Value Length (Core spec Vol 3 Part G,
					// 3.3.3.3), not silently truncated or accepted.
					bool bCccd = BtAttEntryIsCccd(entry);

					if (bCccd && dlen != 2)
					{
						retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_ATT_VALUE);
						break;
					}

					uint8_t err = BtAttWritePermError(ConnHdl, entry,
													  BT_ATT_OPCODE_ATT_WRITE_REQ,
													  req->Data, (uint16_t)dlen);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, err);
						break;
					}

					// A value longer than the attribute can hold must be
					// rejected with Invalid Attribute Value Length, not written
					// in part and acknowledged as success (Vol 3 Part F 3.4.5.1).
					// Check the capacity before the write so a rejected write
					// leaves the stored value untouched. A zero-length write is
					// valid. Non-char-value attributes (declarations, the CCCD
					// checked above) keep their existing handling.
					BtGattChar_t *pWrChar = nullptr;
					if (BtAttEntryIsCharValue(entry))
					{
						pWrChar = BtAttEntryChar(entry);
						if (pWrChar == nullptr || (uint16_t)dlen > pWrChar->MaxDataLen)
						{
							retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_ATT_VALUE);
							break;
						}
					}

					BtAttWriteValueForConn(ConnHdl, entry, 0, req->Data, (uint16_t)dlen);

					// A Write Request replaces the whole attribute value, so the
					// current length becomes exactly the written length: a
					// shorter write truncates and a zero-length write clears the
					// value (Vol 3 Part F 3.4.5.1). BtAttWriteValue only extends
					// on an offset write (prepared writes rely on that), so the
					// replacement length is set here for the full-value path.
					if (pWrChar != nullptr)
					{
						pWrChar->ValueLen = (uint16_t)dlen;
					}

					pRspAtt->OpCode = BT_ATT_OPCODE_ATT_WRITE_RSP;

					retval = 1;
				}
				else
				{
					// A write to a handle that is not in the database is an
					// invalid handle, not attribute-not-found (Vol 3 Part F
					// 3.4.5.1). Attribute Not Found is a discovery-request code.
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_HANDLE);

				}
			}
			break;
		case BT_ATT_OPCODE_ATT_CMD:		// Write without response
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_CMD (0x52):\r\n");

				// Write Command (opcode 0x52) uses BtAttWriteCmd_t (handle + data).
				// SignedWriteCmd is opcode 0xD2 (handle + data + 12B signature).
				// Both union members overlay so the data offset is identical,
				// but the field name should match the opcode being handled.
				BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->WriteCmd.Hdl);

				if (entry)
				{
					// ReqLen counts opcode + handle + data, so the data length
					// is ReqLen - 1 (opcode) - 2 (handle) = ReqLen - 3.
					int dlen = ReqLen - 3;
					if (dlen < 0)
					{
						dlen = 0;
					}
					if (BtAttWritePermError(ConnHdl, entry, BT_ATT_OPCODE_ATT_CMD,
											pReqAtt->WriteCmd.Data, (uint16_t)dlen) == 0)
					{
						// Write Command has no error response, so an invalid
						// length cannot be reported: the whole command is ignored
						// (do not touch the stored value or its length) per Vol 3
						// Part F 3.4.5.3. Validate the length in the handler, as
						// the Write Request path does, rather than relying on the
						// write helper to reject it: a characteristic value must
						// fit the attribute maximum, and a CCCD must be exactly
						// two octets. A valid write replaces the value, so the
						// current length becomes exactly dlen.
						BtGattChar_t *pCmdChar =
							BtAttEntryIsCharValue(entry) ? BtAttEntryChar(entry) : nullptr;

						bool valid;
						if (pCmdChar != nullptr)
						{
							valid = (uint16_t)dlen <= pCmdChar->MaxDataLen;
						}
						else if (BtAttEntryIsCccd(entry))
						{
							// The value itself was already checked by
							// BtAttWritePermError above; only the fixed length
							// is left to enforce here.
							valid = (dlen == 2);
						}
						else
						{
							valid = true;
						}

						if (valid)
						{
							BtAttWriteValueForConn(ConnHdl, entry, 0,
											pReqAtt->WriteCmd.Data, (uint16_t)dlen);
							if (pCmdChar != nullptr)
							{
								pCmdChar->ValueLen = (uint16_t)dlen;
							}
						}
					}

					retval = 0;
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD:
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD (0xD2):\r\n");

				// Signed Write Command is a command PDU: no ATT Error Response.
				// The generic fallback does not own CSRK/signature state, so it
				// must ignore the command unless a port/SMP override verifies it.
				int dlen = ReqLen - 15;	// opcode + handle + 12-byte signature
				if (dlen < 0)
				{
					retval = 0;
					break;
				}

				BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->SignedWriteCmd.Hdl);
				const uint8_t *sig = &pReqAtt->SignedWriteCmd.Data[dlen];

				if (entry != nullptr &&
					BtAttSignedWriteVerify(ConnHdl, &pReqAtt->SignedWriteCmd,
										   (uint16_t)dlen, sig))
				{
					uint8_t err = BtAttWritePermError(ConnHdl, entry,
													  BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD,
													  pReqAtt->SignedWriteCmd.Data,
													  (uint16_t)dlen);
					if (err == 0)
					{
						BtAttWriteValueForConn(ConnHdl, entry, 0,
											   pReqAtt->SignedWriteCmd.Data,
											   (uint16_t)dlen);
					}
				}

				retval = 0;
			}
			break;
		case BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ:
			{
				BtAttPrepareWriteReq_t *req = &pReqAtt->PrepareWriteReq;
				BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);

				// Value bytes = total PDU - opcode(1) - handle(2) - offset(2).
				int vlen = ReqLen - 5;
				if (vlen < 0)
				{
					vlen = 0;
				}

				if (pConn == nullptr || pConn->Conn.pLongWrBuff == nullptr)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ, BT_ATT_ERROR_PREPARE_QUE_FULL);
					break;
				}
				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);
				if (entry == nullptr)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				uint8_t err = BtAttWritePermError(ConnHdl, entry,
												  BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ,
												  req->Data, (uint16_t)vlen);
				if (err != 0)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ, err);
					break;
				}

				// Append a { Hdl, Offset, Len, Data } record to the per-link
				// prepare queue. The 6-byte header keeps it self-describing so
				// the execute step can walk and reassemble it.
				uint16_t vl   = (uint16_t)vlen;
				uint32_t need = 6 + (uint32_t)vl;
				if ((uint32_t)pConn->Conn.LongWrLen + need > pConn->Conn.LongWrBuffSize)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ, BT_ATT_ERROR_PREPARE_QUE_FULL);
					break;
				}

				uint8_t *q = pConn->Conn.pLongWrBuff + pConn->Conn.LongWrLen;
				memcpy(q,     &req->Hdl,    2);
				memcpy(q + 2, &req->Offset, 2);
				memcpy(q + 4, &vl,          2);
				memcpy(q + 6, req->Data,    vl);
				pConn->Conn.LongWrLen += (uint16_t)need;

				// Echo the request back as the Prepare Write Response.
				pRspAtt->OpCode               = BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP;
				pRspAtt->PrepareWriteRsp.Hdl    = req->Hdl;
				pRspAtt->PrepareWriteRsp.Offset = req->Offset;
				memcpy(pRspAtt->PrepareWriteRsp.Data, req->Data, vl);
				retval = 1 + 2 + 2 + vl;	// opcode + handle + offset + value
			}
			break;
		case BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ:
			{
				BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
				uint8_t flag = pReqAtt->ExecuteWriteReq.Flag;

				if (flag != 0x00 && flag != 0x01)
				{
					retval = BtAttError(pRspAtt, 0,
										BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ,
										BT_ATT_ERROR_INVALID_PDU);
					break;
				}

				if (flag == 0x01)
				{
					// Execute: apply the queued writes, firing each char WrCB.
					// A queued write that fails validation aborts the execute
					// with an Error Response carrying the offending handle.
					uint16_t failHdl = 0;
					uint8_t  err = BtAttExecLongWrite(pConn, &failHdl);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, failHdl, BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ, err);
						break;
					}
				}
				else if (pConn != nullptr)
				{
					// Cancel: discard the queue without applying it.
					pConn->Conn.LongWrLen = 0;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP;
				retval = 1;	// opcode only
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ:
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ (0x20) \r\n");
				// The handle list sits one octet into the PDU (after the
				// opcode), so it is 2-byte misaligned. Walk it with a byte
				// pointer and assemble each handle little-endian rather than
				// dereferencing a uint16_t*, which is undefined and faults on
				// targets without unaligned access (for example Cortex-M0).
				uint8_t *hdlp = (uint8_t*)pReqAtt->ReadMultipleVarReq.Hdl;
				uint8_t *p = pRspAtt->ReadMultipleVarRsp.Data;
				int l = 0;
				int hdlBytes = ReqLen - 1;	// exclude opcode

				if (hdlBytes < 2 || (hdlBytes & 1))
				{
					retval = BtAttError(pRspAtt, 0,
										BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ,
										BT_ATT_ERROR_INVALID_PDU);
					break;
				}

				while (hdlBytes > 0 && l + 3 <= rspMtu)
				{
					uint16_t curHdl = (uint16_t)(hdlp[0] | (hdlp[1] << 8));
					BtAttDBEntry_t *entry = BtAttDBFindHandle(curHdl);

					if (entry == nullptr)
					{
						retval = BtAttError(pRspAtt, curHdl,
											BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ,
											BT_ATT_ERROR_INVALID_HANDLE);
						break;
					}
					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, curHdl,
											BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ, err);
						break;
					}

					pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP;
					uint16_t space = (uint16_t)(rspMtu - 1 - l - 2);	// MTU - opcode - existing payload - length field
					// The Length field holds the complete attribute value
					// length, even when the value is truncated to fit (Vol 3
					// Part F 3.4.4.13), so the client can detect that the last
					// value was cut and re-read it. Only value bytes that fit
					// are emitted.
					uint16_t full = BtAttFullValueLen(entry);
					uint16_t n = BtAttReadValueForConn(ConnHdl, entry, 0, p + 2, space);
					p[0] = full & 0xFF;
					p[1] = full >> 8;
					p += n + 2;
					l += n + 2;
					hdlp += 2;
					hdlBytes -= 2;

					// Only the last tuple may be truncated: once a value did
					// not fit, stop rather than emitting more tuples.
					if (n < full)
					{
						break;
					}
				}

				if (retval != 0 && pRspAtt->OpCode == BT_ATT_OPCODE_ATT_ERROR_RSP)
				{
					break;
				}
				else if (l > 0)
				{
					retval = l + 1;
				}
				else
				{
					retval = BtAttError(pRspAtt,
										pReqAtt->ReadMultipleVarReq.Hdl[0],
										BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ,
										BT_ATT_ERROR_ATT_NOT_FOUND);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF:
			{
				// Server notified us (client role). Deliver the value to the
				// application. PDU is opcode(1) + value handle(2) + value.
				int dlen = ReqLen - 3;
				if (dlen < 0)
				{
					dlen = 0;
				}
				BtGattClientNotified(ConnHdl, pReqAtt->HandleValueNtf.ValHdl,
									 pReqAtt->HandleValueNtf.Data, (uint16_t)dlen);
			}
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND:
			{
				// We are the client and the server indicated us. ATT requires a
				// Handle Value Confirmation in reply before the server may send
				// another indication (Core spec Vol 3 Part F, 3.4.7.2). The
				// confirmation is the opcode alone, no parameters. Deliver the
				// value first, then confirm.
				int dlen = ReqLen - 3;
				if (dlen < 0)
				{
					dlen = 0;
				}
				BtGattClientNotified(ConnHdl, pReqAtt->HandleValueInd.Hdl,
									 pReqAtt->HandleValueInd.Data, (uint16_t)dlen);

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM;
				retval = 1;
			}
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:
			{
				//DEBUG_PRINTF("BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:\r\n");
				// Peer confirmed our indication; release the per-link slot so
				// the next indication on this connection can be sent.
				BtGattHandleValueConfirm(ConnHdl);
			}
			break;
		default:
			//DEBUG_PRINTF("OpCode : %x\r\n", pReqAtt->OpCode);
			// ATT spec: a server that receives a Request it does not support
			// MUST answer with an Error Response (Request Not Supported).
			// Staying silent leaves the client waiting and the link idles out.
			// Commands (bit6=0x40), notifications/indications/confirmations and
			// responses do not get a reply. A request is anything else with an
			// even-ish method that expects a response; treat unknown opcodes
			// that are not command/notify/confirm/response as requests.
			{
				uint8_t op = pReqAtt->OpCode;
				bool isCommand = (op & 0x40) != 0;	// Write Command, Signed Write
				bool noRsp = isCommand ||
							 op == BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF ||
							 op == BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND ||
							 op == BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM ||
							 op == BT_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF ||
							 op == BT_ATT_OPCODE_ATT_ERROR_RSP ||
							 (op & 0x01);	// odd opcodes in 1..0x1B are responses
				if (!noRsp)
				{
					retval = BtAttError(pRspAtt, 0x0000, op,
										 BT_ATT_ERROR_REQUEST_NOT_SUPP);
				}
			}
			break;
	}

	return retval;
}

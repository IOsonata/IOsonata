/**-------------------------------------------------------------------------
@file	bt_adv.cpp

@brief	Bluetooth advertisement generic implementation


@author	Hoang Nguyen Hoan
@date	Oct. 2, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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
#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_appearance.h"

/******** For DEBUG ************/
//#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

// Cap used when the full local name does not fit in the adv payload.
// Per BT Core spec the split between complete and shortened name is
// implementation defined ("as much as fits"); this is the upper bound.
#ifndef BT_ADV_SHORT_NAME_MAX
#define BT_ADV_SHORT_NAME_MAX		30
#endif

static void BtAdvWriteU16Le(uint8_t *pData, uint16_t Val)
{
	pData[0] = (uint8_t)(Val & 0xFF);
	pData[1] = (uint8_t)(Val >> 8);
}

static int BtAdvDataFindAdvTag(uint8_t Tag, uint8_t *pData, int Len)
{
	int retval = -1;
	int idx = 0;

	if (pData == NULL || Len <= 0)
	{
		return -1;
	}

	while (Len > 0)
	{
		BtAdvDataHdr_t *hdr = (BtAdvDataHdr_t*)&pData[idx];
		int recLen = hdr->Len + 1;

		if (hdr->Len == 0 || recLen > Len)
		{
			break;
		}

		if (hdr->Type == Tag)
		{
			retval = idx;
			break;
		}
		idx += recLen;
		Len -= recLen;
	}

	return retval;
}

/**
 * @brief	Allocate space to add new advertisement data
 *
 * This function allocate space in the advertisement packet to add new data.
 * If enough space available, it will prefill the data header. Caller needs only
 * to copy new data into it.
 * If type already exists, it will be removed if enough space to store new data
 *
 * @param 	pAdvPkt : Pointer to Adv packet to add data into
 * @param 	Type 	: GAP data type of the data
 * @param	Len		: Length in bytes of the data
 *
 * @return	Pointer to location to store new data.
 * 			NULL if not enough space. Old data will not be removed
 */
BtAdvData_t *BtAdvDataAllocate(BtAdvPacket_t *pAdvPkt, uint8_t Type, int Len)
{
	if (pAdvPkt == nullptr || pAdvPkt->pData == nullptr || Len < 0)
	{
		return nullptr;
	}

	// Cap Len to the packet capacity. A record cannot be larger than the
	// packet, and this also bounds Len + 2 against signed overflow when
	// computing recLen below.
	if (Len > pAdvPkt->MaxLen)
	{
		return nullptr;
	}

	int recLen = Len + 2;
	int idx = BtAdvDataFindAdvTag(Type, pAdvPkt->pData, pAdvPkt->Len);

	if (idx >= 0)
	{
		// Tag already exists, remove it first
		BtAdvData_t *p = (BtAdvData_t*)&pAdvPkt->pData[idx];
		int oldRecLen = p->Hdr.Len + 1;
		int l = pAdvPkt->Len - oldRecLen;

		if (recLen > (pAdvPkt->MaxLen - l))
		{
			return nullptr;
		}

		memmove(&pAdvPkt->pData[idx], &pAdvPkt->pData[idx + oldRecLen], l - idx);
		pAdvPkt->Len = l;
	}
	else if (recLen > (pAdvPkt->MaxLen - pAdvPkt->Len))
	{
		return nullptr;
	}

	BtAdvData_t *p = (BtAdvData_t*)&pAdvPkt->pData[pAdvPkt->Len];
	p->Hdr.Len = Len + 1;
	p->Hdr.Type = Type;
	pAdvPkt->Len += recLen;

	return p;
}

/**
 * @brief	Add advertisement data into the adv packet
 *
 * @param 	pAdvPkt : Pointer to Adv packet to add data into
 * @param 	Type 	: GAP data type of the data
 * @param	pData	: Pointer to data to add
 * @param	Len		: Length in bytes of the data
 *
 * @return	true - success
 */
bool BtAdvDataAdd(BtAdvPacket_t * const pAdvPkt, uint8_t Type, uint8_t *pData, int Len)
{
	BtAdvData_t *p = BtAdvDataAllocate(pAdvPkt, Type, Len);

	if (p == nullptr)
	{
		return false;
	}

	if (pData != NULL && Len > 0)
	{
		memcpy(p->Data, pData, Len);
	}

	return true;
}

/**
 * @brief	Remove advertisement data from the adv packet
 *
 * @param 	pAdvPkt	: Pointer to Adv packet to add data into
 * @param 	Type 	: GAP data type of the data
 *
 * @return	none
 */
void BtAdvDataRemove(BtAdvPacket_t * const pAdvPkt, uint8_t Type)
{
	if (pAdvPkt == nullptr || pAdvPkt->pData == nullptr || pAdvPkt->Len <= 0)
		return;

	int idx = BtAdvDataFindAdvTag(Type, pAdvPkt->pData, pAdvPkt->Len);

	if (idx >= 0)
	{
		BtAdvData_t *p = (BtAdvData_t*)&pAdvPkt->pData[idx];
		int recLen = p->Hdr.Len + 1;
		int tailLen = pAdvPkt->Len - idx - recLen;

		if (tailLen > 0)
		{
			memmove(&pAdvPkt->pData[idx], &pAdvPkt->pData[idx + recLen], tailLen);
		}
		pAdvPkt->Len -= recLen;
	}
}

/**
 * @brief	Add UUID list to the advertising data
 *
 * When BaseIdx > 0 the array carries short identifiers (uuid16 or uuid32)
 * on top of a 128-bit base UUID. The output AD record is always uuid128:
 * each entry is expanded to a full 128-bit UUID by combining the base
 * with the short.
 *
 * When BaseIdx == 0 the array is emitted as-is in the format matching
 * Type (uuid16, uuid32 or uuid128).
 *
 * @param 	pAdvPkt	: Pointer to Adv packet to add data into
 * @param 	pUid	: Pointer to UUID array list
 * @param 	bComplete : true - UUID list is complete, false - partial
 *
 * @return	true - success
 */
bool BtAdvDataAddUuid(BtAdvPacket_t * const pAdvPkt, const BtUuidArr_t *pUid, bool bComplete)
{
	int l = 0;
	uint8_t gaptype = 0;

	if (pAdvPkt == nullptr || pUid == nullptr || pUid->Count <= 0)
	{
		return false;
	}

	if (pUid->BaseIdx > 0)
	{
		// Custom-base UUID. Only uuid16 and uuid32 shorthand make sense
		// here; a full uuid128 array with a base is not a meaningful
		// combination.
		if (pUid->Type != BT_UUID_TYPE_16 && pUid->Type != BT_UUID_TYPE_32)
		{
			return false;
		}

		uint8_t base[16];

		if (BtUuidGetBase(pUid->BaseIdx, base) == false)
		{
			return false;
		}

		l = 16 * pUid->Count;
		gaptype = bComplete ? BT_GAP_DATA_TYPE_COMPLETE_SRVC_UUID128 : BT_GAP_DATA_TYPE_INCOMPLETE_SRVC_UUID128;

		// Allocate the record in the packet and write the expanded UUIDs
		// straight into it. No temp buffer needed.
		BtAdvData_t *p = BtAdvDataAllocate(pAdvPkt, gaptype, l);
		if (p == nullptr)
		{
			return false;
		}

		for (int i = 0; i < pUid->Count; i++)
		{
			uint8_t *slot = &p->Data[i * 16];
			memcpy(slot, base, 16);

			if (pUid->Type == BT_UUID_TYPE_16)
			{
				slot[12] = (uint8_t)(pUid->Uuid16[i] & 0xFF);
				slot[13] = (uint8_t)(pUid->Uuid16[i] >> 8);
			}
			else // BT_UUID_TYPE_32
			{
				slot[12] = (uint8_t)(pUid->Uuid32[i] & 0xFF);
				slot[13] = (uint8_t)((pUid->Uuid32[i] >> 8) & 0xFF);
				slot[14] = (uint8_t)((pUid->Uuid32[i] >> 16) & 0xFF);
				slot[15] = (uint8_t)((pUid->Uuid32[i] >> 24) & 0xFF);
			}
		}

		return true;
	}

	switch (pUid->Type)
	{
		case BT_UUID_TYPE_16:
			gaptype = bComplete ? BT_GAP_DATA_TYPE_COMPLETE_SRVC_UUID16 : BT_GAP_DATA_TYPE_INCOMPLETE_SRVC_UUID16;
			l = pUid->Count * 2;
			break;
		case BT_UUID_TYPE_32:
			gaptype = bComplete ? BT_GAP_DATA_TYPE_COMPLETE_SRVC_UUID32 : BT_GAP_DATA_TYPE_INCOMPLETE_SRVC_UUID32;
			l = pUid->Count * 4;
			break;
		case BT_UUID_TYPE_128:
			gaptype = bComplete ? BT_GAP_DATA_TYPE_COMPLETE_SRVC_UUID128 : BT_GAP_DATA_TYPE_INCOMPLETE_SRVC_UUID128;
			l = pUid->Count * 16;
			break;
		default:
			return false;
	}

	return BtAdvDataAdd(pAdvPkt, gaptype, (uint8_t*)pUid->Uuid16, l);
}

bool BtAdvDataSetDevName(BtAdvPacket_t * const pAdvPkt, const char *pName)
{
	if (pAdvPkt == nullptr || pName == nullptr)
	{
		return false;
	}

	// Need at least 3 bytes free: length byte + type byte + 1 name byte.
	if (pAdvPkt->MaxLen <= pAdvPkt->Len + 2)
	{
		return false;
	}

	size_t l = strlen(pName);
	size_t mxl = (size_t)(pAdvPkt->MaxLen - pAdvPkt->Len - 2);
	uint8_t type = BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME;

	if (l > BT_ADV_SHORT_NAME_MAX || l > mxl)
	{
		type = BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME;
		l = min((size_t)BT_ADV_SHORT_NAME_MAX, mxl);
	}

	if (l == 0)
	{
		return false;
	}

	return BtAdvDataAdd(pAdvPkt, type, (uint8_t*)pName, l);
}

size_t BtAdvDataGetDevName(uint8_t *pAdvData, size_t AdvLen, char *pName, size_t NameLen)
{
	size_t retval = 0;

	if (pAdvData == NULL || pName == NULL || NameLen == 0)
	{
		return 0;
	}

	int idx = BtAdvDataFindAdvTag(BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME, pAdvData, AdvLen);

	if (idx < 0)
	{
		idx = BtAdvDataFindAdvTag(BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME, pAdvData, AdvLen);
	}
	if (idx >= 0)
	{
		BtAdvData_t *p = (BtAdvData_t*)&pAdvData[idx];
		size_t payloadLen = (p->Hdr.Len > 0) ? (size_t)(p->Hdr.Len - 1) : 0;
		retval = min(NameLen - 1, payloadLen);
		memcpy(pName, p->Data, retval);
		pName[retval] = '\0';
	}
	else
	{
		pName[0] = '\0';
	}

	return retval;
}

size_t BtAdvDataGetManData(uint8_t *pAdvData, size_t AdvLen, uint8_t *pBuff, size_t BuffLen)
{
	size_t retval = 0;

	if (pAdvData == NULL || pBuff == NULL || BuffLen == 0)
	{
		return 0;
	}

	int idx = BtAdvDataFindAdvTag(BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, pAdvData, AdvLen);

	if (idx >= 0)
	{
		BtAdvData_t *p = (BtAdvData_t*)&pAdvData[idx];
		size_t payloadLen = (p->Hdr.Len > 0) ? (size_t)(p->Hdr.Len - 1) : 0;
		retval = min(BuffLen, payloadLen);
		memcpy(pBuff, p->Data, retval);
	}

	return retval;
}

bool BtAdvEncode(const BtAppCfg_t *pCfg, BtAdvPacket_t *pAdvPkt, BtAdvPacket_t *pSrPkt)
{
	if (pCfg == nullptr || pAdvPkt == nullptr)
	{
		return false;
	}

	// Flags: BR/EDR not supported; limited or general discoverable based on
	// whether the app set a timeout (limited mode).
	uint8_t flags = BT_GAP_DATA_TYPE_FLAGS_NO_BREDR;

	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		if (pCfg->AdvTimeout != 0)
		{
			flags |= BT_GAP_DATA_TYPE_FLAGS_LIMITED_DISCOVERABLE;
		}
		else
		{
			flags |= BT_GAP_DATA_TYPE_FLAGS_GENERAL_DISCOVERABLE;
		}
	}

	if (BtAdvDataAdd(pAdvPkt, BT_GAP_DATA_TYPE_FLAGS, &flags, 1) == false)
	{
		return false;
	}

	// Appearance is optional; log if dropped for lack of room.
	if (pCfg->Appearance != BT_APPEAR_UNKNOWN_GENERIC)
	{
		uint8_t appBuf[2];
		BtAdvWriteU16Le(appBuf, pCfg->Appearance);
		if (BtAdvDataAdd(pAdvPkt, BT_GAP_DATA_TYPE_APPEARANCE, appBuf, 2) == false)
		{
			DEBUG_PRINTF("BtAdvEncode: appearance dropped, no room\r\n");
		}
	}

	// Manufacturer specific data.
	// Legacy: separate records on adv and sr.
	// Extended: merge adv+sr into a single record on the adv packet
	// (extended adv has no scan response in the same sense).
	if (pCfg->bExtAdv == false)
	{
		if (pCfg->pAdvManData != NULL)
		{
			int l = pCfg->AdvManDataLen + 2;
			BtAdvData_t *p = BtAdvDataAllocate(pAdvPkt,
				BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
			if (p == NULL)
			{
				return false;
			}
			BtAdvWriteU16Le(p->Data, pCfg->VendorId);
			memcpy(&p->Data[2], pCfg->pAdvManData, pCfg->AdvManDataLen);
		}

		if (pCfg->pSrManData != NULL)
		{
			if (pSrPkt == nullptr)
			{
				DEBUG_PRINTF("BtAdvEncode: sr man data set but no sr packet\r\n");
			}
			else
			{
				int l = pCfg->SrManDataLen + 2;
				BtAdvData_t *p = BtAdvDataAllocate(pSrPkt,
					BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
				if (p == NULL)
				{
					return false;
				}
				BtAdvWriteU16Le(p->Data, pCfg->VendorId);
				memcpy(&p->Data[2], pCfg->pSrManData, pCfg->SrManDataLen);
			}
		}
	}
	else
	{
		int l = 2;
		if (pCfg->pAdvManData != NULL) l += pCfg->AdvManDataLen;
		if (pCfg->pSrManData  != NULL) l += pCfg->SrManDataLen;

		if (l > 2)
		{
			BtAdvData_t *p = BtAdvDataAllocate(pAdvPkt,
				BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
			if (p == NULL)
			{
				return false;
			}
			BtAdvWriteU16Le(p->Data, pCfg->VendorId);
			int off = 2;
			if (pCfg->pAdvManData != NULL)
			{
				memcpy(&p->Data[off], pCfg->pAdvManData, pCfg->AdvManDataLen);
				off += pCfg->AdvManDataLen;
			}
			if (pCfg->pSrManData != NULL)
			{
				memcpy(&p->Data[off], pCfg->pSrManData, pCfg->SrManDataLen);
			}
		}
	}

	// Device name. BtAdvDataSetDevName picks full or short based on space and
	// adds it to the adv packet. UUIDs go on the scan response in legacy mode
	// to free room in the 31-byte adv payload, or on the adv packet in extended.
	BtAdvPacket_t *uidadvpkt;
	if (pCfg->pDevName != NULL)
	{
		if (BtAdvDataSetDevName(pAdvPkt, pCfg->pDevName) == false)
		{
			return false;
		}
		uidadvpkt = pCfg->bExtAdv ? pAdvPkt : pSrPkt;
	}
	else
	{
		uidadvpkt = pAdvPkt;
	}

	// Service UUIDs (peripheral role only). Optional; log if dropped for lack of room.
	if (pCfg->pAdvUuid != NULL && (pCfg->Role & BTAPP_ROLE_PERIPHERAL) && uidadvpkt != nullptr)
	{
		if (BtAdvDataAddUuid(uidadvpkt, pCfg->pAdvUuid, pCfg->bCompleteUuidList) == false)
		{
			DEBUG_PRINTF("BtAdvEncode: service UUIDs dropped, no room\r\n");
		}
	}

	return true;
}

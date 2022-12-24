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

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_gatt.h"
//#include "bluetooth/bt_ctlr.h"

#ifndef BT_GATT_ENTRY_MAX_COUNT
#define BT_GATT_ENTRY_MAX_COUNT		100
#endif

#define BT_GATT_SRVC_MAX_COUNT		((BT_GATT_ENTRY_MAX_COUNT) >> 1)

#if 0
#pragma pack(push,4)
typedef struct {
	bool bValid;
	BtGattSrvc_t *pSrvc;
} BtGattSrvcEntry_t;
#pragma pack(pop)
#endif

static BtGattCharSrvcChanged_t s_BtGattCharSrvcChanged = {0,};

static BtGattChar_t s_BtGattDftlChar[] = {
	{
		// Read characteristic
		.Uuid = BT_UUID_GATT_CHAR_SERVICE_CHANGED,
		.MaxDataLen = sizeof(BtGattCharSrvcChanged_t),
		.Property =	BT_GATT_CHAR_PROP_INDICATE,
		.pDesc = NULL,						// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pValue = &s_BtGattCharSrvcChanged,
		.ValueLen = 0,
	},
};

static BtGattSrvcCfg_t s_BtGattDftlSrvcCfg = {
	//.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.bCustom = false,
	.UuidBase = {0,},		// Base UUID
	.UuidSrvc = BT_UUID_GATT_SERVICE_GENERIC_ATTRIBUTE,		// Service UUID
	.NbChar = sizeof(s_BtGattDftlChar) / sizeof(BtGattChar_t),				// Total number of characteristics for the service
	.pCharArray = s_BtGattDftlChar,				// Pointer a an array of characteristic
};

alignas(4) static BtGattListEntry_t s_BtGattEntryTbl[BT_GATT_ENTRY_MAX_COUNT] = {0,};
static int s_NbGattListEntry = 0;
//alignas(4) static BtGattSrvcEntry_t s_BtGattSrvcTbl[BT_GATT_SRVC_MAX_COUNT] = {{0,}, };
//static int s_NbGattSrvcEntry = 0;
static BtGattSrvc_t *s_pBtGattSrvcList = nullptr;
//static BtGattSrvc_t *s_pBtGattSrvcTail = nullptr;

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

uint16_t BtGattRegister(BtUuid16_t *pTypeUuid, void * const pAttVal)
{
	if (pAttVal == nullptr || s_NbGattListEntry >= BT_GATT_ENTRY_MAX_COUNT)
	{
		return -1;
	}

	BtGattListEntry_t *p = (BtGattListEntry_t*)&s_BtGattEntryTbl[s_NbGattListEntry];

	p->TypeUuid = *pTypeUuid;

	if (p->TypeUuid.BaseIdx == 0)
	{
		// Standard Bluetooth 128 bits based UUID
		switch (p->TypeUuid.Uuid)
		{
			case BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_GATT_DECLARATIONS_SECONDARY_SERVICE:
				s_BtGattEntryTbl[s_NbGattListEntry].SrvcDeclar = *(BtGattSrvcDeclar_t *)pAttVal;
				break;
			case BT_UUID_GATT_DECLARATIONS_INCLUDE:
				memcpy(&s_BtGattEntryTbl[s_NbGattListEntry].SrvcInc, pAttVal, sizeof(BtGattSrvcInclude_t));
				break;
			case BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC:
				memcpy(&s_BtGattEntryTbl[s_NbGattListEntry].CharDeclar, pAttVal, sizeof(BtGattCharDeclar_t));
				s_BtGattEntryTbl[s_NbGattListEntry].CharDeclar.ValHdl = s_NbGattListEntry + 2;
				s_BtGattEntryTbl[s_NbGattListEntry].Hdl = s_NbGattListEntry + 1;
				//s_NbGattListEntry++;
				//s_BtGatEntryTbl[s_NbGattListEntry].TypeUuid = ((BtGattCharDeclar_t*)pAttVal)->Uuid;
				//((BtGattCharDeclar_t*)pAttVal)->ValHdl = s_NbGattListEntry + 1;
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			//case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
			//	s_BtGatEntryTbl[s_NbGattListEntry].pVal = pAttVal;
			//	break;
			case BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				s_BtGattEntryTbl[s_NbGattListEntry].Val32 = *(uint16_t*)pAttVal;
				break;
			case BT_UUID_GATT_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;

			default:
				s_BtGattEntryTbl[s_NbGattListEntry].pChar = (BtGattChar_t*)pAttVal;
//				memcpy(&s_BtGattEntryTbl[s_NbGattListEntry].CharVal, pAttVal, sizeof(BtGattCharValue_t));
		}
	}
	else
	{
		s_BtGattEntryTbl[s_NbGattListEntry].pChar = (BtGattChar_t*)pAttVal;
//		memcpy(&s_BtGattEntryTbl[s_NbGattListEntry].CharVal, pAttVal, sizeof(BtGattCharValue_t));
/*		BtGattCharValHandler_t *p = (BtGattCharValHandler_t*)pAttVal;

		s_BtGatEntryTbl[s_NbGattListEntry].ValHandler.RdHandler = p->RdHandler;
		s_BtGatEntryTbl[s_NbGattListEntry].ValHandler.WrHandler = p->WrHandler;
		s_BtGatEntryTbl[s_NbGattListEntry].ValHandler.pCtx = p->pCtx;*/

//		s_BtGatEntryTbl[s_NbGattListEntry].pCharVal = (BtGattCharValue_t*)pAttVal;
	}
	s_BtGattEntryTbl[s_NbGattListEntry].Hdl = s_NbGattListEntry + 1;

	s_NbGattListEntry++;

	return s_NbGattListEntry;
}

bool BtGattUpdate(uint16_t Hdl, void * const pAttVal, size_t Len)
{
	if (Hdl <= 0 || Hdl > s_NbGattListEntry)
	{
		return false;
	}

	BtGattListEntry_t *p = (BtGattListEntry_t*)&s_BtGattEntryTbl[Hdl - 1];

	if (p->TypeUuid.BaseIdx == 0)
	{
		switch (p->TypeUuid.Uuid)
		{
			case BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_GATT_DECLARATIONS_SECONDARY_SERVICE:
				memcpy(&p->SrvcDeclar, pAttVal, sizeof(BtGattSrvcDeclar_t));
				break;
			case BT_UUID_GATT_DECLARATIONS_INCLUDE:
				memcpy(&p->SrvcInc, pAttVal, sizeof(BtGattSrvcInclude_t));
				break;
			case BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC:
				memcpy(&p->CharDeclar, pAttVal, sizeof(BtGattCharDeclar_t));
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
//			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
//				break;
			case BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				p->Val32 = *(uint8_t*)pAttVal;
				break;
			case BT_UUID_GATT_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				p->pChar = (BtGattChar_t*)pAttVal;
//				memcpy(&p->CharVal, pAttVal, sizeof(BtGattCharValue_t));
		}
	}
	else
	{
		p->pChar = (BtGattChar_t*)pAttVal;
//		memcpy(&p->CharVal, pAttVal, sizeof(BtGattCharValue_t));
	}

	return true;
}

int BtGattGetListHandle(uint16_t StartHdl, uint16_t EndHdl, BtGattListEntry_t *pArr, int MaxEntry, uint16_t *pLastHdl)
{
	int idx = 0;

	for (int i = StartHdl; i < min(EndHdl + 1, s_NbGattListEntry) && idx < MaxEntry; i++)
	{
		memcpy(&pArr[idx], &s_BtGattEntryTbl[i - 1], sizeof(BtGattListEntry_t));
		idx++;
	}

	if (pLastHdl && s_NbGattListEntry > 0)
	{
		*pLastHdl = s_BtGattEntryTbl[s_NbGattListEntry - 1].Hdl;
	}

	return idx;
}

int BtGattGetListUuid(BtUuid16_t *pTypeUuid, uint16_t StartHdl, BtGattListEntry_t *pArr, int MaxEntry, uint16_t *pLastHdl)
{
	int idx = 0;

	for (int i = StartHdl - 1; i < s_NbGattListEntry && idx < MaxEntry; i++)
	{
		if (memcmp(&s_BtGattEntryTbl[i].TypeUuid, pTypeUuid, sizeof(BtUuid16_t)) == 0)
		{
			memcpy(&pArr[idx], &s_BtGattEntryTbl[i], sizeof(BtGattListEntry_t));
			idx++;
		}
	}

	if (pLastHdl && s_NbGattListEntry > 0)
	{
		*pLastHdl = s_BtGattEntryTbl[s_NbGattListEntry - 1].Hdl;
	}

	return idx;
}

bool BeGattFindEntryUuid(BtUuid16_t *pTypeUuid, uint16_t StartHdl, uint16_t EndHdl, BtGattListEntry_t *pEntry)
{
	for (int i = StartHdl - 1; i < EndHdl && i < s_NbGattListEntry; i++)
	{
		if (memcmp(&s_BtGattEntryTbl[i].TypeUuid, pTypeUuid, sizeof(BtUuid16_t)) == 0)
		{
			memcpy(pEntry, &s_BtGattEntryTbl[i], sizeof(BtGattListEntry_t));
			return true;
		}
	}

	return false;
}

bool BtGattGetEntryHandle(uint16_t Hdl, BtGattListEntry_t *pEntry)
{
	if (Hdl <= 0 || Hdl > s_NbGattListEntry)
	{
		return false;
	}

	memcpy(pEntry, &s_BtGattEntryTbl[Hdl - 1], sizeof(BtGattListEntry_t));

	return true;
}

size_t BtGattGetValue(BtGattListEntry_t *pEntry, uint8_t *pBuff)
{
	if (pBuff == nullptr)
	{
		return 0;
	}

	size_t len = 0;
	//uint8_t uuid128[16];

	if (pEntry->TypeUuid.BaseIdx == 0)
	{
		switch (pEntry->TypeUuid.Uuid)
		{
			case BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_GATT_DECLARATIONS_SECONDARY_SERVICE:
				if (pEntry->SrvcDeclar.Uuid.BaseIdx > 0)
				{
					BtUuidGetBase(pEntry->SrvcDeclar.Uuid.BaseIdx, pBuff);

					pBuff[12] = pEntry->SrvcDeclar.Uuid.Uuid16 & 0xFF;
					pBuff[13] = pEntry->SrvcDeclar.Uuid.Uuid16 >> 8;

					len = 16;
				}
				else
				{
					pBuff[0] = pEntry->SrvcDeclar.Uuid.Uuid16 & 0xFF;
					pBuff[1] = pEntry->SrvcDeclar.Uuid.Uuid16 >> 8;

					len = 2;
				}
				break;
			case BT_UUID_GATT_DECLARATIONS_INCLUDE:
				len = pEntry->SrvcInc.SrvcUuid.BaseIdx > 0 ? 20 : 6;
				break;
			case BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC:
				{
#if 1
					BtGattCharDeclarVal_t *p = (BtGattCharDeclarVal_t*)pBuff;

					p->Prop = pEntry->CharDeclar.Prop;
					p->ValHdl = pEntry->CharDeclar.ValHdl;
					if (pEntry->CharDeclar.Uuid.BaseIdx > 0)
					{
						BtUuidGetBase(pEntry->CharDeclar.Uuid.BaseIdx, p->Uuid.Uuid128);

						p->Uuid.Uuid128[12] = pEntry->CharDeclar.Uuid.Uuid16 & 0xFF;
						p->Uuid.Uuid128[13] = pEntry->CharDeclar.Uuid.Uuid16 >> 8;

						len = 19;
					}
					else
					{
						p->Uuid.Uuid16 = pEntry->CharDeclar.Uuid.Uuid16;
						len = 5;
					}

#else
				pBuff[0] = pEntry->Hdl & 0xFF;
				pBuff[1] = pEntry->Hdl >> 8;
				pBuff += 2;
				*pBuff++ = pEntry->CharDeclar.Prop;
				pBuff[0] = pEntry->CharDeclar.ValHdl & 0xFF;
				pBuff[1] = pEntry->CharDeclar.ValHdl >> 8;
				pBuff += 2;
				if (pEntry->CharDeclar.Uuid.BaseIdx > 0)
				{
					BtUuidGetBase(pEntry->CharDeclar.Uuid.BaseIdx, pBuff);

					pBuff[12] = pEntry->CharDeclar.Uuid.Uuid & 0xFF;
					pBuff[13] = pEntry->CharDeclar.Uuid.Uuid >> 8;

					len = 21;
				}
				else
				{
					pBuff[0] = pEntry->CharDeclar.Uuid.Uuid & 0xFF;
					pBuff[1] = pEntry->CharDeclar.Uuid.Uuid >> 8;

					len = 7;
				}
#endif
				}
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
//			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
//				if (pEntry->pVal)
//				{
//					strcpy((char*)pBuff, (char*)pEntry->pVal);
//					len = strlen((char*)pEntry->pVal);
//				}
//				break;
			case BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				pBuff[0] = pEntry->Val32 & 0xFF;
				pBuff[1] = (pEntry->Val32 >> 8) & 0xFF;
				len = 2;
				break;
			case BT_UUID_GATT_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
//			case BT_UUID_GATT_CHAR_SERVICE_CHANGED:
//				pBuff[0] = pEntry->Val32 & 0xFF;
//				pBuff[1] = (pEntry->Val32 >> 8) & 0xFF;
//				len = 2;
			default:
				if (pEntry->pChar->pValue)
				{
					memcpy(pBuff, pEntry->pChar->pValue, pEntry->pChar->ValueLen);
					len = pEntry->pChar->ValueLen;
				}

		}
	}
	else
	{
		if (pEntry->pChar->pValue)
		{
			memcpy(pBuff, pEntry->pChar->pValue, pEntry->pChar->ValueLen);
			len = pEntry->pChar->ValueLen;
		}
		/*
		if (pEntry->ValHandler.RdHandler)
		{
			len = pEntry->ValHandler.RdHandler(pEntry->Hdl, pBuff, 10, pEntry->ValHandler.pCtx);
		}
*/
//		len = pEntry->pCharVal->Len;
//		memcpy(pBuff, pEntry->pCharVal->pData, len);
	}

	return len;
}

size_t BtGattWriteValue(uint16_t Hdl, uint8_t *pBuff, size_t Len)
{
	BtGattListEntry_t *p = &s_BtGattEntryTbl[Hdl - 1];

	size_t len = 0;

	if (p->TypeUuid.BaseIdx == 0)
	{
		switch (p->TypeUuid.Uuid)
		{
			case BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_GATT_DECLARATIONS_SECONDARY_SERVICE:
				break;
			case BT_UUID_GATT_DECLARATIONS_INCLUDE:
				break;
			case BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				p->Val32 = *(uint16_t*)pBuff;
				len = 2;
				break;
			case BT_UUID_GATT_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				if (p->pChar->WrCB)
				{
//					len = p->CharVal.WrHandler(p->Hdl, pBuff, Len, p->CharVal.pCtx);
					p->pChar->WrCB((BtGattChar_t*)p->pChar, pBuff, 0, Len);
				}

		}
	}
	else
	{
		if (p->pChar->WrCB)
		{
//			len = p->CharVal.WrHandler(p->Hdl, pBuff, Len, p->CharVal.pCtx);
			p->pChar->WrCB((BtGattChar_t*)p->pChar, pBuff, 0, Len);
		}
	}

	return len;
}

BtGattListEntry_t *GetEntryTable(size_t *count)
{
	*count = s_NbGattListEntry;
	return s_BtGattEntryTbl;
}

__attribute__((weak)) bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar->ValHdl == BT_GATT_HANDLE_INVALID)
	{
		return false;
	}

	BtGattListEntry_t *p = &s_BtGattEntryTbl[pChar->ValHdl - 1];

	if (p->pChar->MaxDataLen < Len)
	{
		return false;
	}

	if (pVal != pChar->pValue)
	{
		memcpy(pChar->pValue, pVal, Len);
	}

	if (pChar->Property & BT_GATT_CHAR_PROP_VALEN)
	{
		p->pChar->ValueLen = Len;
	}

	return true;
}

bool isBtGattCharNotifyEnabled(BtGattChar_t *pChar)
{
	if (pChar->CccdHdl == BT_GATT_HANDLE_INVALID)
	{
		return false;
	}

	BtGattListEntry_t *p = &s_BtGattEntryTbl[pChar->CccdHdl - 1];

	if (p->Val32 & BT_GATT_CLIENT_CHAR_CONFIG_NOTIFICATION)
	{
		return true;
	}

	return false;
}
/*
bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (BtGattCharSetValue(pChar, pVal, Len) == false)
	{
		return false;
	}

	BtGattListEntry_t *p = &s_BtGattEntryTbl[pChar->CccdHdl - 1];

	if (p->Val32 & BT_GATT_CLIENT_CHAR_CONFIG_NOTIFICATION)
	{
		BtCtlrNotify(ConnHdl, pChar->ValHdl, pVal, Len);
	}

	return true;
}
*/
__attribute__((weak)) bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc, BtGattSrvcCfg_t const * const pCfg)
{
	bool retval = false;
	uint8_t baseidx = 0;

	// Add base UUID to internal list.
	if (pCfg->bCustom)
	{
		pSrvc->Uuid.BaseIdx = BtUuidAddBase(pCfg->UuidBase);
	}
	pSrvc->Uuid.Type = BT_UUID_TYPE_16;
	pSrvc->Uuid.Uuid16 = pCfg->UuidSrvc;

	BtGattListEntry_t *p = &s_BtGattEntryTbl[s_NbGattListEntry];
	s_NbGattListEntry++;

    p->TypeUuid = {0, BT_UUID_TYPE_16, BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE };
	p->SrvcDeclar.Uuid = pSrvc->Uuid;
	p->Hdl = s_NbGattListEntry;
	pSrvc->Hdl = p->Hdl;


	pSrvc->NbChar = pCfg->NbChar;
    pSrvc->pCharArray = pCfg->pCharArray;


    BtGattChar_t *c = pSrvc->pCharArray;


    for (int i = 0; i < pCfg->NbChar; i++, c++)
    {
    	s_NbGattListEntry++;
        p++;

        c->pSrvc = pSrvc;
    	c->BaseUuidIdx = pSrvc->Uuid.BaseIdx;

        // Characteristic Declaration
        p->TypeUuid = {0, BT_UUID_TYPE_16, BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC };
    	p->Hdl = s_NbGattListEntry;
    	p->CharDeclar = {(uint8_t)c->Property, (uint16_t)(s_NbGattListEntry + 1), {c->BaseUuidIdx, BT_UUID_TYPE_16, c->Uuid}};

    	c->ValHdl = BT_GATT_HANDLE_INVALID;
    	c->DescHdl = BT_GATT_HANDLE_INVALID;
    	c->CccdHdl = BT_GATT_HANDLE_INVALID;
    	c->SccdHdl = BT_GATT_HANDLE_INVALID;
		c->Hdl = p->Hdl;

    	s_NbGattListEntry++;
    	p++;

    	// Characteristic value
    	p->TypeUuid = {c->BaseUuidIdx, BT_UUID_TYPE_16, c->Uuid };
    	p->Hdl = s_NbGattListEntry;
    	//p->CharVal = { c->MaxDataLen, c->ValueLen, c->pValue, c->WrCB, c };
    	p->pChar = c;
    	c->ValHdl = s_NbGattListEntry;

    	c->bNotify = false;
        if (c->Property & (BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
        {
        	s_NbGattListEntry++;
            p++;

            // Characteristic Descriptor CCC
            p->TypeUuid = {0, BT_UUID_TYPE_16, BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION };
        	p->Hdl = s_NbGattListEntry;
        	p->Val32 = 0;

        	c->CccdHdl = s_NbGattListEntry;
        }

        if (c->pDesc)
        {
        	s_NbGattListEntry++;
            p++;

        	// Characteristic Description
        	p->TypeUuid = {0, BT_UUID_TYPE_16, BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION };
        	p->Hdl = s_NbGattListEntry;
//        	p->pVal = (void*)c->pDesc;
        	size_t l = strlen(c->pDesc);
        	//p->CharVal = { l, l, (void*)c->pDesc, c->WrCB, c };
        	p->pChar = c;
        	c->DescHdl = p->Hdl;
        }
    }

    BtGattInsertSrvcList(pSrvc);


/*
    for (int i = 0; i < BT_GATT_SRVC_MAX_COUNT; i++)
    {
    	if (s_BtGattSrvcTbl[i].bValid == false)
    	{
    		s_BtGattSrvcTbl[i].bValid = true;
    		s_BtGattSrvcTbl[i].pSrvc = pSrvc;
    		break;
    	}
    }
*/

	return true;
}

__attribute__((weak)) void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc)
{
	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		if (pSrvc->pCharArray[i].CccdHdl != BT_GATT_HANDLE_INVALID)
		{
			s_BtGattEntryTbl[s_NbGattListEntry].Val32 = 0;
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

/*
void BtGattServiceInit(BtGattSrvc_t * const pSrvc)
{
	BtGattSrvcAdd(pSrvc, &s_BtGattDftlSrvcCfg);
}
*/

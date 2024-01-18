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

//#include "coredev/uart.h"
//extern UART g_Uart;
/*
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
/*
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
*/
//alignas(4) static BtGattListEntry_t s_BtGattEntryTbl[BT_GATT_ENTRY_MAX_COUNT] = {0,};
//static int s_NbGattListEntry = 0;
//alignas(4) static BtGattSrvcEntry_t s_BtGattSrvcTbl[BT_GATT_SRVC_MAX_COUNT] = {{0,}, };
//static int s_NbGattSrvcEntry = 0;
static BtGattSrvc_t *s_pBtGattSrvcList = nullptr;
//static BtGattSrvc_t *s_pBtGattSrvcTail = nullptr;
/*static uint16_t s_GattMaxMtu = 247;

uint16_t BtGttGetMaxMtu(uint16_t MaxMtu)
{
	return s_GattMaxMtu;
}

uint16_t BtGattSetMaxMtu(uint16_t MaxMtu)
{
	if (MaxMtu > 27)
	{
		s_GattMaxMtu = MaxMtu;
	}

	return s_GattMaxMtu;
}
*/
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
	if (pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	int l = min((uint16_t)Len, pChar->MaxDataLen);

	memcpy(pChar->pValue, pVal, l);
	pChar->ValueLen = l;

	return true;
}

bool isBtGattCharNotifyEnabled(BtGattChar_t *pChar)
{
	if (pChar->CccdHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	return pChar->bNotify;
}

__attribute__((weak)) bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc, BtGattSrvcCfg_t const * const pCfg)
{
	bool retval = false;
	uint8_t baseidx = 0;

	// Add base UUID to internal list.
	if (pCfg->bCustom)
	{
		baseidx = BtUuidAddBase(pCfg->UuidBase);
	}

	pSrvc->Uuid = { baseidx, BT_UUID_TYPE_16, pCfg->UuidSrvc };

	BtUuid16_t typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DECLARATIONS_PRIMARY_SERVICE };

	int l = sizeof(BtAttSrvcDeclar_t);// + (pCfg->NbChar - 1) * sizeof(BtAttDBEntry_t*);

	BtAttDBEntry_t *srvcentry = BtAttDBAddEntry(&typeuuid, l);

	if (srvcentry == nullptr)
	{
		return false;
	}

	BtAttSrvcDeclar_t *srvcdec = (BtAttSrvcDeclar_t*) srvcentry->Data;

	srvcdec->Uuid = pSrvc->Uuid;
	srvcdec->pSrvc = pSrvc;
	//srvcdec->NbChar = pCfg->NbChar;

	pSrvc->Hdl = srvcentry->Hdl;
	pSrvc->NbChar = pCfg->NbChar;
    pSrvc->pCharArray = pCfg->pCharArray;

    BtGattChar_t *c = pSrvc->pCharArray;

	BtAttDBEntry_t *entry = nullptr;

    for (int i = 0; i < pCfg->NbChar; i++, c++)
    {
    	typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DECLARATIONS_CHARACTERISTIC };
    	l = sizeof(BtAttCharDeclar_t);

    	entry = BtAttDBAddEntry(&typeuuid, l);
    	if (entry == nullptr)
    	{
    		return false;
    	}

    	//srvcdec->pCharEntry[i] = entry;

    	BtAttCharDeclar_t *chardec = (BtAttCharDeclar_t*)entry->Data;

    	//chardec->Prop = (uint8_t)c->Property;
    	chardec->Uuid = {baseidx, BT_UUID_TYPE_16, c->Uuid};
    	chardec->pChar = c;


/*		if (c->BaseUuidIdx > 0)
		{
			BtUuidGetBase(c->BaseUuidIdx, chardec->Uuid.Uuid128);

			chardec->Uuid.Uuid128[12] = chardec->Uuid.Uuid16 & 0xFF;
			chardec->Uuid.Uuid128[13] = chardec->Uuid.Uuid16 >> 8;

			entry->DataLen = 19;
		}
		else
		{
			chardec->Uuid.Uuid16 = c->Uuid;
			entry->DataLen = 5;
		}
*/
    	c->ValHdl = BT_ATT_HANDLE_INVALID;
		c->DescHdl = BT_ATT_HANDLE_INVALID;
		c->CccdHdl = BT_ATT_HANDLE_INVALID;
		c->SccdHdl = BT_ATT_HANDLE_INVALID;
        c->pSrvc = pSrvc;
    	c->BaseUuidIdx = pSrvc->Uuid.BaseIdx;

    	// Characteristic value
    	typeuuid = {baseidx, BT_UUID_TYPE_16, c->Uuid };
    	entry = BtAttDBAddEntry(&typeuuid, c->MaxDataLen + sizeof(BtAttCharValue_t));
    	if (entry == nullptr)
    	{
    		return false;
    	}
    	BtAttCharValue_t *charval = (BtAttCharValue_t*)entry->Data;

    	charval->pChar = c;
    	//chardec->pChar->ValHdl = entry->Hdl;
    	c->ValHdl = entry->Hdl;//chardec->ValHdl;
    	//c->pData = charval;
    	c->pValue = charval->Data;

    	//charval->MaxDataLen = c->MaxDataLen;
    	//charval->DataLen = 0;
    	//charval->WrCB = c->WrCB;
    	//charval->TxCompleteCB = c->TxCompleteCB;

    	c->bNotify = false;
        if (c->Property & (BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
        {
            // Characteristic Descriptor CCC
            typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION };
            l = sizeof(BtDescClientCharConfig_t);
        	entry = BtAttDBAddEntry(&typeuuid, l);
        	if (entry == nullptr)
        	{
        		return false;
        	}

        	BtDescClientCharConfig_t *ccc = (BtDescClientCharConfig_t*)entry->Data;

        	ccc->pChar = c;
        	ccc->CccVal = 0;
        	//ccc->SetIndCB = c->SetIndCB;
        	//ccc->SetNtfCB = c->SetNotifCB;
    		c->CccdHdl = entry->Hdl;

        }

        if (c->pDesc)
        {
        	// Characteristic Description
        	typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION };
        	size_t l = sizeof(BtDescCharUserDesc_t);
        	entry = BtAttDBAddEntry(&typeuuid, l);
        	if (entry == nullptr)
        	{
        		return false;
        	}

        	BtDescCharUserDesc_t *dcud = (BtDescCharUserDesc_t*)entry->Data;

        	dcud->pChar = c;
        	//dcud->pDescStr = c->pDesc;
        	c->DescHdl = entry->Hdl;
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
		if (pSrvc->pCharArray[i].CccdHdl != BT_ATT_HANDLE_INVALID)
		{
			pSrvc->pCharArray[i].bNotify = false;
			pSrvc->pCharArray[i].bIndic = false;

			BtAttDBEntry_t *entry = BtAttDBFindHandle(pSrvc->pCharArray[i].CccdHdl);
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

void BtGattSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent)
{
	if (s_pBtGattSrvcList)
	{
		BtGattSrvc_t *p = s_pBtGattSrvcList;

		while (p)
		{
			for (int i = 0 ; i < p->NbChar; i++)
			{
				if (p->pCharArray[i].TxCompleteCB)
				{
					p->pCharArray[i].TxCompleteCB(&p->pCharArray[i], i);
				}
			}

			p = p->pNext;
		}
	}
}



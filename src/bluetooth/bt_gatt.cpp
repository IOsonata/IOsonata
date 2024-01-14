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
size_t BtGattReadAttValue(BtAttDBEntry_t *pEntry, uint16_t Offset, uint8_t *pBuff, uint16_t Len)
{
	if (pBuff == nullptr)
	{
		return 0;
	}

	size_t len = 0;

	if (pEntry->TypeUuid.BaseIdx == 0)
	{
		switch (pEntry->TypeUuid.Uuid)
		{
			case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
				{
					BtGattSrvcDeclar_t *p = (BtGattSrvcDeclar_t*)pEntry->Data;

					if (p->Uuid.BaseIdx > 0)
					{
						BtUuidGetBase(p->Uuid.BaseIdx, pBuff);

						pBuff[12] = p->Uuid.Uuid16 & 0xFF;
						pBuff[13] = p->Uuid.Uuid16 >> 8;

						len = 16;
					}
					else
					{
						pBuff[0] = p->Uuid.Uuid16 & 0xFF;
						pBuff[1] = p->Uuid.Uuid16 >> 8;

						len = 2;
					}
				}
				break;
			case BT_UUID_DECLARATIONS_INCLUDE:
				{
					BtGattSrvcInclude_t *p = (BtGattSrvcInclude_t*)pEntry->Data;
					len = p->SrvcUuid.BaseIdx > 0 ? 20 : 6;
				}

				break;
			case BT_UUID_DECLARATIONS_CHARACTERISTIC:
				{
					BtGattCharDeclar_t *p = (BtGattCharDeclar_t*)pEntry->Data;
					//len = pEntry->DataLen;
					//memcpy(pBuff, pEntry->Data, len);
					pBuff[0] = p->Prop;
					pBuff[1] = p->ValHdl & 0xFF;
					pBuff[2] = (p->ValHdl >> 8)& 0xFF;

					BtUuidVal_t *u = (BtUuidVal_t*)&pBuff[3];
					if (p->Uuid.BaseIdx > 0)
					{
						BtUuidGetBase(p->Uuid.BaseIdx, u->Uuid128);

						u->Uuid128[12] = p->Uuid.Uuid16 & 0xFF;
						u->Uuid128[13] = p->Uuid.Uuid16 >> 8;

						len = 19;
					}
					else
					{
						u->Uuid16 = p->Uuid.Uuid16;
						len = 5;
					}
				}
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
//				if (pEntry->pVal)
//				{
//					strcpy((char*)pBuff, (char*)pEntry->pVal);
//					len = strlen((char*)pEntry->pVal);
//				}
				break;
			case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				{
					BtGattCharClientConfig_t *d = (BtGattCharClientConfig_t*)pEntry->Data;

					*(uint16_t*)pBuff = d->CccVal;
					len = 2;
				}
				break;
			case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
//			case BT_UUID_GATT_CHAR_SERVICE_CHANGED:
//				pBuff[0] = pEntry->Val32 & 0xFF;
//				pBuff[1] = (pEntry->Val32 >> 8) & 0xFF;
//				len = 2;
			default:
				{
					BtGattCharValue_t *p = (BtGattCharValue_t*)pEntry->Data;

					size_t l = min(p->pChar->ValueLen - Offset, BtAttGetMtu());
					memcpy(pBuff, p->Data + Offset, l);
					len = l;
				}

		}
	}
	else
	{
		BtGattCharValue_t *p = (BtGattCharValue_t*)pEntry->Data;

		size_t l = min(p->pChar->ValueLen - Offset, BtAttGetMtu());
		memcpy(pBuff, p->Data + Offset, l);
		len = l;
	}

	return len;
}

//size_t BtGattWriteValue(uint16_t Hdl, uint8_t *pBuff, size_t Len)
size_t BtGattWriteAttValue(BtAttDBEntry_t *pEntry, uint16_t Offset, uint8_t *pData, uint16_t Len)
{
	size_t len = 0;
	//BtAttDBEntry_t *entry = BtAttDBFindHandle(Hdl);//&s_BtGattEntryTbl[Hdl - 1];

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
					//g_Uart.printf("BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION\r\n");

					BtGattCharClientConfig_t *p = (BtGattCharClientConfig_t*)pEntry->Data;

					p->CccVal = *(uint16_t*)pData;
					p->pChar->bNotify = p->CccVal & BT_GATT_CLIENT_CHAR_CONFIG_NOTIFICATION ? true: false;
					if (p->pChar->SetNotifCB)
					{
						p->pChar->SetNotifCB(p->pChar, p->pChar->bNotify);
					}
//					g_Uart.printf("CccdHdl : %x\r\n", p->pChar->CccdHdl);
				}
				break;
			case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				{
					BtGattCharValue_t *p = (BtGattCharValue_t*)pEntry->Data;

					p->pChar->ValueLen = min(Len, p->pChar->MaxDataLen);// - sizeof(BtGattCharValue_t));
					memcpy(p->Data, pData, p->pChar->ValueLen);

					if (p->pChar->WrCB)
					{
	//					len = p->CharVal.WrHandler(p->Hdl, pBuff, Len, p->CharVal.pCtx);
						p->pChar->WrCB(p->pChar, pData, 0, Len);
					}
				}

		}
	}
	else
	{
		BtGattCharValue_t *p = (BtGattCharValue_t*)pEntry->Data;

		p->pChar->ValueLen = min(Len,  p->pChar->MaxDataLen);//- sizeof(BtGattCharValue_t));
		memcpy(p->Data, pData, p->pChar->ValueLen);

		if (p->pChar->WrCB)
		{
//					len = p->CharVal.WrHandler(p->Hdl, pBuff, Len, p->CharVal.pCtx);
			p->pChar->WrCB(p->pChar, pData, 0, Len);
		}
	}

	return len;
}

__attribute__((weak)) bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar->ValHdl == BT_GATT_HANDLE_INVALID)
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
	if (pChar->CccdHdl == BT_GATT_HANDLE_INVALID)
	{
		return false;
	}

	return pChar->bNotify;
}

__attribute__((weak)) bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc, BtGattSrvcCfg_t const * const pCfg)
{
	bool retval = false;
	uint8_t baseidx = 0;
	BtAttDBEntry_t *entry = nullptr;

	// Add base UUID to internal list.
	if (pCfg->bCustom)
	{
		pSrvc->Uuid.BaseIdx = BtUuidAddBase(pCfg->UuidBase);
	}
	pSrvc->Uuid.Type = BT_UUID_TYPE_16;
	pSrvc->Uuid.Uuid16 = pCfg->UuidSrvc;

	BtUuid16_t typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DECLARATIONS_PRIMARY_SERVICE };
	int l = sizeof(BtGattSrvcDeclar_t);

	entry = BtAttDBAddEntry(&typeuuid, l);

	if (entry == nullptr)
	{
		return false;
	}

	BtGattSrvcDeclar_t *srvcdec = (BtGattSrvcDeclar_t*) entry->Data;
	srvcdec->Uuid = pSrvc->Uuid;
	entry->DataLen = l;

	pSrvc->Hdl = entry->Hdl;
	pSrvc->NbChar = pCfg->NbChar;
    pSrvc->pCharArray = pCfg->pCharArray;

    BtGattChar_t *c = pSrvc->pCharArray;


    for (int i = 0; i < pCfg->NbChar; i++, c++)
    {
    	typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DECLARATIONS_CHARACTERISTIC };
    	l = sizeof(BtGattCharDeclar_t);

    	entry = BtAttDBAddEntry(&typeuuid, l);
    	if (entry == nullptr)
    	{
    		return false;
    	}

    	BtGattCharDeclar_t *chardec = (BtGattCharDeclar_t*)entry->Data;
    	chardec->Prop = (uint8_t)c->Property;
    	chardec->Uuid = {c->BaseUuidIdx, BT_UUID_TYPE_16, c->Uuid};
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
    	c->ValHdl = BT_GATT_HANDLE_INVALID;
		c->DescHdl = BT_GATT_HANDLE_INVALID;
		c->CccdHdl = BT_GATT_HANDLE_INVALID;
		c->SccdHdl = BT_GATT_HANDLE_INVALID;
        c->pSrvc = pSrvc;
    	c->BaseUuidIdx = pSrvc->Uuid.BaseIdx;

    	// Characteristic value
    	typeuuid = {c->BaseUuidIdx, BT_UUID_TYPE_16, c->Uuid };
    	entry = BtAttDBAddEntry(&typeuuid, c->MaxDataLen + sizeof(BtGattCharValue_t));
    	if (entry == nullptr)
    	{
    		return false;
    	}
    	BtGattCharValue_t *charval = (BtGattCharValue_t*)entry->Data;

    	chardec->ValHdl = entry->Hdl;
    	c->ValHdl = chardec->ValHdl;
    	//c->pData = charval;
    	c->pValue = charval->Data;

    	//charval->MaxDataLen = c->MaxDataLen;
    	//charval->DataLen = 0;
    	charval->pChar = c;
    	//charval->WrCB = c->WrCB;

    	c->bNotify = false;
        if (c->Property & (BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
        {
            // Characteristic Descriptor CCC
            typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION };
            l = sizeof(BtGattCharClientConfig_t);
        	entry = BtAttDBAddEntry(&typeuuid, l);
        	if (entry == nullptr)
        	{
        		return false;
        	}

        	BtGattCharClientConfig_t *ccc = (BtGattCharClientConfig_t*)entry->Data;

        	ccc->pChar = c;
        	ccc->CccVal = 0;
        	ccc->SetIndCB = c->SetIndCB;
        	ccc->SetNtfCB = c->SetNotifCB;
    		c->CccdHdl = entry->Hdl;

        }

        if (c->pDesc)
        {
        	// Characteristic Description
        	typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION };
        	size_t l = sizeof(BtGattDescCharUserDesc_t);
        	entry = BtAttDBAddEntry(&typeuuid, l);
        	if (entry == nullptr)
        	{
        		return false;
        	}

        	BtGattDescCharUserDesc_t *dcud = (BtGattDescCharUserDesc_t*)entry->Data;

        	dcud->pChar = c;
        	dcud->pDescStr = c->pDesc;
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
		if (pSrvc->pCharArray[i].CccdHdl != BT_GATT_HANDLE_INVALID)
		{
			pSrvc->pCharArray[i].bNotify = false;
			pSrvc->pCharArray[i].bIndic = false;

			BtAttDBEntry_t *entry = BtAttDBFindHandle(pSrvc->pCharArray[i].CccdHdl);
			if (entry)
			{
				BtGattCharClientConfig_t *p = (BtGattCharClientConfig_t*)entry->Data;
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

/*
void BtGattServiceInit(BtGattSrvc_t * const pSrvc)
{
	BtGattSrvcAdd(pSrvc, &s_BtGattDftlSrvcCfg);
}
*/
#if 0
uint32_t BtGattProcessReq(uint16_t ConnHdl, BtAttReqRsp_t * const pReqAtt, int ReqLen, BtAttReqRsp_t * const pRspAtt)
{
	uint32_t retval = 0;

//	g_Uart.printf("ATT OpCode %x, L2Cap len %d\n", pReqAtt->OpCode, ReqLen);

	switch (pReqAtt->OpCode)
	{
		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:
			{
				BtAttExchgMtuReqRsp_t *req = (BtAttExchgMtuReqRsp_t*)&pReqAtt->ExchgMtuReqRsp;

//				g_Uart.printf("ATT_EXCHANGE_MTU_REQ:\r\n");
//				g_Uart.printf("RxMtu %d %d\r\n", pReqAtt->ExchgMtuReqRsp.RxMtu, s_AttMaxMtu);

				if (pReqAtt->ExchgMtuReqRsp.RxMtu < BT_ATT_MTU_MIN)
				{
					retval = BtAttError(pRspAtt, ConnHdl, BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ, BT_ATT_ERROR_INVALID_ATT_VALUE);
					break;
				}
				retval = sizeof(BtAttExchgMtuReqRsp_t) + 1;
				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP;
				pRspAtt->ExchgMtuReqRsp.RxMtu = BtAttSetMtu(pReqAtt->ExchgMtuReqRsp.RxMtu);

				//g_Uart.printf("MTU : %d\r\n", s_AttMaxMtu);
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ:
			{
				BtAttFindInfoReq_t *req = (BtAttFindInfoReq_t*)&pReqAtt->FindInfoReq;

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP;

				int l = 0;
				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->StartHdl);

				if (entry)
				{
					if (entry->TypeUuid.BaseIdx > 0)
					{
						uint8_t uuid128[16];

						BtUuidGetBase(entry->TypeUuid.BaseIdx, uuid128);

						pRspAtt->FindInfoRsp.Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128;

					//	for (int i = 0; i < c; i++)
						{
							pRspAtt->FindInfoRsp.HdlUuid128[0].Hdl = entry->Hdl;
							uuid128[12] = entry->TypeUuid.Uuid & 0xff;
							uuid128[13] = entry->TypeUuid.Uuid >> 8;
							memcpy(pRspAtt->FindInfoRsp.HdlUuid128[0].Uuid, uuid128, 16);

//								g_Uart.printf("HDL : %d, ", rsp->HdlUuid128[0].Hdl);
//
//								for (int j = 0; j < 16; j++)
//								{
//									g_Uart.printf("%02x ", rsp->HdlUuid128[0].Uuid[j]);
//								}
//								g_Uart.printf("\r\n");
						}

						l = sizeof(BtAttHdlUuid128_t);
					}
					else
					{
						pRspAtt->FindInfoRsp.Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;

						//for (int i = 0; i < c; i++)
						{
							pRspAtt->FindInfoRsp.HdlUuid16[0].Hdl = entry->Hdl;
							pRspAtt->FindInfoRsp.HdlUuid16[0].Uuid = entry->TypeUuid.Uuid;

//								g_Uart.printf("HDL : %d, Uuid16 : 0x%04x\r\n", rsp->HdlUuid16[0].Hdl, rsp->HdlUuid16[0].Uuid);

						}

						l = sizeof(BtAttHdlUuid16_t);
					}

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
				//g_Uart.printf("BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:\r\n");
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
			{
//				g_Uart.printf("ATT_READ_BY_TYPE_REQ:\r\n");

				// Only the attributes with attribute handles between and including
				// the Starting Handle and the Ending Handle with the attribute type
				// that is the same as the Attribute Type given will be returned. To
				// search through all attributes, the starting handle shall be set to
				// 0x0001 and the ending handle shall be set to 0xFFFF.

				BtAttReadByTypeReq_t *req = (BtAttReadByTypeReq_t*)&pReqAtt->ReadByTypeReq;

				if (req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

//				BtAttReadByTypeRsp_t *rsp = (BtAttReadByTypeRsp_t*)&l2pdu->Att;
//				BtAtt_t *rsp = (BtAtt_t*)&l2pdu->Att;

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP;

				uint8_t *p = (uint8_t*)pRspAtt->ReadByTypeRsp.Data;

//				g_Uart.printf("sHdl: %x, eHdl: %x, Type: %x\r\n", req->StartHdl, req->EndHdl, req->Uuid.Uuid16);
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};

				BtAttDBEntry_t *entry = BtAttDBFindUuidRange(&uid16, req->StartHdl, req->EndHdl);

				if (entry)
				{
					p[0] = entry->Hdl & 0xFF;
					p[1] = entry->Hdl >> 8;
					p +=2;
					pRspAtt->ReadByTypeRsp.Len = BtGattReadAttValue(entry, 0, p, 20) + 2;
//					g_Uart.printf("Att Val: ");

//					for (int j = 0; j < rsp->Len; j++)
//					{
//						g_Uart.printf("%x ", rsp->Data[j]);
//					}
//					g_Uart.printf("\r\n");

					retval = 2 + pRspAtt->ReadByTypeRsp.Len;//rsp->Len * c;
					break;
				}
				retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
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
					retval = BtGattReadAttValue(entry, 0, pRspAtt->ReadRsp.Data, 20)  + 1;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_READ_REQ, BT_ATT_ERROR_INVALID_HANDLE);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ:
			{
				//BtAttReadBlobReq_t *req = (BtAttReadBlobReq_t*)&pReqAtt->BlobReq;

				//g_Uart.printf("BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ:\r\n");
//				g_Uart.printf("Hdl: %x, offset: %x\r\n", req->Hdl, req->Offset);

				//BtAtt_t *rsp = (BtAtt_t*)&l2pdu->Att;

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_REQ_BLOB_RSP;
				BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->ReadBlobReq.Hdl);

				if (entry)
				{
					retval = BtGattReadAttValue(entry, pReqAtt->ReadBlobReq.Offset, pRspAtt->ReadBlobRsp.Data, 20) + 1;
				}
				else
				{
					retval = BtAttError(pRspAtt, pReqAtt->ReadBlobReq.Hdl, BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ, BT_ATT_ERROR_INVALID_HANDLE);
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
				int nhdl = (ReqLen - 1) >> 1;
				uint8_t *p = pRspAtt->ReadMultipleRsp.Data;
				retval = 1;
				for (int i = 0; i < nhdl; i++)
				{
					BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->ReadMultipleReq.Hdl[i]);
					if (entry == nullptr)
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadMultipleReq.Hdl[i], BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
						break;
					}
					int l = BtGattReadAttValue(entry, 0, p, 20);
					p += l;
					retval += l;
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
			{
				BtAttReadByGroupTypeReq_t *req = (BtAttReadByGroupTypeReq_t*)&pReqAtt->ReadByGroupTypeReq;

				if (req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP;

//				g_Uart.printf("ATT_READ_BY_GROUP_TYPE_REQ:\r\n");
//				g_Uart.printf("sHdl:%x, eHdl:%x, Uuid: %x\r\n", req->StartHdl, req->EndHdl, req->Uuid.Uuid16);

				uint8_t *p = (uint8_t*)pRspAtt->ReadByGroupTypeRsp.Data;
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};
				BtAttHdlRange_t *hu = (BtAttHdlRange_t*)p;
				int l = 0;

				pRspAtt->ReadByGroupTypeRsp.Len = 0;

//					g_Uart.printf("%x\r\n", req->Uid.Uuid16);

				hu->StartHdl = req->StartHdl;
				hu->EndHdl = req->EndHdl;
				BtAttDBEntry_t *entry = BtAttDBFindHdlRange(&uid16, &hu->StartHdl, &hu->EndHdl);

				int baseidx = entry->TypeUuid.BaseIdx;

				if (entry)
				{
					baseidx = entry->TypeUuid.BaseIdx;
				}
				uint8_t prevlen = 0;

				while (entry && l < 20)
				{
					if (entry->Hdl >= req->StartHdl && entry->Hdl <= req->EndHdl && baseidx == entry->TypeUuid.BaseIdx)
					{
//						g_Uart.printf("shdl : %x, ehdl : %x\r\n", hu->StartHdl, hu->EndHdl);
						p += sizeof(BtAttHdlRange_t);
						pRspAtt->ReadByGroupTypeRsp.Len = BtGattReadAttValue(entry, 0, p, 20);
/*
						g_Uart.printf("Val Len : %d\r\n", pRspAtt->ReadByGroupTypeRsp.Len);
						for (int i=0; i< pRspAtt->ReadByGroupTypeRsp.Len; i++)
						{
							g_Uart.printf("%02x ", p[i]);
						}
						g_Uart.printf("\r\n");
*/
						if (prevlen > 0 && prevlen != pRspAtt->ReadByGroupTypeRsp.Len)
						{
							pRspAtt->ReadByGroupTypeRsp.Len = prevlen;
							break;
						}
						else
						{
							prevlen = pRspAtt->ReadByGroupTypeRsp.Len;
						}
						p += pRspAtt->ReadByGroupTypeRsp.Len;
						l += 4 + pRspAtt->ReadByGroupTypeRsp.Len;
						((BtAttHdlRange_t*)p)->StartHdl = hu->EndHdl;
						hu = (BtAttHdlRange_t*)p;
						hu->EndHdl = req->EndHdl;
//						g_Uart.printf("Len : %d\r\n", l);
					}
					else
					{
						break;
					}
					entry = BtAttDBFindHdlRange(&uid16, &hu->StartHdl, &hu->EndHdl);
				}

				if (l > 0)
				{
					pRspAtt->ReadByGroupTypeRsp.Len += 4;

					retval = l + 2;
//					g_Uart.printf("retval %d\r\n", retval);
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

				//g_Uart.printf("BT_ATT_OPCODE_ATT_WRITE_REQ:\r\n");
				//g_Uart.printf("Hdl: %x\r\n", req->Hdl);

				if (req->Hdl < 1)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
				//BtAttWriteRsp_t *rsp = (BtAttWriteRsp_t*)&l2pdu->Att;
					size_t l = ReqLen;

					l = BtGattWriteAttValue(entry, 0, req->Data, l);

					pRspAtt->OpCode = BT_ATT_OPCODE_ATT_WRITE_RSP;

					retval = 1;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);

				}
			}
			break;
		case BT_ATT_OPCODE_ATT_CMD:		// Write without response
			{
				BtAttSignedWriteCmd_t *req = (BtAttSignedWriteCmd_t*)&pReqAtt->SignedWriteCmd;

//				g_Uart.printf("ATT_CMD:\r\n");
//				g_Uart.printf("Hdl: %x, data len: %d\r\n", req->Hdl, pRcvPdu->Hdr.Len -  3);

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
					size_t l = ReqLen;

					l = BtGattWriteAttValue(entry, 0, req->Data, l - 3);

					retval = 0;
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:
			break;
		case BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD:
			break;
	}

	return retval;
}
#endif


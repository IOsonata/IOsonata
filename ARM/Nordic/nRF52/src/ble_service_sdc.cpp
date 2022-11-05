/**-------------------------------------------------------------------------
@file	ble_service_sdc.c

@brief	Implement Bluetooth LE service and characteristic

Implementation allow the creation of generic custom Bluetooth Smart service
with multiple user defined characteristics.

This implementation is to be used with Nordic nrfxlib Softdevice Controller

@author	Hoang Nguyen Hoan
@date	Oct. 15, 2022

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
#include <stdint.h>
#include <string.h>


#include "bluetooth/ble_srvc.h"
#include "bluetooth/bt_gatt.h"


static size_t BleSrvcCharRdHandler(uint16_t Hdl, void *pBuff, size_t Len, void *pCtx)
{
	BleSrvcChar_t *p = (BleSrvcChar_t*)pCtx;


	return Len;
}

static size_t BleSrvcCharWrHandler(uint16_t Hdl, void *pBuff, size_t Len, void *pCtx)
{
	BleSrvcChar_t *p = (BleSrvcChar_t*)pCtx;

	if (p->WrCB)
	{
		p->WrCB(p->pSrvc, (uint8_t*)pBuff, 0, Len);
	}

	return Len;
}

uint32_t BleSrvcCharNotify(BleSrvc_t *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen)
{
	return 0;
}

bool BleSrvcAddChar(BleSrvc_t *pSrvc, BleSrvcChar_t *pChar, uint32_t SecType)
{
	if (pChar == NULL)
	{
		return false;
	}

	pChar->pSrvc = pSrvc;
	pChar->BaseUuidIdx = pSrvc->Uuid.BaseIdx;


	BtGattCharDeclar_t gatt = {(uint8_t)pChar->Property, 0, {pChar->BaseUuidIdx, BT_UUID_TYPE_16, pChar->Uuid}};

	BtUuid16_t TypeUuid = { 0, BT_UUID_TYPE_16, BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC };

	pChar->Hdl = BtGattRegister(&TypeUuid, &gatt);
	pChar->ValHdl = gatt.ValHdl;

	TypeUuid.BaseIdx = pChar->BaseUuidIdx;
	TypeUuid.Uuid = pChar->Uuid;

//	BtGattCharValue_t handler = { pChar->MaxDataLen, pChar->ValueLen, pChar->pValue, BleSrvcCharWrHandler, pChar };

//	pChar->ValHdl = BtGattRegister(&TypeUuid, &handler);

	pChar->bNotify = false;
    if (pChar->Property & (BLESRVC_CHAR_PROP_NOTIFY | BLESRVC_CHAR_PROP_INDICATE))
    {
    	TypeUuid.Uuid = BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION;

    	uint16_t x = pChar->Property >> 4;
    	pChar->CccdHdl = BtGattRegister(&TypeUuid, &x);
    }

    return true;
}

uint32_t BleSrvcInit(BleSrvc_t *pSrvc, const BleSrvcCfg_t *pCfg)
{
	uint32_t   err;
	uint8_t baseidx = 0;
//	BtGattCharValue_t charval;

	// Initialize service structure
	pSrvc->ConnHdl  = -1;

	// Add base UUID to internal list.
	if (pCfg->bCustom)
	{
		pSrvc->Uuid.BaseIdx = BtUuidAddBase(pCfg->UuidBase);
	}
	pSrvc->Uuid.Type = BT_UUID_TYPE_16;
	pSrvc->Uuid.Uuid16 = pCfg->UuidSrvc;

//	BtUuid16_t uid16 = { baseidx, BT_UUID_TYPE_16, pCfg->UuidSvc};

	BtUuid16_t TypeUuid = { 0, BT_UUID_TYPE_16, BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE };

	pSrvc->Hdl = BtGattRegister(&TypeUuid, &pSrvc->Uuid);

	pSrvc->NbChar = pCfg->NbChar;
    pSrvc->pCharArray = pCfg->pCharArray;

    for (int i = 0; i < pCfg->NbChar; i++)
    {
    	BleSrvcAddChar(pSrvc, &pSrvc->pCharArray[i], 0);
    }

	return 0;

}

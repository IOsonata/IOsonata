/**-------------------------------------------------------------------------
@file	bt_gatt.h

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
#ifndef __BT_GATT_H__
#define __BT_GATT_H__

#include <inttypes.h>

#include "bt_uuid.h"

typedef struct __Bt_Gatt_Srvc {
	BtUuid16_t Uuid;			//!< Service UUID
	uint16_t Hdl;				//!< Service handle
} BtGattSrvc_t;

typedef struct __Bt_Gatt_List_Entry {
	uint16_t Hdl;					//!< Handle
	uint16_t GattUuid;				//!< Gatt UUID
	BtUuid16_t Uuid;				//!< Entry UUID
} BtGattListEntry_t;

#ifdef __cplusplus
extern "C" {
#endif

uint16_t BtGattRegister(uint16_t GatUuid, BtUuid16_t *pUuid);
//uint16_t BtGattDeclarPrimSrvc(BtUuid16_t *pUuid);
//uint16_t BtGattDeclarSecondSrvc(BtUuid16_t *pUuid);
//uint16_t BtGattDeclarChar(BtUuid16_t *pUuid);

int BtGattGetList(uint16_t GattUuid, BtGattListEntry_t *pArr, int MaxEntry);
bool BtGattGetEntryHandle(uint16_t Hdl, BtGattListEntry_t *pArr);

#ifdef __cplusplus
}
#endif

#endif // __BT_GATT_H__


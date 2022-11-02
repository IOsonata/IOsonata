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

#define BT_GATT_HANDLE_INVALID				-1

#pragma pack(push,1)

// Service attribute : type UUID 0x2800 Primary, 0x2801 Secondary
typedef struct __Bt_Gatt_Srvc_Declar {
	BtUuid16_t Uuid;			//!< Service UUID
} BtGattSrvcDeclar_t;

// Service include attribute : 0x2802
typedef struct __Bt_Gatt_Srvc_Include {
	uint16_t SrvcHdl;			//!< Service attribute handle
	uint16_t EndGrpHdl;			//!< End group handle
	BtUuid16_t SrvcUuid;		//!< Service UUID
} BtGattSrvcInclude_t;

#define BT_GATT_CHAR_PROP_BROADCAST			1	//!< If set, permits broadcasts of the
												//!< Characteristic Value using Server
												//! Characteristic Configuration Descriptor.
												//! If set, the Server Characteristic Configuration
												//! Descriptor shall exist.
#define BT_GATT_CHAR_PROP_READ				2	//!< If set, permits reads of the Characteristic Value
#define BT_GATT_CHAR_PROP_WRITE_WORESP		4	//!<
#define BT_GATT_CHAR_PROP_WRITE				8
#define BT_GATT_CHAR_PROP_NOTIFY			0x10
#define BT_GATT_CHAR_PROP_INDICATE			0x20
#define BT_GATT_CHAR_PROP_AUTH_SIGNED_WR	0x40
#define BT_GATT_CHAR_PROP_EXT_PROP			0x80


// Characteristic declaration attribute : type UUID 0x2803
typedef struct __Bt_Gatt_Char_Declar {
	uint8_t Prop;				//!< Orable properties
	uint16_t ValHdl;			//!< Value handle
	BtUuid16_t Uuid;			//!< Characteristic UUID
} BtGattCharDeclar_t;

#pragma pack(pop)

#define BT_GATT_CHAR_EXT_PROP_RELIABLE_WRITE	1	//!< Reliable write using procedure Section 4.9.5
#define BT_GATT_CHAR_EXT_PROP_WRITABLE_AUX		2	//!< Write to descriptor defined in Section 3.3.3.2

// Characteristic extended property attribute : type UUID 0x2900
typedef uint8_t		BtGattCharExtProp_t;

// Characteristic user description attribute : Type UUID 0x2901
// Attribute value : UTF-8
typedef char *		BtGattCharUserDesc_t;

#define BT_GATT_CLIENT_CHAR_CONFIG_NOTIFICATION		1
#define BT_GATT_CLIENT_CHAR_CONFIG_INDICATION		2

// Client characteristic configuration declaration attribute UUID 0x2902
typedef uint8_t BtGatClientCharConfig_t;

#define BT_GATT_SERVER_CHAR_CONFIG_BROADCAST		1

// Server characteristic configuration declaration attribute UUID 0x2903
typedef uint8_t		BtGattServerCharConfig_t;

typedef size_t (*BtGattAttRdHandler_t)(uint16_t Hdl, void *pBuff, size_t Len, void *pCtx);
typedef size_t (*BtGattAttWrHandler_t)(uint16_t Hdl, void *pData, size_t Len, void *pCtx);

#pragma pack(push,4)

/// NOTE: Variable length
typedef struct __Bt_Gatt_Char_Value {
	size_t MaxLen;			//!< Max data buffer length
	size_t Len;				//!< Length of actual data
	uint8_t *pData;			//!< Data buffer
	BtGattAttWrHandler_t WrHandler;
	void *pCtx;
} BtGattCharValue_t;

typedef struct __Bt_Gatt_List_Entry {
	uint16_t Hdl;				//!< Attribute handle
	BtUuid16_t TypeUuid;		//!< Attribute type UUID
	uint32_t Permission;		//!< Attribute Permission
	union {						//!< Attribute value
		uint32_t Val32;
		BtUuid_t Uuid;
		//BtGattCharValHandler_t ValHandler;
		BtGattCharValue_t CharVal;
		BtGattSrvcDeclar_t SrvcDeclar;	//!< Service declaration value
		BtGattSrvcInclude_t SrvcInc;	//!< Service definition value
		BtGattCharDeclar_t CharDeclar;	//!< Characteristic declaration value
	};
} BtGattListEntry_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

uint16_t BtGattRegister(BtUuid16_t *pTypeUuid, void *pAttVal);
bool BtGattUpdate(uint16_t Hdl, void *pAttVal);
int BtGattGetListHandle(uint16_t StartHdl, uint16_t EndHdl, BtGattListEntry_t *pArr, int MaxEntry, uint16_t *pLastHdl);
int BtGattGetListUuid(BtUuid16_t *pTypeUuid, uint16_t StartHdl, BtGattListEntry_t *pArr, int MaxEntry, uint16_t *pLastHdl);
bool BtGattGetEntryHandle(uint16_t Hdl, BtGattListEntry_t *pArr);
size_t BtGattGetValue(BtGattListEntry_t *pEntry, uint8_t *pBuff);
size_t BtGattWriteValue(uint16_t Hdl, uint8_t *pBuff, size_t Len);

#ifdef __cplusplus
}
#endif

#endif // __BT_GATT_H__


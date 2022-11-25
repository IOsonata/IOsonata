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
#include <stddef.h>

#include "bluetooth/bt_uuid.h"

#define BT_GATT_HANDLE_INVALID				0xFFFFU

#pragma pack(push,1)

// Service attribute : type UUID 0x2800 Primary, 0x2801 Secondary
typedef struct __Bt_Gatt_Srvc_Declar {
	BtUuid_t Uuid;				//!< Service UUID
} BtGattSrvcDeclar_t;

// Service include attribute : 0x2802
typedef struct __Bt_Gatt_Srvc_Include {
	uint16_t SrvcHdl;			//!< Service attribute handle
	uint16_t EndGrpHdl;			//!< End group handle
	BtUuid_t SrvcUuid;			//!< Service UUID
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
#define BT_GATT_CHAR_PROP_AUTH_SIGNED		0x40
#define BT_GATT_CHAR_PROP_EXT_PROP			0x80
#define BT_GATT_CHAR_PROP_VALEN				0x8000

// Characteristic declaration attribute : type UUID 0x2803
typedef struct __Bt_Gatt_Char_Declar {
	uint8_t Prop;				//!< Orable properties
	uint16_t ValHdl;			//!< Value handle
	BtUuid_t Uuid;				//!< Characteristic UUID
} BtGattCharDeclar_t;

// Characteristic declaration attribute value : type UUID 0x2803
typedef struct __Bt_Gatt_Char_Declar_Val {
	uint8_t Prop;				//!< Orable properties
	uint16_t ValHdl;			//!< Value handle
	BtUuidVal_t Uuid;			//!< Characteristic UUID
} BtGattCharDeclarVal_t;

typedef struct __Bt_Gatt_Char_Srvc_Changed {
	uint16_t StartHdl;			//!< Start service handle
	uint16_t EndHdl;			//!< End service handle
} BtGattCharSrvcChanged_t;

typedef struct __Bt_Gatt_Char_Prefered_Conn_Params {
	uint16_t IntervalMin;		//!< Min connection interval in 1.25ms counts
	uint16_t IntervalMax;		//!< Max connection interval in 1.25ms counts
	uint16_t Latency;			//!< Peripheral latency
	uint16_t Timeout;			//!< Supervision timeout in 10ms count
} BtGattPreferedConnParams_t;

typedef struct __Bt_Gatt_Char_Notify {
	uint16_t ValHdl;			//!< Characteristic value handle
	uint8_t Data[1];			//!< Characteristic attribute value
} BtGattCharNotify_t;

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

//typedef size_t (*BtGattAttRdHandler_t)(uint16_t Hdl, void *pBuff, size_t Len, void *pCtx);
//typedef size_t (*BtGattAttWrHandler_t)(uint16_t Hdl, void *pData, size_t Len, void *pCtx);

typedef struct __Bt_Gatt_Characteristic		BtGattChar_t;
typedef struct __Bt_Gatt_Service			BtGattSrvc_t;

/**
 * @brief	Callback on write
 */
typedef void (*BtGattCharWrCb_t) (BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

/**
 * @brief	Callback on set notification
 */
typedef void (*BtGattCharSetNotifCb_t) (BtGattChar_t *pChar, bool bEnable);

/**
 * @brief	Callback on set indication
 */
typedef void (*BtGattCharSetIndCb_t) (BtGattChar_t *pChar, bool bEnable);

/**
 * @brief	Callback when transmission is completed
 *
 * @param	pBlueIOSvc
 * @param	CharIdx
 */
typedef void (*BtGattCharTxComplete_t) (BtGattChar_t *pChar, int CharIdx);

/**
 * @brief	Callback on authorization request
 *
 * @param	pBlueIOSvc
 * @param	p_ble_evt
 */
typedef void (*BtGattSrvcAuthRqst_t)(BtGattSrvc_t *pBleSvc, uint32_t evt);

#pragma pack(push,4)

struct __Bt_Gatt_Characteristic {
	uint16_t Uuid;						//!< Characteristic UUID
	uint16_t MaxDataLen;				//!< Characteristic max data length in bytes
	uint32_t Property;              	//!< char properties defined by orable BT_GATT_CHAR_PROP_...
	const char *pDesc;                  //!< char UTF-8 description string
	BtGattCharWrCb_t WrCB;              //!< Callback for write char, set to NULL for read char
	BtGattCharSetNotifCb_t SetNotifCB;	//!< Callback on set notification
	BtGattCharSetIndCb_t SetIndCB;		//!< Callback on set indication
	BtGattCharTxComplete_t TxCompleteCB;//!< Callback when TX is completed
	void * const pValue;				//!< Characteristic data value
	uint16_t ValueLen;					//!< Current length in bytes of data value
	// Bellow are private data. Do not modify
	bool bNotify;                       //!< Notify flag for read characteristic
	uint8_t BaseUuidIdx;				//!< Index of Base UUID used for this characteristic.
	uint16_t Hdl;       				//!< char handle
	uint16_t ValHdl;					//!< char value handle
	uint16_t DescHdl;					//!< descriptor handle
	uint16_t CccdHdl;					//!< client char configuration descriptor handle
	uint16_t SccdHdl;					//!< Server char configuration value
	BtGattSrvc_t *pSrvc;				//!< Pointer to the service instance which this char belongs to.
};

typedef struct __Bt_Gatt_Service_Config {
	uint8_t SecType;			//!< Secure or Open service/char
	bool bCustom;						//!< True - for custom service Base UUID, false - Bluetooth SIG standard
	uint8_t UuidBase[16];				//!< 128 bits custom base UUID
	uint16_t UuidSrvc;					//!< Service UUID
	int NbChar;							//!< Total number of characteristics for the service
	BtGattChar_t *pCharArray;          	//!< Pointer a an array of characteristic
    uint8_t *pLongWrBuff;				//!< pointer to user long write buffer
    int	LongWrBuffSize;					//!< long write buffer size
    BtGattSrvcAuthRqst_t AuthReqCB;		//!< Authorization request callback
} BtGattSrvcCfg_t;

/*
 * Bluetooth service private data to be passed when calling service related functions.
 * The data is filled by BleSrvcInit function.
 * Pointer to this structure is often referred as Service Handle
 *
 */
struct __Bt_Gatt_Service {
    int NbChar;							//!< Number of characteristic defined for this service
    BtGattChar_t *pCharArray;			//!< Pointer to array of characteristics
    uint16_t Hdl;            			//!< Service handle
//    uint16_t ConnHdl;					//!< Connection handle
    BtUuid_t Uuid;						//!< Service UUID
    uint8_t	*pLongWrBuff;				//!< pointer to user long write buffer
    int	LongWrBuffSize;					//!< long write buffer size
    void *pContext;
    BtGattSrvcAuthRqst_t AuthReqCB;		//!< Authorization request callback
    BtGattSrvc_t *pPrev;
    BtGattSrvc_t *pNext;
};

/*
typedef struct __Bt_Gatt_Char_Value {
	size_t MaxLen;					//!< Max length of data buffer
	size_t Len;						//!< Length of actual data
	void *pData;					//!< pointer to characteristic static data buffer
	BtGattCharWrCb_t WrHandler;	//!< Pointer to characteristic write callback
	BtGattChar_t *pChar;
} BtGattCharValue_t;
*/
typedef struct __Bt_Gatt_List_Entry {
	uint16_t Hdl;						//!< Attribute handle
	BtUuid16_t TypeUuid;				//!< Attribute type UUID
	uint32_t Permission;				//!< Attribute Permission
	union {								//!< Attribute value
		void *pVal;
		uint32_t Val32;
		BtUuid_t Uuid;
		BtGattChar_t *pChar;
//		BtGattCharValue_t CharVal;		//!< Characteristic value
		BtGattSrvcDeclar_t SrvcDeclar;	//!< Service declaration value
		BtGattSrvcInclude_t SrvcInc;	//!< Service include definition value
		BtGattCharDeclar_t CharDeclar;	//!< Characteristic declaration value
	};
} BtGattListEntry_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

BtGattSrvc_t * const BtGattGetSrvcList();
void BtGattInsertSrvcList(BtGattSrvc_t * const pSrvc);
uint16_t BtGattRegister(BtUuid16_t *pTypeUuid, void * const pAttVal);
bool BtGattUpdate(uint16_t Hdl, void * const pAttVal, size_t Len);
int BtGattGetListHandle(uint16_t StartHdl, uint16_t EndHdl, BtGattListEntry_t *pArr, int MaxEntry, uint16_t *pLastHdl);
int BtGattGetListUuid(BtUuid16_t *pTypeUuid, uint16_t StartHdl, BtGattListEntry_t *pArr, int MaxEntry, uint16_t *pLastHdl);
bool BeGattFindEntryUuid(BtUuid16_t *pTypeUuid, uint16_t StartHdl, uint16_t EndHdl, BtGattListEntry_t *pEntry);
bool BtGattGetEntryHandle(uint16_t Hdl, BtGattListEntry_t *pArr);
size_t BtGattGetValue(BtGattListEntry_t *pEntry, uint8_t *pBuff);
size_t BtGattWriteValue(uint16_t Hdl, uint8_t *pBuff, size_t Len);
BtGattListEntry_t *GetEntryTable(size_t *pCount);
bool isBtGattCharNotifyEnabled(BtGattChar_t *pChar);
bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len);
bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len);
bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc, BtGattSrvcCfg_t const * const pCfg);
void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc);
//void BtGattServiceInit(BtGattSrvc_t * const pSrvc);
void BtGattEvtHandler(uint32_t Evt);

#ifdef __cplusplus
}
#endif

#endif // __BT_GATT_H__


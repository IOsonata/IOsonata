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
#include "bluetooth/bt_att.h"

#define BT_GATT_CHAR_EXT_PROP_RELIABLE_WRITE	1	//!< Reliable write using procedure Section 4.9.5
#define BT_GATT_CHAR_EXT_PROP_WRITABLE_AUX		2	//!< Write to descriptor defined in Section 3.3.3.2

// Characteristic extended property attribute : type UUID 0x2900
//typedef uint8_t		BtGattCharExtProp_t;

// Characteristic user description attribute : Type UUID 0x2901
// Attribute value : UTF-8
//typedef char *		BtGattCharUserDesc_t;


// Client characteristic configuration declaration attribute UUID 0x2902
//typedef uint8_t BtGatClientCharConfig_t;

//#define BT_GATT_SERVER_CHAR_CONFIG_BROADCAST		1

// Server characteristic configuration declaration attribute UUID 0x2903
//typedef uint8_t		BtGattServerCharConfig_t;

#define BT_GATT_CHAR_PROP_BROADCAST			BT_CHAR_PROP_BROADCAST	//!< If set, permits broadcasts of the
												//!< Characteristic Value using Server
												//! Characteristic Configuration Descriptor.
												//! If set, the Server Characteristic Configuration
												//! Descriptor shall exist.
#define BT_GATT_CHAR_PROP_READ				BT_CHAR_PROP_READ	//!< If set, permits reads of the Characteristic Value
#define BT_GATT_CHAR_PROP_WRITE_WORESP		BT_CHAR_PROP_WRITE_WORESP	//!<
#define BT_GATT_CHAR_PROP_WRITE				BT_CHAR_PROP_WRITE
#define BT_GATT_CHAR_PROP_NOTIFY			BT_CHAR_PROP_NOTIFY
#define BT_GATT_CHAR_PROP_INDICATE			BT_CHAR_PROP_INDICATE
#define BT_GATT_CHAR_PROP_AUTH_SIGNED		BT_CHAR_PROP_AUTH_SIGNED
#define BT_GATT_CHAR_PROP_EXT_PROP			BT_CHAR_PROP_EXT_PROP
#define BT_GATT_CHAR_PROP_VALEN				BT_CHAR_PROP_VALEN

#pragma pack(push,1)

typedef struct __Bt_Gatt_Char_Srvc_Changed {
	uint16_t StartHdl;					//!< Start service handle
	uint16_t EndHdl;					//!< End service handle
} BtGattCharSrvcChanged_t;

typedef struct __Bt_Gatt_Char_Prefered_Conn_Params {
	uint16_t IntervalMin;				//!< Min connection interval in 1.25ms counts
	uint16_t IntervalMax;				//!< Max connection interval in 1.25ms counts
	uint16_t Latency;					//!< Peripheral latency
	uint16_t Timeout;					//!< Supervision timeout in 10ms count
} BtGattPreferedConnParams_t;

#pragma pack(pop)


#pragma pack(push,4)

typedef struct __Bt_Gatt_Service_Config {
	uint8_t SecType;					//!< Secure or Open service/char
	bool bCustom;						//!< True - for custom service Base UUID, false - Bluetooth SIG standard
	uint8_t UuidBase[16];				//!< 128 bits custom base UUID
	uint16_t UuidSrvc;					//!< Service UUID
	int NbChar;							//!< Total number of characteristics for the service
	BtChar_t *pCharArray;          	//!< Pointer a an array of characteristic
    uint8_t *pLongWrBuff;				//!< pointer to user long write buffer
    int	LongWrBuffSize;					//!< long write buffer size
    BtSrvcAuthRqst_t AuthReqCB;			//!< Authorization request callback
} BtGattSrvcCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

BtGattSrvc_t * const BtGattGetSrvcList();
void BtGattInsertSrvcList(BtGattSrvc_t * const pSrvc);
bool BtGattUpdate(uint16_t Hdl, void * const pAttVal, size_t Len);
bool isBtGattCharNotifyEnabled(BtGattChar_t *pChar);
bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len);
bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len);
bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc, BtGattSrvcCfg_t const * const pCfg);
void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc);
//void BtGattServiceInit(BtGattSrvc_t * const pSrvc);
void BtGattEvtHandler(uint32_t Evt, void * const pCtx);
void BtGattSrvcEvtHandler(BtGattSrvc_t * const pSrvc, uint32_t Evt, void * const pCtx);
void BtGattSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent);

#ifdef __cplusplus
}
#endif

#endif // __BT_GATT_H__


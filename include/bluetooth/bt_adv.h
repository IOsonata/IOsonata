/**-------------------------------------------------------------------------
@file	bt_adv.h

@brief	Bluetooth advertisement definitions


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
#ifndef __BT_ADV_H__
#define __BT_ADV_H__

#include <stdint.h>

#include "bluetooth/bt_uuid.h"

#pragma pack(push, 1)

typedef enum __Bt_Addr_Type {
	BT_ADDR_TYPE_PUBLIC = 0,			//!< Public device address
	BT_ADDR_TYPE_RAND = 1,				//!< Random device address
	BT_ADDR_TYPE_RESOLV = 2,			//!< Public Identity Address (Corresponds to Resolved Private Address)
	BT_ADDR_TYPE_RANDOM_STATIC = 3		//!< Random (static) Identity Address (Corresponds to Resolved Private Address)
} BT_ADDR_TYPE;

/// BLE Advertising type
typedef enum __Bt_Adv_Type {
	BTADV_TYPE_ADV_IND = 0,				//!< Connectable and scannable undirected advertising
	BTADV_TYPE_ADV_DIRECT_IND = 1,		//!< Connectable high duty cycle directed advertising
	BTADV_TYPE_ADV_SCAN_IND = 2,		//!< Scannable undirected advertising
	BTADV_TYPE_ADV_NONCONN_IND = 3,		//!< Non connectable undirected advertising
	BTADV_TYPE_SCAN_RSP = 4				//!< Scan Response (SCAN_RSP)
} BTADV_TYPE;

typedef enum __Bt_Adv_Filter_Policy {
	BTADV_FILTER_POLICY_NONE = 0,		//!< Accept request from all conn & scan
	BTADV_FILTER_POLICY_SCAN = 1,		//!< Accept request from all conn, scan list
	BTADV_FILTER_POLICY_CONN = 2,		//!< Accept request from all scan, conn list
	BTADV_FILTER_POLICY_ALL = 3			//!< Accept request only from list
} BTADV_FILTER_POLICY;

// Orable advertisement channels
#define	BTADV_CHAN_37		1
#define	BTADV_CHAN_38 		2
#define	BTADV_CHAN_39 		4

/// Convert msec time to BLE interval value of 0.625ms units
//#define BLEADV_MS_TO_INTERVAL(Val)		(((Val) * 1000UL + 500UL)/ 625UL)

/// BLE Advertising parameters
typedef struct __Bt_Adv_Param {
	uint16_t IntervalMin;				//!< Advertising interval min. t = minval * 0.625ms
	uint16_t IntervalMax;				//!< Advertising interval max. t = maxval * 0.625ms
	BTADV_TYPE Type:8;
	BT_ADDR_TYPE OwnAddrType:8;
	BT_ADDR_TYPE PeerAddrType:8;		//!< only BLEADV_ADDR_TYPE_PUBLIC or BLEADV_ADDR_TYPE_RAND
	uint8_t PeerAddr[6];				//!< Peer address
	uint8_t ChanMap;					//!< Advertising channel map
	BTADV_FILTER_POLICY FilterPolicy;	//!< Advertising filter policy
} BtAdvParam_t;

#define BTADV_EXTADV_EVT_PROP_CONNECTABLE				(1<<0)	//!< Connectable advertising
#define BTADV_EXTADV_EVT_PROP_SCANNABLE					(1<<1)	//!< Scannable advertising
#define BTADV_EXTADV_EVT_PROP_DIRECT					(1<<2)	//!< Direct advertising
#define BTADV_EXTADV_EVT_PROP_HIGH_DUTY					(1<<3)	//!< High duty cycle direct connectable <= 3.75ms interval
#define BTADV_EXTADV_EVT_PROP_LEGACY					(1<<4)	//!< Legacy advertising using PDU
#define BTADV_EXTADV_EVT_PROP_OMIT_ADDR					(1<<5)	//!< Omit advertise's address from all PDU (anonymous)
#define BTADV_EXTADV_EVT_PROP_TXPWR						(1<<6)	//!< Include Tx power in the extended header

#define BTADV_EXTADV_PHY_1M								1
#define BTADV_EXTADV_PHY_2M								2
#define BTADV_EXTADV_PHY_CODED							3


/// BLE extended advertising parameters
typedef struct _Bt_Ext_Adv_Param {
	uint32_t AdvHdl:8;					//!< Advertising handle
	uint32_t EvtProp:16;				//!< Advertising event property
	uint32_t PrimIntervalMin:24;		//!< Primary advertising interval min. in 625 usec unit
	uint32_t PrimIntervalMax:24;		//!< Primary advertising interval max. in 625 usec unit
	uint32_t PrimChanMap:8;				//!< Primary channel map
	BT_ADDR_TYPE OwnAddrType:8;
	BT_ADDR_TYPE PeerAddrType:8;		//!< only BLEADV_ADDR_TYPE_PUBLIC or BLEADV_ADDR_TYPE_RAND
	uint8_t PeerAddr[6];				//!< Peer address
	BTADV_FILTER_POLICY FilterPolicy;			//!< Advertising filter policy
	uint8_t TxPwr;						//!< Advertising TX power in dBm
	uint8_t PrimPhy;					//!< Primary advertising PHY
	uint8_t SecondMaxSkip;				//!< Secondary advertising max skip
	uint8_t SecondPhy;					//!< Secondary advertising PHY
	uint8_t Sid;						//!< Advertising SID
	uint8_t ScanNotifEnable;			//!< Scan request notification enable
} BtExtAdvParam_t;

typedef struct __Bt_Adv_Data_Header {
	uint8_t Len;						//!< Length of data including the Type byte
	uint8_t Type;						//!< GAP Data type
} BtAdvDataHdr_t;

typedef struct __Bt_Adv_Data {
	BtAdvDataHdr_t Hdr;					//!< Advertisement data header
	uint8_t Data[1];					//!< Variable data field
} BtAdvData_t;

typedef struct __Bt_Adv_Data_Flags {
	BtAdvDataHdr_t Hdr;					//!< Advertisement data header
	uint8_t Flags;						//!< GAP Flags
} BtdvDataFlags_t;

typedef struct __Bt_Adv_Packet {
	int MaxLen;							//!< Max adv data length
	int Len;							//!< Advertisement data length
	uint8_t * const pData;				//!< Pointer to advertisement data
} BtAdvPacket_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// Convert msec time to BLE interval value of 0.625ms units
//static inline uint16_t BleAdvMsToInterval(uint32_t Val) {
//	return (uint16_t)((Val * 1000UL + 500UL) / 625UL);
//};

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
BtAdvData_t *BtAdvDataAllocate(BtAdvPacket_t * const pAdvPkt, uint8_t Type, int Len);

/**
 * @brief	Add advertisement data into the adv packet
 *
 * @param 	pAdvPkt	: Pointer to Adv packet to add data into
 * @param 	Type 	: GAP data type of the data
 * @param	pData	: Pointer to data to add
 * @param	Len		: Length in bytes of the data
 *
 * @return	true - success
 */
bool BtAdvDataAdd(BtAdvPacket_t * const pAdvPkt, uint8_t Type, uint8_t *pData, int Len);

/**
 * @brief	Remove advertisement data from the adv packet
 *
 * @param 	pAdvPkt	: Pointer to Adv packet to add data into
 * @param 	Type 	: GAP data type of the data
 *
 * @return	none
 */
void BtAdvDataRemove(BtAdvPacket_t * const pAdvPkt, uint8_t Type);

/**
 * @brief	Add UUID list to the advertising data
 *
 * @param 	pAdvPkt	: Pointer to Adv packet to add data into
 * @param 	pUid	: Pointer to UUID array list
 * @param 	bComplete : true - UUID list is complete, false - partial
 *
 * @return	true - success
 */
bool BtAdvDataAddUuid(BtAdvPacket_t * const pAdvPkt, const BtUuidArr_t *pUid, bool bComplete);

/**
 * @brief	Set device name to the advertisement packet
 *
 * @param 	pAdvPkt	: Pointer to Adv packet to add data into
 * @param	pName	: Pointer to device name string
 *
 * @return	true - success
 */
bool BtAdvDataSetDevName(BtAdvPacket_t * const pAdvPkt, const char *pName);

size_t BtAdvDataGetDevName(uint8_t *pAdvData, size_t AdvLen, char *pName, size_t NameLen);
size_t BtAdvDataGetManData(uint8_t *pAdvData, size_t AdvLen, uint8_t *pBuff, size_t BuffLen);

void BtAdvStart();
void BtAdvStop();

#ifdef __cplusplus
}
#endif

#endif // __BT_ADV_H__

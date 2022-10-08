/**-------------------------------------------------------------------------
@file	ble_adv.h

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
#ifndef __BLE_ADV_H__
#define __BLE_ADV_H__

#include <stdint.h>

#pragma pack(push, 1)

typedef enum __Ble_Addr_Type {
	BLE_ADDR_TYPE_PUBLIC = 0,		//!< Public device address
	BLE_ADDR_TYPE_RAND = 1,			//!< Random device address
	BLE_ADDR_TYPE_RESOLV = 2,		//!< Resolvable private
	BLE_ADDR_TYPE_RESOLV_RAND = 3	//!< Resolvable private, if not exist use random
} BLE_ADDR_TYPE;

/// BLE Advertising type
typedef enum __Ble_Adv_Type {
	BLEADV_TYPE_ADV_IND = 0,			//!< Connectable and scannable undirected advertising
	BLEADV_TYPE_ADV_DIRECT_HIGH_IND = 1,//!< Connectable high duty cycle directed advertising
	BLEADV_TYPE_ADV_SCAN_IND = 2,		//!< Scannable undirected advertising
	BLEADV_TYPE_ADV_NONCONN_IND = 3,	//!< Non connectable undirected advertising
	BLEADV_TYPE_ADV_DIRECT_LOW_IND = 4	//!< Connectable low duty cycle directed advertising
} BLEADV_TYPE;

typedef enum __Ble_Adv_Filter_Policy {
	BLE_ADV_FILTER_POLICY_NONE = 0,		//!< Accept request from all conn & scan
	BLE_ADV_FILTER_POLICY_SCAN = 1,		//!< Accept request from all conn, scan list
	BLE_ADV_FILTER_POLICY_CONN = 2,		//!< Accept request from all scan, conn list
	BLE_ADV_FILTER_POLICY_ALL = 3		//!< Accept request only from list
} BLE_ADV_FILTER_POLICY;

// Orable advertisement channels
#define	BLEADV_CHAN_37		1
#define	BLEADV_CHAN_38 		2
#define	BLEADV_CHAN_39 		4

/// Convert msec time to BLE interval value of 0.625ms units
#define BLEADV_MS_TO_INTERVAL(Val)		(((Val) * 1000UL + 500UL)/ 625UL)

/// BLE Advertising parameters
typedef struct __Ble_Adv_Param {
	uint16_t IntervalMin;			//!< Advertising interval min. t = minval * 0.625ms
	uint16_t IntervalMax;			//!< Advertising interval max. t = maxval * 0.625ms
	BLEADV_TYPE Type:8;
	BLE_ADDR_TYPE OwnAddrType:8;
	BLE_ADDR_TYPE PeerAddrType:8;	//!< only BLEADV_ADDR_TYPE_PUBLIC or BLEADV_ADDR_TYPE_RAND
	uint8_t PeerAddr[6];			//!< Peer address
	uint8_t ChanMap;				//!< Advertising channel map
	BLE_ADV_FILTER_POLICY FilterPolicy;			//!< Advertising filter policy
} BleAdvParam_t;

#define BLE_EXT_ADV_EVT_PROP_CONNECTABLE				(1<<0)	//!< Connectable advertising
#define BLE_EXT_ADV_EVT_PROP_SCANNABLE					(1<<1)	//!< Scannable advertising
#define BLE_EXT_ADV_EVT_PROP_DIRECT						(1<<2)	//!< Direct advertising
#define BLE_EXT_ADV_EVT_PROP_HIGH_DUTY					(1<<3)	//!< High duty cycle direct connactable <= 3.75ms interval
#define BLE_EXT_ADV_EVT_PROP_LEGACY						(1<<4)	//!< Legacy advertising using PDU
#define BLE_EXT_ADV_EVT_PROP_OMIT_ADDR					(1<<5)	//!< Omit advertise's address from all PDU (anonymous)
#define BLE_EXT_ADV_EVT_PROP_TXPWR						(1<<6)	//!< Include Tx power in the extended header

#define BLE_EXT_ADV_PHY_1M								1
#define BLE_EXT_ADV_PHY_2M								2
#define BLE_EXT_ADV_PHY_CODED							3


/// BLE extended advertising parameters
typedef struct _Ble_Ext_Adv_Param {
	uint32_t AdvHdl:8;					//!< Advertising handle
	uint32_t EvtProp:16;				//!< Advertising event property
	uint32_t PrimIntervalMin:24;	//!< Primary advertising interval min. in 625 usec unit
	uint32_t PrimIntervalMax:24;	//!< Primary advertising interval max. in 625 usec unit
	uint32_t PrimChanMap:8;			//!< Primary channel map
	BLE_ADDR_TYPE OwnAddrType:8;
	BLE_ADDR_TYPE PeerAddrType:8;	//!< only BLEADV_ADDR_TYPE_PUBLIC or BLEADV_ADDR_TYPE_RAND
	uint8_t PeerAddr[6];			//!< Peer address
	BLE_ADV_FILTER_POLICY FilterPolicy;			//!< Advertising filter policy
	uint8_t TxPwr;					//!< Advertising TX power in dBm
	uint8_t PrimPhy;				//!< Primary advertising PHY
	uint8_t SecondMaxSkip;			//!< Secondary advertising max skip
	uint8_t SecondPhy;				//!< Secondary advertising PHY
	uint8_t Sid;					//!< Advertising SID
	uint8_t ScanNotifEnable;		//!< Scan request notification enable
} BleExtAdvParam_t;

typedef struct __Ble_Adv_Data_Header {
	uint8_t Len;					//!< Length of data including the Type byte
	uint8_t Type;					//!< GAP Data type
} BleAdvDataHdr_t;

typedef struct __Ble_Adv_Data {
	BleAdvDataHdr_t Hdr;			//!< Advertisement data header
	uint8_t Data[1];				//!< Variable data field
} BleAdvData_t;

typedef struct __Ble_Adv_Data_Flags {
	BleAdvDataHdr_t Hdr;			//!< Advertisement data header
	uint8_t Flags;					//!< GAP Flags
} BleAdvDataFlags_t;

typedef struct __Ble_Adv_Packet {
	int MaxLen;						//!< Max adv data length
	int Len;						//!< Advertisement data length
	uint8_t * const pData;			//!< Pointer to advertisement data
} BleAdvPacket_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// Convert msec time to BLE interval value of 0.625ms units
static inline uint16_t BleAdvMsToInterval(uint32_t Val) {
	return (uint16_t)((Val * 1000UL + 500UL) / 625UL);
};

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
bool BleAdvAddData(BleAdvPacket_t *pAdvPkt, uint8_t Type, uint8_t *pData, int Len);

/**
 * @brief	Remove advertisement data from the adv packet
 *
 * @param 	pAdvPkt	: Pointer to Adv packet to add data into
 * @param 	Type 	: GAP data type of the data
 *
 * @return	none
 */
void BleAdvRemoveData(BleAdvPacket_t *pAdvPkt, uint8_t Type);


#ifdef __cplusplus
}
#endif

#endif // __BLE_ADV_H__

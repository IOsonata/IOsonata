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
	uint8_t FilterPolicy;			//!< Advertising filter policy
} BleAdvParam_t;

typedef struct __Ble_Adv_Data_Header {
	uint8_t Len;					//!< Length of data
	uint8_t Type;					//!< Data type
} BleAdvDataHdr_t;

typedef struct __Ble_Adv_Data {
	BleAdvDataHdr_t Hdr;			//!< Advertisement data header
	uint8_t Data[1];				//!< Variable data field
} BleAdvData_t;

typedef struct __Ble_Adv_Data_Flags {
	BleAdvDataHdr_t Hdr;			//!< Advertisement data header
	uint8_t Flags;					//!< GAP Flags
} BleAdvDataFlags_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// Convert msec time to BLE interval value of 0.625ms units
static inline uint16_t BleAdvMsToInterval(uint32_t Val) {
	return (uint16_t)((Val * 1000UL + 500UL) / 625UL);
};

/**
 * @brief
 *
 * @param Type
 * @param pData
 * @param Len
 * @return
 */
int BleAdvSetAdvData(uint8_t Type, uint8_t *pData, int Len);
int BleAdvGetAdvData(uint8_t *pBuff, int Len);

#ifdef __cplusplus
}
#endif

#endif // __BLE_ADV_H__

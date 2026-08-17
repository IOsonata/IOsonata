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

#include <stddef.h>
#include <stdint.h>

#include "bluetooth/bt_ead.h"
#include "bluetooth/bt_uuid.h"

/// Maximum advertising or scan-response data payload (in octets) that fits in
/// legacy advertising PDUs. Above this, extended advertising PDUs are required.
#define BT_ADV_LEGACY_DATA_MAX		31

#pragma pack(push, 1)

typedef enum __Bt_Addr_Type {
	BTADDR_TYPE_PUBLIC = 0,			//!< Public device address
	BTADDR_TYPE_RAND = 1,				//!< Random device address
	BTADDR_TYPE_RESOLV = 2,			//!< Public Identity Address (Corresponds to Resolved Private Address)
	BTADDR_TYPE_RANDOM_STATIC = 3		//!< Random (static) Identity Address (Corresponds to Resolved Private Address)
} BTADDR_TYPE;

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
	BTADDR_TYPE OwnAddrType:8;
	BTADDR_TYPE PeerAddrType:8;			//!< only BLEADV_ADDR_TYPE_PUBLIC or BLEADV_ADDR_TYPE_RAND
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
// Advertising Coding Selection, Core 5.4. The coding used on the LE Coded PHY
// is chosen by the Host through the [v2] parameters command rather than left
// to the Controller, which is what the feature adds.
//
// Primary_Advertising_PHY_Options and Secondary_Advertising_PHY_Options,
// Vol 4 Part E 7.8.53. A preference lets the Controller fall back, a
// requirement does not.
#define BTADV_PHY_OPT_NONE								0x00	//!< No preferred or required coding
#define BTADV_PHY_OPT_PREFER_S2							0x01	//!< Prefers S=2
#define BTADV_PHY_OPT_PREFER_S8							0x02	//!< Prefers S=8
#define BTADV_PHY_OPT_REQUIRE_S2						0x03	//!< Requires S=2
#define BTADV_PHY_OPT_REQUIRE_S8						0x04	//!< Requires S=8
#define BTADV_PHY_OPT_MAX								0x04

/// FeatureSet bit the Host sets to say it supports Advertising Coding
/// Selection, Vol 6 Part B 4.6.33.3. With it clear a Primary or Secondary PHY
/// of 0x03 means LE Coded with the Controller choosing; with it set 0x03 is
/// S=8 and 0x04 is S=2.
#define BTADV_FEATURE_BIT_CODING_SELECTION				41

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
	BTADDR_TYPE OwnAddrType:8;
	BTADDR_TYPE PeerAddrType:8;			//!< only BLEADV_ADDR_TYPE_PUBLIC or BLEADV_ADDR_TYPE_RAND
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
} BtAdvDataFlags_t;

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

// Own address currently in use on air for advertising (the address the
// advertising set was last programmed with). Falls back to the device's
// configured address when the set was never programmed. A new peripheral
// connection is stamped with this so the SMP toolbox computes f5/f6/c1 with
// the address the peer actually saw.
void BtAdvOwnAddrGet(uint8_t *pType, uint8_t pAddr[6]);

/**
 * @brief	Tell the controller this host supports Advertising Coding Selection
 *
 * Issues LE Set Host Feature for FeatureSet bit 41 (Vol 4 Part E 7.8.115).
 * Until this succeeds a Primary or Secondary PHY of 0x03 means LE Coded with
 * the controller choosing the coding, and the PHY options of the [v2]
 * parameters command have no effect.
 *
 * Vol 4 Part E 7.8.115 has the controller answer Command Disallowed while it
 * holds any connection, so this belongs in initialization rather than in
 * response to anything. A controller without the feature answers Unsupported
 * Feature or Parameter Value.
 *
 * @param	bEnable	: true to set the bit, false to clear it
 *
 * @return	true when the controller accepted it
 */
bool BtAdvCodingSelectionEnable(bool bEnable);

/**
 * @brief	Arm or disarm Encrypted Advertising Data
 *
 * With key material installed, BtAdvEncode wraps the advertising data and the
 * scan response data it builds in Encrypted Data AD structures (Core 5.4
 * Vol 3 Part C 10.10, CSS Part A 1.23). With none installed it does nothing
 * at all, so a build that never calls this is unchanged.
 *
 * This is the only place the feature is switched on. Every port reaches the
 * advertising data through the same BtAdvEncode, so arming it here arms it
 * everywhere rather than per port.
 *
 * The key material is copied. Pass null to disarm and wipe the copy.
 *
 * The AES and random engines must already be bound through BtEadInit; the
 * randomizer has to be unpredictable or the feature gives back the tracking
 * it exists to prevent.
 *
 * @param	pKey	: Session key and IV, or null to disarm
 *
 * @return	true on success
 */
bool BtAdvEadKeySet(const BtEadKey_t * const pKey);

/**
 * @brief	Whether Encrypted Advertising Data is armed
 *
 * @return	true when key material is installed
 */
bool BtAdvEadIsArmed(void);

/**
 * @brief	Wrap one advertising packet in an Encrypted Data AD structure
 *
 * Rewrites pPkt in place as a single AD structure holding a fresh randomizer,
 * the encrypted former contents and the MIC. BtAdvEncode calls this for both
 * packets when armed, so a port has nothing to do; it is public because a
 * port that builds advertising data some other way still needs it.
 *
 * A fresh randomizer is drawn for each call, which is what CSS Part A 1.23.4
 * requires whenever the payload changes.
 *
 * Nothing is written unless the whole structure fits, so a failure leaves the
 * plaintext packet as it was rather than half encrypted.
 *
 * @param	pPkt	: Packet to wrap in place
 *
 * @return	true on success, false when not armed, out of room, or the crypto
 *			failed
 */
bool BtAdvEncrypt(BtAdvPacket_t *pPkt);

/**
 * @brief	Recover the payload of an Encrypted Data AD structure in a report
 *
 * The receiving half of BtAdvEncrypt, for a scan report handler to call. The
 * advertising data is walked for an Encrypted Data structure and its payload
 * is decrypted into pOut. The recovered payload is itself a sequence of AD
 * structures, so a caller reads it the same way it reads a plain report.
 *
 * The MIC is verified first, so nothing is returned for a structure that does
 * not authenticate under pKey.
 *
 * @param	pKey		: Session key and IV of the advertiser being followed
 * @param	pAdvData	: Advertising data of the report
 * @param	Len			: Advertising data length
 * @param	pOut		: Output buffer for the recovered payload
 * @param	OutLen		: Output buffer size
 *
 * @return	Payload octets recovered. 0 when the report holds no Encrypted Data
 *			structure, when it is malformed, or when the MIC does not verify.
 */
size_t BtAdvDecrypt(const BtEadKey_t * const pKey, const uint8_t *pAdvData,
					size_t Len, uint8_t *pOut, size_t OutLen);

/**
 * @brief	Choose the PHY the next advertising set advertises on
 *
 * Takes effect at the next BtAppAdvInit. The default is 1M primary and 2M
 * secondary, which is what every set used before the PHY could be chosen.
 *
 * A coding selected with BtAdvCodingSet only reaches the air on LE Coded, so
 * this is the call that has to come first.
 *
 * @param	PrimPhy	: BTADV_EXTADV_PHY_1M or BTADV_EXTADV_PHY_CODED. 7.8.53
 *					  gives the primary PHY no 2M, since 2M cannot carry an
 *					  ADV_EXT_IND.
 * @param	SecPhy	: BTADV_EXTADV_PHY_1M, _2M or _CODED
 *
 * @return	true when both values are legal for their position
 */
bool BtAdvPhySet(uint8_t PrimPhy, uint8_t SecPhy);

/**
 * @brief	Choose the LE Coded PHY coding the next advertising set uses
 *
 * The values take effect at the next BtAppAdvInit, which sends the [v2]
 * parameters command instead of [v1] when either option is not
 * BTADV_PHY_OPT_NONE. Both BTADV_PHY_OPT_NONE restores [v1], which is what a
 * controller predating Core 5.4 accepts.
 *
 * A coding only means anything on LE Coded: 7.8.53 has the controller ignore
 * the option when the PHY in that position is 1M or 2M. Asking for one there
 * is refused rather than accepted and dropped, so call BtAdvPhySet with
 * BTADV_EXTADV_PHY_CODED first.
 *
 * @param	PrimOpt	: BTADV_PHY_OPT_* for the primary advertising PHY
 * @param	SecOpt	: BTADV_PHY_OPT_* for the secondary advertising PHY
 *
 * @return	true when both values are in range and their PHY is LE Coded
 */
bool BtAdvCodingSet(uint8_t PrimOpt, uint8_t SecOpt);

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

void BtAdvStart(void);
void BtAdvStop(void);

/**
 * @brief	Decide whether extended advertising PDUs are required.
 *
 * Legacy advertising PDUs cap both the advertising data and the scan response
 * data at BT_ADV_LEGACY_DATA_MAX (31) octets each.
 *
 * @param	AdvLen	Assembled advertising data length in octets.
 * @param	SrLen	Assembled scan-response data length in octets.
 *
 * @return	true if extended advertising PDUs are required, false if the
 * 			payloads fit in legacy advertising PDUs.
 */
static inline bool BtAdvUseExtended(size_t AdvLen, size_t SrLen)
{
	return AdvLen > BT_ADV_LEGACY_DATA_MAX || SrLen > BT_ADV_LEGACY_DATA_MAX;
}

/**
 * @brief	Check a static random device address.
 *
 * Vol 6 Part B 1.3.2.1 puts two requirements on a static random address: the
 * two most significant bits of the most significant octet are both 1, and the
 * remaining 46 bits hold at least one bit equal to 0 and at least one bit
 * equal to 1. The second requirement is what rules out the all-zero address
 * left by an uninitialised identity and the all-ones address; the bit pattern
 * check alone accepts both, and a controller then rejects the command or the
 * device advertises an address no peer can bond to.
 *
 * @param	pAddr	Six octet address, least significant octet first.
 *
 * @return	true if the address is a valid static random address.
 */
static inline bool BtAddrIsStaticRandom(const uint8_t *pAddr)
{
	if (pAddr == NULL)
	{
		return false;
	}

	if ((pAddr[5] & 0xC0) != 0xC0)
	{
		return false;
	}

	// Accumulate the random part twice: once as it is and once inverted. A
	// zero accumulator means that bit value never appeared in the 46 bits.
	uint8_t ones = pAddr[5] & 0x3F;
	uint8_t zeros = (uint8_t)(~pAddr[5]) & 0x3F;

	for (int i = 0; i < 5; i++)
	{
		ones |= pAddr[i];
		zeros |= (uint8_t)~pAddr[i];
	}

	return ones != 0 && zeros != 0;
}

#ifdef __cplusplus
}
#endif

#endif // __BT_ADV_H__

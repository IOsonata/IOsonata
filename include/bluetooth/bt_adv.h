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

#include "bluetooth/bt_uuid.h"

// Periodic advertising takes the HCI device the commands go to. Forward
// declare it with the same guard bt_hci.h uses, so including that header is
// not forced on every user of this one.
#ifndef BT_HCI_DEVICE_T_DEFINED
#define BT_HCI_DEVICE_T_DEFINED
typedef struct __Bt_Hci_Device		BtHciDevice_t;
#endif

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
 * @brief	Request an LE Coded PHY coding for the advertising set.
 *
 * Advertising Coding Selection, Core 5.4. Values are BT_HCI_ADV_PHY_OPT_*: no
 * preference, prefer S=2 or S=8, or require S=2 or S=8. A non zero option also
 * selects the LE Coded PHY for that advertising physical channel, because the
 * coding field has no meaning on any other PHY.
 *
 * Takes effect at the next advertising init. Silently ignored at that point if
 * the controller lacks either the LE Coded PHY or Advertising Coding
 * Selection, since a controller without the feature refuses any non zero
 * option outright.
 *
 * @param	PrimOption	Coding for the primary advertising physical channel.
 * @param	SecOption	Coding for the secondary advertising physical channel.
 *
 * @return	true when both values are in range, false otherwise, in which case
 * 			nothing is changed.
 */
bool BtAdvCodingSelectionSet(uint8_t PrimOption, uint8_t SecOption);

/**
 * @brief	Read back the requested advertising coding selection.
 *
 * Reports what was asked for, not what the controller granted. Either pointer
 * may be NULL.
 */
void BtAdvCodingSelectionGet(uint8_t *pPrimOption, uint8_t *pSecOption);

// --- Periodic advertising, Core Vol 4 Part E 7.8.61 to 7.8.63 ---

//!< Advertising set the periodic advertising train rides on. Section 7.8.61
//!< refuses a train on a set that is scannable, connectable, legacy or
//!< anonymous, so it cannot be the set bt_adv_hci uses for a connectable
//!< peripheral. A controller reporting fewer sets than this cannot run one.
#define BT_ADV_PERIODIC_ADV_HANDLE			1

//!< Smallest periodic advertising interval, 1.25 ms units, 7.5 ms.
#define BT_ADV_PERIODIC_INTERVAL_MIN		0x0006

//!< Periodic_Advertising_Properties bit 6, include TxPower in the PDU.
#define BT_ADV_PERIODIC_PROP_INCLUDE_TXPOWER	(1U << 6)

//!< Data octets one LE Set Periodic Advertising Data command can hold.
#define BT_ADV_PERIODIC_FRAGMENT_MAX		252

//!< Periodic advertising data a train can hold, reassembled from fragments.
#define BT_ADV_PERIODIC_DATA_MAX			1650

typedef struct __Bt_Adv_Periodic_Cfg {
	uint16_t IntervalMin;		//!< Periodic advertising interval min, 1.25 ms units
	uint16_t IntervalMax;		//!< Periodic advertising interval max, 1.25 ms units
	uint8_t  OwnAddrType;		//!< BTADDR_TYPE_PUBLIC or BTADDR_TYPE_RAND
	uint8_t  Sid;				//!< Advertising set id, 0 to 15
	bool     IncludeTxPower;	//!< Include TxPower in the periodic advertising PDU
} BtAdvPeriodicCfg_t;

/**
 * @brief	Create the advertising set and configure a periodic advertising train.
 *
 * Refuses when the controller lacks LE Periodic Advertising, extended
 * advertising, any of the three periodic commands, or reports too few
 * advertising sets to hold a second one.
 *
 * @return	true when the set and the train parameters were accepted.
 */
bool BtAdvPeriodicInit(BtHciDevice_t * const pDev, const BtAdvPeriodicCfg_t *pCfg);

/**
 * @brief	Replace the periodic advertising data.
 *
 * Fragments to what one command can hold. A run that fails part way leaves the
 * train holding partial data, which the controller refuses to advertise, so
 * the retained length is cleared until a whole set of fragments lands.
 *
 * @return	true when every fragment was accepted.
 */
bool BtAdvPeriodicDataSet(const uint8_t *pData, size_t Len);

/**
 * @brief	Read back the periodic advertising data last accepted.
 *
 * @return	Octets held. Nothing is copied when pBuff is NULL or too small.
 */
size_t BtAdvPeriodicDataGet(uint8_t *pBuff, size_t BuffLen);

/**
 * @brief	Start the periodic advertising train and the set that points at it.
 *
 * A train does not go out until its advertising set is enabled, so both are
 * turned on. If the set refuses, the train is turned back off rather than left
 * running with nothing advertising it.
 */
bool BtAdvPeriodicStart(void);

/**
 * @brief	Stop the periodic advertising train and its advertising set.
 *
 * Disabling the set alone does not stop a train that is already running, so
 * both are turned off. The set is stopped even when the train refuses.
 */
bool BtAdvPeriodicStop(void);

//!< Is a periodic advertising train running?
bool BtAdvPeriodicIsRunning(void);

// --- Synchronising to someone else's train, Core Vol 4 Part E 7.8.67 to 7.8.69 ---

//!< Skip range, periodic advertising events that may be missed after a receive.
#define BT_ADV_PERIODIC_SKIP_MAX			0x01F3

//!< Sync timeout range, 10 ms units, 100 ms to 163.84 s.
#define BT_ADV_PERIODIC_SYNC_TIMEOUT_MIN	0x000A
#define BT_ADV_PERIODIC_SYNC_TIMEOUT_MAX	0x4000

//!< Largest advertising set id a train can use.
#define BT_ADV_PERIODIC_SID_MAX				0x0F

//!< Data_Status in a periodic advertising report, 7.7.65.15.
#define BT_ADV_PERIODIC_DATA_COMPLETE		0x00
#define BT_ADV_PERIODIC_DATA_MORE			0x01
#define BT_ADV_PERIODIC_DATA_TRUNCATED		0x02
#define BT_ADV_PERIODIC_DATA_RX_FAILED		0xFF

typedef struct __Bt_Adv_Periodic_Sync_Cfg {
	uint8_t  AdvSid;			//!< Advertising set id of the train, 0 to 15
	uint8_t  AdvAddrType;		//!< 0 public, 1 random
	uint8_t  AdvAddr[6];		//!< Advertiser address
	uint16_t Skip;				//!< Events that may be skipped, 0 to 0x01F3
	uint16_t SyncTimeout;		//!< 10 ms units, 0x000A to 0x4000
	bool     DisableReporting;	//!< Synchronise without receiving reports
} BtAdvPeriodicSyncCfg_t;

/**
 * @brief	Ask the controller to synchronise to a periodic advertising train.
 *
 * The train is named by advertiser address and set id. Completion arrives as
 * BtAdvPeriodicSyncEstablished, which may report a failure status, so a true
 * return means the request was accepted rather than that a sync exists.
 *
 * Scanning has to be running for the controller to find the train.
 *
 * @return	true when the controller accepted the request.
 */
bool BtAdvPeriodicSyncCreate(BtHciDevice_t * const pDev,
	const BtAdvPeriodicSyncCfg_t *pCfg);

/**
 * @brief	Cancel a synchronisation request that has not completed.
 *
 * Refused by the controller when no request is outstanding.
 */
bool BtAdvPeriodicSyncCancel(BtHciDevice_t * const pDev);

/**
 * @brief	Drop an established synchronisation.
 *
 * No Sync Lost event follows a terminate, so the caller owns the handle's fate
 * from here.
 */
bool BtAdvPeriodicSyncTerminate(BtHciDevice_t * const pDev, uint16_t SyncHdl);

/**
 * @brief	Synchronisation attempt finished.
 *
 * Weak, override to be told. Status is zero on success; every other value
 * means no train was joined and SyncHdl is meaningless.
 */
void BtAdvPeriodicSyncEstablished(uint8_t Status, uint16_t SyncHdl,
	uint8_t AdvSid, uint8_t AdvAddrType, const uint8_t AdvAddr[6],
	uint8_t AdvPhy, uint16_t Interval);

/**
 * @brief	A whole periodic advertising payload arrived.
 *
 * Weak, override to be told. Reassembled across however many reports the
 * controller needed. A payload the controller marked truncated is dropped
 * rather than reported short.
 */
void BtAdvPeriodicSyncReport(uint16_t SyncHdl, int8_t TxPower, int8_t Rssi,
	size_t Len, const uint8_t *pData);

//!< A synchronisation was lost. Weak, override to be told. The handle is dead.
void BtAdvPeriodicSyncLost(uint16_t SyncHdl);

// Seams the HCI event path calls. bt_hci_host declares each weak and empty so
// a build without the periodic advertising module links; bt_adv_periodic_hci
// overrides them, tracks the sync handle and reassembly, then hands the result
// to the three callbacks above.
void BtAdvPeriodicSyncEstablishedEvt(uint8_t Status, uint16_t SyncHdl,
	uint8_t AdvSid, uint8_t AdvAddrType, const uint8_t AdvAddr[6],
	uint8_t AdvPhy, uint16_t Interval);
void BtAdvPeriodicReportFragment(uint16_t SyncHdl, int8_t TxPower, int8_t Rssi,
	uint8_t DataStatus, size_t Len, const uint8_t *pData);
void BtAdvPeriodicSyncLostEvt(uint16_t SyncHdl);

// --- Periodic Advertising with Responses, Core 5.4 ---

//!< Subevents a PAwR train may have, Core Vol 4 Part E 7.8.61.
#define BT_ADV_PAWR_SUBEVENT_MAX			0x80

//!< Subevent interval, 1.25 ms units, 7.5 ms to 318.75 ms.
#define BT_ADV_PAWR_SUBEVENT_INTERVAL_MIN	0x06

//!< Response slot delay, 1.25 ms units. Zero means no response slots.
#define BT_ADV_PAWR_RSP_SLOT_DELAY_MAX		0xFE

//!< Response slot spacing, 0.125 ms units. Zero means no response slots.
#define BT_ADV_PAWR_RSP_SLOT_SPACING_MIN	0x02

//!< Data octets one subevent can hold.
#define BT_ADV_PAWR_SUBEVENT_DATA_MAX		251

typedef struct __Bt_Adv_Pawr_Cfg {
	uint16_t IntervalMin;			//!< Periodic advertising interval min, 1.25 ms units
	uint16_t IntervalMax;			//!< Periodic advertising interval max, 1.25 ms units
	uint8_t  OwnAddrType;			//!< BTADDR_TYPE_PUBLIC or BTADDR_TYPE_RAND
	uint8_t  Sid;					//!< Advertising set id, 0 to 15
	bool     IncludeTxPower;		//!< Include TxPower in the periodic advertising PDU
	uint8_t  NumSubevents;			//!< Subevents per event, 1 to 0x80
	uint8_t  SubeventInterval;		//!< Time between subevents, 0x06 to 0xFF
	uint8_t  ResponseSlotDelay;		//!< 0 for no response slots, else 0x01 to 0xFE
	uint8_t  ResponseSlotSpacing;	//!< 0 for no response slots, else 0x02 to 0xFF
	uint8_t  NumResponseSlots;		//!< 0 for no response slots, else 0x01 to 0xFF
} BtAdvPawrCfg_t;

/**
 * @brief	Create the advertising set and configure a PAwR train.
 *
 * Periodic Advertising with Responses is a periodic advertising train divided
 * into subevents, each of which may open response slots the scanners answer
 * in. It uses the v2 form of the parameters, which is the only one with the
 * subevent fields, and the same advertising set the plain train uses.
 *
 * The four response slot parameters are all or nothing: either every one is
 * zero, meaning a train nobody answers, or all of them are in range.
 *
 * @return	true when the set and the train parameters were accepted.
 */
bool BtAdvPawrInit(BtHciDevice_t * const pDev, const BtAdvPawrCfg_t *pCfg);

/**
 * @brief	Fill one subevent with the data it will advertise.
 *
 * Answers a subevent data request. The response slot window named here is the
 * part of the subevent's slots this data invites answers in.
 *
 * @param	Subevent		Subevent to fill, below the configured count.
 * @param	RspSlotStart	First response slot the data invites answers in.
 * @param	RspSlotCount	Response slots the data invites answers in.
 * @param	pData			Subevent data, at most BT_ADV_PAWR_SUBEVENT_DATA_MAX.
 * @param	Len				Data length, may be zero.
 */
bool BtAdvPawrSubeventDataSet(uint8_t Subevent, uint8_t RspSlotStart,
	uint8_t RspSlotCount, const uint8_t *pData, size_t Len);

/**
 * @brief	The controller wants data for subevents it is about to advertise.
 *
 * Weak, override to be told. The handler is expected to call
 * BtAdvPawrSubeventDataSet for each subevent in the range, from
 * SubeventStart, wrapping at the configured subevent count.
 */
void BtAdvPawrSubeventDataRequest(uint8_t AdvHandle, uint8_t SubeventStart,
	uint8_t SubeventDataCount);

/**
 * @brief	A scanner answered in a response slot.
 *
 * Weak, override to be told. Called once per response in a report. A response
 * the controller marked incomplete is not delivered.
 */
void BtAdvPawrResponse(uint8_t AdvHandle, uint8_t Subevent,
	uint8_t ResponseSlot, int8_t Rssi, size_t Len, const uint8_t *pData);

// Event seams, weak and empty in bt_hci_host, overridden by the module.
void BtAdvPawrSubeventDataRequestEvt(uint8_t AdvHandle, uint8_t SubeventStart,
	uint8_t SubeventDataCount);
void BtAdvPawrResponseReportEvt(uint8_t AdvHandle, uint8_t Subevent,
	uint8_t TxStatus, uint8_t NumResponses, const uint8_t *pResponses,
	size_t ResponsesLen);

// --- PAwR scanner ---

//!< Subevents one Set Periodic Sync Subevent command may name.
#define BT_ADV_PAWR_SYNC_SUBEVENT_MAX		0x80

//!< Largest subevent number a scanner may synchronise with.
#define BT_ADV_PAWR_SUBEVENT_NUM_MAX		0x7F

//!< Data octets one response slot can hold.
#define BT_ADV_PAWR_RESPONSE_DATA_MAX		251

/**
 * @brief	Choose which subevents of a PAwR train to listen to.
 *
 * Core Vol 4 Part E 7.8.87. The controller stops listening to any subevent not
 * named here, so this is the whole subset each time rather than an addition.
 *
 * @param	SyncHdl		Train from BtAdvPeriodicSyncEstablished.
 * @param	pSubevents	Subevent numbers, each 0 to 0x7F.
 * @param	Count		How many, 1 to 0x80.
 */
bool BtAdvPawrSyncSubeventSet(BtHciDevice_t * const pDev, uint16_t SyncHdl,
	const uint8_t *pSubevents, uint8_t Count);

/**
 * @brief	Answer a PAwR advertiser in one response slot.
 *
 * Core Vol 4 Part E 7.8.88. The request event and subevent identify the packet
 * being answered and come from the report that delivered it. Data for a slot
 * is transmitted once.
 *
 * @param	SyncHdl				Train the answer belongs to.
 * @param	RequestEvent		Event counter the report carried.
 * @param	RequestSubevent		Subevent the report arrived in.
 * @param	ResponseSubevent	Subevent to answer in.
 * @param	ResponseSlot		Response slot to answer in.
 * @param	pData				Response data, may be NULL when Len is zero.
 * @param	Len					At most BT_ADV_PAWR_RESPONSE_DATA_MAX.
 */
bool BtAdvPawrResponseDataSet(BtHciDevice_t * const pDev, uint16_t SyncHdl,
	uint16_t RequestEvent, uint8_t RequestSubevent, uint8_t ResponseSubevent,
	uint8_t ResponseSlot, const uint8_t *pData, size_t Len);

/**
 * @brief	A subevent of a synchronised PAwR train delivered data.
 *
 * Weak, override to be told. EventCounter and Subevent are what a reply has to
 * quote back through BtAdvPawrResponseDataSet. Reassembled across reports; an
 * incomplete or truncated payload is not delivered.
 */
void BtAdvPawrSubeventReport(uint16_t SyncHdl, uint16_t EventCounter,
	uint8_t Subevent, int8_t Rssi, size_t Len, const uint8_t *pData);

// Event seam, weak and empty in bt_hci_host, overridden by the module.
void BtAdvPawrSubeventReportEvt(uint16_t SyncHdl, uint16_t EventCounter,
	uint8_t Subevent, int8_t TxPower, int8_t Rssi, uint8_t DataStatus,
	size_t Len, const uint8_t *pData);

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

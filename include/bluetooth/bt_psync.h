/**-------------------------------------------------------------------------
@file	bt_psync.h

@brief	Bluetooth LE periodic advertising synchronization, receiving side

The advertiser transmits a periodic advertising train (bt_padv.h). This is the
other end: an observer synchronizes to a train and receives its data.

Split the same way the ordinary advertising path is. The commands live in
bt_psync_hci.cpp, which needs an HCI device to reach the controller. The event
handling, the report reassembly and the application hooks live in bt_psync.cpp,
which needs neither, so the HCI event dispatcher can call into it without
dragging the application state in.

Core spec Vol 4 Part E: commands 7.8.67 to 7.8.73 and 7.8.88, events 7.7.65.14
to 7.7.65.16.

@author	Hoang Nguyen Hoan
@date	Aug. 15, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#ifndef __BT_PSYNC_H__
#define __BT_PSYNC_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Options of LE Periodic Advertising Create Sync, Vol 4 Part E 7.8.67.
#define BTPSYNC_OPT_USE_LIST				(1<<0)	//!< Use the Periodic Advertiser List, ignore the SID and address
#define BTPSYNC_OPT_REPORTING_DISABLED		(1<<1)	//!< Reporting starts disabled
#define BTPSYNC_OPT_DUPLICATE_FILTER		(1<<2)	//!< Duplicate filtering starts enabled

/// Sync_CTE_Type bits. Each one refuses a Constant Tone Extension kind; zero
/// means the presence or absence of one is irrelevant.
#define BTPSYNC_CTE_NO_AOA					(1<<0)
#define BTPSYNC_CTE_NO_AOD_1US				(1<<1)
#define BTPSYNC_CTE_NO_AOD_2US				(1<<2)
#define BTPSYNC_CTE_NO_TYPE3				(1<<3)
#define BTPSYNC_CTE_NO_CTE					(1<<4)
#define BTPSYNC_CTE_ALL						0x1F	//!< Every non-reserved bit, which the controller refuses

/// Parameter ranges, Vol 4 Part E 7.8.67 and 7.8.69.
#define BTPSYNC_SID_MAX						0x0F
#define BTPSYNC_SKIP_MAX					0x01F3
#define BTPSYNC_TIMEOUT_MIN					0x000A	//!< 100 ms in 10 ms units
#define BTPSYNC_TIMEOUT_MAX					0x4000	//!< 163.84 s in 10 ms units
#define BTPSYNC_HDL_MAX						0x0EFF	//!< Sync_Handle, 12 bits meaningful

/// Advertiser_Address_Type of the create command. The event form also uses
/// 0x02 and 0x03 for the resolved identity kinds, which the command does not.
#define BTPSYNC_ADDR_PUBLIC					0x00
#define BTPSYNC_ADDR_RANDOM					0x01

/// Data_Status of the report, shared with the extended advertising report.
#define BTPSYNC_DATA_COMPLETE				0
#define BTPSYNC_DATA_MORE					1
#define BTPSYNC_DATA_TRUNCATED				2

/// Largest reassembled report this layer will hold.
#define BTPSYNC_REPORT_MAX					512


// --- Periodic Advertising with Responses, receiving side ---

/// Periodic_Advertising_Properties of LE Set Periodic Sync Subevent, Vol 4
/// Part E 7.8.127. Bit 6 is the only one defined.
#define BTPSYNC_PROP_TXPWR					(1<<6)

/// Num_Subevents range of LE Set Periodic Sync Subevent, Vol 4 Part E 7.8.127.
#define BTPSYNC_SUBEVENT_COUNT_MAX			0x80

/// Subevent index range, Vol 4 Part E 7.8.127.
#define BTPSYNC_SUBEVENT_MAX				0x7F

/// Num_Responses range of the Response Report event, Vol 4 Part E 7.7.65.37.
#define BTPSYNC_RESPONSE_MAX				0x19

/// Tx_Status of the Response Report event, Vol 4 Part E 7.7.65.37.
#define BTPSYNC_TX_STATUS_SENT				0x00
#define BTPSYNC_TX_STATUS_NOT_SENT			0x01

/// Response_Data_Length range, Vol 4 Part E 7.8.126.
#define BTPSYNC_RESPONSE_DATA_MAX			251

#pragma pack(push, 4)

/// Create Sync parameters. AdvSid, AdvAddrType and AdvAddr are ignored by the
/// controller when BTPSYNC_OPT_USE_LIST is set.
typedef struct __Bt_Psync_Cfg {
	uint8_t Options;				//!< BTPSYNC_OPT_* bits
	uint8_t AdvSid;					//!< Advertising SID, 0 to 0x0F
	uint8_t AdvAddrType;			//!< BTPSYNC_ADDR_PUBLIC or BTPSYNC_ADDR_RANDOM
	uint8_t AdvAddr[6];				//!< Advertiser address
	uint16_t Skip;					//!< Periodic events that may be skipped, 0 to 0x01F3
	uint16_t SyncTimeout;			//!< 10 ms units, 0x000A to 0x4000
	uint8_t SyncCteType;			//!< BTPSYNC_CTE_* bits
} BtPsyncCfg_t;

/// What a Sync Established event reported.
typedef struct __Bt_Psync_Info {
	uint8_t Status;					//!< 0 on success, otherwise a controller error code
	uint16_t SyncHdl;				//!< Handle naming the train in later commands and events
	uint8_t AdvSid;					//!< Advertising SID of the train
	uint8_t AdvAddrType;			//!< Advertiser address type, 0 to 3
	uint8_t AdvAddr[6];				//!< Advertiser address
	uint8_t AdvPhy;					//!< PHY the periodic advertising uses
	uint16_t Interval;				//!< Periodic advertising interval, 1.25 ms units
	uint8_t ClockAccuracy;			//!< Advertiser clock accuracy
} BtPsyncInfo_t;

/// One complete periodic advertising report.
///
/// EvtCounter and Subevent are only present on the V2 form of the event, which
/// a PAwR train uses, and bSubevent says whether they were reported. They are
/// what BtPsyncResponseSet names as ReqEvent and ReqSubevent, so answering a
/// subevent is only possible from a report that has them.
typedef struct __Bt_Psync_Report_Info {
	uint16_t SyncHdl;				//!< Train the report belongs to
	int8_t TxPwr;					//!< Transmit power, 0x7F when not available
	int8_t Rssi;					//!< Received signal strength, 0x7F when not available
	uint8_t CteType;				//!< Constant Tone Extension type, 0xFF for none
	bool bSubevent;					//!< EvtCounter and Subevent below are meaningful
	uint16_t EvtCounter;			//!< paEventCounter the advertisement arrived in
	uint8_t Subevent;				//!< Subevent the advertisement arrived in
	const uint8_t *pData;			//!< Reassembled advertising data
	uint16_t Len;					//!< Length of pData in bytes
} BtPsyncReportInfo_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

//
// Commands, bt_psync_hci.cpp
//

/**
 * @brief	Start synchronizing to a periodic advertising train
 *
 * Issues LE Periodic Advertising Create Sync, which the controller answers
 * with Command Status rather than Command Complete. The Sync Established
 * event follows when the first packet arrives, and reaches the application
 * through BtPsyncEstablished.
 *
 * Synchronization only happens while scanning is enabled, though the command
 * may be issued either way.
 *
 * Parameters outside the ranges Vol 4 Part E 7.8.67 gives are refused before
 * the command goes out, including a Sync_CTE_Type with every non-reserved bit
 * set, which the controller answers with Command Disallowed.
 *
 * @param	pCfg	: Synchronization parameters
 *
 * @return	true when the controller accepted the command
 */
bool BtPsyncCreate(const BtPsyncCfg_t * const pCfg);

/**
 * @brief	Cancel a pending Create Sync
 *
 * Only legal while a Create Sync is pending; the controller answers Command
 * Disallowed otherwise. On success it then sends a Sync Established event
 * with status Operation Cancelled by Host (0x44), so the application sees the
 * attempt end through the same path a failure takes.
 *
 * @return	true when the controller accepted the command
 */
bool BtPsyncCreateCancel(void);

/**
 * @brief	Stop receiving a periodic advertising train
 *
 * The handle is destroyed on success, so nothing further may name it. Any
 * partial report held for it is dropped.
 *
 * @param	SyncHdl	: Handle from the Sync Established event
 *
 * @return	true when the controller accepted the command
 */
bool BtPsyncTerminate(uint16_t SyncHdl);

/**
 * @brief	Add an advertiser to the Periodic Advertiser List
 *
 * @param	AddrType	: BTPSYNC_ADDR_PUBLIC or BTPSYNC_ADDR_RANDOM
 * @param	Addr		: Advertiser address
 * @param	Sid			: Advertising SID, 0 to 0x0F
 *
 * @return	true when the controller accepted the command
 */
bool BtPsyncListAdd(uint8_t AddrType, const uint8_t Addr[6], uint8_t Sid);

/**
 * @brief	Remove an advertiser from the Periodic Advertiser List
 *
 * @param	AddrType	: BTPSYNC_ADDR_PUBLIC or BTPSYNC_ADDR_RANDOM
 * @param	Addr		: Advertiser address
 * @param	Sid			: Advertising SID, 0 to 0x0F
 *
 * @return	true when the controller accepted the command
 */
bool BtPsyncListRemove(uint8_t AddrType, const uint8_t Addr[6], uint8_t Sid);

/**
 * @brief	Empty the Periodic Advertiser List
 *
 * @return	true when the controller accepted the command
 */
bool BtPsyncListClear(void);

/**
 * @brief	Read how many entries the Periodic Advertiser List holds
 *
 * @param	pSize	: Receives the list size
 *
 * @return	true when the controller accepted the command
 */
bool BtPsyncListSizeGet(uint8_t *pSize);

/**
 * @brief	Turn reporting on or off for a synchronized train
 *
 * A train created with BTPSYNC_OPT_REPORTING_DISABLED stays synchronized
 * without delivering reports until this enables them.
 *
 * @param	SyncHdl	: Handle from the Sync Established event
 * @param	Enable	: true to receive reports
 *
 * @return	true when the controller accepted the command
 */
bool BtPsyncReceiveEnable(uint16_t SyncHdl, bool Enable);

//
// Event handling, bt_psync.cpp. Called by the HCI event dispatcher with the
// LE meta event payload after the subevent code, and the octets remaining.
//

void BtPsyncEvtEstablished(const uint8_t *pData, int Len, bool bV2);
void BtPsyncEvtReport(const uint8_t *pData, int Len, bool bV2);
void BtPsyncEvtLost(const uint8_t *pData, int Len);

/**
 * @brief	Drop any partial report held for a train
 *
 * Called when the train ends, so a later train given the same handle does not
 * inherit a fragment of this one.
 *
 * @param	SyncHdl	: Handle whose state is dropped
 */
void BtPsyncReasmReset(uint16_t SyncHdl);

//
// Application hooks, weak. Override to receive.
//

/**
 * @brief	A synchronization attempt ended
 *
 * Status 0 means synchronized and pInfo describes the train. Any other status
 * means the attempt failed or was cancelled, and only Status is meaningful.
 *
 * @param	pInfo	: What the event reported
 */
void BtPsyncEstablished(const BtPsyncInfo_t * const pInfo);

/**
 * @brief	A complete periodic advertising report arrived
 *
 * pRep->pData is the reassembled data of one advertisement. A report the
 * controller could not complete is not delivered.
 *
 * On a PAwR train pRep also names the subevent and the event counter the
 * advertisement arrived in, which is what BtPsyncResponseSet needs to answer
 * it. The response slot has to be chosen by the application, since the train
 * decides what a slot means.
 *
 * @param	pRep	: What the report carried
 */
void BtPsyncReport(const BtPsyncReportInfo_t * const pRep);

/**
 * @brief	Synchronization with a train was lost
 *
 * The handle is destroyed by this event and may not be named again.
 *
 * @param	SyncHdl	: Train that was lost
 */
void BtPsyncLost(uint16_t SyncHdl);

/**
 * @brief	Synchronize with a subset of the subevents of a PAwR train
 *
 * Issues LE Set Periodic Sync Subevent (Vol 4 Part E 7.8.127). The list
 * replaces whatever the controller was synchronized to: any subevent not
 * named here is dropped, so this is not an add.
 *
 * @param	SyncHdl		: Train to act on
 * @param	Properties	: BTPSYNC_PROP_* bits for the response PDUs
 * @param	pSubevents	: Subevent indices to synchronize with
 * @param	NbSubevents	: Number of entries, 1 to BTPSYNC_SUBEVENT_COUNT_MAX
 *
 * @return	true on success
 */
bool BtPsyncSubeventSet(uint16_t SyncHdl, uint16_t Properties,
						const uint8_t *pSubevents, uint8_t NbSubevents);

/**
 * @brief	Answer a PAwR subevent in one of its response slots
 *
 * Issues LE Set Periodic Advertising Response Data (Vol 4 Part E 7.8.126).
 * The data for a response slot is transmitted once, and a slot that has
 * already passed is answered Too Late, so this belongs in the report handler
 * rather than deferred.
 *
 * @param	SyncHdl		: Train the subevent belongs to
 * @param	ReqEvent	: paEventCounter of the packet being answered
 * @param	ReqSubevent	: Subevent the packet was received in
 * @param	RspSubevent	: Subevent to answer in
 * @param	RspSlot		: Response slot to answer in
 * @param	pData		: Response data, may be null only when Len is 0
 * @param	Len			: Response data length, 0 to 251
 *
 * @return	true on success
 */
bool BtPsyncResponseSet(uint16_t SyncHdl, uint16_t ReqEvent,
						uint8_t ReqSubevent, uint8_t RspSubevent,
						uint8_t RspSlot, const uint8_t *pData, uint8_t Len);

/**
 * @brief	A device answered a PAwR subevent this device advertised
 *
 * Weak, called from the LE Periodic Advertising Response Report event (Vol 4
 * Part E 7.7.65.37) once a response is whole. A response split across several
 * reports is reassembled first, and a response whose reassembly was abandoned
 * is not delivered at all.
 *
 * @param	AdvHdl		: Advertising set the train is on
 * @param	Subevent	: Subevent the response was received in
 * @param	RspSlot		: Response slot the response arrived in
 * @param	TxPwr		: Transmit power, 0x7F when not available
 * @param	Rssi		: RSSI, 0x7F when not available
 * @param	pData		: Response data
 * @param	Len			: Response data length
 */
void BtPsyncResponseReport(uint8_t AdvHdl, uint8_t Subevent, uint8_t RspSlot,
						   int8_t TxPwr, int8_t Rssi, const uint8_t *pData,
						   uint16_t Len);

/**
 * @brief	A PAwR subevent this device advertised was not transmitted
 *
 * Weak. Vol 4 Part E 7.7.65.37 lets the controller report that it failed to
 * send the AUX_SYNC_SUBEVENT_IND that would have let responders answer, so
 * silence in that subevent means nothing about the responders.
 *
 * @param	AdvHdl		: Advertising set the train is on
 * @param	Subevent	: Subevent that was not transmitted
 */
void BtPsyncSubeventNotSent(uint8_t AdvHdl, uint8_t Subevent);

/**
 * @brief	Handle an LE Periodic Advertising Subevent Data Request event
 *
 * @param	pData	: Event payload after the subevent code
 * @param	Len		: Payload length
 */
void BtPsyncEvtDataRequest(const uint8_t *pData, int Len);

/**
 * @brief	Handle an LE Periodic Advertising Response Report event
 *
 * @param	pData	: Event payload after the subevent code
 * @param	Len		: Payload length
 */
void BtPsyncEvtResponseReport(const uint8_t *pData, int Len);

#ifdef __cplusplus
}
#endif

#endif // __BT_PSYNC_H__

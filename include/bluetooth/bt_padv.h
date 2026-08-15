/**-------------------------------------------------------------------------
@file	bt_padv.h

@brief	Bluetooth LE periodic advertising, advertiser side

Periodic advertising rides on an extended advertising set. The set is
configured first with the ordinary advertising path, then this layer adds the
periodic train to it: parameters, data, and enable, matching Core spec Vol 4
Part E sections 7.8.61, 7.8.62 and 7.8.63.

The set the train is attached to has to be extended and non-connectable,
non-scannable and non-anonymous. A controller answers a train on a set that is
any of those with Invalid HCI Command Parameters on the parameters command
and Command Disallowed on enable.

Periodic advertising and the advertising set are enabled independently. The
train does not go on air until the advertising set is also enabled, and
disabling the set afterwards does not stop the train.

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
#ifndef __BT_PADV_H__
#define __BT_PADV_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Periodic_Advertising_Properties, Vol 4 Part E 7.8.61. Bit 6 is the only
/// one defined; every other bit is reserved.
#define BTPADV_PROP_TXPWR					(1<<6)

/// Periodic_Advertising_Interval_Min and _Max are in 1.25 ms units with a
/// range of 0x0006 to 0xFFFF, so 7.5 ms to 81.91875 s (Vol 4 Part E 7.8.61).
#define BTPADV_INTERVAL_MIN					6
#define BTPADV_INTERVAL_MAX					0xFFFF

/// Largest Advertising_Data_Length one LE Set Periodic Advertising Data
/// command accepts (Vol 4 Part E 7.8.62). Longer data is fragmented.
#define BTPADV_DATA_FRAG_MAX				252

/// Advertising_Handle range, Vol 4 Part E 7.8.61.
#define BTPADV_ADV_HDL_MAX					0xEF

// --- Periodic Advertising with Responses ---

/// Num_Subevents range in the [v2] parameters command, Vol 4 Part E 7.8.61.
/// Zero means the train has no subevents, which is plain periodic
/// advertising, and the other four PAwR parameters are then ignored.
#define BTPADV_SUBEVENT_COUNT_MAX			0x80

/// Subevent index range, Vol 4 Part E 7.8.61 and 7.8.125.
#define BTPADV_SUBEVENT_MAX					0x7F

/// Num_Subevents range of one LE Set Periodic Advertising Subevent Data
/// command, Vol 4 Part E 7.8.125. Narrower than the train can have, so a
/// train with more subevents needs several commands.
#define BTPADV_SUBEVENT_DATA_MAX			0x0F

/// Subevent_Data_Length range of one subevent, Vol 4 Part E 7.8.125.
#define BTPADV_SUBEVENT_DATA_LEN_MAX		251

#pragma pack(push, 4)

/// Periodic advertising train configuration.
///
/// NbSubevents zero configures a plain periodic advertising train through the
/// [v1] command. Anything else configures Periodic Advertising with
/// Responses through [v2], which is a separate opcode rather than a version
/// parameter, and the four fields below it then apply.
typedef struct __Bt_Padv_Cfg {
	uint8_t AdvHdl;					//!< Advertising set the train attaches to, 0 to 0xEF
	uint16_t IntervalMin;			//!< Periodic advertising interval min, 1.25 ms units
	uint16_t IntervalMax;			//!< Periodic advertising interval max, 1.25 ms units
	uint16_t Properties;			//!< BTPADV_PROP_* bits
	uint8_t NbSubevents;			//!< PAwR subevents per periodic advertising event, 0 for none
	uint8_t SubeventInterval;		//!< Time between subevents, 1.25 ms units
	uint8_t RspSlotDelay;			//!< Subevent start to first response slot, 1.25 ms units
	uint8_t RspSlotSpacing;			//!< Time between response slots, 0.125 ms units
	uint8_t NbRspSlots;				//!< Response slots per subevent
} BtPadvCfg_t;

/// One subevent's worth of data for LE Set Periodic Advertising Subevent
/// Data. The command takes an array of these, interleaved on the wire.
typedef struct __Bt_Padv_Subevent_Data {
	uint8_t Subevent;				//!< Subevent index, 0 to 0x7F
	uint8_t RspSlotStart;			//!< First response slot used in this subevent
	uint8_t RspSlotCount;			//!< Response slots used in this subevent
	uint8_t DataLen;				//!< Significant octets of pData, 0 to 251
	const uint8_t *pData;			//!< Advertising data for this subevent
} BtPadvSubeventData_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Configure the periodic advertising train on an advertising set
 *
 * Issues LE Set Periodic Advertising Parameters. The advertising set named by
 * pCfg->AdvHdl must already exist, which means the ordinary advertising path
 * has run its LE Set Extended Advertising Parameters for that handle.
 *
 * Refused before the command goes out when the interval range is outside the
 * spec range or reversed, since a controller answers those with an error the
 * caller then has to map back to a parameter.
 *
 * @param	pCfg	: Train configuration
 *
 * @return	true on success
 */
bool BtPadvInit(const BtPadvCfg_t * const pCfg);

/**
 * @brief	Set the periodic advertising data
 *
 * Issues one LE Set Periodic Advertising Data with the complete operation
 * when the data fits a single command, or a first / intermediate / last
 * sequence when it does not.
 *
 * Fragmenting is only legal while the train is disabled: the controller
 * answers any operation other than complete or unchanged with Command
 * Disallowed once periodic advertising is enabled (Vol 4 Part E 7.8.62). Data
 * that needs fragmenting is refused here rather than sent and half rejected,
 * which would leave the controller holding a partial set.
 *
 * Len 0 clears the data.
 *
 * @param	AdvHdl	: Advertising set the train is on
 * @param	pData	: Data to advertise, may be null only when Len is 0
 * @param	Len		: Data length in bytes
 *
 * @return	true on success
 */
bool BtPadvDataSet(uint8_t AdvHdl, const uint8_t *pData, size_t Len);

/**
 * @brief	Update the Advertising DID without changing the data
 *
 * Issues LE Set Periodic Advertising Data with the unchanged operation, which
 * the controller answers by re-sending the data it holds with a new DID. The
 * train has to be enabled and already hold data, or the controller answers
 * Invalid HCI Command Parameters (Vol 4 Part E 7.8.62).
 *
 * @param	AdvHdl	: Advertising set the train is on
 *
 * @return	true on success
 */
bool BtPadvDataRefresh(uint8_t AdvHdl);

/**
 * @brief	Start the periodic advertising train
 *
 * The train does not reach the air until the advertising set itself is
 * enabled, which is what BtAppAdvStart does.
 *
 * @param	AdvHdl	: Advertising set the train is on
 *
 * @return	true on success
 */
bool BtPadvStart(uint8_t AdvHdl);

/**
 * @brief	Stop the periodic advertising train
 *
 * Leaves the advertising set itself alone.
 *
 * @param	AdvHdl	: Advertising set the train is on
 *
 * @return	true on success
 */
bool BtPadvStop(uint8_t AdvHdl);

/**
 * @brief	Whether the train is enabled as far as this layer knows
 *
 * Tracks the enable and disable commands this layer issued. The controller
 * can stop a train for reasons the host does not see, so this answers what was
 * asked for, not what is on air.
 *
 * @param	AdvHdl	: Advertising set the train is on
 *
 * @return	true when the last accepted command on this handle was an enable
 */
bool BtPadvIsEnabled(uint8_t AdvHdl);

/**
 * @brief	Supply the data for one or more PAwR subevents
 *
 * Issues LE Set Periodic Advertising Subevent Data (Vol 4 Part E 7.8.125).
 * This answers a Subevent Data Request and has to reach the controller within
 * the subevent interval: a subevent already gone is answered Too Late and one
 * too far ahead Too Early, and in both cases the data is discarded.
 *
 * The arrayed parameters interleave on the wire, one record per subevent,
 * which is the ordering Vol 4 Part E 5.4.1 states for arrayed parameters.
 *
 * The data for a subevent is transmitted once, so this is called again for
 * every request.
 *
 * @param	AdvHdl		: Advertising set the train is on
 * @param	pSubevents	: Per subevent data, NbSubevents entries
 * @param	NbSubevents	: Number of entries, 1 to BTPADV_SUBEVENT_DATA_MAX
 *
 * @return	true on success
 */
bool BtPadvSubeventDataSet(uint8_t AdvHdl,
						   const BtPadvSubeventData_t * const pSubevents,
						   uint8_t NbSubevents);

/**
 * @brief	The controller is ready for the data of one or more subevents
 *
 * Weak, called from the LE Periodic Advertising Subevent Data Request event
 * (Vol 4 Part E 7.7.65.36). The application answers with
 * BtPadvSubeventDataSet for the subevents named here and no others: a
 * subevent outside the requested range is answered Command Disallowed.
 *
 * The subevent numbers wrap, from one less than the number of subevents in
 * the train back to zero, so the requested set is not simply Start to
 * Start + Count. BtPadvSubeventOfRequest walks it correctly.
 *
 * The default does nothing, which leaves those subevents carrying no data.
 *
 * @param	AdvHdl	: Advertising set the request is for
 * @param	Start	: First subevent data is requested for
 * @param	Count	: Number of subevents data is requested for
 */
void BtPadvSubeventDataRequest(uint8_t AdvHdl, uint8_t Start, uint8_t Count);

/**
 * @brief	Subevents the train was last configured with
 *
 * The Subevent Data Request event names its subevents modulo this, so the
 * wrap cannot be resolved from the event alone.
 *
 * @return	Num_Subevents of the last accepted configuration, 0 for a plain
 *			periodic advertising train
 */
uint8_t BtPadvNbSubevents(void);

/**
 * @brief	The Nth subevent of a data request, accounting for the wrap
 *
 * @param	Start		: Subevent_Start from the request
 * @param	Idx			: Which of the requested subevents, 0 to Count-1
 * @param	NbSubevents	: Subevents the train was configured with
 *
 * @return	The subevent index, or 0xFF when NbSubevents is 0
 */
static inline uint8_t BtPadvSubeventOfRequest(uint8_t Start, uint8_t Idx,
											  uint8_t NbSubevents)
{
	if (NbSubevents == 0)
	{
		return 0xFF;
	}

	return (uint8_t)(((uint16_t)Start + Idx) % NbSubevents);
}

#ifdef __cplusplus
}
#endif

#endif // __BT_PADV_H__

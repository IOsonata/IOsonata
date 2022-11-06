/**-------------------------------------------------------------------------
@file	bt_gap.h

@brief	Bluetooth Generic Access Profile (GAP) definitions

Core_v5.3 - 2.1.1.7
The Generic Access Profile (GAP) block represents the base functionality common
to all Bluetooth devices such as modes and access procedures used by the
transports, protocols and application profiles. GAP services include device
discovery, connection modes, security, authentication, association models and
service discovery.

@author	Hoang Nguyen Hoan
@date	Sep. 29, 2022

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
#ifndef __BT_GAP_H__
#define __BT_GAP_H__

#include <stddef.h>

/** @addtogroup Bluetooth
 * @{ */

// Assigned numbers and GAP

#define BT_GAP_DATA_TYPE_FLAGS							1		//!< Flags
#define BT_GAP_DATA_TYPE_INCOMPLETE_SRVC_UUID16			2		//!< Incomplete list of 16 bits service class UUID
#define BT_GAP_DATA_TYPE_COMPLETE_SRVC_UUID16			3		//!< Complete list of 16 bits service class UUID
#define BT_GAP_DATA_TYPE_INCOMPLETE_SRVC_UUID32			4		//!< Incomplete list of 32 bits service class UUID
#define BT_GAP_DATA_TYPE_COMPLETE_SRVC_UUID32			5		//!< Complete list of 32 bits service class UUID
#define BT_GAP_DATA_TYPE_INCOMPLETE_SRVC_UUID128		6		//!< Incomplete list of 128 bits service class UUID
#define BT_GAP_DATA_TYPE_COMPLETE_SRVC_UUID128			7		//!< Complete list of 128 bits service class UUID
#define BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME				8		//!< Shortened local name
#define BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME			9		//!< Complete local name
#define BT_GAP_DATA_TYPE_TX_PWR_LEVEL					0xA		//!< Tx power level
#define BT_GAP_DATA_TYPE_DEVICE_CLASS					0xD		//!< Class of device
#define BT_GAP_DATA_TYPE_SIMPLE_PAIRING_HASH_C			0xE		//!< Simple pairing hash C
#define BT_GAP_DATA_TYPE_SIMPLE_PAIRING_HASH_C192		0xE		//!< Simple pairing hash C-192
#define BT_GAP_DATA_TYPE_SIMPLE_PAIRING_RANDOM_R		0xF		//!< Simple pairing randomizer R
#define BT_GAP_DATA_TYPE_SIMPLE_PAIRING_RANDOM_R192		0xF		//!< Simple pairing randomizer R-192
#define BT_GAP_DATA_TYPE_DEVICE_ID						0x10	//!< Device ID
#define BT_GAP_DATA_TYPE_SECURITY_MNGR_TKVALUE			0x10	//!< Security managerTK value
#define BT_GAP_DATA_TYPE_SECURITY_MNGR_OOB_FLAGS		0x11	//!< Security manager out of band flags
#define BT_GAP_DATA_TYPE_SLAVE_CONN_INTERVAL_RANGE		0x12	//!< Slave connection interval range
#define BT_GAP_DATA_TYPE_SRVC_SOLICITATION_UUID16		0x14	//!< List of 16 bits service solicitation UUID
#define BT_GAP_DATA_TYPE_SRVC_SOLICITATION_UUID128		0x15	//!< List of 128 bits service solicitation UUID
#define BT_GAP_DATA_TYPE_SRVC_DATA						0x16	//!< Service data
#define BT_GAP_DATA_TYPE_SRVC_DATA_UUID16				0x16	//!< Service data - 16 bits UUID
#define BT_GAP_DATA_TYPE_PUBLIC_TARGET_ADDR				0x17	//!< Public target address
#define BT_GAP_DATA_TYPE_RANDOM_TARGET_ADDR				0x18	//!< Random target address
#define BT_GAP_DATA_TYPE_APPEARANCE						0x19	//!< Appearance
#define BT_GAP_DATA_TYPE_ADV_INTERVAL					0x1A	//!< Advertising interval
#define BT_GAP_DATA_TYPE_LE_DEVICE_ADDR					0x1B	//!< LE Bluetooth device address
#define BT_GAP_DATA_TYPE_LE_ROLE 						0x1C	//!< LE role
#define BT_GAP_DATA_TYPE_SIMPLE_PAIRING_HASH_C256		0x1D	//!< Simple pairing hash C-256
#define BT_GAP_DATA_TYPE_SIMPLE_PAIRING_RANDOM_R256		0x1E	//!< Simple pairing randomizer R-256
#define BT_GAP_DATA_TYPE_SRVC_SOLICITATION_UUID32		0x1F	//!< List of 32 bits service solicitation UUID
#define BT_GAP_DATA_TYPE_SRVC_DATA_UUID32				0x20	//!< Service data 32 bits UUID
#define BT_GAP_DATA_TYPE_SRVC_DATA_UUID128				0x21	//!< Service data 128 bits UUID
#define BT_GAP_DATA_TYPE_LE_SECURE_CONN_CONFIRM_VAL		0x22	//!< LE secure connections confirmation value
#define BT_GAP_DATA_TYPE_LE_SECURE_CONN_RAND_VAL		0x23	//!< LE secure connections random value
#define BT_GAP_DATA_TYPE_URI							0x24	//!< URI
#define BT_GAP_DATA_TYPE_INDOOR_POSITIONING				0x25	//!< Indoor positioning
#define BT_GAP_DATA_TYPE_TRANSPORT_DISCOVERY_DATA		0x26	//!< Transport discovery data
#define BT_GAP_DATA_TYPE_LE_SUPP_FEATURES				0x27	//!< LE supported features
#define BT_GAP_DATA_TYPE_CHAN_MAP_UPDATE_IND			0x28	//!< Channel map update indication
#define BT_GAP_DATA_TYPE_PB_ADV							0x29	//!< PB-ADV	mesh profile specs section 5.2.1
#define BT_GAP_DATA_TYPE_MESH_MESSAGE					0x2A	//!< Mesh message
#define BT_GAP_DATA_TYPE_MESH_BEACON					0x2B	//!< Mesh beacon
#define BT_GAP_DATA_TYPE_BIG_INFO						0x2C	//!< BIG info
#define BT_GAP_DATA_TYPE_BROADCAST_CODE					0x2D	//!< Broadcast code
#define BT_GAP_DATA_TYPE_RESOLVABLE_SET_ID				0x2E	//!< Resolvable set identifier
#define BT_GAP_DATA_TYPE_ADV_INTERVAL_LONG				0x2F	//!< Advertising interval long
#define BT_GAP_DATA_TYPE_BROADCAST_NAME					0x30	//!< Broadcast name
#define BT_GAP_DATA_TYPE_3D_INFO_DATA					0x3D	//!< 3D synchronization profile
#define BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA			0xFF	//!< Manufacture specific data

/// Flags or-able
#define BT_GAP_DATA_TYPE_FLAGS_LIMITED_DISCOVERABLE		(1<<0)	//!< Flag LE limited discoverable
#define BT_GAP_DATA_TYPE_FLAGS_GENERAL_DISCOVERABLE		(1<<1)	//!< Flag LE general discoverable
#define BT_GAP_DATA_TYPE_FLAGS_NO_BREDR					(1<<2)	//!< Flag BR/EDR not supported
#define BT_GAP_DATA_TYPE_FLAGS_LE_BREDR_SUPPORT			(1<<3)	//!< Flag simultaneous support LE & BR/EDR

/// GAP roles or-able
#define	BT_GAP_ROLE_BROADCASTER							(1<<0)	//!< Optimize for transmitter, no connection support. Use advertising to broadcast data
#define	BT_GAP_ROLE_OBSERVER							(1<<1)	//!< Optimized for receiver, no connection support.
#define	BT_GAP_ROLE_PERIPHERAL							(1<<2)	//!< Optimized for device that supports a single connection
#define	BT_GAP_ROLE_CENTRAL								(1<<3)	//!< The initiator for all connection with devices. Support multiple connections.


#pragma pack(push, 4)

typedef struct __Bt_Gap_Connection {
	uint16_t Hdl;				//!< Connection handle
	uint8_t Role;
	uint8_t PeerAddrType;
	uint8_t PeerAddr[6];
} BtGapConnection_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

void BtGapInit();
bool isBtGapConnected();
uint16_t BtGapGetConnection();
size_t BtGapGetConnectedHandles(uint16_t *pHdl, size_t MaxCount);
bool BtGapAddConnection(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6]);
void BtGapDeleteConnection(uint16_t Hdl);

#ifdef __cplusplus
}
#endif

/** @} */

#endif // __BT_GAP_H__

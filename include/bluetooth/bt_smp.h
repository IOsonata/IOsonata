/**-------------------------------------------------------------------------
@file	bt_smp.h

@brief	Bluetooth Security Manger Protocol (SMP)  

Generic implementation & definitions of Bluetooth Security Manager Protocol

L2CAP channel 6

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2022

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
#ifndef __BT_SMP_H__
#define __BT_SMP_H__

#include <inttypes.h>
#include <stddef.h>

#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_dev.h"

#define BT_SMP_CODE_PAIRING_REQ						1
#define BT_SMP_CODE_PAIRING_RSP						2
#define BT_SMP_CODE_PAIRING_CONFIRM					3
#define BT_SMP_CODE_PAIRING_RANDOM					4
#define BT_SMP_CODE_PAIRING_FAILED					5
#define BT_SMP_CODE_PAIRING_ENCRYP_INFO				6
#define BT_SMP_CODE_PAIRING_CENTRAL_ID				7
#define BT_SMP_CODE_PAIRING_ID_INFO					8
#define BT_SMP_CODE_PAIRING_ID_ADDR_INFO			9
#define BT_SMP_CODE_PAIRING_SIGNING_INFO			0xA
#define BT_SMP_CODE_PAIRING_SECURITY_REQ			0xB
#define BT_SMP_CODE_PAIRING_PAIRING_PUBLIC_KEY		0xC
#define BT_SMP_CODE_PAIRING_PAIRING_DHKEY_CHECK		0xD
#define BT_SMP_CODE_PAIRING_PAIRING_KEYPRESS_NOTIF	0xE


#define BT_SMP_IOCAPS_DISPLAY_ONLY					0
#define BT_SMP_IOCAPS_DISPLAY_YESNO					1
#define BT_SMP_IOCAPS_KEYBOARD_ONLY					2
#define BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT			3
#define BT_SMP_IOCAPS_KEYBOARD_DISPLAY				4

#define BT_SMP_OOB_AUTH_NOT_PRESENT					0	//!< OOB Authentication data not present
#define BT_SMP_OOB_AUTH_PRESENT 					1	//!< OOB Authentication data from remote device present

#define BT_SMP_AUTHREQ_BONDING_FLAG_MASK			(3<<0)
#define BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING		(0<<0)	//!< No bonding
#define BT_SMP_AUTHREQ_BONDING_FLAG_BONDING			(1<<1)
#define BT_SMP_AUTHREQ_MITM							(1<<2)
#define BT_SMP_AUTHREQ_SC							(1<<3)	//!< LE secure connection
#define BT_SMP_AUTHREQ_KEYPRESS						(1<<4)	//!<
#define BT_SMP_AUTHREQ_CT2							(1<<5)	//!<
#define BT_SMP_AUTHREQ_RFU_MASK						(3<<6)	//!<


#pragma pack(push, 1)

typedef struct __Bt_Smp_Packet {
	uint8_t Code;
	uint8_t Data[64];			//!< Max data size is 65 byte secure or 23 non secure
} BtSmpPacket_t;

typedef struct __Bt_Smp_Paring_Req {
	uint8_t Code;				//!< SMP code
	uint8_t IOCaps;				//!< IO capability
	uint8_t OOBFlag;			//!< OOB data flag
	uint8_t AuthReq;			//!<
	uint8_t MaxKeySize;			//!< Max encryption key size in bytes
	uint8_t InitiatorKeyDist;	//!< Initiator key distribution
	uint8_t ResponderKeyDist;	//!< Responder key distribution
} BtSmpPairingReq_t;

typedef struct __Bt_Smp_Paring_Rsp {
	uint8_t Code;				//!< SMP code
	uint8_t IOCaps;				//!< IO capability
	uint8_t OOBFlag;			//!< OOB data flag
	uint8_t AuthReq;			//!<
	uint8_t MaxKeySize;			//!< Max encryption key size in bytes
	uint8_t InitiatorKeyDist;	//!< Initiator key distribution
	uint8_t ResponderKeyDist;	//!< Responder key distribution
} BtSmpPairingRsp_t;

typedef struct __Bt_Smp_Pairing_Confirm {
	uint8_t Code;			//!< SMP code
	uint8_t Value[16];
} BtSmpPairingConfirm_t;

#pragma pack(pop)


#ifdef __cplusplus
extern "C" {
#endif

void BtProcessSmpData(BtDev_t * const pDev, BtL2CapPdu_t * const pRcvPdu);

#ifdef __cplusplus
}
#endif

#endif // __BT_SMP_H__

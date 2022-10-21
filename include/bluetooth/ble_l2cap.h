/**-------------------------------------------------------------------------
@file	ble_l2cap.h

@brief	Generic Bluetooth L2CAP

Generic definitions for BLE L2CAP

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#ifndef __BLE_L2CAP_H__
#define __BLE_L2CAP_H__

#include <inttypes.h>

#define BLE_L2CAP_CID_FIXED_START			1
#define BLE_L2CAP_CID_FIXED_END				0x3F	//

#define BLE_L2CAP_CID_CONNECTIONLESS		2		//!< Connectionless
#define BLE_L2CAP_CID_ATT					4		//!< Attribute protocol
#define BLE_L2CAP_CID_SIGNAL				5		//!< LE signaling
#define BLE_L2CAP_CID_SEC_MNGR				6		//!< Security manager

#define BLE_L2CAP_CID_DYNA_START			0x40	//!< Start of dynamically allocated
#define BLE_L2CAP_CID_DYNA_END				0x7F	//!< End of dynamically allocated

typedef enum __Ble_L2CAP_Mode {
	L2CAP_MODE_BASIC,              //!< L2CAP_MODE_BASIC
	L2CAP_MODE_FLOW_CTRL,          //!< L2CAP_MODE_FLOW_CTRL
	L2CAP_MODE_RETRANSMISSION,     //!< L2CAP_MODE_RETRANSMISSION
	L2CAP_MODE_ENH_RETRANSMISSION, //!< L2CAP_MODE_ENH_RETRANSMISSION
	L2CAP_MODE_STREAMING,          //!< L2CAP_MODE_STREAMING
	L2CAP_MODE_LE_CREDIT_FLOW_CTRL,//!< L2CAP_MODE_LE_CREDIT_FLOW_CTRL
	L2CAP_MODE_ENH_CREDIT_FLOW_CTRL//!< L2CAP_MODE_ENH_CREDIT_FLOW_CTRL
} L2CAP_MODE;

#define L2CAP_SDU_SAR_UNSEGMENTED			0	//!< Unsegmented SDU
#define L2CAP_SDU_SAR_START					1	//!< Start of SDU
#define L2CAP_SDU_SAR_END					2	//!< End of SDU
#define L2CAP_SDU_SAR_CONT					3	//!< Continuation SDU

#define L2CAP_SUPER_RR						0	//!< Receiver ready
#define L2CAP_SUPER_REJ						1	//!< Reject
#define L2CAP_SUPER_RNR						2	//!< Receiver not ready
#define L2CAP_SUPER_SREJ					3	//!< Select reject

#define L2CAP_CONTROL_TYPE_MASK				(1<<0)
#define L2CAP_CONTROL_TYPE_SFRAME			(1<<0)

#pragma pack(push, 1)

typedef struct __Ble_L2Cap_Header {
	uint16_t Len;				//!< PDU length
	uint16_t Cid;				//!< CID - Channel ID
} BleL2CapHdr_t;

/// L2CAP Supervisory frame
/// NOTE: Variable length structure
typedef union __Ble_L2Cap_SFrame_Fields {
	// Standard control field
	struct {
		uint16_t Type:1;			//!< Frame type 0 - IFrame, 1 - SFrame
		uint16_t Rfu0:1;			//!< Reserved
		uint16_t Super:2;			//!< S - Supervisory function
		uint16_t Rfu1:3;			//!< Reserved
		uint16_t RetransDis:1;		//!< R - Retransmission disable bit
		uint16_t ReqSeq:6;			//!< Receive sequence number
		uint16_t Rfu2:2;			//!< Reserved
		uint16_t Fcs;				//!< Frame check sequence
	} Std;
	// Enhanced control field
	struct {
		uint16_t Type:1;			//!< Frame type 0 - IFrame, 1 - SFrame
		uint16_t Rfu0:1;			//!< Reserved
		uint16_t Super:2;			//!< S - Supervisory function
		uint16_t Poll:1;			//!< P - Solicit a response from receiver
		uint16_t Rfu1:2;			//!< Reserved
		uint16_t Final:1;			//!< F - Final set to 1 in response to a SFrame with Poll bit set
		uint16_t ReqSeq:6;			//!< Receive sequence number
		uint16_t Rfu2:2;			//!< Reserved
		uint16_t Fcs;				//!< Frame check sequence
	} Enh;
	// Extended control field
	struct {
		uint16_t Type:1;			//!< Frame type 0 - IFrame, 1 - SFrame
		uint16_t Final:1;			//!< F - Final set to 1 in response to a SFrame with Poll bit set
		uint16_t ReqSeq:14;			//!< Receive sequence number
		uint16_t Super:2;			//!< S - Supervisory function
		uint16_t Poll:1;			//!< P - Solicit a response from receiver
		uint16_t Rfu:13;			//!< Reserved
		uint16_t Fcs;				//!< Frame check sequence
	} Ext;
} BleL2CapSFrame_t;

/// L2CAP Information frame
/// NOTE: Variable length structure
typedef union __Ble_L2Cap_IFrame_Fields {
	// Standard control field
	struct {
		uint16_t Type:1;			//!< Frame type 0 - IFrame, 1 - SFrame
		uint16_t TxSeq:6;			//!< Tx sequence number
		uint16_t RetransDis:1;		//!< R - Retransmission disable bit
		uint16_t ReqSeq:6;			//!< Receive sequence number
		uint16_t Sar:2;				//!< Segmentation & Reassembly
		uint16_t Len;				//!< SDU length
		uint8_t Data[1];			//!< Information data
	} Std;
	// Enhanced control field
	struct {
		uint16_t Type:1;			//!< Frame type 0 - IFrame, 1 - SFrame
		uint16_t TxSeq:6;			//!< Tx sequence number
		uint16_t Final:1;			//!< F - Final set to 1 in response to a SFrame with Poll bit set
		uint16_t ReqSeq:6;			//!< Receive sequence number
		uint16_t Sar:2;				//!< Segmentation & Reassembly
		uint16_t Len;				//!< SDU length
		uint8_t Data[1];			//!< Information data
	} Enh;
	// Extended control field
	struct {
		uint16_t Type:1;			//!< Frame type 0 - IFrame, 1 - SFrame
		uint16_t Final:1;			//!< F - Final set to 1 in response to a SFrame with Poll bit set
		uint16_t ReqSeq:14;			//!< Receive sequence number
		uint16_t Sar:2;				//!< Segmentation & Reassembly
		uint16_t TxSeq:14;			//!< Tx sequence number
		uint16_t Len;				//!< SDU length
		uint8_t Data[1];			//!< Information data
	} Ext;
} BleL2CapIFrame_t;

/// NOTE: Variable length structure
typedef struct __Ble_L2CAP_Att {
	uint8_t OpCode;					//!< Attribute protocol Opcode
	uint8_t Param[1];				//!< Attribute protocol parameters
} BleL2CapAtt_t;

/// NOTE: Variable length structure
typedef struct __Ble_L2CAP_Pdu {
	BleL2CapHdr_t Hdr;				//!< L2CAP header
	union {
		uint16_t Control;
		uint32_t ExtControl;
		BleL2CapSFrame_t SFrame;	//!< S-Frame
		BleL2CapIFrame_t IFrame;	//!< I-Frame header
		BleL2CapAtt_t Att;
	};
} BleL2CapPdu_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif // __BLE_L2CAP_H__

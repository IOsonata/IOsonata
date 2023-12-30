/**-------------------------------------------------------------------------
@file	bt_l2cap.h

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
#ifndef __BT_L2CAP_H__
#define __BT_L2CAP_H__

#include <inttypes.h>

#include "bt_att.h"

#define BT_L2CAP_CID_FIXED_START			1
#define BT_L2CAP_CID_FIXED_END				0x3F	//

#define BT_L2CAP_CID_ACL_U					1		//!< ACL-U signaling, MTU 48 bytes, ext 672 bytes
#define BT_L2CAP_CID_CONNECTIONLESS			2		//!< Connectionless
#define BT_L2CAP_CID_ATT					4		//!< Attribute protocol
#define BT_L2CAP_CID_SIGNAL					5		//!< LE-U signaling, MTU 23 bytes
#define BT_L2CAP_CID_SEC_MNGR				6		//!< Security manager

#define BT_L2CAP_CID_DYNA_START				0x40	//!< Start of dynamically allocated
#define BT_L2CAP_CID_DYNA_END				0x7F	//!< End of dynamically allocated

typedef enum __Ble_L2CAP_Mode {
	L2CAP_MODE_BASIC,						//!< L2CAP_MODE_BASIC
	L2CAP_MODE_FLOW_CTRL,          			//!< L2CAP_MODE_FLOW_CTRL
	L2CAP_MODE_RETRANSMISSION,     			//!< L2CAP_MODE_RETRANSMISSION
	L2CAP_MODE_ENH_RETRANSMISSION, 			//!< L2CAP_MODE_ENH_RETRANSMISSION
	L2CAP_MODE_STREAMING,          			//!< L2CAP_MODE_STREAMING
	L2CAP_MODE_LE_CREDIT_FLOW_CTRL,			//!< L2CAP_MODE_LE_CREDIT_FLOW_CTRL
	L2CAP_MODE_ENH_CREDIT_FLOW_CTRL			//!< L2CAP_MODE_ENH_CREDIT_FLOW_CTRL
} L2CAP_MODE;

#define BT_L2CAP_SDU_SAR_UNSEGMENTED			0	//!< Unsegmented SDU
#define BT_L2CAP_SDU_SAR_START					1	//!< Start of SDU
#define BT_L2CAP_SDU_SAR_END					2	//!< End of SDU
#define BT_L2CAP_SDU_SAR_CONT					3	//!< Continuation SDU

#define BT_L2CAP_SUPER_RR						0	//!< Receiver ready
#define BT_L2CAP_SUPER_REJ						1	//!< Reject
#define BT_L2CAP_SUPER_RNR						2	//!< Receiver not ready
#define BT_L2CAP_SUPER_SREJ						3	//!< Select reject

#define BT_L2CAP_CONTROL_TYPE_MASK				(1<<0)
#define BT_L2CAP_CONTROL_TYPE_SFRAME			(1<<0)

#define BT_L2CAP_CODE_COMMAND_REJECT_RSP				1	//!< CID 1 & 5
#define BT_L2CAP_CODE_CONNECTION_REQ					2	//!< CID 1
#define BT_L2CAP_CODE_CONNECTION_RSP					3	//!< CID 1
#define BT_L2CAP_CODE_CONFIGURATION_REQ					4	//!< CID 1
#define BT_L2CAP_CODE_CONFIGURATION_RSP					5	//!< CID 1
#define BT_L2CAP_CODE_DISCONNECTION_REQ					6	//!< CID 1 & 5
#define BT_L2CAP_CODE_DISCONNECTION_RSP					7	//!< CID 1 & 5
#define BT_L2CAP_CODE_ECHO_REQ							8	//!< CID 1
#define BT_L2CAP_CODE_ECHO_RSP							9	//!< CID 1
#define BT_L2CAP_CODE_INFORMATION_REQ					0xA	//!< CID 1
#define BT_L2CAP_CODE_INFORMATION_RSP					0xB	//!< CID 1
#define BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_REQ	0x12	//!< CID 5
#define BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_RSP	0x13	//!< CID 5
#define BT_L2CAP_CODE_LE_CREDIT_BASED_CONNECTION_REQ	0x14	//!< CID 5
#define BT_L2CAP_CODE_LE_CREDIT_BASED_CONNECTION_RSP	0x15	//!< CID 5
#define BT_L2CAP_CODE_FLOW_CONTROL_CREDIT_IND			0x16	//!< CID 1 & 5
#define BT_L2CAP_CODE_CREDIT_BASED_CONNECTION_REQ		0x17	//!< CID 1 & 5
#define BT_L2CAP_CODE_CREDIT_BASED_CONNECTION_RSP		0x18	//!< CID 1 & 5
#define BT_L2CAP_CODE_CREDIT_BASED_RECONFIGURE_REQ		0x19	//!< CID 1 & 5
#define BT_L2CAP_CODE_CREDIT_BASED_RECONFIGURE_RSP		0x1A	//!< CID 1 & 5

#pragma pack(push, 1)

/// L2CAP PDU header
typedef struct __Bt_L2Cap_Header {
	uint16_t Len;				//!< PDU length
	uint16_t Cid;				//!< CID - Channel ID
} BtL2CapHdr_t;

/// L2CAP C-Frame
/// NOTE: Variable length
typedef struct __Bt_L2Cap_CFrame {
	uint8_t Code;					//!< L2CAP command control code
	uint8_t Id;						//!< Command Code of requested command
	uint16_t Len;					//!< Length of following data
	uint8_t Data[1];				//!< Variable length data depending of the command code
} BtL2CapCFrame_t;

/// L2CAP Supervisory frame
/// NOTE: Variable length structure
typedef union __Bt_L2Cap_SFrame_Fields {
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
} BtL2CapSFrame_t;

/// L2CAP Information frame
/// NOTE: Variable length structure
typedef union __Bt_L2Cap_IFrame_Fields {
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
} BtL2CapIFrame_t;
#if 0
/// NOTE: Variable length structure
typedef struct __Bt_L2Cap_Att {
	uint8_t OpCode;					//!< Attribute protocol Opcode
	uint8_t Param[1];				//!< Attribute protocol parameters
} BtL2CapAtt_t;
#endif
typedef struct __Bt_L2Cap_Smp {
	uint8_t Code;
	uint8_t Data[1];			//!< Max data size is 65 byte secure or 23 non secure
} BtL2CapSmp_t;


/// NOTE: Variable length structure
/// FCS not defined in structure due to variable length of data payload. It is to
/// be processed externally when needed
typedef struct __Bt_L2Cap_Pdu {
	BtL2CapHdr_t Hdr;				//!< L2CAP header

	// Different type of payload depending on CID
	union {
		BtL2CapCFrame_t CFrame;	//!< Control frame payload, CID 1 & 5
		uint16_t Control;			//!< L2CAP I-Frame and S-Frame 16 bits control bits
		uint32_t ExtControl;		//!< L2CAP I-Frame and S-Frame 32 bits ext control bits
		BtL2CapSFrame_t SFrame;		//!< S-Frame control bit fields and payload
		BtL2CapIFrame_t IFrame;		//!< I-Frame control bit fields and payload
		//BtL2CapAtt_t Att;			//!< Attribute payload, CID 4
		BtAtt_t Att;				//!< Attribute payload, CID 4
		BtL2CapSmp_t Smp;			//!< SMP payload CID 6
	};
} BtL2CapPdu_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif // __BT_L2CAP_H__

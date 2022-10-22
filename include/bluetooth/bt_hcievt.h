/**-------------------------------------------------------------------------
@file	bt_hcievt.h

@brief	Bluetooth Generic HCI event handlers

Generic implementation of Bluetooth HCI event handling

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2022

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
#ifndef __BT_HCIEVT_H__
#define __BT_HCIEVT_H__

#include <stdint.h>

#pragma pack(push, 1)

/// HCI Event packet header
typedef struct __Bt_Hci_Evt_Packet_Header {
	uint8_t Evt;
	uint8_t Len;
} BtHciEvtPacketHdr_t;

/// HCI event packet
/// NOTE: This structure is variable length
typedef struct __Bt_Hci_Evt_Packet {
	BtHciEvtPacketHdr_t Hdr;
	uint8_t Data[1];
} BtHciEvtPacket_t;

/// HCI Event Connection Complete
typedef struct __Bt_Hci_Evt_Conn_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Addr[6];			//!< BD address of the other connected device
	uint8_t LinkType;			//!< Link type SCO or ACL
	uint8_t Encrypt;			//!< Encryption : 1 - enable, 0 - disable
} BtHciEvtConnCompete_t;

typedef struct __Bt_Hci_Evt_Conn_Rqst {
	uint8_t Addr[6];			//!< BD address of the device requesting connection
	uint8_t DevClass[3];		//!< Class of Device
	uint8_t LinkType;			//!< Link type
} BtHciEvtConnRqst_t;

typedef struct __Bt_Hci_Evt_Discon_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Reason;				//!< Reason for disconnection
} BtHciEvtDisconComplete_t;

typedef struct __Bt_Hci_Evt_Auth_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
} BtHciEvtAuthComplete_t;

typedef struct __Bt_Hci_Evt_Remote_Name_Rqst_Complete {
	uint8_t Status;				//!< Status
	uint8_t Addr[6];			//!< BD Address
	uint8_t Name[248];			//!< Name in UTF-8
} BtHciEvtRemoteNameRqstComplete_t;

typedef struct __Bt_Hci_Evt_Encrypt_Change {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Encrypt;			//!< Encryption : 1 - Enable, 0 - Disable, 2 - Enable with AES-CCM for BR/EDR
	uint8_t KeySize;			//!< Encryption key size in bytes
} BtHciEvtEncryptChange_t;

typedef struct __Bt_Hci_Evt_Change_Conn_Key_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
} BtHciEvtChangeConnKeyComplete_t;

typedef struct __Bt_Hci_Evt_Key_Type_Change {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Flag;				//!< Key flag
} BtHciEvtKeyTypeChange_t;

typedef struct __Bt_Hci_Evt_Read_Remote_Feature_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Features[8];		//!< Bit mask LMP features
} BtHciEvtReadRemoteFeatureComplete_t;

typedef struct __Bt_Hci_Evt_Read_Remote_Info_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Version;			//!< Version number
	uint16_t CompanyId;			//!< Company identifier
	uint16_t SubVers;			//!< Subversion number
} BtHciEvtReadRemoteInfoComplete_t;

typedef struct __Bt_Hci_Evt_QoS_Setup_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Unsused;
	uint8_t SrvcType;			//!< Service type
	uint32_t TikenRate;			//!< Token rate bytes/sec
	uint32_t PeakBandwidth;		//!< Peak bandwidth byte/sec
	uint32_t Latency;			//!< Available latency in usec
	uint32_t DelayVar;			//!< Available delay variation in usec
} BtHciEvtQoSSetupComplete_t;

/// NOTE: This structure is variable length
typedef struct __Bt_Hci_Evt_Cmd_Complete {
	uint8_t NbCmdPacket;		//!< Number of HCI command packet allowed to be sent
	uint16_t CmdCode;			//!< Command opcode
	uint8_t RetParam[1];		//!< Variable length of return parameters depending on the command
} BtHciEvtCmdComplete_t;

typedef struct __Bt_Hci_Evt_Cmd_Status {
	uint8_t Status;				//!< Status
	uint8_t NbCmdPacket;		//!< Number of HCI command packet allowed to be sent
	uint16_t CmdCode;			//!< Command opcode
} BtHciEvtCmdStatus_t;

typedef struct __Bt_Hci_Evt_Role_Change {
	uint8_t Status;				//!< Status
	uint8_t Addr[6];			//!< BD address of the device
	uint8_t Role;				//!< New role
} BtHciEvtRoleChange_t;

typedef struct __Bt_Hci_Evt_Mode_Change {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Mode;				//!< Current mode
	uint16_t Interval;			//!< Hold interval in 625usec
} BtHciEvtModeChange_t;


/// HCI LE Meta event packet
/// NOTE: This structure is variable length
typedef struct __Bt_Hci_MetaEvt_Packet {
	uint8_t Evt;				//!< Meta event code
	uint8_t Data[1];
} BtHciMetaEvtPacket_t;

typedef struct __Bt_Hci_MetaEvt_ConnComplete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Role;				//!< Role central/peripheral
	uint8_t PeerAddrType;		//!< Peer address type
	uint8_t PeerAddr[6];		//!< Peer address
	uint16_t ConnInterval;		//!< Connection interval in 1.25ms, time = ConnInterval * 1.25
	uint16_t PeriphLatency;		//!< Peripheral latency in number of connection events
	uint16_t SupervTimeout;		//!< Supervision timeout in 1.25ms
	uint8_t CentralClkAccu;		//!< Central clock accuracy PPM table
} BtHciMetaEvtConnComplete_t;

typedef struct __Bt_Hci_MetaEvt_Adv_Report {
	uint8_t NbReport;			//!< Number of responses in event
	struct Report {
		uint8_t EvtType;
		uint8_t AddrType;
		uint8_t Addr[6];
		uint8_t DataLen;
	};
} BtHciMetaEvtAdvReport_t;

typedef struct __Bt_Hci_MetaEvt_Data_Len_Change {
	uint16_t ConnHdl;			//!< Connection handle
	uint16_t MaxTxLen;			//!< Max number length of transmit payload in bytes
	uint16_t MaxTxTime; 		//!< Max transmit time
	uint16_t MaxRxLen;			//!< Max number length of receive payload in bytes
	uint16_t MaxRxTime;			//!< Max receive time
} BtHciMetaEvtDataLenChange_t;

typedef struct __Bt_Hci_MetaEvt_Enhence_ConnComplete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Role;				//!< Role central/peripheral
	uint8_t PeerAddrType;		//!< Peer address type
	uint8_t PeerAddr[6];		//!< Peer address
	uint8_t LocalResolPriAddr[6];	//!< Local resolvable private address
	uint8_t PeerResolPriAddr[6];	//!< Peer resolvable private address
	uint16_t ConnInterval;		//!< Connection interval in 1.25ms, time = ConnInterval * 1.25
	uint16_t PeriphLatency;		//!< Peripheral latency in number of connection events
	uint16_t SupervTimeout;		//!< Supervision timeout in 1.25ms
	uint8_t CentralClkAccu;		//!< Central clock accuracy PPM table
} BtHciMetaEvtEnhConnComplete_t;

typedef struct __Bt_Hci_MetaEvt_Chan_Sel_Algo {
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t ChanSelAlgo;		//!< Channel selection algorithm
} BtHciMetaEvtChanSelAlgo_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

void BtHciProcessEvent(BtHciEvtPacket_t *pEvtPkt);
void BtHciProcessMetaEvent(BtHciMetaEvtPacket_t *pMetaEvtPkt);

#ifdef __cplusplus
}
#endif

#endif // __BT_HCIEVT_H__

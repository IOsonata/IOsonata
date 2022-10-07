/**-------------------------------------------------------------------------
@file	ble_hcievt.h

@brief	Bluetooth Generic HCI event handlers


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
#ifndef __BLE_HCIEVT_H__
#define __BLE_HCIEVT_H__

#include <stdint.h>

#pragma pack(push, 1)

/// HCI Event packet header
typedef struct __Ble_Hci_Evt_Packet_Header {
	uint8_t Evt;
	uint8_t Len;
} BleHciEvtPacketHdr_t;

/// HCI event packet
/// NOTE: This structure is variable length
typedef struct __Ble_Hci_Evt_Packet {
	BleHciEvtPacketHdr_t Hdr;
	uint8_t Data[1];
} BleHciEvtPacket_t;

/// HCI Event Connection Complete
typedef struct __Ble_Hci_Evt_Conn_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Addr[6];			//!< BD address of the other connected device
	uint8_t LinkType;			//!< Link type SCO or ACL
	uint8_t Encrypt;			//!< Encryption : 1 - enable, 0 - disable
} BleHciEvtConnCompete_t;

typedef struct __Ble_Hci_Evt_Conn_Rqst {
	uint8_t Addr[6];			//!< BD address of the device requesting connection
	uint8_t DevClass[3];		//!< Class of Device
	uint8_t LinkType;			//!< Link type
} BleHciEvtConnRqst_t;

typedef struct __Ble_Hci_Evt_Discon_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Reason;				//!< Reason for disconnection
} BleHciEvtDisconComplete_t;

typedef struct __Ble_Hci_Evt_Auth_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
} BleHciEvtAuthComplete_t;

typedef struct __Ble_Hci_Evt_Remote_Name_Rqst_Complete {
	uint8_t Status;				//!< Status
	uint8_t Addr[6];			//!< BD Address
	uint8_t Name[248];			//!< Name in UTF-8
} BleHciEvtRemoteNameRqstComplete_t;

typedef struct __Ble_Hci_Evt_Encrypt_Change {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Encrypt;			//!< Encryption : 1 - Enable, 0 - Disable, 2 - Enable with AES-CCM for BR/EDR
	uint8_t KeySize;			//!< Encryption key size in bytes
} BleHciEvtEncryptChange_t;

typedef struct __Ble_Hci_Evt_Change_Conn_Key_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
} BleHciEvtChangeConnKeyComplete_t;

typedef struct __Ble_Hci_Evt_Key_Type_Change {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Flag;				//!< Key flag
} BleHciEvtKeyTypeChange_t;

typedef struct __Ble_Hci_Evt_Read_Remote_Feature_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Features[8];		//!< Bit mask LMP features
} BleHciEvtReadRemoteFeatureComplete_t;

typedef struct __Ble_Hci_Evt_Read_Remote_Info_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Version;			//!< Version number
	uint16_t CompanyId;			//!< Company identifier
	uint16_t SubVers;			//!< Subversion number
} BleHciEvtReadRemoteInfoComplete_t;

typedef struct __Ble_Hci_Evt_QoS_Setup_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Unsused;
	uint8_t SrvcType;			//!< Service type
	uint32_t TikenRate;			//!< Token rate bytes/sec
	uint32_t PeakBandwidth;		//!< Peak bandwidth byte/sec
	uint32_t Latency;			//!< Available latency in usec
	uint32_t DelayVar;			//!< Available delay variation in usec
} BleHciEvtQoSSetupComplete_t;

/// NOTE: This structure is variable length
typedef struct __Ble_Hci_Evt_Cmd_Complete {
	uint8_t NbCmdPacket;		//!< Number of HCI command packet allowed to be sent
	uint16_t CmdCode;			//!< Command opcode
	uint8_t RetParam[1];		//!< Variable length of return parameters depending on the command
} BleHciEvtCmdComplete_t;

typedef struct __Ble_Hci_Evt_Cmd_Status {
	uint8_t Status;				//!< Status
	uint8_t NbCmdPacket;		//!< Number of HCI command packet allowed to be sent
	uint16_t CmdCode;			//!< Command opcode
} BleHciEvtCmdStatus_t;

typedef struct __Ble_Hci_Evt_Role_Change {
	uint8_t Status;				//!< Status
	uint8_t Addr[6];			//!< BD address of the device
	uint8_t Role;				//!< New role
} BleHciEvtRoleChange_t;

typedef struct __Ble_Hci_Evt_Mode_Change {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Mode;				//!< Current mode
	uint16_t Interval;			//!< Hold interval in 625usec
} BleHciEvtModeChange_t;


/// HCI LE Meta event packet
/// NOTE: This structure is variable length
typedef struct __Ble_Hci_MetaEvt_Packet {
	uint8_t Evt;				//!< Meta event code
	uint8_t Data[1];
} BleHciMetaEvtPacket_t;

typedef struct __Ble_Hci_MetaEvt_ConnComplete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Role;				//!< Role central/peripheral
	uint8_t PeerAddrType;		//!< Peer address type
	uint8_t PeerAddr[6];		//!< Peer address
	uint16_t ConnInterval;		//!< Connection interval in 1.25ms, time = ConnInterval * 1.25
	uint16_t PeriphLatency;		//!< Peripheral latency in number of connection events
	uint16_t SupervTimeout;		//!< Supervision timeout in 1.25ms
	uint8_t CentralClkAccu;		//!< Central clock accuracy PPM table
} BleHciMetaEvtConnComplete_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

void BleHciProcessEvent(BleHciEvtPacket_t *pEvtPkt);
void BleHciProcessMetaEvent(BleHciMetaEvtPacket_t *pMetaEvtPkt);

#ifdef __cplusplus
}
#endif

#endif // __BLE_HCIEVT_H__

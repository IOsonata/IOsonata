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
#include "bluetooth/bt_hci.h"

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

/// NOTE: Variable length
typedef struct __Bt_Hci_Evt_Nb_Completed_Pkt {
	uint8_t NbHdl;			//!< Number of connection handle
	struct {
		uint16_t Hdl;			//!< Connection handle
		uint16_t NbPkt;			//!< Number of completed packet
	} Completed[1];
} BtHciEvtNbCompletedPkt_t;

typedef struct __Bt_Hci_Evt_Mode_Change {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Mode;				//!< Current mode
	uint16_t Interval;			//!< Hold interval in 625usec
} BtHciEvtModeChange_t;

/// Event Return link keys
typedef struct __Bt_Hci_Evt_Rtn_Link_Keys {
	uint8_t NbKeys;				//!< Number of link keys
	struct {
		uint8_t Addr[6];		//!< BD_ADDR
		uint8_t Key[16];		//!< Key
	} Keys[1];
} BtHciEvtRtnLinkKey_t;

/// HCI LE Meta event packet
/// NOTE: This structure is variable length
typedef struct __Bt_HciLe_Evt_Packet {
	uint8_t Evt;				//!< Meta event code
	uint8_t Data[1];
} BtHciLeEvtPacket_t;

typedef struct __Bt_HciLe_Evt_ConnComplete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t Role;				//!< Role central/peripheral
	uint8_t PeerAddrType;		//!< Peer address type
	uint8_t PeerAddr[6];		//!< Peer address
	uint16_t ConnInterval;		//!< Connection interval in 1.25ms, time = ConnInterval * 1.25
	uint16_t PeriphLatency;		//!< Peripheral latency in number of connection events
	uint16_t SupervTimeout;		//!< Supervision timeout in 1.25ms
	uint8_t CentralClkAccu;		//!< Central clock accuracy PPM table
} BtHciLeEvtConnComplete_t;

typedef struct __Bt_HciLe_Evt_Adv_Report {
	uint8_t NbReport;			//!< Number of responses in event
	struct Report {
		uint8_t EvtType;
		uint8_t AddrType;
		uint8_t Addr[6];
		uint8_t DataLen;
	};
} BtHciLeEvtAdvReport_t;

typedef struct __Bt_HciLe_Evt_Conn_Update_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint16_t ConnInterval;		//!< Connection interval
	uint16_t Latency;			//!< Peripheral latency
	uint16_t SuperTimeout;		//!< Supervision timeout
} BtHciLeEvtConnUpdateComplete_t;

typedef struct __Bt_HciLe_Evt_Read_Remote_Feature_Complete {
	uint8_t Status;				//!< Status
	uint16_t ConnHdl;			//!< Connection handle
	uint64_t Features;			//!< LE features
} BtHciLeEvtReadRemoteFeatureComplete_t;

typedef struct __Bt_HciLe_Evt_Longterm_Key_Request {
	uint16_t ConnHdl;			//!< Connection handle
	uint64_t RandNumber;		//!< 64 bits random number
	uint16_t EncryptDivers;		//!< 16 bits encrypted diversifier
} BtHciLeEvtLongtermKeyReq_t;

typedef struct __Bt_HciLe_Evt_Remote_Conn_Param_Request {
	uint16_t ConnHdl;			//!< Connection handle
	uint16_t IntervalMin;		//!< Min. connection interval
	uint16_t IntervalMax;		//!< Max. connection interval
	uint16_t Latency;			//!< Maximum allowed Peripheral latency
	uint16_t Timeout;			//!< Supervision timeout
} BtHciLeEvtRemoteConnParamReq_t;

typedef struct __Bt_HciLe_Evt_Data_Len_Change {
	uint16_t ConnHdl;			//!< Connection handle
	uint16_t MaxTxLen;			//!< Max number length of transmit payload in bytes
	uint16_t MaxTxTime; 		//!< Max transmit time
	uint16_t MaxRxLen;			//!< Max number length of receive payload in bytes
	uint16_t MaxRxTime;			//!< Max receive time
} BtHciLeEvtDataLenChange_t;

typedef struct __Bt_HciLe_Evt_Read_Local_P256_Public_Key_Complete {
	uint8_t Status;				//!< Status
	uint8_t KeyXCoord[32];		//!< Local P-256 public key X coordinate.
	uint8_t KeyYCoord[32];		//!< Local P-256 public key Y coordinate.
} BtHciLeEvtReadLocalP256PubKeyComplete_t;

typedef struct __Bt_HciLe_Generate_DHKey_Complete {
	uint8_t Status;				//!< Status
	uint8_t DHKey[32];			//!< Diffie Hellman kay
} BtHciLeEvtGenerateDHKeyComplete_t;

typedef struct __Bt_HciLe_Evt_Enhence_ConnComplete {
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
} BtHciLeEvtEnhConnComplete_t;

/// NOTE: variable length
typedef struct __Bt_HciLe_Evt_Directed_Adv_Report {
	uint8_t NbReports;			//!< Number of responses in event
	struct {
		uint8_t Type;			//!<
		uint8_t AddrType;		//!< Address type of the advertising device
		uint8_t Addr[6];		//!< Address of the advertising device
		uint8_t DirAddrType;	//!< device address type
		uint8_t DirAddr[6];		//!< device address
		uint8_t Rssi;			//!< in dBm (-127 to +20)
	} Report[1];
} BtHciLeDirectedAdvReport_t;

typedef struct __Bt_HciLe_Evt_Phy_Update_Complete {
	uint8_t Status;
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t TxPhy;				//!<
	uint8_t RxPhy;
} BtHciLeEvtPhyUpdateComplete_t;

/// NOTE: Variable length
typedef struct __Bt_ExtAdv_Report {
	uint16_t Type;
	uint8_t AddrType;
	uint8_t Addr[6];
	uint8_t PrimPhy;
	uint8_t SecPhy;
	uint8_t AdvSid;
	uint8_t TxPwr;
	uint8_t Rssi;
	uint16_t PeriodAdvInterval;
	uint8_t DirAddrType;
	uint8_t DirAddr[6];
	uint8_t DataLen;
	uint8_t Data[1];
} BtExtAdvReport_t;

/// NOTE: Variable length
typedef struct __Bt_HciLe_Evt_ExtAdv_Report {
	uint8_t NbReport;
	BtExtAdvReport_t Report[1];	//!< Variable length report data
} BtHciLeEvtExtAdvReport_t;

typedef struct __Bt_HciLe_Evt_PeriodAdvSyncEstablished {
	uint8_t Status;
	uint16_t SyncHdl;
	uint8_t SyncSid;
	uint8_t AdvAddrType;
	uint8_t AdvAddr[6];
	uint8_t AdvPhy;
	uint16_t PeriodAdvInterval;
	uint8_t AdvClockAccuracy;
} BtHciLeEvtPeriodAdvSyncEstablished_t;

/// NOTE: Variable length
typedef struct __Bt_HciLe_Evt_Periodic_Adv_Report {
	uint16_t SyncHdl;
	uint8_t TxPwr;
	uint8_t Rssi;
	uint8_t CteType;
	uint8_t DataStatus;
	uint8_t DataLen;
	uint8_t Data[1];
} BtHciLeAdvPeriodAdvReport_t;;

typedef struct __Bt_HciLe_Evt_Periodic_Adv_Sync_Lost {
	uint16_t SyncHdl;
} BtHciLeEvtPeriodAdvSyncLost_t;

typedef struct __Bt_HciLe_Evt_Adv_set_Terminated {
	uint8_t Status;
	uint8_t AdvHdl;
	uint16_t ConnHdl;
	uint8_t NbCompletedEvt;
} BtHciLeEvtAdvSetTerminated_t;

typedef struct __Bt_HciLe_Evt_Scan_Request_Received {
	uint8_t AdvHdl;
	uint8_t ScannerAddrType;
	uint8_t ScannerAddr[6];
} BtHciLeEvtScanReqReceived_t;

typedef struct __Bt_HciLe_Evt_Chan_Sel_Algo {
	uint16_t ConnHdl;			//!< Connection handle
	uint8_t ChanSelAlgo;		//!< Channel selection algorithm
} BtHciLeEvtChanSelAlgo_t;

/// NOTE: Variable length
typedef struct __Bt_HciLe_Evt_Connectionless_IQ_Report {
	uint16_t SyncHdl;
	uint8_t ChanIdx;
	uint16_t Rssi;
	uint8_t RssiAntId;
	uint8_t CteType;
	uint8_t SlotDur;
	uint8_t PacketStatus;
	uint16_t PeriodEvtCounter;
	uint8_t SampleCount;
	struct {
		uint8_t ISample;
		uint8_t QSample;
	} Sample[1];
} BtHciLeEvtConnlessIQReport_t;

/// NOTE: Variable length
typedef struct __Bt_HciLe_Evt_Conn_IQ_Report {
	uint16_t ConnHdl;
	uint8_t RxPhy;
	uint8_t DataChanIdx;
	uint16_t Rssi;
	uint8_t RssiAntId;
	uint8_t CteType;
	uint8_t SlotDur;
	uint8_t PacketStatus;
	uint16_t ConnEvtCount;
	uint8_t SampleCount;
	struct {
		uint8_t ISample;
		uint8_t QSample;
	} Sample[1];
} BtHciLeEvtConnIQReport_t;

typedef struct __Bt_HciLe_Evt_CTE_Request_Failed {
	uint8_t Status;
	uint16_t ConnHdl;
} BtHciLeEvtCTEReqFailed_t;

typedef struct __Bt_HciLe_Evt_PeriodAdv_Sync_Trans_Received {
	uint8_t Status;
	uint16_t ConnHdl;
	uint16_t SrvcData;
	uint16_t SyncHdl;
	uint8_t AdvSid;
	uint8_t AdvAddrType;
	uint8_t AdvAddr[6];
	uint8_t AdvPhy;
	uint16_t PeriodAdvInterval;
	uint8_t AdvClockAccuracy;
} BtHciLeEvtPriodAdvSyncTransReceived_t;

typedef struct __Bt_HciLe_Evt_CIS_Established {
	uint8_t Status;
	uint16_t ConnHdl;
	uint32_t CIGSyncDelay:24;
	uint32_t CISSyncDelay:24;
	uint32_t TransportLatencyC2P:24;
	uint32_t TransportLatencyP2C:24;
	uint8_t PhyC2P;
	uint8_t PhyP2C;
	uint8_t Nse;
	uint8_t BnC2P;
	uint8_t BnP2C;
	uint8_t FtC2P;
	uint8_t FtP2C;
	uint16_t MaxPduC2P;
	uint16_t MaxPduP2C;
	uint16_t IsoInterval;
} BtHciLeEvtCISEstablished_t;

typedef struct __Bt_HciLe_Evt_CIS_Request {
	uint16_t ACLConnHdl;
	uint16_t CISConnHdl;
	uint8_t CIGId;
	uint8_t CISId;
} BtHciLeEvtCISReq_t;

/// NOTE: Variable length
typedef struct __Bt_HciLe_Evt_Create_BIG_Complete {
	uint8_t Status;
	uint8_t BigHdl;
	uint32_t BigSyncDelay:24;
	uint32_t TransLatencyBig:24;
	uint8_t Phy;
	uint8_t Nse;
	uint8_t Bn;
	uint8_t Pto;
	uint8_t Irc;
	uint16_t MaxPdu;
	uint16_t IsoInterval;
	uint8_t NbBis;
	uint16_t ConnHdl[1];
} BtHciLeEvtCreateBIGComplete_t;

typedef struct __Bt_HciLe_Evt_Terminate_BIG_Complete {
	uint8_t BigHdl;
	uint8_t Reason;
} BtHciLeEvtTerminateBIGComplete_t;

typedef struct __Bt_HciLe_Evt_BIG_Sync_Established {
	uint8_t Status;
	uint8_t BigHdl;
	uint32_t TransLatencyBig:24;
	uint32_t Nse:8;
	uint8_t Bn;
	uint8_t Pto;
	uint8_t Irc;
	uint16_t MaxPdu;
	uint16_t IsoInterval;
	uint8_t NbBis;
	uint16_t ConnHdl[1];
} BtHciLeEvtBIGSyncEstablished_t;

typedef struct __Bt_HciLe_Evt_BIG_Sync_Lost {
	uint8_t BigHdl;
	uint8_t Reason;
} BtHciLeEvtBIGSyncLost_t;

typedef struct __Bt_HciLe_Evt_Request_Peer_SCA_Complete {
	uint8_t Status;
	uint16_t ConnHdl;
	uint8_t PeerClockAccuracy;
} BtHciLeEvtReqPeerSCAComplete_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

void BtHciProcessEvent(BtHciDevice_t * const pDev, BtHciEvtPacket_t *pEvtPkt);
void BtHciProcessData(BtHciDevice_t * const pDev, BtHciACLDataPacket_t *pPkt);

#ifdef __cplusplus
}
#endif

#endif // __BT_HCIEVT_H__

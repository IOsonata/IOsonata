/**-------------------------------------------------------------------------
@file	bt_hci_host.cpp

@brief	Bluetooth HCI implementation Host side

Generic implementation & definitions of Bluetooth HCI (Host Controller Interface) Host side

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2022

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
#include <memory.h>

#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_scan.h"
#include "istddef.h"

void BtProcessAttData(BtHciDevice_t * const pDev, uint16_t ConnHdl, BtL2CapPdu_t * const pRcvPdu);

/******** For DEBUG ************/
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
/*******************************/


//alignas(4) static BtHciDevice_t s_HciDevice = {0,};

/*
bool BtHciInit(BtHciDevCfg_t const *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	s_HciDevice.SendData = pCfg->SendData;
	s_HciDevice.EvtHandler = pCfg->EvtHandler;
	s_HciDevice.ConnectedHandler = pCfg->ConnectedHandler;

	return true;
}*/


//void BtHciProcessLeEvent(BtDev_t * const pDev, BtHciLeEvtPacket_t *pLeEvtPkt)
void BtHciProcessLeEvent(BtHciDevice_t * const pDev, BtHciLeEvtPacket_t *pLeEvtPkt)
{
	//DEBUG_PRINTF("BtHciProcessMetaEvent : Evt %x\r\n", pLeEvtPkt->Evt);

	switch (pLeEvtPkt->Evt)
	{
		case BT_HCI_EVT_LE_CONN_COMPLETE:
			{
				BtHciLeEvtConnComplete_t *p = (BtHciLeEvtConnComplete_t*)pLeEvtPkt->Data;
				if (p->Status == 0)
				{
					pDev->Connected(p->ConnHdl, p->Role, p->PeerAddrType, p->PeerAddr);
					//BLEAPP_ROLE role = p->Role == 1 ? BLEAPP_ROLE_PERIPHERAL : BLEAPP_ROLE_CENTRAL;
					DEBUG_PRINTF("BT_HCI_EVT_LE_CONN_COMPLETE \r\n");
					DEBUG_PRINTF("Target device Role = %d \n", p->Role);

	//					printf("hdl %x, %d\n", conhdl, role);
				}
			}
			break;
		case BT_HCI_EVT_LE_ADV_REPORT:
			{
				BtHciLeEvtAdvReport_t *report = (BtHciLeEvtAdvReport_t*)pLeEvtPkt->Data;
				BtAdvReport_t *p = (BtAdvReport_t*)report->Report;

				for (int i = 0; i < report->NbReport; i++)
				{
					int8_t rssi = (int8_t)p[i].Data[p[i].DataLen];
					pDev->ScanReport(rssi, p[i].AddrType, p[i].Addr, p[i].DataLen, p[i].Data);
				}
				//BtScanReport(pLeEvtPkt->Evt, p->NbReport, p->Report);
			}
			break;
		case BT_HCI_EVT_LE_CONN_UPDATE_COMPLETE:
			{
				BtHciLeEvtConnUpdateComplete_t *p = (BtHciLeEvtConnUpdateComplete_t*)pLeEvtPkt->Data;
				DEBUG_PRINTF("BT_HCI_EVT_LE_CONN_UPDATE_COMPLETE \r\n");
				DEBUG_PRINTF("Latency = %d, SuperTimeout = %d\r\n", p->Latency, p->SuperTimeout);
			}
			break;
		case BT_HCI_EVT_LE_READ_REMOTE_FEATURES_COMPLETE:
			{
				BtHciLeEvtReadRemoteFeatureComplete_t *p = (BtHciLeEvtReadRemoteFeatureComplete_t*)pLeEvtPkt->Data;
//				DEBUG_PRINTF("BT_HCI_EVT_LE_READ_REMOTE_FEATURES_COMPLETE \r\n");
			}
			break;
		case BT_HCI_EVT_LE_LONGTERM_KEY_RQST:
			break;
		case BT_HCI_EVT_LE_REMOTE_CONN_PARAM_RQST:
			{
				BtHciLeEvtRemoteConnParamReq_t *p = (BtHciLeEvtRemoteConnParamReq_t*)pLeEvtPkt->Data;
				DEBUG_PRINTF("BT_HCI_EVT_LE_REMOTE_CONN_PARAM_RQST\r\n");
			}
			break;
		case BT_HCI_EVT_LE_DATA_LEN_CHANGE:
			{
				BtHciLeEvtDataLenChange_t *p = (BtHciLeEvtDataLenChange_t*)pLeEvtPkt->Data;

				pDev->RxDataLen = p->MaxRxLen;
				pDev->TxDataLen = p->MaxTxLen;

				DEBUG_PRINTF("BT_HCI_EVT_LE_DATA_LEN_CHANGE:\r\n", p->ConnHdl, p->MaxRxLen);
				DEBUG_PRINTF(
						"MaxRxLen = %d, MaxRxTime = %d \r\n"
						"MaxTxLen = %d, MaxTxTime = %d \r\n",
						p->MaxRxLen, p->MaxRxTime, p->MaxTxLen, p->MaxTxTime);
			}
			break;
		case BT_HCI_EVT_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE:
			break;
		case BT_HCI_EVT_LE_GENERATE_DHKEY_COMPLETE:
			break;
		case BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE_V1:
		case BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE_V2:
			{
				BtHciLeEvtEnhConnComplete_t *p = (BtHciLeEvtEnhConnComplete_t*)pLeEvtPkt->Data;

//				DEBUG_PRINTF("BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE : hdl %x, role:%d\n", p->ConnHdl, p->Role);

				if (p->Status == 0)
				{
					DEBUG_PRINTF("BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE_V1/V2\r\n");
					DEBUG_PRINTF("Target device Role = %d\r\n", p->Role);
					pDev->Connected(p->ConnHdl, p->Role, p->PeerAddrType, p->PeerAddr);

					//s_ConnHdl = p->ConnHdl;

					//g_BleConn.Handle = p->ConnHdl;
					//g_BleConn.PeerAddrType = p->PeerAddrType;
					//memcpy(g_BleConn.PeerAddr, p->PeerAddr, 6);
					//BLEAPP_ROLE role = p->Role == 1 ? BLEAPP_ROLE_PERIPHERAL : BLEAPP_ROLE_CENTRAL;

					//sdc_hci_cmd_le_read_remote_features_t rrf;
					//rrf.conn_handle = g_BleConn.Handle;
					//sdc_hci_cmd_le_read_remote_features(&rrf);
/*
					uint8_t buf[255];
					BtHciCmd_Packet_t *c = (BtHciCmd_Packet_t*)buf;

					c->Hdr.OpCode = BT_HCI_CMD_LINKCTRL_ACCEPT_CONN_RQST;
					c->Hdr.ParamLen = 7;
					memcpy(c->Param, p->PeerAddr, 6);
					c->Param[6] = 1;// staty Periph
					int res = sdc_hci_cmd_put(buf);
					DEBUG_PRINTF("sdc_hci_cmd_put : res %d\r\n", res);*/
				}
			}
			break;
		case BT_HCI_EVT_LE_DIRECTED_ADV_REPORT:
			break;
		case BT_HCI_EVT_LE_PHY_UPDATE_COMPLETE:
		{
			BtHciLeEvtPhyUpdateComplete_t *p = (BtHciLeEvtPhyUpdateComplete_t *) pLeEvtPkt->Data;
			DEBUG_PRINTF("Status = %d, RxPhy = %d, TxPhy = %d \r\n", p->Status, p->RxPhy, p->TxPhy);
		}
			break;
		case BT_HCI_EVT_LE_EXT_ADV_REPORT:
			{
				//DEBUG_PRINTF("BT_HCI_EVT_LE_EXT_ADV_REPORT:\r\n");
				BtHciLeEvtExtAdvReport_t *report = (BtHciLeEvtExtAdvReport_t*)pLeEvtPkt->Data;
				BtExtAdvReport_t *p = (BtExtAdvReport_t*)report->Report;

				//DEBUG_PRINTF("BT_HCI_EVT_LE_EXT_ADV_REPORT\r\n");
				//DEBUG_PRINTF("Nb reports : %d\r\n", report->NbReport);
				for (int i = 0; i < report->NbReport; i++)
				{
//					DEBUG_PRINTF("%02x %02x %02x %02x %02x %02x, Datlen = %d\r\n",
//								p[i].Addr[0], p[i].Addr[1], p[i].Addr[2],
//								p[i].Addr[3], p[i].Addr[4], p[i].Addr[5],
//								p[i].DataLen);
					pDev->ScanReport(p[i].Rssi, p[i].AddrType, p[i].Addr, p[i].DataLen, p[i].Data);
				}

				//BtScanReport(pLeEvtPkt->Evt, p->NbReport, p->Report);
			}
			break;
		case BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_ESTABLISHED_V1:
		case BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_ESTABLISHED_V2:
			break;
		case BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V1:
		case BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V2:
			break;
		case BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_LOST:
			break;
		case BT_HCI_EVT_LE_SCAN_TIMEOUT:
			break;
		case BT_HCI_EVT_LE_ADV_SET_TERMINATED:
			break;
		case BT_HCI_EVT_LE_SCAN_RQST_RECEIVED:
			break;
		case BT_HCI_EVT_LE_CHAN_SELECTION_ALGO:
			{
				BtHciLeEvtChanSelAlgo_t *p = (BtHciLeEvtChanSelAlgo_t *)pLeEvtPkt->Data;

//				DEBUG_PRINTF("BT_HCI_EVT_LE_CHAN_SELECTION_ALGO : hdl:%x %d\r\n", p->ConnHdl, p->ChanSelAlgo);

				//sdc_hci_cmd_le_read_channel_map_t cmc;
				//sdc_hci_cmd_le_read_channel_map_return_t cmr;

				//cmc.conn_handle = g_BleConn.Handle;
				//sdc_hci_cmd_le_read_channel_map(&cmc, &cmr);

				//DEBUG_PRINTF("  %d %d %d %d %d\r\n", cmr.conn_handle, cmr.channel_map[0], cmr.channel_map[1], cmr.channel_map[2], cmr.channel_map[4]);
			}
			break;
		case BT_HCI_EVT_LE_CONNLESS_IQ_REPORT:
			break;
		case BT_HCI_EVT_LE_CONN_IQ_REPORT:
			break;
		case BT_HCI_EVT_LE_CTE_RQST_FAILED:
			break;
		case BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED_V1:
		case BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED_V2:
			break;
		case BT_HCI_EVT_LE_CIS_ESTABLISHED:
			break;
		case BT_HCI_EVT_LE_CIS_RQST:
			break;
		case BT_HCI_EVT_LE_CREATE_BIG_COMPLETE:
			break;
		case BT_HCI_EVT_LE_TERMINATE_BIG_COMPLETE:
			break;
		case BT_HCI_EVT_LE_BIG_SYNC_ESTABLISHED:
			break;
		case BT_HCI_EVT_LE_BIG_SYNC_LOST:
			break;
		case BT_HCI_EVT_LE_RQST_PEER_SCA_COMPLETE:
			break;
		case BT_HCI_EVT_LE_PATH_LOSS_THREESHOLD:
			break;
		case BT_HCI_EVT_LE_TRANSMIT_PWR_REPORTING:
			break;
		case BT_HCI_EVT_LE_BIGINFO_ADV_REPORT:
			break;
		case BT_HCI_EVT_LE_SUBRATE_CHANGE:
			break;
	}
}

void BtHciProcessEvent(BtHciDevice_t *pDev, BtHciEvtPacket_t *pEvtPkt)
{
	//DEBUG_PRINTF("### BtHciProcessEvent %x ###\r\n", pEvtPkt->Hdr.Evt);

	switch (pEvtPkt->Hdr.Evt)
	{
		case BT_HCI_EVT_INQUERY_COMPLETE:
			break;
		case BT_HCI_EVT_INQUERY_RESULT:
			break;
		case BT_HCI_EVT_CONN_COMPLETE:
		{
			BtHciEvtConnCompete_t *p = (BtHciEvtConnCompete_t *) pEvtPkt->Data;
			DEBUG_PRINTF("BtHciProcessEvent: ConnectionComplete, Status = %d (0x%x) \r\n",
									p->Status, p->Status);
		}
			break;
		case BT_HCI_EVT_CONN_REQUEST:
			break;
		case BT_HCI_EVT_DISCONN_COMPLETE:
			{
				BtHciEvtDisconComplete_t *p = (BtHciEvtDisconComplete_t*)pEvtPkt->Data;
				DEBUG_PRINTF("BtHciProcessEvent: Disconnected, Status = %d (0x%x) \r\n",
						p->Status, p->Status);

				pDev->Disconnected(p->ConnHdl, p->Reason);
			}
			break;
		case BT_HCI_EVT_AUTHEN_COMPLETE:
			break;
		case BT_HCI_EVT_REMOTE_NAME_RQST_COMPLETE:
			break;
		case BT_HCI_EVT_ENCRYPTION_CHANGE_V1:
			break;
		case BT_HCI_EVT_ENCRYPTION_CHANGE_V2:
			break;
		case BT_HCI_EVT_CHANGE_CONN_LINK_KEY_COMPLETE:
			break;
		case BT_HCI_EVT_LINK_KEY_TYPE_CHANGED:
			break;
		case BT_HCI_EVT_READ_REMOTE_SUPPORTED_FEATURES_COMPLETE:
			break;
		case BT_HCI_EVT_READ_REMOTE_VERS_INFO_COMPLETE:
			break;
		case BT_HCI_EVT_QOS_SETTUP_COMPLETE:
			break;
		case BT_HCI_EVT_COMMAND_COMPLETE:
			break;
		case BT_HCI_EVT_COMMAND_STATUS:
//			DEBUG_PRINTF("BT_HCI_EVT_COMMAND_STATUS : %d\r\n", pEvtPkt->Hdr.Len);//
//			for (int i = 0; i < pEvtPkt->Hdr.Len; i++)
//				DEBUG_PRINTF("%x ", pEvtPkt->Data[i]);
//			DEBUG_PRINTF("\r\n");
			break;
		case BT_HCI_EVT_HARDWARE_ERROR:
			break;
		case BT_HCI_EVT_FLUSH_OCCURED:
			break;
		case BT_HCI_EVT_ROLE_CHANGE:
			break;
		case BT_HCI_EVT_NB_COMPLETED_PACKET:
			{
				BtHciEvtNbCompletedPkt_t *p = (BtHciEvtNbCompletedPkt_t*)pEvtPkt->Data;

				//				DEBUG_PRINTF("BT_HCI_EVT_NB_COMPLETED_PACKET: %d\r\n", p->NbHdl);
//				for (int i = 0; i < p->NbHdl; i++)
//				{
//					DEBUG_PRINTF("Hdl: %x - NbPkt: %d\r\n", p->Completed[i].Hdl, p->Completed[i].NbPkt);
//				}
				if (pDev->SendCompleted)
				{
					for (int i = 0; i < p->NbHdl; i++)
					{
						pDev->SendCompleted(p->Completed[i].Hdl, p->Completed[i].NbPkt);
					}
				}

			}
			break;
		case BT_HCI_EVT_MODE_CHANGE:
			break;
		case BT_HCI_EVT_RETURN_LINK_KEYS:
			break;
		case BT_HCI_EVT_PIN_CODE_RQST:
			break;
		case BT_HCI_EVT_LINK_KEY_RQST:
			break;
		case BT_HCI_EVT_LINK_KEY_NOTIF:
			break;
		case BT_HCI_EVT_LOOPBACK_COMMAND:
			break;
		case BT_HCI_EVT_DATA_BUFFER_OVERFLOW:
			break;
		case BT_HCI_EVT_MAX_SLOT_CHANGE:
			break;
		case BT_HCI_EVT_READ_CLOCK_OFFSET_COMPLETE:
			break;
		case BT_HCI_EVT_CONN_PACKET_TYPE_CHANGED:
			break;
		case BT_HCI_EVT_QOS_VIOLATION:
			break;
		case BT_HCI_EVT_PAGE_SCAN_REPETITION_MODE_CHANGE:
			break;
		case BT_HCI_EVT_FLOW_SPECS_COMPLETE:
//			DEBUG_PRINTF("BT_HCI_EVT_FLOW_SPECS_COMPLETE\r\n");
			break;
		case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
			break;
		case BT_HCI_EVT_READ_REMOTE_EXT_FEATURES_COMPLETE:
			break;
		case BT_HCI_EVT_SYNCHRONOUS_CONN_COMPLETE:
			break;
		case BT_HCI_EVT_SYNCHRONOUS_CONN_CHANGED:
			break;
		case BT_HCI_EVT_SNIFF_SUBRATING:
			break;
		case BT_HCI_EVT_EXT_INQUIRY_RESULT:
			break;
		case BT_HCI_EVT_ENCRYPTION_KEY_REFRESH_COMPLETE:
			break;
		case BT_HCI_EVT_IO_CAPABILITY_RQST:
//			DEBUG_PRINTF("BT_HCI_EVT_IO_CAPABILITY_RQST\r\n");
			break;
		case BT_HCI_EVT_IO_CAPABILITY_RESPONSE:
			break;
		case BT_HCI_EVT_USER_CONFIRM_RQST:
			break;
		case BT_HCI_EVT_USER_PASSKEY_RQST:
			break;
		case BT_HCI_EVT_REMOTE_OOB_DATA_RQST:
			break;
		case BT_HCI_EVT_SIMPLE_PAIRING_COMPLETE:
			break;
		case BT_HCI_EVT_LINK_SUPERVISION_TIMEOUT_CHANGED:
			break;
		case BT_HCI_EVT_ENHANCED_FLUSH_COMPLETE:
			break;
		case BT_HCI_EVT_USER_PASSKEY_NOTIF:
			break;
		case BT_HCI_EVT_KEYPRESS_NOTIF:
			break;
		case BT_HCI_EVT_REMOTE_HOST_SUPPORTED_FEATURES_NOTIF:
			break;
		case BT_HCI_EVT_NB_COMPLETED_DATA_BLOCKS:
			break;
		case BT_HCI_EVT_TRIGGERED_CLOCK_CAPTURE:
			break;
		case BT_HCI_EVT_SYNC_TRAIN_COMPLETE:
			break;
		case BT_HCI_EVT_SYNC_TRAIN_RECEIVED:
			break;
		case BT_HCI_EVT_CONNLESS_PERIPH_BROADCAST_RECEIVE:
			break;
		case BT_HCI_EVT_CONNLESS_PERIPH_BROADCAST_TIMEOUT:
			break;
		case BT_HCI_EVT_TRUNCATED_PAGE_COMPLETE:
			break;
		case BT_HCI_EVT_PERIPH_PAGE_RESPONSE_TIMNEOUT:
			break;
		case BT_HCI_EVT_CONNLESS_PERIPH_BROADCAST_CHAN_MAP_CHANGE:
			break;
		case BT_HCI_EVT_INQUIRY_RESPONSE_NOTIF:
			break;
		case BT_HCI_EVT_AUTHEN_PAYLOAD_TIMEOUT_EXPIRED:
			break;
		case BT_HCI_EVT_SAM_STATUS_CHANGE:
			break;
		case BT_HCI_EVT_LE:
			//DEBUG_PRINTF("BT_HCI_EVT_LE\r\n");
			BtHciProcessLeEvent(pDev, (BtHciLeEvtPacket_t *)pEvtPkt->Data);
			break;
	}
//	DEBUG_PRINTF("+++++\r\n");
}


void BtHciProcessData(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pPkt)
{
	BtL2CapPdu_t *l2rcv = (BtL2CapPdu_t*)pPkt->Data;
/*
	DEBUG_PRINTF("** BtHciProcessData : Con :%d, PB :%d, PC :%d, Len :%d\r\n", pPkt->Hdr.ConnHdl, pPkt->Hdr.PBFlag, pPkt->Hdr.BCFlag, pPkt->Hdr.Len);
	for (int i = 0; i < pPkt->Hdr.Len; i++)
	{
		DEBUG_PRINTF("%x ", pPkt->Data[i]);
	}
	DEBUG_PRINTF("\r\nCID: %x\r\n", l2rcv->Hdr.Cid);
*/
	switch (l2rcv->Hdr.Cid)
	{
		case BT_L2CAP_CID_ACL_U:
			break;
		case BT_L2CAP_CID_CONNECTIONLESS:
			break;
		case BT_L2CAP_CID_ATT:
		{
			//DEBUG_PRINTF("BT_L2CAP_CID_ATT\r\n");
			uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
			BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
			BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

			acl->Hdr.ConnHdl = pPkt->Hdr.ConnHdl;
			acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
			acl->Hdr.BCFlag = 0;

			l2pdu->Hdr = l2rcv->Hdr;

			l2pdu->Hdr.Len = BtAttProcessReq(pPkt->Hdr.ConnHdl, &l2rcv->Att, l2rcv->Hdr.Len, &l2pdu->Att);

			if (l2pdu->Hdr.Len > 0)
			{
				acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

				uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
			}
			else
			{
				BtAttProcessRsp(pPkt->Hdr.ConnHdl, &l2rcv->Att, l2rcv->Hdr.Len);
			}
		}
//			BtProcessAttData(pDev, pPkt->Hdr.ConnHdl, l2frame);
			break;
		case BT_L2CAP_CID_SIGNAL:
			break;
		case BT_L2CAP_CID_SEC_MNGR:
			BtProcessSmpData(pDev, l2rcv);
			break;
	}
//	DEBUG_PRINTF("-----\r\n");
	//DEBUG_PRINTF("L2Cap : Len %d, Chan %d, Type %d\r\n", l2frame->Hdr.Len, l2frame->Hdr.Cid, l2frame->SFrame.Std.Type);
/*
	if (l2frame->Control & BT_L2CAP_CONTROL_TYPE_SFRAME)
	{
		// S-Frame
	}
	else
	{
		// I-Frame
		DEBUG_PRINTF("I : TxSeq %d, R %d, Req %d, Sar %d, L %d\r\n", l2frame->IFrame.Std.TxSeq,
					  l2frame->IFrame.Std.RetransDis, l2frame->IFrame.Std.ReqSeq,
					  l2frame->IFrame.Std.Sar, l2frame->IFrame.Std.Len);

		if (l2frame->Hdr.Cid == BT_L2CAP_CID_ATT)
		{
			//BtAttPdu_t *att = (BtAttPdu_t*)l2frame->Att;

			DEBUG_PRINTF("att %x\r\n", l2frame->Att.OpCode);
		}
	}
*/
}
/*
void BtHciNotify(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t ValHdl, void * const pData, size_t Len)
{
	if (s_NbPktSent > 3)
	{
		return;
	}
	uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

//	DEBUG_PRINTF("BtHciMotify : %d %d \r\n", ConnHdl, ValHdl);

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF;

//	BtGattCharNotify_t *p = (BtGattCharNotify_t*)l2pdu->Att.Param;
	BtAttReqRsp_t *p = (BtAttReqRsp_t*)&l2pdu->Att;

	p->HandleValueNtf.ValHdl = ValHdl;
	memcpy(p->HandleValueNtf.Data, pData, Len);
	l2pdu->Hdr.Len = sizeof(BtAttHandleValueNtf_t) + Len;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

//	DEBUG_PRINTF("Len : %d, %d, %d\r\n", Len, l2pdu->Hdr.Len, acl->Hdr.Len);
	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
	s_NbPktSent++;
//	DEBUG_PRINTF("n=%d\r\n", n);
}
*/

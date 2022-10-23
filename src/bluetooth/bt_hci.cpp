/**-------------------------------------------------------------------------
@file	bt_hci.cpp

@brief	Bluetooth HCI implementation

Generic implementation & definitions of Bluetooth HCI (Host Controller Interface)

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
#include "istddef.h"
#include "coredev/uart.h"

extern UART g_Uart;

alignas(4) static BtHciDevice_t s_HciDevice = {0,};

bool BtHciInit(BtHciDevCfg_t const *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	s_HciDevice.SendData = pCfg->SendData;
	s_HciDevice.EvtHandler = pCfg->EvtHandler;
	return true;
}

void BtHciProcessMetaEvent(BtHciMetaEvtPacket_t *pMetaEvtPkt)
{
	g_Uart.printf("BtHciProcessMetaEvent : Evt %x\r\n", pMetaEvtPkt->Evt);

	switch (pMetaEvtPkt->Evt)
	{
		case BT_HCI_EVT_LE_META_CONN_COMPLETE:
			{
				BtHciMetaEvtConnComplete_t *p = (BtHciMetaEvtConnComplete_t*)pMetaEvtPkt->Data;
				if (p->Status == 0)
				{
					uint16_t conhdl = p->ConnHdl;
					//BLEAPP_ROLE role = p->Role == 1 ? BLEAPP_ROLE_PERIPHERAL : BLEAPP_ROLE_CENTRAL;

	//					printf("hdl %x, %d\n", conhdl, role);
				}
			}
			break;
		case BT_HCI_EVT_LE_META_ADV_REPORT:
			{
				BtHciMetaEvtAdvReport_t *p = (BtHciMetaEvtAdvReport_t*)pMetaEvtPkt->Data;

				g_Uart.printf("BT_HCI_EVT_LE_META_ADV_REPORT: %d\r\n", p->NbReport);

				for (int i = 0; i < p->NbReport; i++)
				{

				}
			}
			break;
		case BT_HCI_EVT_LE_META_CONN_UPDATE_COMPLETE:
			break;
		case BT_HCI_EVT_LE_META_READ_REMOTE_FEATURES_COMPLETE:
			g_Uart.printf("BT_HCI_EVT_LE_META_READ_REMOTE_FEATURES_COMPLETE \r\n");
			break;
		case BT_HCI_EVT_LE_META_LONGTERM_KEY_RQST:
			break;
		case BT_HCI_EVT_LE_META_REMOTE_CONN_PARAM_RQST:
			break;
		case BT_HCI_EVT_LE_META_DATA_LEN_CHANGE:
			{
				BtHciMetaEvtDataLenChange_t *p = (BtHciMetaEvtDataLenChange_t*)pMetaEvtPkt->Data;

				s_HciDevice.RxDataLen = p->MaxRxLen;
				s_HciDevice.TxDataLen = p->MaxTxLen;

				g_Uart.printf("BT_HCI_EVT_LE_META_DATA_LEN_CHANGE: %x %d\r\n", p->ConnHdl, p->MaxRxLen);

			}
			break;
		case BT_HCI_EVT_LE_META_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE:
			break;
		case BT_HCI_EVT_LE_META_GENERATE_DHKEY_COMPLETE:
			break;
		case BT_HCI_EVT_LE_META_ENHANCED_CONN_COMPLETE:
			{
				BtHciMetaEvtEnhConnComplete_t *p = (BtHciMetaEvtEnhConnComplete_t*)pMetaEvtPkt->Data;

				g_Uart.printf("BT_HCI_EVT_LE_META_ENHANCED_CONN_COMPLETE : hdl %x, role:%d\n", p->ConnHdl, p->Role);

				if (p->Status == 0)
				{
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
					g_Uart.printf("sdc_hci_cmd_put : res %d\r\n", res);*/
				}
			}
			break;
		case BT_HCI_EVT_LE_META_DIRECTED_ADV_REPORT:
			break;
		case BT_HCI_EVT_LE_META_PHY_UPDATE_COMPLETE:
			break;
		case BT_HCI_EVT_LE_META_EXT_ADV_REPORT:
			break;
		case BT_HCI_EVT_LE_META_PERIODIC_ADV_SYNC_ESTABLISHED:
			break;
		case BT_HCI_EVT_LE_META_PERIODIC_ADV_REPORT:
			break;
		case BT_HCI_EVT_LE_META_PERIODIC_ADV_SYNC_LOST:
			break;
		case BT_HCI_EVT_LE_META_SCAN_TIMEOUT:
			break;
		case BT_HCI_EVT_LE_META_ADV_SET_TERMINATED:
			break;
		case BT_HCI_EVT_LE_META_SCAN_RQST_RECEIVED:
			break;
		case BT_HCI_EVT_LE_META_CHAN_SELECTION_ALGO:
			{
				BtHciMetaEvtChanSelAlgo_t *p = (BtHciMetaEvtChanSelAlgo_t *)pMetaEvtPkt->Data;

				g_Uart.printf("BT_HCI_EVT_LE_META_CHAN_SELECTION_ALGO : hdl:%x %d\r\n", p->ConnHdl, p->ChanSelAlgo);

				//sdc_hci_cmd_le_read_channel_map_t cmc;
				//sdc_hci_cmd_le_read_channel_map_return_t cmr;

				//cmc.conn_handle = g_BleConn.Handle;
				//sdc_hci_cmd_le_read_channel_map(&cmc, &cmr);

				//g_Uart.printf("  %d %d %d %d %d\r\n", cmr.conn_handle, cmr.channel_map[0], cmr.channel_map[1], cmr.channel_map[2], cmr.channel_map[4]);
			}
			break;
		case BT_HCI_EVT_LE_META_CONNLESS_IQ_REPORT:
			break;
		case BT_HCI_EVT_LE_META_CONN_IQ_REPORT:
			break;
		case BT_HCI_EVT_LE_META_CTE_RQST_FAILED:
			break;
		case BT_HCI_EVT_LE_META_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED:
			break;
		case BT_HCI_EVT_LE_META_CIS_ESTABLISHED:
			break;
		case BT_HCI_EVT_LE_META_CIS_RQST:
			break;
		case BT_HCI_EVT_LE_META_CREATE_BIG_COMPLETE:
			break;
		case BT_HCI_EVT_LE_META_TERMINATE_BIG_COMPLETE:
			break;
		case BT_HCI_EVT_LE_META_BIG_SYNC_ESTABLISHED:
			break;
		case BT_HCI_EVT_LE_META_BIG_SYNC_LOST:
			break;
		case BT_HCI_EVT_LE_META_RQST_PEER_SCA_COMPLETE:
			break;
		case BT_HCI_EVT_LE_META_PATH_LOSS_THREESHOLD:
			break;
		case BT_HCI_EVT_LE_META_TRANSMIT_PWR_REPORTING:
			break;
		case BT_HCI_EVT_LE_META_BIGINFO_ADV_REPORT:
			break;
		case BT_HCI_EVT_LE_META_SUBRATE_CHANGE:
			break;
	}
}

void BtHciProcessEvent(BtHciEvtPacket_t *pEvtPkt)
{
	g_Uart.printf("BtHciProcessEvent %x\r\n", pEvtPkt->Hdr.Evt);

	switch (pEvtPkt->Hdr.Evt)
	{
		case BT_HCI_EVT_INQUERY_COMPLETE:
			break;
		case BT_HCI_EVT_INQUERY_RESULT:
			break;
		case BT_HCI_EVT_CONN_COMPLETE:
			break;
		case BT_HCI_EVT_CONN_REQUEST:
			break;
		case BT_HCI_EVT_DISCONN_COMPLETE:
			g_Uart.printf("Disconnected\r\n");
			break;
		case BT_HCI_EVT_AUTHEN_COMPLETE:
			break;
		case BT_HCI_EVT_REMOTE_NAME_RQST_COMPLETE:
			break;
		case BT_HCI_EVT_ENCRYPTION_CHANGE:
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
			g_Uart.printf("BT_HCI_EVT_COMMAND_STATUS : %d\r\n", pEvtPkt->Hdr.Len);//
			for (int i = 0; i < pEvtPkt->Hdr.Len; i++)
				g_Uart.printf("%x ", pEvtPkt->Data[i]);
			g_Uart.printf("\r\n");
			break;
		case BT_HCI_EVT_HARDWARE_ERROR:
			break;
		case BT_HCI_EVT_FLUSH_OCCURED:
			break;
		case BT_HCI_EVT_ROLE_CHANGE:
			break;
		case BT_HCI_EVT_NB_COMPLETED_PACKET:
			g_Uart.printf("BT_HCI_EVT_NB_COMPLETED_PACKET %d, %d\r\n", pEvtPkt->Hdr.Len, pEvtPkt->Data[0]);

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
			g_Uart.printf("BT_HCI_EVT_FLOW_SPECS_COMPLETE\r\n");
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
		case BT_HCI_EVT_LE_META:
			BtHciProcessMetaEvent((BtHciMetaEvtPacket_t *)pEvtPkt->Data);
			break;
	}
}

void BtProcessAttData(uint16_t ConnHdl, BtL2CapPdu_t *pRcvPdu)
{
	uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	g_Uart.printf("ATT %d : %x\n", pRcvPdu->Hdr.Len, pRcvPdu->Att.OpCode);

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Hdr = pRcvPdu->Hdr;


	switch (pRcvPdu->Att.OpCode)
	{
		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:
			{
				BtAttExchgMtuReqRsp_t *req = (BtAttExchgMtuReqRsp_t*)&pRcvPdu->Att;

				g_Uart.printf("BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ : %d\r\n", req->RxMtu);

				l2pdu->Hdr.Len = sizeof(BtAttExchgMtuReqRsp_t);

				BtAttExchgMtuReqRsp_t *att = (BtAttExchgMtuReqRsp_t*)&l2pdu->Att;

				att->OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP;
				att->RxMtu = 512;	// TODO: get real mtu

				acl->Hdr.Len = sizeof(BtAttExchgMtuReqRsp_t) + sizeof(BtL2CapHdr_t);

				uint32_t n = s_HciDevice.SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
				g_Uart.printf("n=%d\r\n", n);
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_INFO_REQ:
			break;
		case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
			{
				BtAttReadByGroupTypeReq_t *req = (BtAttReadByGroupTypeReq_t*)&pRcvPdu->Att;

				g_Uart.printf("Att : %x, %d %d\n", req->OpCode, req->StartHdl, req->EndHdl);
				if (pRcvPdu->Hdr.Len < 8)
				{
					g_Uart.printf("UUID16 : %x\r\n", req->Uid.Uuid16);

					BtGattListEntry_t list[20];

					int c = BtGattGetList(req->Uid.Uuid16, list, 20);

					BtAttReadByGroupTypeRsp_t *rsp = (BtAttReadByGroupTypeRsp_t*)&l2pdu->Att;
					uint8_t *p = (uint8_t*)rsp->Data;

					rsp->OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP;

					if (c > 0)
					{
						//BtAttHdlRange_t *hu = (BtAttHdlRange_t*)rsp->Data;
						//hu->StartHdl = req->StartHdl;
						//hu->EndHdl = req->StartHdl + 1;
						int baseidx = list[0].Uuid.BaseIdx;
						int l = 0;
						BtUuid_t uid;
						uid.Type = BT_UUID_TYPE_16;
						rsp->Len = 6;

						if (baseidx > 0)
						{
							rsp->Len += 14;
							uid.Type = BT_UUID_TYPE_128;
							BtUuidGetBase(baseidx - 1, uid.Val.Uuid128);
						}

						for (int i = 0; i < c; i++)
						{
							if (list[i].Hdl >= req->StartHdl && list[i].Hdl <= req->EndHdl && baseidx == list[i].Uuid.BaseIdx)
							{
								BtAttHdlRange_t *hu = (BtAttHdlRange_t*)p;
								hu->StartHdl = list[i].Hdl;
								if ((i + 1) < c)
								{
									hu->EndHdl = list[i + 1].Hdl;
								}
								else
								{
									hu->EndHdl = req->EndHdl;
								}

								p += sizeof(BtAttHdlRange_t);
								if (baseidx > 0)
								{
									uid.Val.Uuid128[11] = list[i].Uuid.Uuid & 0xFF;
									uid.Val.Uuid128[12] = list[i].Uuid.Uuid >> 8;

									memcpy(p, &uid.Val, 16);
									p += 16;
									l += 20;
									for (int j = 0; j < 16; j++)
									{
										g_Uart.printf("%x ", rsp->Data[4 + j]);
									}
									g_Uart.printf("\r\n");
								}
								else
								{
									p[0] = list[i].Uuid.Uuid & 0xFF;
									p[1] = list[i].Uuid.Uuid >> 8;
									p += 2;
									l += 6;
								}
							}
						}

						l2pdu->Hdr.Len = sizeof(BtAttReadByGroupTypeRsp_t) - 1 + l;
						acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t) + l;

					}
					else
					{
						BtAttErrorRsp_t *errsp = (BtAttErrorRsp_t*)&l2pdu->Att;

						errsp->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
						errsp->ReqOpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ;
						errsp->Hdl = 1;
						errsp->Error = BT_ATT_ERROR_ATT_NOT_FOUND;

						l2pdu->Hdr.Len = sizeof(BtAttErrorRsp_t);
						acl->Hdr.Len = sizeof(BtAttErrorRsp_t) + sizeof(BtL2CapHdr_t);
					}
					g_Uart.printf("l : %d, %d, hdl %d, %x\r\n", acl->Hdr.Len, l2pdu->Hdr.Len, list->Hdl, list->Uuid.Uuid);

#if 1
/*
					rsp->OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP;
					rsp->Len = 6;

					BtAttHdlRange_t *hu = (BtAttHdlRange_t*)rsp->Data;
					hu->StartHdl = req->StartHdl;
					hu->EndHdl = 0xFFFF;
					// = 0x290A;

					l2pdu->Hdr.Len = sizeof(BtAttReadByGroupTypeRsp_t) - 1 + sizeof(BtAttHdlRange_t) + 2;
					rsp->Data[4] = 0xA;
					rsp->Data[5] = 0x29;

//					BtAttHdlRange_t *hu1 = (BtAttHdlRange_t*)&rsp->Data[6];
//					hu1->StartHdl = req->StartHdl + 1;
//					hu1->EndHdl = req->EndHdl;

					acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);*/
#else
					BtAttErrorRsp_t *errsp = (BtAttErrorRsp_t*)&l2pdu->Att;

					errsp->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
					errsp->ReqOpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ;
					errsp->Hdl = 1;
					errsp->Error = BT_ATT_ERROR_ATT_NOT_FOUND;

					l2pdu->Hdr.Len = sizeof(BtAttErrorRsp_t);
					acl->Hdr.Len = sizeof(BtAttErrorRsp_t) + sizeof(BtL2CapHdr_t);
#endif
//					int res = sdc_hci_data_put((uint8_t*)acl);

					uint32_t n = s_HciDevice.SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
					g_Uart.printf("n=%d\r\n", n);

				}
				else
				{
					g_Uart.printf("UUID128 : ", req->Uid.Uuid16);
					for (int i = 0; i <16; i++)
					{
						g_Uart.printf("%x \r\n", req->Uid.Uuid128[i]);
					}
				}


			}
			break;
		case BT_ATT_OPCODE_ATT_WRITE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_CMD:
			break;
		case BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:
			break;
		case BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD:
			break;
	}
}

void BtHciProcessData(BtHciACLDataPacket_t *pPkt)
{
	BtL2CapPdu_t *l2frame = (BtL2CapPdu_t*)pPkt->Data;

	g_Uart.printf("BtHciProcessData : Con :%d, PB :%d, PC :%d, Len :%d\r\n", pPkt->Hdr.ConnHdl, pPkt->Hdr.PBFlag, pPkt->Hdr.BCFlag, pPkt->Hdr.Len);
	for (int i = 0; i < pPkt->Hdr.Len; i++)
	{
		g_Uart.printf("%x ", pPkt->Data[i]);
	}
	g_Uart.printf("\r\n");

	switch (l2frame->Hdr.Cid)
	{
		case BT_L2CAP_CID_ACL_U:
			break;
		case BT_L2CAP_CID_CONNECTIONLESS:
			break;
		case BT_L2CAP_CID_ATT:
			BtProcessAttData(pPkt->Hdr.ConnHdl, l2frame);
			break;
		case BT_L2CAP_CID_SIGNAL:
			break;
		case BT_L2CAP_CID_SEC_MNGR:
			break;
	}

	//g_Uart.printf("L2Cap : Len %d, Chan %d, Type %d\r\n", l2frame->Hdr.Len, l2frame->Hdr.Cid, l2frame->SFrame.Std.Type);
/*
	if (l2frame->Control & BT_L2CAP_CONTROL_TYPE_SFRAME)
	{
		// S-Frame
	}
	else
	{
		// I-Frame
		g_Uart.printf("I : TxSeq %d, R %d, Req %d, Sar %d, L %d\r\n", l2frame->IFrame.Std.TxSeq,
					  l2frame->IFrame.Std.RetransDis, l2frame->IFrame.Std.ReqSeq,
					  l2frame->IFrame.Std.Sar, l2frame->IFrame.Std.Len);

		if (l2frame->Hdr.Cid == BT_L2CAP_CID_ATT)
		{
			//BtAttPdu_t *att = (BtAttPdu_t*)l2frame->Att;

			g_Uart.printf("att %x\r\n", l2frame->Att.OpCode);
		}
	}
*/
}

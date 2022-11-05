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

uint16_t s_ConnHdl = -1;


void BtHciProcessLeEvent(BtHciDevice_t * const pDev, BtHciLeEvtPacket_t *pLeEvtPkt)
{
	g_Uart.printf("BtHciProcessMetaEvent : Evt %x\r\n", pLeEvtPkt->Evt);

	switch (pLeEvtPkt->Evt)
	{
		case BT_HCI_EVT_LE_CONN_COMPLETE:
			{
				BtHciLeEvtConnComplete_t *p = (BtHciLeEvtConnComplete_t*)pLeEvtPkt->Data;
				if (p->Status == 0)
				{
					s_ConnHdl = p->ConnHdl;
					//BLEAPP_ROLE role = p->Role == 1 ? BLEAPP_ROLE_PERIPHERAL : BLEAPP_ROLE_CENTRAL;

	//					printf("hdl %x, %d\n", conhdl, role);
				}
			}
			break;
		case BT_HCI_EVT_LE_ADV_REPORT:
			{
				BtHciLeEvtAdvReport_t *p = (BtHciLeEvtAdvReport_t*)pLeEvtPkt->Data;

				g_Uart.printf("BT_HCI_EVT_LE_ADV_REPORT: %d\r\n", p->NbReport);

				for (int i = 0; i < p->NbReport; i++)
				{

				}
			}
			break;
		case BT_HCI_EVT_LE_CONN_UPDATE_COMPLETE:
			{
				BtHciLeEvtConnUpdateComplete_t *p = (BtHciLeEvtConnUpdateComplete_t*)pLeEvtPkt->Data;
			}
			break;
		case BT_HCI_EVT_LE_READ_REMOTE_FEATURES_COMPLETE:
			{
				BtHciLeEvtReadRemoteFeatureComplete_t *p = (BtHciLeEvtReadRemoteFeatureComplete_t*)pLeEvtPkt->Data;
				g_Uart.printf("BT_HCI_EVT_LE_READ_REMOTE_FEATURES_COMPLETE \r\n");
			}
			break;
		case BT_HCI_EVT_LE_LONGTERM_KEY_RQST:
			break;
		case BT_HCI_EVT_LE_REMOTE_CONN_PARAM_RQST:
			{
				BtHciLeEvtRemoteConnParamReq_t *p = (BtHciLeEvtRemoteConnParamReq_t*)pLeEvtPkt->Data;
			}
			break;
		case BT_HCI_EVT_LE_DATA_LEN_CHANGE:
			{
				BtHciLeEvtDataLenChange_t *p = (BtHciLeEvtDataLenChange_t*)pLeEvtPkt->Data;

				pDev->RxDataLen = p->MaxRxLen;
				pDev->TxDataLen = p->MaxTxLen;

				g_Uart.printf("BT_HCI_EVT_LE_DATA_LEN_CHANGE: %x %d\r\n", p->ConnHdl, p->MaxRxLen);

			}
			break;
		case BT_HCI_EVT_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE:
			break;
		case BT_HCI_EVT_LE_GENERATE_DHKEY_COMPLETE:
			break;
		case BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE:
			{
				BtHciLeEvtEnhConnComplete_t *p = (BtHciLeEvtEnhConnComplete_t*)pLeEvtPkt->Data;

				g_Uart.printf("BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE : hdl %x, role:%d\n", p->ConnHdl, p->Role);

				if (p->Status == 0)
				{
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
					g_Uart.printf("sdc_hci_cmd_put : res %d\r\n", res);*/
				}
			}
			break;
		case BT_HCI_EVT_LE_DIRECTED_ADV_REPORT:
			break;
		case BT_HCI_EVT_LE_PHY_UPDATE_COMPLETE:
			break;
		case BT_HCI_EVT_LE_EXT_ADV_REPORT:
			break;
		case BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_ESTABLISHED:
			break;
		case BT_HCI_EVT_LE_PERIODIC_ADV_REPORT:
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

				g_Uart.printf("BT_HCI_EVT_LE_CHAN_SELECTION_ALGO : hdl:%x %d\r\n", p->ConnHdl, p->ChanSelAlgo);

				//sdc_hci_cmd_le_read_channel_map_t cmc;
				//sdc_hci_cmd_le_read_channel_map_return_t cmr;

				//cmc.conn_handle = g_BleConn.Handle;
				//sdc_hci_cmd_le_read_channel_map(&cmc, &cmr);

				//g_Uart.printf("  %d %d %d %d %d\r\n", cmr.conn_handle, cmr.channel_map[0], cmr.channel_map[1], cmr.channel_map[2], cmr.channel_map[4]);
			}
			break;
		case BT_HCI_EVT_LE_CONNLESS_IQ_REPORT:
			break;
		case BT_HCI_EVT_LE_CONN_IQ_REPORT:
			break;
		case BT_HCI_EVT_LE_CTE_RQST_FAILED:
			break;
		case BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED:
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
	g_Uart.printf("### BtHciProcessEvent %x ###\r\n", pEvtPkt->Hdr.Evt);

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
			{
				BtHciEvtDisconComplete_t *p = (BtHciEvtDisconComplete_t*)pEvtPkt->Data;
				g_Uart.printf("Disconnected\r\n");
			}
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
			{
				BtHciEvtNbCompletedPkt_t *p = (BtHciEvtNbCompletedPkt_t*)pEvtPkt->Data;
				g_Uart.printf("BT_HCI_EVT_NB_COMPLETED_PACKET: %d\r\n", p->NbHdl);
				for (int i = 0; i < p->NbHdl; i++)
				{
					g_Uart.printf("Hdl: %x - NbPkt: %d\r\n", p->Completed[i].Hdl, p->Completed[i].NbPkt);
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
		case BT_HCI_EVT_LE:
			BtHciProcessLeEvent(pDev, (BtHciLeEvtPacket_t *)pEvtPkt->Data);
			break;
	}
	g_Uart.printf("+++++\r\n");
}

void BtHciSendError(BtHciDevice_t *pDev, BtHciACLDataPacket_t *pAcl, uint16_t Hdl, uint8_t OpCode, uint8_t ErrCode)
{
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)pAcl->Data;
	BtAttErrorRsp_t *errsp = (BtAttErrorRsp_t*)&l2pdu->Att;

	errsp->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	errsp->ReqOpCode = OpCode;
	errsp->Hdl = Hdl;
	errsp->Error = ErrCode;

	l2pdu->Hdr.Len = sizeof(BtAttErrorRsp_t);
	pAcl->Hdr.Len = sizeof(BtAttErrorRsp_t) + sizeof(BtL2CapHdr_t);

	g_Uart.printf("Error %x\r\n", ErrCode);

	uint32_t n = pDev->SendData((uint8_t*)pAcl, pAcl->Hdr.Len + sizeof(pAcl->Hdr));
//	g_Uart.printf("n=%d\r\n", n);
}

void BtProcessAttData(BtHciDevice_t *pDev, uint16_t ConnHdl, BtL2CapPdu_t *pRcvPdu)
{
	uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	g_Uart.printf("ATT OpCode %x, L2Cap len %d\n", pRcvPdu->Att.OpCode, pRcvPdu->Hdr.Len);

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Hdr = pRcvPdu->Hdr;


	switch (pRcvPdu->Att.OpCode)
	{
		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:
			{
				BtAttExchgMtuReqRsp_t *req = (BtAttExchgMtuReqRsp_t*)&pRcvPdu->Att;

				g_Uart.printf("ATT_EXCHANGE_MTU_REQ:\r\n");
				g_Uart.printf("RxMtu %d\r\n", req->RxMtu);

				l2pdu->Hdr.Len = sizeof(BtAttExchgMtuReqRsp_t);

				BtAttExchgMtuReqRsp_t *att = (BtAttExchgMtuReqRsp_t*)&l2pdu->Att;

				att->OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP;
				att->RxMtu = 512;	// TODO: get real mtu

				acl->Hdr.Len = sizeof(BtAttExchgMtuReqRsp_t) + sizeof(BtL2CapHdr_t);

				uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
				g_Uart.printf("n=%d\r\n", n);
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ:
			{
				g_Uart.printf("ATT_FIND_INFORMATION_REQ:\r\n");
				BtAttFindInfoReq_t *req = (BtAttFindInfoReq_t*)&pRcvPdu->Att;

				g_Uart.printf("sHdl: %x, eHdl: %x\r\n", req->StartHdl, req->EndHdl);

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					BtHciSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttFindInfoRsp_t *rsp = (BtAttFindInfoRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP;

				int l = 0;
#if 0
				BtGattListEntry_t list[50];
				uint16_t lasthdl = 0;

				int c = BtGattGetListHandle(req->StartHdl, req->EndHdl, list, 50, &lasthdl);

				if (c > 0)
				{
					if (list[0].TypeUuid.BaseIdx > 0)
					{
						uint8_t uuid128[16];

						BtUuidGetBase(list[0].TypeUuid.BaseIdx, uuid128);

						rsp->Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128;

						for (int i = 0; i < c; i++)
						{
							rsp->HdlUuid128[i].Hdl = list[i].Hdl;
							uuid128[12] = list[i].TypeUuid.Uuid & 0xff;
							uuid128[13] = list[i].TypeUuid.Uuid >> 8;
							memcpy(rsp->HdlUuid128[i].Uuid, uuid128, 16);

							g_Uart.printf("HDL : %d, ", rsp->HdlUuid128[i].Hdl);

							for (int j = 0; j < 16; j++)
							{
								g_Uart.printf("%02x ", rsp->HdlUuid128[i].Uuid[j]);
							}
							g_Uart.printf("\r\n");
						}

						l = c * sizeof(BtAttHdlUuid128_t);
					}
					else
					{
						rsp->Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;

						for (int i = 0; i < c; i++)
						{
							rsp->HdlUuid16[i].Hdl = list[i].Hdl;
							rsp->HdlUuid16[i].Uuid = list[i].TypeUuid.Uuid;

							g_Uart.printf("HDL : %d, Uuid16 : 0x%04x\r\n", rsp->HdlUuid16[i].Hdl, rsp->HdlUuid16[i].Uuid);

						}

						l = c * sizeof(BtAttHdlUuid16_t);
					}
//					rsp->Fmt = list[0].Uuid.BaseIdx > 0 ? BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128 : BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;
#else
					BtGattListEntry_t en;

					if (BtGattGetEntryHandle(req->StartHdl, &en))
					{
						if (en.TypeUuid.BaseIdx > 0)
						{
							uint8_t uuid128[16];

							BtUuidGetBase(en.TypeUuid.BaseIdx, uuid128);

							rsp->Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128;

						//	for (int i = 0; i < c; i++)
							{
								rsp->HdlUuid128[0].Hdl = en.Hdl;
								uuid128[12] = en.TypeUuid.Uuid & 0xff;
								uuid128[13] = en.TypeUuid.Uuid >> 8;
								memcpy(rsp->HdlUuid128[0].Uuid, uuid128, 16);

								g_Uart.printf("HDL : %d, ", rsp->HdlUuid128[0].Hdl);

								for (int j = 0; j < 16; j++)
								{
									g_Uart.printf("%02x ", rsp->HdlUuid128[0].Uuid[j]);
								}
								g_Uart.printf("\r\n");
							}

							l = sizeof(BtAttHdlUuid128_t);
						}
						else
						{
							rsp->Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;

							//for (int i = 0; i < c; i++)
							{
								rsp->HdlUuid16[0].Hdl = en.Hdl;
								rsp->HdlUuid16[0].Uuid = en.TypeUuid.Uuid;

								g_Uart.printf("HDL : %d, Uuid16 : 0x%04x\r\n", rsp->HdlUuid16[0].Hdl, rsp->HdlUuid16[0].Uuid);

							}

							l = sizeof(BtAttHdlUuid16_t);
						}

#endif

					l2pdu->Hdr.Len = 2 + l;
					acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

					g_Uart.printf("l2pdu len : %d, %d\r\n", l2pdu->Hdr.Len, acl->Hdr.Len);

					uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
					g_Uart.printf("n : %d\r\n", n);

					uint8_t*p = (uint8_t*)acl;
					g_Uart.printf("acl: ");
					for (int i = 0; i < acl->Hdr.Len + sizeof(acl->Hdr); i++)
					{
						g_Uart.printf("0x%02x ", p[i]);
					}
					g_Uart.printf("\r\n");

				}
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
			{
				g_Uart.printf("ATT_READ_BY_TYPE_REQ:\r\n");

				// Only the attributes with attribute handles between and including
				// the Starting Handle and the Ending Handle with the attribute type
				// that is the same as the Attribute Type given will be returned. To
				// search through all attributes, the starting handle shall be set to
				// 0x0001 and the ending handle shall be set to 0xFFFF.

				BtAttReadByTypeReq_t *req = (BtAttReadByTypeReq_t*)&pRcvPdu->Att;

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					BtHciSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttReadByTypeRsp_t *rsp = (BtAttReadByTypeRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP;

				uint8_t *p = (uint8_t*)rsp->Data;

				g_Uart.printf("sHdl: %x, eHdl: %x, Type: %x\r\n", req->StartHdl, req->EndHdl, req->Uuid.Uuid16);
#if 0
				BtGattListEntry_t list[50];
				uint16_t lasthdl = 0;
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};

				int c = BtGattGetListUuid(&uid16, req->StartHdl, list, 50, &lasthdl);

				if (c > 0)
				{
					int l = 0;

					for (int i = 0; i < c; i++)
					{
						if (list[i].Hdl <= req->EndHdl)// && baseidx == list[i].Uuid.BaseIdx)
						{
							p[0] = list[i].Hdl & 0xFF;
							p[1] = list[i].Hdl >> 8;
							p +=2;

							rsp->Len = BtGattGetValue(&list[i], p) + 2;

							p += rsp->Len;
							l += rsp->Len;
						}
					}

					if (l > 0)
					{
						l2pdu->Hdr.Len = 2 + l;//rsp->Len * c;
						acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

						g_Uart.printf("rsp->Len : %d, hdr.len : %d, %d\r\n", rsp->Len, l2pdu->Hdr.Len, acl->Hdr.Len);

						uint32_t n = s_HciDevice.SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
						g_Uart.printf("n : %d\r\n", n);
						break;
					}
				}
#else
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};
				BtGattListEntry_t entry;

				if (BeGattFindEntryUuid(&uid16, req->StartHdl, req->EndHdl, &entry))
				{
					p[0] = entry.Hdl & 0xFF;
					p[1] = entry.Hdl >> 8;
					p +=2;
					rsp->Len = BtGattGetValue(&entry, p) + 2;
					g_Uart.printf("Att Val: ");

					for (int j = 0; j < rsp->Len; j++)
					{
						g_Uart.printf("%x ", rsp->Data[j]);
					}
					g_Uart.printf("\r\n");

					l2pdu->Hdr.Len = 2 + rsp->Len;//rsp->Len * c;
					acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

					g_Uart.printf("rsp->Len : %d, hdr.len : %d, %d\r\n", rsp->Len, l2pdu->Hdr.Len, acl->Hdr.Len);

					uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
					g_Uart.printf("n : %d\r\n", n);
					break;
				}
#endif
				BtHciSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ:
			{
				// The ATT_READ_REQ PDU is used to request the server to read the value
				// of an attribute and return its value in an ATT_READ_RSP PDU.
				BtAttReadReq_t *req = (BtAttReadReq_t*)&pRcvPdu->Att;
				BtAttReadRsp_t *rsp = (BtAttReadRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_READ_RSP;

				g_Uart.printf("ATT_READ_REQ : Hdl = %d\r\n", req->Hdl);

				BtGattListEntry_t eg;

				if (BtGattGetEntryHandle(req->Hdl, &eg) == true)
				{
					int baseidx = eg.Uuid.BaseIdx;
					int l = BtGattGetValue(&eg, rsp->Data);

					g_Uart.printf("Data : ");
					for (int j= 0; j < l; j++)
					{
						g_Uart.printf("%x ", rsp->Data[j]);
					}
					g_Uart.printf("\r\n");
					/*
					BtUuid_t uid;
					uid.Type = BT_UUID_TYPE_16;

					if (baseidx > 0)
					{
						l = 16;
						uid.Type = BT_UUID_TYPE_128;
						BtUuidGetBase(baseidx, rsp->Data);

						rsp->Data[12] = eg.Uuid.Uuid & 0xFF;
						rsp->Data[13] = eg.Uuid.Uuid >> 8;
					}
					else
					{
						l = 2;
						rsp->Data[0] = eg.Uuid.Uuid & 0xFF;
						rsp->Data[1] = eg.Uuid.Uuid >> 8;
					}
*/
					l2pdu->Hdr.Len = sizeof(BtAttReadRsp_t) - 1 + l;
					acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

					uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
					g_Uart.printf("l : %d, %d, %d\r\n", l, l2pdu->Hdr.Len, acl->Hdr.Len);
				}
				else
				{
					BtHciSendError(pDev, acl, req->Hdl, BT_ATT_OPCODE_ATT_READ_REQ, BT_ATT_ERROR_INVALID_HANDLE);
/*
					BtAttErrorRsp_t *errsp = (BtAttErrorRsp_t*)&l2pdu->Att;

					errsp->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
					errsp->ReqOpCode = BT_ATT_OPCODE_ATT_READ_REQ;
					errsp->Hdl = 1;
					errsp->Error = BT_ATT_ERROR_ATT_NOT_FOUND;

					l2pdu->Hdr.Len = sizeof(BtAttErrorRsp_t);
					acl->Hdr.Len = sizeof(BtAttErrorRsp_t) + sizeof(BtL2CapHdr_t);*/
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ:
			{
				BtAttBlobReq_t *req = (BtAttBlobReq_t*)&pRcvPdu->Att;

				g_Uart.printf("BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ:\r\n");
				g_Uart.printf("Hdl: %x, offset: %x\r\n", req->Hdl, req->Offset);

				BtAttBlobRsp_t *rsp = (BtAttBlobRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_READ_REQ_BLOB_RSP;
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ:
			// The ATT_READ_MULTIPLE_REQ PDU is used to request the server to read
			// two or more values of a set of attributes and return their values in
			// an ATT_READ_MULTIPLE_RSP PDU. Only values that have a known fixed size
			// can be read, with the exception of the last value that can have a variable
			// length. The knowledge of whether attributes have a known fixed size is
			// defined in a higher layer specification.
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
			{
				BtAttReadByGroupTypeReq_t *req = (BtAttReadByGroupTypeReq_t*)&pRcvPdu->Att;

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					BtHciSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttReadByGroupTypeRsp_t *rsp = (BtAttReadByGroupTypeRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP;

				g_Uart.printf("ATT_READ_BY_GROUP_TYPE_REQ:\r\n");
				g_Uart.printf("sHdl:%d, eHdl:%d, Uuid: ", req->StartHdl, req->EndHdl, req->Uid.Uuid16);

				if (pRcvPdu->Hdr.Len < 8)
				{
					g_Uart.printf("%x\r\n", req->Uid.Uuid16);

					BtGattListEntry_t list[20];
					uint16_t lasthdl = 0;
					BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uid.Uuid16};

					int c = BtGattGetListUuid(&uid16, req->StartHdl, list, 20, &lasthdl);

					uint8_t *p = (uint8_t*)rsp->Data;

					rsp->Len = 0;

					if (c > 0)
					{
						int baseidx = list[0].Uuid.BaseIdx;
						int l = 0;
						//BtUuid_t uid;
						//uid.Type = BT_UUID_TYPE_16;
//						rsp->Len = BtGattGetValueLen(list, );

						for (int i = 0; i < c && list[i].Hdl <= req->EndHdl && baseidx == list[i].Uuid.BaseIdx; i++)
						{
							BtAttHdlRange_t *hu = (BtAttHdlRange_t*)p;
							hu->StartHdl = list[i].Hdl;
							if ((i + 1) < c)
							{
								hu->EndHdl = list[i+1].Hdl - 1;
							}
							else
							{
								hu->EndHdl = lasthdl;
							}
							g_Uart.printf("sHdl: %d, eHdl: %d\r\n", hu->StartHdl, hu->EndHdl);

							p += sizeof(BtAttHdlRange_t);

							rsp->Len = BtGattGetValue(&list[i], p);

							g_Uart.printf("UUID: ");
							for (int j = 0; j < rsp->Len; j++)
							{
								g_Uart.printf("%x ", p[j]);
							}
							g_Uart.printf("\r\n");

							p += rsp->Len;
							l += 4 + rsp->Len;
						}

						if (l > 0)
						{
							rsp->Len += 4;

							l2pdu->Hdr.Len = sizeof(BtAttReadByGroupTypeRsp_t) - 1 + l;
							acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

							g_Uart.printf("rsp->Len : %d, %d, %d, %d\r\n", rsp->Len, l, l2pdu->Hdr.Len, acl->Hdr.Len);
							uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
							g_Uart.printf("n=%d\r\n", n);
							break;
						}
					}

					g_Uart.printf("List not found\r\n");

					BtHciSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
/*
					BtAttErrorRsp_t *errsp = (BtAttErrorRsp_t*)&l2pdu->Att;

					errsp->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
					errsp->ReqOpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ;
					errsp->Hdl = 1;
					errsp->Error = BT_ATT_ERROR_ATT_NOT_FOUND;

					l2pdu->Hdr.Len = sizeof(BtAttErrorRsp_t);
					acl->Hdr.Len = sizeof(BtAttErrorRsp_t) + sizeof(BtL2CapHdr_t);

					g_Uart.printf("Error\r\n");

					uint32_t n = s_HciDevice.SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
					g_Uart.printf("n=%d\r\n", n);
*/
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
			{
				BtAttWriteReq_t *req = (BtAttWriteReq_t*)&pRcvPdu->Att;

				g_Uart.printf("BT_ATT_OPCODE_ATT_WRITE_REQ:\r\n");
				g_Uart.printf("Hdl: %x\r\n", req->Hdl);

				if (req->Hdl < 1)
				{
					BtHciSendError(pDev, acl, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}
				BtAttWriteRsp_t *rsp = (BtAttWriteRsp_t*)&l2pdu->Att;
				size_t l = pRcvPdu->Hdr.Len;

				l = BtGattWriteValue(req->Hdl, req->Data, l);

				rsp->OpCode = BT_ATT_OPCODE_ATT_WRITE_RSP;

				l2pdu->Hdr.Len = 1;
				acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

				g_Uart.printf("l2pdu->Hdr.Len %d, acl->Hdr.Len %d\r\n", l2pdu->Hdr.Len, acl->Hdr.Len);
				uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
				g_Uart.printf("n=%d\r\n", n);
			}
			break;
		case BT_ATT_OPCODE_ATT_CMD:
			{
				BtAttSignedWriteCmd_t *req = (BtAttSignedWriteCmd_t*)&pRcvPdu->Att;

				g_Uart.printf("ATT_CMD:\r\n");
				g_Uart.printf("Hdl: %x, data len: %d\r\n", req->Hdl, pRcvPdu->Hdr.Len -  3);

				size_t l = pRcvPdu->Hdr.Len;

				l = BtGattWriteValue(req->Hdl, req->Data, l - 3);
			}
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

void BtHciProcessData(BtHciDevice_t *pDev, BtHciACLDataPacket_t *pPkt)
{
	BtL2CapPdu_t *l2frame = (BtL2CapPdu_t*)pPkt->Data;

	g_Uart.printf("** BtHciProcessData : Con :%d, PB :%d, PC :%d, Len :%d\r\n", pPkt->Hdr.ConnHdl, pPkt->Hdr.PBFlag, pPkt->Hdr.BCFlag, pPkt->Hdr.Len);
	for (int i = 0; i < pPkt->Hdr.Len; i++)
	{
		g_Uart.printf("%x ", pPkt->Data[i]);
	}
	g_Uart.printf("\r\nCID: %x\r\n", l2frame->Hdr.Cid);

	switch (l2frame->Hdr.Cid)
	{
		case BT_L2CAP_CID_ACL_U:
			break;
		case BT_L2CAP_CID_CONNECTIONLESS:
			break;
		case BT_L2CAP_CID_ATT:
			BtProcessAttData(pDev, pPkt->Hdr.ConnHdl, l2frame);
			break;
		case BT_L2CAP_CID_SIGNAL:
			break;
		case BT_L2CAP_CID_SEC_MNGR:
			break;
	}
	g_Uart.printf("-----\r\n");
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

void BtHciMotify(BtHciDevice_t *pDev, uint16_t ConnHdl, uint16_t ValHdl, void * const pData, size_t Len)
{
	uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	g_Uart.printf("BtHciMotify : %d %d %d", ConnHdl, s_ConnHdl, ValHdl);

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF;

	BtGattCharNotify_t *p = (BtGattCharNotify_t*)l2pdu->Att.Param;

	p->ValHdl = ValHdl;
	memcpy(p->Data, pData, Len);
	l2pdu->Hdr.Len = sizeof(BtGattCharNotify_t) + Len;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	g_Uart.printf("Len : %d, %d, %d\r\n", Len, l2pdu->Hdr.Len, acl->Hdr.Len);
	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
	g_Uart.printf("n=%d\r\n", n);
}

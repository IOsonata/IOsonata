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
#include "syslog.h"

void BtProcessAttData(BtHciDevice_t * const pDev, uint16_t ConnHdl, BtL2CapPdu_t * const pRcvPdu);

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the app configured (UART, USB, RTT, BLE, or any other
// DeviceIntrf); the trace does not assume a transport. A release build
// defines NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
// The disabled form keeps the arguments in an unevaluated sizeof so variables
// used only in debug prints do not raise unused warnings. No code and no call
// is generated, the expression is never evaluated.
#define DEBUG_PRINTF(...)	((void)sizeof(SysLogPrintf(SysLogGet(), __VA_ARGS__)))
#endif
/*******************************/

//alignas(4) static BtHciDevice_t s_HciDevice = {0,};



// EvtLen is the HCI event parameter length (bytes of pLeEvtPkt, i.e. Evt + Data),
// used to bound walks over variable-length, controller-supplied report lists.
// --- ACL fragment reassembly -------------------------------------------------
// The controller delivers an L2CAP PDU larger than its ACL data buffer as a
// START fragment followed by CONTINUING fragments. One context per connection
// holds the partial PDU until it is complete. BT_L2CAP_REASSEMBLY_COUNT bounds
// how many connections can be mid-reassembly at once; raise it for multi-link.
#ifndef BT_L2CAP_REASSEMBLY_COUNT
#define BT_L2CAP_REASSEMBLY_COUNT		2
#endif

typedef struct __Bt_Hci_Reassembly {
	bool     Active;						//!< Slot holds a partial PDU
	uint16_t ConnHdl;						//!< Owning connection handle
	uint16_t Expected;						//!< Total L2CAP PDU bytes expected
	uint16_t Received;						//!< Bytes accumulated so far
	uint8_t  Buf[BT_HCI_BUFFER_MAX_SIZE];	//!< Accumulated L2CAP PDU
} BtHciReasm_t;

static BtHciReasm_t s_BtHciReasm[BT_L2CAP_REASSEMBLY_COUNT];

// --- Extended advertising report reassembly ----------------------------------
// A single advertising event can be split across several LE Extended Advertising
// Report subevents (Data_Status = "incomplete, more data to come"). One context
// per advertiser, keyed by address type, address and Advertising SID, holds the
// partial AD until the terminating fragment (Data_Status = "complete") so the
// application receives one reassembled report instead of disjoint fragments
// (Core Vol 4 Part E 7.7.65.13).
#ifndef BT_EXT_ADV_REASSEMBLY_COUNT
#define BT_EXT_ADV_REASSEMBLY_COUNT		2
#endif
#ifndef BT_EXT_ADV_REASSEMBLY_MAX
#define BT_EXT_ADV_REASSEMBLY_MAX		256
#endif

typedef struct __Bt_ExtAdv_Reassembly {
	bool     Active;						//!< Slot holds a partial report
	uint8_t  AddrType;						//!< Advertiser address type
	uint8_t  Addr[6];						//!< Advertiser address
	uint8_t  Sid;							//!< Advertising SID
	uint16_t Len;							//!< Bytes accumulated so far
	uint8_t  Buf[BT_EXT_ADV_REASSEMBLY_MAX];//!< Accumulated AD
} BtExtAdvReasm_t;

static BtExtAdvReasm_t s_BtExtAdvReasm[BT_EXT_ADV_REASSEMBLY_COUNT];

// Advertisers whose reassembly was abandoned, because no context was free or
// because the accumulated data overflowed. Their remaining fragments must not
// be mistaken for standalone reports: the terminating fragment of a dropped
// reassembly looks exactly like a complete single-fragment report, and
// delivering it hands the application a truncated suffix as if it were whole
// advertising data.
#ifndef BT_EXT_ADV_DROPPED_COUNT
#define BT_EXT_ADV_DROPPED_COUNT		4
#endif

typedef struct __Bt_ExtAdv_Dropped {
	bool    Active;							//!< Slot names a dropped reassembly
	uint8_t AddrType;						//!< Advertiser address type
	uint8_t Addr[6];						//!< Advertiser address
	uint8_t Sid;							//!< Advertising SID
} BtExtAdvDropped_t;

static BtExtAdvDropped_t s_BtExtAdvDropped[BT_EXT_ADV_DROPPED_COUNT];
static uint8_t s_BtExtAdvDroppedNext = 0;

static BtExtAdvDropped_t *BtExtAdvDroppedFind(uint8_t AddrType,
											  const uint8_t Addr[6], uint8_t Sid)
{
	for (int i = 0; i < BT_EXT_ADV_DROPPED_COUNT; i++)
	{
		BtExtAdvDropped_t *d = &s_BtExtAdvDropped[i];
		if (d->Active && d->AddrType == AddrType && d->Sid == Sid &&
			memcmp(d->Addr, Addr, 6) == 0)
		{
			return d;
		}
	}
	return nullptr;
}

// Oldest entry is overwritten. Losing the record of a very old dropped
// reassembly only risks delivering one stale suffix; holding the table open
// forever would be worse.
static void BtExtAdvDroppedMark(uint8_t AddrType, const uint8_t Addr[6], uint8_t Sid)
{
	if (BtExtAdvDroppedFind(AddrType, Addr, Sid) != nullptr)
	{
		return;
	}

	BtExtAdvDropped_t *d = &s_BtExtAdvDropped[s_BtExtAdvDroppedNext];
	s_BtExtAdvDroppedNext = (uint8_t)((s_BtExtAdvDroppedNext + 1) % BT_EXT_ADV_DROPPED_COUNT);

	d->Active = true;
	d->AddrType = AddrType;
	d->Sid = Sid;
	memcpy(d->Addr, Addr, 6);
}

static BtExtAdvReasm_t *BtExtAdvReasmFind(uint8_t AddrType, const uint8_t Addr[6], uint8_t Sid)
{
	for (int i = 0; i < BT_EXT_ADV_REASSEMBLY_COUNT; i++)
	{
		BtExtAdvReasm_t *c = &s_BtExtAdvReasm[i];
		if (c->Active && c->AddrType == AddrType && c->Sid == Sid &&
			memcmp(c->Addr, Addr, 6) == 0)
		{
			return c;
		}
	}
	return nullptr;
}

static BtExtAdvReasm_t *BtExtAdvReasmAlloc(uint8_t AddrType, const uint8_t Addr[6], uint8_t Sid)
{
	for (int i = 0; i < BT_EXT_ADV_REASSEMBLY_COUNT; i++)
	{
		BtExtAdvReasm_t *c = &s_BtExtAdvReasm[i];
		if (c->Active == false)
		{
			c->Active   = true;
			c->AddrType = AddrType;
			c->Sid      = Sid;
			memcpy(c->Addr, Addr, 6);
			c->Len      = 0;
			return c;
		}
	}
	return nullptr;
}

void BtHciProcessLeEvent(BtHciDevice_t * const pDev, BtHciLeEvtPacket_t *pLeEvtPkt, int EvtLen)
{
	// End of the received event payload; report parsers must not read past it.
	const uint8_t *evtEnd = (const uint8_t*)pLeEvtPkt + (EvtLen > 0 ? EvtLen : 0);

	//DEBUG_PRINTF("BtHciProcessMetaEvent : Evt %x\r\n", pLeEvtPkt->Evt);

	switch (pLeEvtPkt->Evt)
	{
		case BT_HCI_EVT_LE_CONN_COMPLETE:
			{
				if ((const uint8_t*)pLeEvtPkt->Data + sizeof(BtHciLeEvtConnComplete_t) > evtEnd)
				{
					break;
				}
				BtHciLeEvtConnComplete_t *p = (BtHciLeEvtConnComplete_t*)pLeEvtPkt->Data;
				if (p->Status == 0)
				{
					if (pDev->Connected)
					{
						pDev->Connected(p->ConnHdl, p->Role, p->PeerAddrType, p->PeerAddr);
					}
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

				// The dispatcher proved the subevent byte exists, not the count
				// byte after it. Reading NbReport before checking is an
				// out-of-bounds read on a truncated event.
				if ((const uint8_t*)pLeEvtPkt->Data + 1 > evtEnd)
				{
					break;
				}

				// Each BtAdvReport_t is variable-length: 9-byte fixed header
				// (EvtType, AddrType, Addr[6], DataLen) + Data[DataLen] +
				// 1 byte trailing RSSI. Walk by stride, do not index as
				// fixed-size array.
				uint8_t *cur = (uint8_t*)report->Report;

				for (int i = 0; i < report->NbReport; i++)
				{
					BtAdvReport_t *r = (BtAdvReport_t*)cur;
					// Bound every access against the received event length:
					// the 9-byte fixed header, the DataLen payload, and the
					// trailing RSSI byte. A malformed/hostile controller event
					// with a bad NbReport/DataLen would otherwise read OOB.
					if (cur + 9 > evtEnd)
					{
						break;
					}
					if (cur + 9 + r->DataLen + 1 > evtEnd)
					{
						break;
					}
					int8_t rssi = (int8_t)r->Data[r->DataLen];
					if (pDev->ScanReport)
					{
						pDev->ScanReport(rssi, r->AddrType, r->Addr, r->DataLen, r->Data);
					}
					cur += 9 + r->DataLen + 1;
				}
			}
			break;
		case BT_HCI_EVT_LE_CONN_UPDATE_COMPLETE:
			{
				if ((const uint8_t*)pLeEvtPkt->Data + sizeof(BtHciLeEvtConnUpdateComplete_t) > evtEnd)
				{
					break;
				}
				BtHciLeEvtConnUpdateComplete_t *p = (BtHciLeEvtConnUpdateComplete_t*)pLeEvtPkt->Data;
				DEBUG_PRINTF("BT_HCI_EVT_LE_CONN_UPDATE_COMPLETE \r\n");
				DEBUG_PRINTF("Latency = %d, SuperTimeout = %d\r\n", p->Latency, p->SuperTimeout);
			}
			break;
		case BT_HCI_EVT_LE_READ_REMOTE_FEATURES_COMPLETE:
			{
//				BtHciLeEvtReadRemoteFeatureComplete_t *p = (BtHciLeEvtReadRemoteFeatureComplete_t*)pLeEvtPkt->Data;
//				DEBUG_PRINTF("BT_HCI_EVT_LE_READ_REMOTE_FEATURES_COMPLETE \r\n");
			}
			break;
		case BT_HCI_EVT_LE_LONGTERM_KEY_RQST:
			{
				if ((const uint8_t*)pLeEvtPkt->Data + sizeof(BtHciLeEvtLongtermKeyReq_t) > evtEnd)
				{
					break;
				}
				BtHciLeEvtLongtermKeyReq_t *p = (BtHciLeEvtLongtermKeyReq_t*)pLeEvtPkt->Data;
				BtSmpProcessLtkRequest(pDev, p->ConnHdl, p->RandNumber, p->EncryptDivers);
			}
			break;
		case BT_HCI_EVT_LE_REMOTE_CONN_PARAM_RQST:
			{
//				BtHciLeEvtRemoteConnParamReq_t *p = (BtHciLeEvtRemoteConnParamReq_t*)pLeEvtPkt->Data;
				DEBUG_PRINTF("BT_HCI_EVT_LE_REMOTE_CONN_PARAM_RQST\r\n");
			}
			break;
		case BT_HCI_EVT_LE_DATA_LEN_CHANGE:
			{
				if ((const uint8_t*)pLeEvtPkt->Data + sizeof(BtHciLeEvtDataLenChange_t) > evtEnd)
				{
					break;
				}
				BtHciLeEvtDataLenChange_t *p = (BtHciLeEvtDataLenChange_t*)pLeEvtPkt->Data;

				pDev->RxDataLen = p->MaxRxLen;
				pDev->TxDataLen = p->MaxTxLen;

				DEBUG_PRINTF("BT_HCI_EVT_LE_DATA_LEN_CHANGE: ConnHdl=%d, MaxRxLen=%d\r\n", p->ConnHdl, p->MaxRxLen);
				DEBUG_PRINTF(
						"MaxRxLen = %d, MaxRxTime = %d \r\n"
						"MaxTxLen = %d, MaxTxTime = %d \r\n",
						p->MaxRxLen, p->MaxRxTime, p->MaxTxLen, p->MaxTxTime);
			}
			break;
		case BT_HCI_EVT_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE:
			{
				if ((const uint8_t*)pLeEvtPkt->Data + sizeof(BtHciLeEvtReadLocalP256PubKeyComplete_t) > evtEnd)
				{
					break;
				}
				BtHciLeEvtReadLocalP256PubKeyComplete_t *p =
						(BtHciLeEvtReadLocalP256PubKeyComplete_t*)pLeEvtPkt->Data;
				BtSmpLocalPubKeyReady(pDev, p->Status, p->KeyXCoord, p->KeyYCoord);
			}
			break;
		case BT_HCI_EVT_LE_GENERATE_DHKEY_COMPLETE:
			{
				if ((const uint8_t*)pLeEvtPkt->Data + sizeof(BtHciLeEvtGenerateDHKeyComplete_t) > evtEnd)
				{
					break;
				}
				BtHciLeEvtGenerateDHKeyComplete_t *p =
						(BtHciLeEvtGenerateDHKeyComplete_t*)pLeEvtPkt->Data;
				BtSmpDhKeyReady(pDev, p->Status, p->DHKey);
			}
			break;
		case BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE_V1:
		case BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE_V2:
			{
				if ((const uint8_t*)pLeEvtPkt->Data + sizeof(BtHciLeEvtEnhConnComplete_t) > evtEnd)
				{
					break;
				}
				BtHciLeEvtEnhConnComplete_t *p = (BtHciLeEvtEnhConnComplete_t*)pLeEvtPkt->Data;

//				DEBUG_PRINTF("BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE : hdl %x, role:%d\n", p->ConnHdl, p->Role);

				if (p->Status == 0)
				{
					DEBUG_PRINTF("BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE_V1/V2\r\n");
					DEBUG_PRINTF("Target device Role = %d\r\n", p->Role);
					if (pDev->Connected)
					{
						pDev->Connected(p->ConnHdl, p->Role, p->PeerAddrType, p->PeerAddr);
					}

					//s_ConnHdl = p->ConnHdl;

					//g_BleConn.Handle = p->ConnHdl;
					//g_BleConn.PeerAddrType = p->PeerAddrType;
					//memcpy(g_BleConn.PeerAddr, p->PeerAddr, 6);
					//BLEAPP_ROLE role = p->Role == 1 ? BLEAPP_ROLE_PERIPHERAL : BLEAPP_ROLE_CENTRAL;
				}
			}
			break;
		case BT_HCI_EVT_LE_DIRECTED_ADV_REPORT:
			break;
		case BT_HCI_EVT_LE_PHY_UPDATE_COMPLETE:
		{
			if ((const uint8_t*)pLeEvtPkt->Data + sizeof(BtHciLeEvtPhyUpdateComplete_t) > evtEnd)
			{
				break;
			}
			BtHciLeEvtPhyUpdateComplete_t *p = (BtHciLeEvtPhyUpdateComplete_t *) pLeEvtPkt->Data;
			DEBUG_PRINTF("Status = %d, RxPhy = %d, TxPhy = %d \r\n", p->Status, p->RxPhy, p->TxPhy);
		}
			break;
		case BT_HCI_EVT_LE_EXT_ADV_REPORT:
			{
				BtHciLeEvtExtAdvReport_t *report = (BtHciLeEvtExtAdvReport_t*)pLeEvtPkt->Data;

				// Same as the legacy report: prove the count byte is inside the
				// event before reading it.
				if ((const uint8_t*)pLeEvtPkt->Data + 1 > evtEnd)
				{
					break;
				}

				// Each BtExtAdvReport_t is variable-length: 24-byte fixed
				// header + Data[DataLen]. Walk by stride.
				uint8_t *cur = (uint8_t*)report->Report;

				for (int i = 0; i < report->NbReport; i++)
				{
					BtExtAdvReport_t *r = (BtExtAdvReport_t*)cur;
					// Bound the 24-byte fixed header and DataLen payload against
					// the received event length before dereferencing.
					if (cur + 24 > evtEnd)
					{
						break;
					}
					if (cur + 24 + r->DataLen > evtEnd)
					{
						break;
					}

					// Data_Status is bits 5-6 of the Event_Type field: 0 =
					// complete, 1 = incomplete with more data to come, 2 =
					// incomplete and truncated (no more data), 3 = reserved.
					uint8_t dataStatus = (uint8_t)((r->Type >> 5) & 0x03);
					BtExtAdvReasm_t *ctx =
						BtExtAdvReasmFind(r->AddrType, r->Addr, r->AdvSid);

					if (dataStatus == 1)
					{
						// More data to come: buffer this fragment.
						if (ctx == nullptr &&
							BtExtAdvDroppedFind(r->AddrType, r->Addr, r->AdvSid) == nullptr)
						{
							ctx = BtExtAdvReasmAlloc(r->AddrType, r->Addr, r->AdvSid);
						}
						if (ctx != nullptr)
						{
							if ((uint32_t)ctx->Len + r->DataLen <= BT_EXT_ADV_REASSEMBLY_MAX)
							{
								memcpy(ctx->Buf + ctx->Len, r->Data, r->DataLen);
								ctx->Len = (uint16_t)(ctx->Len + r->DataLen);
							}
							else
							{
								// Reassembly buffer would overflow: drop the
								// partial report rather than deliver a truncation.
								ctx->Active = false;
								BtExtAdvDroppedMark(r->AddrType, r->Addr, r->AdvSid);
							}
						}
						else
						{
							// No context free, or this advertiser is already
							// abandoned. Either way the report cannot be
							// reassembled and its tail must not be delivered.
							BtExtAdvDroppedMark(r->AddrType, r->Addr, r->AdvSid);
						}
					}
					else if (dataStatus == 0)
					{
						// Complete. Deliver the reassembled AD if this terminates
						// a multi-fragment report, otherwise deliver directly.
						if (ctx != nullptr)
						{
							if ((uint32_t)ctx->Len + r->DataLen <= BT_EXT_ADV_REASSEMBLY_MAX)
							{
								memcpy(ctx->Buf + ctx->Len, r->Data, r->DataLen);
								ctx->Len = (uint16_t)(ctx->Len + r->DataLen);
								if (pDev->ScanReport)
								{
									pDev->ScanReport(r->Rssi, r->AddrType, r->Addr, ctx->Len, ctx->Buf);
								}
							}
							ctx->Active = false;
						}
						else
						{
							// No context. Either this is a standalone complete
							// report, or it is the tail of a reassembly that was
							// abandoned, which looks identical on the wire.
							// Delivering the second case hands the application a
							// truncated suffix as whole advertising data.
							BtExtAdvDropped_t *d =
								BtExtAdvDroppedFind(r->AddrType, r->Addr, r->AdvSid);

							if (d != nullptr)
							{
								d->Active = false;
							}
							else if (pDev->ScanReport)
							{
								pDev->ScanReport(r->Rssi, r->AddrType, r->Addr, r->DataLen, r->Data);
							}
						}
					}
					else
					{
						// Truncated (2) or reserved (3): no more data is coming
						// and the report is incomplete. Drop any partial state
						// and do not deliver a partial report as if complete.
						if (ctx != nullptr)
						{
							ctx->Active = false;
						}
					}

					cur += 24 + r->DataLen;
				}
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
			if (pDev->AdvTimeout)
			{
				pDev->AdvTimeout();
			}
			break;
		case BT_HCI_EVT_LE_SCAN_RQST_RECEIVED:
			break;
		case BT_HCI_EVT_LE_CHAN_SELECTION_ALGO:
			{
//				BtHciLeEvtChanSelAlgo_t *p = (BtHciLeEvtChanSelAlgo_t *)pLeEvtPkt->Data;

//				DEBUG_PRINTF("BT_HCI_EVT_LE_CHAN_SELECTION_ALGO : hdl:%x %d\r\n", p->ConnHdl, p->ChanSelAlgo);
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
static BtHciReasm_t *BtHciReasmFind(uint16_t ConnHdl)
{
	for (int i = 0; i < BT_L2CAP_REASSEMBLY_COUNT; i++)
	{
		if (s_BtHciReasm[i].Active && s_BtHciReasm[i].ConnHdl == ConnHdl)
		{
			return &s_BtHciReasm[i];
		}
	}

	return nullptr;
}

static BtHciReasm_t *BtHciReasmAlloc(uint16_t ConnHdl)
{
	BtHciReasm_t *p = BtHciReasmFind(ConnHdl);

	if (p != nullptr)
	{
		return p;
	}

	for (int i = 0; i < BT_L2CAP_REASSEMBLY_COUNT; i++)
	{
		if (s_BtHciReasm[i].Active == false)
		{
			return &s_BtHciReasm[i];
		}
	}

	return nullptr;
}

static void BtHciReasmReset(uint16_t ConnHdl)
{
	BtHciReasm_t *p = BtHciReasmFind(ConnHdl);

	if (p != nullptr)
	{
		p->Active = false;
	}
}

// Configure the controller's LE ACL buffer parameters. Clamps the packet
// length to what a single host buffer can hold.
void BtHciSetLeAclBuffer(BtHciDevice_t * const pDev, uint16_t MaxLen, uint8_t PktCount)
{
	if (pDev == nullptr)
	{
		return;
	}

	uint16_t cap = (uint16_t)(BT_HCI_BUFFER_MAX_SIZE - sizeof(BtHciACLDataPacketHdr_t));
	if (MaxLen > cap)
	{
		MaxLen = cap;
	}

	pDev->AclMaxLen = MaxLen;
	pDev->AclCreditMax = PktCount;
	pDev->AclCredit = (int16_t)PktCount;

	// Command flow control starts with one credit per spec: the host may send a
	// single command before the first Command Complete or Command Status.
	pDev->CmdCredit = 1;
}

// Terminate a link the host can no longer keep consistent. Reason 0x13 (Remote
// User Terminated Connection) is the conventional host-initiated code and one
// of the few the Disconnect command accepts (Vol 4 Part E 7.1.6).
static void BtHciDropLink(BtHciDevice_t * const pDev, uint16_t ConnHdl)
{
	if (pDev == nullptr || pDev->Command == nullptr)
	{
		return;
	}

	uint8_t param[3];
	param[0] = (uint8_t)(ConnHdl & 0xFF);
	param[1] = (uint8_t)((ConnHdl >> 8) & 0xFF);
	param[2] = 0x13;

	pDev->Command(pDev, BT_HCI_CMD_LINKCTRL_DISCONNECT, param, sizeof(param), nullptr, 0);
}

uint32_t BtHciSendAcl(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pAcl)
{
	if (pDev == nullptr || pDev->SendData == nullptr || pAcl == nullptr)
	{
		return 0;
	}

	uint16_t l2Len = (uint16_t)pAcl->Hdr.Len;	// L2CAP PDU bytes (header + payload)

	// Single-packet path: used when fragmentation is not configured or the PDU
	// already fits one ACL data packet. Byte-for-byte the prior behaviour, plus
	// a credit gate when flow control is configured.
	if (pDev->AclMaxLen == 0 || l2Len <= pDev->AclMaxLen)
	{
		if (pDev->AclCreditMax > 0)
		{
			if (pDev->AclCredit <= 0)
			{
				return 0;					// no controller buffer available
			}
		}

		uint32_t txLen = (uint32_t)l2Len + sizeof(pAcl->Hdr);
		uint32_t sent = pDev->SendData((uint8_t*)pAcl, txLen);
		if (sent != txLen)
		{
			return 0;
		}
		if (pDev->AclCreditMax > 0)
		{
			pDev->AclCredit--;
		}
		return sent;
	}

	// Fragmentation path: split the L2CAP PDU across ACL packets of at most
	// AclMaxLen payload bytes. The first holds a START boundary flag, the rest
	// CONTINUING.
	uint16_t nFrag = (uint16_t)((l2Len + pDev->AclMaxLen - 1) / pDev->AclMaxLen);

	// Take the credits for the whole PDU before the first fragment goes out.
	// The earlier code only compared the count and then decremented one
	// fragment at a time, so anything else sending on this device between
	// fragments could spend the credits this PDU still needed and leave the
	// controller holding a START with no way to finish it.
	if (pDev->AclCreditMax > 0)
	{
		if (pDev->AclCredit < (int16_t)nFrag)
		{
			return 0;
		}

		pDev->AclCredit = (int16_t)(pDev->AclCredit - (int16_t)nFrag);
	}

	uint8_t fbuf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *frag = (BtHciACLDataPacket_t*)fbuf;
	const uint8_t *src = pAcl->Data;
	uint16_t off = 0;
	uint16_t nSent = 0;

	while (off < l2Len)
	{
		uint16_t chunk = (uint16_t)(l2Len - off);
		if (chunk > pDev->AclMaxLen)
		{
			chunk = pDev->AclMaxLen;
		}

		frag->Hdr.ConnHdl = pAcl->Hdr.ConnHdl;
		frag->Hdr.PBFlag = (off == 0) ? BT_HCI_PBFLAG_START_NONFLUSHABLE
									  : BT_HCI_PBFLAG_CONTINUING_FRAGMENT;
		frag->Hdr.BCFlag = 0;
		frag->Hdr.Len = chunk;
		memcpy(frag->Data, src + off, chunk);

		uint32_t txLen = (uint32_t)chunk + sizeof(frag->Hdr);
		uint32_t sent = pDev->SendData((uint8_t*)frag, txLen);
		if (sent != txLen)
		{
			// Hand back the credits for the fragments that never went out. The
			// ones already accepted stay spent; the controller holds those
			// buffers until it reports them completed.
			if (pDev->AclCreditMax > 0)
			{
				pDev->AclCredit = (int16_t)(pDev->AclCredit + (int16_t)(nFrag - nSent));
			}

			if (nSent > 0)
			{
				// A START fragment is already in the controller and the rest of
				// this PDU cannot follow it. Nothing completes that PDU, and the
				// caller cannot resend the whole thing without putting a second
				// START on a link that is still waiting for the first one to
				// finish, so drop the link instead (Vol 4 Part E 5.4.2 has no
				// way to withdraw a fragment already given to the controller).
				BtHciDropLink(pDev, pAcl->Hdr.ConnHdl);
			}

			return 0;
		}

		nSent++;
		off = (uint16_t)(off + chunk);
	}

	return (uint32_t)l2Len + sizeof(pAcl->Hdr);
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
				// Status(1) + ConnHdl(2) + Reason(1); the handler hands the
				// handle to SMP and the app, so a short event must not reach
				// them with fields read past the packet.
				if (pEvtPkt->Hdr.Len < sizeof(BtHciEvtDisconComplete_t))
				{
					break;
				}

				BtHciEvtDisconComplete_t *p = (BtHciEvtDisconComplete_t*)pEvtPkt->Data;
				SysLogPrintf(SysLogGet(), "HCI disconnect hdl=%d reason=0x%02x\r\n",
							 p->ConnHdl, p->Reason);
				DEBUG_PRINTF("BtHciProcessEvent: Disconnected, Status = %d (0x%x) \r\n",
						p->Status, p->Status);

				if (pDev->Disconnected)
				{
					pDev->Disconnected(p->ConnHdl, p->Reason);
				}

				// Free the SMP link record here so cleanup does not depend on
				// the application callback: without it the four SMP slots leak
				// as connection handles change, and stale pairing/lockout state
				// and crypto-engine ownership survive the disconnect. Idempotent
				// - a port whose callback already called it sees a no-op.
				BtSmpDisconnected(p->ConnHdl);

				BtHciReasmReset(p->ConnHdl);
			}
			break;
		case BT_HCI_EVT_AUTHEN_COMPLETE:
			break;
		case BT_HCI_EVT_REMOTE_NAME_RQST_COMPLETE:
			break;
		case BT_HCI_EVT_ENCRYPTION_CHANGE_V1:
		case BT_HCI_EVT_ENCRYPTION_CHANGE_V2:
		case 0x5a:	// Encryption Change [v2] as emitted by the SDC controller
					// (payload: status, conn_handle, enabled[, key_size]).
			{
				if (pEvtPkt->Hdr.Len < 4)
				{
					break;
				}
				uint8_t  status  = pEvtPkt->Data[0];
				uint16_t connHdl = pEvtPkt->Data[1] | (pEvtPkt->Data[2] << 8);
				uint8_t  enabled = pEvtPkt->Data[3];
				BtSmpEncryptionChanged(pDev, connHdl, status, enabled);
			}
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
			{
				// NbCmdPacket(1) + CmdCode(2) are the minimum Command Complete
				// payload; a shorter event is malformed and must not be parsed.
				if (pEvtPkt->Hdr.Len < 3)
				{
					break;
				}

				BtHciEvtCmdComplete_t *p = (BtHciEvtCmdComplete_t*)pEvtPkt->Data;

				// Num_HCI_Command_Packets is the absolute command allowance per
				// spec, so set rather than add.
				pDev->CmdCredit = p->NbCmdPacket;

				// Match a blocking command waiting on this opcode. RetParam[0] is
				// the status; the command return parameters follow it.
				if (pDev->CmdOpCode != 0 && p->CmdCode == pDev->CmdOpCode)
				{
					// The status byte is present only when Hdr.Len >= 4; a
					// Command Complete with no return parameters still unblocks
					// the waiter but has no status to copy.
					if (pEvtPkt->Hdr.Len >= 4)
					{
						pDev->CmdStatus = p->RetParam[0];

						int retlen = (int)pEvtPkt->Hdr.Len - 4;		// less NbCmdPacket, CmdCode, status
						if (retlen > 0 && pDev->pCmdRet != nullptr)
						{
							int l = retlen < (int)pDev->CmdRetLen ? retlen : (int)pDev->CmdRetLen;
							memcpy(pDev->pCmdRet, &p->RetParam[1], l);
						}
					}

					pDev->CmdDone = true;
				}
			}
			break;
		case BT_HCI_EVT_COMMAND_STATUS:
			{
				// Status(1) + NbCmdPacket(1) + CmdCode(2). A shorter event
				// would leave these fields reading bytes the packet does not
				// contain.
				if (pEvtPkt->Hdr.Len < sizeof(BtHciEvtCmdStatus_t))
				{
					break;
				}

				BtHciEvtCmdStatus_t *p = (BtHciEvtCmdStatus_t*)pEvtPkt->Data;

				pDev->CmdCredit = p->NbCmdPacket;

				if (pDev->CmdOpCode != 0 && p->CmdCode == pDev->CmdOpCode)
				{
					pDev->CmdStatus = p->Status;
					pDev->CmdDone = true;
				}
			}
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
				// The count byte has to be inside the event before it can be
				// read. The clamp below bounded the walk but the read itself
				// came first, which is an out-of-bounds read on a zero-length
				// event however the walk then behaves.
				if (pEvtPkt->Hdr.Len < 1)
				{
					break;
				}

				// NbHdl is controller-supplied; bound the walk to the entries the
				// event length can actually hold (1 count byte + 4 bytes each) so
				// a malformed event cannot read past the received payload.
				int nbHdl = p->NbHdl;
				int maxHdl = (pEvtPkt->Hdr.Len - 1) / (int)sizeof(p->Completed[0]);
				if (nbHdl > maxHdl)
				{
					nbHdl = maxHdl;
				}
				for (int i = 0; i < nbHdl; i++)
				{
					// Replenish ACL TX credits for completed packets when flow
					// control is configured. Independent of SendCompleted so
					// credits recover even with no completion callback wired.
					if (pDev->AclCreditMax > 0)
					{
						pDev->AclCredit += p->Completed[i].NbPkt;
						if (pDev->AclCredit > pDev->AclCreditMax)
						{
							pDev->AclCredit = pDev->AclCreditMax;
						}
					}

					if (pDev->SendCompleted)
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
			// Hdr.Len is the event parameter length = bytes of the LE meta
			// event payload (Evt + Data); pass it so report walks stay bounded.
			// At least the subevent code byte must be present before the
			// dispatch below reads it.
			if (pEvtPkt->Hdr.Len < 1)
			{
				break;
			}
			BtHciProcessLeEvent(pDev, (BtHciLeEvtPacket_t *)pEvtPkt->Data, pEvtPkt->Hdr.Len);
			break;
	}
//	DEBUG_PRINTF("+++++\r\n");
}


// Packets this link sent that carry no GATT completion. The GATT layer keeps
// the per-link ring that attributes controller completions to notifications,
// and an ATT response or L2CAP signaling packet consumes a completion without
// owning a callback. Telling it here keeps the two in step; without this an
// ATT response completing would fire the next notification's callback early.
//
// Weak default for builds that do not link bt_gatt.cpp, which the host test
// harnesses do. bt_gatt.cpp defines the real one and overrides this.
__attribute__((weak)) void BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt)
{
	(void)ConnHdl;
	(void)NbPkt;
}

// HCI ACL packets a PDU of AclLen octets goes out as.
static uint16_t BtHciAclPktCount(BtHciDevice_t * const pDev, uint16_t AclLen)
{
	if (pDev == nullptr || pDev->AclMaxLen == 0 || AclLen == 0)
	{
		return 1;
	}

	return (uint16_t)((AclLen + pDev->AclMaxLen - 1) / pDev->AclMaxLen);
}

void BtHciProcessData(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pPkt)
{
	uint16_t connHdl = pPkt->Hdr.ConnHdl;
	BtL2CapPdu_t *l2rcv;
	uint16_t avail;

	// Reassemble fragmented ACL data. The controller sends the first fragment
	// of an L2CAP PDU with a START packet-boundary flag and the remainder with
	// the CONTINUING flag. A continuation does not carry an L2CAP header, so it
	// is only meaningful appended to the connection's pending start fragment.
	if (pPkt->Hdr.PBFlag == BT_HCI_PBFLAG_CONTINUING_FRAGMENT)
	{
		BtHciReasm_t *ctx = BtHciReasmFind(connHdl);

		if (ctx == nullptr)
		{
			// Continuation with no pending start on this connection - drop.
			return;
		}

		uint32_t next = (uint32_t)ctx->Received + pPkt->Hdr.Len;
		if (next > ctx->Expected || next > BT_HCI_BUFFER_MAX_SIZE)
		{
			// Would overrun the declared L2CAP PDU or the reassembly buffer -
			// abort this PDU. A complete reassembled PDU must end exactly at
			// Expected, not carry trailing bytes from a malformed fragment.
			ctx->Active = false;
			return;
		}

		memcpy(&ctx->Buf[ctx->Received], pPkt->Data, pPkt->Hdr.Len);
		ctx->Received = (uint16_t)next;

		if (ctx->Received < ctx->Expected)
		{
			// More fragments still to come.
			return;
		}

		ctx->Active = false;
		l2rcv = (BtL2CapPdu_t*)ctx->Buf;
		avail = ctx->Received;
	}
	else
	{
		// Start of a new L2CAP PDU (PB flag START or COMPLETE).
		if (pPkt->Hdr.Len < sizeof(BtL2CapHdr_t))
		{
			return;
		}

		BtL2CapPdu_t *frag = (BtL2CapPdu_t*)pPkt->Data;
		uint32_t expected = (uint32_t)frag->Hdr.Len + sizeof(BtL2CapHdr_t);

		// A new start supersedes any half-built PDU on this connection.
		BtHciReasm_t *stale = BtHciReasmFind(connHdl);
		if (stale != nullptr)
		{
			stale->Active = false;
		}

		if (pPkt->Hdr.Len >= expected)
		{
			// Whole PDU delivered in a single fragment.
			l2rcv = frag;
			avail = pPkt->Hdr.Len;
		}
		else
		{
			// Partial first fragment - buffer it and wait for continuations.
			if (expected > BT_HCI_BUFFER_MAX_SIZE)
			{
				// Larger than this host can reassemble.
				return;
			}

			BtHciReasm_t *ctx = BtHciReasmAlloc(connHdl);
			if (ctx == nullptr)
			{
				// No free reassembly slot.
				return;
			}

			memcpy(ctx->Buf, pPkt->Data, pPkt->Hdr.Len);
			ctx->ConnHdl = connHdl;
			ctx->Expected = (uint16_t)expected;
			ctx->Received = pPkt->Hdr.Len;
			ctx->Active = true;

			return;
		}
	}

	// l2rcv now points at a complete L2CAP PDU holding 'avail' bytes. Validate
	// the L2CAP length does not claim more than was received before the ATT/SMP
	// parsers read it.
	if (avail < sizeof(BtL2CapHdr_t) ||
		(uint32_t)l2rcv->Hdr.Len + sizeof(BtL2CapHdr_t) > avail)
	{
		return;
	}

	{
		const uint8_t *raw = (const uint8_t*)l2rcv;
		uint8_t d[6] = { 0, 0, 0, 0, 0, 0 };

		for (uint16_t i = 0; i < sizeof(d) && (uint16_t)(sizeof(BtL2CapHdr_t) + i) < avail; i++)
		{
			d[i] = raw[sizeof(BtL2CapHdr_t) + i];
		}

		DEBUG_PRINTF("L2 RX cid=0x%04x len=%d d=%02x%02x%02x%02x%02x%02x\r\n",
				 l2rcv->Hdr.Cid, l2rcv->Hdr.Len,
				 d[0], d[1], d[2], d[3], d[4], d[5]);
	}
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
			// LE-U host->controller: the first (here only) fragment of a
			// complete L2CAP PDU uses 0b00 (first non-automatically-flushable).
			// 0b11 is a controller->host value and is rejected by strict
			// controllers (Vol 4 Part E 5.4.2).
			acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
			acl->Hdr.BCFlag = 0;

			l2pdu->Hdr = l2rcv->Hdr;

			l2pdu->Hdr.Len = BtAttProcessReq(pPkt->Hdr.ConnHdl, &l2rcv->Att, l2rcv->Hdr.Len, &l2pdu->Att);

			if (l2pdu->Hdr.Len > 0)
			{
				acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

				DEBUG_PRINTF("ATT TX rsp=0x%02x (req=0x%02x) len=%d\r\n",
					l2pdu->Att.OpCode, l2rcv->Att.OpCode, l2pdu->Hdr.Len);

				uint16_t nbpkt = BtHciAclPktCount(pDev, acl->Hdr.Len);
				uint32_t sent = BtHciSendAcl(pDev, acl);
				if (sent == 0)
				{
					SysLogPrintf(SysLogGet(),
						"ATT TX FAILED opcode=0x%02x rsp=0x%02x len=%u\r\n",
						l2rcv->Att.OpCode, l2pdu->Att.OpCode, acl->Hdr.Len);
				}
				else
				{
					BtGattTxPendUntracked(pPkt->Hdr.ConnHdl, nbpkt);
				}
			}
			else
			{
				DEBUG_PRINTF("ATT no rsp for req=0x%02x (treated as client rsp)\r\n",
					l2rcv->Att.OpCode);
				BtAttProcessRsp(pPkt->Hdr.ConnHdl, &l2rcv->Att, l2rcv->Hdr.Len);
			}
		}
//			BtProcessAttData(pDev, pPkt->Hdr.ConnHdl, l2frame);
			break;
		case BT_L2CAP_CID_SIGNAL:
		{
			uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
			BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
			BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

			acl->Hdr.ConnHdl = pPkt->Hdr.ConnHdl;
			// LE-U host->controller: the first (here only) fragment of a
			// complete L2CAP PDU uses 0b00 (first non-automatically-flushable).
			// 0b11 is a controller->host value and is rejected by strict
			// controllers (Vol 4 Part E 5.4.2).
			acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
			acl->Hdr.BCFlag = 0;

			l2pdu->Hdr.Len = BtL2CapProcessSignal(pDev, pPkt->Hdr.ConnHdl,
											  l2rcv, l2rcv->Hdr.Len, l2pdu);

			if (l2pdu->Hdr.Len > 0)
			{
				l2pdu->Hdr.Cid = BT_L2CAP_CID_SIGNAL;
				acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

				uint16_t nbpkt = BtHciAclPktCount(pDev, acl->Hdr.Len);
				uint32_t sent = BtHciSendAcl(pDev, acl);
				if (sent == 0)
				{
					SysLogPrintf(SysLogGet(),
						"L2CAP SIG TX FAILED code=0x%02x len=%u\r\n",
						l2rcv->CFrame.Code, acl->Hdr.Len);
				}
				else
				{
					BtGattTxPendUntracked(pPkt->Hdr.ConnHdl, nbpkt);
				}
			}
		}
			break;
		case BT_L2CAP_CID_SEC_MNGR:
			BtProcessSmpData(pDev, pPkt->Hdr.ConnHdl, &l2rcv->Smp, l2rcv->Hdr.Len);
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
	// LE-U host->controller: 0b00 for the first (only) fragment; 0b11 is a
	// controller->host value (Vol 4 Part E 5.4.2).
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
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
	BtHciSendAcl(pDev, acl);
	s_NbPktSent++;
//	DEBUG_PRINTF("... \r\n");
}
*/

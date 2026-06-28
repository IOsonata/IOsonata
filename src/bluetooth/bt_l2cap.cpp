/**-------------------------------------------------------------------------
@file	bt_l2cap.cpp

@brief	Generic Bluetooth LE L2CAP signaling implementation


@author	Hoang Nguyen Hoan
@date	Jun. 28, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdint.h>
#include <string.h>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "syslog.h"

#ifndef BT_L2CAP_LE_SIG_RSP_MAX
#define BT_L2CAP_LE_SIG_RSP_MAX \
	(BT_HCI_BUFFER_MAX_SIZE - sizeof(BtHciACLDataPacketHdr_t) - sizeof(BtL2CapHdr_t))
#endif

static bool BtL2CapAppendFrame(uint8_t *pOut, uint16_t *pOutLen, uint16_t OutMax,
							   uint8_t Code, uint8_t Id,
							   const void *pPayload, uint16_t PayloadLen)
{
	uint16_t need = (uint16_t)(sizeof(BtL2CapCFrame_t) - 1 + PayloadLen);

	if (pOut == nullptr || pOutLen == nullptr || (*pOutLen + need) > OutMax)
	{
		return false;
	}

	BtL2CapCFrame_t *pFrame = (BtL2CapCFrame_t *)(pOut + *pOutLen);

	pFrame->Code = Code;
	pFrame->Id   = Id;
	pFrame->Len  = PayloadLen;

	if (PayloadLen > 0 && pPayload != nullptr)
	{
		memcpy(pFrame->Data, pPayload, PayloadLen);
	}

	*pOutLen += need;

	return true;
}

static bool BtL2CapAppendCmdReject(uint8_t *pOut, uint16_t *pOutLen, uint16_t OutMax,
								   uint8_t Id, uint16_t Reason,
								   const void *pData, uint16_t DataLen)
{
	uint8_t payload[6];

	if (DataLen > sizeof(payload) - sizeof(uint16_t))
	{
		DataLen = sizeof(payload) - sizeof(uint16_t);
	}

	payload[0] = (uint8_t)(Reason & 0xFF);
	payload[1] = (uint8_t)(Reason >> 8);

	if (DataLen > 0 && pData != nullptr)
	{
		memcpy(&payload[2], pData, DataLen);
	}

	return BtL2CapAppendFrame(pOut, pOutLen, OutMax,
							  BT_L2CAP_CODE_COMMAND_REJECT_RSP,
							  Id, payload, (uint16_t)(sizeof(uint16_t) + DataLen));
}

static bool BtL2CapConnParamValid(const BtL2CapConnParamUpdateReq_t *pReq)
{
	if (pReq == nullptr)
	{
		return false;
	}

	if (pReq->IntervalMin < 6 || pReq->IntervalMax > 0x0C80 ||
		pReq->IntervalMin > pReq->IntervalMax)
	{
		return false;
	}

	if (pReq->Latency > 0x01F3)
	{
		return false;
	}

	if (pReq->Timeout < 0x000A || pReq->Timeout > 0x0C80)
	{
		return false;
	}

	// Supervision timeout must be greater than:
	// (1 + latency) * max_interval * 2.
	// Use protocol units without floating point:
	// Timeout is 10 ms units, interval is 1.25 ms units.
	uint32_t left  = (uint32_t)pReq->Timeout * 40UL;
	uint32_t right = ((uint32_t)pReq->Latency + 1UL) *
					 (uint32_t)pReq->IntervalMax * 10UL;

	return left > right;
}

__attribute__((weak)) bool BtL2CapConnParamUpdateAccept(uint16_t ConnHdl,
														const BtL2CapConnParamUpdateReq_t *pReq)
{
	(void)ConnHdl;
	(void)pReq;

	// The generic host has no controller-agnostic LL connection-update command
	// path here. Reject by default so peers get a deterministic response.
	// A central-role port can override this hook, start the controller update,
	// and return true.
	return false;
}

__attribute__((weak)) void BtL2CapConnParamUpdateRsp(uint16_t ConnHdl, uint8_t Id,
													 uint16_t Result)
{
	(void)ConnHdl;
	(void)Id;
	(void)Result;
}

__attribute__((weak)) uint16_t BtL2CapLeCreditBasedConnectionReq(uint16_t ConnHdl,
																 const BtL2CapLeCreditBasedConnReq_t *pReq,
																 BtL2CapLeCreditBasedConnRsp_t *pRsp)
{
	(void)ConnHdl;
	(void)pReq;

	if (pRsp != nullptr)
	{
		pRsp->Dcid = 0;
		pRsp->Mtu = 0;
		pRsp->Mps = 0;
		pRsp->InitialCredits = 0;
		pRsp->Result = BT_L2CAP_LE_CBFC_RESULT_SPSM_NOT_SUPPORTED;
	}

	return BT_L2CAP_LE_CBFC_RESULT_SPSM_NOT_SUPPORTED;
}

__attribute__((weak)) bool BtL2CapFlowControlCreditInd(uint16_t ConnHdl,
													   uint16_t Cid,
													   uint16_t Credits)
{
	(void)ConnHdl;
	(void)Cid;
	(void)Credits;

	return false;
}

__attribute__((weak)) bool BtL2CapDisconnectReq(uint16_t ConnHdl,
												uint16_t Dcid,
												uint16_t Scid)
{
	(void)ConnHdl;
	(void)Dcid;
	(void)Scid;

	return false;
}

__attribute__((weak)) void BtL2CapDisconnectRsp(uint16_t ConnHdl,
												uint16_t Dcid,
												uint16_t Scid)
{
	(void)ConnHdl;
	(void)Dcid;
	(void)Scid;
}

__attribute__((weak)) uint16_t BtL2CapCreditBasedReconfigureReq(uint16_t ConnHdl,
																const BtL2CapCreditBasedReconfReq_t *pReq,
																uint16_t Len)
{
	(void)ConnHdl;
	(void)pReq;
	(void)Len;

	return BT_L2CAP_RECONFIG_RESULT_INVALID_DCID;
}

__attribute__((weak)) void BtL2CapCreditBasedReconfigureRsp(uint16_t ConnHdl,
															uint16_t Result)
{
	(void)ConnHdl;
	(void)Result;
}

uint32_t BtL2CapProcessSignal(BtHciDevice_t * const pDev,
							  uint16_t ConnHdl,
							  BtL2CapPdu_t const * const pReqPdu,
							  uint16_t ReqLen,
							  BtL2CapPdu_t * const pRspPdu)
{
	(void)pDev;

	if (pReqPdu == nullptr || pRspPdu == nullptr)
	{
		return 0;
	}

	if (ReqLen < (sizeof(BtL2CapCFrame_t) - 1))
	{
		return 0;
	}

	const uint8_t *p = (const uint8_t *)&pReqPdu->CFrame;
	uint16_t remain = ReqLen;
	uint8_t *pOut = (uint8_t *)&pRspPdu->CFrame;
	uint16_t outLen = 0;
	uint16_t outMax = BT_L2CAP_LE_SIG_RSP_MAX;

	pRspPdu->Hdr.Cid = BT_L2CAP_CID_SIGNAL;

	while (remain >= (sizeof(BtL2CapCFrame_t) - 1))
	{
		BtL2CapCFrame_t const *pCmd = (BtL2CapCFrame_t const *)p;
		uint16_t cmdHdrLen = sizeof(BtL2CapCFrame_t) - 1;
		uint16_t cmdLen = pCmd->Len;

		if (cmdLen > (remain - cmdHdrLen))
		{
			BtL2CapAppendCmdReject(pOut, &outLen, outMax, pCmd->Id,
									BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD,
									nullptr, 0);
			break;
		}

		uint8_t const *data = pCmd->Data;

		switch (pCmd->Code)
		{
			case BT_L2CAP_CODE_COMMAND_REJECT_RSP:
				break;

			case BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_REQ:
			{
				BtL2CapConnParamUpdateRsp_t rsp;
				rsp.Result = BT_L2CAP_CONN_PARAM_REJECTED;

				if (cmdLen == sizeof(BtL2CapConnParamUpdateReq_t))
				{
					BtL2CapConnParamUpdateReq_t const *req =
						(BtL2CapConnParamUpdateReq_t const *)data;

					if (BtL2CapConnParamValid(req) &&
						BtL2CapConnParamUpdateAccept(ConnHdl, req))
					{
						rsp.Result = BT_L2CAP_CONN_PARAM_ACCEPTED;
					}
				}

				BtL2CapAppendFrame(pOut, &outLen, outMax,
									BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_RSP,
									pCmd->Id, &rsp, sizeof(rsp));
				break;
			}

			case BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_RSP:
				if (cmdLen == sizeof(BtL2CapConnParamUpdateRsp_t))
				{
					BtL2CapConnParamUpdateRsp_t const *rsp =
						(BtL2CapConnParamUpdateRsp_t const *)data;
					BtL2CapConnParamUpdateRsp(ConnHdl, pCmd->Id, rsp->Result);
				}
				break;

			case BT_L2CAP_CODE_LE_CREDIT_BASED_CONNECTION_REQ:
			{
				BtL2CapLeCreditBasedConnRsp_t rsp;

				memset(&rsp, 0, sizeof(rsp));
				rsp.Result = BT_L2CAP_LE_CBFC_RESULT_INVALID_PARAMETERS;

				if (cmdLen == sizeof(BtL2CapLeCreditBasedConnReq_t))
				{
					BtL2CapLeCreditBasedConnReq_t const *req =
						(BtL2CapLeCreditBasedConnReq_t const *)data;

					BtL2CapLeCreditBasedConnectionReq(ConnHdl, req, &rsp);
				}

				BtL2CapAppendFrame(pOut, &outLen, outMax,
									BT_L2CAP_CODE_LE_CREDIT_BASED_CONNECTION_RSP,
									pCmd->Id, &rsp, sizeof(rsp));
				break;
			}

			case BT_L2CAP_CODE_LE_CREDIT_BASED_CONNECTION_RSP:
				break;

			case BT_L2CAP_CODE_FLOW_CONTROL_CREDIT_IND:
			{
				if (cmdLen != sizeof(BtL2CapFlowControlCreditInd_t))
				{
					BtL2CapAppendCmdReject(pOut, &outLen, outMax, pCmd->Id,
											BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD,
											nullptr, 0);
					break;
				}

				BtL2CapFlowControlCreditInd_t const *ind =
					(BtL2CapFlowControlCreditInd_t const *)data;

				if (BtL2CapFlowControlCreditInd(ConnHdl, ind->Cid, ind->Credits) == false)
				{
					BtL2CapInvalidCidRejData_t rej = { ind->Cid, 0 };
					BtL2CapAppendCmdReject(pOut, &outLen, outMax, pCmd->Id,
											BT_L2CAP_CMD_REJECT_REASON_INVALID_CID,
											&rej, sizeof(rej));
				}
				break;
			}

			case BT_L2CAP_CODE_DISCONNECTION_REQ:
			{
				if (cmdLen != sizeof(BtL2CapDisconnReq_t))
				{
					BtL2CapAppendCmdReject(pOut, &outLen, outMax, pCmd->Id,
											BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD,
											nullptr, 0);
					break;
				}

				BtL2CapDisconnReq_t const *req = (BtL2CapDisconnReq_t const *)data;

				if (BtL2CapDisconnectReq(ConnHdl, req->Dcid, req->Scid))
				{
					BtL2CapDisconnRsp_t rsp = { req->Dcid, req->Scid };
					BtL2CapAppendFrame(pOut, &outLen, outMax,
										BT_L2CAP_CODE_DISCONNECTION_RSP,
										pCmd->Id, &rsp, sizeof(rsp));
				}
				else
				{
					BtL2CapInvalidCidRejData_t rej = { req->Dcid, req->Scid };
					BtL2CapAppendCmdReject(pOut, &outLen, outMax, pCmd->Id,
											BT_L2CAP_CMD_REJECT_REASON_INVALID_CID,
											&rej, sizeof(rej));
				}
				break;
			}

			case BT_L2CAP_CODE_DISCONNECTION_RSP:
				if (cmdLen == sizeof(BtL2CapDisconnRsp_t))
				{
					BtL2CapDisconnRsp_t const *rsp = (BtL2CapDisconnRsp_t const *)data;
					BtL2CapDisconnectRsp(ConnHdl, rsp->Dcid, rsp->Scid);
				}
				break;

			case BT_L2CAP_CODE_CREDIT_BASED_CONNECTION_REQ:
			case BT_L2CAP_CODE_CREDIT_BASED_CONNECTION_RSP:
				if (pCmd->Code == BT_L2CAP_CODE_CREDIT_BASED_CONNECTION_REQ)
				{
					BtL2CapAppendCmdReject(pOut, &outLen, outMax, pCmd->Id,
											BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD,
											nullptr, 0);
				}
				break;

			case BT_L2CAP_CODE_CREDIT_BASED_RECONFIGURE_REQ:
			{
				BtL2CapCreditBasedReconfRsp_t rsp;
				rsp.Result = BT_L2CAP_RECONFIG_RESULT_INVALID_DCID;

				// MTU (2) + MPS (2) + at least one Destination CID (2).
				if (cmdLen >= 6)
				{
					rsp.Result = BtL2CapCreditBasedReconfigureReq(
						ConnHdl, (BtL2CapCreditBasedReconfReq_t const *)data, cmdLen);
				}

				BtL2CapAppendFrame(pOut, &outLen, outMax,
									BT_L2CAP_CODE_CREDIT_BASED_RECONFIGURE_RSP,
									pCmd->Id, &rsp, sizeof(rsp));
				break;
			}

			case BT_L2CAP_CODE_CREDIT_BASED_RECONFIGURE_RSP:
				if (cmdLen == sizeof(BtL2CapCreditBasedReconfRsp_t))
				{
					BtL2CapCreditBasedReconfRsp_t const *rsp =
						(BtL2CapCreditBasedReconfRsp_t const *)data;
					BtL2CapCreditBasedReconfigureRsp(ConnHdl, rsp->Result);
				}
				break;

			default:
				BtL2CapAppendCmdReject(pOut, &outLen, outMax, pCmd->Id,
										BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD,
										nullptr, 0);
				break;
		}

		p += cmdHdrLen + cmdLen;
		remain -= cmdHdrLen + cmdLen;
	}

	pRspPdu->Hdr.Len = outLen;

	return outLen;
}

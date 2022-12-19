/**-------------------------------------------------------------------------
@file	bt_ctlr_sdc.cpp

@brief	Generic implementation of Bluetooth controller device.

Implementation of Bluetooth controller using Nordic Softdevice Controller

@author	Hoang Nguyen Hoan
@date	Nov. 30, 2022

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
#include <memory.h>

#include "sdc_hci.h"

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_ctlr.h"

#define BTCTLR_FIFO_MEM_SIZE			BTCTLR_PKT_CFIFO_TOTAL_MEMSIZE(4, BT_CTLR_MTU_MAX)
alignas(4) static uint8_t s_BtCtlrRxFifoMem[BTCTLR_FIFO_MEM_SIZE];

static inline size_t BtCtlrSendData(BtCtlrDev_t * const pDev, void *pData, size_t Len) {
	return sdc_hci_data_put((uint8_t*)pData) == 0 ? Len : 0;
}

size_t BtCtlrReceive(BtCtlrDev_t * const pDev, uint16_t Hdl, void * const pData, size_t Len)
{
	BtCtlrPkt_t *p = (BtCtlrPkt_t*)CFifoPut(pDev->hRxFifo);
	if (p)
	{
		p->Len = Len;
		p->ValHdl = Hdl;
		p->Off = 0;

		size_t l = min(Len, pDev->PacketSize);
		memcpy(p->Data, pData, l);

		return l;
	}

	return 0;
}

static void SdcCtlrDisable(DevIntrf_t * const pDev)
{
}

static void SdcCtlrEnable(DevIntrf_t * const pDev)
{
}

static uint32_t SdcCtlrGetRate(DevIntrf_t * const pDev)
{
	uint32_t rate = 0;

	return rate;
}

static uint32_t SdcCtlrSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	return 0;
}

static bool SdcCtlrStartRx(DevIntrf_t * const pSerDev, uint32_t DevAddr)
{
	return true;
}

static int SdcCtlrRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	return 0;
}

static void SdcCtlrStopRx(DevIntrf_t * const pDev)
{
}

static bool SdcCtlrStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	BtCtlrDev_t *p = (BtCtlrDev_t *)pDev->pDevData;

	p->ConnHdl = DevAddr >> 16;
	p->ValHdl = DevAddr & 0xFFFF;

	return true;
}

static int SdcCtlrTxData(DevIntrf_t * const pDev, uint8_t *pData, int DataLen)
{
	BtCtlrDev_t *p = (BtCtlrDev_t *)pDev->pDevData;
	uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

//	g_Uart.printf("BtHciMotify : %d %d \r\n", ConnHdl, ValHdl);

	acl->Hdr.ConnHdl = p->ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF;

	uint8_t *param = l2pdu->Att.Param;

	*(uint16_t*)param = p->ValHdl;
	param += 2;
	memcpy(param, pData, DataLen);
	l2pdu->Hdr.Len = 2 + DataLen;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return sdc_hci_data_put((uint8_t*)pData) == 0 ? acl->Hdr.Len + sizeof(acl->Hdr) : 0;

//	g_Uart.printf("Len : %d, %d, %d\r\n", Len, l2pdu->Hdr.Len, acl->Hdr.Len);
	//uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
}

static void SdcCtlrStopTx(DevIntrf_t * const pDev)
{
}

void SdcCtlrPowerOff(DevIntrf_t * const pDev)
{
}

bool BtCtlrInit(BtCtlrDev_t * const pDev, const BtCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->PacketSize <= 0)
	{
		pDev->PacketSize = BT_CTLR_MTU_MAX + BTCTLR_PKTHDR_LEN;
	}
	else
	{
		pDev->PacketSize = pCfg->PacketSize + BTCTLR_PKTHDR_LEN;
	}

	if (pCfg->pRxFifoMem == NULL)// || pCfg->pTxFifoMem == NULL)
	{
		pDev->hRxFifo = CFifoInit(s_BtCtlrRxFifoMem, BTCTLR_FIFO_MEM_SIZE, pDev->PacketSize, true);
		//pDev->hTxFifo = CFifoInit(s_BtDevIntrfRxFifoMem, BTINTRF_CFIFO_SIZE, pIntrf->PacketSize, pCfg->bBlocking);
	}
	else
	{
		//pDev->hRxFifo = CFifoInit(pCfg->pRxFifoMem, pCfg->RxFifoMemSize, pIntrf->PacketSize, pCfg->bBlocking);
		//pDev->hTxFifo = CFifoInit(pCfg->pTxFifoMem, pCfg->TxFifoMemSize, pIntrf->PacketSize, pCfg->bBlocking);
	}

	pDev->DevIntrf.Type = DEVINTRF_TYPE_BT;
	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.Disable = SdcCtlrDisable;
	pDev->DevIntrf.Enable = SdcCtlrEnable;
	pDev->DevIntrf.GetRate = SdcCtlrGetRate;
	pDev->DevIntrf.SetRate = SdcCtlrSetRate;
	pDev->DevIntrf.StartRx = SdcCtlrStartRx;
	pDev->DevIntrf.RxData = SdcCtlrRxData;
	pDev->DevIntrf.StopRx = SdcCtlrStopRx;
	pDev->DevIntrf.StartTx = SdcCtlrStartTx;
	pDev->DevIntrf.TxData = SdcCtlrTxData;
	pDev->DevIntrf.StopTx = SdcCtlrStopTx;
	pDev->DevIntrf.MaxRetry = 1;
	pDev->DevIntrf.PowerOff = SdcCtlrPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.EvtCB = pCfg->EvtHandler;
	pDev->Receive = BtCtlrReceive;
	pDev->Send = BtCtlrSendData;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

void BtCtlrProcessData(BtCtlrDev_t * const pDev, BtHciACLDataPacket_t * const pPkt)
{
	BtL2CapPdu_t *l2frame = (BtL2CapPdu_t*)pPkt->Data;

	switch (l2frame->Hdr.Cid)
	{
		case BT_L2CAP_CID_ACL_U:
			break;
		case BT_L2CAP_CID_CONNECTIONLESS:
			break;
		case BT_L2CAP_CID_ATT:
			BtCtlrProcessAttData(pDev, pPkt->Hdr.ConnHdl, l2frame);
			break;
		case BT_L2CAP_CID_SIGNAL:
			break;
		case BT_L2CAP_CID_SEC_MNGR:
			BtCtlrProcessSmpData(pDev, l2frame);
			break;
	}
}

void BtCtlrHandler(BtCtlrDev_t * const pDev)
{
	uint8_t buf[HCI_MSG_BUFFER_MAX_SIZE];
	int32_t res = 0;
	sdc_hci_msg_type_t mtype;
	res = sdc_hci_get(buf, &mtype);
	if (res == 0)
	{
		switch (mtype)
		{
			case SDC_HCI_MSG_TYPE_EVT:
				// Event available
				BtCtlrProcessEvent(pDev, (BtHciEvtPacket_t*)buf);
				break;
			case SDC_HCI_MSG_TYPE_DATA:
				BtCtlrProcessData(pDev, (BtHciACLDataPacket_t*)buf);
				break;
		}
	}
}


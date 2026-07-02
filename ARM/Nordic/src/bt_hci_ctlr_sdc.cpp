/**-------------------------------------------------------------------------
@file	bt_hci_ctlr_sdc.cpp

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
#include "bluetooth/bt_hci_ctlr.h"

#define BTHCICTLR_FIFO_MEM_SIZE			BTHCICTLR_PKT_CFIFO_TOTAL_MEMSIZE(4, BT_HCI_CTLR_MTU_MAX)
alignas(4) static uint8_t s_BtHciCtlrRxFifoMem[BTHCICTLR_FIFO_MEM_SIZE];

static inline size_t BtHciCtlrSendData(BtHciCtlrDev_t * const pDev, void *pData, size_t Len) {
	return sdc_hci_data_put((uint8_t*)pData) == 0 ? Len : 0;
}

size_t BtHciCtlrReceive(BtHciCtlrDev_t * const pDev, uint16_t Hdl, void * const pData, size_t Len)
{
	BtHciCtlrPkt_t *p = (BtHciCtlrPkt_t*)CFifoPut(pDev->hRxFifo);
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
	BtHciCtlrDev_t *p = (BtHciCtlrDev_t *)pDev->pDevData;

	p->ConnHdl = DevAddr >> 16;
	p->ValHdl = DevAddr & 0xFFFF;

	return true;
}

static int SdcCtlrTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	// Forward a formed HCI ACL packet to the SDC controller. The host assembles
	// the packet above the HCI line; the controller only transmits it.
	return sdc_hci_data_put((uint8_t*)pData) == 0 ? DataLen : 0;
}

static void SdcCtlrStopTx(DevIntrf_t * const pDev)
{
}

void SdcCtlrPowerOff(DevIntrf_t * const pDev)
{
}

bool BtHciCtlrInit(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->PacketSize <= 0)
	{
		pDev->PacketSize = BT_HCI_CTLR_MTU_MAX + BTHCICTLR_PKTHDR_LEN;
	}
	else
	{
		pDev->PacketSize = pCfg->PacketSize + BTHCICTLR_PKTHDR_LEN;
	}

	if (pCfg->pRxFifoMem == NULL)// || pCfg->pTxFifoMem == NULL)
	{
		pDev->hRxFifo = CFifoInit(s_BtHciCtlrRxFifoMem, BTHCICTLR_FIFO_MEM_SIZE, pDev->PacketSize, true);
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
	pDev->Receive = BtHciCtlrReceive;
	pDev->Send = BtHciCtlrSendData;
	pDev->RxHandler = pCfg->RxHandler;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

void BtHciCtlrProcess(BtHciCtlrDev_t * const pDev)
{
	if (pDev == nullptr || pDev->RxHandler == nullptr)
	{
		return;
	}

	uint8_t buf[HCI_MSG_BUFFER_MAX_SIZE];
	sdc_hci_msg_type_t mtype;

	// Drain every queued message. The controller can queue several at once,
	// for example a command completion followed by an Encryption Change event
	// during pairing; stopping after one strands the later packets.
	while (sdc_hci_get(buf, (uint8_t*)&mtype) == 0)
	{
		pDev->RxHandler(pDev, mtype == SDC_HCI_MSG_TYPE_EVT, buf);
	}
}

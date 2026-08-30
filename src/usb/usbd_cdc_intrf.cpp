/**-------------------------------------------------------------------------
@file	usbd_cdc_intrf.cpp

@brief	IOsonata USB CDC DeviceIntrf adapter.

Routes the data plane through UsbdBulkIntrf and the CDC ACM protocol through
UsbdCdcFunc.

@author	Hoang Nguyen Hoan
@date	Aug. 30, 2026

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
#include <string.h>

#include "usb/usb_dev.h"
#include "usb/usbd_cdc.h"
#include "usb/usbd_bulk_intrf.h"
#include "usbd_cdc_priv.h"

#define USBD_CDC_INTRF_MAXCNT		3
#define USBD_CDC_DEFAULT_FIFO_MEMSIZE \
	CFIFO_MEMSIZE(4U * USBD_CDC_BULK_FS_MPS)

typedef struct __Usbd_Cdc_State {
	UsbdBulkDevIntrf_t Bulk;
	UsbdCdcFunc_t Func;
	UsbdCdcDevIntrf_t *pPublic;
	DevIntrfEvtHandler_t AppEvt;
} UsbdCdcState_t;

alignas(4) static uint8_t s_RxFifoMem[USBD_CDC_INTRF_MAXCNT]
	[USBD_CDC_DEFAULT_FIFO_MEMSIZE];
alignas(4) static uint8_t s_TxFifoMem[USBD_CDC_INTRF_MAXCNT]
	[USBD_CDC_DEFAULT_FIFO_MEMSIZE];
static UsbdCdcState_t s_State[USBD_CDC_INTRF_MAXCNT];

static UsbdCdcState_t *UsbdCdcFromPublic(UsbdCdcDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->ItfNo < 0 ||
		pIntrf->ItfNo >= USBD_CDC_INTRF_MAXCNT)
	{
		return nullptr;
	}

	UsbdCdcState_t *pState = &s_State[pIntrf->ItfNo];
	return pState->pPublic == pIntrf ? pState : nullptr;
}

static UsbdCdcState_t *UsbdCdcFromDev(DevIntrf_t *pDevIntrf)
{
	return pDevIntrf != nullptr ?
		static_cast<UsbdCdcState_t *>(pDevIntrf->pDevData) : nullptr;
}

static UsbdCdcState_t *UsbdCdcFromBulk(DevIntrf_t *pDevIntrf)
{
	for (int i = 0; i < USBD_CDC_INTRF_MAXCNT; i++)
	{
		if (&s_State[i].Bulk.DevIntrf == pDevIntrf &&
			s_State[i].pPublic != nullptr)
		{
			return &s_State[i];
		}
	}

	return nullptr;
}

static int UsbdCdcEvent(DevIntrf_t * const pDevIntrf,
						 DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len)
{
	UsbdCdcState_t *pState = UsbdCdcFromBulk(pDevIntrf);
	if (pState == nullptr || pState->pPublic == nullptr)
	{
		return 0;
	}

	if (EvtId == DEVINTRF_EVT_STATECHG)
	{
		pState->pPublic->bPortOpen = Len != 0;
	}
	else if (EvtId == DEVINTRF_EVT_RX_FIFO_FULL)
	{
		pState->pPublic->RxDropCnt = pState->Bulk.hRxFifo->DropCnt;
	}

	return pState->AppEvt != nullptr ?
		pState->AppEvt(&pState->pPublic->DevIntrf, EvtId, pBuffer, Len) : 0;
}

static void UsbdCdcPump(void *pContext)
{
	UsbdCdcState_t *pState = static_cast<UsbdCdcState_t *>(pContext);
	if (pState == nullptr || pState->pPublic == nullptr)
	{
		return;
	}

	UsbdCdcFuncProcess(&pState->Func);
	pState->pPublic->RxDropCnt = pState->Bulk.hRxFifo->DropCnt;
	pState->pPublic->TxDropCnt = pState->Bulk.hTxFifo->DropCnt;
}

static void UsbdCdcDisable(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	if (pState != nullptr)
	{
		DeviceIntrfDisable(&pState->Bulk.DevIntrf);
		pState->pPublic->bEnabled = false;
	}
}

static void UsbdCdcEnable(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	if (pState != nullptr)
	{
		DeviceIntrfEnable(&pState->Bulk.DevIntrf);
		pState->pPublic->bEnabled = true;
	}
}

static uint32_t UsbdCdcGetRate(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	return pState != nullptr ? DeviceIntrfGetRate(&pState->Bulk.DevIntrf) : 0;
}

static uint32_t UsbdCdcSetRate(DevIntrf_t * const pDevIntrf, uint32_t Rate)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	return pState != nullptr ?
		DeviceIntrfSetRate(&pState->Bulk.DevIntrf, Rate) : 0;
}

static bool UsbdCdcStartRx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	return pState != nullptr &&
		pState->Bulk.DevIntrf.StartRx(&pState->Bulk.DevIntrf, DevAddr);
}

static int UsbdCdcRxData(DevIntrf_t * const pDevIntrf,
						 uint8_t *pBuffer, int BufferLen)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	return pState != nullptr ?
		pState->Bulk.DevIntrf.RxData(&pState->Bulk.DevIntrf,
								 pBuffer, BufferLen) : 0;
}

static void UsbdCdcStopRx(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	if (pState != nullptr)
	{
		pState->Bulk.DevIntrf.StopRx(&pState->Bulk.DevIntrf);
	}
}

static bool UsbdCdcStartTx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	return pState != nullptr &&
		pState->Bulk.DevIntrf.StartTx(&pState->Bulk.DevIntrf, DevAddr);
}

static int UsbdCdcTxData(DevIntrf_t * const pDevIntrf,
						 const uint8_t *pData, int DataLen)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	if (pState == nullptr)
	{
		return 0;
	}

	const int count = pState->Bulk.DevIntrf.TxData(&pState->Bulk.DevIntrf,
											 pData, DataLen);
	pState->pPublic->TxDropCnt = pState->Bulk.hTxFifo->DropCnt;
	return count;
}

static int UsbdCdcTxSrData(DevIntrf_t * const pDevIntrf,
						   const uint8_t *pData, int DataLen)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	if (pState == nullptr)
	{
		return 0;
	}

	const int count = pState->Bulk.DevIntrf.TxSrData(&pState->Bulk.DevIntrf,
											   pData, DataLen);
	pState->pPublic->TxDropCnt = pState->Bulk.hTxFifo->DropCnt;
	return count;
}

static void UsbdCdcStopTx(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	if (pState != nullptr)
	{
		pState->Bulk.DevIntrf.StopTx(&pState->Bulk.DevIntrf);
	}
}

static void UsbdCdcReset(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	if (pState != nullptr)
	{
		pState->Bulk.DevIntrf.Reset(&pState->Bulk.DevIntrf);
	}
}

static void UsbdCdcPowerOff(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcDisable(pDevIntrf);
}

static void *UsbdCdcGetHandle(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcState_t *pState = UsbdCdcFromDev(pDevIntrf);
	return pState != nullptr ? pState->pPublic : nullptr;
}

bool UsbdCdcIntrfInit(UsbdCdcDevIntrf_t * const pIntrf,
					  const UsbdCdcIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr ||
		pCfg->ItfNo < 0 || pCfg->ItfNo >= USBD_CDC_INTRF_MAXCNT)
	{
		return false;
	}

	// The current interface keeps three statically allocated CDC instances.
	// High-speed operation is not enabled here because the default FIFO sizing
	// is based on the full-speed CDC packet size.
	if (UsbdMaxSpeed() != USBD_SPEED_FULL)
	{
		return false;
	}

	const UsbDevCfg_t *pDevCfg = UsbDevGetCfg();
	if (pDevCfg == nullptr || pCfg->ItfNo >= pDevCfg->NbCdc ||
		s_State[pCfg->ItfNo].pPublic != nullptr)
	{
		return false;
	}

	uint8_t *pRxMem = pCfg->pRxFifoMem;
	int rxMemSize = pCfg->RxFifoMemSize;
	uint8_t *pTxMem = pCfg->pTxFifoMem;
	int txMemSize = pCfg->TxFifoMemSize;

	if (pRxMem == nullptr)
	{
		pRxMem = s_RxFifoMem[pCfg->ItfNo];
		rxMemSize = sizeof(s_RxFifoMem[pCfg->ItfNo]);
	}
	if (pTxMem == nullptr)
	{
		pTxMem = s_TxFifoMem[pCfg->ItfNo];
		txMemSize = sizeof(s_TxFifoMem[pCfg->ItfNo]);
	}

	UsbdCdcState_t *pState = &s_State[pCfg->ItfNo];
	pState->pPublic = pIntrf;
	pState->AppEvt = pCfg->EvtCB;

	UsbdBulkIntrfCfg_t bulkCfg = {};
	bulkCfg.bBlocking = pCfg->bBlocking;
	bulkCfg.RxFifoMemSize = rxMemSize;
	bulkCfg.pRxFifoMem = pRxMem;
	bulkCfg.TxFifoMemSize = txMemSize;
	bulkCfg.pTxFifoMem = pTxMem;
	bulkCfg.EvtCB = UsbdCdcEvent;

	if (!UsbdBulkIntrfInit(&pState->Bulk, &bulkCfg) ||
		!UsbdCdcFuncInit(&pState->Func, pCfg->ItfNo, &pState->Bulk) ||
		!UsbDevRegisterFunc(UsbdCdcPump, pState))
	{
		pState->pPublic = nullptr;
		pState->AppEvt = nullptr;
		return false;
	}

	pIntrf->DevIntrf.pDevData = pState;
	pIntrf->DevIntrf.IntPrio = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.bDma = false;
	pIntrf->DevIntrf.bIntEn = true;
	pIntrf->DevIntrf.Disable = UsbdCdcDisable;
	pIntrf->DevIntrf.Enable = UsbdCdcEnable;
	pIntrf->DevIntrf.GetRate = UsbdCdcGetRate;
	pIntrf->DevIntrf.SetRate = UsbdCdcSetRate;
	pIntrf->DevIntrf.StartRx = UsbdCdcStartRx;
	pIntrf->DevIntrf.RxData = UsbdCdcRxData;
	pIntrf->DevIntrf.StopRx = UsbdCdcStopRx;
	pIntrf->DevIntrf.StartTx = UsbdCdcStartTx;
	pIntrf->DevIntrf.TxData = UsbdCdcTxData;
	pIntrf->DevIntrf.TxSrData = UsbdCdcTxSrData;
	pIntrf->DevIntrf.StopTx = UsbdCdcStopTx;
	pIntrf->DevIntrf.Reset = UsbdCdcReset;
	pIntrf->DevIntrf.PowerOff = UsbdCdcPowerOff;
	pIntrf->DevIntrf.GetHandle = UsbdCdcGetHandle;

	pIntrf->RxDropCnt = 0;
	pIntrf->TxDropCnt = 0;
	memset(pIntrf->TransBuff, 0, sizeof(pIntrf->TransBuff));
	pIntrf->TransBuffLen = 0;
	pIntrf->TxPendingOfs = 0;
	pIntrf->TxPendingLen = 0;
	pIntrf->RxErrCnt = 0;
	pIntrf->TxBusyCnt = 0;
	pIntrf->hRxFifo = pState->Bulk.hRxFifo;
	pIntrf->hTxFifo = pState->Bulk.hTxFifo;
	pIntrf->ItfNo = pCfg->ItfNo;
	pIntrf->bEnabled = true;
	pIntrf->bPortOpen = false;

	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 1);
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	return true;
}

void UsbdCdcIntrfProcess(UsbdCdcDevIntrf_t * const pIntrf)
{
	UsbdCdcState_t *pState = UsbdCdcFromPublic(pIntrf);
	if (pState != nullptr)
	{
		UsbdCdcPump(pState);
	}
}

bool UsbdCdcIntrfPortIsOpen(UsbdCdcDevIntrf_t * const pIntrf)
{
	UsbdCdcState_t *pState = UsbdCdcFromPublic(pIntrf);
	return pState != nullptr && UsbdCdcFuncPortIsOpen(&pState->Func);
}

bool UsbdCdcIntrf::Init(const UsbdCdcIntrfCfg_t &Cfg)
{
	return UsbdCdcIntrfInit(&vUsbDevIntrf, &Cfg);
}

bool UsbdCdcIntrf::IsPortOpen(void)
{
	return UsbdCdcIntrfPortIsOpen(&vUsbDevIntrf);
}

bool UsbdCdcIntrf::RequestToSend(int NbBytes)
{
	UsbdCdcState_t *pState = UsbdCdcFromPublic(&vUsbDevIntrf);
	return pState != nullptr &&
		UsbdBulkIntrfRequestToSend(&pState->Bulk, NbBytes);
}

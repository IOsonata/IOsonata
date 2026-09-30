/**-------------------------------------------------------------------------
@file	usb_iso.cpp

@brief	USB isochronous specialization of UsbIntrf.

@author	Hoang Nguyen Hoan
@date	Sep. 8, 2026

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

#include "coredev/interrupt.h"
#include "usb/usb_iso.h"

static bool UsbIsoIntrfEpSupported(int DevNo, uint8_t EpNo)
{
	if (DevNo < 0 || DevNo >= USB_CTRLR_CNT || EpNo == 0U || EpNo > 15U ||
		!USB_ISO_SUPPORTED(DevNo))
	{
		return false;
	}

	const uint16_t bit = (uint16_t)(1U << EpNo);
	return (USB_ISO_EPIN_MASK(DevNo) & bit) != 0U &&
		(USB_ISO_EPOUT_MASK(DevNo) & bit) != 0U;
}

// Close both directions of the endpoint pair, then drop the data path.
static void UsbIsoIntrfRelease(UsbIsoIntrf_t *pIntrf, bool bCloseEp)
{
	pIntrf->Opened = false;
	if (bCloseEp)
	{
		UsbCtrlrEpClose(pIntrf->pData->DevNo, pIntrf->EpNo, false);
		UsbCtrlrEpClose(pIntrf->pData->DevNo, pIntrf->EpNo, true);
	}
	UsbIntrfUnconfigure(pIntrf->pData);
}

static bool UsbIsoIntrfActivate(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf->Opened)
	{
		return true;
	}
	if (pIntrf->Mps == 0U || !UsbIntrfConfigure(pIntrf->pData, pIntrf->Mps))
	{
		return false;
	}
	if (!UsbCtrlrIsoOpen(pIntrf->pData->DevNo, pIntrf->EpNo, true,
			pIntrf->Mps) ||
		!UsbCtrlrIsoOpen(pIntrf->pData->DevNo, pIntrf->EpNo, false,
			pIntrf->Mps))
	{
		UsbIsoIntrfRelease(pIntrf, true);
		return false;
	}
	pIntrf->Opened = true;
	return true;
}

static void UsbIsoIntrfDisable(DevIntrf_t * const pDev)
{
	UsbIsoIntrf_t *pIntrf = UsbIsoIntrfFromDev(pDev);
	UsbIsoIntrfRelease(pIntrf, pIntrf->Opened);
}

static void UsbIsoIntrfEnable(DevIntrf_t * const pDev)
{
	(void)UsbIsoIntrfActivate(UsbIsoIntrfFromDev(pDev));
}

static void UsbIsoIntrfResetDev(DevIntrf_t * const pDev)
{
	UsbIsoIntrfReset(UsbIsoIntrfFromDev(pDev));
}

// One frame per call from the RX FIFO. A frame larger than the caller's
// buffer stays queued; zero-length frames are counted and skipped, as the
// packet RxData in UsbIntrf does.
static int UsbIsoIntrfRxData(DevIntrf_t * const pDev, uint8_t *pBuffer,
							 int BufferLen)
{
	UsbDevIntrf_t *pData = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	UsbIsoIntrf_t *pIntrf =
		static_cast<UsbIsoIntrf_t *>(pData->pClassContext);

	if (pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	// The copy runs under interrupt exclusion so a close or bus reset in the
	// interrupt cannot empty the FIFO under the frame being read. A frame is
	// at most the packet size.
	int count = 0;
	const uint32_t state = DisableInterrupt();
	UsbPkt_t *pkt;
	while ((pkt = reinterpret_cast<UsbPkt_t *>(
				CFifoPeek(pData->hRxFifo))) != nullptr)
	{
		const uint16_t len = pkt->Hdr.Length;
		if (len == 0U)
		{
			pIntrf->RxEmptyCnt++;
			(void)CFifoGet(pData->hRxFifo);
			continue;
		}
		if (len > pIntrf->Mps)
		{
			pIntrf->RxMissCnt++;
			(void)CFifoGet(pData->hRxFifo);
			continue;
		}
		if (BufferLen >= (int)len)
		{
			memcpy(pBuffer, pkt->Data, len);
			(void)CFifoGet(pData->hRxFifo);
			count = len;
		}
		break;
	}
	EnableInterrupt(state);

	return count;
}

// One frame per call into the TX FIFO.
static int UsbIsoIntrfTxData(DevIntrf_t * const pDev,
							 const uint8_t *pData, int DataLen)
{
	UsbDevIntrf_t *pIntrfData = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	UsbIsoIntrf_t *pIntrf =
		static_cast<UsbIsoIntrf_t *>(pIntrfData->pClassContext);
	if (DataLen < 0 || DataLen > 0xFFFF ||
		!UsbIsoIntrfSendFrame(pIntrf, pData, (uint16_t)DataLen))
	{
		return 0;
	}
	return DataLen;
}

// Service interval: offer the head of the TX FIFO. The controller keeps the
// slot until the frame's DMA has ended (the IN completion releases it), and
// refuses the offer while the previous frame is still in flight, in which
// case the same head is offered again next interval. With no frame queued
// the call still services the OUT direction.
static void UsbIsoIntrfProcessEvent(UsbIsoIntrf_t *pIntrf, uint16_t FrameNo)
{
	if (!pIntrf->Opened || pIntrf->Suspended ||
		(FrameNo & ((1U << (pIntrf->Interval - 1U)) - 1U)) != 0U)
	{
		return;
	}

	const UsbPkt_t *pPacket =
		reinterpret_cast<const UsbPkt_t *>(CFifoPeek(pIntrf->pData->hTxFifo));
	uint8_t *pBuffer = nullptr;
	uint16_t length = 0U;
	if (pPacket != nullptr)
	{
		pBuffer = const_cast<uint8_t *>(pPacket->Data);
		length = pPacket->Hdr.Length;
	}
	(void)UsbCtrlrIsoSend(pIntrf->pData->DevNo, pIntrf->EpNo, pBuffer, length);
}

// IN completion: the head of the TX FIFO is the frame whose DMA has ended.
// Release it and report the captured DMA length: TX_READY while
// more frames wait, TX_FIFO_EMPTY when the queue drained, TX_TIMEOUT when
// the controller failed the frame. Nothing is started here; the next frame
// leaves at its own service interval.
static void UsbIsoIntrfTxComplete(UsbIsoIntrf_t *pIntrf,
								  UsbCtrlrXferResult_t Result, uint16_t length)
{
	hCFifo_t hTx = pIntrf->pData->hTxFifo;
	if (CFifoGet(hTx) == nullptr)
	{
		return;
	}

	const bool empty = CFifoUsed(hTx) == 0;
	atomic_store_explicit(&pIntrf->pData->DevIntrf.bTxReady, empty,
		memory_order_release);

	DEVINTRF_EVT event = empty ? DEVINTRF_EVT_TX_FIFO_EMPTY : DEVINTRF_EVT_TX_READY;
	if (Result != USB_CTRLR_XFER_SUCCESS)
	{
		pIntrf->TxMissCnt++;
		event = DEVINTRF_EVT_TX_TIMEOUT;
	}
	else if (length == 0U)
		pIntrf->TxEmptyCnt++;
	UsbIntrfNotify(pIntrf->pData, event, length);
}

// Controller callback for the ISO IN endpoint. The interval and the IN
// completion are isochronous concerns, separate from the bulk and interrupt
// send path in UsbIntrf, the same way EP0 has its own send path in the
// controller. The OUT endpoint stays with UsbIntrf.
static void UsbIsoIntrfCtrlrInEvent(UsbCtrlrEvtType_t Event,
									uint16_t Length, void *pContext)
{
	UsbIsoIntrf_t *pIntrf = static_cast<UsbIsoIntrf_t *>(pContext);

	switch (Event)
	{
		case USB_CTRLR_EVT_SOF:
			UsbIsoIntrfProcessEvent(pIntrf, Length);
			return;

		case USB_CTRLR_EVT_XFER_CMPL:
		case USB_CTRLR_EVT_XFER_FAILED:
			UsbIsoIntrfTxComplete(pIntrf, Event == USB_CTRLR_EVT_XFER_CMPL ?
				USB_CTRLR_XFER_SUCCESS : USB_CTRLR_XFER_FAILED, Length);
			return;

		default:
			// CANCEL: the close path flushes the queue.
			return;
	}
}

bool UsbIsoIntrfInit(UsbIsoIntrf_t *pIntrf, UsbDevIntrf_t *pData,
					 const UsbIsoIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pData == nullptr || pCfg == nullptr ||
		USB_ISO_INTRF_MAX_MPS == 0U ||
		!UsbIsoIntrfEpSupported(pCfg->DevNo, pCfg->EpNo) ||
		!USB_CTRLR_ISO_INIT(pCfg->DevNo))
	{
		return false;
	}

	// The controller moves each OUT frame straight into the RX FIFO block
	// UsbIntrf reserved for it, so a block holds one frame of up to
	// BufferSize bytes, within the controller's ISO packet limit.
	if (pCfg->BufferSize == 0U || pCfg->BufferSize > USB_ISO_INTRF_MAX_MPS)
	{
		return false;
	}

	memset(pIntrf, 0, sizeof(*pIntrf));
	pIntrf->pData = pData;
	pIntrf->pContext = pCfg->pContext;
	pIntrf->EpNo = pCfg->EpNo;

	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = pCfg->DevNo;
	cfg.EpNo = pCfg->EpNo;
	// Blocking: the controller reads the TX head in place (peek at the
	// service interval, get at completion), so a put must never reclaim it;
	// a full TX queue refuses the frame at SendFrame. A full RX FIFO has no
	// destination for the next OUT frame, which is then dropped: an
	// isochronous endpoint cannot hold the host off.
	cfg.bBlocking = true;
	cfg.Mode = USB_INTRF_MODE_PACKET;
	cfg.BufferSize = pCfg->BufferSize;
	cfg.RxFifoMemSize = (int)USB_ISO_INTRF_FIFO_MEMSIZE(pCfg->BufferSize);
	cfg.pRxFifoMem = pCfg->pRxFifoMem;
	cfg.TxFifoMemSize = (int)USB_ISO_INTRF_FIFO_MEMSIZE(pCfg->BufferSize);
	cfg.pTxFifoMem = pCfg->pTxFifoMem;
	cfg.TxFifoBlkSize = (uint16_t)USB_INTRF_PKT_BLKSIZE(pCfg->BufferSize);
	// The application's callback, as for every other UsbIntrf user.
	cfg.EvtCB = pCfg->EvtCB;

	if (!UsbIntrfInit(pIntrf->pData, &cfg))
	{
		return false;
	}

	// UsbIntrf keeps the OUT direction, the FIFOs and DeviceIntrf. The IN
	// direction is isochronous: frames leave at the service interval, not
	// from the previous completion, so the endpoint callback is ours.
	UsbCtrlrEpBind(pCfg->DevNo, pCfg->EpNo, true, true,
		UsbIsoIntrfCtrlrInEvent, pIntrf);
	// The DeviceIntrf data calls map onto one frame per call.
	pIntrf->pData->DevIntrf.RxData = UsbIsoIntrfRxData;
	pIntrf->pData->DevIntrf.TxData = UsbIsoIntrfTxData;
	pIntrf->pData->DevIntrf.TxSrData = UsbIsoIntrfTxData;
	pIntrf->pData->pClassContext = pIntrf;
	pIntrf->pData->DevIntrf.Disable = UsbIsoIntrfDisable;
	pIntrf->pData->DevIntrf.Enable = UsbIsoIntrfEnable;
	pIntrf->pData->DevIntrf.Reset = UsbIsoIntrfResetDev;
	return true;
}

bool UsbIsoIntrfOpen(UsbIsoIntrf_t *pIntrf, uint16_t Mps, uint8_t Interval)
{
	if (pIntrf == nullptr || pIntrf->pData == nullptr ||
		Mps == 0U || Mps > USB_ISO_INTRF_MAX_MPS ||
		Mps > pIntrf->pData->BufferSize ||
		Interval == 0U || Interval > 16U)
	{
		return false;
	}

	UsbIsoIntrfClose(pIntrf);
	pIntrf->Mps = Mps;
	pIntrf->Interval = Interval;

	if (atomic_load_explicit(&pIntrf->pData->DevIntrf.EnCnt,
			memory_order_acquire) > 0 &&
		!UsbIsoIntrfActivate(pIntrf))
	{
		pIntrf->Mps = 0U;
		pIntrf->Interval = 0U;
		return false;
	}

	return true;
}

void UsbIsoIntrfClose(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	UsbIsoIntrfRelease(pIntrf, pIntrf->Opened);
	pIntrf->Suspended = false;
	pIntrf->Mps = 0U;
	pIntrf->Interval = 0U;
}

void UsbIsoIntrfReset(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	UsbIsoIntrfClose(pIntrf);
	pIntrf->RxMissCnt = 0U;
	pIntrf->TxMissCnt = 0U;
	pIntrf->RxEmptyCnt = 0U;
	pIntrf->TxEmptyCnt = 0U;
}

void UsbIsoIntrfSuspend(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf != nullptr && pIntrf->Mps != 0U)
	{
		pIntrf->Suspended = true;
	}
}

bool UsbIsoIntrfResume(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->Mps == 0U)
	{
		return false;
	}

	pIntrf->Suspended = false;
	return true;
}

bool UsbIsoIntrfSendFrame(UsbIsoIntrf_t *pIntrf, const uint8_t *pData,
						  uint16_t Length)
{
	// Check state and fill under interrupt exclusion: a lifecycle event
	// must not close the interface between accepting and queuing the frame.
	const uint32_t state = DisableInterrupt();
	if (pIntrf == nullptr ||
		!pIntrf->Opened || pIntrf->Suspended || Length > pIntrf->Mps ||
		(Length != 0U && pData == nullptr))
	{
		EnableInterrupt(state);
		return false;
	}

	UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(
		CFifoPut(pIntrf->pData->hTxFifo));
	if (pPacket == nullptr)
	{
		EnableInterrupt(state);
		return false;
	}
	pPacket->Hdr.Length = Length;
	pPacket->Hdr.Reserved = 0U;
	if (Length != 0U)
	{
		memcpy(pPacket->Data, pData, Length);
	}
	atomic_store_explicit(&pIntrf->pData->DevIntrf.bTxReady, false,
		memory_order_release);
	EnableInterrupt(state);
	return true;
}



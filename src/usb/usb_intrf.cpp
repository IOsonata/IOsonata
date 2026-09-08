/**-------------------------------------------------------------------------
@file	usb_intrf.cpp

@brief	Generic USB device data interface implementation.

@author	Hoang Nguyen Hoan
@date	Sep. 1, 2026

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

#include "istddef.h"
#include "coredev/interrupt.h"
#include "usb/usb_intrf.h"


static int UsbIntrfEpSendByteMode(UsbDevIntrf_t *pIntrf);
static int UsbIntrfEpSendPktMode(UsbDevIntrf_t *pIntrf);
static void UsbIntrfRxXferComplete(UsbDevIntrf_t *pIntrf, uint16_t Length,
								   UsbCtrlrXferResult_t Result);
static void UsbIntrfTxXferComplete(UsbDevIntrf_t *pIntrf, uint16_t Length,
								   UsbCtrlrXferResult_t Result);
static void UsbIntrfRxResumePending(UsbDevIntrf_t *pIntrf);

/**
 * Submit the DMA request for one controller DRDY indication.
 *
 * Blocking mode defers the request when no RX packet slot is available. The
 * controller already holds that packet, so RxPending means exactly one DRDY
 * event remains to be serviced. Non-blocking mode never gates the transfer on
 * CFifo availability; CFifoPut() applies the non-blocking overflow policy on
 * completion.
 */
static inline __attribute__((always_inline))
void UsbIntrfRxSubmit(UsbDevIntrf_t *pIntrf)
{
	if (CFifoAvail(pIntrf->hRxFifo) <= 0)
	{
		pIntrf->RxPending = true;
		return;
	}

	(void)UsbCtrlrEpXfer(pIntrf->DevNo,
						USB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->Mps);
	pIntrf->RxPending = false;
}

/** Service only a DRDY event that was previously deferred. */
static void UsbIntrfRxResumePending(UsbDevIntrf_t *pIntrf)
{
	if (!pIntrf->RxPending)
	{
		return;
	}

	if (pIntrf->RxPending && CFifoAvail(pIntrf->hRxFifo) > 0)
	{
		(void)UsbCtrlrEpXfer(pIntrf->DevNo,
							USB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->Mps);
		pIntrf->RxPending = false;
	}
}

static inline __attribute__((always_inline))
void UsbIntrfSetTxIdle(UsbDevIntrf_t *pIntrf)
{
	atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
						  memory_order_release);
}

static inline __attribute__((always_inline))
bool UsbIntrfTakeTx(UsbDevIntrf_t *pIntrf)
{
	return atomic_exchange_explicit(&pIntrf->DevIntrf.bTxReady, false,
									memory_order_acquire);
}

static void UsbIntrfTxFailure(UsbDevIntrf_t *pIntrf, uint16_t Length)
{
	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_TIMEOUT,
							   nullptr, Length);
	}
}

static int UsbIntrfEpSendByteMode(UsbDevIntrf_t *pIntrf)
{
	int cnt = 0;
	int length = pIntrf->Mps;
	uint8_t *buff = pIntrf->pTxBuffer;
	uint32_t state = DisableInterrupt();

	while (length > 0)
	{
		int l = length;
		uint8_t *p = CFifoGetMultiple(pIntrf->hTxFifo, &l);

		if (p == nullptr)
		{
			break;
		}

		memcpy(buff, p, l);
		length -= l;
		buff += l;
		cnt += l;
	}

	EnableInterrupt(state);

	if (cnt <= 0)
	{
		UsbIntrfSetTxIdle(pIntrf);
		return -1;
	}

	(void)UsbCtrlrEpXfer(pIntrf->DevNo,
						 USB_ENDPADDR_DIRIN(pIntrf->EpNo), (uint16_t)cnt);

	return cnt;
}

static int UsbIntrfEpSendPktMode(UsbDevIntrf_t *pIntrf)
{
	UsbPkt_t *pkt = reinterpret_cast<UsbPkt_t *>(CFifoPeek(pIntrf->hTxFifo));

	if (pkt == nullptr)
	{
		UsbIntrfSetTxIdle(pIntrf);
		return -1;
	}

	int cnt = pkt->Hdr.Length;
	if (cnt > 0)
	{
		memcpy(pIntrf->pTxBuffer, pkt->Data, cnt);
	}
	(void)CFifoGet(pIntrf->hTxFifo);

	(void)UsbCtrlrEpXfer(pIntrf->DevNo,
						 USB_ENDPADDR_DIRIN(pIntrf->EpNo), (uint16_t)cnt);

	return cnt;
}

static void UsbIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void UsbIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static uint32_t UsbIntrfGetRate(DevIntrf_t * const pDevIntrf)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);

	if (pIntrf == nullptr || pIntrf->Mps == 0U)
	{
		return 0;
	}

	return UsbCtrlrHighSpeed(pIntrf->DevNo) ?
		   USB_LINK_RATE_HIGH : USB_LINK_RATE_FULL;
}

static uint32_t UsbIntrfSetRate(DevIntrf_t * const pDevIntrf, uint32_t)
{
	return UsbIntrfGetRate(pDevIntrf);
}

static bool UsbIntrfStartRx(DevIntrf_t * const, uint32_t)
{
	return true;
}

static int UsbIntrfRxData(DevIntrf_t * const pDevIntrf, uint8_t *pBuffer,
						 int BufferLen)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);

	if (pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	int cnt = 0;
	bool released = false;

	while (BufferLen > 0)
	{
		UsbPkt_t *pkt = reinterpret_cast<UsbPkt_t *>(CFifoPeek(pIntrf->hRxFifo));
		if (pkt == nullptr)
		{
			break;
		}

		const uint16_t len = pkt->Hdr.Length;

		if (BufferLen < (int)len)
		{
			break;
		}

		if (len > 0U)
		{
			memcpy(pBuffer, pkt->Data, len);
		}

		(void)CFifoGet(pIntrf->hRxFifo);
		released = true;
		pBuffer += len;
		BufferLen -= len;
		cnt += len;
	}

	if (released && pIntrf->RxPending)
	{
		UsbIntrfRxResumePending(pIntrf);
	}

	return cnt;
}

static void UsbIntrfStopRx(DevIntrf_t * const)
{
}

static bool UsbIntrfStartTx(DevIntrf_t * const, uint32_t)
{
	return true;
}

static int UsbIntrfTxPackets(DevIntrf_t * const pDevIntrf,
							 const uint8_t *pData, int DataLen)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);

	if (pIntrf == nullptr || pIntrf->Mps == 0U ||
		pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	const int blockSize = (int)CFifoBlockSize(pIntrf->hTxFifo);
	uint32_t state = DisableInterrupt();
	int cnt = 0;

	while (DataLen >= blockSize)
	{
		const UsbPkt_t *pPacket = reinterpret_cast<const UsbPkt_t *>(pData);
		if (pPacket->Hdr.Length > pIntrf->Mps)
		{
			break;
		}

		uint8_t *p = CFifoPut(pIntrf->hTxFifo);
		if (p == nullptr)
		{
			break;
		}

		memcpy(p, pData, (size_t)blockSize);
		pData += blockSize;
		DataLen -= blockSize;
		cnt += blockSize;
	}

	EnableInterrupt(state);

	if (UsbIntrfTakeTx(pIntrf))
	{
		UsbIntrfEpSendPktMode(pIntrf);
	}

	return cnt;
}

static int UsbIntrfTxBytes(DevIntrf_t * const pDevIntrf,
						   const uint8_t *pData, int DataLen)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);

	uint32_t state = DisableInterrupt();
	int cnt = 0;

	while (DataLen > 0)
	{
		int length = DataLen;
		uint8_t *p = CFifoPutMultiple(pIntrf->hTxFifo, &length);
		if (p == nullptr || length <= 0)
		{
			break;
		}

		if (length == 1)
		{
			*p = *pData;
		}
		else
		{
			memcpy(p, pData, (size_t)length);
		}

		pData += length;
		DataLen -= length;
		cnt += length;
	}

	EnableInterrupt(state);

	if (UsbIntrfTakeTx(pIntrf))
	{
		(void)UsbIntrfEpSendByteMode(pIntrf);
	}

	return cnt;
}

static int UsbIntrfTxSrData(DevIntrf_t * const pDevIntrf,
							 const uint8_t *pData, int DataLen)
{
	return pDevIntrf->TxData(pDevIntrf, pData, DataLen);
}

static void UsbIntrfStopTx(DevIntrf_t * const)
{
}

static void UsbIntrfReset(DevIntrf_t * const pDevIntrf)
{
	UsbIntrfUnconfigure(static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData));
}

static void UsbIntrfPowerOff(DevIntrf_t * const pDevIntrf)
{
	UsbIntrfDisable(pDevIntrf);
}

static void *UsbIntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return pDevIntrf->pDevData;
}

static void UsbIntrfCtrlrEvent(uint8_t EpAddr, UsbCtrlrEvtType_t Event,
							   uint16_t Length,
							   UsbCtrlrXferResult_t Result,
							   void *pContext)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pContext);

#if 0
	if (Event == USB_CTRLR_EVT_DRDY)
	{
		UsbIntrfRxSubmit(pIntrf);
	}

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		UsbIntrfTxXferComplete(pIntrf, Length, Result);
	}
	else
	{
		UsbIntrfRxXferComplete(pIntrf, Length, Result);
	}

#else
	switch (Event)
	{
		case USB_CTRLR_EVT_DRDY:
			{
				if (CFifoAvail(pIntrf->hRxFifo) <= 0)
				{
					pIntrf->RxPending = true;

					if (pIntrf->DevIntrf.EvtCB != nullptr)
					{
						pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
										   DEVINTRF_EVT_RX_FIFO_FULL, nullptr, 0);
					}
					return;
				}

				(void)UsbCtrlrEpXfer(pIntrf->DevNo,
									USB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->Mps);
				pIntrf->RxPending = false;
				return;
			}
			break;
		case USB_CTRLR_EVT_XFER_CMPL:
			{
				if (USB_ENDPADDR_IS_IN(EpAddr))
				{
					if (Result == USB_CTRLR_XFER_FAILED)
					{
						UsbIntrfTxFailure(pIntrf, Length);
						return;
					}

					if (pIntrf->EpSend(pIntrf) >= 0)
					{
						return;
					}

					if (pIntrf->DevIntrf.EvtCB != nullptr)
					{
						pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
											   DEVINTRF_EVT_TX_FIFO_EMPTY,
											   nullptr, 0);
					}
				}
				else
				{
					if (Result == USB_CTRLR_XFER_SUCCESS)
					{
						UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(
							CFifoPut(pIntrf->hRxFifo));
						pPacket->Hdr.Length = Length;
						pPacket->Hdr.Reserved = 0U;
						if (Length > 0U)
						{
							memcpy(pPacket->Data, pIntrf->pRxBuffer, Length);
						}

						if (pIntrf->DevIntrf.EvtCB != nullptr)
						{
							const int used = CFifoUsed(pIntrf->hRxFifo);
							pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
											   DEVINTRF_EVT_RX_DATA, nullptr, used);
						}
						return;
					}

					if (Result == USB_CTRLR_XFER_FAILED)
					{
						pIntrf->RxDropCnt++;
						if (pIntrf->DevIntrf.EvtCB != nullptr)
						{
							pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
											   DEVINTRF_EVT_RX_TIMEOUT, nullptr, Length);
						}
					}

				}
			}
			break;
	}
#endif
}

bool UsbIntrfInit(UsbDevIntrf_t *pIntrf, const UsbIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr ||
		pCfg->EpNo == 0U || pCfg->EpNo > USB_ENDPADDR_NUM_MASK ||
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0 ||
		pCfg->pRxBuffer == nullptr || pCfg->pTxBuffer == nullptr ||
		pCfg->BufferSize == 0U || pCfg->TxFifoBlkSize == 0U ||
		(((uintptr_t)pCfg->pRxFifoMem & 3U) != 0U) ||
		(((uintptr_t)pCfg->pTxFifoMem & 3U) != 0U) ||
		(((uintptr_t)pCfg->pRxBuffer & 3U) != 0U) ||
		(((uintptr_t)pCfg->pTxBuffer & 3U) != 0U))
	{
		return false;
	}

	pIntrf->hTxFifo = CFifoInit(pCfg->pTxFifoMem,
								 (uint32_t)pCfg->TxFifoMemSize,
								 pCfg->TxFifoBlkSize, pCfg->bBlocking);
	pIntrf->hRxFifo = CFifoInit(pCfg->pRxFifoMem,
								 (uint32_t)pCfg->RxFifoMemSize,
								 USB_INTRF_PKT_BLKSIZE(pCfg->BufferSize),
								 pCfg->bBlocking);
	if (pIntrf->hTxFifo == nullptr || pIntrf->hRxFifo == nullptr)
	{
		return false;
	}

	pIntrf->DevNo = pCfg->DevNo;
	pIntrf->EpNo = pCfg->EpNo;
	pIntrf->pRxBuffer = pCfg->pRxBuffer;
	pIntrf->pTxBuffer = pCfg->pTxBuffer;
	pIntrf->BufferSize = pCfg->BufferSize;
	pIntrf->Mps = 0U;
	pIntrf->RxDropCnt = 0U;
	pIntrf->bBlocking = pCfg->bBlocking;
	pIntrf->RxPending = false;
	pIntrf->pClassContext = nullptr;

	pIntrf->DevIntrf.pDevData = pIntrf;
	pIntrf->DevIntrf.IntPrio = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.bDma = true;
	pIntrf->DevIntrf.bIntEn = true;
	pIntrf->DevIntrf.Disable = UsbIntrfDisable;
	pIntrf->DevIntrf.Enable = UsbIntrfEnable;
	pIntrf->DevIntrf.GetRate = UsbIntrfGetRate;
	pIntrf->DevIntrf.SetRate = UsbIntrfSetRate;
	pIntrf->DevIntrf.StartRx = UsbIntrfStartRx;
	pIntrf->DevIntrf.RxData = UsbIntrfRxData;
	pIntrf->DevIntrf.StopRx = UsbIntrfStopRx;
	pIntrf->DevIntrf.StartTx = UsbIntrfStartTx;
	if (pCfg->TxFifoBlkSize == 1)
	{
		pIntrf->DevIntrf.TxData = UsbIntrfTxBytes;
		pIntrf->EpSend = UsbIntrfEpSendByteMode;
	}
	else
	{
		pIntrf->DevIntrf.TxData = UsbIntrfTxPackets;
		pIntrf->EpSend = UsbIntrfEpSendPktMode;
	}
	pIntrf->DevIntrf.TxSrData = UsbIntrfTxSrData;
	pIntrf->DevIntrf.StopTx = UsbIntrfStopTx;
	pIntrf->DevIntrf.Reset = UsbIntrfReset;
	pIntrf->DevIntrf.PowerOff = UsbIntrfPowerOff;
	pIntrf->DevIntrf.GetHandle = UsbIntrfGetHandle;

	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	if (!UsbCtrlrEpRegister(pIntrf->DevNo,
		USB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->pRxBuffer,
		pCfg->bBlocking, UsbIntrfCtrlrEvent, pIntrf) ||
		!UsbCtrlrEpRegister(pIntrf->DevNo,
		USB_ENDPADDR_DIRIN(pIntrf->EpNo), pIntrf->pTxBuffer,
		pCfg->bBlocking, UsbIntrfCtrlrEvent, pIntrf))
	{
		return false;
	}

	DeviceIntrfEnable(&pIntrf->DevIntrf);
	return true;
}

bool UsbIntrfConfigure(UsbDevIntrf_t *pIntrf, uint16_t Mps)
{
	if (pIntrf == nullptr || Mps == 0U || Mps > pIntrf->BufferSize ||
		pIntrf->hRxFifo == nullptr || pIntrf->hTxFifo == nullptr ||
		(CFifoBlockSize(pIntrf->hTxFifo) != 1U &&
		 CFifoBlockSize(pIntrf->hTxFifo) < sizeof(UsbPktHdr_t) + Mps))
	{
		return false;
	}

	pIntrf->Mps = Mps;
	pIntrf->RxPending = false;
	CFifoFlush(pIntrf->hRxFifo);
	CFifoFlush(pIntrf->hTxFifo);
	UsbIntrfSetTxIdle(pIntrf);
	return true;
}

void UsbIntrfUnconfigure(UsbDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->Mps = 0U;
	pIntrf->RxPending = false;
	if (pIntrf->hRxFifo != nullptr)
	{
		CFifoFlush(pIntrf->hRxFifo);
	}
	if (pIntrf->hTxFifo != nullptr)
	{
		CFifoFlush(pIntrf->hTxFifo);
	}
	UsbIntrfSetTxIdle(pIntrf);
}

static void UsbIntrfRxXferComplete(UsbDevIntrf_t *pIntrf, uint16_t Length,
								UsbCtrlrXferResult_t Result)
{
	if (Result == USB_CTRLR_XFER_SUCCESS)
	{
		UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(
			CFifoPut(pIntrf->hRxFifo));
		pPacket->Hdr.Length = Length;
		pPacket->Hdr.Reserved = 0U;
		if (Length > 0U)
		{
			memcpy(pPacket->Data, pIntrf->pRxBuffer, Length);
		}

		if (pIntrf->DevIntrf.EvtCB != nullptr)
		{
			const int used = CFifoUsed(pIntrf->hRxFifo);
			if (CFifoAvail(pIntrf->hRxFifo) == 0)
			{
				pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
								   DEVINTRF_EVT_RX_FIFO_FULL, nullptr, used);
			}
			pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_RX_DATA, nullptr, used);
		}
		return;
	}

	if (Result == USB_CTRLR_XFER_FAILED)
	{
		pIntrf->RxDropCnt++;
		if (pIntrf->DevIntrf.EvtCB != nullptr)
		{
			pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_RX_TIMEOUT, nullptr, Length);
		}
	}
}

static void UsbIntrfTxXferComplete(UsbDevIntrf_t *pIntrf,
								uint16_t Length, UsbCtrlrXferResult_t Result)
{
	if (Result != USB_CTRLR_XFER_SUCCESS)
	{
		if (Result == USB_CTRLR_XFER_FAILED)
		{
			UsbIntrfTxFailure(pIntrf, Length);
		}
		return;
	}

	if (pIntrf->EpSend(pIntrf) >= 0)
	{
		return;
	}

	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_FIFO_EMPTY,
							   nullptr, 0);
	}
}

void UsbIntrfXferComplete(UsbDevIntrf_t *pIntrf, uint8_t EpAddr,
						  uint16_t Length, UsbCtrlrXferResult_t Result)
{
	if (pIntrf == nullptr || USB_ENDPADDR_NUM(EpAddr) != pIntrf->EpNo)
	{
		return;
	}

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		UsbIntrfTxXferComplete(pIntrf, Length, Result);
	}
	else
	{
		UsbIntrfRxXferComplete(pIntrf, Length, Result);
	}
}

bool UsbIntrfRequestToSend(UsbDevIntrf_t *pIntrf, int NbBytes)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr || NbBytes <= 0)
	{
		return false;
	}

	if (pIntrf->hTxFifo->bBlocking == false)
	{
		return true;
	}

	const uint32_t blockSize = CFifoBlockSize(pIntrf->hTxFifo);
	int blocks = NbBytes;
	if (blockSize != 1U)
	{
		if ((NbBytes % (int)blockSize) != 0)
		{
			return false;
		}
		blocks = NbBytes / (int)blockSize;
	}

	return CFifoAvail(pIntrf->hTxFifo) >= blocks;
}

int UsbIntrfTxUsed(UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hTxFifo != nullptr ?
		   CFifoUsed(pIntrf->hTxFifo) : 0;
}

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

static inline __attribute__((always_inline))
bool UsbIntrfDirectReady(const UsbPkt_t *pPacket)
{
	return pPacket != nullptr &&
		(pPacket->Hdr.Reserved & USB_INTRF_SLOT_READY) != 0U;
}

static inline __attribute__((always_inline))
void UsbIntrfDirectClear(UsbPkt_t *pPacket)
{
	if (pPacket != nullptr)
	{
		pPacket->Hdr.Length = 0U;
		pPacket->Hdr.Reserved = 0U;
	}
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
		pBuffer += len;
		BufferLen -= len;
		cnt += len;
	}

	if (cnt > 0 && pIntrf->RxPending)
	{
		pIntrf->RxPending = false;
		(void)UsbCtrlrEpXfer(pIntrf->DevNo,
			USB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->Mps);
	}

	return cnt;
}

static int UsbIntrfRxDirect(DevIntrf_t * const pDevIntrf, uint8_t *pBuffer,
							int BufferLen)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);
	if (pIntrf == nullptr || pIntrf->Mode != USB_INTRF_MODE_DIRECT ||
		pIntrf->pRxDirectBuffer == nullptr || BufferLen < 0)
	{
		return 0;
	}

	const uint32_t state = DisableInterrupt();
	UsbPkt_t *pPacket = pIntrf->pRxDirectBuffer;
	if (!UsbIntrfDirectReady(pPacket))
	{
		EnableInterrupt(state);
		return 0;
	}

	const uint16_t len = pPacket->Hdr.Length;
	if ((len > 0U && pBuffer == nullptr) || BufferLen < (int)len)
	{
		EnableInterrupt(state);
		return 0;
	}

	if (len > 0U)
	{
		memcpy(pBuffer, pPacket->Data, len);
	}
	const bool restart = pIntrf->RxPending;
	pIntrf->RxPending = false;
	UsbIntrfDirectClear(pPacket);
	EnableInterrupt(state);
	if (restart && !UsbCtrlrEpXfer(pIntrf->DevNo,
		USB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->Mps))
	{
		pIntrf->RxPending = true;
	}
	return len;
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

static int UsbIntrfTxDirect(DevIntrf_t * const pDevIntrf,
							const uint8_t *pData, int DataLen)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);
	if (pIntrf == nullptr || pIntrf->Mode != USB_INTRF_MODE_DIRECT ||
		pIntrf->Mps == 0U || pIntrf->pTxDirectBuffer == nullptr ||
		DataLen < 0 || DataLen > (int)pIntrf->Mps ||
		(DataLen > 0 && pData == nullptr))
	{
		return 0;
	}

	if (!UsbIntrfTakeTx(pIntrf))
	{
		return 0;
	}

	UsbPkt_t *pPacket = pIntrf->pTxDirectBuffer;
	if (DataLen > 0)
	{
		memcpy(pPacket->Data, pData, (size_t)DataLen);
	}
	pPacket->Hdr.Length = (uint16_t)DataLen;
	pPacket->Hdr.Reserved = USB_INTRF_SLOT_READY;

	if (!UsbCtrlrEpXfer(pIntrf->DevNo,
		USB_ENDPADDR_DIRIN(pIntrf->EpNo), (uint16_t)DataLen))
	{
		UsbIntrfDirectClear(pPacket);
		UsbIntrfSetTxIdle(pIntrf);
		UsbIntrfTxFailure(pIntrf, (uint16_t)DataLen);
		return 0;
	}

	return DataLen;
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

static void UsbIntrfDirectRxComplete(UsbDevIntrf_t *pIntrf, uint16_t Length)
{
	UsbPkt_t *pPacket = pIntrf->pRxDirectBuffer;
	if (pPacket == nullptr || Length > pIntrf->Mps)
	{
		pIntrf->RxDropCnt++;
		return;
	}

	if (UsbIntrfDirectReady(pPacket))
	{
		pIntrf->RxDropCnt++;
	}

	pPacket->Hdr.Length = Length;
	pPacket->Hdr.Reserved = USB_INTRF_SLOT_READY;

	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		const int processed = pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
			DEVINTRF_EVT_RX_DATA, pPacket->Data, Length);
		if (processed >= (int)Length)
		{
			UsbIntrfDirectClear(pPacket);
		}
	}
}

static void UsbIntrfCtrlrEvent(uint8_t EpAddr, UsbCtrlrEvtType_t Event,
							   uint16_t Length,
							   UsbCtrlrXferResult_t Result,
							   void *pContext)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pContext);

	switch (Event)
	{
		case USB_CTRLR_EVT_DRDY:
			if (pIntrf->Mode == USB_INTRF_MODE_DIRECT)
			{
				if (!pIntrf->bBlocking)
				{
					return;
				}
				if (UsbIntrfDirectReady(pIntrf->pRxDirectBuffer))
				{
					pIntrf->RxPending = true;
					return;
				}
				pIntrf->RxPending = !UsbCtrlrEpXfer(pIntrf->DevNo,
					USB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->Mps);
				return;
			}

			if (CFifoAvail(pIntrf->hRxFifo) <= 0)
			{
				pIntrf->RxPending = true;
				return;
			}

			(void)UsbCtrlrEpXfer(pIntrf->DevNo,
				USB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->Mps);
			pIntrf->RxPending = false;
			return;

		case USB_CTRLR_EVT_XFER_CMPL:
			if (USB_ENDPADDR_IS_IN(EpAddr))
			{
				if (pIntrf->Mode == USB_INTRF_MODE_DIRECT)
				{
					const uint16_t requested = pIntrf->pTxDirectBuffer != nullptr ?
						pIntrf->pTxDirectBuffer->Hdr.Length : Length;
					UsbIntrfDirectClear(pIntrf->pTxDirectBuffer);
					UsbIntrfSetTxIdle(pIntrf);

					if (Result != USB_CTRLR_XFER_SUCCESS)
					{
						UsbIntrfTxFailure(pIntrf, requested);
						return;
					}

					if (pIntrf->DevIntrf.EvtCB != nullptr)
					{
						pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							DEVINTRF_EVT_TX_FIFO_EMPTY, nullptr, Length);
					}
					return;
				}

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
						DEVINTRF_EVT_TX_FIFO_EMPTY, nullptr, 0);
				}
				return;
			}

			if (Result == USB_CTRLR_XFER_SUCCESS)
			{
				if (pIntrf->Mode == USB_INTRF_MODE_DIRECT)
				{
					UsbIntrfDirectRxComplete(pIntrf, Length);
					return;
				}

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
					if (used >= pIntrf->hRxFifo->MaxIdxCnt)
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
			return;

		case USB_CTRLR_EVT_CANCEL:
			if (pIntrf->Mode == USB_INTRF_MODE_DIRECT &&
				USB_ENDPADDR_IS_IN(EpAddr))
			{
				UsbIntrfDirectClear(pIntrf->pTxDirectBuffer);
				UsbIntrfSetTxIdle(pIntrf);
			}
			return;

		default:
			return;
	}
}

bool UsbIntrfInit(UsbDevIntrf_t *pIntrf, const UsbIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr ||
		pCfg->EpNo == 0U || pCfg->EpNo > USB_ENDPADDR_NUM_MASK ||
		pCfg->pRxBuffer == nullptr || pCfg->pTxBuffer == nullptr ||
		pCfg->BufferSize == 0U ||
		(((uintptr_t)pCfg->pRxBuffer & 3U) != 0U) ||
		(((uintptr_t)pCfg->pTxBuffer & 3U) != 0U))
	{
		return false;
	}

	UsbIntrfMode_t mode = pCfg->Mode;
	if (mode == USB_INTRF_MODE_AUTO)
	{
		mode = pCfg->TxFifoBlkSize == 1U ?
			USB_INTRF_MODE_BYTE : USB_INTRF_MODE_PACKET;
	}
	if (mode != USB_INTRF_MODE_BYTE && mode != USB_INTRF_MODE_PACKET &&
		mode != USB_INTRF_MODE_DIRECT)
	{
		return false;
	}

	pIntrf->hTxFifo = nullptr;
	pIntrf->hRxFifo = nullptr;
	pIntrf->pRxDirectBuffer = nullptr;
	pIntrf->pTxDirectBuffer = nullptr;

	if (mode == USB_INTRF_MODE_DIRECT)
	{
		pIntrf->pRxDirectBuffer = reinterpret_cast<UsbPkt_t *>(pCfg->pRxBuffer);
		pIntrf->pTxDirectBuffer = reinterpret_cast<UsbPkt_t *>(pCfg->pTxBuffer);
		UsbIntrfDirectClear(pIntrf->pRxDirectBuffer);
		UsbIntrfDirectClear(pIntrf->pTxDirectBuffer);
		pIntrf->pRxBuffer = pIntrf->pRxDirectBuffer->Data;
		pIntrf->pTxBuffer = pIntrf->pTxDirectBuffer->Data;
	}
	else
	{
		if (pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
			pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0 ||
			pCfg->TxFifoBlkSize == 0U ||
			(((uintptr_t)pCfg->pRxFifoMem & 3U) != 0U) ||
			(((uintptr_t)pCfg->pTxFifoMem & 3U) != 0U))
		{
			return false;
		}

		pIntrf->hTxFifo = CFifoInit(pCfg->pTxFifoMem,
			(uint32_t)pCfg->TxFifoMemSize, pCfg->TxFifoBlkSize,
			pCfg->bBlocking);
		pIntrf->hRxFifo = CFifoInit(pCfg->pRxFifoMem,
			(uint32_t)pCfg->RxFifoMemSize,
			USB_INTRF_PKT_BLKSIZE(pCfg->BufferSize), pCfg->bBlocking);
		if (pIntrf->hTxFifo == nullptr || pIntrf->hRxFifo == nullptr)
		{
			return false;
		}

		pIntrf->pRxBuffer = pCfg->pRxBuffer;
		pIntrf->pTxBuffer = pCfg->pTxBuffer;
	}

	pIntrf->DevNo = pCfg->DevNo;
	pIntrf->EpNo = pCfg->EpNo;
	pIntrf->BufferSize = pCfg->BufferSize;
	pIntrf->Mps = 0U;
	pIntrf->RxDropCnt = 0U;
	pIntrf->bBlocking = pCfg->bBlocking;
	pIntrf->RxPending = false;
	pIntrf->Mode = mode;
	pIntrf->EpSend = nullptr;
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
	pIntrf->DevIntrf.StopRx = UsbIntrfStopRx;
	pIntrf->DevIntrf.StartTx = UsbIntrfStartTx;

	switch (mode)
	{
		case USB_INTRF_MODE_BYTE:
			pIntrf->DevIntrf.RxData = UsbIntrfRxData;
			pIntrf->DevIntrf.TxData = UsbIntrfTxBytes;
			pIntrf->EpSend = UsbIntrfEpSendByteMode;
			break;

		case USB_INTRF_MODE_PACKET:
			pIntrf->DevIntrf.RxData = UsbIntrfRxData;
			pIntrf->DevIntrf.TxData = UsbIntrfTxPackets;
			pIntrf->EpSend = UsbIntrfEpSendPktMode;
			break;

		case USB_INTRF_MODE_DIRECT:
			pIntrf->DevIntrf.RxData = UsbIntrfRxDirect;
			pIntrf->DevIntrf.TxData = UsbIntrfTxDirect;
			break;

		default:
			return false;
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
	if (pIntrf == nullptr || Mps == 0U || Mps > pIntrf->BufferSize)
	{
		return false;
	}

	if (pIntrf->Mode != USB_INTRF_MODE_DIRECT)
	{
		if (pIntrf->hRxFifo == nullptr || pIntrf->hTxFifo == nullptr ||
			(CFifoBlockSize(pIntrf->hTxFifo) != 1U &&
			 CFifoBlockSize(pIntrf->hTxFifo) < sizeof(UsbPktHdr_t) + Mps))
		{
			return false;
		}
	}

	pIntrf->Mps = Mps;
	pIntrf->RxPending = false;
	if (pIntrf->Mode == USB_INTRF_MODE_DIRECT)
	{
		UsbIntrfDirectClear(pIntrf->pRxDirectBuffer);
		UsbIntrfDirectClear(pIntrf->pTxDirectBuffer);
	}
	else
	{
		CFifoFlush(pIntrf->hRxFifo);
		CFifoFlush(pIntrf->hTxFifo);
	}
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
	if (pIntrf->Mode == USB_INTRF_MODE_DIRECT)
	{
		UsbIntrfDirectClear(pIntrf->pRxDirectBuffer);
		UsbIntrfDirectClear(pIntrf->pTxDirectBuffer);
	}
	else
	{
		if (pIntrf->hRxFifo != nullptr)
		{
			CFifoFlush(pIntrf->hRxFifo);
		}
		if (pIntrf->hTxFifo != nullptr)
		{
			CFifoFlush(pIntrf->hTxFifo);
		}
	}
	UsbIntrfSetTxIdle(pIntrf);
}

bool UsbIntrfRequestToSend(UsbDevIntrf_t *pIntrf, int NbBytes)
{
	if (pIntrf == nullptr || NbBytes < 0)
	{
		return false;
	}

	if (pIntrf->Mode == USB_INTRF_MODE_DIRECT)
	{
		return pIntrf->Mps > 0U && NbBytes <= (int)pIntrf->Mps &&
			pIntrf->pTxDirectBuffer != nullptr &&
			atomic_load_explicit(&pIntrf->DevIntrf.bTxReady,
				memory_order_acquire) &&
			!UsbIntrfDirectReady(pIntrf->pTxDirectBuffer);
	}

	if (pIntrf->hTxFifo == nullptr || NbBytes <= 0)
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
	if (pIntrf == nullptr)
	{
		return 0;
	}
	if (pIntrf->Mode == USB_INTRF_MODE_DIRECT)
	{
		return UsbIntrfDirectReady(pIntrf->pTxDirectBuffer) ? 1 : 0;
	}
	return pIntrf->hTxFifo != nullptr ? CFifoUsed(pIntrf->hTxFifo) : 0;
}

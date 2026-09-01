/**-------------------------------------------------------------------------
@file	usbd_bulk_intrf.cpp

@brief	Generic USB device bulk data interface implementation.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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
#include "usb/usbd_bulk_intrf.h"

static UsbdBulkDevIntrf_t *UsbdBulkIntrfData(DevIntrf_t * const pDevIntrf)
{
	if (pDevIntrf == nullptr || pDevIntrf->pDevData == nullptr)
	{
		return nullptr;
	}

	return static_cast<UsbdBulkDevIntrf_t *>(pDevIntrf->pDevData);
}

static bool UsbdBulkIntrfCanTx(UsbdBulkDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->bEnabled &&
		   pIntrf->TxEpAddr != 0U && pIntrf->pTxBuffer != nullptr &&
		   (pIntrf->TxReady == nullptr ||
		    pIntrf->TxReady(pIntrf, pIntrf->pContext));
}

static void UsbdBulkIntrfSetTxIdle(UsbdBulkDevIntrf_t *pIntrf)
{
	atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
						  memory_order_release);
}

static bool UsbdBulkIntrfTakeTx(UsbdBulkDevIntrf_t *pIntrf)
{
	return atomic_exchange_explicit(&pIntrf->DevIntrf.bTxReady, false,
									memory_order_acquire);
}

static bool UsbdBulkIntrfTxIdle(UsbdBulkDevIntrf_t *pIntrf)
{
	return atomic_load_explicit(&pIntrf->DevIntrf.bTxReady,
								memory_order_acquire);
}

static void UsbdBulkIntrfTxFailure(UsbdBulkDevIntrf_t *pIntrf,
								   uint16_t Length)
{
	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_TIMEOUT,
							   nullptr, Length);
	}
}

// Pull up to one packet out of the TX FIFO into the staging buffer.
static int UsbdBulkIntrfFillPacket(UsbdBulkDevIntrf_t *pIntrf)
{
	uint8_t *pBuffer = pIntrf->pTxBuffer;
	int remain = (int)pIntrf->TxMps;
	int cnt = 0;

	while (remain > 0)
	{
		int length = remain;
		uint8_t *p = CFifoGetMultiple(pIntrf->hTxFifo, &length);
		if (p == nullptr || length <= 0)
		{
			break;
		}

		memcpy(pBuffer + cnt, p, (size_t)length);
		cnt += length;
		remain -= length;
	}

	return cnt;
}

/**
 * Submit one Bulk IN transfer. The caller owns the endpoint, on failure the
 * endpoint is returned to idle.
 *
 * Tail false sends only a full packet, so a continuous producer never emits
 * short packets. Tail true also sends a partial packet or a required ZLP.
 */
static bool UsbdBulkIntrfSubmit(UsbdBulkDevIntrf_t *pIntrf, bool Tail)
{
	const int used = CFifoUsed(pIntrf->hTxFifo);
	int length = 0;

	if (used >= (int)pIntrf->TxMps || (Tail && used > 0))
	{
		length = UsbdBulkIntrfFillPacket(pIntrf);
	}

	if (length == 0 && !(Tail && pIntrf->TxZlpRequired))
	{
		UsbdBulkIntrfSetTxIdle(pIntrf);
		return false;
	}

	// Data or a ZLP leaves now, either one ends the pending tail
	pIntrf->TxZlpRequired = false;
	pIntrf->TxTailArmed = false;

	if (UsbdCtrlrEpXfer(pIntrf->TxEpAddr, pIntrf->pTxBuffer,
						(uint16_t)length))
	{
		return true;
	}

	// The endpoint refused the transfer, the staged bytes are lost
	UsbdBulkIntrfSetTxIdle(pIntrf);
	if (length > 0)
	{
		UsbdBulkIntrfTxFailure(pIntrf, (uint16_t)length);
	}

	return false;
}

// Take the idle endpoint and submit, see UsbdBulkIntrfSubmit().
static bool UsbdBulkIntrfStartXfer(UsbdBulkDevIntrf_t *pIntrf, bool Tail)
{
	// Cheap exit first, a streaming producer lands here on every write
	// while the previous packet is still in flight
	if (!UsbdBulkIntrfTxIdle(pIntrf) || !UsbdBulkIntrfCanTx(pIntrf) ||
		!UsbdBulkIntrfTakeTx(pIntrf))
	{
		return false;
	}

	return UsbdBulkIntrfSubmit(pIntrf, Tail);
}

static void UsbdBulkIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	UsbdBulkDevIntrf_t *pIntrf = UsbdBulkIntrfData(pDevIntrf);

	if (pIntrf != nullptr)
	{
		pIntrf->bEnabled = false;
	}
}

static void UsbdBulkIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	UsbdBulkDevIntrf_t *pIntrf = UsbdBulkIntrfData(pDevIntrf);

	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->bEnabled = true;

	if (pIntrf->RxKick != nullptr)
	{
		pIntrf->RxKick(pIntrf, pIntrf->pContext);
	}

	(void)UsbdBulkIntrfStartXfer(pIntrf, false);
}

static uint32_t UsbdBulkIntrfGetRate(DevIntrf_t * const)
{
	return 0;
}

static uint32_t UsbdBulkIntrfSetRate(DevIntrf_t * const, uint32_t)
{
	return 0;
}

static bool UsbdBulkIntrfStartRx(DevIntrf_t * const, uint32_t)
{
	return true;
}

static int UsbdBulkIntrfRxData(DevIntrf_t * const pDevIntrf,
							   uint8_t *pBuffer, int BufferLen)
{
	UsbdBulkDevIntrf_t *pIntrf = UsbdBulkIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pIntrf->hRxFifo == nullptr ||
		pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	int cnt = 0;

	while (BufferLen > 0)
	{
		int length = BufferLen;
		uint8_t *p = CFifoGetMultiple(pIntrf->hRxFifo, &length);
		if (p == nullptr || length <= 0)
		{
			break;
		}

		memcpy(pBuffer, p, (size_t)length);
		pBuffer += length;
		BufferLen -= length;
		cnt += length;
	}

	EnableInterrupt(state);

	if (cnt > 0 && pIntrf->bEnabled && pIntrf->RxKick != nullptr)
	{
		pIntrf->RxKick(pIntrf, pIntrf->pContext);
	}

	return cnt;
}

static void UsbdBulkIntrfStopRx(DevIntrf_t * const)
{
}

static bool UsbdBulkIntrfStartTx(DevIntrf_t * const, uint32_t)
{
	return true;
}

static int UsbdBulkIntrfTxData(DevIntrf_t * const pDevIntrf,
							   const uint8_t *pData, int DataLen)
{
	UsbdBulkDevIntrf_t *pIntrf = UsbdBulkIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

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

		memcpy(p, pData, (size_t)length);
		pData += length;
		DataLen -= length;
		cnt += length;
	}

	EnableInterrupt(state);

	if (cnt > 0 && pIntrf->TxMps > 0U)
	{
		const int used = CFifoUsed(pIntrf->hTxFifo);

		// A write into an empty FIFO starts a transaction and leaves at
		// once when the endpoint is idle, like a UART. While earlier data
		// is still queued the write aggregates into full packets and the
		// SOF handler sends the tail, so a fast producer that outruns the
		// bus keeps the FIFO non empty and never emits short packets.
		if (used >= (int)pIntrf->TxMps || used == cnt)
		{
			(void)UsbdBulkIntrfStartXfer(pIntrf, used == cnt);
		}
	}

	return cnt;
}

static int UsbdBulkIntrfTxSrData(DevIntrf_t * const pDevIntrf,
								 const uint8_t *pData, int DataLen)
{
	return UsbdBulkIntrfTxData(pDevIntrf, pData, DataLen);
}

static void UsbdBulkIntrfStopTx(DevIntrf_t * const)
{
}

static void UsbdBulkIntrfReset(DevIntrf_t * const pDevIntrf)
{
	UsbdBulkDevIntrf_t *pIntrf = UsbdBulkIntrfData(pDevIntrf);
	UsbdBulkIntrfResetTx(pIntrf);
}

static void UsbdBulkIntrfPowerOff(DevIntrf_t * const pDevIntrf)
{
	UsbdBulkIntrfDisable(pDevIntrf);
}

static void *UsbdBulkIntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return UsbdBulkIntrfData(pDevIntrf);
}

bool UsbdBulkIntrfInit(UsbdBulkDevIntrf_t *pIntrf,
					   const UsbdBulkIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr ||
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0)
	{
		return false;
	}

	hCFifo_t hRxFifo = CFifoInit(pCfg->pRxFifoMem,
								 (uint32_t)pCfg->RxFifoMemSize,
								 1, pCfg->bBlocking);
	hCFifo_t hTxFifo = CFifoInit(pCfg->pTxFifoMem,
								 (uint32_t)pCfg->TxFifoMemSize,
								 1, pCfg->bBlocking);

	if (hRxFifo == nullptr || hTxFifo == nullptr)
	{
		return false;
	}

	pIntrf->hRxFifo = hRxFifo;
	pIntrf->hTxFifo = hTxFifo;
	pIntrf->RxKick = pCfg->RxKick;
	pIntrf->TxReady = pCfg->TxReady;
	pIntrf->pContext = pCfg->pContext;
	pIntrf->TxEpAddr = 0U;
	pIntrf->pTxBuffer = nullptr;
	pIntrf->TxMps = 0U;
	pIntrf->TxZlpRequired = false;
	pIntrf->TxTailArmed = false;
	pIntrf->bEnabled = false;

	pIntrf->DevIntrf.pDevData = pIntrf;
	pIntrf->DevIntrf.IntPrio = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.bDma = false;
	pIntrf->DevIntrf.bIntEn = false;
	pIntrf->DevIntrf.Disable = UsbdBulkIntrfDisable;
	pIntrf->DevIntrf.Enable = UsbdBulkIntrfEnable;
	pIntrf->DevIntrf.GetRate = UsbdBulkIntrfGetRate;
	pIntrf->DevIntrf.SetRate = UsbdBulkIntrfSetRate;
	pIntrf->DevIntrf.StartRx = UsbdBulkIntrfStartRx;
	pIntrf->DevIntrf.RxData = UsbdBulkIntrfRxData;
	pIntrf->DevIntrf.StopRx = UsbdBulkIntrfStopRx;
	pIntrf->DevIntrf.StartTx = UsbdBulkIntrfStartTx;
	pIntrf->DevIntrf.TxData = UsbdBulkIntrfTxData;
	pIntrf->DevIntrf.TxSrData = UsbdBulkIntrfTxSrData;
	pIntrf->DevIntrf.StopTx = UsbdBulkIntrfStopTx;
	pIntrf->DevIntrf.Reset = UsbdBulkIntrfReset;
	pIntrf->DevIntrf.PowerOff = UsbdBulkIntrfPowerOff;
	pIntrf->DevIntrf.GetHandle = UsbdBulkIntrfGetHandle;

	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	DeviceIntrfEnable(&pIntrf->DevIntrf);

	return true;
}

int UsbdBulkIntrfPutRxData(UsbdBulkDevIntrf_t *pIntrf,
						   const uint8_t *pData, int DataLen)
{
	if (pIntrf == nullptr || pIntrf->hRxFifo == nullptr ||
		pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	const int requested = DataLen;
	uint32_t dropCnt = pIntrf->hRxFifo->DropCnt;
	uint32_t state = DisableInterrupt();
	int cnt = 0;

	while (DataLen > 0)
	{
		int length = DataLen;
		uint8_t *p = CFifoPutMultiple(pIntrf->hRxFifo, &length);
		if (p == nullptr || length <= 0)
		{
			break;
		}

		memcpy(p, pData, (size_t)length);
		pData += length;
		DataLen -= length;
		cnt += length;
	}

	EnableInterrupt(state);

	if ((cnt < requested || pIntrf->hRxFifo->DropCnt != dropCnt) &&
		pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_RX_FIFO_FULL,
							   nullptr, CFifoUsed(pIntrf->hRxFifo));
	}

	if (cnt > 0 && pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_RX_DATA,
							   nullptr, CFifoUsed(pIntrf->hRxFifo));
	}

	return cnt;
}

void UsbdBulkIntrfConfigTx(UsbdBulkDevIntrf_t *pIntrf,
						   uint8_t EpAddr, uint16_t Mps,
						   uint8_t *pBuffer)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->TxEpAddr = EpAddr;
	pIntrf->TxMps = Mps;
	pIntrf->pTxBuffer = pBuffer;
	UsbdBulkIntrfResetTx(pIntrf);
}

void UsbdBulkIntrfResetTx(UsbdBulkDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->TxZlpRequired = false;
	pIntrf->TxTailArmed = false;
	UsbdBulkIntrfSetTxIdle(pIntrf);
}

void UsbdBulkIntrfTxXferComplete(UsbdBulkDevIntrf_t *pIntrf,
								 uint16_t Length,
								 UsbdCtrlrXferResult_t Result)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr)
	{
		return;
	}

	if (Result != USBD_CTRLR_XFER_SUCCESS)
	{
		// Whatever is still queued goes out on the next write or SOF
		if (Result == USBD_CTRLR_XFER_FAILED)
		{
			UsbdBulkIntrfTxFailure(pIntrf, Length);
		}
		UsbdBulkIntrfSetTxIdle(pIntrf);
		return;
	}

	// Chain the next full packet right away, still owning the endpoint
	if (UsbdBulkIntrfCanTx(pIntrf) && UsbdBulkIntrfSubmit(pIntrf, false))
	{
		return;
	}

	// Submit failure already returned the endpoint to idle
	if (!UsbdBulkIntrfTxIdle(pIntrf))
	{
		UsbdBulkIntrfSetTxIdle(pIntrf);
	}

	// Less than one packet left, SOF sends that tail
	if (CFifoUsed(pIntrf->hTxFifo) > 0)
	{
		return;
	}

	// The stream stopped on a full packet, the host is still waiting for a
	// short packet to end its transfer. Only send it if nothing follows.
	if (Length == pIntrf->TxMps)
	{
		pIntrf->TxZlpRequired = true;
		return;
	}

	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_FIFO_EMPTY,
							   nullptr, 0);
	}
}

void UsbdBulkIntrfSof(UsbdBulkDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr ||
		!UsbdBulkIntrfTxIdle(pIntrf))
	{
		return;
	}

	if (CFifoUsed(pIntrf->hTxFifo) == 0 && !pIntrf->TxZlpRequired)
	{
		pIntrf->TxTailArmed = false;
		return;
	}

	// Give the producer one full frame to complete the packet
	if (!pIntrf->TxTailArmed)
	{
		pIntrf->TxTailArmed = true;
		return;
	}

	(void)UsbdBulkIntrfStartXfer(pIntrf, true);
}

bool UsbdBulkIntrfRequestToSend(UsbdBulkDevIntrf_t *pIntrf, int NbBytes)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr || NbBytes <= 0)
	{
		return false;
	}

	if (pIntrf->hTxFifo->bBlocking == false)
	{
		return true;
	}

	if (CFifoAvail(pIntrf->hTxFifo) < NbBytes)
	{
		(void)UsbdBulkIntrfStartXfer(pIntrf, false);
	}

	return CFifoAvail(pIntrf->hTxFifo) >= NbBytes;
}

int UsbdBulkIntrfRxUsed(UsbdBulkDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hRxFifo != nullptr ?
		   CFifoUsed(pIntrf->hRxFifo) : 0;
}

int UsbdBulkIntrfTxUsed(UsbdBulkDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hTxFifo != nullptr ?
		   CFifoUsed(pIntrf->hTxFifo) : 0;
}

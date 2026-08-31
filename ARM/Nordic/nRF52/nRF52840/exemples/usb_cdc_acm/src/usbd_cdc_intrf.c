/**-------------------------------------------------------------------------
@file	usbd_cdc_intrf.c

@brief	Nordic SDK CDC ACM DeviceIntrf adapter for the USB CDC benchmark

@author	Hoang Nguyen Hoan
@date	May 2, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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

#include "nrf_drv_usbd.h"
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"

#include "coredev/interrupt.h"
#include "usbd_cdc_intrf.h"

#define CDC_PACKET_SIZE			NRF_DRV_USBD_EPSIZE
#define CDC_DEFAULT_FIFO_SIZE	CFIFO_MEMSIZE(4 * CDC_PACKET_SIZE)

#define CDC_COMM_INTERFACE		0
#define CDC_COMM_EPIN			NRF_DRV_USBD_EPIN2
#define CDC_DATA_INTERFACE		1
#define CDC_DATA_EPIN			NRF_DRV_USBD_EPIN1
#define CDC_DATA_EPOUT			NRF_DRV_USBD_EPOUT1

static void UsbdCdcUserEvtHandler(app_usbd_class_inst_t const *pInst,
								  app_usbd_cdc_acm_user_event_t Event);

APP_USBD_CDC_ACM_GLOBAL_DEF(s_CdcAcm, UsbdCdcUserEvtHandler,
	CDC_COMM_INTERFACE, CDC_DATA_INTERFACE, CDC_COMM_EPIN,
	CDC_DATA_EPIN, CDC_DATA_EPOUT, APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

static uint8_t s_DefaultRxFifoMem[CDC_DEFAULT_FIFO_SIZE]
	__attribute__((aligned(4)));
static uint8_t s_DefaultTxFifoMem[CDC_DEFAULT_FIFO_SIZE]
	__attribute__((aligned(4)));
static UsbdCdcDevIntrf_t *s_pCdcIntrf;

static UsbdCdcDevIntrf_t *IntrfData(DevIntrf_t * const pDevIntrf)
{
	return pDevIntrf != NULL ? (UsbdCdcDevIntrf_t *)pDevIntrf->pDevData : NULL;
}

bool UsbdCdcIntrfIsPortOpen(const UsbdCdcDevIntrf_t *pIntrf)
{
	return pIntrf != NULL &&
		atomic_load_explicit(&pIntrf->bPortOpen, memory_order_relaxed);
}

static int PutRx(UsbdCdcDevIntrf_t *pIntrf, const uint8_t *pData, int DataLen)
{
	int count = 0;
	uint32_t state = DisableInterrupt();

	while (DataLen > 0)
	{
		int length = DataLen;
		uint8_t *p = CFifoPutMultiple(pIntrf->hRxFifo, &length);
		if (p == NULL || length <= 0)
			break;
		memcpy(p, pData, (size_t)length);
		pData += length;
		DataLen -= length;
		count += length;
	}

	EnableInterrupt(state);
	if (DataLen > 0)
	{
		pIntrf->RxDropCnt += (uint32_t)DataLen;
		if (pIntrf->DevIntrf.EvtCB != NULL)
			pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
				DEVINTRF_EVT_RX_FIFO_FULL, NULL, CFifoUsed(pIntrf->hRxFifo));
	}
	if (count > 0 && pIntrf->DevIntrf.EvtCB != NULL)
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
			DEVINTRF_EVT_RX_DATA, NULL, CFifoUsed(pIntrf->hRxFifo));
	return count;
}

static void RxArm(UsbdCdcDevIntrf_t *pIntrf)
{
	if (!atomic_load_explicit(&pIntrf->bEnabled, memory_order_relaxed) ||
		!atomic_load_explicit(&pIntrf->bPortOpen, memory_order_relaxed) ||
		atomic_load_explicit(&pIntrf->bRxPending, memory_order_relaxed))
		return;

	for (;;)
	{
		ret_code_t ret = app_usbd_cdc_acm_read_any(&s_CdcAcm,
			pIntrf->RxTransBuff, sizeof(pIntrf->RxTransBuff));
		if (ret == NRF_ERROR_IO_PENDING || ret == NRF_ERROR_BUSY)
		{
			atomic_store_explicit(&pIntrf->bRxPending, true, memory_order_relaxed);
			return;
		}
		if (ret != NRF_SUCCESS)
			return;

		size_t length = app_usbd_cdc_acm_rx_size(&s_CdcAcm);
		if (length == 0U)
			return;
		PutRx(pIntrf, pIntrf->RxTransBuff, (int)length);
	}
}

static int LoadTx(UsbdCdcDevIntrf_t *pIntrf)
{
	if (pIntrf->TxTransBuffLen > 0)
		return pIntrf->TxTransBuffLen;

	int left = sizeof(pIntrf->TxTransBuff);
	int count = 0;
	while (left > 0)
	{
		int length = left;
		uint8_t *p = CFifoGetMultiple(pIntrf->hTxFifo, &length);
		if (p == NULL || length <= 0)
			break;
		memcpy(&pIntrf->TxTransBuff[count], p, (size_t)length);
		count += length;
		left -= length;
	}
	pIntrf->TxTransBuffLen = count;
	return count;
}

static void TxKick(UsbdCdcDevIntrf_t *pIntrf);

static void TxIdle(UsbdCdcDevIntrf_t *pIntrf)
{
	atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true, memory_order_relaxed);

	/* Recheck after publishing idle so queued data cannot be stranded. */
	if (atomic_load_explicit(&pIntrf->bEnabled, memory_order_relaxed) &&
		atomic_load_explicit(&pIntrf->bPortOpen, memory_order_relaxed) &&
		CFifoUsed(pIntrf->hTxFifo) > 0 &&
		atomic_exchange_explicit(&pIntrf->DevIntrf.bTxReady, false,
			memory_order_relaxed))
	{
		TxKick(pIntrf);
		return;
	}

	if (pIntrf->DevIntrf.EvtCB != NULL)
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
			DEVINTRF_EVT_TX_FIFO_EMPTY, NULL, 0);
}

static void TxKick(UsbdCdcDevIntrf_t *pIntrf)
{
	if (!atomic_load_explicit(&pIntrf->bEnabled, memory_order_relaxed) ||
		!atomic_load_explicit(&pIntrf->bPortOpen, memory_order_relaxed))
	{
		atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
			memory_order_relaxed);
		return;
	}

	int length = LoadTx(pIntrf);
	if (length <= 0)
	{
		TxIdle(pIntrf);
		return;
	}

	ret_code_t ret = app_usbd_cdc_acm_write(&s_CdcAcm,
		pIntrf->TxTransBuff, (size_t)length);
	if (ret != NRF_SUCCESS)
	{
		/* The FIFO bytes are already staged. Retry this same transfer. */
		atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
			memory_order_relaxed);
	}
}

static void TxDone(UsbdCdcDevIntrf_t *pIntrf)
{
	pIntrf->TxTransBuffLen = 0;
	if (CFifoUsed(pIntrf->hTxFifo) > 0 &&
		atomic_load_explicit(&pIntrf->bEnabled, memory_order_relaxed) &&
		atomic_load_explicit(&pIntrf->bPortOpen, memory_order_relaxed))
		TxKick(pIntrf);
	else
		TxIdle(pIntrf);
}

static void UsbdCdcUserEvtHandler(app_usbd_class_inst_t const *pInst,
								  app_usbd_cdc_acm_user_event_t Event)
{
	(void)pInst;
	UsbdCdcDevIntrf_t *pIntrf = s_pCdcIntrf;
	if (pIntrf == NULL)
		return;

	switch (Event)
	{
		case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
			atomic_store_explicit(&pIntrf->bPortOpen, true, memory_order_relaxed);
			atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
				memory_order_relaxed);
			RxArm(pIntrf);
			if (CFifoUsed(pIntrf->hTxFifo) > 0 &&
				atomic_exchange_explicit(&pIntrf->DevIntrf.bTxReady, false,
					memory_order_relaxed))
				TxKick(pIntrf);
			break;

		case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
			atomic_store_explicit(&pIntrf->bPortOpen, false, memory_order_relaxed);
			atomic_store_explicit(&pIntrf->bRxPending, false, memory_order_relaxed);
			atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
				memory_order_relaxed);
			pIntrf->TxTransBuffLen = 0;
			break;

		case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
			TxDone(pIntrf);
			break;

		case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
			atomic_store_explicit(&pIntrf->bRxPending, false, memory_order_relaxed);
		{
			size_t length = app_usbd_cdc_acm_rx_size(&s_CdcAcm);
			if (length > 0U)
				PutRx(pIntrf, pIntrf->RxTransBuff, (int)length);
		}
			RxArm(pIntrf);
			break;

		default:
			break;
	}
}

static void IntrfDisable(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcDevIntrf_t *pIntrf = IntrfData(pDevIntrf);
	if (pIntrf != NULL)
		atomic_store_explicit(&pIntrf->bEnabled, false, memory_order_relaxed);
}

static void IntrfEnable(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcDevIntrf_t *pIntrf = IntrfData(pDevIntrf);
	if (pIntrf == NULL)
		return;
	atomic_store_explicit(&pIntrf->bEnabled, true, memory_order_relaxed);
	if (!UsbdCdcIntrfIsPortOpen(pIntrf))
		return;
	RxArm(pIntrf);
	if (CFifoUsed(pIntrf->hTxFifo) > 0 &&
		atomic_exchange_explicit(&pIntrf->DevIntrf.bTxReady, false,
			memory_order_relaxed))
		TxKick(pIntrf);
}

static uint32_t IntrfGetRate(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	return 0;
}

static uint32_t IntrfSetRate(DevIntrf_t * const pDevIntrf, uint32_t Rate)
{
	(void)pDevIntrf;
	(void)Rate;
	return 0;
}

static bool IntrfStart(DevIntrf_t * const pDevIntrf, uint32_t DevAddr)
{
	(void)pDevIntrf;
	(void)DevAddr;
	return true;
}

static int IntrfRxData(DevIntrf_t * const pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	UsbdCdcDevIntrf_t *pIntrf = IntrfData(pDevIntrf);
	if (pIntrf == NULL || pBuff == NULL || BuffLen <= 0)
		return 0;

	int count = 0;
	uint32_t state = DisableInterrupt();
	while (BuffLen > 0)
	{
		int length = BuffLen;
		uint8_t *p = CFifoGetMultiple(pIntrf->hRxFifo, &length);
		if (p == NULL || length <= 0)
			break;
		memcpy(pBuff, p, (size_t)length);
		pBuff += length;
		BuffLen -= length;
		count += length;
	}
	EnableInterrupt(state);
	return count;
}

static int IntrfTxData(DevIntrf_t * const pDevIntrf,
					   const uint8_t *pData, int DataLen)
{
	UsbdCdcDevIntrf_t *pIntrf = IntrfData(pDevIntrf);
	if (pIntrf == NULL || pData == NULL || DataLen <= 0)
		return 0;

	int requested = DataLen;
	int count = 0;

	/* CFifoPutMultiple publishes PutIdx before memcpy; block TX_DONE until both finish. */
	uint32_t state = DisableInterrupt();
	while (DataLen > 0)
	{
		int length = DataLen;
		uint8_t *p = CFifoPutMultiple(pIntrf->hTxFifo, &length);
		if (p == NULL || length <= 0)
			break;
		memcpy(p, pData, (size_t)length);
		pData += length;
		DataLen -= length;
		count += length;
	}
	EnableInterrupt(state);

	if (count < requested)
		pIntrf->TxDropCnt += (uint32_t)(requested - count);

	if (count > 0 && UsbdCdcIntrfIsPortOpen(pIntrf) &&
		atomic_load_explicit(&pIntrf->bEnabled, memory_order_relaxed) &&
		atomic_load_explicit(&pIntrf->DevIntrf.bTxReady, memory_order_relaxed))
	{
		atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, false,
			memory_order_relaxed);
		TxKick(pIntrf);
	}
	return count;
}

static int IntrfTxSrData(DevIntrf_t * const pDevIntrf,
						 const uint8_t *pData, int DataLen)
{
	return IntrfTxData(pDevIntrf, pData, DataLen);
}

static void IntrfStop(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void IntrfReset(DevIntrf_t * const pDevIntrf)
{
	UsbdCdcDevIntrf_t *pIntrf = IntrfData(pDevIntrf);
	if (pIntrf != NULL)
	{
		pIntrf->TxTransBuffLen = 0;
		atomic_store_explicit(&pIntrf->bRxPending, false, memory_order_relaxed);
		atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
			memory_order_relaxed);
	}
}

static void IntrfPowerOff(DevIntrf_t * const pDevIntrf)
{
	IntrfDisable(pDevIntrf);
}

static void *IntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return IntrfData(pDevIntrf);
}

bool UsbdCdcIntrfInit(UsbdCdcDevIntrf_t * const pIntrf,
					  const UsbdCdcIntrfCfg_t *pCfg)
{
	if (pIntrf == NULL || pCfg == NULL)
		return false;

	uint8_t *pRxMem = pCfg->pRxFifoMem != NULL ? pCfg->pRxFifoMem : s_DefaultRxFifoMem;
	uint32_t rxSize = pCfg->pRxFifoMem != NULL ?
		(uint32_t)pCfg->RxFifoMemSize : (uint32_t)sizeof(s_DefaultRxFifoMem);
	uint8_t *pTxMem = pCfg->pTxFifoMem != NULL ? pCfg->pTxFifoMem : s_DefaultTxFifoMem;
	uint32_t txSize = pCfg->pTxFifoMem != NULL ?
		(uint32_t)pCfg->TxFifoMemSize : (uint32_t)sizeof(s_DefaultTxFifoMem);

	pIntrf->hRxFifo = CFifoInit(pRxMem, rxSize, 1, pCfg->bBlocking);
	pIntrf->hTxFifo = CFifoInit(pTxMem, txSize, 1, pCfg->bBlocking);
	if (pIntrf->hRxFifo == NULL || pIntrf->hTxFifo == NULL)
		return false;

	pIntrf->RxDropCnt = 0;
	pIntrf->TxDropCnt = 0;
	pIntrf->TxTransBuffLen = 0;

	pIntrf->DevIntrf.pDevData = pIntrf;
	pIntrf->DevIntrf.IntPrio = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.bDma = true;
	pIntrf->DevIntrf.bIntEn = true;
	pIntrf->DevIntrf.Disable = IntrfDisable;
	pIntrf->DevIntrf.Enable = IntrfEnable;
	pIntrf->DevIntrf.GetRate = IntrfGetRate;
	pIntrf->DevIntrf.SetRate = IntrfSetRate;
	pIntrf->DevIntrf.StartRx = IntrfStart;
	pIntrf->DevIntrf.RxData = IntrfRxData;
	pIntrf->DevIntrf.StopRx = IntrfStop;
	pIntrf->DevIntrf.StartTx = IntrfStart;
	pIntrf->DevIntrf.TxData = IntrfTxData;
	pIntrf->DevIntrf.TxSrData = IntrfTxSrData;
	pIntrf->DevIntrf.StopTx = IntrfStop;
	pIntrf->DevIntrf.Reset = IntrfReset;
	pIntrf->DevIntrf.PowerOff = IntrfPowerOff;
	pIntrf->DevIntrf.GetHandle = IntrfGetHandle;

	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);
	atomic_store(&pIntrf->bPortOpen, false);
	atomic_store(&pIntrf->bEnabled, false);
	atomic_store(&pIntrf->bRxPending, false);

	s_pCdcIntrf = pIntrf;
	app_usbd_class_inst_t const *pClass = app_usbd_cdc_acm_class_inst_get(&s_CdcAcm);
	if (app_usbd_class_append(pClass) != NRF_SUCCESS)
	{
		s_pCdcIntrf = NULL;
		return false;
	}

	DeviceIntrfEnable(&pIntrf->DevIntrf);
	return true;
}

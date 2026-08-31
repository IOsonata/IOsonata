/**-------------------------------------------------------------------------
@file	usbd_cdc_intrf.c

@brief	Generic implementation of USBD CDC device interface

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

#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "nrf_drv_usbd.h"

#include "coredev/interrupt.h"
#include "usbd_cdc_intrf.h"

#define USBD_CDC_PACKET_SIZE		NRF_DRV_USBD_EPSIZE
#define USBD_CDC_CFIFO_MEMSIZE		CFIFO_MEMSIZE(4 * USBD_CDC_PACKET_SIZE)

#define CDC_ACM_COMM_INTERFACE		0
#define CDC_ACM_COMM_EPIN			NRF_DRV_USBD_EPIN2
#define CDC_ACM_DATA_INTERFACE		1
#define CDC_ACM_DATA_EPIN			NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT			NRF_DRV_USBD_EPOUT1

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *pInst,
									app_usbd_cdc_acm_user_event_t Event);

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
	cdc_acm_user_ev_handler,
	CDC_ACM_COMM_INTERFACE,
	CDC_ACM_DATA_INTERFACE,
	CDC_ACM_COMM_EPIN,
	CDC_ACM_DATA_EPIN,
	CDC_ACM_DATA_EPOUT,
	APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

static uint8_t s_UsbdCdcDevIntrfRxFifoMem[USBD_CDC_CFIFO_MEMSIZE]
	__attribute__((aligned(4)));
static uint8_t s_UsbdCdcDevIntrfTxFifoMem[USBD_CDC_CFIFO_MEMSIZE]
	__attribute__((aligned(4)));
static UsbdCdcDevIntrf_t *s_pIntrf;

static void UsbdCdcIntrfSend(UsbdCdcDevIntrf_t *pIntrf)
{
	int length = sizeof(pIntrf->TransBuff);
	uint32_t state = DisableInterrupt();
	uint8_t *p = CFifoGetMultiple(pIntrf->hTxFifo, &length);
	if (p != NULL && length > 0)
	{
		memcpy(pIntrf->TransBuff, p, (size_t)length);
	}
	EnableInterrupt(state);

	if (p == NULL || length <= 0)
	{
		atomic_store(&pIntrf->DevIntrf.bTxReady, true);
		return;
	}

	pIntrf->TransBuffLen = length;
	if (app_usbd_cdc_acm_write(&m_app_cdc_acm,
								pIntrf->TransBuff, (size_t)length) != NRF_SUCCESS)
	{
		pIntrf->TxDropCnt += (uint32_t)length;
		pIntrf->TransBuffLen = 0;
		atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	}
}

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *pInst,
									app_usbd_cdc_acm_user_event_t Event)
{
	(void)pInst;

	if (s_pIntrf == NULL)
	{
		return;
	}

	switch (Event)
	{
		case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
			atomic_store(&s_pIntrf->DevIntrf.bTxReady, true);
			break;

		case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
			atomic_store(&s_pIntrf->DevIntrf.bTxReady, false);
			break;

		case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
			s_pIntrf->TransBuffLen = 0;
			atomic_store(&s_pIntrf->DevIntrf.bTxReady, true);
			break;

		case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
		default:
			break;
	}
}

static void UsbdCdcIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void UsbdCdcIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static uint32_t UsbdCdcIntrfGetRate(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	return 0;
}

static uint32_t UsbdCdcIntrfSetRate(DevIntrf_t * const pDevIntrf, uint32_t Rate)
{
	(void)pDevIntrf;
	(void)Rate;
	return 0;
}

static bool UsbdCdcIntrfStart(DevIntrf_t * const pDevIntrf, uint32_t DevAddr)
{
	(void)pDevIntrf;
	(void)DevAddr;
	return true;
}

static int UsbdCdcIntrfRxData(DevIntrf_t * const pDevIntrf,
							  uint8_t *pBuff, int BuffLen)
{
	UsbdCdcDevIntrf_t *pIntrf = (UsbdCdcDevIntrf_t *)pDevIntrf->pDevData;
	int length = BuffLen;
	uint8_t *p = CFifoGetMultiple(pIntrf->hRxFifo, &length);

	if (p == NULL || length <= 0)
	{
		return 0;
	}

	memcpy(pBuff, p, (size_t)length);
	return length;
}

static int UsbdCdcIntrfTxData(DevIntrf_t * const pDevIntrf,
							  const uint8_t *pData, int DataLen)
{
	UsbdCdcDevIntrf_t *pIntrf = (UsbdCdcDevIntrf_t *)pDevIntrf->pDevData;

	/* If the previous transfer completed while the FIFO was full, free a packet first. */
	if (CFifoUsed(pIntrf->hTxFifo) > 0 &&
		atomic_exchange(&pIntrf->DevIntrf.bTxReady, false))
	{
		UsbdCdcIntrfSend(pIntrf);
	}

	int count = 0;
	uint32_t state = DisableInterrupt();

	while (DataLen > 0)
	{
		int length = DataLen;
		uint8_t *p = CFifoPutMultiple(pIntrf->hTxFifo, &length);
		if (p == NULL || length <= 0)
		{
			break;
		}

		memcpy(p, pData, (size_t)length);
		pData += length;
		DataLen -= length;
		count += length;
	}

	EnableInterrupt(state);

	if (count == 0)
	{
		pIntrf->TxDropCnt++;
		return 0;
	}

	if (atomic_exchange(&pIntrf->DevIntrf.bTxReady, false))
	{
		UsbdCdcIntrfSend(pIntrf);
	}

	return count;
}

static void UsbdCdcIntrfStop(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void UsbdCdcIntrfReset(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void UsbdCdcIntrfPowerOff(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void *UsbdCdcIntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return pDevIntrf->pDevData;
}

bool UsbdCdcIntrfInit(UsbdCdcDevIntrf_t *pIntrf,
					  const UsbdCdcIntrfCfg_t *pCfg)
{
	if (pIntrf == NULL || pCfg == NULL)
	{
		return false;
	}

	pIntrf->hRxFifo = CFifoInit(
		pCfg->pRxFifoMem != NULL ? pCfg->pRxFifoMem : s_UsbdCdcDevIntrfRxFifoMem,
		pCfg->pRxFifoMem != NULL ? (uint32_t)pCfg->RxFifoMemSize : sizeof(s_UsbdCdcDevIntrfRxFifoMem),
		1, pCfg->bBlocking);
	pIntrf->hTxFifo = CFifoInit(
		pCfg->pTxFifoMem != NULL ? pCfg->pTxFifoMem : s_UsbdCdcDevIntrfTxFifoMem,
		pCfg->pTxFifoMem != NULL ? (uint32_t)pCfg->TxFifoMemSize : sizeof(s_UsbdCdcDevIntrfTxFifoMem),
		1, pCfg->bBlocking);

	if (pIntrf->hRxFifo == NULL || pIntrf->hTxFifo == NULL)
	{
		return false;
	}

	pIntrf->DevIntrf.pDevData = pIntrf;
	pIntrf->DevIntrf.IntPrio = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.bDma = true;
	pIntrf->DevIntrf.bIntEn = true;
	pIntrf->DevIntrf.Disable = UsbdCdcIntrfDisable;
	pIntrf->DevIntrf.Enable = UsbdCdcIntrfEnable;
	pIntrf->DevIntrf.GetRate = UsbdCdcIntrfGetRate;
	pIntrf->DevIntrf.SetRate = UsbdCdcIntrfSetRate;
	pIntrf->DevIntrf.StartRx = UsbdCdcIntrfStart;
	pIntrf->DevIntrf.RxData = UsbdCdcIntrfRxData;
	pIntrf->DevIntrf.StopRx = UsbdCdcIntrfStop;
	pIntrf->DevIntrf.StartTx = UsbdCdcIntrfStart;
	pIntrf->DevIntrf.TxData = UsbdCdcIntrfTxData;
	pIntrf->DevIntrf.TxSrData = UsbdCdcIntrfTxData;
	pIntrf->DevIntrf.StopTx = UsbdCdcIntrfStop;
	pIntrf->DevIntrf.Reset = UsbdCdcIntrfReset;
	pIntrf->DevIntrf.PowerOff = UsbdCdcIntrfPowerOff;
	pIntrf->DevIntrf.GetHandle = UsbdCdcIntrfGetHandle;

	pIntrf->TransBuffLen = 0;
	pIntrf->RxDropCnt = 0;
	pIntrf->TxDropCnt = 0;
	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, false);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	s_pIntrf = pIntrf;

	app_usbd_class_inst_t const *pClass =
		app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
	if (app_usbd_class_append(pClass) != NRF_SUCCESS)
	{
		return false;
	}

	DeviceIntrfEnable(&pIntrf->DevIntrf);
	return true;
}

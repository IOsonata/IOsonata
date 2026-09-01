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
#include <memory.h>

#include "nrf_drv_usbd.h"
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"

#include "istddef.h"
#include "cfifo.h"
#include "usbd_cdc_intrf.h"
#include "coredev/interrupt.h"

#define USBD_CDC_PACKET_SIZE			NRF_DRV_USBD_EPSIZE
#define USBD_CDC_CFIFO_MEMSIZE			CFIFO_MEMSIZE(4 * USBD_CDC_PACKET_SIZE)

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

static uint8_t s_UsbdCdcDevIntrfRxFifoMem[USBD_CDC_CFIFO_MEMSIZE];
static uint8_t s_UsbdCdcDevIntrfTxFifoMem[USBD_CDC_CFIFO_MEMSIZE];
static UsbdCdcDevIntrf_t *s_pIntrf;

static void UsbdCdcIntrfSend(UsbdCdcDevIntrf_t *pIntrf)
{
	if (pIntrf->TransBuffLen == 0)
	{
		int l = sizeof(pIntrf->TransBuff);
		uint32_t state = DisableInterrupt();
		uint8_t *p = CFifoGetMultiple(pIntrf->hTxFifo, &l);

		if (p != NULL && l > 0)
		{
			memcpy(pIntrf->TransBuff, p, l);
			pIntrf->TransBuffLen = l;
		}
		EnableInterrupt(state);
	}

	if (pIntrf->TransBuffLen == 0)
	{
		atomic_store(&pIntrf->DevIntrf.bTxReady, true);
		return;
	}

	if (app_usbd_cdc_acm_write(&m_app_cdc_acm,
			pIntrf->TransBuff, pIntrf->TransBuffLen) != NRF_SUCCESS)
	{
		atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	}
}

void UsbdCdcIntrfTxKick(UsbdCdcDevIntrf_t *pIntrf)
{
	if (pIntrf == NULL)
	{
		return;
	}

	if (atomic_exchange(&pIntrf->DevIntrf.bTxReady, false))
	{
		UsbdCdcIntrfSend(pIntrf);
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
			UsbdCdcIntrfSend(s_pIntrf);
			break;

		default:
			break;
	}
}

void UsbdCdcIntrfDisable(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

void UsbdCdcIntrfEnable(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

uint32_t UsbdCdcIntrfGetRate(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
	return 0;
}

uint32_t UsbdCdcIntrfSetRate(DevIntrf_t *pDevIntrf, uint32_t Rate)
{
	(void)pDevIntrf;
	(void)Rate;
	return 0;
}

bool UsbdCdcIntrfStartRx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
{
	(void)pDevIntrf;
	(void)DevAddr;
	return true;
}

int UsbdCdcIntrfRxData(DevIntrf_t *pDevIntrf, uint8_t *pBuff, int Bufflen)
{
	UsbdCdcDevIntrf_t *intrf = (UsbdCdcDevIntrf_t*)pDevIntrf->pDevData;
	int cnt = 0;
	int l = Bufflen;
	uint8_t *p = CFifoGetMultiple(intrf->hRxFifo, &l);

	if (p != NULL)
	{
		cnt = min(Bufflen, l);
		memcpy(pBuff, p, cnt);
	}

	return cnt;
}

void UsbdCdcIntrfStopRx(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

bool UsbdCdcIntrfStartTx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
{
	(void)pDevIntrf;
	(void)DevAddr;
	return true;
}

int UsbdCdcIntrfTxData(DevIntrf_t *pDevIntrf, const uint8_t *pData, int Datalen)
{
	UsbdCdcDevIntrf_t *intrf = (UsbdCdcDevIntrf_t*)pDevIntrf->pDevData;
	int cnt = 0;

	while (Datalen > 0)
	{
		bool send = false;
		uint32_t state = DisableInterrupt();

		if (atomic_load(&pDevIntrf->bTxReady) &&
			intrf->TransBuffLen < sizeof(intrf->TransBuff))
		{
			int l = min(Datalen,
				(int)sizeof(intrf->TransBuff) - intrf->TransBuffLen);

			memcpy(&intrf->TransBuff[intrf->TransBuffLen], pData, l);
			intrf->TransBuffLen += l;
			Datalen -= l;
			pData += l;
			cnt += l;

			if (intrf->TransBuffLen == sizeof(intrf->TransBuff))
			{
				atomic_store(&pDevIntrf->bTxReady, false);
				send = true;
			}
			EnableInterrupt(state);

			if (send && app_usbd_cdc_acm_write(&m_app_cdc_acm,
					intrf->TransBuff, intrf->TransBuffLen) != NRF_SUCCESS)
			{
				atomic_store(&pDevIntrf->bTxReady, true);
			}
			continue;
		}

		int l = Datalen;
		uint8_t *p = CFifoPutMultiple(intrf->hTxFifo, &l);

		if (p != NULL)
		{
			memcpy(p, pData, l);
		}
		EnableInterrupt(state);

		if (p == NULL)
		{
			intrf->TxDropCnt++;
			break;
		}

		Datalen -= l;
		pData += l;
		cnt += l;
	}

	return cnt;
}

void UsbdCdcIntrfStopTx(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

void UsbdCdcIntrfReset(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

static void UsbdCdcIntrfPowerOff(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

static void *UsbdCdcIntrfGetHandle(DevIntrf_t *pDevIntrf)
{
	return pDevIntrf->pDevData;
}

bool UsbdCdcIntrfInit(UsbdCdcDevIntrf_t *pIntrf, const UsbdCdcIntrfCfg_t *pCfg)
{
	if (pIntrf == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->pRxFifoMem == NULL)
	{
		pIntrf->hRxFifo = CFifoInit(s_UsbdCdcDevIntrfRxFifoMem,
			USBD_CDC_CFIFO_MEMSIZE, 1, pCfg->bBlocking);
	}
	else
	{
		pIntrf->hRxFifo = CFifoInit(pCfg->pRxFifoMem,
			pCfg->RxFifoMemSize, 1, pCfg->bBlocking);
	}

	if (pCfg->pTxFifoMem == NULL)
	{
		pIntrf->hTxFifo = CFifoInit(s_UsbdCdcDevIntrfTxFifoMem,
			USBD_CDC_CFIFO_MEMSIZE, 1, pCfg->bBlocking);
	}
	else
	{
		pIntrf->hTxFifo = CFifoInit(pCfg->pTxFifoMem,
			pCfg->TxFifoMemSize, 1, pCfg->bBlocking);
	}

	if (pIntrf->hRxFifo == NULL || pIntrf->hTxFifo == NULL)
	{
		return false;
	}

	app_usbd_class_inst_t const *class_cdc_acm =
		app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
	if (app_usbd_class_append(class_cdc_acm) != NRF_SUCCESS)
	{
		return false;
	}

	pIntrf->DevIntrf.pDevData = (void*)pIntrf;
	pIntrf->DevIntrf.IntPrio = 0;
	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.bDma = true;
	pIntrf->DevIntrf.bIntEn = true;
	pIntrf->DevIntrf.Enable = UsbdCdcIntrfEnable;
	pIntrf->DevIntrf.Disable = UsbdCdcIntrfDisable;
	pIntrf->DevIntrf.GetRate = UsbdCdcIntrfGetRate;
	pIntrf->DevIntrf.SetRate = UsbdCdcIntrfSetRate;
	pIntrf->DevIntrf.StartRx = UsbdCdcIntrfStartRx;
	pIntrf->DevIntrf.RxData = UsbdCdcIntrfRxData;
	pIntrf->DevIntrf.StopRx = UsbdCdcIntrfStopRx;
	pIntrf->DevIntrf.StartTx = UsbdCdcIntrfStartTx;
	pIntrf->DevIntrf.TxData = UsbdCdcIntrfTxData;
	pIntrf->DevIntrf.TxSrData = UsbdCdcIntrfTxData;
	pIntrf->DevIntrf.StopTx = UsbdCdcIntrfStopTx;
	pIntrf->DevIntrf.Reset = UsbdCdcIntrfReset;
	pIntrf->DevIntrf.PowerOff = UsbdCdcIntrfPowerOff;
	pIntrf->DevIntrf.GetHandle = UsbdCdcIntrfGetHandle;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;

	pIntrf->TransBuffLen = 0;
	pIntrf->RxDropCnt = 0;
	pIntrf->TxDropCnt = 0;
	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, false);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	s_pIntrf = pIntrf;
	DeviceIntrfEnable(&pIntrf->DevIntrf);

	return true;
}

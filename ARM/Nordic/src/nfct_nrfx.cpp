/**-------------------------------------------------------------------------
@file	nfct_nrfx.cpp

@brief	Nordic NFCT frame transport implementation, all NFCT equipped nRF series

DeviceIntrf over the NFCT peripheral in target mode using the nrfx_nfct
driver. Frames only. Activation, anticollision, CRC and reply timing run in
hardware. Received frames are forwarded to the frame callback from the NFCT
interrupt. Transmission maps DeviceIntrfTx to nrfx_nfct_tx with the window
grid delay mode so the response lands in the hardware timed reply window.

@author	Hoang Nguyen Hoan
@date	Jul. 5, 2026

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

#include "nrfx_nfct.h"
#include "coredev/interrupt.h"
#include "nfct_nrfx.h"

// The NFCT peripheral is a singleton and the nrfx callback has no context
// argument. The active transport instance is kept here.
static NfctIntrfDev_t *s_pNfctDev = nullptr;

static void NfctIntrfArmRx(NfctIntrfDev_t * const pDev)
{
	nrfx_nfct_data_desc_t desc;

	desc.p_data = pDev->RxFrame;
	desc.data_size = sizeof(pDev->RxFrame);

	nrfx_nfct_rx(&desc);
}

static void NfctIntrfNrfxHandler(nrfx_nfct_evt_t const * pEvt)
{
	NfctIntrfDev_t *pDev = s_pNfctDev;

	if (pDev == nullptr)
	{
		return;
	}

	switch (pEvt->evt_id)
	{
		case NRFX_NFCT_EVT_FIELD_DETECTED:
			pDev->bFieldOn = true;
			nrfx_nfct_state_force(NRFX_NFCT_STATE_ACTIVATED);
			if (pDev->pEvtCB)
			{
				pDev->pEvtCB(pDev, NFCT_INTRF_EVT_FIELD_ON);
			}
			break;

		case NRFX_NFCT_EVT_FIELD_LOST:
			pDev->bFieldOn = false;
			pDev->bSelected = false;
			nrfx_nfct_state_force(NRFX_NFCT_STATE_SENSING);
			if (pDev->pEvtCB)
			{
				pDev->pEvtCB(pDev, NFCT_INTRF_EVT_FIELD_OFF);
			}
			break;

		case NRFX_NFCT_EVT_SELECTED:
			// Hardware anticollision done. Arm reception for the first frame.
			pDev->bSelected = true;
			NfctIntrfArmRx(pDev);
			if (pDev->pEvtCB)
			{
				pDev->pEvtCB(pDev, NFCT_INTRF_EVT_SELECTED);
			}
			break;

		case NRFX_NFCT_EVT_RX_FRAMEEND:
			pDev->RxLen = pEvt->params.rx_frameend.rx_data.data_size;
			pDev->bTxPending = false;
			if (pDev->pFrameCB && pDev->RxLen > 0)
			{
				// The callback normally replies through DeviceIntrfTx, which
				// sets bTxPending. Reception is re armed at TX_FRAMEEND then.
				pDev->pFrameCB(pDev, pDev->RxFrame, pDev->RxLen);
			}
			if (pDev->bTxPending == false)
			{
				// No response was queued for this frame, listen again.
				NfctIntrfArmRx(pDev);
			}
			break;

		case NRFX_NFCT_EVT_TX_FRAMEEND:
			// Response sent, listen for the next reader frame.
			pDev->bTxPending = false;
			NfctIntrfArmRx(pDev);
			break;

		case NRFX_NFCT_EVT_ERROR:
			if (pDev->pEvtCB)
			{
				pDev->pEvtCB(pDev, NFCT_INTRF_EVT_ERROR);
			}
			NfctIntrfArmRx(pDev);
			break;

		default:
			break;
	}
}

// DevIntrf vtable

static void NfctIntrfDisable(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;

	nrfx_nfct_disable();
}

static void NfctIntrfEnable(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;

	nrfx_nfct_enable();
}

static uint32_t NfctIntrfGetRate(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;

	// NFC-A 106 kbps fixed
	return 106000;
}

static uint32_t NfctIntrfSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	(void)pIntrf;
	(void)Rate;

	return 106000;
}

static bool NfctIntrfStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)DevAddr;

	NfctIntrfDev_t *pDev = (NfctIntrfDev_t *)pIntrf->pDevData;

	return pDev->bSelected;
}

// Reception is event driven. RxData returns the last received frame when
// called from the frame callback context.
static int NfctIntrfRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	NfctIntrfDev_t *pDev = (NfctIntrfDev_t *)pIntrf->pDevData;

	if (pBuff == nullptr || BuffLen <= 0 || pDev->RxLen <= 0)
	{
		return 0;
	}

	int l = pDev->RxLen < BuffLen ? pDev->RxLen : BuffLen;

	memcpy(pBuff, pDev->RxFrame, l);
	pDev->RxLen = 0;

	return l;
}

static void NfctIntrfStopRx(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
}

static bool NfctIntrfStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)DevAddr;

	NfctIntrfDev_t *pDev = (NfctIntrfDev_t *)pIntrf->pDevData;

	return pDev->bFieldOn;
}

static int NfctIntrfTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	NfctIntrfDev_t *pDev = (NfctIntrfDev_t *)pIntrf->pDevData;

	if (pData == nullptr || DataLen <= 0 || DataLen > (int)sizeof(pDev->TxFrame))
	{
		return 0;
	}

	// The frame goes out through EasyDMA, keep it in the transport buffer so
	// the caller buffer may be reused immediately.
	memcpy(pDev->TxFrame, pData, DataLen);

	nrfx_nfct_data_desc_t desc;

	desc.p_data = pDev->TxFrame;
	desc.data_size = DataLen;

	if (nrfx_nfct_tx(&desc, NRF_NFCT_FRAME_DELAY_MODE_WINDOWGRID) != 0)
	{
		return 0;
	}

	pDev->bTxPending = true;

	return DataLen;
}

static void NfctIntrfStopTx(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
}

static void NfctIntrfReset(DevIntrf_t * const pIntrf)
{
	NfctIntrfDev_t *pDev = (NfctIntrfDev_t *)pIntrf->pDevData;

	pDev->bSelected = false;
	pDev->bTxPending = false;
	pDev->RxLen = 0;

	nrfx_nfct_disable();
	nrfx_nfct_enable();
}

static void *NfctIntrfGetHandle(DevIntrf_t * const pIntrf)
{
	return pIntrf ? pIntrf->pDevData : nullptr;
}

bool NfctIntrfInit(NfctIntrfDev_t * const pDev, const NfctIntrfCfg_t * const pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pFrameCB == nullptr)
	{
		return false;
	}

	memset(pDev, 0, sizeof(NfctIntrfDev_t));

	pDev->pFrameCB = pCfg->pFrameCB;
	pDev->pEvtCB = pCfg->pEvtCB;
	pDev->pCtx = pCfg->pCtx;

	s_pNfctDev = pDev;

	nrfx_nfct_config_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.rxtx_int_mask = NRFX_NFCT_EVT_FIELD_DETECTED | NRFX_NFCT_EVT_FIELD_LOST |
						NRFX_NFCT_EVT_SELECTED | NRFX_NFCT_EVT_RX_FRAMEEND |
						NRFX_NFCT_EVT_TX_FRAMEEND | NRFX_NFCT_EVT_ERROR;
	cfg.cb = NfctIntrfNrfxHandler;
	cfg.irq_priority = IRQ_PRIO_NORMAL;

	if (nrfx_nfct_init(&cfg) != 0)
	{
		return false;
	}

	if (pCfg->IdLen == 4 || pCfg->IdLen == 7 || pCfg->IdLen == 10)
	{
		nrfx_nfct_param_t param;

		param.id = NRFX_NFCT_PARAM_ID_NFCID1;
		param.data.nfcid1.p_id = pCfg->NfcId1;
		param.data.nfcid1.id_size = pCfg->IdLen;

		if (nrfx_nfct_parameter_set(&param) != 0)
		{
			return false;
		}
	}

	if (pCfg->SelRes != 0)
	{
		nrfx_nfct_param_t param;

		param.id = NRFX_NFCT_PARAM_ID_SEL_RES;
		param.data.sel_res_protocol = pCfg->SelRes;

		if (nrfx_nfct_parameter_set(&param) != 0)
		{
			return false;
		}
	}

	nrfx_nfct_autocolres_enable();

	DevIntrf_t *p = &pDev->DevIntrf;

	p->pDevData = pDev;
	p->Type = DEVINTRF_TYPE_UNKOWN;
	p->bDma = true;
	p->bIntEn = true;
	p->MaxRetry = 0;
	p->Disable = NfctIntrfDisable;
	p->Enable = NfctIntrfEnable;
	p->GetRate = NfctIntrfGetRate;
	p->SetRate = NfctIntrfSetRate;
	p->StartRx = NfctIntrfStartRx;
	p->RxData = NfctIntrfRxData;
	p->StopRx = NfctIntrfStopRx;
	p->StartTx = NfctIntrfStartTx;
	p->TxData = NfctIntrfTxData;
	p->TxSrData = NfctIntrfTxData;
	p->StopTx = NfctIntrfStopTx;
	p->Reset = NfctIntrfReset;
	p->PowerOff = NfctIntrfDisable;
	p->GetHandle = NfctIntrfGetHandle;
	p->EvtCB = nullptr;
	p->MaxTrxLen = NFCT_INTRF_FRAME_MAX;
	atomic_flag_clear(&p->bBusy);
	atomic_store(&p->EnCnt, 0);
	atomic_store(&p->bTxReady, true);
	atomic_store(&p->bNoStop, false);

	return true;
}

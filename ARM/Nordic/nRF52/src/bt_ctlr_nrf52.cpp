/**-------------------------------------------------------------------------
@file	bt_ctlr_nrf52.cpp

@brief	Generic implementation of Bluetooth controller device.

Implementation of Bluetooth controller using Nordic nRF5_SDK

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

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_gatts.h"

#include "cfifo.h"
#include "bluetooth/bt_ctlr.h"

#define BTCTLR_FIFO_MEM_SIZE			BTCTLR_PKT_CFIFO_TOTAL_MEMSIZE(4, BT_CTLR_MTU_MAX)
alignas(4) static uint8_t s_BtCtlrRxFifoMem[BTCTLR_FIFO_MEM_SIZE];

static void nRF5CtlrDisable(DevIntrf_t * const pDev)
{
}

static void nRF5CtlrEnable(DevIntrf_t * const pDev)
{
}

static uint32_t nRF5CtlrGetRate(DevIntrf_t * const pDev)
{
	BtCtlrDev_t *p = (BtCtlrDev_t *)pDev->pDevData;

	return p->Rate;
}

static uint32_t nRF5CtlrSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	BtCtlrDev_t *p = (BtCtlrDev_t *)pDev->pDevData;

	if (Rate < 200000)
	{
		// Coded phy 125Kbps
		Rate = 125000;
	}
	if (Rate < 750000)
	{
		// Coded phy 500Kbps
		Rate = 500000;
	}
	else if (Rate < 1500000)
	{
		// 1Mbps phy
		Rate = 1000000;
	}
	else
	{
		// 2Mbps phy
		Rate = 2000000;
	}

	p->Rate = Rate;

	return 0;
}

static bool nRF5CtlrStartRx(DevIntrf_t * const pSerDev, uint32_t DevAddr)
{
	return true;
}

static int nRF5CtlrRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	return 0;
}

static void nRF5CtlrStopRx(DevIntrf_t * const pDev)
{
}

static bool nRF5CtlrStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	BtCtlrDev_t *p = (BtCtlrDev_t *)pDev->pDevData;

	p->ConnHdl = DevAddr >> 16;
	p->ValHdl = DevAddr & 0xFFFF;

	return true;
}

static int nRF5CtlrTxData(DevIntrf_t * const pDev, uint8_t *pData, int DataLen)
{
	int cnt = 0;
	BtCtlrDev_t *p = (BtCtlrDev_t *)pDev->pDevData;

	ble_gatts_hvx_params_t params;

	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_NOTIFICATION;
	params.handle = p->ValHdl;//.value_handle;
	params.p_data = pData;
	params.p_len = (uint16_t *)&DataLen;

	uint32_t err = sd_ble_gatts_hvx(p->ConnHdl, &params);
	if (err == NRF_SUCCESS)
	{
		cnt = DataLen;
	}

	return cnt;
}

static void nRF5CtlrStopTx(DevIntrf_t * const pDev)
{
}

void nRF5CtlrPowerOff(DevIntrf_t * const pDev)
{
}

bool BtCtlrInit(BtCtlrDev_t * const pDev, const BtCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->PacketSize <= 0)
	{
		pDev->PacketSize = BT_CTLR_MTU_MAX + BTCTLR_PKTHDR_LEN;
	}
	else
	{
		pDev->PacketSize = pCfg->PacketSize + BTCTLR_PKTHDR_LEN;
	}

	if (pCfg->pRxFifoMem == NULL)// || pCfg->pTxFifoMem == NULL)
	{
		pDev->hRxFifo = CFifoInit(s_BtCtlrRxFifoMem, BTCTLR_FIFO_MEM_SIZE, pDev->PacketSize, true);
		//pDev->hTxFifo = CFifoInit(s_BtDevIntrfRxFifoMem, BTINTRF_CFIFO_SIZE, pIntrf->PacketSize, pCfg->bBlocking);
	}
	else
	{
		//pDev->hRxFifo = CFifoInit(pCfg->pRxFifoMem, pCfg->RxFifoMemSize, pIntrf->PacketSize, pCfg->bBlocking);
		//pDev->hTxFifo = CFifoInit(pCfg->pTxFifoMem, pCfg->TxFifoMemSize, pIntrf->PacketSize, pCfg->bBlocking);
	}

	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_BT;
	pDev->DevIntrf.Disable = nRF5CtlrDisable;
	pDev->DevIntrf.Enable = nRF5CtlrEnable;
	pDev->DevIntrf.GetRate = nRF5CtlrGetRate;
	pDev->DevIntrf.SetRate = nRF5CtlrSetRate;
	pDev->DevIntrf.StartRx = nRF5CtlrStartRx;
	pDev->DevIntrf.RxData = nRF5CtlrRxData;
	pDev->DevIntrf.StopRx = nRF5CtlrStopRx;
	pDev->DevIntrf.StartTx = nRF5CtlrStartTx;
	pDev->DevIntrf.TxData = nRF5CtlrTxData;
	pDev->DevIntrf.StopTx = nRF5CtlrStopTx;
	pDev->DevIntrf.MaxRetry = 1;
	pDev->DevIntrf.PowerOff = nRF5CtlrPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.EvtCB = pCfg->EvtHandler;
	pDev->AttHandler = pCfg->AttHandler;
	pDev->SmpHandler = pCfg->SmpHandler;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void BtCtlrDispatch(ble_evt_t const * p_ble_evt, void *p_context)
{
#if 0
 //   uint16_t role = ble_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);

    if (s_BtDevnRF5.Role & BTDEV_ROLE_PERIPHERAL)
    {
//    	BtGattSrvc_t *p = s_BtDevnRF5.pSrvc;

//    	while (p)
    	{
    		BtGattEvtHandler((uint32_t)p_ble_evt, p_context);

//    		p = p->pNext;
    	}
    }
    on_ble_evt(p_ble_evt);
    if ((role == BLE_GAP_ROLE_CENTRAL) || s_BtDevnRF5.Role & (BTDEV_ROLE_CENTRAL | BTDEV_ROLE_OBSERVER))
    {
#if 0
    	switch (p_ble_evt->header.evt_id)
        {
            case BLE_GAP_EVT_TIMEOUT:
            {
                const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

                ble_gap_evt_timeout_t const * p_timeout = &p_gap_evt->params.timeout;

                if (p_timeout->src == BLE_GAP_TIMEOUT_SRC_SCAN)
                {
                    g_BleAppData.bScan = false;
                }
            }
            break;
        }
#endif
    	//BtAppCentralEvtHandler((ble_evt_t *)p_ble_evt);
    }
#endif
}

#define BTCTLR_OBSERVER_PRIO           1

NRF_SDH_BLE_OBSERVER(g_BtCtlrObserver, BTCTLR_OBSERVER_PRIO, BtCtlrDispatch, NULL);



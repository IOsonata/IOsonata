/**-------------------------------------------------------------------------
@file	bt_hci_ctlr_sdc.cpp

@brief	Generic implementation of Bluetooth controller device.

Implementation of Bluetooth controller using Nordic Softdevice Controller

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

#include "sdc_hci.h"

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_ctlr.h"

#define BTHCICTLR_FIFO_MEM_SIZE			BTHCICTLR_PKT_CFIFO_TOTAL_MEMSIZE(4, BT_HCI_CTLR_MTU_MAX)
alignas(4) static uint8_t s_BtHciCtlrRxFifoMem[BTHCICTLR_FIFO_MEM_SIZE];

static inline size_t BtHciCtlrSendData(BtHciCtlrDev_t * const pDev, void *pData, size_t Len) {
	return sdc_hci_data_put((uint8_t*)pData) == 0 ? Len : 0;
}

size_t BtHciCtlrReceive(BtHciCtlrDev_t * const pDev, uint16_t Hdl, void * const pData, size_t Len)
{
	BtHciCtlrPkt_t *p = (BtHciCtlrPkt_t*)CFifoPut(pDev->hRxFifo);
	if (p)
	{
		p->Len = Len;
		p->ValHdl = Hdl;
		p->Off = 0;

		size_t l = min(Len, pDev->PacketSize);
		memcpy(p->Data, pData, l);

		return l;
	}

	return 0;
}

static void SdcCtlrDisable(DevIntrf_t * const pDev)
{
}

static void SdcCtlrEnable(DevIntrf_t * const pDev)
{
}

static uint32_t SdcCtlrGetRate(DevIntrf_t * const pDev)
{
	uint32_t rate = 0;

	return rate;
}

static uint32_t SdcCtlrSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	return 0;
}

static bool SdcCtlrStartRx(DevIntrf_t * const pSerDev, uint32_t DevAddr)
{
	return true;
}

static int SdcCtlrRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	return 0;
}

static void SdcCtlrStopRx(DevIntrf_t * const pDev)
{
}

static bool SdcCtlrStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	BtHciCtlrDev_t *p = (BtHciCtlrDev_t *)pDev->pDevData;

	p->ConnHdl = DevAddr >> 16;
	p->ValHdl = DevAddr & 0xFFFF;

	return true;
}

static int SdcCtlrTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	// Forward a formed HCI ACL packet to the SDC controller. The host assembles
	// the packet above the HCI line; the controller only transmits it.
	return sdc_hci_data_put((uint8_t*)pData) == 0 ? DataLen : 0;
}

static void SdcCtlrStopTx(DevIntrf_t * const pDev)
{
}

void SdcCtlrPowerOff(DevIntrf_t * const pDev)
{
}

bool BtHciCtlrInit(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->PacketSize <= 0)
	{
		pDev->PacketSize = BT_HCI_CTLR_MTU_MAX + BTHCICTLR_PKTHDR_LEN;
	}
	else
	{
		pDev->PacketSize = pCfg->PacketSize + BTHCICTLR_PKTHDR_LEN;
	}

	if (pCfg->pRxFifoMem == NULL)// || pCfg->pTxFifoMem == NULL)
	{
		pDev->hRxFifo = CFifoInit(s_BtHciCtlrRxFifoMem, BTHCICTLR_FIFO_MEM_SIZE, pDev->PacketSize, true);
		//pDev->hTxFifo = CFifoInit(s_BtDevIntrfRxFifoMem, BTINTRF_CFIFO_SIZE, pIntrf->PacketSize, pCfg->bBlocking);
	}
	else
	{
		//pDev->hRxFifo = CFifoInit(pCfg->pRxFifoMem, pCfg->RxFifoMemSize, pIntrf->PacketSize, pCfg->bBlocking);
		//pDev->hTxFifo = CFifoInit(pCfg->pTxFifoMem, pCfg->TxFifoMemSize, pIntrf->PacketSize, pCfg->bBlocking);
	}

	pDev->DevIntrf.Type = DEVINTRF_TYPE_BT;
	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.Disable = SdcCtlrDisable;
	pDev->DevIntrf.Enable = SdcCtlrEnable;
	pDev->DevIntrf.GetRate = SdcCtlrGetRate;
	pDev->DevIntrf.SetRate = SdcCtlrSetRate;
	pDev->DevIntrf.StartRx = SdcCtlrStartRx;
	pDev->DevIntrf.RxData = SdcCtlrRxData;
	pDev->DevIntrf.StopRx = SdcCtlrStopRx;
	pDev->DevIntrf.StartTx = SdcCtlrStartTx;
	pDev->DevIntrf.TxData = SdcCtlrTxData;
	pDev->DevIntrf.StopTx = SdcCtlrStopTx;
	pDev->DevIntrf.MaxRetry = 1;
	pDev->DevIntrf.PowerOff = SdcCtlrPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.EvtCB = pCfg->EvtHandler;
	pDev->Receive = BtHciCtlrReceive;
	pDev->Send = BtHciCtlrSendData;
	pDev->RxHandler = pCfg->RxHandler;
	pDev->OnWake = pCfg->OnWake;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

void BtHciCtlrProcess(BtHciCtlrDev_t * const pDev)
{
	if (pDev == nullptr || pDev->RxHandler == nullptr)
	{
		return;
	}

	uint8_t buf[HCI_MSG_BUFFER_MAX_SIZE];
	sdc_hci_msg_type_t mtype;

	// Drain every queued message. The controller can queue several at once,
	// for example a command completion followed by an Encryption Change event
	// during pairing; stopping after one strands the later packets.
	while (sdc_hci_get(buf, (uint8_t*)&mtype) == 0)
	{
		pDev->RxHandler(pDev, mtype == SDC_HCI_MSG_TYPE_EVT, buf);
	}
}


// ---- SDC controller bring-up (relocated from bt_app_sdc) ----

#include <stdlib.h>
#include "nrf.h"
#include "sdc.h"
#include "sdc_soc.h"
#include "sdc_hci_vs.h"
#include "nrf_mpsl.h"
#include "bluetooth/bt_gap.h"
#include "coredev/system_core_clock.h"

#if 0
/******** For DEBUG ************/
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
/*******************************/
#else
#define DEBUG_PRINTF(...)
#endif

static BtHciCtlrDev_t *s_pBtHciCtlrSdc = nullptr;

extern "C" size_t BtHciCtlrSdcSend(void *pData, size_t Len)
{
	if (s_pBtHciCtlrSdc == nullptr || s_pBtHciCtlrSdc->Send == nullptr)
	{
		return 0;
	}

	return s_pBtHciCtlrSdc->Send(s_pBtHciCtlrSdc, pData, Len);
}

alignas(8) static uint8_t s_BtStackSdcMemPool[10000];

static void BtStackSdcAssert(const char * file, const uint32_t line)
{
	DEBUG_PRINTF("SDC Fault: %s, %d\n", file, line);
	while(1);
}

static uint8_t BtStackRandPrioLowGet(uint8_t *pBuff, uint8_t Len)
{
	DEBUG_PRINTF("BtStackRandPrioLowGet\r\n");
	for (int i = 0; i < Len; i++)
	{
		pBuff[i] = rand();
	}

	return Len;
}

static uint8_t BtStackRandPrioHighGet(uint8_t *pBuff, uint8_t Len)
{
	return BtStackRandPrioLowGet(pBuff, Len);
}

static void BtStackRandPrioLowGetBlocking(uint8_t *pBuff, uint8_t Len)
{
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
	NRF_CRACEN_Type *reg = NRF_CRACEN_S;

	BtStackRandPrioLowGet(pBuff, Len);

#else
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	NRF_RNG_Type *reg = NRF_RNG_NS;
#else
	NRF_RNG_Type *reg = NRF_RNG_S;
#endif
#else
	NRF_RNG_Type *reg = NRF_RNG;
#endif

	reg->CONFIG = RNG_CONFIG_DERCEN_Enabled;

	reg->EVENTS_VALRDY = 0;
	reg->TASKS_START = 1;

	for (int i = 0; i < Len; i++)
	{
		while (reg->EVENTS_VALRDY == 0);

		pBuff[i] = reg->VALUE;
		reg->EVENTS_VALRDY = 0;		// clear so the next VALUE is fresh entropy
	}

	reg->TASKS_STOP = 1;

	reg->CONFIG = RNG_CONFIG_DERCEN_Disabled;
#endif
}

static void BtStackSdcCB()
{
	if (s_pBtHciCtlrSdc != nullptr && s_pBtHciCtlrSdc->OnWake != nullptr)
	{
		s_pBtHciCtlrSdc->OnWake();
	}

	BtHciCtlrProcess(s_pBtHciCtlrSdc);
}

bool BtHciCtlrEnable(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	BtHciCtlrInit(pDev, pCfg);
	s_pBtHciCtlrSdc = pDev;

	int32_t res = sdc_init(BtStackSdcAssert);

	//sdc_hci_cmd_cb_reset();

	sdc_rand_source_t rand_functions = {
		//.rand_prio_low_get = BtStackRandPrioLowGet,
		//.rand_prio_high_get = BtStackRandPrioHighGet,
		.rand_poll = BtStackRandPrioLowGetBlocking
	};

	res = sdc_rand_source_register(&rand_functions);

	sdc_support_le_2m_phy();
	sdc_support_le_coded_phy();
	//sdc_support_le_power_control();

	if (pCfg->Role & (BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_BROADCASTER))
	{
		// Config for peripheral role
		sdc_support_adv();
		sdc_support_ext_adv();
		sdc_support_le_periodic_adv();
		sdc_support_le_periodic_sync();
		sdc_support_peripheral();
		sdc_support_dle_peripheral();
		sdc_support_phy_update_peripheral();
		sdc_support_le_power_control_peripheral();
		sdc_support_le_conn_cte_rsp_peripheral();
	}
	if (pCfg->Role & (BT_GAP_ROLE_CENTRAL | BT_GAP_ROLE_OBSERVER))
	{
		// Config for central role
		sdc_support_scan();
		sdc_support_ext_scan();
		sdc_support_central();
		sdc_support_ext_central();
		sdc_support_dle_central();
		sdc_support_phy_update_central();
		sdc_support_le_power_control_central();
		sdc_support_le_conn_cte_rsp_central();
	}

    uint32_t ram = 0;
	sdc_cfg_t cfg;

	// Reserve max always. It seems sdc lib is not capable of changing it in runtime
	cfg.buffer_cfg.rx_packet_size = pCfg->MaxDataLen;
	cfg.buffer_cfg.tx_packet_size = pCfg->MaxDataLen;
	cfg.buffer_cfg.rx_packet_count = pCfg->RxPktCount;
	cfg.buffer_cfg.tx_packet_count = pCfg->TxPktCount;

	DEBUG_PRINTF("sdc_cfg_set\r\n");

	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_BUFFER_CFG,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}


	sdc_hci_cmd_vs_event_length_set_t evlen = {
		.event_length_us = 7500,
	};
	sdc_hci_cmd_vs_event_length_set(&evlen);

	/*
	cfg.event_length.event_length_us = 7500;
	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_EVENT_LENGTH,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}
*/
	if (pCfg->Role & (BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_BROADCASTER))
	{
		// Config for peripheral role
		cfg.peripheral_count.count = pCfg->PeriLinkCount;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
					       	  SDC_CFG_TYPE_PERIPHERAL_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}

		cfg.adv_count.count = 1;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}

		cfg.adv_buffer_cfg.max_adv_data = 255;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_BUFFER_CFG,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}
	}

	if (pCfg->Role & (BT_GAP_ROLE_CENTRAL | BT_GAP_ROLE_OBSERVER))
	{
		// Config for central role
		cfg.central_count.count = pCfg->CentLinkCount;
		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
					       	  SDC_CFG_TYPE_CENTRAL_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}


		cfg.scan_buffer_cfg.count = 10;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
						  SDC_CFG_TYPE_SCAN_BUFFER_CFG,
						  &cfg);
		if (ram < 0)
		{
			return false;
		}
	}

	if (sizeof(s_BtStackSdcMemPool) < ram)
	{
		return false;
	}

	if (MpslInit() == false)
	{
		return false;
	}

	// Enable BLE stack.

	DEBUG_PRINTF("sdc_enable\r\n");

	res = sdc_enable(BtStackSdcCB, s_BtStackSdcMemPool);
	if (res != 0)
	{
		return false;
	}

	return true;
}

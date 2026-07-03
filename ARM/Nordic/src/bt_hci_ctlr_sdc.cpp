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
#include "sdc_hci_cmd_le.h"
#include "sdc_hci_cmd_controller_baseband.h"
#include "sdc_hci_cmd_link_control.h"

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_ctlr.h"

static inline size_t BtHciCtlrSendData(BtHciCtlrDev_t * const pDev, void *pData, size_t Len) {
	return sdc_hci_data_put((uint8_t*)pData) == 0 ? Len : 0;
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

// SDC command executor. Sends the command, then pumps the controller until the
// matching Command Complete or Command Status sets CmdDone, and returns the HCI
// status. Command credit and opcode match are the generic fields filled by
// BtHciProcessEvent. This is the SDC response model: a busy wait on sdc_hci_get
// through BtHciCtlrProcess. A target with an event driven SDK would bind its own.
// SDC command executor. The SDC HCI command interface is typed, not raw: each
// opcode maps to a sdc_hci_cmd_le_* wrapper. The generic parameters are the
// standard HCI wire layout, which matches the SDC command struct, so they cast
// directly. The wrappers are synchronous, so this returns the status inline; the
// generic command credit and match path is used only by a raw HCI controller.
extern "C" uint8_t BtHciCmdSdc(BtHciDevice_t * const pDev, uint16_t OpCode, const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	int32_t res;

	switch (OpCode)
	{
		case BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM:
			{
				sdc_hci_cmd_le_set_ext_adv_params_return_t r;
				res = sdc_hci_cmd_le_set_ext_adv_params((const sdc_hci_cmd_le_set_ext_adv_params_t*)pParam, &r);
				if (pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA:
			res = sdc_hci_cmd_le_set_ext_adv_data((const sdc_hci_cmd_le_set_ext_adv_data_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_EXT_SCAN_RESP_DATA:
			res = sdc_hci_cmd_le_set_ext_scan_response_data((const sdc_hci_cmd_le_set_ext_scan_response_data_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE:
			res = sdc_hci_cmd_le_set_ext_adv_enable((const sdc_hci_cmd_le_set_ext_adv_enable_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR:
			res = sdc_hci_cmd_le_set_adv_set_random_address((const sdc_hci_cmd_le_set_adv_set_random_address_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM:
			res = sdc_hci_cmd_le_set_ext_scan_params((const sdc_hci_cmd_le_set_ext_scan_params_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_EXT_SCAN_ENABLE:
			res = sdc_hci_cmd_le_set_ext_scan_enable((const sdc_hci_cmd_le_set_ext_scan_enable_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_CREATE_CONN:
			res = sdc_hci_cmd_le_create_conn((const sdc_hci_cmd_le_create_conn_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_ENABLE_ENCRYPTION:
			res = sdc_hci_cmd_le_enable_encryption((const sdc_hci_cmd_le_enable_encryption_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_REPLY:
			{
				sdc_hci_cmd_le_long_term_key_request_reply_return_t r;
				res = sdc_hci_cmd_le_long_term_key_request_reply((const sdc_hci_cmd_le_long_term_key_request_reply_t*)pParam, &r);
			}
			break;

		case BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_NEG_REPLY:
			{
				sdc_hci_cmd_le_long_term_key_request_negative_reply_return_t r;
				res = sdc_hci_cmd_le_long_term_key_request_negative_reply((const sdc_hci_cmd_le_long_term_key_request_negative_reply_t*)pParam, &r);
			}
			break;

		case BT_HCI_CMD_CTLR_SET_RANDOM_ADDR:
			res = sdc_hci_cmd_le_set_random_address((const sdc_hci_cmd_le_set_random_address_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN:
			{
				sdc_hci_cmd_le_read_max_data_length_return_t r;
				res = sdc_hci_cmd_le_read_max_data_length(&r);
				if (pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_WRITE_SUGG_DEFAULT_DATA_LEN:
			res = sdc_hci_cmd_le_write_suggested_default_data_length((const sdc_hci_cmd_le_write_suggested_default_data_length_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_EVENT_MASK:
			res = sdc_hci_cmd_le_set_event_mask((const sdc_hci_cmd_le_set_event_mask_t*)pParam);
			break;

		case BT_HCI_CMD_BASEBAND_SET_EVENT_MASK:
			res = sdc_hci_cmd_cb_set_event_mask((const sdc_hci_cmd_cb_set_event_mask_t*)pParam);
			break;

		case BT_HCI_CMD_BASEBAND_SET_EVENT_MASK_PAGE2:
			res = sdc_hci_cmd_cb_set_event_mask_page_2((const sdc_hci_cmd_cb_set_event_mask_page_2_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_ENCRYPT:
			{
				sdc_hci_cmd_le_encrypt_return_t r;
				res = sdc_hci_cmd_le_encrypt((const sdc_hci_cmd_le_encrypt_t*)pParam, &r);
				if (pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_LINKCTRL_DISCONNECT:
			res = sdc_hci_cmd_lc_disconnect((const sdc_hci_cmd_lc_disconnect_t*)pParam);
			break;

		default:
			return 0xFF;
	}

	return res == 0 ? 0 : 0xFF;
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
	pDev->Send = BtHciCtlrSendData;		// SDC ACL transmit, forwarded by BtHciCtlrSdcSend
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

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

// ---- SDC controller bring-up (relocated from bt_app_sdc) ----

#include <stdlib.h>
#include "nrf.h"
#include "sdc.h"
#include "sdc_soc.h"
#include "sdc_hci_vs.h"
#include "nrf_mpsl.h"

#include "sdc_hci.h"
#include "sdc_hci_cmd_le.h"
#include "sdc_hci_cmd_info_params.h"
#include "sdc_hci_cmd_controller_baseband.h"
#include "sdc_hci_cmd_link_control.h"

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "bluetooth/bt_gap.h"
#include "coredev/system_core_clock.h"
#include "crypto_rng_nrf.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on the SMP handshake trace for this file: every
// SMP PDU in/out and the link state. Output goes to the SysLog transport the
// app configured (UART, USB, RTT, BLE, or any other DeviceIntrf); the trace
// does not assume a transport. A release build defines NDEBUG, which strips all
// trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

// Which persistence the build uses. Deliberately independent of DEBUG_ENABLE
// and of NDEBUG: the release build is where a bond has to survive a reset, and
// this one line says whether the store is even in the picture.
#define STORE_TRACE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

// PCD-side controller device pointer, set by BtHciCtlrStart.
static BtHciCtlrDev_t *s_pBtHciCtlrSdc = nullptr;

static void BtHciCtlrSdcApplyCapabilities(BtHciDevice_t * const pDev)
{
	if (pDev == nullptr || s_pBtHciCtlrSdc == nullptr ||
		s_pBtHciCtlrSdc->CapabilitiesApplied)
	{
		return;
	}

	const BtHciCapabilities_t *pCapabilities =
		&s_pBtHciCtlrSdc->Capabilities;
	if ((pCapabilities->Valid & BT_HCI_CAP_VALID_BUFFER_SIZE) == 0 ||
		pCapabilities->LeAclDataLen == 0 ||
		pCapabilities->LeAclPacketCount == 0)
	{
		return;
	}

	// LE Read Buffer Size reports the controller buffers available for
	// host-to-controller ACL data. Replace the configured startup assumption
	// once, before the first real host command can lead to ACL traffic.
	BtHciSetLeAclBuffer(pDev, pCapabilities->LeAclDataLen,
		pCapabilities->LeAclPacketCount);
	s_pBtHciCtlrSdc->pHciDev = pDev;
	s_pBtHciCtlrSdc->CapabilitiesApplied = true;
}

static inline size_t BtHciCtlrSendData(BtHciCtlrDev_t * const pDev, void *pData, size_t Len) {
	return sdc_hci_data_put((uint8_t*)pData) == 0 ? Len : 0;
}

void BtHciCtlrProcess(BtHciCtlrDev_t * const pDev)
{
	if (pDev == nullptr || pDev->RxHandler == nullptr)
	{
		return;
	}

	// HCI_MSG_BUFFER_MAX_SIZE is what sdc_hci_get documents as the minimum
	// buffer. It does not cover isochronous channels: enabling ISO requires
	// HCI_MSG_BUFFER_ISO_MAX_SIZE (or the configured rx_sdu_buffer_size)
	// here, so revisit this buffer if ISO support is ever turned on.
	uint8_t buf[HCI_MSG_BUFFER_MAX_SIZE];

	// sdc_hci_get takes uint8_t *p_msg_type_out and writes exactly one byte.
	// Passing a sdc_hci_msg_type_t through a uint8_t cast only works while the
	// enum happens to be one byte wide, which is the arm-none-eabi default
	// (-fshort-enums, AAPCS) but not guaranteed and not set by any project
	// file here. With a four byte enum the call fills the low byte and leaves
	// the other three indeterminate, so the comparison below can fail for a
	// real event and route every event into the ACL data path. Use the type
	// the API asks for.
	uint8_t mtype = SDC_HCI_MSG_TYPE_NONE;

	// Drain every queued message. The controller can queue several at once,
	// for example a command completion followed by an Encryption Change event
	// during pairing; stopping after one strands the later packets.
	while (sdc_hci_get(buf, &mtype) == 0)
	{
		pDev->RxHandler(pDev, mtype == SDC_HCI_MSG_TYPE_EVT, buf);
	}
}

size_t BtHciCtlrSdcSend(void *pData, size_t Len)
{
	if (s_pBtHciCtlrSdc == nullptr || s_pBtHciCtlrSdc->Send == nullptr)
	{
		return 0;
	}

	return s_pBtHciCtlrSdc->Send(s_pBtHciCtlrSdc, pData, Len);
}

// SDC command executor. The SDC HCI command interface is typed, not raw: each
// opcode maps to a sdc_hci_cmd_* wrapper. The generic parameters are the
// standard HCI wire layout, which matches the SDC command struct, so they cast
// directly. The wrappers are synchronous and return the HCI status inline.
uint8_t BtHciCmdSdc(BtHciDevice_t * const pDev, uint16_t OpCode, const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	BtHciCtlrSdcApplyCapabilities(pDev);

	uint8_t res;

	switch (OpCode)
	{
		case BT_HCI_CMD_INFO_READ_LOCAL_VERS_INFO:
			{
				sdc_hci_cmd_ip_read_local_version_information_return_t r = {};
				res = sdc_hci_cmd_ip_read_local_version_information(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS:
			{
				sdc_hci_cmd_ip_read_local_supported_commands_return_t r = {};
				res = sdc_hci_cmd_ip_read_local_supported_commands(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, r.raw, RetLen < sizeof(r.raw) ? RetLen : sizeof(r.raw));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES:
			{
				sdc_hci_cmd_le_read_local_supported_features_return_t r = {};
				res = sdc_hci_cmd_le_read_local_supported_features(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, r.raw, RetLen < sizeof(r.raw) ? RetLen : sizeof(r.raw));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_READ_SUPPORTED_STATES:
			{
				sdc_hci_cmd_le_read_supported_states_return_t r = {};
				res = sdc_hci_cmd_le_read_supported_states(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, r.le_states,
						RetLen < sizeof(r.le_states) ? RetLen : sizeof(r.le_states));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT:
			{
				sdc_hci_cmd_le_read_buffer_size_v2_return_t r = {};
				res = sdc_hci_cmd_le_read_buffer_size_v2(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_READ_BUFF_SIZE:
			{
				sdc_hci_cmd_le_read_buffer_size_return_t r = {};
				res = sdc_hci_cmd_le_read_buffer_size(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_RESOLVING_LIST_ADD_DEV:
			res = sdc_hci_cmd_le_add_device_to_resolving_list(
				(const sdc_hci_cmd_le_add_device_to_resolving_list_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_RESOLVING_LIST_REMOVE_DEV:
			res = sdc_hci_cmd_le_remove_device_from_resolving_list(
				(const sdc_hci_cmd_le_remove_device_from_resolving_list_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_RESOLVING_LIST_CLEAR:
			res = sdc_hci_cmd_le_clear_resolving_list();
			break;

		case BT_HCI_CMD_CTLR_RESOLVING_LIST_READ_SIZE:
			{
				sdc_hci_cmd_le_read_resolving_list_size_return_t r = {};
				res = sdc_hci_cmd_le_read_resolving_list_size(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE:
			res = sdc_hci_cmd_le_set_address_resolution_enable(
				(const sdc_hci_cmd_le_set_address_resolution_enable_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_RESOLVABLE_PRIVATE_ADDR_TIMEOUT:
			res = sdc_hci_cmd_le_set_resolvable_private_address_timeout(
				(const sdc_hci_cmd_le_set_resolvable_private_address_timeout_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_PRIVACY_MODE:
			res = sdc_hci_cmd_le_set_privacy_mode(
				(const sdc_hci_cmd_le_set_privacy_mode_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_READ_MAX_ADV_DATA_LEN:
			{
				sdc_hci_cmd_le_read_max_adv_data_length_return_t r = {};
				res = sdc_hci_cmd_le_read_max_adv_data_length(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_READ_NB_SUPPORTED_ADV_SETS:
			{
				sdc_hci_cmd_le_read_number_of_supported_adv_sets_return_t r = {};
				res = sdc_hci_cmd_le_read_number_of_supported_adv_sets(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE:
			{
				sdc_hci_cmd_le_read_periodic_adv_list_size_return_t r = {};
				res = sdc_hci_cmd_le_read_periodic_adv_list_size(&r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

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

		case BT_HCI_CMD_CTLR_SET_SCAN_PARAM:
			res = sdc_hci_cmd_le_set_scan_params((const sdc_hci_cmd_le_set_scan_params_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_SCAN_ENABLE:
			res = sdc_hci_cmd_le_set_scan_enable((const sdc_hci_cmd_le_set_scan_enable_t*)pParam);
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

		case BT_HCI_CMD_CTLR_EXT_CREATE_CONN:
			res = sdc_hci_cmd_le_ext_create_conn((const sdc_hci_cmd_le_ext_create_conn_t*)pParam);
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

		case BT_HCI_CMD_CTLR_SET_DATA_LEN:
			{
				sdc_hci_cmd_le_set_data_length_return_t r = {};
				res = sdc_hci_cmd_le_set_data_length(
					(const sdc_hci_cmd_le_set_data_length_t*)pParam, &r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_READ_PHY:
			{
				sdc_hci_cmd_le_read_phy_return_t r = {};
				res = sdc_hci_cmd_le_read_phy(
					(const sdc_hci_cmd_le_read_phy_t*)pParam, &r);
				if (res == 0 && pRet != nullptr && RetLen > 0)
				{
					memcpy(pRet, &r, RetLen < sizeof(r) ? RetLen : sizeof(r));
				}
			}
			break;

		case BT_HCI_CMD_CTLR_SET_DEFAULT_PHY:
			res = sdc_hci_cmd_le_set_default_phy(
				(const sdc_hci_cmd_le_set_default_phy_t*)pParam);
			break;

		case BT_HCI_CMD_CTLR_SET_PHY:
			res = sdc_hci_cmd_le_set_phy(
				(const sdc_hci_cmd_le_set_phy_t*)pParam);
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

	// SDC typed command functions already return the standard HCI status.
	// Keep 0xFF for an opcode that has no SDC implementation above.
	return res;
}

alignas(8) static uint8_t s_BtStackSdcMemPool[10000];

static void BtStackSdcAssert(const char * file, const uint32_t line)
{
	DEBUG_PRINTF("SDC Fault: %s, %d\n", file, line);
	while(1);
}

// Controller entropy source. The SoftDevice Controller draws its entropy pool
// through these fixed-signature callbacks; they route to the Nordic hardware RNG
// (CryptoRngNrf): a CRACEN CTR-DRBG seeded from the CRACEN TRNG on nRF54L/H, the
// NRF_RNG peripheral on nRF52/53/91. This removes the libc rand path so BLE
// pairing, SMP, OOB and key material never draw from a software PRNG. The build
// must link the Nordic RNG (rng_nrfx.cpp); a part without an RNG peripheral does
// not link.
static uint8_t BtStackRandPrioLowGet(uint8_t *pBuff, uint8_t Len)
{
	DEBUG_PRINTF("BtStackRandPrioLowGet\r\n");
	// Fail closed: report 0 bytes if the hardware RNG did not deliver entropy.
	return CryptoRngNrfInstance()->Random(pBuff, Len) == CRYPTO_STATUS_OK ?
		   Len : 0;
}

static uint8_t BtStackRandPrioHighGet(uint8_t *pBuff, uint8_t Len)
{
	return BtStackRandPrioLowGet(pBuff, Len);
}

static void BtStackRandPrioLowGetBlocking(uint8_t *pBuff, uint8_t Len)
{
	// Fail closed: the poll source must not report weak entropy. Retry until the
	// hardware RNG delivers, rather than proceeding with an unfilled buffer.
	while (CryptoRngNrfInstance()->Random(pBuff, Len) != CRYPTO_STATUS_OK)
	{
	}
}

static void BtStackSdcCB()
{
	if (s_pBtHciCtlrSdc != nullptr && s_pBtHciCtlrSdc->OnWake != nullptr)
	{
		s_pBtHciCtlrSdc->OnWake();
	}

	BtHciCtlrProcess(s_pBtHciCtlrSdc);
}

// SDC bring-up, reached from the generic BtHciCtlrEnable once BtHciCtlrInit
// has succeeded. The argument checks and the initialization check live there,
// so this starts from a device that is already wired.
bool BtHciCtlrStart(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	pDev->Send = BtHciCtlrSendData;		// SDC ACL transmit, forwarded by BtHciCtlrSdcSend
	s_pBtHciCtlrSdc = pDev;

	// MPSL first. sdc_init answers -NRF_EPERM when MPSL is not up, and it used
	// to run 188 lines ahead of MpslInit, so it had been returning that error
	// on every boot. The result was assigned and never read, which is why
	// nothing showed until the result was checked.
	if (MpslInit() == false)
	{
		DEBUG_PRINTF("MpslInit failed\r\n");

		return false;
	}

	// sdc_init returns -NRF_EINVAL or -NRF_EPERM (MPSL not up, or the low
	// frequency clock accuracy is not good enough). Everything below operates
	// on a controller that was never initialised, so stop here rather than
	// carry on and fail somewhere less obvious.
	int32_t res = sdc_init(BtStackSdcAssert);
	if (res != 0)
	{
		DEBUG_PRINTF("sdc_init failed %d\r\n", (int)res);

		return false;
	}

	//sdc_hci_cmd_cb_reset();

	sdc_rand_source_t rand_functions = {
		//.rand_prio_low_get = BtStackRandPrioLowGet,
		//.rand_prio_high_get = BtStackRandPrioHighGet,
		.rand_poll = BtStackRandPrioLowGetBlocking
	};

	// Without an entropy source the controller cannot produce the random
	// numbers pairing and private addresses need. sdc_enable does refuse to
	// start in that case, but failing here names the cause.
	res = sdc_rand_source_register(&rand_functions);
	if (res != 0)
	{
		DEBUG_PRINTF("sdc_rand_source_register failed %d\r\n", (int)res);

		return false;
	}

	// The sdc_support_* calls below are not checked. What they document is a
	// call order error, returned when they run after sdc_cfg_set or
	// sdc_enable, and the order here is fixed. Their return type also differs
	// between nrfxlib versions, so nothing should depend on the value.
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

	int32_t ram = 0;
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
		DEBUG_PRINTF("sdc_cfg_set SDC_CFG_TYPE_BUFFER_CFG failed %d\r\n", (int)ram);

		return false;
	}

	// Vendor specific command, answered with an HCI status. A refusal leaves
	// the controller default event length in place, which costs throughput but
	// still gives a working link, so report it and carry on.
	sdc_hci_cmd_vs_event_length_set_t evlen = {
		.event_length_us = 7500,
	};

	uint8_t hcistat = sdc_hci_cmd_vs_event_length_set(&evlen);
	if (hcistat != 0)
	{
		DEBUG_PRINTF("sdc_hci_cmd_vs_event_length_set status 0x%02x\r\n", hcistat);
	}

	/*
	cfg.event_length.event_length_us = 7500;
	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
						  SDC_CFG_TYPE_EVENT_LENGTH,
						  &cfg);
	if (ram < 0)
	{
		DEBUG_PRINTF("sdc_cfg_set SDC_CFG_TYPE_EVENT_LENGTH failed %d\r\n", (int)ram);

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
			DEBUG_PRINTF("sdc_cfg_set SDC_CFG_TYPE_PERIPHERAL_COUNT failed %d\r\n", (int)ram);

			return false;
		}

		cfg.adv_count.count = 1;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_COUNT,
							  &cfg);
		if (ram < 0)
		{
			DEBUG_PRINTF("sdc_cfg_set SDC_CFG_TYPE_ADV_COUNT failed %d\r\n", (int)ram);

			return false;
		}

		cfg.adv_buffer_cfg.max_adv_data = 255;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_BUFFER_CFG,
							  &cfg);
		if (ram < 0)
		{
			DEBUG_PRINTF("sdc_cfg_set SDC_CFG_TYPE_ADV_BUFFER_CFG failed %d\r\n", (int)ram);

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
			DEBUG_PRINTF("sdc_cfg_set SDC_CFG_TYPE_CENTRAL_COUNT failed %d\r\n", (int)ram);

			return false;
		}

		cfg.scan_buffer_cfg.count = 10;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
						  SDC_CFG_TYPE_SCAN_BUFFER_CFG,
						  &cfg);
		if (ram < 0)
		{
			DEBUG_PRINTF("sdc_cfg_set SDC_CFG_TYPE_SCAN_BUFFER_CFG failed %d\r\n", (int)ram);

			return false;
		}
	}

	// ram is the pool size the configuration above needs, reported by the last
	// sdc_cfg_set. It is non-negative here because every call was checked, but
	// compare as a signed value so a negative one cannot convert to a huge
	// unsigned and pass.
	if ((int32_t)sizeof(s_BtStackSdcMemPool) < ram)
	{
		DEBUG_PRINTF("sdc mem pool too small: have %d need %d\r\n",
					 (int)sizeof(s_BtStackSdcMemPool), (int)ram);

		return false;
	}

	// The radio is about to run, so the memory controller stops being ours
	// alone. Offer the timeslot session as its arbiter; without this a write
	// would go straight at the controller while the link is live.
	int arb = MpslNvmArbiterStart();
	if (arb != 0)
	{
		DEBUG_PRINTF("MpslNvmArbiterStart failed %d\r\n", arb);

		return false;
	}

	// Enable BLE stack.
	DEBUG_PRINTF("sdc_enable\r\n");

	res = sdc_enable(BtStackSdcCB, s_BtStackSdcMemPool);
	if (res != 0)
	{
		DEBUG_PRINTF("sdc_enable failed %d\r\n", (int)res);

		// The radio never started, so the timeslot session opened above has
		// nothing to arbitrate against. Close it, or the memory controller
		// keeps going through a session that will never see a radio event.
		MpslNvmArbiterStop();

		return false;
	}

	// Capability queries are valid only after the controller is enabled. Use a
	// temporary host device bound to the same typed command executor; the
	// controller owns the result and the real host receives the ACL limits on
	// its first command. A failed discovery leaves Valid at zero, so the
	// application's configured ACL fallback remains active.
	memset(&pDev->Capabilities, 0, sizeof(pDev->Capabilities));
	pDev->pHciDev = nullptr;
	pDev->CapabilitiesApplied = false;

	BtHciDevice_t hciDev = {};
	hciDev.Command = BtHciCmdSdc;
	if (BtHciCapabilitiesRead(&hciDev, &pDev->Capabilities) == false)
	{
		DEBUG_PRINTF("SDC capability discovery failed\r\n");
	}

	return true;
}

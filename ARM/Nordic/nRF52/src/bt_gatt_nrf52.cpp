/**-------------------------------------------------------------------------
 * @file	bt_gatt_nrf52.cpp
 *
 * @brief	Implement Bluetooth GATT service and characteristic with nRF5_SDK
 *
 * Implementation allow the creation of generic custom Bluetooth Smart service
 * with multiple user defined characteristics.
 *
 * This implementation is to be used with Nordic SDK
 *
 * @author	Hoang Nguyen Hoan
 * @date	Mar. 25, 2014
 *
 * @license
 *
 * MIT License
 *
 * Copyright (c) 2014-2020 I-SYST inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "app_error.h"
#include "ble_srv_common.h"

#include "bluetooth/bt_gatt.h"

#pragma pack(push, 1)
// Service connection security types
typedef enum {
	BTSRVC_SECTYPE_NONE,				//!< open, no security
	BTSRVC_SECTYPE_STATICKEY_NO_MITM,	//!< Bonding static pass key without Man In The Middle
	BTSRVC_SECTYPE_STATICKEY_MITM,		//!< Bonding static pass key with MITM
	BTSRVC_SECTYPE_LESC_MITM,			//!< LE secure encryption
	BTSRVC_SECTYPE_SIGNED_NO_MITM,		//!< AES signed encryption without MITM
	BTSRVC_SECTYPE_SIGNED_MITM,			//!< AES signed encryption with MITM
} BTSRVC_SECTYPE;

typedef struct {
	uint16_t Handle;
	uint16_t Offset;
	uint16_t Len;
} GATLWRHDR;

typedef struct {
	GATLWRHDR Hdr;
	uint8_t Data[1];
} GETTLWRMEM;

#pragma pack(pop)

static uint16_t s_ConnHandle = BLE_CONN_HANDLE_INVALID;
static BtGattSrvc_t *s_pBtGattSrvcHead = nullptr;
static BtGattSrvc_t *s_pBtGattSrvcTail = nullptr;

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	(void)ConnHdl;

	if (s_ConnHandle == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

	if (pChar == nullptr)
	{
		return false;
	}

	if (pChar->Runtime.bNotify == false)
	{
		return false;
	}

	ble_gatts_hvx_params_t params;

	memset(&params, 0, sizeof(params));
	params.type   = BLE_GATT_HVX_NOTIFICATION;
	params.handle = pChar->Runtime.ValHdl;
	params.p_data = (uint8_t*)pVal;

	// SoftDevice expects p_len to be a pointer to a uint16_t length.
	uint16_t l = (uint16_t)Len;
	params.p_len = &l;

	uint32_t err_code = sd_ble_gatts_hvx(s_ConnHandle, &params);

	return err_code == NRF_SUCCESS;
}

bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar == nullptr)
	{
		return false;
	}

	ble_gatts_value_t value;

	memset(&value, 0, sizeof(ble_gatts_value_t));

	value.offset  = 0;
	value.len     = (uint16_t)Len;
	value.p_value = (uint8_t*)pVal;

	uint32_t err_code = sd_ble_gatts_value_set(s_ConnHandle,
	                                           pChar->Runtime.ValHdl,
	                                           &value);
	return err_code == NRF_SUCCESS;
}

void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc)
{
	(void)pSrvc;
}

static void GatherLongWrBuff(GATLWRHDR *pHdr)
{
	uint8_t *p = (uint8_t*)pHdr + pHdr->Len + sizeof(GATLWRHDR);
	GATLWRHDR *hdr = (GATLWRHDR*)p;
	if (hdr->Handle == pHdr->Handle)
	{
		GatherLongWrBuff(hdr);
		pHdr->Len += hdr->Len;
		memcpy(p, p + sizeof(GATLWRHDR), hdr->Len);
	}
}

void BtGattSrvcEvtHandler(BtGattSrvc_t * const pSrvc, uint32_t Evt, void * const pCtx)
{
	(void)pCtx;

	ble_evt_t *pBleEvt = (ble_evt_t *)Evt;

	switch (pBleEvt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			s_ConnHandle = pBleEvt->evt.gap_evt.conn_handle;
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			s_ConnHandle = BLE_CONN_HANDLE_INVALID;
			for (int i = 0; i < pSrvc->NbChar; i++)
			{
				// New API: notify state is in Runtime
				pSrvc->pCharArray[i].Runtime.bNotify = false;
				pSrvc->pCharArray[i].Runtime.bIndic  = false;
			}
			break;

		case BLE_GATTS_EVT_WRITE:
		{
			ble_gatts_evt_write_t *p_evt_write = &pBleEvt->evt.gatts_evt.params.write;

			for (int i = 0; i < pSrvc->NbChar; i++)
			{
				BtGattChar_t *pChar = &pSrvc->pCharArray[i];

				if (p_evt_write->op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
				{
					// Long write execute
					GATLWRHDR *hdr = (GATLWRHDR *)pSrvc->pLongWrBuff;
					uint8_t *p = (uint8_t*)pSrvc->pLongWrBuff + sizeof(GATLWRHDR);

					if (hdr && hdr->Handle == pChar->Runtime.ValHdl)
					{
						GatherLongWrBuff(hdr);
						if (pChar->WrCB)
						{
							pChar->WrCB(pChar, p, hdr->Offset, hdr->Len);
						}
					}
				}
				else
				{
					// CCCD write (subscribe/unsubscribe)
					if ((p_evt_write->handle == pChar->Runtime.CccdHdl) &&
						(p_evt_write->len == 2))
					{
						bool en = ble_srv_is_notification_enabled(p_evt_write->data) ? true : false;
						pChar->Runtime.bNotify = en;

						if (pChar->SetNotifCB)
						{
							pChar->SetNotifCB(pChar, en);
						}
					}
					// Value write
					else if ((p_evt_write->handle == pChar->Runtime.ValHdl) &&
							 (pChar->WrCB != NULL))
					{
						pChar->WrCB(pChar, p_evt_write->data, 0, p_evt_write->len);
					}
					else
					{
						// Not for us
					}
				}
			}
		}
		break;

		case BLE_EVT_USER_MEM_REQUEST:
		{
			if (pSrvc->pLongWrBuff != NULL)
			{
				ble_user_mem_block_t mblk;
				memset(&mblk, 0, sizeof(ble_user_mem_block_t));
				mblk.p_mem = pSrvc->pLongWrBuff;
				mblk.len   = pSrvc->LongWrBuffSize;
				memset(pSrvc->pLongWrBuff, 0, pSrvc->LongWrBuffSize);

				uint32_t err_code = sd_ble_user_mem_reply(pBleEvt->evt.gatts_evt.conn_handle, &mblk);
				APP_ERROR_CHECK(err_code);
			}
		}
		break;

		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
			if (pSrvc->AuthReqCB)
			{
				// Not wired in this port yet
			}
			break;

#if (NRF_SD_BLE_API_VERSION > 3)
		case BLE_GATTS_EVT_HVN_TX_COMPLETE:
#else
		case BLE_EVT_TX_COMPLETE:
#endif
		{
			for (int i = 0; i < pSrvc->NbChar; i++)
			{
				BtGattChar_t *pChar = &pSrvc->pCharArray[i];

				if (pBleEvt->evt.gatts_evt.params.hvc.handle == pChar->Runtime.ValHdl &&
					pChar->TxCompleteCB != NULL)
				{
					pChar->TxCompleteCB(pChar, i);
				}
			}
		}
		break;

		default:
			break;
	}
}

void BtGattEvtHandler(uint32_t Evt)
{
	(void)Evt;
}

static void BtSrvcEncSec(ble_gap_conn_sec_mode_t *pSecMode, BTSRVC_SECTYPE SecType)
{
	switch (SecType)
	{
		case BTSRVC_SECTYPE_STATICKEY_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(pSecMode);
			break;
		case BTSRVC_SECTYPE_STATICKEY_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(pSecMode);
			break;
		case BTSRVC_SECTYPE_LESC_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(pSecMode);
			break;
		case BTSRVC_SECTYPE_SIGNED_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(pSecMode);
			break;
		case BTSRVC_SECTYPE_SIGNED_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_WITH_MITM(pSecMode);
			break;
		case BTSRVC_SECTYPE_NONE:
		default:
			BLE_GAP_CONN_SEC_MODE_SET_OPEN(pSecMode);
			break;
	}
}

/**
 * @brief Add control characteristic.
 *
 * @param[in]   	pSrvc   : Service data.
 * @param[in/out]   pChar   : characteristic to initialize.
 * @param[in]		SecType : Security type
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t BtGattCharAdd(BtGattSrvc_t *pSrvc, BtGattChar_t *pChar, BTSRVC_SECTYPE SecType)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;

	memset(&cccd_md, 0, sizeof(cccd_md));
	memset(&attr_md, 0, sizeof(attr_md));
	memset(&char_md, 0, sizeof(char_md));

	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	char_md.p_char_user_desc = (uint8_t*)pChar->pDesc;
	if (pChar->pDesc != NULL)
	{
		char_md.char_user_desc_max_size = strlen(pChar->pDesc) + 1;
		char_md.char_user_desc_size     = strlen(pChar->pDesc) + 1;
	}
	char_md.p_char_pf      = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md      = NULL;
	char_md.p_sccd_md      = NULL;

	if (pChar->Property & BT_GATT_CHAR_PROP_NOTIFY)
	{
		char_md.char_props.notify = 1;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);

		// Cast to avoid enum/float deprecated warning from some macro internals.
		BtSrvcEncSec(&cccd_md.write_perm, (BTSRVC_SECTYPE)(int)SecType);
		BtSrvcEncSec(&attr_md.read_perm,  (BTSRVC_SECTYPE)(int)SecType);

		char_md.p_cccd_md = &cccd_md;
	}

	if (pChar->Property & BT_GATT_CHAR_PROP_BROADCAST)
	{
		char_md.char_props.broadcast = 1;
	}

	if (pChar->Property & BT_GATT_CHAR_PROP_READ)
	{
		char_md.char_props.read = 1;
		BtSrvcEncSec(&attr_md.read_perm, (BTSRVC_SECTYPE)(int)SecType);
	}
	else
	{
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
	}

	if (pChar->Property & (BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_WRITE_WORESP))
	{
		if (pChar->Property & BT_GATT_CHAR_PROP_WRITE)
		{
			char_md.char_props.write = 1;
		}
		if (pChar->Property & BT_GATT_CHAR_PROP_WRITE_WORESP)
		{
			char_md.char_props.write_wo_resp = 1;
		}

		BtSrvcEncSec(&attr_md.write_perm, (BTSRVC_SECTYPE)(int)SecType);
	}
	else
	{
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	}

	ble_uuid.type = pSrvc->Uuid.BaseIdx;
	ble_uuid.uuid = pChar->Uuid;

	attr_md.vloc = BLE_GATTS_VLOC_STACK;

	attr_md.rd_auth = (pChar->Property & BT_GATT_CHAR_PROP_AUTH_SIGNED) ? 1 : 0;
	attr_md.wr_auth = (pChar->Property & BT_GATT_CHAR_PROP_AUTH_SIGNED) ? 1 : 0;

	// Variable length values by default
	attr_md.vlen = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = pChar->MaxDataLen;
	attr_char_value.init_len  = pChar->Runtime.ValueLen;
	attr_char_value.p_value   = (uint8_t*)pChar->Runtime.pValue;

	ble_gatts_char_handles_t hdl;
	uint32_t res = sd_ble_gatts_characteristic_add(pSrvc->Hdl, &char_md, &attr_char_value, &hdl);

	pChar->Runtime.Hdl     = hdl.value_handle;
	pChar->Runtime.ValHdl  = hdl.value_handle;
	pChar->Runtime.DescHdl = hdl.user_desc_handle;
	pChar->Runtime.CccdHdl = hdl.cccd_handle;
	pChar->Runtime.SccdHdl = hdl.sccd_handle;
	pChar->Runtime.pSrvc   = pSrvc;

	return res;
}

/**
 * @brief Create BLE service
 *
 * @param [in/out]	pSrvc : Service descriptor (config + runtime, single struct)
 */
bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	uint32_t   err;
	ble_uuid_t ble_uuid;

	if (pSrvc == nullptr || pSrvc->pCharArray == nullptr || pSrvc->NbChar <= 0)
	{
		return false;
	}

	// Add base UUID to softdevice's internal list for custom services.
	if (pSrvc->bCustom == true)
	{
		pSrvc->Uuid.BaseIdx = BtUuidAddBase(pSrvc->UuidBase);

		uint8_t type;
		err = sd_ble_uuid_vs_add((ble_uuid128_t*)pSrvc->UuidBase, &type);
		if (err != NRF_SUCCESS)
		{
			return false;
		}
		pSrvc->Uuid.BaseIdx = type;
	}

	ble_uuid.type = pSrvc->Uuid.BaseIdx;
	ble_uuid.uuid = pSrvc->UuidSrvc;

	err = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &pSrvc->Hdl);
	if (err != NRF_SUCCESS)
	{
		return false;
	}

	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		err = BtGattCharAdd(pSrvc, &pSrvc->pCharArray[i], BTSRVC_SECTYPE_NONE);
		if (err != NRF_SUCCESS)
		{
			return false;
		}
		pSrvc->pCharArray[i].Runtime.bNotify = false;
		pSrvc->pCharArray[i].Runtime.bIndic  = false;
	}

	BtGattInsertSrvcList(pSrvc);

	return true;
}

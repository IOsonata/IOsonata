/**-------------------------------------------------------------------------
@file	bt_gatt_bm.cpp

@brief	Implement Bluetooth GATT service and characteristic

Implementation for Nordic sdk-nrf-bm with SoftDevice S145 (nRF54L15).
Uses the same sd_ble_gatts_* SVC API as older SoftDevices (S132/S140).

Based on bt_gatt_nrf52.cpp

@author	Hoang Nguyen Hoan
@date	Feb. 17, 2026

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
#include <stdint.h>
#include <string.h>

#include <nrf_error.h>
#include <ble.h>
#include <ble_gap.h>
#include <ble_gatts.h>

#include "bluetooth/bt_gatt.h"

#pragma pack(push, 1)

// Service connection security types
typedef enum {
	BTSRVC_SECTYPE_NONE,				//!< open, no security
	BTSRVC_SECTYPE_STATICKEY_NO_MITM,	//!< Bonding static pass key without Man In The Middle
	BTSRVC_SECTYPE_STATICKEY_MITM,		//!< Bonding static pass key with MITM
	BTSRVC_SECTYPE_LESC_MITM,			//!< LE secure encryption
	BTSRVC_SECTYPE_SIGNED_NO_MITM,		//!< AES signed encryption without MITM
	BTSRVC_SECTYPE_SIGNED_MITM,		//!< AES signed encryption with MITM
} BTSRVC_SECTYPE;

typedef struct {
	uint16_t Handle;
	uint16_t Offset;
	uint16_t Len;
} GATLWRHDR;

#pragma pack(pop)

static uint16_t s_ConnHandle = BLE_CONN_HANDLE_INVALID;

/**
 * @brief Check if notification is enabled from CCCD write data.
 *
 * Replaces ble_srv_is_notification_enabled() from old nRF5 SDK.
 * CCCD is a 16-bit LE value: bit 0 = notification, bit 1 = indication.
 */
static inline bool IsNotificationEnabled(const uint8_t *pData)
{
	uint16_t cccd = pData[0] | ((uint16_t)pData[1] << 8);
	return (cccd & BLE_GATT_HVX_NOTIFICATION) != 0;
}

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	uint16_t ch = (ConnHdl != BLE_CONN_HANDLE_INVALID) ? ConnHdl : s_ConnHandle;

	if (ch == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

	if (pChar->bNotify == false)
	{
		return false;
	}

	ble_gatts_hvx_params_t params;
	memset(&params, 0, sizeof(params));
	params.type   = BLE_GATT_HVX_NOTIFICATION;
	params.handle = pChar->ValHdl;
	params.p_data = (uint8_t*)pVal;

	// SoftDevice expects p_len to point to a uint16_t (it may modify it)
	uint16_t l = (uint16_t)Len;
	params.p_len = &l;

	uint32_t err_code = sd_ble_gatts_hvx(ch, &params);

	return err_code == NRF_SUCCESS;
}

bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	ble_gatts_value_t value;

	memset(&value, 0, sizeof(ble_gatts_value_t));

	value.offset = 0;
	value.len = Len;
	value.p_value = (uint8_t*)pVal;

	uint32_t err_code = sd_ble_gatts_value_set(s_ConnHandle,
											   pChar->ValHdl,
											   &value);
	return err_code == NRF_SUCCESS;
}

void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc)
{

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
				pSrvc->pCharArray[i].bNotify = false;
			}
			break;

		case BLE_GATTS_EVT_WRITE:
			{
				ble_gatts_evt_write_t *p_evt_write = &pBleEvt->evt.gatts_evt.params.write;

				for (int i = 0; i < pSrvc->NbChar; i++)
				{
					if (p_evt_write->op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
					{
						// Long write execution
						GATLWRHDR *hdr = (GATLWRHDR *)pSrvc->pLongWrBuff;
						uint8_t *p = (uint8_t*)pSrvc->pLongWrBuff + sizeof(GATLWRHDR);
						if (hdr->Handle == pSrvc->pCharArray[i].ValHdl)
						{
							GatherLongWrBuff(hdr);
							pSrvc->pCharArray[i].WrCB(&pSrvc->pCharArray[i], p, hdr->Offset, hdr->Len);
						}
					}
					else
					{
						if ((p_evt_write->handle == pSrvc->pCharArray[i].CccdHdl) &&
							(p_evt_write->len == 2))
						{
							// CCCD write — enable/disable notification
							if (IsNotificationEnabled(p_evt_write->data))
							{
								pSrvc->pCharArray[i].bNotify = true;
							}
							else
							{
								pSrvc->pCharArray[i].bNotify = false;
							}
							// Set notify callback
							if (pSrvc->pCharArray[i].SetNotifCB)
							{
								pSrvc->pCharArray[i].SetNotifCB(&pSrvc->pCharArray[i], pSrvc->pCharArray[i].bNotify);
							}
						}
						else if ((p_evt_write->handle == pSrvc->pCharArray[i].ValHdl) &&
								 (pSrvc->pCharArray[i].WrCB != NULL))
						{
							// Value write
							pSrvc->pCharArray[i].WrCB(&pSrvc->pCharArray[i], p_evt_write->data, 0, p_evt_write->len);
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
					mblk.len = pSrvc->LongWrBuffSize;
					memset(pSrvc->pLongWrBuff, 0, pSrvc->LongWrBuffSize);
					sd_ble_user_mem_reply(pBleEvt->evt.gatts_evt.conn_handle, &mblk);
				}
			}
			break;
		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
			if (pSrvc->AuthReqCB)
			{
				pSrvc->AuthReqCB(pSrvc, pBleEvt);
			}
			break;

		case BLE_GATTS_EVT_HVN_TX_COMPLETE:
			{
				for (int i = 0; i < pSrvc->NbChar; i++)
				{
					if (pBleEvt->evt.gatts_evt.params.hvn_tx_complete.handle == pSrvc->pCharArray[i].ValHdl &&
						pSrvc->pCharArray[i].TxCompleteCB != NULL)
					{
						pSrvc->pCharArray[i].TxCompleteCB(&pSrvc->pCharArray[i], i);
					}
				}
			}
			break;

		default:
			break;
	}
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
		case BTSRVC_SECTYPE_SIGNED_MITM:
			// S145 does not support signed writes — fall through to open
		case BTSRVC_SECTYPE_NONE:
		default:
			BLE_GAP_CONN_SEC_MODE_SET_OPEN(pSecMode);
			break;
	}
}

/**
 * @brief Add a characteristic to a GATT service.
 */
static uint32_t BtGattCharAdd(BtGattSrvc_t *pSrvc, BtGattChar_t *pChar,
									 BTSRVC_SECTYPE SecType)
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

	char_md.p_char_user_desc  = (uint8_t*)pChar->pDesc;
	if (pChar->pDesc != NULL)
	{
		char_md.char_user_desc_max_size = strlen(pChar->pDesc) + 1;
		char_md.char_user_desc_size = strlen(pChar->pDesc) + 1;
	}
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = NULL;
	char_md.p_sccd_md = NULL;

	if (pChar->Property & BT_GATT_CHAR_PROP_NOTIFY)
	{
		char_md.char_props.notify = 1;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BtSrvcEncSec(&cccd_md.write_perm, SecType);
		BtSrvcEncSec(&attr_md.read_perm, SecType);
		char_md.p_cccd_md = &cccd_md;
	}

	if (pChar->Property & BT_GATT_CHAR_PROP_BROADCAST)
	{
		char_md.char_props.broadcast = 1;
	}
	if (pChar->Property & BT_GATT_CHAR_PROP_READ)
	{
		char_md.char_props.read = 1;
		BtSrvcEncSec(&attr_md.read_perm, SecType);
	}
	else
	{
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
	}

	if (pChar->Property & (BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_WRITE_WORESP))
	{
		if (pChar->Property & BT_GATT_CHAR_PROP_WRITE)
			char_md.char_props.write = 1;
		if (pChar->Property & BT_GATT_CHAR_PROP_WRITE_WORESP)
			char_md.char_props.write_wo_resp = 1;

		BtSrvcEncSec(&attr_md.write_perm, SecType);
	}
	else
	{
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	}

	ble_uuid.type = pSrvc->Uuid.BaseIdx;
	ble_uuid.uuid = pChar->Uuid;

	attr_md.vloc = BLE_GATTS_VLOC_STACK;

	if (pChar->Property & BT_GATT_CHAR_PROP_AUTH_SIGNED)
	{
		attr_md.rd_auth = 1;
		attr_md.wr_auth = 1;
	}
	else
	{
		attr_md.rd_auth = 0;
		attr_md.wr_auth = 0;
	}

	if (pChar->Property & BT_GATT_CHAR_PROP_VALEN)
	{
		attr_md.vlen = 1;
	}
	else
	{
		attr_md.vlen = 0;
	}

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = pChar->MaxDataLen;
	attr_char_value.init_len  = pChar->ValueLen;
	attr_char_value.p_value   = (uint8_t*)pChar->pValue;

	ble_gatts_char_handles_t hdl;
	uint32_t res = sd_ble_gatts_characteristic_add(pSrvc->Hdl, &char_md, &attr_char_value, &hdl);
	pChar->Hdl = hdl.value_handle;
	pChar->ValHdl = hdl.value_handle;
	pChar->DescHdl = hdl.user_desc_handle;
	pChar->CccdHdl = hdl.cccd_handle;
	pChar->SccdHdl = hdl.sccd_handle;
	pChar->pSrvc = pSrvc;

	return res;
}

/**
 * @brief Create BLE GATT service
 */
bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc, const BtGattSrvcCfg_t *pCfg)
{
	uint32_t err;
	ble_uuid_t ble_uuid;

	// Add base UUID for custom services
	if (pCfg->bCustom == true)
	{
		pSrvc->Uuid.BaseIdx = BtUuidAddBase(pCfg->UuidBase);

		uint8_t type;
		err = sd_ble_uuid_vs_add((ble_uuid128_t*)pCfg->UuidBase, &type);
		if (err != NRF_SUCCESS)
		{
			return false;
		}
		pSrvc->Uuid.BaseIdx = type;
	}

	ble_uuid.type = pSrvc->Uuid.BaseIdx;
	ble_uuid.uuid = pCfg->UuidSrvc;

	err = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &pSrvc->Hdl);
	if (err != NRF_SUCCESS)
	{
		return false;
	}

	pSrvc->NbChar = pCfg->NbChar;
	pSrvc->pCharArray = pCfg->pCharArray;

	for (int i = 0; i < pCfg->NbChar; i++)
	{
		err = BtGattCharAdd(pSrvc, &pSrvc->pCharArray[i], BTSRVC_SECTYPE_NONE);
		if (err != NRF_SUCCESS)
		{
			return false;
		}
		pSrvc->pCharArray[i].bNotify = false;
	}

	pSrvc->pLongWrBuff = pCfg->pLongWrBuff;
	pSrvc->LongWrBuffSize = pCfg->LongWrBuffSize;
	pSrvc->AuthReqCB = pCfg->AuthReqCB;

	BtGattInsertSrvcList(pSrvc);

	return true;
}

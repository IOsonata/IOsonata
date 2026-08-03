/**-------------------------------------------------------------------------
@file	bt_gatt_nrf52.cpp

@brief	Implement Bluetooth GATT service and characteristic with nRF5_SDK

Implementation allow the creation of generic custom Bluetooth Smart service
with multiple user defined characteristics.

This implementation is to be used with Nordic SDK

@author	Hoang Nguyen Hoan
@date	Mar. 25, 2014

@license

MIT License

Copyright (c) 2014-2020 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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

#include "nordic_common.h"
#include "app_error.h"
#include "ble_srv_common.h"

#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"

#pragma pack(push, 1)
typedef enum {
	BTSRVC_SECTYPE_NONE,
	BTSRVC_SECTYPE_STATICKEY_NO_MITM,
	BTSRVC_SECTYPE_STATICKEY_MITM,
	BTSRVC_SECTYPE_LESC_MITM,
	BTSRVC_SECTYPE_SIGNED_NO_MITM,
	BTSRVC_SECTYPE_SIGNED_MITM,
} BTSRVC_SECTYPE;

typedef struct {
	uint16_t Handle;
	uint16_t Offset;
	uint16_t Len;
} GATLWRHDR;
#pragma pack(pop)

#ifndef BT_GATT_LONG_WRITE_RECORD_MAX
#define BT_GATT_LONG_WRITE_RECORD_MAX	32
#endif

// Reassembled value storage. The SoftDevice user-memory queue remains intact
// while every service examines it, so one characteristic cannot destroy the
// records a later characteristic still needs. Applications requiring larger
// values can raise this build-time limit deliberately.
#ifndef BT_GATT_LONG_WRITE_SCRATCH_MAX
#define BT_GATT_LONG_WRITE_SCRATCH_MAX	1024
#endif

alignas(4) static uint8_t s_LongWrScratch[BT_GATT_LONG_WRITE_SCRATCH_MAX];
static uint16_t s_ConnHandle = BLE_CONN_HANDLE_INVALID;

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar,
						  void * const pVal, size_t Len)
{
	if (pChar == nullptr || (Len > 0 && pVal == nullptr) ||
		Len > UINT16_MAX || Len > pChar->MaxDataLen ||
		pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	uint16_t hdl = ConnHdl != BLE_CONN_HANDLE_INVALID ? ConnHdl : s_ConnHandle;
	if (hdl == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

	ble_gatts_hvx_params_t params;
	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_NOTIFICATION;
	params.handle = pChar->ValHdl;
	params.p_data = (uint8_t *)pVal;
	uint16_t len = (uint16_t)Len;
	params.p_len = &len;

	if (!BtGattTxPendingAdd(hdl, pChar))
	{
		return false;
	}

	uint32_t err = sd_ble_gatts_hvx(hdl, &params);
	if (err == NRF_SUCCESS)
	{
		return true;
	}

	BtGattTxPendRelease(hdl);
	return false;
}

bool BtGattCharIndicate(uint16_t ConnHdl, BtGattChar_t *pChar,
							void * const pVal, size_t Len)
{
	if (pChar == nullptr || (Len > 0 && pVal == nullptr) ||
		Len > UINT16_MAX || Len > pChar->MaxDataLen ||
		pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	uint16_t hdl = ConnHdl != BLE_CONN_HANDLE_INVALID ? ConnHdl : s_ConnHandle;
	if (hdl == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

	BtDevice_t *pConn = BtPeerFindByHdl(hdl);
	if (pConn == nullptr || pConn->Conn.bIndCfmPending)
	{
		return false;
	}

	ble_gatts_hvx_params_t params;
	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_INDICATION;
	params.handle = pChar->ValHdl;
	params.p_data = (uint8_t *)pVal;
	uint16_t len = (uint16_t)Len;
	params.p_len = &len;

	uint32_t err = sd_ble_gatts_hvx(hdl, &params);
	if (err != NRF_SUCCESS)
	{
		return false;
	}

	pConn->Conn.bIndCfmPending = true;
	pConn->Conn.IndCfmTime = BtGattMsTick();
	pConn->pIndChar = pChar;
	return true;
}

bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar == nullptr || (Len > 0 && pVal == nullptr) ||
		Len > UINT16_MAX || Len > pChar->MaxDataLen ||
		pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	ble_gatts_value_t value;
	memset(&value, 0, sizeof(value));
	value.offset = 0;
	value.len = (uint16_t)Len;
	value.p_value = (uint8_t *)pVal;

	return sd_ble_gatts_value_set(s_ConnHandle, pChar->ValHdl, &value) ==
			NRF_SUCCESS;
}

void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc)
{
	(void)pSrvc;
}

static bool GatherLongWrBuff(const uint8_t *pBuff, uint16_t BuffSize,
							 uint16_t Handle, uint8_t *pOut,
							 uint16_t OutSize, uint16_t *pOffset,
							 uint16_t *pLen)
{
	if (pBuff == nullptr || pOut == nullptr || pOffset == nullptr ||
		pLen == nullptr || BuffSize < sizeof(GATLWRHDR))
	{
		return false;
	}

	uint16_t used = 0;
	uint16_t count = 0;
	bool terminated = false;

	// Validate the complete queue first. No output and no source bytes are
	// modified until every record header and payload fits inside the block.
	while ((uint32_t)used + sizeof(GATLWRHDR) <= BuffSize)
	{
		GATLWRHDR hdr;
		memcpy(&hdr, pBuff + used, sizeof(hdr));
		if (hdr.Handle == 0 && hdr.Offset == 0 && hdr.Len == 0)
		{
			terminated = true;
			break;
		}

		if (count >= BT_GATT_LONG_WRITE_RECORD_MAX ||
			(uint32_t)hdr.Len >
				(uint32_t)BuffSize - used - sizeof(GATLWRHDR))
		{
			return false;
		}

		count++;
		used = (uint16_t)(used + sizeof(GATLWRHDR) + hdr.Len);
	}

	if (!terminated && used != BuffSize)
	{
		return false;
	}

	bool found = false;
	uint16_t firstOffset = 0;
	uint16_t total = 0;
	uint16_t cur = 0;

	while (cur < used)
	{
		GATLWRHDR hdr;
		memcpy(&hdr, pBuff + cur, sizeof(hdr));
		if (hdr.Handle == Handle)
		{
			if (!found)
			{
				found = true;
				firstOffset = hdr.Offset;
			}
			else if ((uint32_t)firstOffset + total != hdr.Offset)
			{
				return false;
			}

			if ((uint32_t)total + hdr.Len > OutSize)
			{
				return false;
			}
			total = (uint16_t)(total + hdr.Len);
		}
		cur = (uint16_t)(cur + sizeof(GATLWRHDR) + hdr.Len);
	}

	if (!found)
	{
		return false;
	}

	cur = 0;
	uint16_t dst = 0;
	while (cur < used)
	{
		GATLWRHDR hdr;
		memcpy(&hdr, pBuff + cur, sizeof(hdr));
		if (hdr.Handle == Handle && hdr.Len > 0)
		{
			memcpy(pOut + dst, pBuff + cur + sizeof(GATLWRHDR), hdr.Len);
			dst = (uint16_t)(dst + hdr.Len);
		}
		cur = (uint16_t)(cur + sizeof(GATLWRHDR) + hdr.Len);
	}

	*pOffset = firstOffset;
	*pLen = total;
	return true;
}

void BtGattSrvcEvtHandler(BtGattSrvc_t * const pSrvc, uint32_t Evt,
							  void * const pCtx)
{
	(void)Evt;
	if (pSrvc == nullptr || pCtx == nullptr)
	{
		return;
	}

	ble_evt_t *pBleEvt = (ble_evt_t *)pCtx;
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
				pSrvc->pCharArray[i].bIndic = false;
			}
			break;

		case BLE_GATTS_EVT_WRITE:
		{
			ble_gatts_evt_write_t *pWrite =
				&pBleEvt->evt.gatts_evt.params.write;

			if (pWrite->op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
			{
				BtDevice_t *pConn = BtPeerFindByHdl(
					pBleEvt->evt.gatts_evt.conn_handle);
				const uint8_t *pQueue = pConn != nullptr ?
					pConn->Conn.pLongWrBuff : nullptr;
				uint16_t queueSize = pConn != nullptr ?
					pConn->Conn.LongWrBuffSize : 0;

				for (int i = 0; i < pSrvc->NbChar; i++)
				{
					BtGattChar_t *pChar = &pSrvc->pCharArray[i];
					if (pChar->WrCB == nullptr || pQueue == nullptr)
					{
						continue;
					}

					uint16_t offset = 0;
					uint16_t len = 0;
					if (GatherLongWrBuff(pQueue, queueSize, pChar->ValHdl,
							s_LongWrScratch, sizeof(s_LongWrScratch),
							&offset, &len))
					{
						pChar->WrCB(pChar, s_LongWrScratch, offset, len);
					}
				}
				break;
			}

			for (int i = 0; i < pSrvc->NbChar; i++)
			{
				BtGattChar_t *pChar = &pSrvc->pCharArray[i];
				if (pWrite->handle == pChar->CccdHdl && pWrite->len == 2)
				{
					uint16_t cccd = (uint16_t)(pWrite->data[0] |
						((uint16_t)pWrite->data[1] << 8));
					bool notify = (cccd &
						BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0;
					bool indicate = (cccd &
						BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0;
					pChar->bNotify = notify;
					pChar->bIndic = indicate;

					if (pChar->SetNotifCB != nullptr)
					{
						pChar->SetNotifCB(pChar, notify,
							pBleEvt->evt.gatts_evt.conn_handle);
					}
					if (pChar->SetIndCB != nullptr)
					{
						pChar->SetIndCB(pChar, indicate,
							pBleEvt->evt.gatts_evt.conn_handle);
					}
					break;
				}

				if (pWrite->handle == pChar->ValHdl && pChar->WrCB != nullptr)
				{
					pChar->WrCB(pChar, pWrite->data, pWrite->offset,
						pWrite->len);
					break;
				}
			}
			break;
		}

		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
			// Authorization is routed by the application when configured.
			break;

#if (NRF_SD_BLE_API_VERSION > 3)
		case BLE_GATTS_EVT_HVN_TX_COMPLETE:
#else
		case BLE_EVT_TX_COMPLETE:
#endif
			// The global handler drains the per-link send-order ring once.
			break;

		default:
			break;
	}
}

static void BtSrvcEncSec(ble_gap_conn_sec_mode_t *pSecMode,
						 BTSRVC_SECTYPE SecType)
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

static uint32_t BtGattCharAdd(BtGattSrvc_t *pSrvc, BtGattChar_t *pChar,
									 BTSRVC_SECTYPE SecType)
{
	ble_gatts_char_md_t charMd;
	ble_gatts_attr_md_t cccdMd;
	ble_gatts_attr_md_t attrMd;
	ble_gatts_attr_t attrValue;
	ble_uuid_t uuid;
	memset(&charMd, 0, sizeof(charMd));
	memset(&cccdMd, 0, sizeof(cccdMd));
	memset(&attrMd, 0, sizeof(attrMd));
	memset(&attrValue, 0, sizeof(attrValue));

	cccdMd.vloc = BLE_GATTS_VLOC_STACK;
	charMd.p_char_user_desc = (uint8_t *)pChar->pDesc;
	if (pChar->pDesc != nullptr)
	{
		charMd.char_user_desc_max_size = (uint16_t)(strlen(pChar->pDesc) + 1U);
		charMd.char_user_desc_size = charMd.char_user_desc_max_size;
	}

	if (pChar->Property &
		(BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
	{
		charMd.char_props.notify =
			(pChar->Property & BT_GATT_CHAR_PROP_NOTIFY) != 0;
		charMd.char_props.indicate =
			(pChar->Property & BT_GATT_CHAR_PROP_INDICATE) != 0;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.read_perm);
		BtSrvcEncSec(&cccdMd.write_perm, SecType);
		charMd.p_cccd_md = &cccdMd;
	}

	charMd.char_props.broadcast =
		(pChar->Property & BT_GATT_CHAR_PROP_BROADCAST) != 0;
	charMd.char_props.read =
		(pChar->Property & BT_GATT_CHAR_PROP_READ) != 0;
	charMd.char_props.write =
		(pChar->Property & BT_GATT_CHAR_PROP_WRITE) != 0;
	charMd.char_props.write_wo_resp =
		(pChar->Property & BT_GATT_CHAR_PROP_WRITE_WORESP) != 0;

	if (charMd.char_props.read)
	{
		BtSrvcEncSec(&attrMd.read_perm, SecType);
	}
	else
	{
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attrMd.read_perm);
	}

	if (charMd.char_props.write || charMd.char_props.write_wo_resp)
	{
		BtSrvcEncSec(&attrMd.write_perm, SecType);
	}
	else
	{
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attrMd.write_perm);
	}

	uuid.type = pSrvc->Uuid.BaseIdx;
	uuid.uuid = pChar->Uuid;
	attrMd.vloc = BLE_GATTS_VLOC_STACK;
	attrMd.rd_auth =
		(pChar->Property & BT_GATT_CHAR_PROP_AUTH_SIGNED) != 0;
	attrMd.wr_auth = attrMd.rd_auth;
	attrMd.vlen = 1;

	attrValue.p_uuid = &uuid;
	attrValue.p_attr_md = &attrMd;
	attrValue.init_offs = 0;
	attrValue.max_len = pChar->MaxDataLen;
	attrValue.init_len = pChar->ValueLen;
	attrValue.p_value = (uint8_t *)pChar->pValue;

	ble_gatts_char_handles_t handles;
	memset(&handles, 0, sizeof(handles));
	uint32_t err = sd_ble_gatts_characteristic_add(
		pSrvc->Hdl, &charMd, &attrValue, &handles);
	if (err != NRF_SUCCESS)
	{
		return err;
	}

	pChar->Hdl = handles.value_handle;
	pChar->ValHdl = handles.value_handle;
	pChar->DescHdl = handles.user_desc_handle;
	pChar->CccdHdl = handles.cccd_handle;
	pChar->SccdHdl = handles.sccd_handle;
	pChar->pSrvc = pSrvc;
	return NRF_SUCCESS;
}

bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	if (pSrvc == nullptr || pSrvc->pCharArray == nullptr || pSrvc->NbChar <= 0)
	{
		return false;
	}

	uint32_t err;
	ble_uuid_t uuid;
	pSrvc->Uuid.Type = BT_UUID_TYPE_16;
	pSrvc->Uuid.Uuid16 = pSrvc->UuidSrvc;

	if (pSrvc->bCustom)
	{
		(void)BtUuidAddBase(pSrvc->UuidBase);
		uint8_t type = 0;
		err = sd_ble_uuid_vs_add((ble_uuid128_t *)pSrvc->UuidBase, &type);
		if (err != NRF_SUCCESS)
		{
			return false;
		}
		pSrvc->Uuid.BaseIdx = type;
	}
	else
	{
		pSrvc->Uuid.BaseIdx = BLE_UUID_TYPE_BLE;
	}

	uuid.type = pSrvc->Uuid.BaseIdx;
	uuid.uuid = pSrvc->UuidSrvc;
	err = sd_ble_gatts_service_add(
		BLE_GATTS_SRVC_TYPE_PRIMARY, &uuid, &pSrvc->Hdl);
	if (err != NRF_SUCCESS)
	{
		return false;
	}

	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		BtGattChar_t *pChar = &pSrvc->pCharArray[i];
		BTSRVC_SECTYPE sec = (BTSRVC_SECTYPE)(
			pChar->SecType != BT_GAP_SECTYPE_NONE ?
			pChar->SecType : pSrvc->SecType);
		if (BtGattCharAdd(pSrvc, pChar, sec) != NRF_SUCCESS)
		{
			return false;
		}
		pChar->bNotify = false;
		pChar->bIndic = false;
	}

	BtGattInsertSrvcList(pSrvc);
	return true;
}

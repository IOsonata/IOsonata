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

#include <nrf_error.h>
#include <ble.h>
#include <ble_gap.h>
#include <ble_gatts.h>

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gatt_init.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"

/******** For DEBUG ************/
#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

#pragma pack(push, 1)
typedef enum {
	BTSRVC_SECTYPE_NONE,
	BTSRVC_SECTYPE_STATICKEY_NO_MITM,
	BTSRVC_SECTYPE_STATICKEY_MITM,
	BTSRVC_SECTYPE_LESC_MITM,
	BTSRVC_SECTYPE_SIGNED_NO_MITM,
	BTSRVC_SECTYPE_SIGNED_MITM,
	BTSRVC_SECTYPE_OPEN,		//!< Matches BT_GAP_SECTYPE_OPEN
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

#ifndef BT_GATT_LONG_WRITE_SCRATCH_MAX
#define BT_GATT_LONG_WRITE_SCRATCH_MAX	1024
#endif

alignas(4) static uint8_t s_LongWrScratch[BT_GATT_LONG_WRITE_SCRATCH_MAX];
static uint8_t s_EmptyValue;

class BtGattInitGuard {
public:
	~BtGattInitGuard()
	{
		if (!vSuccess)
		{
			BtGattInitStatusFail(BT_GATT_INIT_ERROR_SERVICE_ADD);
		}
	}

	void Success() { vSuccess = true; }

private:
	bool vSuccess = false;
};

static bool BtGattBmValueLenValid(const BtDevice_t *pConn, size_t Len)
{
	if (pConn == nullptr)
	{
		return false;
	}

	uint16_t mtu = pConn->Conn.MaxMtu >= BT_ATT_MTU_MIN ?
			pConn->Conn.MaxMtu : BT_ATT_MTU_MIN;
	return Len <= (size_t)mtu - 3U;
}

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar,
						  void * const pVal, size_t Len)
{
	if (pChar == nullptr || ConnHdl == BLE_CONN_HANDLE_INVALID ||
		(Len > 0 && pVal == nullptr) || Len > UINT16_MAX ||
		Len > pChar->MaxDataLen || pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (!BtGattBmValueLenValid(pConn, Len))
	{
		return false;
	}

	ble_gatts_hvx_params_t params;
	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_NOTIFICATION;
	params.handle = pChar->ValHdl;
	params.p_data = Len > 0 ? (uint8_t *)pVal : &s_EmptyValue;
	uint16_t len = (uint16_t)Len;
	params.p_len = &len;

	if (!BtGattTxPendingAdd(ConnHdl, pChar))
	{
		return false;
	}

	uint32_t err = sd_ble_gatts_hvx(ConnHdl, &params);
	if (err == NRF_SUCCESS && len == Len)
	{
		return true;
	}

	BtGattTxPendRelease(ConnHdl);
	return false;
}

bool BtGattCharIndicate(uint16_t ConnHdl, BtGattChar_t *pChar,
							void * const pVal, size_t Len)
{
	if (pChar == nullptr || ConnHdl == BLE_CONN_HANDLE_INVALID ||
		(Len > 0 && pVal == nullptr) || Len > UINT16_MAX ||
		Len > pChar->MaxDataLen || pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (!BtGattBmValueLenValid(pConn, Len) || pConn->Conn.bIndCfmPending)
	{
		return false;
	}

	ble_gatts_hvx_params_t params;
	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_INDICATION;
	params.handle = pChar->ValHdl;
	params.p_data = Len > 0 ? (uint8_t *)pVal : &s_EmptyValue;
	uint16_t len = (uint16_t)Len;
	params.p_len = &len;

	uint32_t err = sd_ble_gatts_hvx(ConnHdl, &params);
	if (err != NRF_SUCCESS || len != Len)
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
	value.p_value = Len > 0 ? (uint8_t *)pVal : &s_EmptyValue;

	return sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
			pChar->ValHdl, &value) == NRF_SUCCESS;
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
	if (pSrvc == nullptr || pCtx == nullptr)
	{
		return;
	}

	ble_evt_t *pBleEvt = (ble_evt_t *)pCtx;
	DEBUG_PRINTF("BtGattSrvcEvtHandler: 0x%x, 0x%x\r\n",
		Evt, pBleEvt->header.evt_id);

	switch (pBleEvt->header.evt_id)
	{
		case BLE_GAP_EVT_DISCONNECTED:
			BtGattCccdClear(pBleEvt->evt.gap_evt.conn_handle);
			break;

		case BLE_GATTS_EVT_WRITE:
		{
			ble_gatts_evt_write_t *pWrite =
				&pBleEvt->evt.gatts_evt.params.write;
			uint16_t connHdl = pBleEvt->evt.gatts_evt.conn_handle;

			if (pWrite->op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
			{
				BtDevice_t *pConn = BtPeerFindByHdl(connHdl);
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
					(void)BtGattCccdSet(connHdl, pChar->CccdHdl, cccd);
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
			break;

		case BLE_GATTS_EVT_HVN_TX_COMPLETE:
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
		case BTSRVC_SECTYPE_NONE:
		case BTSRVC_SECTYPE_OPEN:
		default:
			BLE_GAP_CONN_SEC_MODE_SET_OPEN(pSecMode);
			break;
	}
}

static uint32_t BtGattCharAdd(BtGattSrvc_t *pSrvc, BtGattChar_t *pChar,
									 BTSRVC_SECTYPE SecType,
									 uint8_t SoftDeviceUuidType)
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

	uuid.type = SoftDeviceUuidType;
	uuid.uuid = pChar->Uuid;
	attrMd.vloc = BLE_GATTS_VLOC_STACK;
	attrMd.vlen = 1;

	attrValue.p_uuid = &uuid;
	attrValue.p_attr_md = &attrMd;
	attrValue.init_offs = 0;
	attrValue.max_len = pChar->MaxDataLen;
	attrValue.init_len = pChar->ValueLen;
	attrValue.p_value = pChar->pValue != nullptr ?
		(uint8_t *)pChar->pValue : &s_EmptyValue;

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
	pChar->BaseUuidIdx = pSrvc->Uuid.BaseIdx;
	pChar->pSrvc = pSrvc;
	return NRF_SUCCESS;
}

bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	BtGattInitGuard initGuard;
	DEBUG_PRINTF("BtGattSrvcAdd[bm]: enter pSrvc=%p\r\n", (void *)pSrvc);
	if (pSrvc == nullptr || pSrvc->pCharArray == nullptr || pSrvc->NbChar <= 0)
	{
		return false;
	}

	for (BtGattSrvc_t *p = BtGattGetSrvcList(); p != nullptr; p = p->pNext)
	{
		if (p == pSrvc)
		{
			initGuard.Success();
			return true;
		}
	}

	if (!pSrvc->bCustom &&
		(pSrvc->UuidSrvc == BT_UUID_GATT_SERVICE_GENERIC_ACCESS ||
		 pSrvc->UuidSrvc == BT_UUID_GATT_SERVICE_GENERIC_ATTRIBUTE))
	{
		initGuard.Success();
		return true;
	}

	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		BtGattChar_t *pChar = &pSrvc->pCharArray[i];
		uint8_t sec = BtGattSecTypeGet(pSrvc, pChar, g_BtAppData.SecType);
		if ((pChar->Property & BT_GATT_CHAR_PROP_AUTH_SIGNED) != 0 ||
			sec == BT_GAP_SECTYPE_SIGNED_NO_MITM ||
			sec == BT_GAP_SECTYPE_SIGNED_MITM ||
			pChar->ValueLen > pChar->MaxDataLen ||
			(pChar->ValueLen > 0 && pChar->pValue == nullptr))
		{
			return false;
		}
	}

	uint32_t err;
	ble_uuid_t uuid;
	uint8_t softDeviceUuidType = BLE_UUID_TYPE_BLE;
	pSrvc->Uuid.Type = BT_UUID_TYPE_16;
	pSrvc->Uuid.Uuid16 = pSrvc->UuidSrvc;

	if (pSrvc->bCustom)
	{
		int baseIdx = BtUuidAddBase(pSrvc->UuidBase);
		if (baseIdx < 0)
		{
			return false;
		}
		pSrvc->Uuid.BaseIdx = (uint8_t)baseIdx;
		err = sd_ble_uuid_vs_add((ble_uuid128_t *)pSrvc->UuidBase,
			&softDeviceUuidType);
		if (err != NRF_SUCCESS)
		{
			return false;
		}
	}
	else
	{
		pSrvc->Uuid.BaseIdx = 0;
	}

	uuid.type = softDeviceUuidType;
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
		BTSRVC_SECTYPE sec = (BTSRVC_SECTYPE)BtGattSecTypeGet(pSrvc, pChar,
			g_BtAppData.SecType);
		if (BtGattCharAdd(pSrvc, pChar, sec,
			softDeviceUuidType) != NRF_SUCCESS)
		{
			return false;
		}
		pChar->bNotify = false;
		pChar->bIndic = false;
	}

	BtGattInsertSrvcList(pSrvc);
	DEBUG_PRINTF("BtGattSrvcAdd[bm]: OK Hdl=%d NbChar=%d\r\n",
		pSrvc->Hdl, pSrvc->NbChar);
	initGuard.Success();
	return true;
}

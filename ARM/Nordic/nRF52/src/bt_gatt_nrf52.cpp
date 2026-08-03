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

//#include "bluetooth/ble_srvc.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"

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

#ifndef BT_GATT_LONG_WRITE_RECORD_MAX
#define BT_GATT_LONG_WRITE_RECORD_MAX	32
#endif

static uint16_t s_ConnHandle = BLE_CONN_HANDLE_INVALID;
static BtGattSrvc_t *s_pBtGattSrvcHead = nullptr;
static BtGattSrvc_t *s_pBtGattSrvcTail = nullptr;

//uint32_t BleSrvcCharNotify(BtGattSrvc_t *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen)
bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar == nullptr)
	{
		return false;
	}

	if (Len > 0 && pVal == nullptr)
	{
		return false;
	}

	if (Len > UINT16_MAX || Len > pChar->MaxDataLen)
	{
		return false;
	}

	if (pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	// Use the caller-supplied link when valid; fall back to the last
	// connected handle for single-link callers. Matches the nRF54 pattern.
	uint16_t hdl = (ConnHdl != BLE_CONN_HANDLE_INVALID) ? ConnHdl : s_ConnHandle;

	if (hdl == BLE_CONN_HANDLE_INVALID)
	{
		return false;//NRF_ERROR_INVALID_STATE;
	}

//	if (pSrvc->pCharArray[Idx].bNotify == false)
//		return NRF_ERROR_INVALID_STATE;

	// No local CCCD gate. The SoftDevice tracks the CCCD per connection (and
	// restores it per bond) and sd_ble_gatts_hvx returns an error when this
	// connection has not enabled notification, so it is the authority.

    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = pChar->ValHdl;//pSrvc->pCharArray[Idx].ValHdl;//.value_handle;
    params.p_data = (uint8_t*)pVal;
    // SoftDevice expects p_len to point to a uint16_t (it may modify it).
    uint16_t l = (uint16_t)Len;
    params.p_len = &l;

    // Reserve the completion entry before the SoftDevice accepts the packet.
    // Otherwise a full ring sends an untracked notification and shifts every
    // later TxCompleteCB by one controller completion.
    if (BtGattTxPendingAdd(hdl, pChar) == false)
    {
        return false;
    }

    uint32_t err_code = sd_ble_gatts_hvx(hdl, &params);

    if (err_code == NRF_SUCCESS)
    {
        return true;
    }

    BtGattTxPendRelease(hdl);
    return false;
}

// Native indication path. The SoftDevice enforces the single-outstanding-
// indication rule (sd_ble_gatts_hvx returns NRF_ERROR_BUSY while one is pending).
// A successful send starts the per-link indication transaction timeout. Its
// TxCompleteCB belongs to BLE_GATTS_EVT_HVC, not BLE_GATTS_EVT_HVN_TX_COMPLETE.
bool BtGattCharIndicate(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar == nullptr)
	{
		return false;
	}

	if (Len > 0 && pVal == nullptr)
	{
		return false;
	}

	if (Len > UINT16_MAX || Len > pChar->MaxDataLen)
	{
		return false;
	}

	if (pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	uint16_t hdl = (ConnHdl != BLE_CONN_HANDLE_INVALID) ? ConnHdl : s_ConnHandle;

	if (hdl == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

	BtDevice_t *pConn = BtPeerFindByHdl(hdl);
	if (pConn == nullptr || pConn->Conn.bIndCfmPending)
	{
		return false;
	}

	// No local CCCD gate. sd_ble_gatts_hvx with BLE_GATT_HVX_INDICATION returns
	// an error when this connection has not enabled indication, so the
	// SoftDevice is the authority.

    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_INDICATION;
    params.handle = pChar->ValHdl;
    params.p_data = (uint8_t*)pVal;
    uint16_t l = (uint16_t)Len;
    params.p_len = &l;

    uint32_t err_code = sd_ble_gatts_hvx(hdl, &params);

    if (err_code == NRF_SUCCESS)
    {
        pConn->Conn.bIndCfmPending = true;
        pConn->Conn.IndCfmTime = BtGattMsTick();
        pConn->pIndChar = pChar;
        return true;
    }

    return false;
}

//uint32_t BleSrvcCharSetValue(BtGattSrvc_t *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen)
bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar == nullptr)
	{
		return false;
	}

	if (Len > 0 && pVal == nullptr)
	{
		return false;
	}

	if (Len > UINT16_MAX || Len > pChar->MaxDataLen)
	{
		return false;
	}

	if (pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	ble_gatts_value_t value;

    memset(&value, 0, sizeof(ble_gatts_value_t));

    value.offset = 0;
    value.len = Len;
    value.p_value = (uint8_t*)pVal;

    uint32_t err_code = sd_ble_gatts_value_set(s_ConnHandle,
    										   pChar->ValHdl,//.value_handle,
											   &value);
    return err_code == NRF_SUCCESS;
}

void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc)
{

}

// Validate the complete SoftDevice user-memory queue before reading or moving
// any payload. The fixed record table bounds stack use and rejects a queue with
// more fragments than this port supports instead of recursing without a limit.
static bool GatherLongWrBuff(uint8_t *pBuff, uint16_t BuffSize, uint16_t Handle,
							 uint16_t *pOffset, uint16_t *pLen)
{
	if (pBuff == nullptr || pOffset == nullptr || pLen == nullptr ||
		BuffSize < sizeof(GATLWRHDR))
	{
		return false;
	}

	uint16_t pos[BT_GATT_LONG_WRITE_RECORD_MAX];
	GATLWRHDR rec[BT_GATT_LONG_WRITE_RECORD_MAX];
	uint16_t count = 0;
	uint16_t cur = 0;

	while ((uint32_t)cur + sizeof(GATLWRHDR) <= BuffSize)
	{
		GATLWRHDR hdr;
		memcpy(&hdr, pBuff + cur, sizeof(hdr));

		// The unused tail of the user-memory block is zero-filled.
		if (hdr.Handle == 0 && hdr.Offset == 0 && hdr.Len == 0)
		{
			break;
		}

		if (count >= BT_GATT_LONG_WRITE_RECORD_MAX ||
			(uint32_t)hdr.Len > (uint32_t)BuffSize - cur - sizeof(GATLWRHDR))
		{
			return false;
		}

		pos[count] = cur;
		rec[count] = hdr;
		count++;
		cur = (uint16_t)(cur + sizeof(GATLWRHDR) + hdr.Len);
	}

	bool found = false;
	uint16_t offset = 0;
	uint16_t total = 0;
	uint16_t match[BT_GATT_LONG_WRITE_RECORD_MAX];
	uint16_t matchCount = 0;

	for (uint16_t i = 0; i < count; i++)
	{
		if (rec[i].Handle != Handle)
		{
			continue;
		}

		if (found == false)
		{
			found = true;
			offset = rec[i].Offset;
		}
		else if (rec[i].Offset != (uint16_t)(offset + total))
		{
			return false;
		}

		if ((uint32_t)total + rec[i].Len >
			(uint32_t)BuffSize - sizeof(GATLWRHDR))
		{
			return false;
		}

		match[matchCount++] = i;
		total = (uint16_t)(total + rec[i].Len);
	}

	if (found == false)
	{
		return false;
	}

	// Move from the last fragment backwards. Destinations are before their
	// sources; reverse order keeps every source intact until it has been read.
	uint16_t end = total;
	for (uint16_t n = matchCount; n > 0; n--)
	{
		uint16_t i = match[n - 1];
		end = (uint16_t)(end - rec[i].Len);
		memmove(pBuff + sizeof(GATLWRHDR) + end,
				pBuff + pos[i] + sizeof(GATLWRHDR), rec[i].Len);
	}

	*pOffset = offset;
	*pLen = total;
	return true;
}

void BtGattSrvcEvtHandler(BtGattSrvc_t * const pSrvc, uint32_t Evt, void * const pCtx)
{
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
        		pSrvc->pCharArray[i].bIndic  = false;
        	}
            break;

        case BLE_GATTS_EVT_WRITE:
			{
				ble_gatts_evt_write_t * p_evt_write = &pBleEvt->evt.gatts_evt.params.write;

				// Long write reassembly reads this link's per-connection buffer
				// (the memory handed to the SoftDevice at USER_MEM_REQUEST).
				BtDevice_t *pLwConn = BtPeerFindByHdl(pBleEvt->evt.gatts_evt.conn_handle);
				uint8_t *pLongWr = (pLwConn != nullptr) ? pLwConn->Conn.pLongWrBuff : nullptr;
				uint16_t longWrSize = (pLwConn != nullptr) ? pLwConn->Conn.LongWrBuffSize : 0;

				//g_Uart.printf("BLE_GATTS_EVT_WRITE: %d\r\n", pSrvc->NbChar);

				for (int i = 0; i < pSrvc->NbChar; i++)
				{
				    if (p_evt_write->op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
					//if (p_evt_write->handle == 0)
					{
						//g_Uart.printf("Long Write\r\n");
						if (pLongWr == nullptr || pSrvc->pCharArray[i].WrCB == nullptr)
						{
							continue;
						}

						uint16_t offset = 0;
						uint16_t len = 0;
						if (GatherLongWrBuff(pLongWr, longWrSize,
								pSrvc->pCharArray[i].ValHdl, &offset, &len))
					    {
							pSrvc->pCharArray[i].WrCB(&pSrvc->pCharArray[i],
								pLongWr + sizeof(GATLWRHDR), offset, len);
					    }
					}
					else
					{
						if ((p_evt_write->handle == pSrvc->pCharArray[i].CccdHdl) && //cccd_handle) &&
							(p_evt_write->len == 2))
						{
							uint16_t cccd = (uint16_t)(p_evt_write->data[0] |
													(p_evt_write->data[1] << 8));
							bool notify = (cccd & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0;
							bool indicate = (cccd & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0;

							pSrvc->pCharArray[i].bNotify = notify;
							pSrvc->pCharArray[i].bIndic = indicate;

							if (pSrvc->pCharArray[i].SetNotifCB)
							{
								pSrvc->pCharArray[i].SetNotifCB(&pSrvc->pCharArray[i],
										notify, pBleEvt->evt.gatts_evt.conn_handle);
							}
							if (pSrvc->pCharArray[i].SetIndCB)
							{
								pSrvc->pCharArray[i].SetIndCB(&pSrvc->pCharArray[i],
										indicate, pBleEvt->evt.gatts_evt.conn_handle);
							}
						}
						else if ((p_evt_write->handle == pSrvc->pCharArray[i].ValHdl) &&//.value_handle) &&
								 (pSrvc->pCharArray[i].WrCB != NULL))
						{
							//g_Uart.printf("Write value handle\r\n");
							pSrvc->pCharArray[i].WrCB(&pSrvc->pCharArray[i], p_evt_write->data, 0, p_evt_write->len);
						}
						else
						{
							// Do Nothing. This event is not relevant for this service.
						}
					}
				}
			}
            break;

        // BLE_EVT_USER_MEM_REQUEST is now handled once per connection in
        // ble_evt_dispatch (bt_app_nrf52), using Conn.pLongWrBuff.

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        	if (pSrvc->AuthReqCB)
        	{
//        		pSrvc->AuthReqCB(pSrvc, pBleEvt);
        	}
        	break;

#if (NRF_SD_BLE_API_VERSION > 3)
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
#else
        case BLE_EVT_TX_COMPLETE:

#endif
        	{
//        		uint16_t hdl = BtGapGetConnection();

//				if (hdl == pBleEvt->evt.gatts_evt.conn_handle)
				{
					// HVN TX complete gives only a count, not a handle. It is
					// handled once per event in the global handler through
					// BtGattSendCompleted, which drains the send-order ring.
				}
        	}
            break;

        default:
            break;
    }
}

void BtGattEvtHandler(uint32_t Evt)
{

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

    if (pChar->Property & (BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
    {
    	if (pChar->Property & BT_GATT_CHAR_PROP_NOTIFY)
    	{
    		char_md.char_props.notify = 1;
    	}
    	if (pChar->Property & BT_GATT_CHAR_PROP_INDICATE)
    	{
    		char_md.char_props.indicate = 1;
    	}
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    	BtSrvcEncSec(&cccd_md.write_perm, SecType);
    	BtSrvcEncSec(&attr_md.read_perm, SecType);
        char_md.p_cccd_md = &cccd_md;
    }

    if (pChar->Property & BT_GATT_CHAR_PROP_BROADCAST)
    {
    	char_md.char_props.broadcast   = 1;
    }
    if (pChar->Property & BT_GATT_CHAR_PROP_READ)
    {
    	char_md.char_props.read   = 1;
    	BtSrvcEncSec(&attr_md.read_perm, SecType);
       // BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    }
    else
    {
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    }

    if (pChar->Property & (BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_WRITE_WORESP))
    {
        if (pChar->Property & BT_GATT_CHAR_PROP_WRITE)
            char_md.char_props.write  = 1;
    	if (pChar->Property & BT_GATT_CHAR_PROP_WRITE_WORESP)
            char_md.char_props.write_wo_resp = 1;

    	BtSrvcEncSec(&attr_md.write_perm, SecType);
        //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    }
    else
    {
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    }

    ble_uuid.type = pSrvc->Uuid.BaseIdx;//Type;//[pChar->BaseUuidIdx];
    ble_uuid.uuid = pChar->Uuid;

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;

    if (pChar->Property & BT_GATT_CHAR_PROP_AUTH_SIGNED)
    {
    	attr_md.rd_auth    = 1;
    }
    else
    {
    	attr_md.rd_auth    = 0;
    }

    if (pChar->Property & BT_GATT_CHAR_PROP_AUTH_SIGNED)
    {
    	attr_md.wr_auth    = 1;
    }
    else
    {
    	attr_md.wr_auth    = 0;
    }

    // Variable-length attribute values are the default; the spec treats
    // value length as naturally variable and most characteristics need
    // this. attr_md.vlen is a SoftDevice-internal switch (not in the BT
    // spec); set it unconditionally so user-facing declarations don't
    // have to know about it.
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = pChar->MaxDataLen;//CharVal.MaxLen;
    attr_char_value.init_len     = pChar->ValueLen;
    attr_char_value.p_value      = (uint8_t*)pChar->pValue;

    ble_gatts_char_handles_t hdl;
    uint32_t res = sd_ble_gatts_characteristic_add(pSrvc->Hdl, &char_md, &attr_char_value, &hdl);
    pChar->Hdl     = hdl.value_handle;
    pChar->ValHdl  = hdl.value_handle;
    pChar->DescHdl = hdl.user_desc_handle;
    pChar->CccdHdl = hdl.cccd_handle;
    pChar->SccdHdl = hdl.sccd_handle;
    pChar->pSrvc   = pSrvc;

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

    // IOsonata standard service declarations are zero-initialized. SoftDevice UUID type 0 is invalid; adopted 16-bit UUIDs use the Bluetooth SIG base type.
    if (pSrvc->bCustom == false)
    {
        pSrvc->Uuid.BaseIdx = BLE_UUID_TYPE_BLE;
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
        BTSRVC_SECTYPE sec = (BTSRVC_SECTYPE)(pSrvc->pCharArray[i].SecType != BT_GAP_SECTYPE_NONE
                             ? pSrvc->pCharArray[i].SecType : pSrvc->SecType);
        err = BtGattCharAdd(pSrvc, &pSrvc->pCharArray[i], sec);
        if (err != NRF_SUCCESS)
        {
            return false;
        }
        pSrvc->pCharArray[i].bNotify = false;
        pSrvc->pCharArray[i].bIndic  = false;
    }

    BtGattInsertSrvcList(pSrvc);

    /*
    if (s_pBtGattSrvcHead == nullptr)
    {
    	s_pBtGattSrvcHead = s_pBtGattSrvcTail = pSrvc;
    	pSrvc->pPrev = pSrvc->pNext = nullptr;
    }
    else
    {
    	s_pBtGattSrvcTail->pNext = pSrvc;
    	pSrvc->pPrev = s_pBtGattSrvcTail;
    	pSrvc->pNext = nullptr;
    	s_pBtGattSrvcTail = pSrvc;
    }
*/
    return true;
}

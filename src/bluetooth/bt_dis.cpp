/**-------------------------------------------------------------------------
@file	bt_dis.cpp

@brief	Bluetooth Device Information Service (DIS) - generic implementation.

        See bt_dis.h for details. This module builds the DIS GATT service
        using only the spec-standard generic GATT layer (BtGattSrvcAdd /
        BtGattCharSetValue). No SDK-specific dependencies.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dis.h"

// Per BLE-spec DIS char value limit is 22 octets for the string chars. Apps
// can override to allow longer model/serial strings if their ATT MTU permits.
#ifndef BT_DIS_STR_MAX_LEN
#define BT_DIS_STR_MAX_LEN		22
#endif

// Backing storage for the PnP ID record. The GATT layer reads from this
// pointer on every client read, so it must outlive the service.
static BtDisPnpId_t s_BtDisPnpId = { 0, };

// Index constants for s_BtDisChar[] - keep in sync with the array order.
enum {
	BT_DIS_CHAR_IDX_MANUF_NAME = 0,
	BT_DIS_CHAR_IDX_MODEL_NUM,
	BT_DIS_CHAR_IDX_SERIAL_NUM,
	BT_DIS_CHAR_IDX_FW_REV,
	BT_DIS_CHAR_IDX_HW_REV,
	BT_DIS_CHAR_IDX_SW_REV,
	BT_DIS_CHAR_IDX_PNP_ID,
	BT_DIS_CHAR_COUNT,
};

static BtGattChar_t s_BtDisChar[] = {
	// Manufacturer Name String (0x2A29)
	{
		.Uuid = BT_UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
		.MaxDataLen = BT_DIS_STR_MAX_LEN,
		.Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_VALEN,
		.pDesc = NULL,
		.WrCB = NULL,
		.SetNotifCB = NULL,
		.TxCompleteCB = NULL,
		.Hdl = BT_ATT_HANDLE_INVALID,
		.ValHdl = BT_ATT_HANDLE_INVALID,
		.DescHdl = BT_ATT_HANDLE_INVALID,
		.CccdHdl = BT_ATT_HANDLE_INVALID,
		.SccdHdl = BT_ATT_HANDLE_INVALID,
	},
	// Model Number String (0x2A24)
	{
		.Uuid = BT_UUID_CHARACTERISTIC_MODEL_NUMBER_STRING,
		.MaxDataLen = BT_DIS_STR_MAX_LEN,
		.Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_VALEN,
		.pDesc = NULL,
		.WrCB = NULL,
		.SetNotifCB = NULL,
		.TxCompleteCB = NULL,
		.Hdl = BT_ATT_HANDLE_INVALID,
		.ValHdl = BT_ATT_HANDLE_INVALID,
		.DescHdl = BT_ATT_HANDLE_INVALID,
		.CccdHdl = BT_ATT_HANDLE_INVALID,
		.SccdHdl = BT_ATT_HANDLE_INVALID,
	},
	// Serial Number String (0x2A25)
	{
		.Uuid = BT_UUID_CHARACTERISTIC_SERIAL_NUMBER_STRING,
		.MaxDataLen = BT_DIS_STR_MAX_LEN,
		.Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_VALEN,
		.pDesc = NULL,
		.WrCB = NULL,
		.SetNotifCB = NULL,
		.TxCompleteCB = NULL,
		.Hdl = BT_ATT_HANDLE_INVALID,
		.ValHdl = BT_ATT_HANDLE_INVALID,
		.DescHdl = BT_ATT_HANDLE_INVALID,
		.CccdHdl = BT_ATT_HANDLE_INVALID,
		.SccdHdl = BT_ATT_HANDLE_INVALID,
	},
	// Firmware Revision String (0x2A26)
	{
		.Uuid = BT_UUID_CHARACTERISTIC_FIRMWARE_REVISION_STRING,
		.MaxDataLen = BT_DIS_STR_MAX_LEN,
		.Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_VALEN,
		.pDesc = NULL,
		.WrCB = NULL,
		.SetNotifCB = NULL,
		.TxCompleteCB = NULL,
		.Hdl = BT_ATT_HANDLE_INVALID,
		.ValHdl = BT_ATT_HANDLE_INVALID,
		.DescHdl = BT_ATT_HANDLE_INVALID,
		.CccdHdl = BT_ATT_HANDLE_INVALID,
		.SccdHdl = BT_ATT_HANDLE_INVALID,
	},
	// Hardware Revision String (0x2A27)
	{
		.Uuid = BT_UUID_CHARACTERISTIC_HARDWARE_REVISION_STRING,
		.MaxDataLen = BT_DIS_STR_MAX_LEN,
		.Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_VALEN,
		.pDesc = NULL,
		.WrCB = NULL,
		.SetNotifCB = NULL,
		.TxCompleteCB = NULL,
		.Hdl = BT_ATT_HANDLE_INVALID,
		.ValHdl = BT_ATT_HANDLE_INVALID,
		.DescHdl = BT_ATT_HANDLE_INVALID,
		.CccdHdl = BT_ATT_HANDLE_INVALID,
		.SccdHdl = BT_ATT_HANDLE_INVALID,
	},
	// Software Revision String (0x2A28). BtAppDevInfo_t does not yet carry a
	// software revision field; this char is reserved here for future use.
	{
		.Uuid = BT_UUID_CHARACTERISTIC_SOFTWARE_REVISION_STRING,
		.MaxDataLen = BT_DIS_STR_MAX_LEN,
		.Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_VALEN,
		.pDesc = NULL,
		.WrCB = NULL,
		.SetNotifCB = NULL,
		.TxCompleteCB = NULL,
		.Hdl = BT_ATT_HANDLE_INVALID,
		.ValHdl = BT_ATT_HANDLE_INVALID,
		.DescHdl = BT_ATT_HANDLE_INVALID,
		.CccdHdl = BT_ATT_HANDLE_INVALID,
		.SccdHdl = BT_ATT_HANDLE_INVALID,
	},
	// PnP ID (0x2A50) - fixed 7 bytes.
	{
		.Uuid = BT_UUID_CHARACTERISTIC_PNP_ID,
		.MaxDataLen = sizeof(BtDisPnpId_t),
		.Property = BT_GATT_CHAR_PROP_READ,
		.pDesc = NULL,
		.WrCB = NULL,
		.SetNotifCB = NULL,
		.TxCompleteCB = NULL,
		.Hdl = BT_ATT_HANDLE_INVALID,
		.ValHdl = BT_ATT_HANDLE_INVALID,
		.DescHdl = BT_ATT_HANDLE_INVALID,
		.CccdHdl = BT_ATT_HANDLE_INVALID,
		.SccdHdl = BT_ATT_HANDLE_INVALID,
	},
};

static const BtGattSrvcCfg_t s_BtDisSrvcCfg = {
	.bCustom = false,
	.UuidBase = { 0, },
	.UuidSrvc = BT_UUID_GATT_SERVICE_DEVICE_INFORMATION,
	.NbChar = BT_DIS_CHAR_COUNT,
	.pCharArray = s_BtDisChar,
};

static BtGattSrvc_t s_BtDisSrvc;

// Set a string characteristic value if the source string is non-NULL and
// non-empty. Length is bounded by BT_DIS_STR_MAX_LEN.
static void SetStrCharValue(BtGattChar_t *pChar, const char *pStr)
{
	if (pStr == NULL || pStr[0] == 0)
	{
		return;
	}

	size_t l = strlen(pStr);
	if (l > BT_DIS_STR_MAX_LEN)
	{
		l = BT_DIS_STR_MAX_LEN;
	}
	BtGattCharSetValue(pChar, (void *)pStr, l);
}

__attribute__((weak)) bool BtDisInit(const struct __Bt_App_Cfg *pCfgIn)
{
	const BtAppCfg_t *pCfg = (const BtAppCfg_t *)pCfgIn;

	if (pCfg == NULL)
	{
		return false;
	}

	if (BtGattSrvcAdd(&s_BtDisSrvc, &s_BtDisSrvcCfg) == false)
	{
		return false;
	}

	// String chars - all optional. The ModelName and ManufName fields in
	// BtAppDevInfo_t are inline char arrays (not pointers), so empty means
	// the first byte is zero.
	if (pCfg->pDevInfo != NULL)
	{
		SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_MANUF_NAME],
		                pCfg->pDevInfo->ManufName);
		SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_MODEL_NUM],
		                pCfg->pDevInfo->ModelName);
		SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_SERIAL_NUM],
		                pCfg->pDevInfo->pSerialNoStr);
		SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_FW_REV],
		                pCfg->pDevInfo->pFwVerStr);
		SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_HW_REV],
		                pCfg->pDevInfo->pHwVerStr);
		// SW Rev currently has no source field; left empty.
	}

	// PnP ID - always populated from the app cfg PnP fields. Source defaults
	// to BT SIG; apps using a USB-IF vendor ID can override after this call
	// via BtGattCharSetValue on the PnP ID char.
	s_BtDisPnpId.VendorIdSrc = BT_DIS_PNP_VENDOR_ID_SRC_BT_SIG;
	s_BtDisPnpId.VendorId    = pCfg->VendorId;
	s_BtDisPnpId.ProductId   = pCfg->ProductId;
	s_BtDisPnpId.ProductVer  = pCfg->ProductVer;
	BtGattCharSetValue(&s_BtDisChar[BT_DIS_CHAR_IDX_PNP_ID],
	                   &s_BtDisPnpId, sizeof(s_BtDisPnpId));

	return true;
}

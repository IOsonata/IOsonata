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

Copyright (c) 2026, I-SYST inc. All rights reserved.

----------------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gatt_init.h"
#include "bluetooth/services/bt_dis.h"

#ifndef BT_DIS_STR_MAX_LEN
#define BT_DIS_STR_MAX_LEN		22
#endif

// The PnP ID characteristic value is seven octets: one for the vendor id
// source, then three 16 bit fields little endian. The struct is written
// straight to the characteristic, so its packed size is the wire length.
static_assert(sizeof(BtDisPnpId_t) == 7, "PnP ID must be 7 octets");

static BtDisPnpId_t s_BtDisPnpId = {};
static uint8_t s_BtDisEmptyValue;
static bool s_BtDisRegistered = false;

// Software Revision sits last so the characteristic count can drop it without
// disturbing the others. A device that flashes one image and boots into it has
// no software revision distinct from its firmware revision, and an empty
// characteristic on air is worse than an absent optional one.
enum {
	BT_DIS_CHAR_IDX_MANUF_NAME = 0,
	BT_DIS_CHAR_IDX_MODEL_NUM,
	BT_DIS_CHAR_IDX_SERIAL_NUM,
	BT_DIS_CHAR_IDX_FW_REV,
	BT_DIS_CHAR_IDX_HW_REV,
	BT_DIS_CHAR_IDX_PNP_ID,
	BT_DIS_CHAR_COUNT_NO_SW,
	BT_DIS_CHAR_IDX_SW_REV = BT_DIS_CHAR_COUNT_NO_SW,
	BT_DIS_CHAR_COUNT,
};

// Device information is what a peer reads to find out what it is talking to,
// so these are OPEN rather than inheriting the application security. Zephyr
// declares every DIS characteristic with plain BT_GATT_PERM_READ and the
// Nordic SDK service offers no way to secure them either.
//
// The serial number is the exception. It identifies one unit rather than a
// product, so it inherits: a device configured with security does not hand it
// to anyone who connects.
static BtGattChar_t s_BtDisChar[] = {
	BT_CHAR(BT_UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
	        BT_DIS_STR_MAX_LEN, BT_GATT_CHAR_PROP_READ, NULL,
	        .SecType = BT_GAP_SECTYPE_OPEN),
	BT_CHAR(BT_UUID_CHARACTERISTIC_MODEL_NUMBER_STRING,
	        BT_DIS_STR_MAX_LEN, BT_GATT_CHAR_PROP_READ, NULL,
	        .SecType = BT_GAP_SECTYPE_OPEN),
	BT_CHAR(BT_UUID_CHARACTERISTIC_SERIAL_NUMBER_STRING,
	        BT_DIS_STR_MAX_LEN, BT_GATT_CHAR_PROP_READ, NULL,
	        .SecType = BT_GAP_SECTYPE_NONE),
	BT_CHAR(BT_UUID_CHARACTERISTIC_FIRMWARE_REVISION_STRING,
	        BT_DIS_STR_MAX_LEN, BT_GATT_CHAR_PROP_READ, NULL,
	        .SecType = BT_GAP_SECTYPE_OPEN),
	BT_CHAR(BT_UUID_CHARACTERISTIC_HARDWARE_REVISION_STRING,
	        BT_DIS_STR_MAX_LEN, BT_GATT_CHAR_PROP_READ, NULL,
	        .SecType = BT_GAP_SECTYPE_OPEN),
	BT_CHAR(BT_UUID_CHARACTERISTIC_PNP_ID,
	        sizeof(BtDisPnpId_t), BT_GATT_CHAR_PROP_READ, NULL,
	        .SecType = BT_GAP_SECTYPE_OPEN),
	BT_CHAR(BT_UUID_CHARACTERISTIC_SOFTWARE_REVISION_STRING,
	        BT_DIS_STR_MAX_LEN, BT_GATT_CHAR_PROP_READ, NULL,
	        .SecType = BT_GAP_SECTYPE_OPEN),
};

static BtGattSrvc_t s_BtDisSrvc = BT_SRVC_STD(BT_UUID_GATT_SERVICE_DEVICE_INFORMATION,
                                              s_BtDisChar);

// Set or clear the current value. Clearing matters on first registration,
// because generic GATT allocates MaxDataLen bytes, and on reinitialisation,
// because an absent field must not leave the previous value readable.
static bool SetStrCharValue(BtGattChar_t *pChar, const char *pStr)
{
	if (pStr == NULL || pStr[0] == 0)
	{
		// Some vendor GATT APIs require a valid value pointer even when the
		// requested length is zero. Keep the semantic value empty without
		// passing a null pointer across the port boundary.
		return BtGattCharSetValue(pChar, &s_BtDisEmptyValue, 0);
	}

	size_t l = strlen(pStr);
	if (l > BT_DIS_STR_MAX_LEN)
	{
		l = BT_DIS_STR_MAX_LEN;
	}
	return BtGattCharSetValue(pChar, (void *)pStr, l);
}

// Existing applications use Bluetooth SIG company identifiers. A product using
// a USB-IF VID overrides this weak hook and returns
// BT_DIS_PNP_VENDOR_ID_SRC_USB_IF. Invalid values fall back to Bluetooth SIG.
__attribute__((weak)) uint8_t BtDisVendorIdSource(const struct __Bt_App_Cfg *pCfg)
{
	(void)pCfg;
	return BT_DIS_PNP_VENDOR_ID_SRC_BT_SIG;
}

__attribute__((weak)) bool BtDisInit(const struct __Bt_App_Cfg *pCfgIn)
{
	const BtAppCfg_t *pCfg = (const BtAppCfg_t *)pCfgIn;

	if (pCfg == NULL)
	{
		BtGattInitStatusFail(BT_GATT_INIT_ERROR_INVALID_CFG);
		return false;
	}

	if ((pCfg->Role & BTAPP_ROLE_PERIPHERAL) == 0)
	{
		return true;
	}

	const BtAppDevInfo_t *pInfo = pCfg->pDevInfo;
	const char *pSwVer = pInfo != NULL ? pInfo->pSwVerStr : NULL;
	bool hasSw = pSwVer != NULL && pSwVer[0] != 0;

	// The characteristic count is decided once, before the service reaches a
	// port, because every port keeps the pointer and builds its database from
	// it. A later init cannot grow or shrink what is already on air, so it
	// only refreshes values.
	if (s_BtDisRegistered == false)
	{
		s_BtDisSrvc.NbChar = hasSw ?
			BT_DIS_CHAR_COUNT : BT_DIS_CHAR_COUNT_NO_SW;
	}

	if (BtGattSrvcAdd(&s_BtDisSrvc) == false)
	{
		BtGattInitStatusFail(BT_GATT_INIT_ERROR_SERVICE_ADD);
		return false;
	}
	s_BtDisRegistered = true;

	bool ok = true;
	ok = SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_MANUF_NAME],
		pInfo != NULL ? pInfo->ManufName : NULL) && ok;
	ok = SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_MODEL_NUM],
		pInfo != NULL ? pInfo->ModelName : NULL) && ok;
	ok = SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_SERIAL_NUM],
		pInfo != NULL ? pInfo->pSerialNoStr : NULL) && ok;
	ok = SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_FW_REV],
		pInfo != NULL ? pInfo->pFwVerStr : NULL) && ok;
	ok = SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_HW_REV],
		pInfo != NULL ? pInfo->pHwVerStr : NULL) && ok;
	// Only when the characteristic is part of the registered service. It is
	// cleared rather than skipped when a later init drops the software
	// revision, so the previous value does not stay readable.
	if (s_BtDisSrvc.NbChar > BT_DIS_CHAR_COUNT_NO_SW)
	{
		ok = SetStrCharValue(&s_BtDisChar[BT_DIS_CHAR_IDX_SW_REV],
			hasSw ? pSwVer : NULL) && ok;
	}

	uint8_t source = BtDisVendorIdSource(pCfgIn);
	if (source != BT_DIS_PNP_VENDOR_ID_SRC_BT_SIG &&
		source != BT_DIS_PNP_VENDOR_ID_SRC_USB_IF)
	{
		source = BT_DIS_PNP_VENDOR_ID_SRC_BT_SIG;
	}
	s_BtDisPnpId.VendorIdSrc = source;
	s_BtDisPnpId.VendorId    = pCfg->VendorId;
	s_BtDisPnpId.ProductId   = pCfg->ProductId;
	s_BtDisPnpId.ProductVer  = pCfg->ProductVer;
	ok = BtGattCharSetValue(&s_BtDisChar[BT_DIS_CHAR_IDX_PNP_ID],
	                   &s_BtDisPnpId, sizeof(s_BtDisPnpId)) && ok;

	if (!ok)
	{
		BtGattInitStatusFail(BT_GATT_INIT_ERROR_VALUE_SET);
	}
	return ok;
}

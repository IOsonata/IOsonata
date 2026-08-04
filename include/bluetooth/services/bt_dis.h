/**-------------------------------------------------------------------------
@file	bt_dis.h

@brief	Bluetooth Device Information Service (DIS) - service UUID 0x180A.

        Generic spec-standard implementation. Builds the standard GATT service
        via BtGattSrvcAdd, so works on any port that uses the generic GATT layer
        (SDC, and any future host-side port). Ports that rely on vendor
        middleware currently use their own BtDisInit and bypass this module.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc. All rights reserved

----------------------------------------------------------------------------*/
#ifndef __BT_DIS_H__
#define __BT_DIS_H__

#include <stdint.h>
#include <stdbool.h>

#include "bluetooth/bt_uuid.h"

#define BT_DIS_PNP_VENDOR_ID_SRC_BT_SIG     1
#define BT_DIS_PNP_VENDOR_ID_SRC_USB_IF     2

#pragma pack(push, 1)
typedef struct __Bt_Dis_Pnp_Id {
	uint8_t  VendorIdSrc;	//!< 1 = Bluetooth SIG, 2 = USB IF
	uint16_t VendorId;		//!< Vendor identifier (BT SIG or USB IF assigned)
	uint16_t ProductId;		//!< Vendor-assigned product identifier
	uint16_t ProductVer;	//!< Vendor-assigned product version (BCD JJMN)
} BtDisPnpId_t;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

struct __Bt_App_Cfg;

/**
 * Return the source namespace for the PnP Vendor ID. The weak default returns
 * BT_DIS_PNP_VENDOR_ID_SRC_BT_SIG. Products using a USB-IF VID override this
 * and return BT_DIS_PNP_VENDOR_ID_SRC_USB_IF.
 */
uint8_t BtDisVendorIdSource(const struct __Bt_App_Cfg *pCfg);

/**
 * Initialize the generic Device Information Service. Missing or empty strings
 * remain registered with a zero-length value, including after reinitialization.
 */
bool BtDisInit(const struct __Bt_App_Cfg *pCfg);

#ifdef __cplusplus
}
#endif

#endif // __BT_DIS_H__

/**-------------------------------------------------------------------------
@file	bt_dis.h

@brief	Bluetooth Device Information Service (DIS) - service UUID 0x180A.

        Generic spec-standard implementation. Builds the standard GATT service
        via BtGattSrvcAdd, so works on any port that uses the generic GATT layer
        (SDC, and any future host-side port). Ports that rely on vendor
        middleware (nRF5_SDK ble_dis, sdk-nrf-bm ble_dis) currently use their
        own BtDisInit and bypass this module.

        Characteristics exposed (all read-only, all optional per BLE spec):
            0x2A24 Model Number String
            0x2A25 Serial Number String
            0x2A26 Firmware Revision String
            0x2A27 Hardware Revision String
            0x2A28 Software Revision String
            0x2A29 Manufacturer Name String
            0x2A50 PnP ID

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __BT_DIS_H__
#define __BT_DIS_H__

#include <stdint.h>
#include <stdbool.h>

#include "bluetooth/bt_uuid.h"

// PnP ID Vendor ID Source field values (0x2A50).
#define BT_DIS_PNP_VENDOR_ID_SRC_BT_SIG     1
#define BT_DIS_PNP_VENDOR_ID_SRC_USB_IF     2

// PnP ID record - packed 7 bytes per BLE spec.
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

// Forward decl. Full type lives in bt_app.h.
// Using the struct tag avoids pulling bt_app.h here, which keeps the
// include graph clean.
struct __Bt_App_Cfg;

/**
 * @brief Initialize the generic Device Information Service.
 *
 *        Reads pCfg->pDevInfo for the string characteristics and
 *        pCfg->VendorId / ProductId / ProductVer for the PnP ID. Strings
 *        that are NULL or empty result in an empty characteristic value
 *        (the char is still registered; clients reading it get zero bytes).
 *
 *        The strings pointed to by pCfg->pDevInfo must remain valid for the
 *        lifetime of the service.
 *
 * @param   pCfg    Application config providing device info and PnP fields.
 * @return  true on success; false on registration failure.
 */
bool BtDisInit(const struct __Bt_App_Cfg *pCfg);

#ifdef __cplusplus
}
#endif

#endif // __BT_DIS_H__

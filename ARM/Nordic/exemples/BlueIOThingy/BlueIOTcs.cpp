/**-------------------------------------------------------------------------
@file	BlueIOTcf.cpp

@brief	Thingy Configuration Service implementation

@author Hoang Nguyen Hoan
@date	Jul 14, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#include <inttypes.h>

#include "bluetooth/bt_gatt.h"
#include "board.h"
#include "BlueIOThingy.h"

// TCS (Configuration service characteristics)

#define BLE_UUID_TCS_DEVICE_NAME_CHAR		0x0101                      /**< The UUID of the device name Characteristic. */
#define BLE_UUID_TCS_ADV_PARAMS_CHAR		0x0102                      /**< The UUID of the advertising parameters Characteristic. */
#define BLE_UUID_TCS_APPEARANCE_CHAR		0x0103                      /**< The UUID of the appearance Characteristic. */
#define BLE_UUID_TCS_CONN_PARAM_CHAR		0x0104                      /**< The UUID of the connection parameters Characteristic. */
#define BLE_UUID_TCS_BEACON_PARAM_CHAR		0x0105                      /**< The UUID of the beacon Characteristic. */
#define BLE_UUID_TCS_CLOUD_PARAM_CHAR		0x0106                      /**< The UUID of the cloud token Characteristic. */
#define BLE_UUID_TCS_FW_VERSION_CHAR		0x0107                      /**< The UUID of the FW version Characteristic. */
#define BLE_UUID_TCS_MTU_CHAR				0x0108                      /**< The UUID of the MTU Characteristic. */

#define THINGY_TCS_FW_VERSIO_CHAR_IDX   	0//5

#define BLE_TCS_MAX_DATA_LEN 				(20) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Thingy Configuration service module. */

#pragma pack(push, 1)
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} ble_tcs_fw_version_t;
#pragma pack(pop)

const ble_tcs_fw_version_t s_ThingyVersion {
	 2, 2, 0
};

BtGattChar_t g_ConfChars[] = {
    {
        // Version characteristic.  This is the minimum required for Thingy App to work
		.Uuid = BLE_UUID_TCS_FW_VERSION_CHAR,
        .MaxDataLen = BLE_TCS_MAX_DATA_LEN,
        .Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_VALEN,
        .pDesc = NULL,    					// char UTF-8 description string
        .WrCB = NULL,                       // Callback for write char, set to NULL for read char
        .SetNotifCB = NULL,                       // Callback on set notification
        .TxCompleteCB = NULL,                       // Tx completed callback
		.pValue = (void*)&s_ThingyVersion,                       // pointer to char default values
        3,                          // Default value length in bytes
    },
};

/// Service definition
const BtGattSrvcCfg_t s_ConfSrvcCfg = {
   // BLESRVC_SECTYPE_NONE,       	// Secure or Open service/char
	.bCustom = true,
    .UuidBase = THINGY_BASE_UUID,           	// Base UUID
//	1,
    .UuidSrvc = BLE_UUID_TCS_SERVICE,       	// Service UUID
    .NbChar = sizeof(g_ConfChars) / sizeof(BtGattChar_t),  // Total number of characteristics for the service
    .pCharArray = g_ConfChars,                 	// Pointer a an array of characteristic
    NULL,                       	// pointer to user long write buffer
    0,                           	// long write buffer size
	NULL,							// Authentication event callback
};

/// TCS instance
BtGattSrvc_t g_ConfSrvc;

BtGattSrvc_t *GetConfSrvcInstance()
{
	return &g_ConfSrvc;
}

uint32_t ConfSrvcInit()
{
	return BtGattSrvcAdd(&g_ConfSrvc, &s_ConfSrvcCfg);
}


/**-------------------------------------------------------------------------
@file	BlueIOTes.cpp

@brief	Thingy Environmental Sensor Service implementation

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
#include "bluetooth/bt_app.h"
#include "board.h"
#include "BlueIOThingy.h"

#define BLE_UUID_TES_SERVICE          	0x0200

#define BLE_UUID_TES_TEMPERATURE_CHAR	0x0201                      /**< The UUID of the temperature Characteristic. */
#define BLE_UUID_TES_PRESSURE_CHAR    	0x0202                      /**< The UUID of the pressure Characteristic. */
#define BLE_UUID_TES_HUMIDITY_CHAR    	0x0203                      /**< The UUID of the humidity Characteristic. */
#define BLE_UUID_TES_GAS_CHAR         	0x0204                      /**< The UUID of the gas Characteristic. */
#define BLE_UUID_TES_COLOR_CHAR       	0x0205                      /**< The UUID of the gas Characteristic. */
#define BLE_UUID_TES_CONFIG_CHAR      	0x0206                      /**< The UUID of the config Characteristic. */

//#define THINGY_TES_CONFIGCHAR_IDX   0
#define THINGY_TES_TEMPCHAR_IDX     0
#define THINGY_TES_PRESCHAR_IDX     1
#define THINGY_TES_HUMICHAR_IDX     2

#define BLE_TES_MAX_DATA_LEN 		(20)       /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Thingy Environment service module. */

#pragma pack(push, 1)

typedef struct {
    int8_t  integer;
    uint8_t decimal;
} ble_tes_temperature_t;

typedef struct {
    int32_t  integer;
    uint8_t  decimal;
} ble_tes_pressure_t;

typedef struct {
    uint8_t  led_red;
    uint8_t  led_green;
    uint8_t  led_blue;
} ble_tes_color_config_t;

#pragma pack(pop)

static const char s_EnvTempCharDescString[] = {
        "Temperature characteristic",
};

static const char s_EnvPresCharDescString[] = {
        "Pressure characteristic",
};

static const char s_EnvHumCharDescString[] = {
        "Humidity characteristic",
};

static uint8_t s_EnvCharTemp[BLE_TES_MAX_DATA_LEN];
static uint8_t s_EnvCharPres[BLE_TES_MAX_DATA_LEN];
static uint8_t s_EnvCharHumi[BLE_TES_MAX_DATA_LEN];

/// Characteristic definitions
BtGattChar_t g_EnvChars[] = {
    {
        // Temperature characteristic
        .Uuid 			= BLE_UUID_TES_TEMPERATURE_CHAR,
        .MaxDataLen 	= BLE_TES_MAX_DATA_LEN,
        .Property 		= BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
        .pDesc 			= s_EnvTempCharDescString,    // char UTF-8 description string
        .WrCB 			= NULL,                       // Callback for write char, set to NULL for read char
        .SetNotifCB 	= NULL,                       // Callback on set notification
        .SetIndCB 		= NULL,                       // Tx completed callback
		.TxCompleteCB	= NULL,
		.pValue			= s_EnvCharTemp,                       // pointer to char default values
		.ValueLen		= 0,                          // Default value length in bytes
    },
    {
        // Pressure characteristic
		.Uuid 			= BLE_UUID_TES_PRESSURE_CHAR, // char UUID
		.MaxDataLen 	= BLE_TES_MAX_DATA_LEN,       // char max data length
		.Property 		= BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		.pDesc 			= s_EnvPresCharDescString,    // char UTF-8 description string
		.WrCB 			= NULL,                       // Callback for write char, set to NULL for read char
		.SetNotifCB 	= NULL,                       // Callback on set notification
		.SetIndCB 		= NULL,                       // Tx completed callback
		.TxCompleteCB	= NULL,
//		s_EnvCharPres,                       // pointer to char default values
//        0                           // Default value length in bytes
    },
    {
        // Humidity characteristic
		.Uuid 			= BLE_UUID_TES_HUMIDITY_CHAR, // char UUID
		.MaxDataLen 	= BLE_TES_MAX_DATA_LEN,       // char max data length
		.Property 		= BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		.pDesc 			= s_EnvHumCharDescString,     // char UTF-8 description string
		.WrCB 			= NULL,                       // Callback for write char, set to NULL for read char
		.SetNotifCB 	= NULL,                       // Callback on set notification
		.SetIndCB 		= NULL,                       // Tx completed callback
		.TxCompleteCB	= NULL,
//		s_EnvCharHumi,                       // pointer to char default values
//       0                           // Default value length in bytes
    },
};

/// Service definition
const BtGattSrvcCfg_t s_EnvSrvcCfg = {
    //BLESRVC_SECTYPE_NONE,       // Secure or Open service/char
	.SecType = 0,
	.bCustom = true,
    .UuidBase = THINGY_BASE_UUID,        // Base UUID
    .UuidSrvc = BLE_UUID_TES_SERVICE,       // Service UUID
    .NbChar = sizeof(g_EnvChars) / sizeof(BtGattChar_t),  // Total number of characteristics for the service
    .pCharArray = g_EnvChars,                 // Pointer a an array of characteristic
   // NULL,                       // pointer to user long write buffer
   // 0,                          // long write buffer size
//	NULL
};

BtGattSrvc_t g_EnvSrvc;

BtGattSrvc_t *GetEnvSrvcInstance()
{
	return &g_EnvSrvc;
}

uint32_t EnvSrvcInit()
{
	return BtGattSrvcAdd(&g_EnvSrvc, &s_EnvSrvcCfg);
}

void EnvSrvcNotifTemp(float Temp)
{
    ble_tes_temperature_t t;


    t.integer = (int)Temp;
    t.decimal = (uint8_t)((Temp - (float)t.integer) * 100.0);

	BtAppNotify(&g_EnvSrvc.pCharArray[THINGY_TES_TEMPCHAR_IDX], (uint8_t*)&t, sizeof(t));
}


void EnvSrvcNotifPressure(float Press)
{
    ble_tes_pressure_t b;

    b.integer = (int)Press;
    b.decimal = (uint8_t)((Press - (float)b.integer) * 100.0);

    BtGattSrvc_t *p = GetEnvSrvcInstance();
    BtAppNotify(&p->pCharArray[THINGY_TES_PRESCHAR_IDX], (uint8_t*)&b, sizeof(b));
}

void EnvSrvcNotifHumi(uint8_t Humi)
{
	BtGattSrvc_t *p = GetEnvSrvcInstance();
	BtAppNotify(&p->pCharArray[THINGY_TES_HUMICHAR_IDX], &Humi, sizeof(Humi));
}




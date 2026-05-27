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

/// Characteristic definitions
BtGattChar_t g_EnvChars[] = {
	// Temperature
	BT_CHAR(BLE_UUID_TES_TEMPERATURE_CHAR, BLE_TES_MAX_DATA_LEN,
	        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
	        s_EnvTempCharDescString),
	// Pressure
	BT_CHAR(BLE_UUID_TES_PRESSURE_CHAR, BLE_TES_MAX_DATA_LEN,
	        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
	        s_EnvPresCharDescString),
	// Humidity
	BT_CHAR(BLE_UUID_TES_HUMIDITY_CHAR, BLE_TES_MAX_DATA_LEN,
	        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
	        s_EnvHumCharDescString),
};

BtGattSrvc_t g_EnvSrvc = BT_SRVC_CUSTOM(THINGY_BASE_UUID,
                                        BLE_UUID_TES_SERVICE,
                                        g_EnvChars);

BtGattSrvc_t *GetEnvSrvcInstance()
{
	return &g_EnvSrvc;
}

uint32_t EnvSrvcInit()
{
	return BtGattSrvcAdd(&g_EnvSrvc);
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




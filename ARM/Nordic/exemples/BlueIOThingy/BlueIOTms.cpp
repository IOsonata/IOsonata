/**-------------------------------------------------------------------------
@file	BlueIOTms.cpp

@brief	Thingy Motion Sensor Service implementation

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
#include "bluetooth/bt_intrf.h"
#include "bluetooth/bt_app.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"
#include "board.h"
#include "BlueIOThingy.h"
#include "BlueIOMPU9250.h"

void ImuConfCharWrhandler(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);
void ImuTapCharSetNotify(BtGattChar_t *pChar, bool bEnable);
void ImuOrientCharSetNotify(BtGattChar_t *pChar, bool bEnable);
void ImuQuaternionCharSetNotify(BtGattChar_t *pChar, bool bEnable);
void ImuPedometerCharSetNotify(BtGattChar_t *pChar, bool bEnable);
void ImuRawCharSetNotify(BtGattChar_t *pChar, bool bEnable);
void ImuEulerCharSetNotify(BtGattChar_t *pChar, bool bEnable);
void ImuRotMatCharSetNotify(BtGattChar_t *pChar, bool bEnable);
void ImuHeadingCharSetNotify(BtGattChar_t *pChar, bool bEnable);
void ImuGravityCharSetNotify(BtGattChar_t *pChar, bool bEnable);

#define BLE_UUID_TMS_SERVICE 0x0400                      /**< The UUID of the Motion Service. */

#define BLE_UUID_TMS_CONFIG_CHAR      0x0401                      /**< The UUID of the config Characteristic. */
#define BLE_UUID_TMS_TAP_CHAR         0x0402                      /**< The UUID of the tap Characteristic. */
#define BLE_UUID_TMS_ORIENTATION_CHAR 0x0403                      /**< The UUID of the orientation Characteristic. */
#define BLE_UUID_TMS_QUATERNION_CHAR  0x0404                      /**< The UUID of the quaternion Characteristic. */
#define BLE_UUID_TMS_PEDOMETER_CHAR   0x0405                      /**< The UUID of the pedometer Characteristic. */
#define BLE_UUID_TMS_RAW_CHAR         0x0406                      /**< The UUID of the raw data Characteristic. */
#define BLE_UUID_TMS_EULER_CHAR       0x0407                      /**< The UUID of the euler Characteristic. */
#define BLE_UUID_TMS_ROT_MAT_CHAR     0x0408                      /**< The UUID of the rotation matrix Characteristic. */
#define BLE_UUID_TMS_HEADING_CHAR     0x0409                      /**< The UUID of the compass heading Characteristic. */
#define BLE_UUID_TMS_GRAVITY_CHAR     0x040A                      /**< The UUID of the gravity vector Characteristic. */

#define BLE_TMS_MAX_DATA_LEN (20) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Motion service module. */

#pragma pack(push, 1)

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ble_tms_raw_accel_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ble_tms_raw_gyro_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ble_tms_raw_compass_t;

typedef struct {
    ble_tms_raw_accel_t   accel;
    ble_tms_raw_gyro_t    gyro;
    ble_tms_raw_compass_t compass;
} ble_tms_raw_t;

typedef struct {
    uint8_t dir;
    uint8_t cnt;
} ble_tms_tap_t;

typedef uint8_t ble_tms_orientation_t;

typedef struct {
    int32_t w;
    int32_t x;
    int32_t y;
    int32_t z;
} ble_tms_quat_t;

typedef struct {
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
} ble_tms_euler_t;

typedef struct {
    int16_t matrix[9];
} ble_tms_rot_mat_t;

typedef int32_t ble_tms_heading_t;

typedef struct {
    float x;
    float y;
    float z;
} ble_tms_gravity_t;

typedef struct {
    uint32_t steps;
    uint32_t time_ms;
} ble_tms_pedo_t;

typedef struct {
    uint16_t pedo_interval_ms;
    uint16_t temp_interval_ms;
    uint16_t compass_interval_ms;
    uint16_t motion_freq_hz;
    uint8_t  wake_on_motion;
} ble_tms_config_t;

#pragma pack(pop)

#define IMUCHAR_IDX_CONFIG					0
#define IMUCHAR_IDX_TAP						1
#define IMUCHAR_IDX_ORIENT					2
#define IMUCHAR_IDX_QUAT					3
#define IMUCHAR_IDX_PEDO					4
#define IMUCHAR_IDX_RAW						5
#define IMUCHAR_IDX_EULER					6
#define IMUCHAR_IDX_ROTMAT					7
#define IMUCHAR_IDX_HEADING					8
#define IMUCHAR_IDX_GRAVITY					9


static const char s_ImuConfigCharDescString[] = {
        "Config characteristic",
};

static const char s_ImuTapCharDescString[] = {
        "Tap characteristic",
};

static const char s_ImuOrientCharDescString[] = {
        "Orientation characteristic",
};

static const char s_ImuQuatCharDescString[] = {
        "Quaternion characteristic",
};

static const char s_ImuPedoCharDescString[] = {
        "Pedometer characteristic",
};

static const char s_ImuRawCharDescString[] = {
        "Raw data characteristic",
};

static const char s_ImuEulerCharDescString[] = {
        "Euler characteristic",
};

static const char s_ImuRotCharDescString[] = {
        "Rotation matrix characteristic",
};

static const char s_ImuHeadCharDescString[] = {
        "Compass heading characteristic",
};

static const char s_ImuGravCharDescString[] = {
        "Gravity vector characteristic",
};

static uint8_t s_ImuCharCfgData[BLE_TMS_MAX_DATA_LEN];
static uint8_t s_ImuCharTapData[BLE_TMS_MAX_DATA_LEN];
static uint8_t s_ImuCharOriData[BLE_TMS_MAX_DATA_LEN];
static uint8_t s_ImuCharQuatData[BLE_TMS_MAX_DATA_LEN];
static uint8_t s_ImuCharPedData[BLE_TMS_MAX_DATA_LEN];
static uint8_t s_ImuCharRawData[BLE_TMS_MAX_DATA_LEN];
static uint8_t s_ImuCharEuData[BLE_TMS_MAX_DATA_LEN];
static uint8_t s_ImuCharRotData[BLE_TMS_MAX_DATA_LEN];
static uint8_t s_ImuCharHeadData[BLE_TMS_MAX_DATA_LEN];
static uint8_t s_ImuCharGravData[BLE_TMS_MAX_DATA_LEN];

/// Characteristic definitions
BtGattChar_t g_ImuChars[] = {
    {
        // Config characteristic
    	BLE_UUID_TMS_CONFIG_CHAR,
		BLE_TMS_MAX_DATA_LEN,
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_VALEN,
		s_ImuConfigCharDescString,  // char UTF-8 description string
		ImuConfCharWrhandler,       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
		NULL,
        NULL,                       // Tx completed callback
		s_ImuCharCfgData,           // pointer to char default values
        0,                          // Default value length in bytes
    },
    {
        // TAP characteristic
		BLE_UUID_TMS_TAP_CHAR, 		// char UUID
		BLE_TMS_MAX_DATA_LEN,       // char max data length
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		s_ImuTapCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ImuTapCharSetNotify,        // Callback on set notification
		NULL,						// Set indication
        NULL,                       // Tx completed callback
		s_ImuCharTapData,           // pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Orientation characteristic
		BLE_UUID_TMS_ORIENTATION_CHAR, // char UUID
		BLE_TMS_MAX_DATA_LEN,       // char max data length
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		s_ImuOrientCharDescString,  // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ImuOrientCharSetNotify,		// Callback on set notification
		NULL,						// Set indication
        NULL,                       // Tx completed callback
		s_ImuCharOriData,			// pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Quaternion characteristic
		BLE_UUID_TMS_QUATERNION_CHAR, // char UUID
		BLE_TMS_MAX_DATA_LEN,       // char max data length
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		s_ImuQuatCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ImuQuaternionCharSetNotify,	// Callback on set notification
		NULL,						// Set indication
		NULL,                       // Tx completed callback
		s_ImuCharQuatData,			// pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Pedometer characteristic
		BLE_UUID_TMS_PEDOMETER_CHAR, // char UUID
		BLE_TMS_MAX_DATA_LEN,       // char max data length
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		s_ImuPedoCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ImuPedometerCharSetNotify,	// Callback on set notification
		NULL,						// Set indication
        NULL,                       // Tx completed callback
		s_ImuCharPedData,			// pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Raw characteristic
		BLE_UUID_TMS_RAW_CHAR, // char UUID
		BLE_TMS_MAX_DATA_LEN,       // char max data length
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		s_ImuRawCharDescString,    	// char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ImuRawCharSetNotify,		// Callback on set notification
		NULL,						// Set indication
        NULL,                       // Tx completed callback
		s_ImuCharRawData,			// pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Euler characteristic
		BLE_UUID_TMS_EULER_CHAR, // char UUID
		BLE_TMS_MAX_DATA_LEN,       // char max data length
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		s_ImuEulerCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ImuEulerCharSetNotify,      // Callback on set notification
		NULL,						// Set indication
        NULL,                       // Tx completed callback
		s_ImuCharEuData,            // pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Rot matrix characteristic
		BLE_UUID_TMS_ROT_MAT_CHAR, // char UUID
		BLE_TMS_MAX_DATA_LEN,       // char max data length
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		s_ImuRotCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ImuRotMatCharSetNotify,     // Callback on set notification
		NULL,						// Set indication
        NULL,                       // Tx completed callback
		s_ImuCharRotData,           // pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Heading characteristic
		BLE_UUID_TMS_HEADING_CHAR, // char UUID
		BLE_TMS_MAX_DATA_LEN,       // char max data length
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		s_ImuHeadCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ImuHeadingCharSetNotify,	// Callback on set notification
		NULL,						// Set indication
        NULL,                       // Tx completed callback
		s_ImuCharHeadData,          // pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Gravity characteristic
		BLE_UUID_TMS_GRAVITY_CHAR, // char UUID
		BLE_TMS_MAX_DATA_LEN,       // char max data length
        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		s_ImuGravCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ImuGravityCharSetNotify,	// Callback on set notification
		NULL,						// Set indication
        NULL,                       // Tx completed callback
		s_ImuCharGravData,          // pointer to char default values
        0                           // Default value length in bytes
    },
};

/// Service definition
const BtGattSrvcCfg_t s_ImuSrvcCfg = {
    0,       // Secure or Open service/char
	true,
    THINGY_BASE_UUID,        // Base UUID
	BLE_UUID_TMS_SERVICE,       // Service UUID
    sizeof(g_ImuChars) / sizeof(BtGattChar_t),  // Total number of characteristics for the service
    g_ImuChars,                 // Pointer a an array of characteristic
    NULL,                       // pointer to user long write buffer
    0,                          // long write buffer size
	NULL
};

BtGattSrvc_t g_ImuSrvc;

static const BtIntrfCfg_t s_ImuQuatIntrfCfg = {
	&g_ImuSrvc,
};

BtGattSrvc_t *GetImuSrvcInstance()
{
	return &g_ImuSrvc;
}

uint32_t ImuSrvcInit()
{
	return BtGattSrvcAdd(&g_ImuSrvc, &s_ImuSrvcCfg);
}

void ImuConfCharWrhandler(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len)
{

}

void ImuTapCharSetNotify(BtGattChar_t *pChar, bool bEnable)
{
	g_ImuChars[IMUCHAR_IDX_TAP].bNotify = true;
	MPU9250EnableFeature(MPU9250_MOTION_FEATURE_TAP);
}

void ImuOrientCharSetNotify(BtGattChar_t *pChar, bool bEnable)
{
	g_ImuChars[IMUCHAR_IDX_ORIENT].bNotify = true;
	MPU9250EnableFeature(MPU9250_MOTION_FEATURE_ORIENTATION);
}

void ImuQuaternionCharSetNotify(BtGattChar_t *pChar, bool bEnable)
{
	g_ImuChars[IMUCHAR_IDX_QUAT].bNotify = true;
	MPU9250EnableFeature(MPU9250_MOTION_FEATURE_QUAT);
}

void ImuQuatDataSend(long Quat[4])
{
	if (g_ImuChars[IMUCHAR_IDX_QUAT].bNotify == false)
		return;

	long q[4];
#if 0
	q[0] = Quat[0];
	q[1] = Quat[2];
	q[2] = Quat[1];
	q[3] = -Quat[3];
#else
	q[0] = Quat[0];
	q[1] = Quat[1];
	q[2] = Quat[2];
	q[3] = Quat[3];
#endif

	uint32_t err = BtAppNotify(&g_ImuSrvc.pCharArray[IMUCHAR_IDX_QUAT], (uint8_t*)q, sizeof(long) * 4);
	if (err != 0)
	{
		//printf("quat Error %x\r\n", err);
	}
}

void ImuPedometerCharSetNotify(BtGattChar_t *pChar, bool bEnable)
{
	g_ImuChars[IMUCHAR_IDX_PEDO].bNotify = true;
	MPU9250EnableFeature(MPU9250_MOTION_FEATURE_PEDOMETER);
}

void ImuRawCharSetNotify(BtGattChar_t *pChar, bool bEnable)
{
	g_ImuChars[IMUCHAR_IDX_RAW].bNotify = true;
	MPU9250EnableFeature(MPU9250_MOTION_FEATURE_RAW);
}

void ImuRawDataSend(ACCELSENSOR_DATA &AccData, GYROSENSOR_DATA GyroData, MAGSENSOR_DATA &MagData)
{
	ble_tms_raw_t raw;

	raw.accel.x = AccData.X / 256;
	raw.accel.y = AccData.Y / 256;
	raw.accel.z = AccData.Z / 256;

	raw.gyro.x = GyroData.X / 256;
	raw.gyro.y = GyroData.Y / 256;
	raw.gyro.z = GyroData.Z / 256;

	raw.compass.x = MagData.X / 256;
	raw.compass.y = MagData.Y / 256;
	raw.compass.z = MagData.Z / 256;

	uint32_t err = BtAppNotify(&GetImuSrvcInstance()->pCharArray[5], (uint8_t*)&raw, sizeof(ble_tms_raw_t));
	if (err != 0)
	{
		///printf("raw Error %x\r\n", err);
	}
}

void ImuEulerCharSetNotify(BtGattChar_t *pChar, bool bEnable)
{
	g_ImuChars[IMUCHAR_IDX_EULER].bNotify = true;
	MPU9250EnableFeature(MPU9250_MOTION_FEATURE_EULER);
}

void ImuRotMatCharSetNotify(BtGattChar_t *pChar, bool bEnable)
{
	g_ImuChars[IMUCHAR_IDX_ROTMAT].bNotify = true;
	MPU9250EnableFeature(MPU9250_MOTION_FEATURE_ROT_MAT);
}
void ImuHeadingCharSetNotify(BtGattChar_t *pChar, bool bEnable)
{
	g_ImuChars[IMUCHAR_IDX_HEADING].bNotify = true;
	MPU9250EnableFeature(MPU9250_MOTION_FEATURE_HEADING);
}

void ImuGravityCharSetNotify(BtGattChar_t *pChar, bool bEnable)
{
	g_ImuChars[IMUCHAR_IDX_GRAVITY].bNotify = true;
	MPU9250EnableFeature(MPU9250_MOTION_FEATURE_GRAVITY_VECTOR);
}


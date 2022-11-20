/**-------------------------------------------------------------------------
@file	bt_dev.h

@brief	Generic implementation of Bluetooth device.


@author	Hoang Nguyen Hoan
@date	Jan. 17, 2019

@license

MIT License

Copyright (c) 2019, I-SYST inc., all rights reserved

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

#ifndef __BT_DEV_H__
#define __BT_DEV_H__

#include <stdint.h>

#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"

#include "device.h"

/** @addtogroup Bluetooth
  * @{
  */

#ifndef BTDEV_NAME_MAXLEN
#define BTDEV_NAME_MAXLEN				20
#endif

#ifndef BTDEV_SERVICE_MAXCNT
#define BTDEV_SERVICE_MAXCNT			20
#endif

#ifndef BTDEV_DEFAULT_MAX_DATA_LEN
#define BTDEV_DEFAULT_MAX_DATA_LEN		251
#endif

typedef enum __Bt_Dev_Role {
	BTDEV_ROLE_BROADCASTER	= BT_GAP_ROLE_BROADCASTER,		//!< non connectable Advertising only
	BTDEV_ROLE_OBSERVER		= BT_GAP_ROLE_OBSERVER,			//!< non connectable central
	BTDEV_ROLE_PERIPHERAL	= BT_GAP_ROLE_PERIPHERAL,		//!< BLE connectable peripheral device
	BTDEV_ROLE_CENTRAL		= BT_GAP_ROLE_CENTRAL,			//!< BLE Central device
	BTDEV_ROLE_MIXED		= BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL	//!< Mixed central/peripheral
} BTDEV_ROLE;

typedef enum __Bt_Dev_Coex_Mode {
	BTDEV_COEXMODE_NONE,			//!< No Co-existance support
	BTDEV_COEXMODE_1W,				//!< 1 wire Co-existance mode
	BTDEV_COEXMODE_3W				//!< 3 wire Co-existance mode
} BTDEV_COEXMODE;

// Service connection security types
typedef enum __Bt_Dev_Security_Type {
	BTDEV_SECTYPE_NONE,						//!< open, no security
	BTDEV_SECTYPE_STATICKEY_NO_MITM,		//!< Bonding static pass key without Man In The Middle
	BTDEV_SECTYPE_STATICKEY_MITM,			//!< Bonding static pass key with MITM
	BTDEV_SECTYPE_LESC_MITM,				//!< LE secure encryption
	BTDEV_SECTYPE_SIGNED_NO_MITM,			//!< AES signed encryption without MITM
	BTDEV_SECTYPE_SIGNED_MITM,				//!< AES signed encryption with MITM
} BTDEV_SECTYPE;

#define BTDEV_SECEXCHG_NONE				0
#define BTDEV_SECEXCHG_KEYBOARD			(1<<0)
#define BTDEV_SECEXCHG_DISPLAY			(1<<1)
#define BTDEV_SECEXCHG_OOB				(1<<2)

typedef enum __Bt_Dev_State {
	BTDEV_STATE_UNKNOWN,
	BTDEV_STATE_INITIALIZED,
	BTDEV_STATE_IDLE,
	BTDEV_STATE_ADVERTISING,
	BTDEV_STATE_CONNECTED
} BTDEV_STATE;

typedef void (*BtDevEvtHandler_t)(uint32_t Evt, void * const pCtx);
typedef void (*BtDevEvtConnected_t)(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PerrAddr[6]);
typedef void (*BtDevEvtDisconnected_t)(uint16_t ConnHdl, uint8_t Reason);
typedef uint32_t (*BtDevSendData_t)(void *pData, uint32_t Len);
typedef void (*BtDevEvtSendCompleted_t)(uint16_t ConnHdl, uint16_t NbPktSend);

#pragma pack(push, 4)

#define BTDEV_INFOSTR_MAX_SIZE			20

/// Bt Device Info
typedef struct __Bt_Dev_Info {
	const char ModelName[BTDEV_INFOSTR_MAX_SIZE];	//!< Model name
	const char ManufName[BTDEV_INFOSTR_MAX_SIZE];	//!< Manufacturer name
	const char *pSerialNoStr;	//!< Serial number string
	const char *pFwVerStr;		//!< Firmware version string
	const char *pHwVerStr;		//!< Hardware version string
} BtDevInfo_t;


typedef struct __Bt_Dev_Cfg {
	BTDEV_ROLE	Role;				//!< Application mode peripheral/central/mix
	int CentLinkCount;				//!< Number of central link
	int	PeriLinkCount;				//!< Number of peripheral link
	const char *pDevName;			//!< Device name
	uint16_t VendorId;				//!< PnP Bluetooth/USB vendor id. iBeacon mode, this is Major value
	uint16_t ProductId;				//!< PnP product ID. iBeacon mode, this is Minor value
	uint16_t ProductVer;			//!< PnP product version
	uint16_t Appearance;			//!< 16 bits Bluetooth appearance value
	const BtDevInfo_t *pDevInfo;	//!< Pointer device info descriptor DIS
	bool bExtAdv;					//!< Extended advertisement true : enable
	const uint8_t *pAdvManData;		//!< Manufacture specific data to advertise
	int AdvManDataLen;				//!< Length of manufacture specific data
	const uint8_t *pSrManData;		//!< Addition Manufacture specific data to advertise in scan response
	int SrManDataLen;				//!< Length of manufacture specific data in scan response
	BTDEV_SECTYPE SecType;			//!< Secure connection type
	uint8_t SecExchg;				//!< Sec key exchange
	bool bCompleteUuidList;			//!< true - Follow is a complete uuid list. false - incomplete list (more uuid than listed here)
	const BtUuidArr_t *pAdvUuid;
	uint32_t AdvInterval;			//!< In msec
	uint32_t AdvTimeout;			//!< In sec
	uint32_t AdvSlowInterval;		//!< Slow advertising interval, if > 0, fallback to
									//!< slow interval on adv timeout and advertise until connected
	uint32_t ConnIntervalMin;   	//!< Min. connection interval
	uint32_t ConnIntervalMax;   	//!< Max connection interval
	int8_t ConnLedPort;				//!< Connection LED port number
	int8_t ConnLedPin;				//!< Connection LED pin number
	uint8_t ConnLedActLevel;        //!< Connection LED ON logic level (0: Logic low, 1: Logic high)
	int TxPower;					//!< Tx power in dBm, -20 to +4 dBm TX power, configurable in 4 dB steps
	uint32_t (*SDEvtHandler)(void); //!< Require for BLEAPP_MODE_RTOS
	uint16_t MaxMtu;				//!< Max MTU size or 0 for default
	BTDEV_COEXMODE CoexMode;		//!< Enable support for CoEx
	int PeriphDevCnt;				//!< Max number of peripheral connection
	uint8_t *pEvtHandlerQueMem;		//!< Memory reserved for AppEvtHandler
	size_t EvtHandlerQueMemSize;	//!< Total pEvtHandlerQueMem length in bytes
} BtDevCfg_t;


typedef struct __Bt_Dev_Data {
	uint8_t Role;							//!< Device role
	char *pDevName;
	uint16_t VendorId;				//!< PnP Bluetooth/USB vendor id. iBeacon mode, this is Major value
	uint16_t ProductId;				//!< PnP product ID. iBeacon mode, this is Minor value
	uint16_t ProductVer;			//!< PnP product version
	uint16_t Appearance;			//!< 16 bits Bluetooth appearance value
	bool bExtAdv;
	uint16_t ConnHdl;
	uint8_t AdvHdl;
	int NbSrvc;
	BtGattSrvc_t Srvc[BTDEV_SERVICE_MAXCNT];
	uint32_t RxDataLen;
	uint32_t TxDataLen;
	BTDEV_COEXMODE CoexMode;
	//int8_t ConnLedPort;				//!< Connection LED port number
	//int8_t ConnLedPin;				//!< Connection LED pin number
	//uint8_t ConnLedActLevel;        //!< Connection LED ON logic level (0: Logic low, 1: Logic high)

	void (*EvtHandler)(uint32_t Evt, void * const pCtx);
	void (*Connected)(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PerrAddr[6]);
	void (*Disconnected)(uint16_t ConnHdl, uint8_t Reason);
	uint32_t (*SendData)(void * const pData, uint32_t Len);
	void (*SendCompleted)(uint16_t ConnHdl, uint16_t NbPktSent);
	BTDEV_STATE State;
	uint16_t MaxMtu;
	bool bSecure;
	bool bScan;
} BtDev_t;

#pragma pack(pop)

#ifdef __cplusplus

class BtDev : public Device {
public:
	virtual bool Init(BtDevCfg_t &Cfg, DeviceIntrf * const pIntrf);
	virtual bool Connect();
	virtual void Disconnect();
	virtual int Send(uint8_t * const pData, int DataLen);
	virtual int Receive(uint8_t * const pBuff, int BuffLen);

protected:
private:
	BtDev_t Dev;
};

extern "C" {
#endif

void BtDevInitCustomSrvc();

bool BtDevInit(const BtDevCfg_t *pCfg);
void BtDevSetDevName(const char *pName);
static inline bool BtDevSetValue(BtGattChar_t * const pChar, uint8_t * const pData, uint16_t DataLen) {
	return BtGattCharSetValue(pChar, pData, DataLen);
}
bool BtDevNotify(BtGattChar_t * const pChar, uint8_t * const pData, uint16_t DataLen);
bool BtDevWrite(BtGattChar_t * const pChar, uint8_t * const pData, uint16_t DataLen);
void BtDevInitCustomData();
void BtDevInitCustomSrvc();
bool BtDevAddSrvc(const BtGattSrvcCfg_t *pSrvcCfg);
bool BtDevAdvManDataSet(uint8_t * const pAdvData, int AdvLen, uint8_t * const pSrData, int SrLen);
void BtDevAdvStart();
void BtDevAdvStop();
BtDev_t * const BtDevGetInstance();
BTDEV_STATE BtDevGetState();

#if 0
//bool BleAppDiscoverDevice(BleDev_t * const pDev);
uint32_t BleAppDiscoverDevice(BtDev_t * const pDev);
/**
 * @brief	Peripheral discovered callback.
 *
 * This function is called in central mode when all services of the
 * device is fully discovered for the connected peripheral device.
 *
 * Application firmware should keep a copy of the peripheral data passed
 * for communication with the device.  The pointer is temporary and will be
 * destroyed upon return
 *
 * @param	pDev :	Pointer to peripheral device data
 */
void BleDevDiscovered(BtDev_t *pDev);
int BleDevFindService(BtDev_t * const pDev, uint16_t Uuid);
int BleDevFindCharacteristic(BtDev_t * const pDev, int SrvcIdx, uint16_t Uuid);
#endif

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BT_DEV_H__


/**-------------------------------------------------------------------------
@file	bt_app.h

@brief	Generic BLE application abstraction


@author	Hoang Nguyen Hoan
@date	Dec 26, 2016

@license

MIT License

Copyright (c) 2016, I-SYST inc., all rights reserved

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
#ifndef __BLE_APP_H__
#define __BLE_APP_H__

#include <stdint.h>
#include <inttypes.h>

#include "bluetooth/bt_gap.h"
#include "bluetooth/ble_adv.h"
#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bleadv_mandata.h"
#include "bluetooth/bt_dev.h"

/** @addtogroup Bluetooth
  * @{
  */

typedef enum __Bt_App_Coex_Mode {
	BTAPP_COEXMODE_NONE,			//!< No Co-existance support
	BTAPP_COEXMODE_1W,				//!< 1 wire Co-existance mode
	BTAPP_COEXMODE_3W				//!< 3 wire Co-existance mode
} BTAPP_COEXMODE;

// App events
typedef enum __Bt_App_Event {
	BTAPP_CONNECTED,
	BTAPP_DISCONNECTED,
	BTAPP_EVENT_EXCHANGE_MTU,
} BTAPP_EVT;

#pragma pack(push, 4)

#if 0
/// BLE App configuration
typedef struct __Bt_App_Config {
	BTDEV_ROLE	Role;				//!< Application mode peripheral/central/mix
	int CentLinkCount;				//!< Number of central link
	int	PeriLinkCount;				//!< Number of peripheral link
	const char *pDevName;			//!< Device name
	uint16_t VendorID;				//!< PnP Bluetooth/USB vendor id. iBeacon mode, this is Major value
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
	BTAPP_COEXMODE CoexMode;		//!< Enable support for CoEx
	int PeriphDevCnt;				//!< Max number of peripheral connection
	uint8_t *pEvtHandlerQueMem;		//!< Memory reserved for AppEvtHandler
	size_t EvtHandlerQueMemSize;	//!< Total pEvtHandlerQueMem length in bytes
} BtAppCfg_t;
#endif
#pragma pack(pop)

#ifdef __cplusplus

class BleApp {
public:
	virtual bool Init(BtDevCfg_t &CfgData);

	virtual void InitCustomData() = 0;
	virtual void InitServices() = 0;
	//virtual void SrvcEvtDispatch(ble_evt_t * p_ble_evt) = 0;

	virtual void ProcessEvt();
	virtual void EnterDfu();
	virtual void Start();

private:
};

extern "C" {
#endif


// ***
// Require implementations per app
//
/**
 * @Brief	User function to initialize any app specific data
 * 	This function is called prio to initializing services
 *
 */
void BtAppInitUserData();

/**
 * @brief	User function to initialize all app services
 * 	This is called before initializing advertisement
 */
void BleAppInitUserServices();

void BleAppUserEvtConnected(uint16_t ConnHdl);

void BleAppUserEvtDisconnected(uint16_t ConnHdl);

/**
 * @Brief	User peripheral app event handler
 */
//void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt);

/**
 * @Brief	User central app event handler
 *
 */
//void BleCentralEvtUserHandler(ble_evt_t * p_ble_evt);

//void BleDevServiceDiscovered(uint16_t ConnHdl, uint16_t Count, ble_gattc_service_t * const pServices);

//*** Require implementation if app operating mode is BLEAPP_MODE_RTOS
// This function should normal wait for RTOS to signal an event on sent by
// Softdevice
void BleAppRtosWaitEvt(void);

/**
 * @brief	BLE main App initialization
 *
 * @param	pBleAppCfg : Pointer to app configuration data
 * @param	bEraseBond : true to force erase all bonding info
 *
 * @return	true - success
 */
bool BtAppInit(const BtDevCfg_t * const pCfg);
void BtAppEnterDfu();
void BtAppRun();
uint16_t BleAppGetConnHandle();
void BleAppGapDeviceNameSet(const char* ppDeviceName);

void BtAppSetDevName(const char *pName);
char * const BtAppGetDevName();
bool BtAppNotify(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen);

/**
 *
 * @return	true - advertising
 * 			false - not advertising
 */
bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen);
void BtAppAdvTimeoutHandler();
void BtAppAdvStart();
void BleAppAdvStop();
void BleAppDisconnect();

//bool BleAppScanInit(BleAppScanCfg_t *pCfg);
void BleAppScan();
void BleAppScanStop();
//bool BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam);
//uint32_t BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam);
bool BleAppEnableNotify(uint16_t ConnHandle, uint16_t CharHandle);
bool BleAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DatLen);
int8_t GetValidTxPower(int TxPwr);
bool isConnected();

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BLE_APP_H__


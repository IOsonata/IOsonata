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
#ifndef __BT_APP_H__
#define __BT_APP_H__

#include <stdint.h>
#include <inttypes.h>

#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_adv.h"
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

#pragma pack(pop)

#ifdef __cplusplus

class BtApp {
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
void BtAppInitCustomData();

/**
 * @brief	User function to initialize all app services
 * 	This is called before initializing advertisement
 */
void BtAppInitCustomServices();
void BtAppEvtConnected(uint16_t ConnHdl);
void BtAppEvtDisconnected(uint16_t ConnHdl);
void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx);
void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx);


//void BleDevServiceDiscovered(uint16_t ConnHdl, uint16_t Count, ble_gattc_service_t * const pServices);

//*** Require implementation if app operating mode is BLEAPP_MODE_RTOS
// This function should normal wait for RTOS to signal an event on sent by
// Softdevice
void BtAppRtosWaitEvt(void);

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
void BtAppGapDeviceNameSet(const char* ppDeviceName);

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

#endif // __BT_APP_H__


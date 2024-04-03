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
#ifndef BTAPP_NAME_MAXLEN
#define BTAPP_NAME_MAXLEN				20
#endif

#ifndef BTAPP_SERVICE_MAXCNT
#define BTAPP_SERVICE_MAXCNT			20
#endif

#ifndef BTAPP_DEFAULT_MAX_DATA_LEN
#define BTAPP_DEFAULT_MAX_DATA_LEN		251
#endif

#ifndef BTAPP_DEFAULT_MAX_MTU
#define BTAPP_DEFAULT_MAX_MTU			515
#endif

#define BT_CONN_HDL_INVALID				0xFFFF	// Invalid Connection Handle
#define BT_CONN_HDL_ALL					0xFFFE	// Applies to all Connection Handles

typedef enum __Bt_App_Role {
	BTAPP_ROLE_BROADCASTER	= BT_GAP_ROLE_BROADCASTER,		//!< non connectable Advertising only
	BTAPP_ROLE_OBSERVER		= BT_GAP_ROLE_OBSERVER,			//!< non connectable central
	BTAPP_ROLE_PERIPHERAL	= BT_GAP_ROLE_PERIPHERAL,		//!< BLE connectable peripheral device
	BTAPP_ROLE_CENTRAL		= BT_GAP_ROLE_CENTRAL,			//!< BLE Central device
	BTAPP_ROLE_MIXED		= BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL	//!< Mixed central/peripheral
} BTAPP_ROLE;

#if 0
// Service connection security types
typedef enum __Bt_App_Security_Type {
	BTAPP_SECTYPE_NONE = BT_GAP_SECTYPE_NONE,				//!< open, no security
	BTAPP_SECTYPE_STATICKEY_NO_MITM = BT_GAP_SECTYPE_STATICKEY_NO_MITM,	//!< Bonding static pass key without Man In The Middle
	BTAPP_SECTYPE_STATICKEY_MITM = BT_GAP_SECTYPE_STATICKEY_MITM,		//!< Bonding static pass key with MITM
	BTAPP_SECTYPE_LESC_MITM = BT_GAP_SECTYPE_LESC_MITM,					//!< LE secure encryption
	BTAPP_SECTYPE_SIGNED_NO_MITM = BT_GAP_SECTYPE_SIGNED_NO_MITM,		//!< AES signed encryption without MITM
	BTAPP_SECTYPE_SIGNED_MITM = BT_GAP_SECTYPE_SIGNED_MITM,				//!< AES signed encryption with MITM
} BTAPP_SECTYPE;
#endif

#define BTAPP_SECEXCHG_NONE				0
#define BTAPP_SECEXCHG_KEYBOARD			(1<<0)
#define BTAPP_SECEXCHG_DISPLAY			(1<<1)
#define BTAPP_SECEXCHG_OOB				(1<<2)

typedef enum __Bt_App_State {
	BTAPP_STATE_UNKNOWN,
	BTAPP_STATE_INITIALIZED,
	BTAPP_STATE_IDLE,
	BTAPP_STATE_ADVERTISING,
	BTAPP_STATE_CONNECTED
} BTAPP_STATE;

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

#define BTAPP_INFOSTR_MAX_SIZE			20

/// Bt Device Info
typedef struct __Bt_App_Dev_Info {
	const char ModelName[BTAPP_INFOSTR_MAX_SIZE];	//!< Model name
	const char ManufName[BTAPP_INFOSTR_MAX_SIZE];	//!< Manufacturer name
	const char *pSerialNoStr;	//!< Serial number string
	const char *pFwVerStr;		//!< Firmware version string
	const char *pHwVerStr;		//!< Hardware version string
} BtAppDevInfo_t;

typedef struct __Bt_App_Cfg {
	BTAPP_ROLE	Role;				//!< Application mode peripheral/central/mix
	int CentLinkCount;				//!< Number of central link
	int	PeriLinkCount;				//!< Number of peripheral link
	const char *pDevName;			//!< Device name
	uint16_t VendorId;				//!< PnP Bluetooth/USB vendor id. iBeacon mode, this is Major value
	uint16_t ProductId;				//!< PnP product ID. iBeacon mode, this is Minor value
	uint16_t ProductVer;			//!< PnP product version
	uint16_t Appearance;			//!< 16 bits Bluetooth appearance value
	const BtAppDevInfo_t *pDevInfo;	//!< Pointer device info descriptor DIS
	bool bExtAdv;					//!< Extended advertisement true : enable
	const uint8_t *pAdvManData;		//!< Manufacture specific data to advertise
	int AdvManDataLen;				//!< Length of manufacture specific data
	const uint8_t *pSrManData;		//!< Addition Manufacture specific data to advertise in scan response
	int SrManDataLen;				//!< Length of manufacture specific data in scan response
	BTGAP_SECTYPE SecType;			//!< Secure connection type
	uint8_t SecExchg;				//!< Sec key exchange
	bool bCompleteUuidList;			//!< true - Follow is a complete uuid list. false - incomplete list (more uuid than listed here)
	const BtUuidArr_t *pAdvUuid;
	uint32_t AdvInterval;			//!< In msec
	uint32_t AdvTimeout;			//!< In sec
	uint32_t AdvSlowInterval;		//!< Slow advertising interval, if > 0, fallback to
									//!< slow interval on adv timeout and advertise until connected
	float ConnIntervalMin;   		//!< Min. connection interval in msec
	float ConnIntervalMax;   		//!< Max connection interval in msec
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
	size_t AttDBMemSize;			//!< User overload mem size for stack usage, set to 0 if not overloading default.
} BtAppCfg_t;

#if 0
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
	//int NbSrvc;
	//BtGattSrvc_t Srvc[BTDEV_SERVICE_MAXCNT];
	BtGattSrvc_t *pSrvc;
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

#endif

#pragma pack(pop)

#ifdef __cplusplus

class BtApp {
public:
	virtual bool Init(BtAppCfg_t &CfgData);

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
void BtAppInitUserServices();
void BtAppEvtConnected(uint16_t ConnHdl);
void BtAppEvtDisconnected(uint16_t ConnHdl);
void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx);
void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx);
void BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *pAdvData);

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
bool BtAppInit(const BtAppCfg_t * const pCfg);
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
void BtAppAdvStop();
void BtAppDisconnect();

//bool BleAppScanInit(BleAppScanCfg_t *pCfg);
bool BtAppScanInit(BtGapScanCfg_t *pCfg);
//void BleAppScan();
void BtAppScan();
void BtAppScanStop();
bool BtAppConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam);//, ble_gap_conn_params_t * const pConnParam);
//bool BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam);
//uint32_t BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam);
bool BtAppEnableNotify(uint16_t ConnHandle, uint16_t CharHandle);
bool BtAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DatLen);
int8_t GetValidTxPower(int TxPwr);
bool isConnected();

/// return true - Ble initialized
bool BtInitialized();

/// return true - Ble connected
bool BtConnected();

void BtAppDiscoverDevice(uint16_t ConnHdl);

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BT_APP_H__


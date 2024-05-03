/**-------------------------------------------------------------------------
@file	bt_dev.h

@brief	Implementation allow the creation of generic BLE peripheral device class.

This BLE device class is used by BLE Central application to connect and communicate
with it.

@author	Hoang Nguyen Hoan
@date	Jan. 17, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

#ifndef __BT_DEV_H__
#define __BT_DEV_H__

#include "device.h"

#ifdef NRFXLIB_SDC
#include "bluetooth/bt_gatt.h"
#else
#include "ble_gatt_db.h"
#endif

/** @addtogroup Bluetooth
  * @{
  */

#define BLEDEV_NAME_MAXLEN			20


typedef struct __Ble_Dev_Cfg {
	char Name[BLEDEV_NAME_MAXLEN];			//!< Device name
	uint8_t Addr[6];						//!< Device MAC address
} BleDevCfg_t;

typedef BleDevCfg_t	BLEDEV_CFG;

#define BLEPERIPH_DEV_SERVICE_MAXCNT	10
#define BLEPERIPH_DEV_NAME_MAXLEN		30

typedef struct __Bt_Dev_Data {
	char Name[BLEPERIPH_DEV_NAME_MAXLEN];
	uint8_t Addr[6];
	uint16_t ConnHdl;
	int NbSrvc;
#ifndef NRFXLIB_SDC
	ble_gatt_db_srv_t Services[BLEPERIPH_DEV_SERVICE_MAXCNT];
#else
	BtHciDevice_t *pHciDev;
	BtGattDBSrvc_t Services[BLEPERIPH_DEV_SERVICE_MAXCNT];
#endif
} BtDev_t;

//typedef BleDev_t	BLEPERIPH_DEV;

#ifdef __cplusplus

class BleDev : public Device {
public:
	virtual bool Init(BleDevCfg_t &Cfg, DeviceIntrf * const pIntrf);
	virtual bool Connect();
	virtual void Disconnect();
	virtual int BleSend(uint8_t * const pData, int DataLen);
	virtual int BleReceive(uint8_t * const pBuff, int BuffLen);

protected:
private:
	BtDev_t Dev;
};

extern "C" {
#endif

//bool BleAppDiscoverDevice(BleDev_t * const pDev);
//uint32_t BleAppDiscoverDevice(BtDev_t * const pDev);
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

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BT_DEV_H__


/**-------------------------------------------------------------------------
@file	bt_scan.h

@brief	Generic Bluetooth scan

Generic implementation of bt device scanning

@author	Hoang Nguyen Hoan
@date	Feb. 06, 2024

@license

MIT License

Copyright (c) 2024, I-SYST, all rights reserved

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
#ifndef __BT_SCAN_H__
#define __BT_SCAN_H__

#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_gap.h"

/** @addtogroup Bluetooth
  * @{
  */

#pragma pack(push,4)

typedef struct __Bt_Adv_Scan_Report {
	uint8_t Rssi;
	uint8_t AddrType;
	uint8_t Addr[6];
	uint8_t AdvLen;
	uint8_t AdvData[1];
} BtAdvScanReport_t;

typedef struct __BtDisc_Char {
	uint16_t Uuid;
	uint16_t Hdl;
} BtDiscChar_t;

#define BTPERIPH_DEV_CHAR_MAXCNT			10

typedef struct __BtDisc_Srvc {
	BtUuid_t Uuid;
	uint16_t Hdl;
	int NbChar;
	BtDiscChar_t Char[BTPERIPH_DEV_CHAR_MAXCNT];
} BtDiscSrvc_t;

#define BTPERIPH_DEV_SERVICE_MAXCNT			10
#define BTPERIPH_DEV_NAME_MAXLEN			30

typedef struct __BtPeriph_Dev {
	char Name[BTPERIPH_DEV_NAME_MAXLEN];
	BtGapPeerAddr_t Addr;
	uint16_t ConnHdl;
	int NbSrvc;
	BtDiscSrvc_t Srvc[BTPERIPH_DEV_SERVICE_MAXCNT];
} BtPeriphDev_t;

typedef struct __Bt_Scan_Cfg {

} BtScanCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool BtScanInit(const BtScanCfg_t *pCfg);
bool BtScanStart();
void BtScanStop();
void BtScanReport(uint8_t Type, uint8_t NbReport, void *pReport);

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BT_SCAN_H__

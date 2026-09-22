/**-------------------------------------------------------------------------
@file	bt_dfu_smp.h

@brief	SMP over Bluetooth LE: the SMP GATT service.

The transport the SMP host tools use over Bluetooth: one primary service,
one characteristic written without response and notified.

	service			8D53DC1D-1DB7-4CD3-868B-8A527460AA84
	characteristic	DA2E7828-FBCE-4E01-AE9E-261174997C48

A request may take several writes, a response several notifications; the
SMP header length says where a packet ends, as the SMP specification
defines for this transport. A complete request is handed to DfuSmp from the
application event queue, never from the stack callback, because it may erase
and write memory.

One request at a time: the parameters answer reports one buffer, so a host
waits for each response before it sends the next request.

Usage, from BtAppInitUserServices:

	static uint8_t s_SmpRx[1024], s_SmpTx[512];
	BtDfuSmpCfg_t cfg = { &g_DfuSmp, s_SmpRx, sizeof(s_SmpRx),
						  s_SmpTx, sizeof(s_SmpTx), BT_GAP_SECTYPE_NONE };
	BtDfuSmpInit(cfg);

Signed images keep an open link from installing anything but what the key
holder signed. A link security type keeps strangers from uploading at all,
or from erasing slot 1.

The DfuSmp BufSize must be the size of the receive buffer: it is what the
host is told a request may be. The link must allow two vendor UUID bases,
one for the service and one for the characteristic.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __BT_DFU_SMP_H__
#define __BT_DFU_SMP_H__

#include <stdint.h>

#include "bluetooth/bt_gatt.h"
#include "dfu/dfu_smp.h"

/** @addtogroup Bluetooth
  * @{
  */

/// 16 bit parts of the service and characteristic UUIDs, on their bases.
#define BT_DFUSMP_UUID_SERVICE		0xDC1D
#define BT_DFUSMP_UUID_CHAR			0x7828

/// Largest write or notification: ATT MTU 247 less the 3 byte header.
#define BT_DFUSMP_CHAR_MAXLEN		244

#ifdef __cplusplus

typedef struct __Bt_DfuSmp_Cfg {
	DfuSmp *pMgr;				//!< Initialised SMP server
	uint8_t *pRxBuf;			//!< One request, reassembled here
	uint16_t RxBufSize;			//!< Its size, the DfuSmp BufSize
	uint8_t *pTxBuf;			//!< One response
	uint16_t TxBufSize;			//!< Its size, 512 covers every response
	uint8_t SecType;			//!< BT_GAP_SECTYPE_ the link must have to
								//!< use the service, NONE for an open link
} BtDfuSmpCfg_t;

/**
 * @brief	Register the SMP service.
 *
 * Call from BtAppInitUserServices, once.
 *
 * @param	Cfg : Configuration, the buffers outlive the call.
 *
 * @return	true when the service is registered.
 */
bool BtDfuSmpInit(const BtDfuSmpCfg_t &Cfg);

/// The service, to list its UUID in the advertising data.
BtGattSrvc_t *BtDfuSmpSrvc(void);

/// Drop a partial request, a response not yet sent and an unfinished upload,
/// as after a disconnection. Call from BtAppEvtDisconnected.
void BtDfuSmpReset(void);

/// True while a response is still going out. An application that resets on
/// request waits for this to clear first.
bool BtDfuSmpTxBusy(void);

#endif

/** @} */

#endif

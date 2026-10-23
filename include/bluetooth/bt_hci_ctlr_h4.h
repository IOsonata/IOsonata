/**-------------------------------------------------------------------------
@file	bt_hci_ctlr_h4.h

@brief	Bridge between an HCI controller and an H4 link

Runs on the core that has the Bluetooth controller when the host is on
another core or chip, for instance the nRF5340 network core talking to the
application core over an IpcIntrf. Packets from the host arrive framed by
BtHciH4 and go down into the controller. Events and ACL data from the
controller go up through BtHciH4Send.

Two functions are plugged in, nothing else is needed:
 - BtHciCtlrH4FromHost is the RxHandler of the BtHciH4 layer.
 - BtHciCtlrH4FromCtlr is the RxHandler of the BtHciCtlrCfg_t.

BtHciCtlrH4Init links the two after both are initialized. The owner calls
BtHciH4Process from its event queue when the H4 layer reports received bytes,
and BtHciCtlrProcess when the controller has messages, as it does for any
other host.

A packet the link cannot take in time is lost, and the host sees it as a
missing response or a stalled flow. The counters tell which direction lost
packets. Size the TX room of the link for what the controller can queue.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#ifndef __BT_HCI_CTLR_H4_H__
#define __BT_HCI_CTLR_H4_H__

#include <stdint.h>
#include <stdbool.h>

#include "bluetooth/bt_hci_ctlr.h"
#include "bluetooth/bt_hci_h4.h"

/** @addtogroup Bluetooth
  * @{
  */

/// Link between one controller and one H4 layer
typedef struct __Bt_Hci_Ctlr_H4 {
	BtHciCtlrDev_t *pCtlr;			//!< Controller
	BtHciH4Dev_t *pH4;				//!< H4 layer to the host
	uint32_t UnsupportedCnt;		//!< Host packets of a type the bridge does not pass down
	uint32_t CmdFailCnt;			//!< Commands the controller did not take
	uint32_t AclFailCnt;			//!< Host ACL packets the controller did not take
	uint32_t UpFailCnt;				//!< Controller packets the link did not take, lost
	uint32_t BadLenCnt;				//!< Controller packets with a length that does not fit a link packet
} BtHciCtlrH4_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Link a controller and an H4 layer.
 *
 * Call it after BtHciCtlrInit (or BtHciCtlrEnable) and BtHciH4Init: it sets
 * pObj of the controller and pCtx of the H4 layer to pBridge.
 *
 * @param	pBridge	Bridge.
 * @param	pCtlr	Controller, initialized.
 * @param	pH4		H4 layer, initialized.
 *
 * @return	true on success.
 */
bool BtHciCtlrH4Init(BtHciCtlrH4_t * const pBridge, BtHciCtlrDev_t * const pCtlr, BtHciH4Dev_t * const pH4);

/**
 * @brief	Packet from the host, goes down into the controller.
 *
 * RxHandler of the H4 layer. Commands go to the SendCommand of the
 * controller and ACL data to its Send. Other packet types are counted and
 * dropped.
 */
void BtHciCtlrH4FromHost(BtHciH4Dev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len);

/**
 * @brief	Packet from the controller, goes up to the host.
 *
 * RxHandler of the controller configuration. The length is taken from the
 * packet header.
 */
void BtHciCtlrH4FromCtlr(BtHciCtlrDev_t * const pDev, bool bIsEvent, uint8_t *pPacket);

#ifdef __cplusplus
}
#endif

/** @} End of group Bluetooth */

#endif

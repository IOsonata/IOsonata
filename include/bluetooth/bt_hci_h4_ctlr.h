/**-------------------------------------------------------------------------
@file	bt_hci_h4_ctlr.h

@brief	Controller port of the HCI host for a controller behind an H4 link

The BtHciCtlr (bt_hci_ctlr.h) of a controller the host does not share an
image with: an nRF5x running HciController behind a UART, the HciController
on the other core of an nRF5340 behind the IPC rings, a USB dongle. The link
is a BtHciH4 (bt_hci_h4.h), the packet queue and command layer every one of
those transports is built on, so this port is written once against it and
the transport only has to open the link.

The transport is the file linked with this one: bt_hci_ctlr_uart.cpp opens a
BtHciUart on g_BtHciUartCfg, bt_hci_ctlr_ipc.cpp attaches a BtHciIpc to the
shared memory block the linker script declares. Each defines BtHciCtlrH4Open
and forwards the three link callbacks to the functions below.

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
#ifndef __BT_HCI_H4_CTLR_H__
#define __BT_HCI_H4_CTLR_H__

#include <stdint.h>
#include <stdbool.h>

#include "bluetooth/bt_hci_h4.h"

/** @addtogroup Bluetooth
  * @{
  */

/// Received packet queue depth of the link. Packets the controller sends
/// while a command waits for its response are kept there.
#ifndef BT_HCI_H4_CTLR_PKT_COUNT
#define BT_HCI_H4_CTLR_PKT_COUNT		8
#endif

/// Packets handled per event before the handler queues itself again, so the
/// other events of the queue run in between.
#define BT_HCI_H4_CTLR_RX_BURST			4

/// HCI Reset attempts at start. The controller may still be starting when
/// this core is up, each attempt waits up to the link timeout.
#define BT_HCI_H4_CTLR_RESET_TRY		5

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Open the link to the controller.
 *
 * Defined by the transport linked with this port. It initializes its
 * BtHciUart, BtHciIpc or other BtHciH4 binding with the packet queue memory
 * given, forwarding the binding's RxHandler, CmdRspHandler and RxReady to
 * BtHciH4CtlrRx, BtHciH4CtlrCmdRsp and BtHciH4CtlrRxReady, and returns the
 * H4 layer of the link. Called once by BtHciCtlrStart.
 *
 * @param	pPktMem		Packet queue memory, BT_HCI_H4_PKTMEM_SIZE(BT_HCI_H4_CTLR_PKT_COUNT)
 * @param	PktMemSize	Its size in bytes
 *
 * @return	The H4 layer of the open link, NULL when it could not be opened
 */
BtHciH4Dev_t *BtHciCtlrH4Open(uint8_t *pPktMem, uint32_t PktMemSize);

/**
 * @brief	Each queued packet, in arrival order. The transport forwards its
 * 			binding's RxHandler here.
 */
void BtHciH4CtlrRx(uint8_t Type, uint8_t *pPkt, uint16_t Len);

/**
 * @brief	Command Complete and Command Status as soon as they are framed.
 * 			The transport forwards its binding's CmdRspHandler here.
 */
void BtHciH4CtlrCmdRsp(uint8_t Type, uint8_t *pPkt, uint16_t Len);

/**
 * @brief	Packets arrived, from the interrupt of the link. The transport
 * 			forwards its binding's RxReady here. Queues the processing on the
 * 			Bluetooth work queue, nothing is framed in the interrupt.
 */
void BtHciH4CtlrRxReady(void);

#ifdef __cplusplus
}
#endif

/** @} End of group Bluetooth */

#endif // __BT_HCI_H4_CTLR_H__

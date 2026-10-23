/**-------------------------------------------------------------------------
@file	bt_hci_ctlr_ipc.cpp

@brief	IPC transport of the HCI host to a controller on another core

Attaches a BtHciIpc (bt_hci_ipc.h) to the shared memory block the controller
core owns and hands its H4 layer to the controller port of the host
(bt_hci_h4_ctlr.cpp). This is the transport of the application core of an
nRF5340, whose network core runs the HciController firmware as the IPC
owner: it formats the rings, this core attaches to what it finds.

The block is declared by the linker script, as ipc/ipc_intrf.h asks: a
MEMORY region exporting __start_ipc0 and __stop_ipc0, the same block on both
cores. The channels and the interrupt priority come from g_BtHciCtlrIpcCfg,
a weak default an application using other channels defines itself. The
default follows the HciController network core build: it raises channel 0
toward this core and listens on channel 1.

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
#include <stdint.h>

#include "bluetooth/bt_hci_ipc.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "bluetooth/bt_hci_h4_ctlr.h"

// The shared memory block, from the linker script
extern "C" uint8_t __start_ipc0[];
extern "C" uint8_t __stop_ipc0[];

// Channels of the HciController network core build: it sends on 0 and
// receives on 1, so this core is the other way around.
extern "C" __attribute__((weak)) const BtHciCtlrIpcCfg_t g_BtHciCtlrIpcCfg = {
	.TxChan = 1,
	.RxChan = 0,
	.IntPrio = 6,
};

// HCI transport to the controller core
static BtHciIpcDev_t s_BtHciIpc;

// The link callbacks, forwarded to the controller port
static void BtHciCtlrIpcRx(BtHciIpcDev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	(void)pDev;

	BtHciH4CtlrRx(Type, pPkt, Len);
}

static void BtHciCtlrIpcCmdRsp(BtHciIpcDev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	(void)pDev;

	BtHciH4CtlrCmdRsp(Type, pPkt, Len);
}

static void BtHciCtlrIpcRxReady(BtHciIpcDev_t * const pDev)
{
	(void)pDev;

	BtHciH4CtlrRxReady();
}

BtHciH4Dev_t *BtHciCtlrH4Open(uint8_t *pPktMem, uint32_t PktMemSize)
{
	BtHciIpcCfg_t cfg = {};
	cfg.pMemBlk = __start_ipc0;
	cfg.MemSize = (uint32_t)(__stop_ipc0 - __start_ipc0);
	cfg.bOwner = false;			// the controller core formats the block
	cfg.TxChan = g_BtHciCtlrIpcCfg.TxChan;
	cfg.RxChan = g_BtHciCtlrIpcCfg.RxChan;
	cfg.IntPrio = g_BtHciCtlrIpcCfg.IntPrio;
	cfg.pPktMem = pPktMem;
	cfg.PktMemSize = PktMemSize;
	cfg.RxHandler = BtHciCtlrIpcRx;
	cfg.CmdRspHandler = BtHciCtlrIpcCmdRsp;
	cfg.RxReady = BtHciCtlrIpcRxReady;

	if (BtHciIpcInit(&s_BtHciIpc, &cfg) == false)
	{
		return nullptr;
	}

	return &s_BtHciIpc.H4;
}

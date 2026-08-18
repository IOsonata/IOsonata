/**-------------------------------------------------------------------------
@file	bt_hci_ctlr.cpp

@brief	Generic implementation of Bluetooth controller device.


@author	Hoang Nguyen Hoan
@date	Nov. 30, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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

#include "bluetooth/bt_hci_ctlr.h"

// Generic HCI controller layer. The ATT and L2CAP handling that previously
// sat here belongs above the HCI line and is provided by bt_att and
// bt_hci_host. Per target HCI controller transport lives in the target file
// bt_hci_ctlr_<target>.cpp, for example bt_hci_ctlr_sdc.cpp for the nrfxlib
// SDC controller.

#include <memory.h>
#include "istddef.h"

// RX packet fifo. Holds controller->host packets until BtHciCtlrProcess drains
// them to RxHandler. Sized for BT_HCI_CTLR_MTU_MAX sized packets.
#define BTHCICTLR_FIFO_MEM_SIZE			BTHCICTLR_PKT_CFIFO_TOTAL_MEMSIZE(4, BT_HCI_CTLR_MTU_MAX)
alignas(4) static uint8_t s_BtHciCtlrRxFifoMem[BTHCICTLR_FIFO_MEM_SIZE];

// Queue one controller->host packet. Hdl identifies the value, data copied up
// to PacketSize. Returns bytes queued, 0 if the fifo is full.
static size_t BtHciCtlrReceive(BtHciCtlrDev_t * const pDev, uint16_t Hdl, void * const pData, size_t Len)
{
	BtHciCtlrPkt_t *p = (BtHciCtlrPkt_t*)CFifoPut(pDev->hRxFifo);
	if (p)
	{
		// Data capacity is the element size less the packet header.
		size_t cap = pDev->PacketSize - BTHCICTLR_PKTHDR_LEN;
		size_t l = min(Len, cap);

		p->Len = l;
		p->ValHdl = Hdl;
		p->Off = 0;

		memcpy(p->Data, pData, l);

		return l;
	}

	return 0;
}

// DevIntrf ops. The HCI controller is not a byte bus: it exchanges formed
// packets through Send / SendCommand and the RX fifo, so the rate and rx/tx
// stream ops are no-ops.
static void BtHciCtlrIntrfDisable(DevIntrf_t * const) {}
static void BtHciCtlrIntrfEnable(DevIntrf_t * const) {}
static uint32_t BtHciCtlrIntrfGetRate(DevIntrf_t * const) { return 0; }
static uint32_t BtHciCtlrIntrfSetRate(DevIntrf_t * const, uint32_t) { return 0; }
static bool BtHciCtlrIntrfStartRx(DevIntrf_t * const, uint32_t) { return true; }
static int BtHciCtlrIntrfRxData(DevIntrf_t * const, uint8_t *, int) { return 0; }
static void BtHciCtlrIntrfStopRx(DevIntrf_t * const) {}

static bool BtHciCtlrIntrfStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	BtHciCtlrDev_t *p = (BtHciCtlrDev_t *)pDev->pDevData;

	p->ConnHdl = DevAddr >> 16;
	p->ValHdl = DevAddr & 0xFFFF;

	return true;
}

// Route a formed ACL packet down through the bound transport send. The target
// controller binds pDev->Send in its Enable (SDC binds it to sdc_hci_data_put).
static int BtHciCtlrIntrfTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	BtHciCtlrDev_t *p = (BtHciCtlrDev_t *)pDev->pDevData;

	return (p->Send != nullptr) ? (int)p->Send(p, (void*)pData, DataLen) : 0;
}

static void BtHciCtlrIntrfStopTx(DevIntrf_t * const) {}
static void BtHciCtlrIntrfPowerOff(DevIntrf_t * const) {}

bool BtHciCtlrInit(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->PacketSize == 0)
	{
		pDev->PacketSize = BT_HCI_CTLR_MTU_MAX + BTHCICTLR_PKTHDR_LEN;
	}
	else
	{
		if (pCfg->PacketSize > UINT32_MAX - BTHCICTLR_PKTHDR_LEN)
		{
			return false;
		}
		pDev->PacketSize = pCfg->PacketSize + BTHCICTLR_PKTHDR_LEN;
	}

	uint8_t *pRxFifoMem = pCfg->pRxFifoMem;
	uint32_t rxFifoMemSize;

	if (pRxFifoMem == nullptr)
	{
		pRxFifoMem = s_BtHciCtlrRxFifoMem;
		rxFifoMemSize = BTHCICTLR_FIFO_MEM_SIZE;
	}
	else
	{
		if (pCfg->RxFifoMemSize <= 0)
		{
			return false;
		}
		rxFifoMemSize = (uint32_t)pCfg->RxFifoMemSize;
	}

	// CFifo stores its header directly in the supplied block and therefore
	// requires word-aligned memory. It also accepts a block too small for one
	// element by returning a FIFO with MaxIdxCnt == 0, so enforce one complete
	// controller packet here before initialization.
	if (((uintptr_t)pRxFifoMem & (alignof(CFifo_t) - 1U)) != 0 ||
		rxFifoMemSize < CFIFO_TOTAL_MEMSIZE(1, pDev->PacketSize))
	{
		return false;
	}

	pDev->hRxFifo = CFifoInit(pRxFifoMem, rxFifoMemSize,
							  (uint32_t)pDev->PacketSize, true);
	if (pDev->hRxFifo == nullptr)
	{
		return false;
	}

	pDev->DevIntrf.Type = DEVINTRF_TYPE_BT;
	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.Disable = BtHciCtlrIntrfDisable;
	pDev->DevIntrf.Enable = BtHciCtlrIntrfEnable;
	pDev->DevIntrf.GetRate = BtHciCtlrIntrfGetRate;
	pDev->DevIntrf.SetRate = BtHciCtlrIntrfSetRate;
	pDev->DevIntrf.StartRx = BtHciCtlrIntrfStartRx;
	pDev->DevIntrf.RxData = BtHciCtlrIntrfRxData;
	pDev->DevIntrf.StopRx = BtHciCtlrIntrfStopRx;
	pDev->DevIntrf.StartTx = BtHciCtlrIntrfStartTx;
	pDev->DevIntrf.TxData = BtHciCtlrIntrfTxData;
	pDev->DevIntrf.StopTx = BtHciCtlrIntrfStopTx;
	pDev->DevIntrf.MaxRetry = 1;
	pDev->DevIntrf.PowerOff = BtHciCtlrIntrfPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.EvtCB = pCfg->EvtHandler;
	pDev->Receive = BtHciCtlrReceive;
	pDev->Send = nullptr;			// bound by the target controller Enable
	pDev->SendCommand = nullptr;	// bound by the target controller Enable
	pDev->RxHandler = pCfg->RxHandler;
	pDev->OnWake = pCfg->OnWake;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

size_t BtHciCtlrSendCommand(BtHciCtlrDev_t * const pDev, uint16_t OpCode, const void *pParam, uint8_t ParamLen)
{
	if (pDev == nullptr || pDev->SendCommand == nullptr)
	{
		return 0;
	}

	if (ParamLen > 0 && pParam == nullptr)
	{
		return 0;
	}

	// Standard HCI command packet: opcode little endian, parameter total
	// length, then the parameters.
	uint8_t buf[BTHCICTLR_CMD_HDR_LEN + BTHCICTLR_CMD_PARAM_MAX];

	buf[0] = (uint8_t)(OpCode & 0xFF);
	buf[1] = (uint8_t)((OpCode >> 8) & 0xFF);
	buf[2] = ParamLen;

	if (ParamLen > 0 && pParam != nullptr)
	{
		memcpy(&buf[BTHCICTLR_CMD_HDR_LEN], pParam, ParamLen);
	}

	return pDev->SendCommand(pDev, buf, BTHCICTLR_CMD_HDR_LEN + ParamLen);
}

// Target controller bring-up. The weak default succeeds, which is right for a
// port whose controller needs nothing beyond the generic wiring: a plain HCI
// transport over UART or USB has no vendor stack to start.
__attribute__((weak)) bool BtHciCtlrStart(BtHciCtlrDev_t * const pDev,
										  const BtHciCtlrCfg_t *pCfg)
{
	(void)pDev;
	(void)pCfg;

	return true;
}

// One bring-up sequence for every port. The ordering is the point: generic
// Why the last bring-up failed. Kept here rather than in a port so every port
// records it the same way and an application reads it the same way, and so the
// storage exists even for a port that records nothing.
static BtHciCtlrError_t s_BtHciCtlrError = BT_HCI_CTLR_ERROR_NONE;
static int32_t s_BtHciCtlrErrorValue = 0;

void BtHciCtlrErrorSet(BtHciCtlrError_t Error, int32_t Value)
{
	s_BtHciCtlrError = Error;
	s_BtHciCtlrErrorValue = Value;
}

BtHciCtlrError_t BtHciCtlrErrorGet(void)
{
	return s_BtHciCtlrError;
}

int32_t BtHciCtlrErrorValueGet(void)
{
	return s_BtHciCtlrErrorValue;
}

// init assigns RxHandler, Receive and the interface table, and returns early
// without them on a bad packet size, a misaligned or undersized RX FIFO
// buffer, or a CFifoInit failure. Starting the controller anyway leaves an
// enabled controller with no receive path, delivering nothing and saying
// nothing. This used to be each port's own function, and the SDC one
// discarded the init result.
bool BtHciCtlrEnable(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (BtHciCtlrInit(pDev, pCfg) == false)
	{
		return false;
	}

	return BtHciCtlrStart(pDev, pCfg);
}

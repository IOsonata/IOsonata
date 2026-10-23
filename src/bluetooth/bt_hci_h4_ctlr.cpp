/**-------------------------------------------------------------------------
@file	bt_hci_h4_ctlr.cpp

@brief	Controller port of the HCI host for a controller behind an H4 link

See bt_hci_h4_ctlr.h. Everything the host asks of a BtHciCtlr is answered
here through the BtHciH4 layer of the link the transport opened: commands are
sent and their response waited for, ACL data is sent as H4 ACL packets, and
what the controller sends is framed from the Bluetooth work queue and handed
to the host receive handler. The controller itself needs nothing started, it
runs its own image; the link is reset to a known state with HCI Reset.

The link callbacks arrive from the transport. RxReady comes from the
interrupt of the link and only queues the processing; Command Complete and
Command Status are handled as soon as they are framed, the other packets in
arrival order from the queued processing.

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
#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "bluetooth/bt_hci_h4.h"
#include "bluetooth/bt_hci_h4_ctlr.h"
#include "bluetooth/bt_smp.h"		// BtCryptoCtlrRng
#include "crypto/crypto_uecc.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. A release build defines
// NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

// Internal host access; not an application API.
BtHciDevice_t *BtAppHciDevice(void);

// The link, once the transport opened it, and the controller device of the
// host it serves.
static BtHciH4Dev_t *s_pBtHciH4 = nullptr;
static BtHciCtlrDev_t *s_pBtHciCtlr = nullptr;

alignas(4) static uint8_t s_BtHciH4CtlrPktMem[BT_HCI_H4_PKTMEM_SIZE(BT_HCI_H4_CTLR_PKT_COUNT)];

// Deferred receive processing: a refused BtEvtQue leaves it pending,
// BtHciCtlrCheckStatus queues it again.
enum {
	BT_HCI_H4_CTLR_RX_IDLE,
	BT_HCI_H4_CTLR_RX_PENDING,
	BT_HCI_H4_CTLR_RX_QUEUED,
};

static volatile uint8_t s_BtHciH4CtlrRxState = BT_HCI_H4_CTLR_RX_IDLE;

static void BtHciH4CtlrRxEvt(uint32_t Evt, void *pCtx);

static void BtHciH4CtlrRxQue(void)
{
	if (s_BtHciH4CtlrRxState == BT_HCI_H4_CTLR_RX_PENDING)
	{
		s_BtHciH4CtlrRxState = BT_HCI_H4_CTLR_RX_QUEUED;
		if (BtEvtQue(0, nullptr, BtHciH4CtlrRxEvt) == false)
		{
			s_BtHciH4CtlrRxState = BT_HCI_H4_CTLR_RX_PENDING;
		}
	}
}

static void BtHciH4CtlrRxEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	// Packets arriving from here on queue the next event
	s_BtHciH4CtlrRxState = BT_HCI_H4_CTLR_RX_IDLE;

	if (s_pBtHciH4 != nullptr && BtHciH4Process(s_pBtHciH4, BT_HCI_H4_CTLR_RX_BURST))
	{
		// Packets left, handled by the next event
		s_BtHciH4CtlrRxState = BT_HCI_H4_CTLR_RX_PENDING;
		BtHciH4CtlrRxQue();
	}
}

// Packets arrived, from the interrupt of the link
void BtHciH4CtlrRxReady(void)
{
	if (s_BtHciH4CtlrRxState == BT_HCI_H4_CTLR_RX_IDLE)
	{
		s_BtHciH4CtlrRxState = BT_HCI_H4_CTLR_RX_PENDING;
		BtHciH4CtlrRxQue();
	}
}

// Command Complete and Command Status, as soon as they are framed. The host
// only sets the command response and credit fields of its device for them.
void BtHciH4CtlrCmdRsp(uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	(void)Type;
	(void)Len;

	if (s_pBtHciCtlr != nullptr && s_pBtHciCtlr->RxHandler != nullptr)
	{
		s_pBtHciCtlr->RxHandler(s_pBtHciCtlr, true, pPkt);
	}
}

// Every other packet, in arrival order, from the queued processing
void BtHciH4CtlrRx(uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	(void)Len;

	if (s_pBtHciCtlr == nullptr || s_pBtHciCtlr->RxHandler == nullptr)
	{
		return;
	}

	switch (Type)
	{
		case BT_HCI_H4_PKT_EVT:
			s_pBtHciCtlr->RxHandler(s_pBtHciCtlr, true, pPkt);
			break;

		case BT_HCI_H4_PKT_ACL:
			s_pBtHciCtlr->RxHandler(s_pBtHciCtlr, false, pPkt);
			break;

		default:
			// No synchronous or ISO data path in the host yet
			break;
	}
}

static size_t BtHciH4CtlrSend(BtHciCtlrDev_t * const pDev, void * const pData, size_t Len)
{
	(void)pDev;

	if (s_pBtHciH4 == nullptr)
	{
		return 0;
	}

	return BtHciH4Send(s_pBtHciH4, BT_HCI_H4_PKT_ACL, pData, (uint32_t)Len) ? Len : 0;
}

// Controller bring-up, reached from the generic BtHciCtlrEnable once
// BtHciCtlrInit has succeeded. The controller runs its own image; what is
// brought up here is the link to it.
bool BtHciCtlrStart(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg)
{
	(void)pCfg;

	if (pDev == nullptr)
	{
		return false;
	}

	BtHciDevice_t *pHci = BtAppHciDevice();
	if (pHci == nullptr)
	{
		return false;
	}

	s_pBtHciCtlr = pDev;
	pDev->Send = BtHciH4CtlrSend;

	s_pBtHciH4 = BtHciCtlrH4Open(s_BtHciH4CtlrPktMem, sizeof(s_BtHciH4CtlrPktMem));
	if (s_pBtHciH4 == nullptr)
	{
		DEBUG_PRINTF("HCI link open failed\r\n");
		return false;
	}

	// One command may be sent before the first Command Complete
	pHci->CmdCredit = 1;

	uint8_t res = BT_HCI_H4_ERR_TIMEOUT;
	for (int i = 0; i < BT_HCI_H4_CTLR_RESET_TRY && res != 0; i++)
	{
		// Bytes from the controller start can leave the framing out of step
		BtHciH4Flush(s_pBtHciH4);
		res = BtHciCommand(pHci, BT_HCI_CMD_BASEBAND_RESET, NULL, 0, NULL, 0);
	}

	if (res != 0)
	{
		DEBUG_PRINTF("HCI Reset failed %d\r\n", res);
		return false;
	}

	return true;
}

// Command executor of this port: send over the link and wait for the
// Command Complete or Command Status that names the opcode.
uint8_t BtHciCtlrCommand(BtHciDevice_t * const pDev, uint16_t OpCode, const void *pParam,
						 uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	if (s_pBtHciH4 == nullptr)
	{
		return BT_HCI_ERR_INVALID_COMMAND_PARAM;
	}

	return BtHciH4Command(s_pBtHciH4, pDev, OpCode, pParam, ParamLen, pRet, RetLen);
}

// Drain the packets of the link. The receive path normally runs from the
// work queue, this is for a caller that pumps the controller itself.
void BtHciCtlrProcess(BtHciCtlrDev_t * const pDev)
{
	(void)pDev;

	if (s_pBtHciH4 != nullptr)
	{
		while (BtHciH4Process(s_pBtHciH4, BT_HCI_H4_CTLR_RX_BURST))
		{
		}
	}
}

void BtHciCtlrCheckStatus(void)
{
	BtHciH4CtlrRxQue();
}

// Feature selection. A controller behind a link has every feature of its
// image and takes no resource configuration from the host, so there is
// nothing to switch on here. The functions exist because connection support
// and the scan and periodic advertising modules call them on every port.
void BtHciCtlrPeripheralSupport(void)
{
}

void BtHciCtlrLinkSupport(void)
{
}

void BtHciCtlrCentralSupport(void)
{
}

void BtHciCtlrPeriodicAdvSupport(void)
{
}

// P-256 key agreement for LE Secure Connections. The host MCU is not known
// here, so the software P-256 is used, with the random numbers of the
// controller. Linked only when the security module references it.
KeyAgreeEngine *BtHciCtlrKeyAgreeEngine(void)
{
	alignas(uint64_t) static uint8_t s_CryptoEcdhMem[CRYPTO_UECC_MEMSIZE];
	KeyAgreeEngine *pEcdh = CryptoUeccCreate(s_CryptoEcdhMem, sizeof(s_CryptoEcdhMem),
											  BtCryptoCtlrRng());
	if (pEcdh != nullptr)
	{
		DEBUG_PRINTF("Crypto ECDH engine: CryptoUecc (software P-256)\r\n");
	}

	return pEcdh;
}

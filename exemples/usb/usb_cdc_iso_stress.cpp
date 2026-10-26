/**-------------------------------------------------------------------------
@example	usb_cdc_iso_stress.cpp

@brief USB CDC and ISO concurrent loopback stress test.

One CDC function and one bidirectional ISO function use five RA4M1 data pipes.
Host runner: Python/usb_cdc_iso_stress.py

@author	Hoang Nguyen Hoan
@date	Sep. 22, 2026

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

#include <string.h>

#if __has_include("board.h")
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "board.h"
#endif
#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#include "app_evt_handler.h"
#include "usb/usbd_cdc.h"
#include "usb/usbd_iso.h"

#define USB_DEVNO			0
#define CDC_BUFFER_SIZE		USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK)

#define CDC_RXFIFO_PKTCNT	4
#define CDC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, CDC_BUFFER_SIZE)
#define LOOPBACK_TXFIFO_MEMSIZE	CFIFO_MEMSIZE(1024)

#define CDC_ISO_STR_INTERFACE	4U


#define ISO_ALT_COUNT			6U
#define ISO_MAX_MPS			63U
#define ISO_REQ_GET_DIAG		0x5AU

#pragma pack(push, 1)
typedef struct __CdcIso_Iso_Diag {
	uint32_t RxMissCnt;
	uint32_t TxMissCnt;
	uint32_t LoopbackDropCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
} CdcIsoIsoDiag_t;
#pragma pack(pop)

typedef struct __CdcIso_Function_State {
	uint32_t IsoLoopbackDropCnt;
} CdcIsoFunctionState_t;

static bool IsoControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
	uint8_t **ppData, uint16_t *pLength);
static void IsoClearDiag(void);

class IsoLoopbackClass final : public UsbdIso {
public:
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
		uint8_t **ppData, uint16_t *pLength) override {
		return IsoControl(pSetup, Stage, ppData, pLength);
	}
protected:
	bool OpenTransport(uint16_t Mps, uint8_t Interval) override {
		IsoClearDiag();
		return UsbdIso::OpenTransport(Mps, Interval);
	}
};

alignas(4) static uint8_t s_LoopbackRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_LoopbackTxFifoMem[LOOPBACK_TXFIFO_MEMSIZE];
static UsbdCdc s_LoopbackCdc;

static const UsbdCdcCfg_t s_LoopbackCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_LoopbackRxFifoMem,
	.TxFifoMemSize = LOOPBACK_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_LoopbackTxFifoMem,
	.EvtCB = nullptr,
};

static CdcIsoFunctionState_t s_Fn;
static const UsbdPeriodicAlt_t s_IsoAlternates[] = {
	{9, 9, 1, 4}, {17, 17, 1, 4}, {25, 25, 1, 4},
	{33, 33, 1, 4}, {49, 49, 1, 4}, {63, 63, 1, 4},
};
static IsoLoopbackClass s_Iso;

static CdcIsoIsoDiag_t s_IsoDiag;




#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0008,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB CDC ISO Stress",
	.pSerial = nullptr,
	.pFuncName = "USB CDC ISO Stress",
	.IntPrio = 6,
#ifdef USB_PINS
	.pIOPinMap = s_UsbPins,
	.NbIOPins = sizeof(s_UsbPins) / sizeof(IOPinCfg_t),
#else
	.pIOPinMap = nullptr,
	.NbIOPins = 0,
#endif
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2U,
	.DeviceProtocol = 1U,
	.bSelfPowered = false,
	.bRemoteWakeup = true,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

static const char s_Banner[] = "\r\nIOsonata USB CDC ISO Stress\r\n";

// Application event queue memory, replaces the 4 event library default. The
// USB controller port queues its deferred endpoint events there.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

// ISO loopback: every received frame goes back out on the next interval.
// Frames are pulled from the RX FIFO with UsbIsoIntrf::Rx and queued with
// UsbIsoIntrf::Tx, the same way any DeviceIntrf user moves data.
static int IsoEvent(DevIntrf_t * const, DEVINTRF_EVT Event,
	uint8_t *, int Length)
{
	if (Event != DEVINTRF_EVT_RX_DATA)
	{
		return 0;
	}

	uint8_t frame[ISO_MAX_MPS];
	int total = 0;
	int len;
	while ((len = s_Iso.Rx(0, frame, sizeof(frame))) > 0)
	{
		if (s_Iso.Tx(0, frame, len) != len)
		{
			s_Fn.IsoLoopbackDropCnt++;
		}
		total += len;
	}
	(void)Length;
	return total;
}

static bool IsoControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
	uint8_t **ppData, uint16_t *pLength)
{
	// The core supplies the setup copy, data pointer and length.
	if (pSetup->bmRequestType !=
			(USB_REQTYPE_DIRHOST | USB_REQTYPE_VEND | USB_REQTYPE_INTERFACE) ||
		pSetup->wValue != 0U ||
		pSetup->wIndex != s_Iso.FirstInterface())
	{
		return false;
	}

	if (pSetup->bRequest != ISO_REQ_GET_DIAG ||
		pSetup->wLength != sizeof(s_IsoDiag))
	{
		return false;
	}
	if (Stage != USB_CTRL_SETUP)
	{
		return true;
	}
	const UsbIsoIntrf_t *pIso = s_Iso;
	s_IsoDiag.RxMissCnt = pIso->RxMissCnt;
	s_IsoDiag.TxMissCnt = pIso->TxMissCnt;
	s_IsoDiag.LoopbackDropCnt = s_Fn.IsoLoopbackDropCnt;
	s_IsoDiag.RxEmptyCnt = pIso->RxEmptyCnt;
	s_IsoDiag.TxEmptyCnt = pIso->TxEmptyCnt;
	*ppData = reinterpret_cast<uint8_t *>(&s_IsoDiag);
	*pLength = sizeof(s_IsoDiag);
	return true;
}

static void IsoClearDiag(void)
{
	s_Fn.IsoLoopbackDropCnt = 0U;
	UsbIsoIntrf_t *pIso = s_Iso;
	pIso->RxMissCnt = 0U;
	pIso->TxMissCnt = 0U;
	pIso->RxEmptyCnt = 0U;
	pIso->TxEmptyCnt = 0U;
}

static bool IsoInit(void)
{
	UsbdIsoCfg_t cfg = {};
	cfg.Function = {USB_DEVNO, s_IsoAlternates, ISO_ALT_COUNT, 0, 0, CDC_ISO_STR_INTERFACE};
	cfg.EvtCB = IsoEvent;
	return s_Iso.Init(cfg);
}


int main()
{
	uint8_t loopbackBuffer[CDC_BUFFER_SIZE];
	int loopbackPending = 0;
	int loopbackOffset = 0;
	bool loopbackConnected = false;

	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg) ||
		!s_LoopbackCdc.Init(s_LoopbackCfg) ||
		!IsoInit())
	{
		return -1;
	}

	if (!UsbEnable(USB_DEVNO))
	{
		return -1;
	}
	while (1)
	{
		AppEvtHandlerExec();

		// A port open starts a new test:
		// drop any echo still pending from the previous session.
		const bool connected = s_LoopbackCdc.IsPortOpen();
		if (connected != loopbackConnected)
		{
			loopbackConnected = connected;
			loopbackPending = 0;
			loopbackOffset = 0;

			if (connected)
			{
				s_LoopbackCdc.Tx(0, reinterpret_cast<const uint8_t *>(s_Banner),
					(int)sizeof(s_Banner) - 1);
			}
		}

		if (loopbackConnected)
		{
			if (loopbackPending > 0)
			{
				const int length = s_LoopbackCdc.Tx(
					0, &loopbackBuffer[loopbackOffset], loopbackPending);
				if (length > 0)
				{
					loopbackOffset += length;
					loopbackPending -= length;
				}
			}
			else
			{
				const int length = s_LoopbackCdc.Rx(
					0, loopbackBuffer, sizeof(loopbackBuffer));
				if (length > 0)
				{
					loopbackPending = length;
					loopbackOffset = 0;
				}
			}
		}

	}

	return 0;
}

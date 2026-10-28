/**-------------------------------------------------------------------------
@example	usb_int_loopback.cpp

@brief	USB interrupt loopback for UsbIntIntrf hardware validation.

This example exposes one vendor-specific interface with alternate setting 0
disabled and alternate settings 1 through 3 selecting different interrupt
endpoint intervals. The USB core allocator chooses the interface and endpoint
pair. Every completed OUT packet is echoed through the independent DIRECT TX
slot; no CFifo or HID behavior is involved.

The host test is Python/usb_int_loopback.py. A vendor/interface IN request
(bRequest 0x5B) returns loopback diagnostics. The request is deliberately part
of this test application, not UsbIntIntrf.

@author	Hoang Nguyen Hoan
@date	Sep. 10, 2026

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

#include "app_evt_handler.h"
#include "usb/usb.h"
#if __has_include("board.h")
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "board.h"
#endif
#include "usb/usbd_int.h"
#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

// board.h can select another controller
#ifndef USB_DEVNO
#define USB_DEVNO			0
#endif
#define INT_ALT_COUNT		3U
#define INT_MPS				64U
#define INT_REQ_GET_DIAG	0x5BU

#define INT_DIAG_FLAG_OPENED		(1U << 0)
#define INT_DIAG_FLAG_SUSPENDED		(1U << 1)
#define INT_DIAG_FLAG_TX_READY		(1U << 2)

#define INT_STR_INTERFACE	4U

#pragma pack(push, 1)
typedef struct __Int_Diag {
	uint32_t RxCnt;
	uint32_t TxSubmitCnt;
	uint32_t TxDoneCnt;
	uint32_t TxFailCnt;
	uint32_t LoopbackDropCnt;
	uint32_t CoreRxDropCnt;
	uint32_t RxErrorCnt;
	uint32_t TxErrorCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
	uint16_t LastRxLength;
	uint16_t LastTxLength;
	uint16_t Mps;
	uint8_t Interval;
	uint8_t Alt;
	uint8_t Flags;
	uint8_t Reserved;
} IntDiag_t;
#pragma pack(pop)

static_assert(sizeof(IntDiag_t) == 50U,
	"interrupt diagnostic wire format changed");

typedef struct __Int_Function_State {
	uint32_t RxCnt;
	uint32_t TxSubmitCnt;
	uint32_t TxDoneCnt;
	uint32_t TxFailCnt;
	uint32_t LoopbackDropCnt;
	uint16_t LastRxLength;
	uint16_t LastTxLength;
} IntFunctionState_t;

static bool IntControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
					   uint8_t **ppData, uint16_t *pLength);
static void IntClearDiag(void);
static int IntEvent(DevIntrf_t *, DEVINTRF_EVT, uint8_t *, int);

// Only application diagnostics extend the reusable device class.
class IntLoopbackClass final : public UsbdInt
{
public:
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
		uint8_t **ppData, uint16_t *pLength) override
		{
		return IntControl(pSetup, Stage, ppData, pLength);
	}
	void Reset(void) override
	{
		UsbdInt::Reset();
		IntClearDiag();
	}

protected:
	bool OpenTransport(uint16_t Mps, uint8_t Interval) override
	{
		IntClearDiag();
		return UsbdInt::OpenTransport(Mps, Interval);
	}
};

static const UsbdPeriodicAlt_t s_IntAlternates[] = {
	{ 64U, 64U, 1U, 1U },
	{ 64U, 64U, 4U, 4U },
	{ 64U, 64U, 16U, 16U },
};

// Application event queue memory, replaces the 4 event library default. The
// USB controller port queues its deferred endpoint events there.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0004,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB Interrupt Loopback",
	.pSerial = nullptr,
	.pFuncName = "USB Interrupt Loopback",
	.IntPrio = 6,
#ifdef USB_PINS
	.pIOPinMap = s_UsbPins,
	.NbIOPins = sizeof(s_UsbPins) / sizeof(IOPinCfg_t),
#else
	.pIOPinMap = nullptr,
	.NbIOPins = 0,
#endif
	.DeviceClass = USB_DEVCLASS_NONE,
	.DeviceSubClass = 0U,
	.DeviceProtocol = 0U,
	.bSelfPowered = false,
	.bRemoteWakeup = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

alignas(4) static uint8_t s_IntRxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_IntTxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
static IntLoopbackClass s_Int;

static IntFunctionState_t s_Fn;
static IntDiag_t s_DiagReply;


static void IntClearDiag(void)
{
	s_Fn.RxCnt = 0U;
	s_Fn.TxSubmitCnt = 0U;
	s_Fn.TxDoneCnt = 0U;
	s_Fn.TxFailCnt = 0U;
	s_Fn.LoopbackDropCnt = 0U;
	s_Fn.LastRxLength = 0U;
	s_Fn.LastTxLength = 0U;

	UsbIntIntrf_t *pInt = s_Int;
	pInt->pData->RxDropCnt = 0U;
	pInt->RxErrorCnt = 0U;
	pInt->TxErrorCnt = 0U;
	pInt->RxEmptyCnt = 0U;
	pInt->TxEmptyCnt = 0U;
}

static void IntBuildDiag(void)
{
	const UsbIntIntrf_t *pInt = s_Int;

	memset(&s_DiagReply, 0, sizeof(s_DiagReply));
	s_DiagReply.RxCnt = s_Fn.RxCnt;
	s_DiagReply.TxSubmitCnt = s_Fn.TxSubmitCnt;
	s_DiagReply.TxDoneCnt = s_Fn.TxDoneCnt;
	s_DiagReply.TxFailCnt = s_Fn.TxFailCnt;
	s_DiagReply.LoopbackDropCnt = s_Fn.LoopbackDropCnt;
	s_DiagReply.CoreRxDropCnt = pInt->pData->RxDropCnt;
	s_DiagReply.RxErrorCnt = pInt->RxErrorCnt;
	s_DiagReply.TxErrorCnt = pInt->TxErrorCnt;
	s_DiagReply.RxEmptyCnt = pInt->RxEmptyCnt;
	s_DiagReply.TxEmptyCnt = pInt->TxEmptyCnt;
	s_DiagReply.LastRxLength = s_Fn.LastRxLength;
	s_DiagReply.LastTxLength = s_Fn.LastTxLength;
	s_DiagReply.Mps = pInt->Mps;
	s_DiagReply.Interval = pInt->Interval;
	s_DiagReply.Alt = s_Int.Alternate();

	if (pInt->pData->Mps != 0U)
	{
		s_DiagReply.Flags |= INT_DIAG_FLAG_OPENED;
	}
	if (UsbSuspended(USB_DEVNO))
	{
		s_DiagReply.Flags |= INT_DIAG_FLAG_SUSPENDED;
	}
	if (pInt->pData->Mps != 0U &&
		atomic_load_explicit(&s_Int.Data()->bTxReady,
			memory_order_acquire))
	{
		s_DiagReply.Flags |= INT_DIAG_FLAG_TX_READY;
	}
}

static bool IntSend(const uint8_t *pData, uint16_t Length)
{
	if (Length != 0U)
	{
		return s_Int.Tx(0, pData, Length) == (int)Length;
	}
	if (!s_Int.StartTx(0))
	{
		return false;
	}
	(void)s_Int.TxData(nullptr, 0);
	const bool accepted = !atomic_load_explicit(&s_Int.Data()->bTxReady,
		memory_order_acquire);
	s_Int.StopTx();
	return accepted;
}

static int IntEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length)
{
	const UsbCtrlrXferResult_t result =
		(event == DEVINTRF_EVT_RX_TIMEOUT || event == DEVINTRF_EVT_TX_TIMEOUT) ?
		USB_CTRLR_XFER_FAILED : USB_CTRLR_XFER_SUCCESS;
	if (event == DEVINTRF_EVT_RX_DATA || event == DEVINTRF_EVT_RX_TIMEOUT)
	{
		if (result != USB_CTRLR_XFER_SUCCESS)
		{
			return result == USB_CTRLR_XFER_SUCCESS ? Length : 0;
		}

		s_Fn.RxCnt++;
		s_Fn.LastRxLength = Length;
		if (IntSend(pData, Length))
		{
			s_Fn.TxSubmitCnt++;
		}
		else
		{
			s_Fn.LoopbackDropCnt++;
		}
	}
	else if (event == DEVINTRF_EVT_TX_FIFO_EMPTY || event == DEVINTRF_EVT_TX_TIMEOUT)
	{
		s_Fn.LastTxLength = Length;
		if (result == USB_CTRLR_XFER_SUCCESS)
		{
			s_Fn.TxDoneCnt++;
		}
		else
		{
			s_Fn.TxFailCnt++;
		}
	}
	return result == USB_CTRLR_XFER_SUCCESS ? Length : 0;
}

static bool IntControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
					   uint8_t **ppData, uint16_t *pLength)
{
	if (pSetup == nullptr)
	{
		return false;
	}
	if (Stage != USB_CTRL_SETUP)
	{
		return true;
	}
	if (ppData == nullptr || pLength == nullptr ||
		pSetup->bmRequestType !=
			(USB_REQTYPE_DIRHOST | USB_REQTYPE_VEND | USB_REQTYPE_INTERFACE) ||
		pSetup->bRequest != INT_REQ_GET_DIAG || pSetup->wValue != 0U ||
		pSetup->wIndex != s_Int.FirstInterface() || pSetup->wLength != sizeof(IntDiag_t))
	{
		return false;
	}

	IntBuildDiag();
	*ppData = reinterpret_cast<uint8_t *>(&s_DiagReply);
	*pLength = sizeof(s_DiagReply);
	return true;
}

static const UsbdIntCfg_t s_IntCfg = {
	.Function = {
		.DevNo = USB_DEVNO,
		.pAlternates = s_IntAlternates,
		.AltCount = INT_ALT_COUNT,
		.SubClass = 0U,
		.Protocol = 0U,
		.InterfaceString = INT_STR_INTERFACE,
	},
	.EvtCB = IntEvent,
	.pContext = nullptr,
	.pRxBuffer = s_IntRxBuffer,
	.pTxBuffer = s_IntTxBuffer,
};

int main()
{
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg) || !s_Int.Init(s_IntCfg))
	{
		return -1;
	}

	(void)UsbEnable(USB_DEVNO);
	AppRun();
	return 0;
}

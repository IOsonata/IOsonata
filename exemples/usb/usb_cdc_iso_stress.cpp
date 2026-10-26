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
#include "usb/usb_iso.h"
#include "usb/usbd_epalloc.h"

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
typedef struct __CdcIso_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} CdcIsoAltDesc_t;


typedef struct __CdcIso_Iso_Function_Descriptor {
	UsbIntrfDesc_t Alt0;
	CdcIsoAltDesc_t Alt[ISO_ALT_COUNT];
} CdcIsoFunctionDesc_t;

typedef struct __CdcIso_Iso_Diag {
	uint32_t RxMissCnt;
	uint32_t TxMissCnt;
	uint32_t LoopbackDropCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
} CdcIsoIsoDiag_t;
#pragma pack(pop)

typedef struct __CdcIso_Function_State {
	bool IsoConfigured;
	uint8_t IsoAlt;
	uint8_t IsoInterfaceNo;
	uint8_t IsoEpNo;
	uint32_t IsoLoopbackDropCnt;
} CdcIsoFunctionState_t;

static bool IsoControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
	uint8_t **ppData, uint16_t *pLength);
static bool IsoSelectConfig(uint8_t Configuration);
static bool IsoSelectInterface(uint8_t InterfaceNo, uint8_t Alt);
static void IsoReset(void);
static void IsoProcess(void);
static void CdcIsoBuildFunctionDesc(const UsbDeviceClass *pClass, uint8_t *pData,
	UsbSpeed_t Speed);

class IsoLoopbackClass final : public UsbDeviceClass {
public:
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
				 uint8_t **ppData, uint16_t *pLength) override {
		return IsoControl(pSetup, Stage, ppData, pLength);
	}
	bool SelectConfig(uint8_t ConfigValue) override {
		return IsoSelectConfig(ConfigValue);
	}
	bool SelectInterface(uint8_t InterfaceNo, uint8_t Option) override {
		return IsoSelectInterface(InterfaceNo, Option);
	}
	void Reset(void) override { IsoReset(); }
	void Process(void) override { IsoProcess(); }
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
static constexpr uint16_t s_IsoMps[ISO_ALT_COUNT] = {9U, 17U, 25U, 33U, 49U, 63U};
static UsbIsoIntrf s_Iso;

static CdcIsoIsoDiag_t s_IsoDiag;

static IsoLoopbackClass s_IsoClass;

static constexpr UsbIntrfDesc_t s_CdcIsoAlt0Desc = {
	.bLength = sizeof(UsbIntrfDesc_t),
	.bDescriptorType = USB_DESCTYPE_INTERFACE,
	.bInterfaceNumber = 0U,
	.bAlternateSetting = 0U,
	.bNumEndpoints = 0U,
	.bInterfaceClass = USB_INTRFCLASS_VENDOR,
	.bInterfaceSubClass = 0U,
	.bInterfaceProtocol = 0U,
	.iInterface = CDC_ISO_STR_INTERFACE,
};

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
		pSetup->wIndex != s_Fn.IsoInterfaceNo)
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

static bool IsoSelectConfig(uint8_t Configuration)
{
	s_Iso.Close();
	s_Fn.IsoConfigured = false;
	s_Fn.IsoAlt = 0U;
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != 1U)
	{
		return false;
	}
	s_Fn.IsoConfigured = true;
	return true;
}

static bool IsoSelectInterface(uint8_t InterfaceNo, uint8_t Alt)
{
	if (!s_Fn.IsoConfigured || InterfaceNo != s_Fn.IsoInterfaceNo ||
		Alt > ISO_ALT_COUNT)
	{
		return false;
	}

	s_Iso.Close();
	s_Fn.IsoAlt = 0U;
	if (Alt == 0U)
	{
		return true;
	}

	s_Fn.IsoLoopbackDropCnt = 0U;
	UsbIsoIntrf_t *pIso = s_Iso;
	pIso->RxMissCnt = 0U;
	pIso->TxMissCnt = 0U;
	pIso->RxEmptyCnt = 0U;
	pIso->TxEmptyCnt = 0U;
	const uint8_t interval = UsbCtrlrHighSpeed(USB_DEVNO) ? 4U : 1U;
	if (!s_Iso.Open(s_IsoMps[Alt - 1U], interval))
	{
		return false;
	}
	s_Fn.IsoAlt = Alt;
	return true;
}

static void IsoReset(void)
{
	s_Fn.IsoConfigured = false;
	s_Fn.IsoAlt = 0U;
	s_Iso.Reset();
}

static void IsoProcess(void)
{
	if (!s_Fn.IsoConfigured || s_Fn.IsoAlt == 0U)
	{
		return;
	}
	const UsbIsoIntrf_t *pIso = s_Iso;
	const bool suspended = UsbSuspended(USB_DEVNO);
	if (suspended && !pIso->Suspended)
	{
		s_Iso.Suspend();
	}
	else if (!suspended && pIso->Suspended)
	{
		(void)s_Iso.Resume();
	}
}

// Alternate setting zero is idle; settings 1..6 select ISO packet size.
static void CdcIsoBuildFunctionDesc(const UsbDeviceClass *pClass, uint8_t *pData,
	UsbSpeed_t Speed)
{
	(void)pClass;
	const uint8_t interfaceNo = s_Fn.IsoInterfaceNo;
	const uint8_t epNo = s_Fn.IsoEpNo;
	const unsigned altCount = ISO_ALT_COUNT;
	const uint8_t attributes = USB_ENDPATT_TRANS_ISO;
	const uint8_t isoInterval = Speed == USB_SPEED_HIGH ? 4U : 1U;

	UsbIntrfDesc_t *pAlt0 = reinterpret_cast<UsbIntrfDesc_t *>(pData);
	*pAlt0 = s_CdcIsoAlt0Desc;
	pAlt0->bInterfaceNumber = interfaceNo;
	CdcIsoAltDesc_t *pAlt = reinterpret_cast<CdcIsoAltDesc_t *>(pAlt0 + 1);
	for (unsigned i = 0U; i < altCount; i++, pAlt++)
	{
		pAlt->Interface = *pAlt0;
		pAlt->Interface.bAlternateSetting = (uint8_t)(i + 1U);
		pAlt->Interface.bNumEndpoints = 2U;
		pAlt->Out.bLength = sizeof(pAlt->Out);
		pAlt->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		pAlt->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(epNo);
		pAlt->Out.bmAttributes = attributes;
		pAlt->Out.wMaxPacketSize = s_IsoMps[i];
		pAlt->Out.bInterval = isoInterval;
		pAlt->In = pAlt->Out;
		pAlt->In.bEndpointAddress = USB_ENDPADDR_DIRIN(epNo);
	}
}

static bool IsoInit(void)
{
	if (!USB_ISO_SUPPORTED(USB_DEVNO))
	{
		return false;
	}

	uint16_t isoMask = (uint16_t)(
		USB_ISO_EPIN_MASK(USB_DEVNO) & USB_ISO_EPOUT_MASK(USB_DEVNO));
	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	// The allocator fills each requested result before returning success.
	UsbdEpAllocRes_t alloc;
	uint8_t epNo = 0U;
	// Some controllers share ISO-capable endpoint numbers with CDC. Try
	// supported pairs until the allocator accepts one that is still free.
	while (isoMask != 0U)
	{
		epNo = (uint8_t)__builtin_ctz(isoMask);
		req.FixedInMask = (uint16_t)(1U << epNo);
		req.FixedOutMask = req.FixedInMask;
		if (UsbdEpAlloc(USB_DEVNO, &req, &s_IsoClass, &alloc))
		{
			break;
		}
		isoMask &= (uint16_t)(isoMask - 1U);
	}
	if (isoMask == 0U)
	{
		return false;
	}

	s_Fn.IsoInterfaceNo = alloc.FirstInterface;
	s_Fn.IsoEpNo = epNo;

	// No FIFO memory given: the UsbIsoIntrf object uses its own.
	UsbIsoIntrfCfg_t cfg = {};
	cfg.DevNo = USB_DEVNO;
	cfg.EpNo = s_Fn.IsoEpNo;
	cfg.pRxFifoMem = nullptr;
	cfg.pTxFifoMem = nullptr;
	cfg.EvtCB = IsoEvent;
	cfg.pContext = nullptr;
	if (!s_Iso.Init(cfg))
	{
		return false;
	}

	return UsbDescRegister(USB_DEVNO, &s_IsoClass,
		nullptr, sizeof(CdcIsoFunctionDesc_t), CdcIsoBuildFunctionDesc);
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


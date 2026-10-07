/**-------------------------------------------------------------------------
@example	usb_combo_stress_taktos.cpp

@brief	Multithreaded USB composite stress test with TaktOS.

Runs all USB data paths on one controller at the same time. Two CDC functions
retain the UsbDualCdcStress loopback/PRBS behavior. HID, raw interrupt and
isochronous functions each provide a bidirectional loopback. Interface and
endpoint numbers are assigned by the common allocator; ISO reserves the
controller-supported ISO endpoint.

Host runner: Python/usb_combo_stress.py

@author	Hoang Nguyen Hoan
@date	Oct. 2, 2026

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

#include <atomic>
#include <string.h>

#include "cfifo.h"
#include "prbs.h"
#include "usb_combo_stress.h"
#include "usb/usb_int.h"
#include "usb/usb_iso.h"
#include "usb/usbd_epalloc.h"
#include "usb/usbd_hid.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "TaktOS.h"
#include "TaktOSThread.h"
#include "TaktOSSem.h"

// Bound traffic work between yields while keeping the bare-metal per-byte
// PRBS Tx calls. Loopback may receive and echo in the same pass.
#define CDC_PASSES_PER_TURN 4U
#define USB_WORK_QUE_SIZE 16U

// Deferred USB work: queued by the controller interrupt, run by ServiceThread.
typedef struct __Usb_Work {
	uint32_t EvtId;
	void *pCtx;
	UsbEvtQueHandler_t Handler;
} UsbWork_t;

static int HidEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length);
static bool IntSelectConfig(uint8_t Configuration);
static bool IntSelectInterface(uint8_t InterfaceNo, uint8_t Alt);
static void IntReset(void);
static bool IsoControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
	uint8_t **ppData, uint16_t *pLength);
static bool IsoSelectConfig(uint8_t Configuration);
static bool IsoSelectInterface(uint8_t InterfaceNo, uint8_t Alt);
static void IsoReset(void);
static void IsoProcess(void);
static void ComboBuildFunctionDesc(const UsbDeviceClass *pClass, uint8_t *pData,
	UsbSpeed_t Speed);

class IntLoopbackClass final : public UsbDeviceClass {
public:
	bool SelectConfig(uint8_t ConfigValue) override {
		return IntSelectConfig(ConfigValue);
	}
	bool SelectInterface(uint8_t InterfaceNo, uint8_t Option) override {
		return IntSelectInterface(InterfaceNo, Option);
	}
	void Reset(void) override { IntReset(); }
};

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
alignas(4) static uint8_t s_PrbsRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_PrbsTxFifoMem[PRBS_TXFIFO_MEMSIZE];

static UsbdCdc s_LoopbackCdc;
static UsbdCdc s_PrbsCdc;

static const UsbdCdcCfg_t s_LoopbackCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_LoopbackRxFifoMem,
	.TxFifoMemSize = LOOPBACK_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_LoopbackTxFifoMem,
	.EvtCB = nullptr,
};

static const UsbdCdcCfg_t s_PrbsCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_PrbsRxFifoMem,
	.TxFifoMemSize = PRBS_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_PrbsTxFifoMem,
	.EvtCB = nullptr,
};

static const uint8_t s_HidReportDesc[] = {
	0x06U, 0x00U, 0xFFU,
	0x09U, 0x01U,
	0xA1U, 0x01U,
	0x75U, 0x08U,
	0x95U, HID_REPORT_SIZE,
	0x09U, 0x01U,
	0x81U, 0x02U,
	0x95U, HID_REPORT_SIZE,
	0x09U, 0x01U,
	0x91U, 0x02U,
	0xC0U,
};

static uint8_t s_HidPending[HID_REPORT_SIZE];
static uint16_t s_HidPendingLength;
static UsbdHid s_Hid;

alignas(4) static uint8_t s_HidRxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_HidTxBuffer[USB_INT_INTRF_PKT_BLKSIZE];

static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = USB_DEVNO,
	.pReportDesc = s_HidReportDesc,
	.ReportDescLength = sizeof(s_HidReportDesc),
	.BcdHid = 0U,
	.FsMps = HID_REPORT_SIZE,
	.HsMps = HID_REPORT_SIZE,
	.FsInterval = 1U,
	.HsInterval = 4U,
	.SubClass = USB_HID_SUBCLASS_NONE,
	.Protocol = USB_HID_PROT_NONE,
	.CountryCode = 0U,
	.InterfaceString = COMBO_STR_INTERFACE,
	.EvtCB = HidEvent,
	.pContext = nullptr,
	.pRxBuffer = s_HidRxBuffer,
	.pTxBuffer = s_HidTxBuffer,
};

static constexpr uint8_t s_IntIntervals[INT_ALT_COUNT] = { 1U, 4U, 16U };

alignas(4) static uint8_t s_IntRxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_IntTxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
static UsbIntIntrf s_Int;

// Small INT and ISO function state in one block: one address literal serves
// every access.
static ComboFunctionState_t s_Fn;

static IntLoopbackClass s_IntClass;

static constexpr uint16_t s_IsoMps[ISO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};

static UsbIsoIntrf s_Iso;

static ComboIsoDiag_t s_IsoDiag;

static IsoLoopbackClass s_IsoClass;

static constexpr UsbIntrfDesc_t s_ComboAlt0Desc = {
	.bLength = sizeof(UsbIntrfDesc_t),
	.bDescriptorType = USB_DESCTYPE_INTERFACE,
	.bInterfaceNumber = 0U,
	.bAlternateSetting = 0U,
	.bNumEndpoints = 0U,
	.bInterfaceClass = USB_INTRFCLASS_VENDOR,
	.bInterfaceSubClass = 0U,
	.bInterfaceProtocol = 0U,
	.iInterface = COMBO_STR_INTERFACE,
};

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0008,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB Combo Stress",
	.pSerial = nullptr,
	.pFuncName = "USB Combo Stress",
	.IntPrio = 6,
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2U,
	.DeviceProtocol = 1U,
	.bSelfPowered = false,
	.bRemoteWakeup = true,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

alignas(8) static uint8_t s_ServiceMem[TAKTOS_THREAD_MEM_SIZE(2048)];
alignas(8) static uint8_t s_LoopMem[TAKTOS_THREAD_MEM_SIZE(1024)];
alignas(8) static uint8_t s_PrbsMem[TAKTOS_THREAD_MEM_SIZE(1024)];
alignas(8) static uint8_t s_HeartbeatMem[TAKTOS_THREAD_MEM_SIZE(512)];

// LoopThread publishes a cumulative count; PrbsThread tracks how many error
// markers it has sent. Neither thread modifies the other's counter.
static_assert(std::atomic<uint32_t>::is_always_lock_free);
static std::atomic<uint32_t> s_LoopErrors{0};
volatile uint32_t g_UsbComboTaktOSHeartbeat = 0;

static const char s_Banner[] = "\r\nIOsonata USB Combo Stress\r\n";
static_assert(sizeof(s_Banner) - 1 <= CDC_BUFFER_SIZE);

alignas(4) static uint8_t s_UsbWorkMem[
	CFIFO_TOTAL_MEMSIZE(USB_WORK_QUE_SIZE, sizeof(UsbWork_t))];
static hCFifo_t s_hUsbWork;
static TaktOSSem_t s_ServiceWake;

static int HidEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length)
{
	// Only RX_DATA uses the return value: returning Length frees the slot.
	if (event == DEVINTRF_EVT_RX_DATA)
	{
		// UsbIntIntrfDataEvent already rejects reports larger than the MPS.
		if (s_Hid.Tx(0, pData, Length) != Length)
		{
			memcpy(s_HidPending, pData, Length);
			s_HidPendingLength = Length;
		}
	}
	else if (event == DEVINTRF_EVT_TX_FIFO_EMPTY && s_HidPendingLength != 0U)
	{
		const uint16_t length = s_HidPendingLength;
		s_HidPendingLength = 0U;
		if (s_Hid.Tx(0, s_HidPending, length) != (int)length)
		{
			s_HidPendingLength = length;
		}
	}
	return Length;
}

static int IntEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length)
{
	if (event == DEVINTRF_EVT_RX_DATA)
	{
		(void)s_Int.Tx(0, pData, Length);
	}
	return Length;
}

static bool IntSelectConfig(uint8_t Configuration)
{
	s_Int.Close();
	s_Fn.IntConfigured = false;
	s_Fn.IntAlt = 0U;
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != 1U)
	{
		return false;
	}
	s_Fn.IntConfigured = true;
	return true;
}

static bool IntSelectInterface(uint8_t InterfaceNo, uint8_t Alt)
{
	if (!s_Fn.IntConfigured || InterfaceNo != s_Fn.IntInterfaceNo ||
		Alt > INT_ALT_COUNT)
	{
		return false;
	}

	s_Int.Close();
	s_Fn.IntAlt = 0U;
	if (Alt == 0U)
	{
		return true;
	}
	if (!s_Int.Open(INT_MPS, s_IntIntervals[Alt - 1U]))
	{
		return false;
	}
	s_Fn.IntAlt = Alt;
	return true;
}

static void IntReset(void)
{
	s_Fn.IntConfigured = false;
	s_Fn.IntAlt = 0U;
	s_Int.Reset();
}

static bool IntInit(void)
{
	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.BidirectionalCount = 1U;
	// The allocator fills each requested result before returning success.
	UsbdEpAllocRes_t alloc;
	if (!UsbdEpAlloc(USB_DEVNO, &req, &s_IntClass, &alloc))
	{
		return false;
	}

	s_Fn.IntInterfaceNo = alloc.FirstInterface;
	s_Fn.IntEpNo = alloc.Bidirectional[0];

	UsbIntIntrfCfg_t cfg;
	cfg.DevNo = USB_DEVNO;
	cfg.EpNo = s_Fn.IntEpNo;
	cfg.EvtCB = IntEvent;
	cfg.pContext = nullptr;
	cfg.pRxBuffer = s_IntRxBuffer;
	cfg.pTxBuffer = s_IntTxBuffer;
	if (!s_Int.Init(cfg))
	{
		return false;
	}

	return UsbDescRegister(USB_DEVNO, &s_IntClass,
		nullptr, sizeof(ComboIntFunctionDesc_t), ComboBuildFunctionDesc);
}

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

// INT and ISO use the same interface/OUT/IN descriptor layout.
static void ComboBuildFunctionDesc(const UsbDeviceClass *pClass, uint8_t *pData,
	UsbSpeed_t Speed)
{
	const bool isInt = pClass == &s_IntClass;
	const uint8_t interfaceNo = isInt ? s_Fn.IntInterfaceNo : s_Fn.IsoInterfaceNo;
	const uint8_t epNo = isInt ? s_Fn.IntEpNo : s_Fn.IsoEpNo;
	const unsigned altCount = isInt ? INT_ALT_COUNT : ISO_ALT_COUNT;
	const uint8_t attributes = isInt ? USB_ENDPATT_TRANS_INT : USB_ENDPATT_TRANS_ISO;
	const uint8_t isoInterval = Speed == USB_SPEED_HIGH ? 4U : 1U;

	UsbIntrfDesc_t *pAlt0 = reinterpret_cast<UsbIntrfDesc_t *>(pData);
	*pAlt0 = s_ComboAlt0Desc;
	pAlt0->bInterfaceNumber = interfaceNo;
	ComboAltDesc_t *pAlt = reinterpret_cast<ComboAltDesc_t *>(pAlt0 + 1);
	for (unsigned i = 0U; i < altCount; i++, pAlt++)
	{
		pAlt->Interface = *pAlt0;
		pAlt->Interface.bAlternateSetting = (uint8_t)(i + 1U);
		pAlt->Interface.bNumEndpoints = 2U;
		pAlt->Out.bLength = sizeof(pAlt->Out);
		pAlt->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		pAlt->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(epNo);
		pAlt->Out.bmAttributes = attributes;
		pAlt->Out.wMaxPacketSize = isInt ? INT_MPS : s_IsoMps[i];
		pAlt->Out.bInterval = isInt ? s_IntIntervals[i] : isoInterval;
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
	// USBHS shares its ISO-capable endpoints with CDC, HID and INT. Try
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
	UsbIsoIntrfCfg_t cfg;
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
		nullptr, sizeof(ComboIsoFunctionDesc_t), ComboBuildFunctionDesc);
}

// Link-time override of the library default, see usb.h. USB work goes to the
// queue of the thread serving USB instead of the application event queue.
// The wake of that thread preempts the traffic threads: it has the higher
// priority and is blocked on the semaphore whenever its queue is empty.
bool UsbEvtQue(uint32_t EvtId, void *pCtx, UsbEvtQueHandler_t Handler)
{
	// The controller interrupt and the USB thread both queue here, and
	// CFifoPut takes one producer at a time.
	uint32_t state = DisableInterrupt();
	UsbWork_t *p = (UsbWork_t *)CFifoPut(s_hUsbWork);
	if (p != nullptr)
	{
		p->EvtId = EvtId;
		p->pCtx = pCtx;
		p->Handler = Handler;
	}
	EnableInterrupt(state);
	if (p == nullptr)
	{
		return false;
	}
	// A full binary semaphore already records a wake for this consumer.
	(void)TaktOSSemGive(&s_ServiceWake, false);
	return true;
}

// Copy the head before releasing it: the interrupt can reuse the slot as soon
// as CFifoGet returns.
static bool UsbWorkGet(UsbWork_t *pWork)
{
	const UsbWork_t *p = (const UsbWork_t *)CFifoPeek(s_hUsbWork);
	if (p == nullptr)
	{
		return false;
	}
	*pWork = *p;
	(void)CFifoGet(s_hUsbWork);
	return true;
}

// Every endpoint event of the stack goes through this thread: an OUT packet
// needs its DRDY served before the DMA starts and its completion served
// before the data reaches the RX FIFO, an IN packet needs its completion
// served before the next one is sent. The thread runs above the traffic
// threads and blocks when its queue is empty, so each event is served as soon
// as its interrupt has posted it, as the bare metal main loop does. Waiting
// for a turn in a round-robin instead leaves the endpoints NAKing the host for
// the length of a turn.
static void ServiceThread(void *pArg)
{
	(void)pArg;
	(void)UsbEnable(USB_DEVNO);
	while (true)
	{
		UsbWork_t work;

		// This thread is the only consumer of the USB work queue
		while (UsbWorkGet(&work))
		{
			work.Handler(work.EvtId, work.pCtx);
		}

		// Work refused by a full queue is queued again once the queue is empty
		UsbCheckStatus();

		// An event posted since the queue was found empty has already given
		// the semaphore: the take returns at once. The binary semaphore keeps
		// one wake for any number of posts.
		(void)TaktOSSemTake(&s_ServiceWake, true, TAKTOS_WAIT_FOREVER);
	}
}

static void LoopThread(void *pArg)
{
	(void)pArg;
	uint8_t buffer[CDC_BUFFER_SIZE];
	uint8_t expected = Prbs8(0xff);
	uint32_t errors = 0;
	int pending = 0;
	int offset = 0;
	bool wasOpen = false;
	while (true)
	{
		for (unsigned pass = 0; pass < CDC_PASSES_PER_TURN; pass++)
		{
			const bool open = s_LoopbackCdc.IsPortOpen();
			if (open != wasOpen)
			{
				wasOpen = open;
				pending = 0;
				offset = 0;
				if (open)
				{
					memcpy(buffer, s_Banner, sizeof(s_Banner) - 1);
					pending = sizeof(s_Banner) - 1;
					expected = Prbs8(0xff);
				}
			}
			if (!open)
			{
				break;
			}
			if (pending == 0)
			{
				const int n = s_LoopbackCdc.Rx(0, buffer, sizeof(buffer));
				if (n <= 0)
				{
					break;
				}
				for (int i = 0; i < n; i++)
				{
					if (buffer[i] != expected)
					{
						errors++;
					}
					expected = Prbs8(buffer[i]);
				}
				s_LoopErrors.store(errors, std::memory_order_relaxed);
				pending = n;
				offset = 0;
			}

			// Echo the packet just read in this turn. Waiting for another
			// pass halves the number of packets the turn can service.
			const int n = s_LoopbackCdc.Tx(0, buffer + offset, pending);
			if (n <= 0)
			{
				break;
			}
			offset += n;
			pending -= n;
			if (pending != 0)
			{
				// The TX FIFO filled: the rest goes out once completions
				// have freed room.
				break;
			}
		}
		TaktOSThreadYield();
	}
}

static void PrbsThread(void *pArg)
{
	(void)pArg;
	uint8_t prbs = 0xff;
	uint32_t reported = 0;
	while (true)
	{
		for (unsigned pass = 0; pass < CDC_PASSES_PER_TURN; pass++)
		{
			const bool error = reported != s_LoopErrors.load(std::memory_order_relaxed);
			const uint8_t byte = error ? 0U : prbs;
			if (!s_PrbsCdc.IsPortOpen() || s_PrbsCdc.Tx(0, &byte, 1) <= 0)
			{
				break;
			}
			if (error)
			{
				reported++;
			}
			else
			{
				prbs = Prbs8(prbs);
			}
		}
		TaktOSThreadYield();
	}
}

static void HeartbeatThread(void *pArg)
{
	(void)pArg;
	while (true)
	{
		g_UsbComboTaktOSHeartbeat = g_UsbComboTaktOSHeartbeat + 1;
		(void)TaktOSThreadSleep(TaktOSCurrentThread(), 1000);
	}
}

int main()
{
	// The work queue and its wake exist before USB can queue its first event.
	s_hUsbWork = CFifoInit(s_UsbWorkMem, sizeof(s_UsbWorkMem),
						   sizeof(UsbWork_t), true);
	if (s_hUsbWork == nullptr ||
		TaktOSSemInit(&s_ServiceWake, 0U, 1U) != TAKTOS_OK)
	{
		return -1;
	}
	// Initialization completes before any traffic thread can access a class.
	if (!UsbInit(&s_UsbCfg) ||
		!s_LoopbackCdc.Init(s_LoopbackCfg) ||
		!s_PrbsCdc.Init(s_PrbsCfg) ||
		!s_Hid.Init(s_HidCfg) || !IntInit() || !IsoInit())
	{
		return -1;
	}
	const TaktOSCfg_t cfg = {
		.KernClockHz = SystemCoreClockGet(),
		.TickHz = 1000,
		.TickClockSrc = TAKTOS_TICK_CLOCK_PROCESSOR,
	};
	if (TaktOSInit(&cfg) != TAKTOS_OK ||
		TaktOSThreadCreate(s_ServiceMem, sizeof(s_ServiceMem),
			ServiceThread, nullptr, TAKTOS_PRIORITY_HIGH) == nullptr ||
		TaktOSThreadCreate(s_LoopMem, sizeof(s_LoopMem),
			LoopThread, nullptr, TAKTOS_PRIORITY_NORMAL) == nullptr ||
		TaktOSThreadCreate(s_PrbsMem, sizeof(s_PrbsMem),
			PrbsThread, nullptr, TAKTOS_PRIORITY_NORMAL) == nullptr ||
		TaktOSThreadCreate(s_HeartbeatMem, sizeof(s_HeartbeatMem),
			HeartbeatThread, nullptr, TAKTOS_PRIORITY_NORMAL) == nullptr)
	{
		return -1;
	}
	TaktOSStart();
	while (true) {}
}


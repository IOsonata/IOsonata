/**-------------------------------------------------------------------------
@example	usb_combo_stress.cpp

@brief	USB composite stress test: dual CDC + HID + interrupt + ISO loopback.

Runs all USB data paths on one controller at the same time. Two CDC functions
retain the UsbDualCdcStress loopback/PRBS behavior. HID, raw interrupt and
isochronous functions each provide a bidirectional loopback. Interface and
endpoint numbers are assigned by the common allocator; ISO reserves the
controller-supported ISO endpoint.

Host runner: Python/usb_combo_stress.py

@author	Hoang Nguyen Hoan
@date	Sep. 22, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>

#include "cfifo.h"
#include "prbs.h"
#include "usb/usb.h"
#include "usb/usb_int.h"
#include "usb/usb_iso.h"
#include "usb/usbd_cdc.h"
#include "usb/usbd_epalloc.h"
#include "usb/usbd_hid.h"

#define USB_DEVNO			0

#define CDC_BUFFER_SIZE		USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK)
#define CDC_RXFIFO_PKTCNT	4
#define CDC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, CDC_BUFFER_SIZE)
#define LOOPBACK_TXFIFO_MEMSIZE	CFIFO_MEMSIZE(1024)
#define PRBS_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(2048)

#define COMBO_STR_INTERFACE	4U
#define HID_REPORT_SIZE			64U

#define INT_ALT_COUNT			3U
#define INT_MPS					64U

#define ISO_ALT_COUNT			6U
#define ISO_MAX_MPS			63U
#define ISO_REQ_GET_DIAG		0x5AU

alignas(4) static uint8_t s_LoopbackRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_LoopbackTxFifoMem[LOOPBACK_TXFIFO_MEMSIZE];
alignas(4) static uint8_t s_PrbsRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_PrbsTxFifoMem[PRBS_TXFIFO_MEMSIZE];

UsbdCdc g_LoopbackCdc;
UsbdCdc g_PrbsCdc;

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

/* HID --------------------------------------------------------------------- */

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
static UsbdHid g_Hid;

static int HidEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length)
{
	// Only RX_DATA uses the return value: returning Length frees the slot.
	if (event == DEVINTRF_EVT_RX_DATA)
	{
		// UsbIntIntrfDataEvent already rejects reports larger than the MPS.
		if (g_Hid.Tx(0, pData, Length) != Length)
		{
			memcpy(s_HidPending, pData, Length);
			s_HidPendingLength = Length;
		}
	}
	else if (event == DEVINTRF_EVT_TX_FIFO_EMPTY && s_HidPendingLength != 0U)
	{
		const uint16_t length = s_HidPendingLength;
		s_HidPendingLength = 0U;
		if (g_Hid.Tx(0, s_HidPending, length) != (int)length)
		{
			s_HidPendingLength = length;
		}
	}
	return Length;
}

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

/* Raw interrupt ----------------------------------------------------------- */

static constexpr uint8_t s_IntIntervals[INT_ALT_COUNT] = { 1U, 4U, 16U };

#pragma pack(push, 1)
typedef struct __Combo_Int_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} ComboIntAltDesc_t;

typedef struct __Combo_Int_Function_Descriptor {
	UsbIntrfDesc_t Alt0;
	ComboIntAltDesc_t Alt[INT_ALT_COUNT];
} ComboIntFunctionDesc_t;
#pragma pack(pop)

alignas(4) static uint8_t s_IntRxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_IntTxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
static UsbIntIntrf_t s_Int;
static UsbDevIntrf_t s_IntData;
// Small INT and ISO function state in one block: one address literal serves
// every access.
static struct {
	bool IntConfigured;
	uint8_t IntAlt;
	uint8_t IntInterfaceNo;
	uint8_t IntEpNo;
	bool IsoConfigured;
	uint8_t IsoAlt;
	uint8_t IsoInterfaceNo;
	uint8_t IsoEpNo;
	uint32_t IsoLoopbackDropCnt;
} s_Fn;

static int IntEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length)
{
	if (event == DEVINTRF_EVT_RX_DATA)
	{
		(void)DeviceIntrfTx(&s_IntData.DevIntrf, 0, pData, Length);
	}
	return Length;
}

static bool IntSelectConfig(uint8_t Configuration)
{
	UsbIntIntrfClose(&s_Int);
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

	UsbIntIntrfClose(&s_Int);
	s_Fn.IntAlt = 0U;
	if (Alt == 0U)
	{
		return true;
	}
	if (!UsbIntIntrfOpen(&s_Int, INT_MPS, s_IntIntervals[Alt - 1U]))
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
	UsbIntIntrfReset(&s_Int);
}

static constexpr ComboIntFunctionDesc_t IntFunctionDescTemplate(void)
{
	ComboIntFunctionDesc_t desc = {};
	desc.Alt0.bLength = sizeof(desc.Alt0);
	desc.Alt0.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Alt0.bInterfaceClass = USB_INTRFCLASS_VENDOR;
	desc.Alt0.iInterface = COMBO_STR_INTERFACE;

	for (unsigned i = 0U; i < INT_ALT_COUNT; i++)
	{
		ComboIntAltDesc_t &alt = desc.Alt[i];
		alt.Interface = desc.Alt0;
		alt.Interface.bAlternateSetting = (uint8_t)(i + 1U);
		alt.Interface.bNumEndpoints = 2U;
		alt.Out.bLength = sizeof(alt.Out);
		alt.Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		alt.Out.bmAttributes = USB_ENDPATT_TRANS_INT;
		alt.Out.wMaxPacketSize = INT_MPS;
		alt.Out.bInterval = s_IntIntervals[i];
		alt.In = alt.Out;
	}
	return desc;
}

static constexpr ComboIntFunctionDesc_t s_IntFunctionDesc =
	IntFunctionDescTemplate();

static void IntPatchFunctionDesc(const UsbDeviceClass *, uint8_t *pData,
	UsbSpeed_t)
{
	ComboIntFunctionDesc_t *pDesc =
		reinterpret_cast<ComboIntFunctionDesc_t *>(pData);
	pDesc->Alt0.bInterfaceNumber = s_Fn.IntInterfaceNo;
	for (unsigned i = 0U; i < INT_ALT_COUNT; i++)
	{
		ComboIntAltDesc_t &alt = pDesc->Alt[i];
		alt.Interface.bInterfaceNumber = s_Fn.IntInterfaceNo;
		alt.Out.bEndpointAddress = USB_ENDPADDR_DIROUT(s_Fn.IntEpNo);
		alt.In.bEndpointAddress = USB_ENDPADDR_DIRIN(s_Fn.IntEpNo);
	}
}

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

static IntLoopbackClass s_IntClass;

static bool IntInit(void)
{
	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.BidirectionalCount = 1U;
	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(USB_DEVNO, &req, &s_IntClass, &alloc))
	{
		return false;
	}

	s_Fn.IntInterfaceNo = alloc.FirstInterface;
	s_Fn.IntEpNo = alloc.Bidirectional[0];

	UsbIntIntrfCfg_t cfg = {};
	cfg.DevNo = USB_DEVNO;
	cfg.EpNo = s_Fn.IntEpNo;
	cfg.EvtCB = IntEvent;
	cfg.pRxBuffer = s_IntRxBuffer;
	cfg.pTxBuffer = s_IntTxBuffer;
	if (!UsbIntIntrfInit(&s_Int, &s_IntData, &cfg))
	{
		return false;
	}

	return UsbDescRegister(USB_DEVNO, &s_IntClass,
		&s_IntFunctionDesc, sizeof(s_IntFunctionDesc), IntPatchFunctionDesc);
}

/* ISO --------------------------------------------------------------------- */

static constexpr uint16_t s_IsoMps[ISO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};

#pragma pack(push, 1)
typedef struct __Combo_Iso_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} ComboIsoAltDesc_t;

typedef struct __Combo_Iso_Function_Descriptor {
	UsbIntrfDesc_t Alt0;
	ComboIsoAltDesc_t Alt[ISO_ALT_COUNT];
} ComboIsoFunctionDesc_t;
#pragma pack(pop)

static UsbIsoIntrf_t s_Iso;
static UsbDevIntrf_t s_IsoData;
alignas(4) static uint8_t s_IsoRxFifoMem[USB_ISO_INTRF_FIFO_MEMSIZE(ISO_MAX_MPS)];
alignas(4) static uint8_t s_IsoTxFifoMem[USB_ISO_INTRF_FIFO_MEMSIZE(ISO_MAX_MPS)];

#pragma pack(push, 1)
typedef struct __Combo_Iso_Diag {
	uint32_t RxMissCnt;
	uint32_t TxMissCnt;
	uint32_t LoopbackDropCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
} ComboIsoDiag_t;
#pragma pack(pop)

static ComboIsoDiag_t s_IsoDiag;

// ISO loopback: every received frame goes back out on the next interval.
// Frames are pulled from the RX FIFO with RxData and queued with TxData,
// the same way any DeviceIntrf user moves data.
static int IsoEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
	uint8_t *, int Length)
{
	if (Event != DEVINTRF_EVT_RX_DATA)
	{
		return 0;
	}

	uint8_t frame[ISO_MAX_MPS];
	int total = 0;
	int len;
	while ((len = DeviceIntrfRxData(pDev, frame, sizeof(frame))) > 0)
	{
		if (DeviceIntrfTxData(pDev, frame, len) != len)
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
	s_IsoDiag.RxMissCnt = s_Iso.RxMissCnt;
	s_IsoDiag.TxMissCnt = s_Iso.TxMissCnt;
	s_IsoDiag.LoopbackDropCnt = s_Fn.IsoLoopbackDropCnt;
	s_IsoDiag.RxEmptyCnt = s_Iso.RxEmptyCnt;
	s_IsoDiag.TxEmptyCnt = s_Iso.TxEmptyCnt;
	*ppData = reinterpret_cast<uint8_t *>(&s_IsoDiag);
	*pLength = sizeof(s_IsoDiag);
	return true;
}

static bool IsoSelectConfig(uint8_t Configuration)
{
	UsbIsoIntrfClose(&s_Iso);
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

	UsbIsoIntrfClose(&s_Iso);
	s_Fn.IsoAlt = 0U;
	if (Alt == 0U)
	{
		return true;
	}

	s_Fn.IsoLoopbackDropCnt = 0U;
	s_Iso.RxMissCnt = 0U;
	s_Iso.TxMissCnt = 0U;
	s_Iso.RxEmptyCnt = 0U;
	s_Iso.TxEmptyCnt = 0U;
	const uint8_t interval = UsbCtrlrHighSpeed(USB_DEVNO) ? 4U : 1U;
	if (!UsbIsoIntrfOpen(&s_Iso, s_IsoMps[Alt - 1U], interval))
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
	UsbIsoIntrfReset(&s_Iso);
}

static void IsoProcess(void)
{
	if (!s_Fn.IsoConfigured || s_Fn.IsoAlt == 0U)
	{
		return;
	}
	const bool suspended = UsbSuspended(USB_DEVNO);
	if (suspended && !s_Iso.Suspended)
	{
		UsbIsoIntrfSuspend(&s_Iso);
	}
	else if (!suspended && s_Iso.Suspended)
	{
		(void)UsbIsoIntrfResume(&s_Iso);
	}
}

static constexpr UsbIntrfDesc_t s_IsoAlt0Desc = {
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

static void IsoBuildFunctionDesc(const UsbDeviceClass *, uint8_t *pData,
	UsbSpeed_t Speed)
{
	ComboIsoFunctionDesc_t *pDesc =
		reinterpret_cast<ComboIsoFunctionDesc_t *>(pData);
	pDesc->Alt0 = s_IsoAlt0Desc;
	pDesc->Alt0.bInterfaceNumber = s_Fn.IsoInterfaceNo;

	const uint8_t interval = Speed == USB_SPEED_HIGH ? 4U : 1U;
	for (unsigned i = 0U; i < ISO_ALT_COUNT; i++)
	{
		ComboIsoAltDesc_t &alt = pDesc->Alt[i];
		// Build from the small components instead of copying a complete
		// alternate template and overwriting its variable fields.
		alt.Interface = pDesc->Alt0;
		alt.Interface.bAlternateSetting = (uint8_t)(i + 1U);
		alt.Interface.bNumEndpoints = 2U;
		alt.Out.bLength = sizeof(alt.Out);
		alt.Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		alt.Out.bEndpointAddress = USB_ENDPADDR_DIROUT(s_Fn.IsoEpNo);
		alt.Out.bmAttributes = USB_ENDPATT_TRANS_ISO;
		alt.Out.wMaxPacketSize = s_IsoMps[i];
		alt.Out.bInterval = interval;
		alt.In = alt.Out;
		alt.In.bEndpointAddress = USB_ENDPADDR_DIRIN(s_Fn.IsoEpNo);
	}
}

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

static IsoLoopbackClass s_IsoClass;

static bool IsoInit(void)
{
	if (!USB_ISO_SUPPORTED(USB_DEVNO))
	{
		return false;
	}

	const uint16_t isoMask = (uint16_t)(
		USB_ISO_EPIN_MASK(USB_DEVNO) & USB_ISO_EPOUT_MASK(USB_DEVNO));
	// Lowest controller ISO endpoint; the mask excludes endpoint 0.
	const uint8_t epNo = isoMask != 0U ? (uint8_t)__builtin_ctz(isoMask) : 0U;
	if (epNo == 0U)
	{
		return false;
	}

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.FixedInMask = (uint16_t)(1U << epNo);
	req.FixedOutMask = (uint16_t)(1U << epNo);
	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(USB_DEVNO, &req, &s_IsoClass, &alloc))
	{
		return false;
	}

	s_Fn.IsoInterfaceNo = alloc.FirstInterface;
	s_Fn.IsoEpNo = epNo;

	UsbIsoIntrfCfg_t cfg = {};
	cfg.DevNo = USB_DEVNO;
	cfg.EpNo = s_Fn.IsoEpNo;
	cfg.BufferSize = ISO_MAX_MPS;
	cfg.pRxFifoMem = s_IsoRxFifoMem;
	cfg.pTxFifoMem = s_IsoTxFifoMem;
	cfg.EvtCB = IsoEvent;
	if (!UsbIsoIntrfInit(&s_Iso, &s_IsoData, &cfg))
	{
		return false;
	}

	return UsbDescRegister(USB_DEVNO, &s_IsoClass,
		nullptr, sizeof(ComboIsoFunctionDesc_t), IsoBuildFunctionDesc);
}

/* Device ------------------------------------------------------------------ */

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

int main()
{
	// Same loopback and PRBS logic as exemples/usb/tinyusb_combo_stress, so
	// both builds do the same work per byte.
	uint8_t loopbackBuffer[CDC_BUFFER_SIZE];
	uint8_t loopbackExpected = Prbs8(0xff);
	uint8_t prbs = 0xff;
	uint32_t loopbackRxErrorNotify = 0;
	int loopbackPending = 0;
	int loopbackOffset = 0;
	bool loopbackConnected = false;

	if (!UsbInit(&s_UsbCfg) ||
		!g_LoopbackCdc.Init(s_LoopbackCfg) ||
		!g_PrbsCdc.Init(s_PrbsCfg) ||
		!g_Hid.Init(s_HidCfg) ||
		!IntInit() ||
		!IsoInit())
	{
		return -1;
	}

	(void)UsbEnable(USB_DEVNO);
	while (1)
	{
		UsbProcess(USB_DEVNO);

		// A port open restarts the host generator: restart the checker and
		// drop any echo still pending from the previous session.
		const bool connected = g_LoopbackCdc.IsPortOpen();
		if (connected != loopbackConnected)
		{
			loopbackConnected = connected;
			loopbackPending = 0;
			loopbackOffset = 0;

			if (connected)
			{
				static const char msg[] = "\r\nIOsonata USB Combo Stress\r\n";
				loopbackExpected = Prbs8(0xff);
				g_LoopbackCdc.Tx(0, reinterpret_cast<const uint8_t *>(msg),
					(int)sizeof(msg) - 1);
			}
		}

		if (loopbackConnected)
		{
			if (loopbackPending > 0)
			{
				const int length = g_LoopbackCdc.Tx(
					0, &loopbackBuffer[loopbackOffset], loopbackPending);
				if (length > 0)
				{
					loopbackOffset += length;
					loopbackPending -= length;
				}
			}
			else
			{
				const int length = g_LoopbackCdc.Rx(
					0, loopbackBuffer, sizeof(loopbackBuffer));
				if (length > 0)
				{
					for (int i = 0; i < length; i++)
					{
						if (loopbackBuffer[i] != loopbackExpected)
						{
							loopbackRxErrorNotify++;
						}
						loopbackExpected = Prbs8(loopbackBuffer[i]);
					}
					loopbackPending = length;
					loopbackOffset = 0;
				}
			}
		}

		const uint8_t prbsByte = loopbackRxErrorNotify > 0U ? 0U : prbs;
		if (g_PrbsCdc.IsPortOpen() && g_PrbsCdc.Tx(0, &prbsByte, 1) > 0)
		{
			if (loopbackRxErrorNotify > 0U)
			{
				loopbackRxErrorNotify--;
			}
			else
			{
				prbs = Prbs8(prbs);
			}
		}
	}

	return 0;
}



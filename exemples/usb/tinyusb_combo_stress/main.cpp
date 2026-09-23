/**-------------------------------------------------------------------------
@example	tinyusb_combo_stress/main.cpp

@brief	TinyUSB composite stress benchmark matching usb_combo_stress.cpp.

Runs dual CDC, HID, raw interrupt and bidirectional isochronous loopback on the
nRF52840 using TinyUSB. Python/usb_combo_stress.py drives the same workload;
uses the same VID/PID/product identity as usb_combo_stress.cpp.

The raw interrupt function uses TinyUSB's Vendor class. The ISO interface uses
a small application class driver because the benchmark intentionally reuses
the nRF52 fixed EP8 across six alternate settings with different MPS values.

@author	Nguyen Hoan Hoang
@date	Sep. 23, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "prbs.h"
#include "tusb.h"
#include "device/usbd_pvt.h"

#define TINYUSB_COMBO_VID			0x1209U
#define TINYUSB_COMBO_PID			0x0008U
#define CDC_NOTIFY_MPS				8U
#define DATA_MPS					64U
#define HID_REPORT_SIZE				64U
#define INT_MPS						64U
#define ISO_MAX_MPS				63U
#define ISO_ALT_COUNT				6U
#define ISO_QUEUE_DEPTH				8U
#define ISO_REQ_GET_DIAG			0x5AU

#define CDC0_NOTIFY_EP				0x81U
#define CDC0_OUT_EP				0x02U
#define CDC0_IN_EP					0x82U
#define CDC1_NOTIFY_EP				0x83U
#define CDC1_OUT_EP				0x04U
#define CDC1_IN_EP					0x84U
#define HID_OUT_EP					0x05U
#define HID_IN_EP					0x85U
#define INT_OUT_EP					0x06U
#define INT_IN_EP					0x86U
#define ISO_OUT_EP					0x08U
#define ISO_IN_EP					0x88U

enum {
	ITF_CDC0 = 0,
	ITF_CDC0_DATA,
	ITF_CDC1,
	ITF_CDC1_DATA,
	ITF_HID,
	ITF_INT,
	ITF_ISO,
	ITF_COUNT
};

enum {
	STR_LANGID = 0,
	STR_MANUFACTURER,
	STR_PRODUCT,
	STR_SERIAL,
	STR_CDC0,
	STR_CDC1,
	STR_HID,
	STR_INT,
	STR_ISO
};

enum {
	VENDOR_INT = 0
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

#define INT_FUNCTION_DESC_LEN	(9U + 3U * (9U + 7U + 7U))
#define ISO_FUNCTION_DESC_LEN	(9U + ISO_ALT_COUNT * (9U + 7U + 7U))
#define CONFIG_TOTAL_LEN 	(TUD_CONFIG_DESC_LEN + 2U * TUD_CDC_DESC_LEN + 	 TUD_HID_INOUT_DESC_LEN + INT_FUNCTION_DESC_LEN + ISO_FUNCTION_DESC_LEN)

#define VENDOR_ALT0(_itf, _str) 	9, TUSB_DESC_INTERFACE, (_itf), 0, 0, TUSB_CLASS_VENDOR_SPECIFIC, 0, 0, (_str)

#define INT_ALT_DESC(_alt, _interval) 	9, TUSB_DESC_INTERFACE, ITF_INT, (_alt), 2, TUSB_CLASS_VENDOR_SPECIFIC, 0, 0, STR_INT, 	7, TUSB_DESC_ENDPOINT, INT_OUT_EP, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(INT_MPS), (_interval), 	7, TUSB_DESC_ENDPOINT, INT_IN_EP,  TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(INT_MPS), (_interval)

#define ISO_ALT_DESC(_alt, _mps) 	9, TUSB_DESC_INTERFACE, ITF_ISO, (_alt), 2, TUSB_CLASS_VENDOR_SPECIFIC, 0, 0, STR_ISO, 	7, TUSB_DESC_ENDPOINT, ISO_OUT_EP, TUSB_XFER_ISOCHRONOUS, U16_TO_U8S_LE(_mps), 1, 	7, TUSB_DESC_ENDPOINT, ISO_IN_EP,  TUSB_XFER_ISOCHRONOUS, U16_TO_U8S_LE(_mps), 1

static const tusb_desc_device_t s_DeviceDesc = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = TINYUSB_COMBO_VID,
	.idProduct = TINYUSB_COMBO_PID,
	.bcdDevice = 0x0100,
	.iManufacturer = STR_MANUFACTURER,
	.iProduct = STR_PRODUCT,
	.iSerialNumber = STR_SERIAL,
	.bNumConfigurations = 1
};

static const uint8_t s_ConfigDesc[] = {
	TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN,
		TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

	TUD_CDC_DESCRIPTOR(ITF_CDC0, STR_CDC0, CDC0_NOTIFY_EP, CDC_NOTIFY_MPS,
		CDC0_OUT_EP, CDC0_IN_EP, DATA_MPS),
	TUD_CDC_DESCRIPTOR(ITF_CDC1, STR_CDC1, CDC1_NOTIFY_EP, CDC_NOTIFY_MPS,
		CDC1_OUT_EP, CDC1_IN_EP, DATA_MPS),

	TUD_HID_INOUT_DESCRIPTOR(ITF_HID, STR_HID, HID_ITF_PROTOCOL_NONE,
		sizeof(s_HidReportDesc), HID_OUT_EP, HID_IN_EP, HID_REPORT_SIZE, 1),

	VENDOR_ALT0(ITF_INT, STR_INT),
	INT_ALT_DESC(1, 1),
	INT_ALT_DESC(2, 4),
	INT_ALT_DESC(3, 16),

	VENDOR_ALT0(ITF_ISO, STR_ISO),
	ISO_ALT_DESC(1, 9),
	ISO_ALT_DESC(2, 17),
	ISO_ALT_DESC(3, 25),
	ISO_ALT_DESC(4, 33),
	ISO_ALT_DESC(5, 49),
	ISO_ALT_DESC(6, 63),
};

static_assert(sizeof(s_ConfigDesc) == CONFIG_TOTAL_LEN,
	"TinyUSB combo configuration descriptor length");

static const char * const s_Strings[] = {
	nullptr,
	"I-SYST",
	"IOsonata USB Combo Stress",
	nullptr,
	"TinyUSB Loopback CDC",
	"TinyUSB PRBS CDC",
	"TinyUSB HID Loopback",
	"TinyUSB Interrupt Loopback",
	"TinyUSB ISO Loopback"
};

static uint16_t s_StringDesc[33];

extern "C" uint8_t const *tud_descriptor_device_cb(void)
{
	return reinterpret_cast<const uint8_t *>(&s_DeviceDesc);
}

extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t Index)
{
	(void)Index;
	return s_ConfigDesc;
}

extern "C" uint8_t const *tud_hid_descriptor_report_cb(uint8_t Instance)
{
	(void)Instance;
	return s_HidReportDesc;
}

static size_t SerialString(uint16_t *pString, size_t MaxLen)
{
	static const char Hex[] = "0123456789ABCDEF";
	const uint32_t id[2] = {
		NRF_FICR->DEVICEID[0],
		NRF_FICR->DEVICEID[1]
	};
	size_t count = 0;

	for (unsigned word = 0; word < 2 && count < MaxLen; word++)
	{
		for (int shift = 28; shift >= 0 && count < MaxLen; shift -= 4)
		{
			pString[count++] =
				static_cast<uint16_t>(Hex[(id[word] >> shift) & 0x0fU]);
		}
	}
	return count;
}

extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t Index,
	uint16_t LangId)
{
	(void)LangId;
	size_t count = 0;

	if (Index == STR_LANGID)
	{
		s_StringDesc[1] = 0x0409;
		count = 1;
	}
	else if (Index == STR_SERIAL)
	{
		count = SerialString(&s_StringDesc[1], 32);
	}
	else
	{
		if (Index >= sizeof(s_Strings) / sizeof(s_Strings[0]) ||
			s_Strings[Index] == nullptr)
		{
			return nullptr;
		}

		const char *p = s_Strings[Index];
		count = strlen(p);
		if (count > 32)
			count = 32;

		for (size_t i = 0; i < count; i++)
			s_StringDesc[1 + i] = static_cast<uint16_t>(p[i]);
	}

	s_StringDesc[0] =
		static_cast<uint16_t>((TUSB_DESC_STRING << 8) | (2U * count + 2U));
	return s_StringDesc;
}

/* HID --------------------------------------------------------------------- */

static uint8_t s_HidPending[HID_REPORT_SIZE];
static uint16_t s_HidPendingLen;

static void HidSendOrPend(const uint8_t *pData, uint16_t Length)
{
	if (Length > HID_REPORT_SIZE)
		return;

	if (tud_hid_ready() && tud_hid_report(0, pData, Length))
		return;

	memcpy(s_HidPending, pData, Length);
	s_HidPendingLen = Length;
}

extern "C" uint16_t tud_hid_get_report_cb(uint8_t Instance, uint8_t ReportId,
	hid_report_type_t ReportType, uint8_t *pBuffer, uint16_t ReqLen)
{
	(void)Instance;
	(void)ReportId;
	(void)ReportType;
	(void)pBuffer;
	(void)ReqLen;
	return 0;
}

extern "C" void tud_hid_set_report_cb(uint8_t Instance, uint8_t ReportId,
	hid_report_type_t ReportType, uint8_t const *pBuffer, uint16_t BufSize)
{
	(void)Instance;
	(void)ReportId;
	(void)ReportType;
	HidSendOrPend(pBuffer, BufSize);
}

extern "C" void tud_hid_report_complete_cb(uint8_t Instance,
	uint8_t const *pReport, uint16_t Length)
{
	(void)Instance;
	(void)pReport;
	(void)Length;

	if (s_HidPendingLen != 0U)
	{
		const uint16_t len = s_HidPendingLen;
		s_HidPendingLen = 0U;
		if (!tud_hid_report(0, s_HidPending, len))
			s_HidPendingLen = len;
	}
}

/* Raw interrupt through TinyUSB Vendor ------------------------------------ */

extern "C" void tud_vendor_int_rx_cb(uint8_t Index,
	const uint8_t *pBuffer, uint32_t Length)
{
	if (Index != VENDOR_INT)
		return;

	if (Length <= INT_MPS)
		(void)tud_vendor_n_int_write(Index, pBuffer, Length);

	(void)tud_vendor_n_int_read_xfer(Index);
}

extern "C" void tud_vendor_int_tx_cb(uint8_t Index, uint32_t Length)
{
	(void)Index;
	(void)Length;
}

/* Custom ISO interface ---------------------------------------------------- */

#pragma pack(push, 1)
typedef struct __TinyUsbIsoDiag {
	uint32_t RxMissCnt;
	uint32_t TxMissCnt;
	uint32_t LoopbackDropCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
} TinyUsbIsoDiag_t;
#pragma pack(pop)

typedef struct __IsoFrame {
	uint8_t Data[ISO_MAX_MPS];
	uint8_t Len;
} IsoFrame_t;

typedef struct __IsoState {
	uint8_t RhPort;
	uint8_t Alt;
	uint8_t Mps;
	bool InBusy;
	const uint8_t *pDesc;
	uint16_t DescLen;
	const tusb_desc_endpoint_t *pOutDesc;
	const tusb_desc_endpoint_t *pInDesc;
	uint8_t RxBuffer[ISO_MAX_MPS];
	IsoFrame_t Queue[ISO_QUEUE_DEPTH];
	uint8_t Put;
	uint8_t Get;
	uint8_t Count;
	TinyUsbIsoDiag_t Diag;
} IsoState_t;

static IsoState_t s_Iso;

static void IsoQueueReset(void)
{
	s_Iso.Put = 0U;
	s_Iso.Get = 0U;
	s_Iso.Count = 0U;
	s_Iso.InBusy = false;
	memset(&s_Iso.Diag, 0, sizeof(s_Iso.Diag));
}

static bool IsoFindAlt(uint8_t Alt,
	const tusb_desc_endpoint_t **ppOut,
	const tusb_desc_endpoint_t **ppIn)
{
	*ppOut = nullptr;
	*ppIn = nullptr;
	if (Alt == 0U)
		return true;

	const uint8_t *p = s_Iso.pDesc;
	const uint8_t *end = p + s_Iso.DescLen;
	bool selected = false;

	while (tu_desc_in_bounds(p, end))
	{
		if (tu_desc_type(p) == TUSB_DESC_INTERFACE)
		{
			const tusb_desc_interface_t *itf =
				reinterpret_cast<const tusb_desc_interface_t *>(p);
			selected = itf->bInterfaceNumber == ITF_ISO &&
				itf->bAlternateSetting == Alt;
		}
		else if (selected && tu_desc_type(p) == TUSB_DESC_ENDPOINT)
		{
			const tusb_desc_endpoint_t *ep =
				reinterpret_cast<const tusb_desc_endpoint_t *>(p);
			if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN)
				*ppIn = ep;
			else
				*ppOut = ep;
		}
		p = tu_desc_next(p);
	}
	return *ppOut != nullptr && *ppIn != nullptr;
}

static void IsoKickIn(bool InIsr)
{
	if (s_Iso.InBusy || s_Iso.Count == 0U || s_Iso.Alt == 0U)
		return;

	IsoFrame_t &frame = s_Iso.Queue[s_Iso.Get];
	if (usbd_edpt_xfer(s_Iso.RhPort, ISO_IN_EP,
			frame.Len != 0U ? frame.Data : nullptr, frame.Len, InIsr))
	{
		s_Iso.InBusy = true;
	}
	else
	{
		s_Iso.Diag.TxMissCnt++;
	}
}

static bool IsoArmOut(bool InIsr)
{
	if (s_Iso.Alt == 0U || s_Iso.Mps == 0U)
		return false;

	if (!usbd_edpt_xfer(s_Iso.RhPort, ISO_OUT_EP,
			s_Iso.RxBuffer, s_Iso.Mps, InIsr))
	{
		s_Iso.Diag.RxMissCnt++;
		return false;
	}
	return true;
}

static bool IsoSetAlt(uint8_t Alt)
{
	if (Alt > ISO_ALT_COUNT)
		return false;

	// Reactivating the previous descriptors aborts any transfer still armed on
	// nRF52 EP8 and clears TinyUSB's endpoint busy state before re-selection.
	if (s_Iso.pOutDesc != nullptr)
		(void)usbd_edpt_iso_activate(s_Iso.RhPort, s_Iso.pOutDesc);
	if (s_Iso.pInDesc != nullptr)
		(void)usbd_edpt_iso_activate(s_Iso.RhPort, s_Iso.pInDesc);

	IsoQueueReset();
	s_Iso.pOutDesc = nullptr;
	s_Iso.pInDesc = nullptr;
	s_Iso.Alt = 0U;
	s_Iso.Mps = 0U;

	if (Alt == 0U)
		return true;

	const tusb_desc_endpoint_t *out = nullptr;
	const tusb_desc_endpoint_t *in = nullptr;
	if (!IsoFindAlt(Alt, &out, &in))
		return false;

	if (!usbd_edpt_iso_activate(s_Iso.RhPort, out) ||
		!usbd_edpt_iso_activate(s_Iso.RhPort, in))
	{
		return false;
	}

	s_Iso.pOutDesc = out;
	s_Iso.pInDesc = in;
	s_Iso.Alt = Alt;
	s_Iso.Mps = static_cast<uint8_t>(tu_edpt_packet_size(out));
	return IsoArmOut(false);
}

static void IsoDriverInit(void)
{
	memset(&s_Iso, 0, sizeof(s_Iso));
}

static bool IsoDriverDeinit(void)
{
	return true;
}

static void IsoDriverReset(uint8_t RhPort)
{
	memset(&s_Iso, 0, sizeof(s_Iso));
	s_Iso.RhPort = RhPort;
}

static uint16_t IsoDriverOpen(uint8_t RhPort,
	const tusb_desc_interface_t *pItf, uint16_t MaxLen)
{
	if (pItf->bInterfaceNumber != ITF_ISO ||
		pItf->bAlternateSetting != 0U ||
		pItf->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC)
	{
		return 0U;
	}

	const uint8_t *start = reinterpret_cast<const uint8_t *>(pItf);
	const uint8_t *p = tu_desc_next(start);
	const uint8_t *end = start + MaxLen;

	while (tu_desc_in_bounds(p, end))
	{
		if (tu_desc_type(p) == TUSB_DESC_INTERFACE)
		{
			const tusb_desc_interface_t *next =
				reinterpret_cast<const tusb_desc_interface_t *>(p);
			if (next->bInterfaceNumber != ITF_ISO)
				break;
		}
		else if (tu_desc_type(p) == TUSB_DESC_INTERFACE_ASSOCIATION)
		{
			break;
		}
		p = tu_desc_next(p);
	}

	IsoDriverReset(RhPort);
	s_Iso.pDesc = start;
	s_Iso.DescLen = static_cast<uint16_t>(p - start);

	if (!usbd_edpt_iso_alloc(RhPort, ISO_OUT_EP, ISO_MAX_MPS) ||
		!usbd_edpt_iso_alloc(RhPort, ISO_IN_EP, ISO_MAX_MPS))
	{
		return 0U;
	}

	return s_Iso.DescLen;
}

static bool IsoDriverControl(uint8_t RhPort, uint8_t Stage,
	const tusb_control_request_t *pRequest)
{
	if (tu_u16_low(pRequest->wIndex) != ITF_ISO)
		return false;

	if (Stage != CONTROL_STAGE_SETUP)
		return true;

	if (pRequest->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
		pRequest->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE)
	{
		if (pRequest->bRequest == TUSB_REQ_GET_INTERFACE)
		{
			return tud_control_xfer(RhPort, pRequest, &s_Iso.Alt, 1U);
		}
		if (pRequest->bRequest == TUSB_REQ_SET_INTERFACE)
		{
			if (!IsoSetAlt(tu_u16_low(pRequest->wValue)))
				return false;
			return tud_control_status(RhPort, pRequest);
		}
	}

	if (pRequest->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
		pRequest->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE &&
		pRequest->bmRequestType_bit.direction == TUSB_DIR_IN &&
		pRequest->bRequest == ISO_REQ_GET_DIAG &&
		pRequest->wValue == 0U &&
		pRequest->wLength == sizeof(s_Iso.Diag))
	{
		return tud_control_xfer(RhPort, pRequest,
			&s_Iso.Diag, sizeof(s_Iso.Diag));
	}

	return false;
}

static bool IsoDriverTransfer(uint8_t RhPort, uint8_t EpAddr,
	xfer_result_t Result, uint32_t Length, bool InIsr)
{
	(void)RhPort;

	if (EpAddr == ISO_OUT_EP)
	{
		if (Result == XFER_RESULT_SUCCESS)
		{
			if (Length <= ISO_MAX_MPS && s_Iso.Count < ISO_QUEUE_DEPTH)
			{
				IsoFrame_t &frame = s_Iso.Queue[s_Iso.Put];
				frame.Len = static_cast<uint8_t>(Length);
				if (Length != 0U)
					memcpy(frame.Data, s_Iso.RxBuffer, Length);
				s_Iso.Put = static_cast<uint8_t>(
					(s_Iso.Put + 1U) % ISO_QUEUE_DEPTH);
				s_Iso.Count++;
			}
			else
			{
				s_Iso.Diag.LoopbackDropCnt++;
			}
		}
		else
		{
			s_Iso.Diag.RxMissCnt++;
		}

		IsoKickIn(InIsr);
		(void)IsoArmOut(InIsr);
		return true;
	}

	if (EpAddr == ISO_IN_EP)
	{
		if (Result != XFER_RESULT_SUCCESS)
			s_Iso.Diag.TxMissCnt++;

		if (s_Iso.InBusy && s_Iso.Count != 0U)
		{
			s_Iso.Get = static_cast<uint8_t>(
				(s_Iso.Get + 1U) % ISO_QUEUE_DEPTH);
			s_Iso.Count--;
		}
		s_Iso.InBusy = false;
		IsoKickIn(InIsr);
		return true;
	}

	return false;
}

static bool IsoDriverXfer(uint8_t RhPort, uint8_t EpAddr,
	xfer_result_t Result, uint32_t Length)
{
	return IsoDriverTransfer(RhPort, EpAddr, Result, Length, false);
}

static bool IsoDriverXferIsr(uint8_t RhPort, uint8_t EpAddr,
	xfer_result_t Result, uint32_t Length)
{
	return IsoDriverTransfer(RhPort, EpAddr, Result, Length, true);
}

static const usbd_class_driver_t s_IsoDriver = {
	.name = "COMBO-ISO",
	.init = IsoDriverInit,
	.deinit = IsoDriverDeinit,
	.reset = IsoDriverReset,
	.open = IsoDriverOpen,
	.control_xfer_cb = IsoDriverControl,
	.xfer_cb = IsoDriverXfer,
	.xfer_isr = IsoDriverXferIsr,
	.sof = nullptr,
};

extern "C" usbd_class_driver_t const *usbd_app_driver_get_cb(
	uint8_t *pDriverCount)
{
	*pDriverCount = 1U;
	return &s_IsoDriver;
}

/* TinyUSB nRF52 power/IRQ glue ------------------------------------------- */

extern "C" void tusb_hal_nrf_power_event(uint32_t Event);

static bool s_Vbus;
static bool s_Ready;

static void PowerProcess(void)
{
	const uint32_t status = NRF_POWER->USBREGSTATUS;
	const bool vbus =
		(status & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0U;
	const bool ready =
		(status & POWER_USBREGSTATUS_OUTPUTRDY_Msk) != 0U;

	if (!vbus)
	{
		if (s_Vbus)
			tusb_hal_nrf_power_event(1U);
		s_Vbus = false;
		s_Ready = false;
		return;
	}

	if (!s_Vbus)
	{
		s_Vbus = true;
		tusb_hal_nrf_power_event(0U);
	}

	if (ready && !s_Ready)
	{
		s_Ready = true;
		tusb_hal_nrf_power_event(2U);
	}
	else if (!ready)
	{
		s_Ready = false;
	}
}

extern "C" void USBD_IRQHandler(void)
{
	tusb_int_handler(0, true);
}

static bool UsbInit(void)
{
	NVIC_SetPriority(USBD_IRQn, 6U);

	tusb_rhport_init_t devInit = {};
	devInit.role = TUSB_ROLE_DEVICE;
	devInit.speed = TUSB_SPEED_FULL;

	if (!tusb_init(0, &devInit))
		return false;

	PowerProcess();
	return true;
}

/* Main workload ----------------------------------------------------------- */

int main()
{
	static constexpr uint8_t LoopbackCdc = 0U;
	static constexpr uint8_t PrbsCdc = 1U;

	uint8_t loopbackBuffer[DATA_MPS];
	uint8_t loopbackExpected = Prbs8(0xff);
	uint8_t prbs = 0xff;
	uint32_t loopbackRxErrorNotify = 0U;
	unsigned loopbackPending = 0U;
	unsigned loopbackOffset = 0U;
	bool loopbackConnected = false;

	if (!UsbInit())
		return -1;

	while (1)
	{
		PowerProcess();
		tud_task_ext(0, false);

		const bool connected = tud_cdc_n_connected(LoopbackCdc);
		if (connected != loopbackConnected)
		{
			loopbackConnected = connected;
			loopbackPending = 0U;
			loopbackOffset = 0U;

			if (connected)
			{
				static const char Msg[] =
					"\r\nIOsonata USB Combo Stress\r\n";
				loopbackExpected = Prbs8(0xff);
				tud_cdc_n_write(LoopbackCdc, Msg, sizeof(Msg) - 1U);
				tud_cdc_n_write_flush(LoopbackCdc);
			}
		}

		if (loopbackConnected)
		{
			if (loopbackPending != 0U)
			{
				const uint32_t room =
					tud_cdc_n_write_available(LoopbackCdc);
				if (room != 0U)
				{
					const unsigned count =
						loopbackPending < room ? loopbackPending : room;
					const uint32_t written = tud_cdc_n_write(
						LoopbackCdc, &loopbackBuffer[loopbackOffset], count);
					if (written != 0U)
					{
						loopbackOffset += written;
						loopbackPending -= written;
						tud_cdc_n_write_flush(LoopbackCdc);
					}
				}
			}
			else
			{
				const uint32_t available =
					tud_cdc_n_available(LoopbackCdc);
				if (available != 0U)
				{
					const uint32_t count =
						available < DATA_MPS ? available : DATA_MPS;
					const uint32_t length =
						tud_cdc_n_read(LoopbackCdc, loopbackBuffer, count);
					for (uint32_t i = 0; i < length; i++)
					{
						if (loopbackBuffer[i] != loopbackExpected)
							loopbackRxErrorNotify++;
						loopbackExpected = Prbs8(loopbackBuffer[i]);
					}
					loopbackPending = length;
					loopbackOffset = 0U;
				}
			}
		}

		const uint8_t prbsByte =
			loopbackRxErrorNotify != 0U ? 0U : prbs;
		if (tud_cdc_n_connected(PrbsCdc) &&
			tud_cdc_n_write_available(PrbsCdc) != 0U &&
			tud_cdc_n_write(PrbsCdc, &prbsByte, 1U) == 1U)
		{
			if (loopbackRxErrorNotify != 0U)
				loopbackRxErrorNotify--;
			else
				prbs = Prbs8(prbs);
		}

		// The Vendor interrupt OUT endpoint is intentionally manual. Polling is
		// also what arms it immediately after SET_INTERFACE selects alt 1.
		if (tud_vendor_n_alt(VENDOR_INT) != 0U)
			(void)tud_vendor_n_int_read_xfer(VENDOR_INT);

	}

	return 0;
}

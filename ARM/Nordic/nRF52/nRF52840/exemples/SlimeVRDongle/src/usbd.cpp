/*
 * usbd.cpp
 *
 * USB HID device for the SlimeVR receiver dongle, implemented directly on
 * nrfx_usbd. No nRF5 SDK app_usbd / nrf_drv dependency.
 *
 * Single HID interface, one 64 byte interrupt IN endpoint and one 64 byte
 * interrupt OUT endpoint on EP3. VID, PID and the HID report descriptor match
 * the SlimeVR reference receiver so the host server enumerates and binds.
 *
 * HID IN report layout matches the reference receiver: each 64 byte report
 * holds up to four 16 byte tracker records. hid_write_packet_n() copies one
 * 16 byte ESB payload, stores rssi in byte 15 for packet types other than 1
 * and 4, queues the record, and the records are packed four per IN report.
 *
 *  Created on: Nov 22, 2024
 *      Author: hoan
 */

#include <string.h>

#include "nrf.h"
#include "nrfx_usbd.h"
#include "nrfx_power.h"

#include "SlimeVRDongle.h"

// Identity. Must match the SlimeVR reference receiver for host binding.
#define USBD_VID						0x1209
#define USBD_PID						0x7690
#define USBD_BCD_DEVICE					0x0100

// HID endpoints on EP3.
#define HID_EPIN						NRFX_USBD_EPIN3
#define HID_EPOUT						NRFX_USBD_EPOUT3
#define HID_EP_ADDR_IN					0x83
#define HID_EP_ADDR_OUT					0x03
#define HID_EP_SIZE						64
#define HID_RECORD_SIZE					16					// one tracker record
#define HID_RECORDS_PER_REPORT			(HID_EP_SIZE / HID_RECORD_SIZE)	// 4
#define HID_RECORD_FIFO_LEN				64					// power of two

// String descriptor indices.
#define USBD_STR_LANGID					0
#define USBD_STR_MANUFACTURER			1
#define USBD_STR_PRODUCT				2
#define USBD_STR_SERIAL					3

// Descriptor types.
#define USB_DESC_DEVICE					0x01
#define USB_DESC_CONFIGURATION			0x02
#define USB_DESC_STRING					0x03
#define USB_DESC_INTERFACE				0x04
#define USB_DESC_ENDPOINT				0x05
#define USB_DESC_HID					0x21
#define USB_DESC_HID_REPORT				0x22

// Standard requests (bRequest).
#define USB_REQ_GET_STATUS				0x00
#define USB_REQ_CLEAR_FEATURE			0x01
#define USB_REQ_SET_FEATURE				0x03
#define USB_REQ_SET_ADDRESS				0x05
#define USB_REQ_GET_DESCRIPTOR			0x06
#define USB_REQ_SET_DESCRIPTOR			0x07
#define USB_REQ_GET_CONFIGURATION		0x08
#define USB_REQ_SET_CONFIGURATION		0x09
#define USB_REQ_GET_INTERFACE			0x0A
#define USB_REQ_SET_INTERFACE			0x0B

// HID class requests (bRequest).
#define HID_REQ_GET_REPORT				0x01
#define HID_REQ_GET_IDLE				0x02
#define HID_REQ_GET_PROTOCOL			0x03
#define HID_REQ_SET_REPORT				0x09
#define HID_REQ_SET_IDLE				0x0A
#define HID_REQ_SET_PROTOCOL			0x0B

// bmRequestType fields.
#define USB_REQ_DIR_MASK				0x80
#define USB_REQ_DIR_IN					0x80
#define USB_REQ_TYPE_MASK				0x60
#define USB_REQ_TYPE_STANDARD			0x00
#define USB_REQ_TYPE_CLASS				0x20
#define USB_REQ_RECIP_MASK				0x1F
#define USB_REQ_RECIP_DEVICE			0x00
#define USB_REQ_RECIP_INTERFACE			0x01
#define USB_REQ_RECIP_ENDPOINT			0x02

// Device descriptor.
static const uint8_t s_DeviceDesc[] =
{
	18,								// bLength
	USB_DESC_DEVICE,				// bDescriptorType
	0x00, 0x02,						// bcdUSB 2.00
	0x00,							// bDeviceClass (per interface)
	0x00,							// bDeviceSubClass
	0x00,							// bDeviceProtocol
	HID_EP_SIZE,					// bMaxPacketSize0
	(uint8_t)(USBD_VID & 0xFF), (uint8_t)(USBD_VID >> 8),
	(uint8_t)(USBD_PID & 0xFF), (uint8_t)(USBD_PID >> 8),
	(uint8_t)(USBD_BCD_DEVICE & 0xFF), (uint8_t)(USBD_BCD_DEVICE >> 8),
	USBD_STR_MANUFACTURER,			// iManufacturer
	USBD_STR_PRODUCT,				// iProduct
	USBD_STR_SERIAL,				// iSerialNumber
	1								// bNumConfigurations
};

// HID report descriptor. Matches the reference receiver: 64 byte IN and
// 64 byte OUT, generic desktop, undefined usage.
static const uint8_t s_HidReportDesc[] =
{
	0x05, 0x01,						// Usage Page (Generic Desktop)
	0x09, 0x00,						// Usage (Undefined)
	0xA1, 0x01,						// Collection (Application)
	0x09, 0x00,						//   Usage (Undefined)
	0x75, 0x08,						//   Report Size (8)
	0x95, 0x40,						//   Report Count (64)
	0x81, 0x02,						//   Input (Data, Var, Abs)
	0x09, 0x00,						//   Usage (Undefined)
	0x75, 0x08,						//   Report Size (8)
	0x95, 0x40,						//   Report Count (64)
	0x91, 0x02,						//   Output (Data, Var, Abs)
	0xC0							// End Collection
};

#define HID_REPORT_DESC_LEN			(sizeof(s_HidReportDesc))

// Configuration descriptor: config + interface + HID + IN endpoint + OUT endpoint.
#define CONFIG_TOTAL_LEN			(9 + 9 + 9 + 7 + 7)

static const uint8_t s_ConfigDesc[] =
{
	// Configuration
	9,								// bLength
	USB_DESC_CONFIGURATION,			// bDescriptorType
	(uint8_t)(CONFIG_TOTAL_LEN & 0xFF), (uint8_t)(CONFIG_TOTAL_LEN >> 8),
	1,								// bNumInterfaces
	1,								// bConfigurationValue
	0,								// iConfiguration
	0x80,							// bmAttributes (bus powered)
	250,							// bMaxPower (500 mA)

	// Interface
	9,								// bLength
	USB_DESC_INTERFACE,				// bDescriptorType
	0,								// bInterfaceNumber
	0,								// bAlternateSetting
	2,								// bNumEndpoints
	0x03,							// bInterfaceClass (HID)
	0x00,							// bInterfaceSubClass (none)
	0x00,							// bInterfaceProtocol (none)
	0,								// iInterface

	// HID
	9,								// bLength
	USB_DESC_HID,					// bDescriptorType
	0x11, 0x01,						// bcdHID 1.11
	0x00,							// bCountryCode
	1,								// bNumDescriptors
	USB_DESC_HID_REPORT,			// bDescriptorType (report)
	(uint8_t)(HID_REPORT_DESC_LEN & 0xFF), (uint8_t)(HID_REPORT_DESC_LEN >> 8),

	// Endpoint IN
	7,								// bLength
	USB_DESC_ENDPOINT,				// bDescriptorType
	HID_EP_ADDR_IN,					// bEndpointAddress
	0x03,							// bmAttributes (interrupt)
	(uint8_t)(HID_EP_SIZE & 0xFF), (uint8_t)(HID_EP_SIZE >> 8),
	1,								// bInterval (1 ms)

	// Endpoint OUT
	7,								// bLength
	USB_DESC_ENDPOINT,				// bDescriptorType
	HID_EP_ADDR_OUT,				// bEndpointAddress
	0x03,							// bmAttributes (interrupt)
	(uint8_t)(HID_EP_SIZE & 0xFF), (uint8_t)(HID_EP_SIZE >> 8),
	1								// bInterval (1 ms)
};

// String descriptors. USB strings are UTF-16LE.
static const uint8_t s_StrLangId[] = { 4, USB_DESC_STRING, 0x09, 0x04 };	// en-US

#define STR_DESC_HDR(n)				(uint8_t)(2 + (n) * 2), USB_DESC_STRING

static const uint8_t s_StrManufacturer[] =
{
	STR_DESC_HDR(6),
	'I', 0, '-', 0, 'S', 0, 'Y', 0, 'S', 0, 'T', 0
};

static const uint8_t s_StrProduct[] =
{
	STR_DESC_HDR(33),
	'S', 0, 'l', 0, 'i', 0, 'm', 0, 'e', 0, 'N', 0, 'R', 0, 'F', 0, ' ', 0,
	'R', 0, 'e', 0, 'c', 0, 'e', 0, 'i', 0, 'v', 0, 'e', 0, 'r', 0, ' ', 0,
	'B', 0, 'L', 0, 'Y', 0, 'S', 0, 'T', 0, '8', 0, '4', 0, '0', 0, ' ', 0,
	'D', 0, 'o', 0, 'n', 0, 'g', 0, 'l', 0, 'e', 0
};

// Serial string is built at init from the FICR device address.
static uint8_t s_StrSerial[2 + 12 * 2];

// HID IN record FIFO. Each record is 16 bytes, packed 4 per 64 byte report.
static uint8_t s_RecFifo[HID_RECORD_FIFO_LEN][HID_RECORD_SIZE];
static volatile uint32_t s_RecHead = 0;		// write index
static volatile uint32_t s_RecTail = 0;		// read index
static uint8_t s_InReport[HID_EP_SIZE];		// IN transfer buffer
static volatile bool s_InBusy = false;		// EPIN3 transfer in progress

static uint8_t s_OutReport[HID_EP_SIZE];	// OUT receive buffer

static uint8_t s_Configured = 0;			// current configuration value
static uint8_t s_Ep0Buf[2];					// small EP0 reply buffer

// EP0 IN bounce buffer in RAM. USBD EasyDMA cannot read from flash, so const
// descriptors must be copied to RAM before the transfer. 128 bytes covers the
// largest descriptor with margin.
static uint8_t s_Ep0DataBuf[128];

// Build the 12 char hex serial from the 48 bit FICR device address.
static void BuildSerial(void)
{
	uint64_t addr = (*(uint64_t *)NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFFUL;
	static const char hex[] = "0123456789ABCDEF";

	s_StrSerial[0] = sizeof(s_StrSerial);
	s_StrSerial[1] = USB_DESC_STRING;
	for (int i = 0; i < 12; i++)
	{
		uint8_t nib = (uint8_t)((addr >> (44 - i * 4)) & 0x0F);
		s_StrSerial[2 + i * 2] = (uint8_t)hex[nib];
		s_StrSerial[2 + i * 2 + 1] = 0;
	}
}

// Send a control IN data stage limited to the requested wLength. Data is
// copied to a RAM buffer first because USBD EasyDMA cannot read from flash.
static void Ep0SendData(const uint8_t *pData, uint32_t Len, uint32_t ReqLen)
{
	if (Len > ReqLen)
	{
		Len = ReqLen;
	}
	if (Len > sizeof(s_Ep0DataBuf))
	{
		Len = sizeof(s_Ep0DataBuf);
	}
	memcpy(s_Ep0DataBuf, pData, Len);
	NRFX_USBD_TRANSFER_IN(xfer, s_Ep0DataBuf, Len, 0);
	nrfx_usbd_ep_transfer(NRFX_USBD_EPIN0, &xfer);
	// nrfx_usbd handles the trailing EP0 status stage for the IN transfer.
}

// Standard GET_DESCRIPTOR.
static void HandleGetDescriptor(const nrfx_usbd_setup_t *pSetup)
{
	uint8_t type = (uint8_t)(pSetup->wValue >> 8);
	uint8_t index = (uint8_t)(pSetup->wValue & 0xFF);

	switch (type)
	{
		case USB_DESC_DEVICE:
			Ep0SendData(s_DeviceDesc, sizeof(s_DeviceDesc), pSetup->wLength);
			break;

		case USB_DESC_CONFIGURATION:
			Ep0SendData(s_ConfigDesc, sizeof(s_ConfigDesc), pSetup->wLength);
			break;

		case USB_DESC_STRING:
			switch (index)
			{
				case USBD_STR_LANGID:
					Ep0SendData(s_StrLangId, sizeof(s_StrLangId), pSetup->wLength);
					break;
				case USBD_STR_MANUFACTURER:
					Ep0SendData(s_StrManufacturer, sizeof(s_StrManufacturer), pSetup->wLength);
					break;
				case USBD_STR_PRODUCT:
					Ep0SendData(s_StrProduct, sizeof(s_StrProduct), pSetup->wLength);
					break;
				case USBD_STR_SERIAL:
					Ep0SendData(s_StrSerial, sizeof(s_StrSerial), pSetup->wLength);
					break;
				default:
					nrfx_usbd_setup_stall();
					break;
			}
			break;

		case USB_DESC_HID:
			// HID descriptor lives inside the configuration descriptor at a
			// fixed offset (after config 9 + interface 9).
			Ep0SendData(&s_ConfigDesc[18], 9, pSetup->wLength);
			break;

		case USB_DESC_HID_REPORT:
			Ep0SendData(s_HidReportDesc, sizeof(s_HidReportDesc), pSetup->wLength);
			break;

		default:
			nrfx_usbd_setup_stall();
			break;
	}
}

// Enable the HID endpoints and arm the OUT endpoint for the first report.
static void OpenHidEndpoints(void)
{
	nrfx_usbd_ep_enable(HID_EPIN);
	nrfx_usbd_ep_enable(HID_EPOUT);

	NRFX_USBD_TRANSFER_OUT(xfer, s_OutReport, sizeof(s_OutReport));
	nrfx_usbd_ep_transfer(HID_EPOUT, &xfer);
}

static void HandleStandardRequest(const nrfx_usbd_setup_t *pSetup)
{
	switch (pSetup->bRequest)
	{
		case USB_REQ_GET_DESCRIPTOR:
			HandleGetDescriptor(pSetup);
			break;

		case USB_REQ_SET_ADDRESS:
			// Address is latched by hardware. Acknowledge only.
			nrfx_usbd_setup_clear();
			break;

		case USB_REQ_SET_CONFIGURATION:
			s_Configured = (uint8_t)(pSetup->wValue & 0xFF);
			if (s_Configured != 0)
			{
				OpenHidEndpoints();
			}
			nrfx_usbd_setup_clear();
			break;

		case USB_REQ_GET_CONFIGURATION:
			s_Ep0Buf[0] = s_Configured;
			Ep0SendData(s_Ep0Buf, 1, pSetup->wLength);
			break;

		case USB_REQ_GET_STATUS:
			s_Ep0Buf[0] = 0;
			s_Ep0Buf[1] = 0;
			Ep0SendData(s_Ep0Buf, 2, pSetup->wLength);
			break;

		case USB_REQ_SET_INTERFACE:
			nrfx_usbd_setup_clear();
			break;

		case USB_REQ_GET_INTERFACE:
			s_Ep0Buf[0] = 0;
			Ep0SendData(s_Ep0Buf, 1, pSetup->wLength);
			break;

		case USB_REQ_CLEAR_FEATURE:
		case USB_REQ_SET_FEATURE:
			nrfx_usbd_setup_clear();
			break;

		default:
			nrfx_usbd_setup_stall();
			break;
	}
}

static void HandleClassRequest(const nrfx_usbd_setup_t *pSetup)
{
	switch (pSetup->bRequest)
	{
		case HID_REQ_GET_REPORT:
			// Return an empty input report.
			memset(s_Ep0Buf, 0, sizeof(s_Ep0Buf));
			Ep0SendData(s_Ep0Buf, (pSetup->wLength < sizeof(s_Ep0Buf)) ?
					pSetup->wLength : sizeof(s_Ep0Buf), pSetup->wLength);
			break;

		case HID_REQ_SET_IDLE:
		case HID_REQ_SET_PROTOCOL:
			nrfx_usbd_setup_clear();
			break;

		case HID_REQ_GET_IDLE:
		case HID_REQ_GET_PROTOCOL:
			s_Ep0Buf[0] = 0;
			Ep0SendData(s_Ep0Buf, 1, pSetup->wLength);
			break;

		case HID_REQ_SET_REPORT:
			if (pSetup->wLength == 0)
			{
				nrfx_usbd_setup_clear();
			}
			else
			{
				// Receive the report data on EP0 OUT then acknowledge in the
				// EPTRANSFER handler.
				NRFX_USBD_TRANSFER_OUT(xfer, s_OutReport,
						(pSetup->wLength < sizeof(s_OutReport)) ?
						pSetup->wLength : sizeof(s_OutReport));
				nrfx_usbd_ep_transfer(NRFX_USBD_EPOUT0, &xfer);
			}
			break;

		default:
			nrfx_usbd_setup_stall();
			break;
	}
}

static void HandleSetup(void)
{
	nrfx_usbd_setup_t setup;
	nrfx_usbd_setup_get(&setup);

	switch (setup.bmRequestType & USB_REQ_TYPE_MASK)
	{
		case USB_REQ_TYPE_STANDARD:
			HandleStandardRequest(&setup);
			break;

		case USB_REQ_TYPE_CLASS:
			HandleClassRequest(&setup);
			break;

		default:
			nrfx_usbd_setup_stall();
			break;
	}
}

// Pack up to four queued records into one 64 byte report and start the IN
// transfer. Caller guarantees EPIN3 is idle.
static void StartInTransfer(void)
{
	if (s_RecTail == s_RecHead)
	{
		return;					// nothing queued
	}

	memset(s_InReport, 0, sizeof(s_InReport));
	int n = 0;
	while (n < HID_RECORDS_PER_REPORT && s_RecTail != s_RecHead)
	{
		memcpy(&s_InReport[n * HID_RECORD_SIZE], s_RecFifo[s_RecTail], HID_RECORD_SIZE);
		s_RecTail = (s_RecTail + 1) & (HID_RECORD_FIFO_LEN - 1);
		n++;
	}

	s_InBusy = true;
	NRFX_USBD_TRANSFER_IN(xfer, s_InReport, sizeof(s_InReport), 0);
	nrfx_usbd_ep_transfer(HID_EPIN, &xfer);
}

static void HandleEpTransfer(const nrfx_usbd_evt_t *pEvent)
{
	nrfx_usbd_ep_t ep = pEvent->data.eptransfer.ep;

	if (ep == NRFX_USBD_EPOUT0)
	{
		// EP0 OUT data stage complete (SET_REPORT payload). Acknowledge.
		nrfx_usbd_setup_clear();
		return;
	}

	if (ep == HID_EPIN)
	{
		s_InBusy = false;
		if (pEvent->data.eptransfer.status == NRFX_USBD_EP_OK)
		{
			StartInTransfer();		// send the next report if records remain
		}
		return;
	}

	if (ep == HID_EPOUT)
	{
		// Host sent a 64 byte OUT report. Consumed here, then re-arm.
		// Server side configuration reports can be parsed if needed.
		NRFX_USBD_TRANSFER_OUT(xfer, s_OutReport, sizeof(s_OutReport));
		nrfx_usbd_ep_transfer(HID_EPOUT, &xfer);
		return;
	}
}

static void UsbdEventHandler(nrfx_usbd_evt_t const *p_event)
{
	switch (p_event->type)
	{
		case NRFX_USBD_EVT_SETUP:
			HandleSetup();
			break;

		case NRFX_USBD_EVT_EPTRANSFER:
			HandleEpTransfer(p_event);
			break;

		case NRFX_USBD_EVT_RESET:
			s_Configured = 0;
			s_InBusy = false;
			s_RecHead = 0;
			s_RecTail = 0;
			break;

		case NRFX_USBD_EVT_SUSPEND:
		case NRFX_USBD_EVT_RESUME:
		case NRFX_USBD_EVT_SOF:
		default:
			break;
	}
}

// nrfx_power USB event handler. Drives the USB power state machine for the
// soft connect and the HFCLK requirement of the USBD peripheral.
static void UsbPowerEventHandler(nrfx_power_usb_evt_t event)
{
	switch (event)
	{
		case NRFX_POWER_USB_EVT_DETECTED:
			if (!nrfx_usbd_is_enabled())
			{
				nrfx_usbd_enable();
			}
			break;

		case NRFX_POWER_USB_EVT_READY:
			nrfx_usbd_start(false);		// SOF events not used
			break;

		case NRFX_POWER_USB_EVT_REMOVED:
			if (nrfx_usbd_is_enabled())
			{
				nrfx_usbd_disable();
			}
			break;

		default:
			break;
	}
}

// Public entry: initialise the USB HID device.
void UsbInit()
{
	BuildSerial();

	nrfx_usbd_init(UsbdEventHandler);

	nrfx_power_config_t power_cfg = { 0 };
	nrfx_power_init(&power_cfg);

	nrfx_power_usbevt_config_t usbevt_cfg;
	memset(&usbevt_cfg, 0, sizeof(usbevt_cfg));
	usbevt_cfg.handler = UsbPowerEventHandler;
	nrfx_power_usbevt_init(&usbevt_cfg);
	nrfx_power_usbevt_enable();

	// If VBUS is already present the DETECTED event drives enable, then READY
	// drives start. nrfx_power reports the current state after enable.
}

// Forward one tracker record to the host. data points to the 16 byte ESB
// payload. rssi is placed in byte 15 for packet types other than 1 and 4
// (full precision quat and accel/mag have no spare byte).
void hid_write_packet_n(uint8_t *data, int rssi)
{
	uint32_t next = (s_RecHead + 1) & (HID_RECORD_FIFO_LEN - 1);
	if (next == s_RecTail)
	{
		return;					// FIFO full, drop
	}

	memcpy(s_RecFifo[s_RecHead], data, HID_RECORD_SIZE);
	if (data[0] != 1 && data[0] != 4)
	{
		s_RecFifo[s_RecHead][15] = (uint8_t)rssi;
	}
	s_RecHead = next;

	if (s_Configured != 0 && !s_InBusy)
	{
		StartInTransfer();
	}
}

// CDC console removed in the nrfx port. Kept as a stub so existing callers
// still link. Re-add as a composite interface if the console is needed.
void init_cli()
{
}

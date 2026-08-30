/**-------------------------------------------------------------------------
@file	usbd_cdc_desc.cpp

@brief	Native USB CDC ACM descriptor provider implementation.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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

#include "usb/usb_cdcdef.h"
#include "usb/usb_dev.h"
#include "usbd_cdc_priv.h"

#define USBD_CDC_STR_MANUFACTURER		1U
#define USBD_CDC_STR_PRODUCT			2U
#define USBD_CDC_STR_SERIAL				3U
#define USBD_CDC_STR_FUNCTION			4U
#define USBD_CDC_STR_MAXINDEX			4U
#define USBD_CDC_STR_MAXLEN				32U

#define USBD_CDC_CONFIG_VALUE			1U
#define USBD_CDC_NOTIF_INTERVAL_FS		16U
#define USBD_CDC_NOTIF_INTERVAL_HS		8U

#define USBD_CDC_DESC_FUNC_LEN \
	(sizeof(UsbInrtfAssDesc_t) + sizeof(UsbIntrfDesc_t) + \
	 sizeof(UsbCdcHeaderDesc_t) + sizeof(UsbCdcCallMngmtDesc_t) + \
	 sizeof(UsbCdcACMDesc_t) + sizeof(UsbCdcUnionDesc_t) + \
	 sizeof(UsbEndPointDesc_t) + sizeof(UsbIntrfDesc_t) + \
	 sizeof(UsbEndPointDesc_t) + sizeof(UsbEndPointDesc_t))

#define USBD_CDC_DESC_CONFIG_MAXLEN \
	(sizeof(UsbCfgDesc_t) + (USBD_CDC_FUNC_MAXCNT * USBD_CDC_DESC_FUNC_LEN))

static_assert(sizeof(UsbDevDesc_t) == 18U, "USB device descriptor size");
static_assert(sizeof(UsbDevQualDesc_t) == 10U, "USB qualifier descriptor size");
static_assert(sizeof(UsbCfgDesc_t) == 9U, "USB configuration descriptor size");
static_assert(sizeof(UsbIntrfDesc_t) == 9U, "USB interface descriptor size");
static_assert(sizeof(UsbEndPointDesc_t) == 7U, "USB endpoint descriptor size");
static_assert(sizeof(UsbInrtfAssDesc_t) == 8U, "USB IAD size");
static_assert(sizeof(UsbCdcHeaderDesc_t) == 5U, "CDC header descriptor size");
static_assert(sizeof(UsbCdcCallMngmtDesc_t) == 5U, "CDC call management descriptor size");
static_assert(sizeof(UsbCdcACMDesc_t) == 4U, "CDC ACM descriptor size");
static_assert(sizeof(UsbCdcUnionDesc_t) == 5U, "CDC union descriptor size");

static UsbDevDesc_t s_UsbdCdcDevDesc;
static UsbDevQualDesc_t s_UsbdCdcQualDesc;
static uint8_t s_UsbdCdcConfigDesc[USBD_CDC_DESC_CONFIG_MAXLEN];
static uint8_t s_UsbdCdcStrDesc[2U + (USBD_CDC_STR_MAXLEN * 2U)];

static bool UsbdCdcDescAppend(uint16_t *pOffset,
							  const void *pData, uint16_t Length)
{
	if (pOffset == nullptr || pData == nullptr ||
		(uint32_t)(*pOffset) + Length > sizeof(s_UsbdCdcConfigDesc))
	{
		return false;
	}

	memcpy(&s_UsbdCdcConfigDesc[*pOffset], pData, Length);
	*pOffset = (uint16_t)(*pOffset + Length);
	return true;
}

static int UsbdCdcDescFunctionCount(const UsbDevCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return 0;
	}

	int count = pCfg->NbCdc;
	if (count < 1)
	{
		count = 1;
	}

	return count <= USBD_CDC_FUNC_MAXCNT ? count : 0;
}

static uint8_t UsbdCdcDescMaxPower(const UsbDevCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->bSelfPowered)
	{
		return 0;
	}

	uint32_t units = ((uint32_t)pCfg->MaxPower + 1U) / 2U;
	return units > 255U ? 255U : (uint8_t)units;
}

static uint16_t UsbdCdcDescBulkMps(UsbdSpeed_t Speed)
{
	return Speed == USBD_SPEED_HIGH ?
		USBD_CDC_BULK_HS_MPS : USBD_CDC_BULK_FS_MPS;
}

static uint8_t UsbdCdcDescNotifInterval(UsbdSpeed_t Speed)
{
	// FS is expressed in frames. HS uses 2^(bInterval-1) microframes;
	// value 8 is 16 ms, matching the default full-speed interval.
	return Speed == USBD_SPEED_HIGH ?
		USBD_CDC_NOTIF_INTERVAL_HS : USBD_CDC_NOTIF_INTERVAL_FS;
}

static const uint8_t *UsbdCdcDescDevice(uint16_t *pLength)
{
	const UsbDevCfg_t *pCfg = UsbDevGetCfg();
	if (pCfg == nullptr || pLength == nullptr ||
		UsbdCdcDescFunctionCount(pCfg) == 0)
	{
		return nullptr;
	}

	memset(&s_UsbdCdcDevDesc, 0, sizeof(s_UsbdCdcDevDesc));
	s_UsbdCdcDevDesc.bLength = sizeof(s_UsbdCdcDevDesc);
	s_UsbdCdcDevDesc.bDescriptorType = USB_DESCTYPE_DEVICE;
	s_UsbdCdcDevDesc.bcdUSB = 0x0200U;

	// USB-IF Interface Association Descriptor device class triple:
	// Miscellaneous / Common Class / IAD protocol = EFh / 02h / 01h.
	s_UsbdCdcDevDesc.bDeviceClass = USB_DEVCLASS_MISC;
	s_UsbdCdcDevDesc.bDeviceSubClass = 2U;
	s_UsbdCdcDevDesc.bDeviceProtocol = 1U;
	s_UsbdCdcDevDesc.bMaxPacketSize = 64U;
	s_UsbdCdcDevDesc.idVendor = pCfg->Vid;
	s_UsbdCdcDevDesc.idProduct = pCfg->Pid;
	s_UsbdCdcDevDesc.bcdDevice = pCfg->DevVer;
	s_UsbdCdcDevDesc.iManufacturer =
		pCfg->pManufacturer != nullptr ? USBD_CDC_STR_MANUFACTURER : 0U;
	s_UsbdCdcDevDesc.iProduct =
		pCfg->pProduct != nullptr ? USBD_CDC_STR_PRODUCT : 0U;
	s_UsbdCdcDevDesc.iSerialNumber = USBD_CDC_STR_SERIAL;
	s_UsbdCdcDevDesc.bNumConfigurations = 1U;

	*pLength = sizeof(s_UsbdCdcDevDesc);
	return reinterpret_cast<const uint8_t *>(&s_UsbdCdcDevDesc);
}

static const uint8_t *UsbdCdcDescQualifier(uint16_t *pLength)
{
	if (pLength == nullptr || UsbdMaxSpeed() != USBD_SPEED_HIGH)
	{
		return nullptr;
	}

	memset(&s_UsbdCdcQualDesc, 0, sizeof(s_UsbdCdcQualDesc));
	s_UsbdCdcQualDesc.bLength = sizeof(s_UsbdCdcQualDesc);
	s_UsbdCdcQualDesc.bDescriptorType = USB_DESCTYPE_DEVICE_QUALIFIER;
	s_UsbdCdcQualDesc.bcdUSB = 0x0200U;
	s_UsbdCdcQualDesc.bDeviceClass = USB_DEVCLASS_MISC;
	s_UsbdCdcQualDesc.bDeviceSubClass = 2U;
	s_UsbdCdcQualDesc.bDeviceProtocol = 1U;
	s_UsbdCdcQualDesc.bMaxPacketSize0 = 64U;
	s_UsbdCdcQualDesc.bNumConfigurations = 1U;
	s_UsbdCdcQualDesc.bReserved = 0U;

	*pLength = sizeof(s_UsbdCdcQualDesc);
	return reinterpret_cast<const uint8_t *>(&s_UsbdCdcQualDesc);
}

static bool UsbdCdcDescBuildFunction(uint16_t *pOffset, int FunctionNo,
									 UsbdSpeed_t Speed,
									 bool HasFunctionString)
{
	const uint8_t ctrlIntrf = USBD_CDC_CTRL_INTRF(FunctionNo);
	const uint8_t dataIntrf = USBD_CDC_DATA_INTRF(FunctionNo);
	const uint16_t bulkMps = UsbdCdcDescBulkMps(Speed);

	UsbInrtfAssDesc_t iad = {};
	iad.bLength = sizeof(iad);
	iad.bDescriptorType = USB_DESCTYPE_IA;
	iad.bFirstInterface = ctrlIntrf;
	iad.bInterfaceCount = 2U;
	iad.bFunctionClass = USB_INTRFCLASS_CDC;
	iad.bFunctionSubClass = USB_CDC_SUBCLASS_ACM;
	iad.bFunctionProtocol = USB_CDC_PROT_NONE;
	iad.iFunction = 0U;

	UsbIntrfDesc_t ctrl = {};
	ctrl.bLength = sizeof(ctrl);
	ctrl.bDescriptorType = USB_DESCTYPE_INTERFACE;
	ctrl.bInterfaceNumber = ctrlIntrf;
	ctrl.bAlternateSetting = 0U;
	ctrl.bNumEndpoints = 1U;
	ctrl.bInterfaceClass = USB_INTRFCLASS_CDC;
	ctrl.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM;
	ctrl.bInterfaceProtocol = USB_CDC_PROT_NONE;
	ctrl.iInterface = HasFunctionString ? USBD_CDC_STR_FUNCTION : 0U;

	UsbCdcHeaderDesc_t header = {};
	header.bFunctionLength = sizeof(header);
	header.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	header.bDescriptorSubtype = USB_CDC_FSUBTYPE_HEADER;
	header.bcdCDC = 0x0120U;

	UsbCdcCallMngmtDesc_t call = {};
	call.bFunctionLength = sizeof(call);
	call.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	call.bDescriptorSubtype = USB_CDC_FSUBTYPE_CM;
	call.bmCapabilities = 0U;
	call.bDataInterface = dataIntrf;

	UsbCdcACMDesc_t acm = {};
	acm.bFunctionLength = sizeof(acm);
	acm.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	acm.bDescriptorSubtype = USB_CDC_FSUBTYPE_ACM;
	acm.bmCapabilities = USB_CDC_ACM_CAP_LINE_CODING;

	UsbCdcUnionDesc_t u = {};
	u.bFunctionLength = sizeof(u);
	u.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	u.bDescriptorSubtype = USB_CDC_FSUBTYPE_UNION;
	u.bControlInterface = ctrlIntrf;
	u.bSubordinateInterf[0] = dataIntrf;

	UsbEndPointDesc_t notif = {};
	notif.bLength = sizeof(notif);
	notif.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	notif.bEndpointAddress = USBD_CDC_NOTIF_EP(FunctionNo);
	notif.bmAttributes = USB_ENDPATT_TRANS_INT;
	notif.wMaxPacketSize = USBD_CDC_NOTIF_MPS;
	notif.bInterval = UsbdCdcDescNotifInterval(Speed);

	UsbIntrfDesc_t data = {};
	data.bLength = sizeof(data);
	data.bDescriptorType = USB_DESCTYPE_INTERFACE;
	data.bInterfaceNumber = dataIntrf;
	data.bAlternateSetting = 0U;
	data.bNumEndpoints = 2U;
	data.bInterfaceClass = USB_INTRFCLASS_CDCDATA;
	data.bInterfaceSubClass = 0U;
	data.bInterfaceProtocol = USB_CDCDATA_PROT_NONE;
	data.iInterface = 0U;

	UsbEndPointDesc_t out = {};
	out.bLength = sizeof(out);
	out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	out.bEndpointAddress = USBD_CDC_DATA_OUT_EP(FunctionNo);
	out.bmAttributes = USB_ENDPATT_TRANS_BULK;
	out.wMaxPacketSize = bulkMps;
	out.bInterval = 0U;

	UsbEndPointDesc_t in = out;
	in.bEndpointAddress = USBD_CDC_DATA_IN_EP(FunctionNo);

	return UsbdCdcDescAppend(pOffset, &iad, sizeof(iad)) &&
		UsbdCdcDescAppend(pOffset, &ctrl, sizeof(ctrl)) &&
		UsbdCdcDescAppend(pOffset, &header, sizeof(header)) &&
		UsbdCdcDescAppend(pOffset, &call, sizeof(call)) &&
		UsbdCdcDescAppend(pOffset, &acm, sizeof(acm)) &&
		UsbdCdcDescAppend(pOffset, &u, sizeof(u)) &&
		UsbdCdcDescAppend(pOffset, &notif, sizeof(notif)) &&
		UsbdCdcDescAppend(pOffset, &data, sizeof(data)) &&
		UsbdCdcDescAppend(pOffset, &out, sizeof(out)) &&
		UsbdCdcDescAppend(pOffset, &in, sizeof(in));
}

static const uint8_t *UsbdCdcDescConfiguration(UsbdSpeed_t Speed,
										   bool OtherSpeed,
										   uint16_t *pLength)
{
	const UsbDevCfg_t *pCfg = UsbDevGetCfg();
	const int functionCount = UsbdCdcDescFunctionCount(pCfg);

	if (pCfg == nullptr || pLength == nullptr || functionCount == 0)
	{
		return nullptr;
	}

	const uint16_t totalLength = (uint16_t)(sizeof(UsbCfgDesc_t) +
		((uint16_t)functionCount * (uint16_t)USBD_CDC_DESC_FUNC_LEN));

	if (totalLength > sizeof(s_UsbdCdcConfigDesc))
	{
		return nullptr;
	}

	memset(s_UsbdCdcConfigDesc, 0, sizeof(s_UsbdCdcConfigDesc));

	UsbCfgDesc_t cfg = {};
	cfg.bLength = sizeof(cfg);
	cfg.bDescriptorType = OtherSpeed ?
		USB_DESCTYPE_OSC : USB_DESCTYPE_CONFIGURATION;
	cfg.wTotalLength = totalLength;
	cfg.bNumInterfaces = (uint8_t)(functionCount * 2);
	cfg.bConfigurationValue = USBD_CDC_CONFIG_VALUE;
	cfg.iConfiguration = 0U;
	cfg.bmAttributes = USB_CONFATT_RESERVED | USB_CONFATT_REMOTE_WAKEUP;
	if (pCfg->bSelfPowered)
	{
		cfg.bmAttributes |= USB_CONFATT_SELF_POWERED;
	}
	cfg.bMaxPower = UsbdCdcDescMaxPower(pCfg);

	uint16_t offset = 0;
	if (!UsbdCdcDescAppend(&offset, &cfg, sizeof(cfg)))
	{
		return nullptr;
	}

	for (int i = 0; i < functionCount; i++)
	{
		if (!UsbdCdcDescBuildFunction(&offset, i, Speed,
									 pCfg->pFuncName != nullptr))
		{
			return nullptr;
		}
	}

	if (offset != totalLength)
	{
		return nullptr;
	}

	*pLength = offset;
	return s_UsbdCdcConfigDesc;
}

static const uint8_t *UsbdCdcDescString(uint8_t Index, uint16_t LangId,
										uint16_t *pLength)
{
	if (pLength == nullptr)
	{
		return nullptr;
	}

	if (Index == 0U)
	{
		s_UsbdCdcStrDesc[0] = 4U;
		s_UsbdCdcStrDesc[1] = USB_DESCTYPE_STRING;
		s_UsbdCdcStrDesc[2] = 0x09U;
		s_UsbdCdcStrDesc[3] = 0x04U;
		*pLength = 4U;
		return s_UsbdCdcStrDesc;
	}

	if (Index > USBD_CDC_STR_MAXINDEX ||
		(LangId != 0U && LangId != 0x0409U))
	{
		return nullptr;
	}

	const UsbDevCfg_t *pCfg = UsbDevGetCfg();
	if (pCfg == nullptr)
	{
		return nullptr;
	}

	const char *pStr = nullptr;
	switch (Index)
	{
		case USBD_CDC_STR_MANUFACTURER:
			pStr = pCfg->pManufacturer;
			break;

		case USBD_CDC_STR_PRODUCT:
			pStr = pCfg->pProduct;
			break;

		case USBD_CDC_STR_SERIAL:
			pStr = UsbDevGetSerial();
			break;

		case USBD_CDC_STR_FUNCTION:
			pStr = pCfg->pFuncName;
			break;

		default:
			return nullptr;
	}

	if (pStr == nullptr)
	{
		return nullptr;
	}

	size_t length = strlen(pStr);
	if (length > USBD_CDC_STR_MAXLEN)
	{
		length = USBD_CDC_STR_MAXLEN;
	}

	s_UsbdCdcStrDesc[0] = (uint8_t)(2U + (length * 2U));
	s_UsbdCdcStrDesc[1] = USB_DESCTYPE_STRING;

	for (size_t i = 0; i < length; i++)
	{
		s_UsbdCdcStrDesc[2U + (i * 2U)] = (uint8_t)pStr[i];
		s_UsbdCdcStrDesc[3U + (i * 2U)] = 0U;
	}

	*pLength = s_UsbdCdcStrDesc[0];
	return s_UsbdCdcStrDesc;
}

const uint8_t *UsbdCdcDescHandler(uint8_t DescType,
								  uint8_t DescIndex,
								  uint16_t LangId,
								  UsbdSpeed_t Speed,
								  uint16_t *pLength,
								  void *pContext)
{
	(void)pContext;

	if (pLength == nullptr)
	{
		return nullptr;
	}
	*pLength = 0U;

	switch (DescType)
	{
		case USB_DESCTYPE_DEVICE:
			return DescIndex == 0U ? UsbdCdcDescDevice(pLength) : nullptr;

		case USB_DESCTYPE_CONFIGURATION:
			return DescIndex == 0U ?
				UsbdCdcDescConfiguration(Speed, false, pLength) : nullptr;

		case USB_DESCTYPE_STRING:
			return UsbdCdcDescString(DescIndex, LangId, pLength);

		case USB_DESCTYPE_DEVICE_QUALIFIER:
			return DescIndex == 0U ? UsbdCdcDescQualifier(pLength) : nullptr;

		case USB_DESCTYPE_OSC:
			if (DescIndex != 0U || UsbdMaxSpeed() != USBD_SPEED_HIGH)
			{
				return nullptr;
			}
			return UsbdCdcDescConfiguration(
				Speed == USBD_SPEED_HIGH ? USBD_SPEED_FULL : USBD_SPEED_HIGH,
				true, pLength);

		default:
			return nullptr;
	}
}

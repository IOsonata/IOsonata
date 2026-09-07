/**-------------------------------------------------------------------------
@example	usb_iso_loopback.cpp

@brief	USB isochronous EP8 loopback for UsbIsoIntrf hardware validation.

This example deliberately contains no Bluetooth logic. It exposes one vendor
specific interface with alternate setting 0 disabled and alternate settings
1 through 6 using bidirectional isochronous endpoint 8. The maximum packet
sizes match the Bluetooth synchronous-interface sizes so the same endpoint
capability used by BtHciUsb is exercised without HCI packet semantics.

The host test is tests/usb/usb_iso_loopback_libusb.c. Each completed OUT frame
is copied by UsbIsoIntrf into its static DMA staging and queued back on IN.
UsbIsoIntrf owns endpoint open/close, DMA staging, RX re-arm, IN completion,
and suspend/resume state. This file owns only the test descriptor and echo
policy.

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2026

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

#include "usb/usb.h"
#include "usb/usb_iso.h"

#define USB_DEVNO			0
#define ISO_INTERFACE_NO	0U
#define ISO_EP_NO			8U
#define ISO_CONFIG_VALUE	1U
#define ISO_ALT_COUNT		6U

#define ISO_STR_MANUFACTURER	1U
#define ISO_STR_PRODUCT		2U
#define ISO_STR_SERIAL			3U
#define ISO_STR_INTERFACE		4U
#define ISO_STR_MAXLEN			40U

static const uint16_t s_IsoMps[ISO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};

#pragma pack(push, 1)
typedef struct __Iso_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} IsoAltDesc_t;

typedef struct __Iso_Config_Descriptor {
	UsbCfgDesc_t Config;
	UsbIntrfDesc_t Alt0;
	IsoAltDesc_t Alt[ISO_ALT_COUNT];
} IsoConfigDesc_t;
#pragma pack(pop)

static UsbIsoIntrf_t s_Iso;
static bool s_Configured;
static uint8_t s_Alt;
static uint32_t s_LoopbackDropCnt;

static UsbDevDesc_t s_DeviceDesc;
static UsbDevQualDesc_t s_QualifierDesc;
static IsoConfigDesc_t s_ConfigDesc;
static uint8_t s_StringDesc[2U + (ISO_STR_MAXLEN * 2U)];

static void IsoRxFrame(UsbIsoIntrf_t *pIntrf, const uint8_t *pData,
					   uint16_t Length, UsbCtrlrXferResult_t Result,
					   void *pContext)
{
	(void)pIntrf;
	(void)pContext;

	if (Result != USB_CTRLR_XFER_SUCCESS)
	{
		return;
	}

	// UsbIsoIntrf copies this data into its own IN DMA staging before returning.
	// A busy IN slot means this service interval could not be echoed; do not
	// queue protocol state here because this is a transport-policy test only.
	if (!UsbIsoIntrfSendFrame(&s_Iso, pData, Length))
	{
		s_LoopbackDropCnt++;
	}
}

static void IsoTxFrame(UsbIsoIntrf_t *, uint16_t,
					   UsbCtrlrXferResult_t, void *)
{
}

static bool IsoConfig(uint8_t Configuration, void *)
{
	UsbIsoIntrfClose(&s_Iso);
	s_Configured = false;
	s_Alt = 0U;

	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != ISO_CONFIG_VALUE)
	{
		return false;
	}

	s_Configured = true;
	return true;
}

static bool IsoSetInterface(uint8_t InterfaceNo, uint8_t Alt, void *)
{
	if (!s_Configured || InterfaceNo != ISO_INTERFACE_NO || Alt > ISO_ALT_COUNT)
	{
		return false;
	}

	UsbIsoIntrfClose(&s_Iso);
	s_Alt = 0U;
	if (Alt == 0U)
	{
		return true;
	}

	const uint8_t interval = UsbCtrlrHighSpeed(USB_DEVNO) ? 4U : 1U;
	if (!UsbIsoIntrfOpen(&s_Iso, s_IsoMps[Alt - 1U], interval))
	{
		return false;
	}

	s_Alt = Alt;
	return true;
}

static void IsoXfer(uint8_t, uint16_t, UsbCtrlrXferResult_t, void *)
{
	// EP8 uses the callback registered by UsbIsoIntrf. This function exists so
	// the USB core can record ownership of the endpoint in UsbFuncCfg_t.
}

static void IsoReset(void *)
{
	s_Configured = false;
	s_Alt = 0U;
	UsbIsoIntrfReset(&s_Iso);
}

static void IsoProcess(void *)
{
	if (!s_Configured || s_Alt == 0U)
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

static uint8_t IsoMaxPower(const UsbCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->bSelfPowered)
	{
		return 0U;
	}
	uint32_t units = ((uint32_t)pCfg->MaxPower + 1U) / 2U;
	return units > 255U ? 255U : (uint8_t)units;
}

static const uint8_t *IsoDeviceDescriptor(uint16_t *pLength)
{
	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr || pLength == nullptr)
	{
		return nullptr;
	}

	memset(&s_DeviceDesc, 0, sizeof(s_DeviceDesc));
	s_DeviceDesc.bLength = sizeof(s_DeviceDesc);
	s_DeviceDesc.bDescriptorType = USB_DESCTYPE_DEVICE;
	s_DeviceDesc.bcdUSB = 0x0200U;
	s_DeviceDesc.bDeviceClass = USB_DEVCLASS_NONE;
	s_DeviceDesc.bMaxPacketSize = USB_PKT_MAXLEN(USB_DEVNO, CONTROL);
	s_DeviceDesc.idVendor = pCfg->Vid;
	s_DeviceDesc.idProduct = pCfg->Pid;
	s_DeviceDesc.bcdDevice = pCfg->DevVer;
	s_DeviceDesc.iManufacturer = pCfg->pManufacturer != nullptr ?
		ISO_STR_MANUFACTURER : 0U;
	s_DeviceDesc.iProduct = pCfg->pProduct != nullptr ? ISO_STR_PRODUCT : 0U;
	s_DeviceDesc.iSerialNumber = UsbGetSerial(USB_DEVNO) != nullptr ?
		ISO_STR_SERIAL : 0U;
	s_DeviceDesc.bNumConfigurations = 1U;
	*pLength = sizeof(s_DeviceDesc);
	return reinterpret_cast<const uint8_t *>(&s_DeviceDesc);
}

static const uint8_t *IsoQualifierDescriptor(uint16_t *pLength)
{
	if (pLength == nullptr || !USB_HIGHSPEED_CAPABLE(USB_DEVNO))
	{
		return nullptr;
	}

	memset(&s_QualifierDesc, 0, sizeof(s_QualifierDesc));
	s_QualifierDesc.bLength = sizeof(s_QualifierDesc);
	s_QualifierDesc.bDescriptorType = USB_DESCTYPE_DEVICE_QUALIFIER;
	s_QualifierDesc.bcdUSB = 0x0200U;
	s_QualifierDesc.bDeviceClass = USB_DEVCLASS_NONE;
	s_QualifierDesc.bMaxPacketSize0 = USB_PKT_MAXLEN(USB_DEVNO, CONTROL);
	s_QualifierDesc.bNumConfigurations = 1U;
	*pLength = sizeof(s_QualifierDesc);
	return reinterpret_cast<const uint8_t *>(&s_QualifierDesc);
}

static const uint8_t *IsoConfigurationDescriptor(UsbSpeed_t Speed,
											 bool OtherSpeed,
											 uint16_t *pLength)
{
	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr || pLength == nullptr)
	{
		return nullptr;
	}

	memset(&s_ConfigDesc, 0, sizeof(s_ConfigDesc));
	s_ConfigDesc.Config.bLength = sizeof(s_ConfigDesc.Config);
	s_ConfigDesc.Config.bDescriptorType = OtherSpeed ?
		USB_DESCTYPE_OSC : USB_DESCTYPE_CONFIGURATION;
	s_ConfigDesc.Config.wTotalLength = sizeof(s_ConfigDesc);
	s_ConfigDesc.Config.bNumInterfaces = 1U;
	s_ConfigDesc.Config.bConfigurationValue = ISO_CONFIG_VALUE;
	s_ConfigDesc.Config.bmAttributes = USB_CONFATT_RESERVED;
	if (pCfg->bSelfPowered)
	{
		s_ConfigDesc.Config.bmAttributes |= USB_CONFATT_SELF_POWERED;
	}
	s_ConfigDesc.Config.bMaxPower = IsoMaxPower(pCfg);

	s_ConfigDesc.Alt0.bLength = sizeof(s_ConfigDesc.Alt0);
	s_ConfigDesc.Alt0.bDescriptorType = USB_DESCTYPE_INTERFACE;
	s_ConfigDesc.Alt0.bInterfaceNumber = ISO_INTERFACE_NO;
	s_ConfigDesc.Alt0.bAlternateSetting = 0U;
	s_ConfigDesc.Alt0.bNumEndpoints = 0U;
	s_ConfigDesc.Alt0.bInterfaceClass = USB_INTRFCLASS_VENDOR;
	s_ConfigDesc.Alt0.iInterface = ISO_STR_INTERFACE;

	const uint8_t interval = Speed == USB_SPEED_HIGH ? 4U : 1U;
	for (uint8_t i = 0U; i < ISO_ALT_COUNT; i++)
	{
		IsoAltDesc_t *pAlt = &s_ConfigDesc.Alt[i];
		pAlt->Interface = s_ConfigDesc.Alt0;
		pAlt->Interface.bAlternateSetting = (uint8_t)(i + 1U);
		pAlt->Interface.bNumEndpoints = 2U;

		pAlt->Out.bLength = sizeof(pAlt->Out);
		pAlt->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		pAlt->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(ISO_EP_NO);
		pAlt->Out.bmAttributes = USB_ENDPATT_TRANS_ISO;
		pAlt->Out.wMaxPacketSize = s_IsoMps[i];
		pAlt->Out.bInterval = interval;

		pAlt->In = pAlt->Out;
		pAlt->In.bEndpointAddress = USB_ENDPADDR_DIRIN(ISO_EP_NO);
	}

	*pLength = sizeof(s_ConfigDesc);
	return reinterpret_cast<const uint8_t *>(&s_ConfigDesc);
}

static const uint8_t *IsoStringDescriptor(uint8_t Index, uint16_t LangId,
										  uint16_t *pLength)
{
	if (pLength == nullptr)
	{
		return nullptr;
	}
	if (Index == 0U)
	{
		s_StringDesc[0] = 4U;
		s_StringDesc[1] = USB_DESCTYPE_STRING;
		s_StringDesc[2] = 0x09U;
		s_StringDesc[3] = 0x04U;
		*pLength = 4U;
		return s_StringDesc;
	}
	if (Index > ISO_STR_INTERFACE ||
		(LangId != 0U && LangId != 0x0409U))
	{
		return nullptr;
	}

	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr)
	{
		return nullptr;
	}

	const char *pStr = nullptr;
	switch (Index)
	{
		case ISO_STR_MANUFACTURER:
			pStr = pCfg->pManufacturer;
			break;
		case ISO_STR_PRODUCT:
			pStr = pCfg->pProduct;
			break;
		case ISO_STR_SERIAL:
			pStr = UsbGetSerial(USB_DEVNO);
			break;
		case ISO_STR_INTERFACE:
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
	if (length > ISO_STR_MAXLEN)
	{
		length = ISO_STR_MAXLEN;
	}
	s_StringDesc[0] = (uint8_t)(2U + (length * 2U));
	s_StringDesc[1] = USB_DESCTYPE_STRING;
	for (size_t i = 0U; i < length; i++)
	{
		s_StringDesc[2U + (i * 2U)] = (uint8_t)pStr[i];
		s_StringDesc[3U + (i * 2U)] = 0U;
	}
	*pLength = s_StringDesc[0];
	return s_StringDesc;
}

static const uint8_t *IsoDescHandler(uint8_t DescType, uint8_t DescIndex,
									 uint16_t LangId, UsbSpeed_t Speed,
									 uint16_t *pLength, void *)
{
	if (pLength == nullptr)
	{
		return nullptr;
	}
	*pLength = 0U;

	switch (DescType)
	{
		case USB_DESCTYPE_DEVICE:
			return DescIndex == 0U ? IsoDeviceDescriptor(pLength) : nullptr;
		case USB_DESCTYPE_CONFIGURATION:
			return DescIndex == 0U ?
				IsoConfigurationDescriptor(Speed, false, pLength) : nullptr;
		case USB_DESCTYPE_STRING:
			return IsoStringDescriptor(DescIndex, LangId, pLength);
		case USB_DESCTYPE_DEVICE_QUALIFIER:
			return DescIndex == 0U ? IsoQualifierDescriptor(pLength) : nullptr;
		case USB_DESCTYPE_OSC:
			if (DescIndex != 0U || !USB_HIGHSPEED_CAPABLE(USB_DEVNO))
			{
				return nullptr;
			}
			return IsoConfigurationDescriptor(
				Speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH,
				true, pLength);
		default:
			return nullptr;
	}
}

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Vid = 0x1209,
	.Pid = 0x0003,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB ISO Loopback",
	.pSerial = nullptr,
	.pFuncName = "USB ISO Loopback",
	.NbCdc = 0,
	.IntPrio = 6,
	.bSelfPowered = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
	.DescHandler = IsoDescHandler,
	.pDescContext = nullptr,
};

int main()
{
	if (!UsbInit(&s_UsbCfg) || !USB_ISO_SUPPORTED(USB_DEVNO) ||
		(USB_ISO_EPIN_MASK(USB_DEVNO) & (1U << ISO_EP_NO)) == 0U ||
		(USB_ISO_EPOUT_MASK(USB_DEVNO) & (1U << ISO_EP_NO)) == 0U)
	{
		return -1;
	}

	UsbIsoIntrfCfg_t isoCfg = {};
	isoCfg.DevNo = USB_DEVNO;
	isoCfg.EpNo = ISO_EP_NO;
	isoCfg.RxHandler = IsoRxFrame;
	isoCfg.TxHandler = IsoTxFrame;
	if (!UsbIsoIntrfInit(&s_Iso, &isoCfg))
	{
		return -1;
	}

	UsbFuncCfg_t func = {};
	func.FirstInterface = ISO_INTERFACE_NO;
	func.InterfaceCount = 1U;
	func.EpInMask = (uint16_t)(1U << ISO_EP_NO);
	func.EpOutMask = (uint16_t)(1U << ISO_EP_NO);
	func.ConfigHandler = IsoConfig;
	func.SetInterfaceHandler = IsoSetInterface;
	func.XferHandler = IsoXfer;
	func.ResetHandler = IsoReset;
	func.ProcessHandler = IsoProcess;
	if (!UsbRegisterFunc(USB_DEVNO, &func))
	{
		return -1;
	}

	(void)UsbEnable(USB_DEVNO);
	while (1)
	{
		UsbProcess(USB_DEVNO);
	}

	return 0;
}

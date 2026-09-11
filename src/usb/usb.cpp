/**-------------------------------------------------------------------------
@file	usb.cpp

@brief	Generic USB layer.

Identity, Chapter 9 and device class dispatch in one place. usb_dev.cpp held the
application entry points and usbd_core.cpp held the protocol engine, which
meant two init calls and two config structs carrying the same interrupt
priority. UsbInit() is now one call.

Bus power is not watched through a port callback. UsbProcess() polls
UsbCtrlrVbusDetected() on the level rather than the edge, because a board
already on a cable at reset never produces an edge, and reports
USB_EVT_ATTACHED and USB_EVT_DETACHED to the application itself.

DevNo is stored at init and passed to every UsbCtrlr call. The state here is
still file scope, so one controller is initialized at a time. That matches
every part shipped so far, where USB_CTRLR_CNT is 1.

@author	Hoang Nguyen Hoan
@date	Sep. 3, 2026

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

#include "usb/usb.h"

/// Controller this instance drives, set by UsbInit.
static int s_UsbDevNo = 0;

/// Bus power level at the previous UsbProcess pass, for edge reporting.
static bool s_UsbVbusLast = false;

#define USB_CORE_CLASS_MAXCNT		8
#define USB_CORE_INTRF_MAXCNT		16

/// Chapter 9 settings. Built by UsbInit from UsbCfg_t and usb_ctrlr.h, never
/// supplied by an application, which is why it is no longer in a header.
typedef struct __Usb_Core_Config {
	uint8_t Ep0Mps;					//!< EP0 max packet size
} UsbCoreCfg_t;

#define USBD_CORE_EP0_MPS_DEFAULT		64U
#define USBD_CORE_DEVICE_DESC_LEN		((uint16_t)sizeof(UsbDevDesc_t))
#define USBD_CORE_CONFIG_DESC_LEN		((uint16_t)sizeof(UsbCfgDesc_t))
#define USB_CORE_STRING_DESC_MAXLEN		66U
#define USB_CORE_STR_MANUFACTURER		1U
#define USB_CORE_STR_PRODUCT			2U
#define USB_CORE_STR_SERIAL			3U
#define USB_CORE_STR_FUNCTION			4U

typedef enum __Usbd_Core_Ctrl_State {
	USB_CTRL_IDLE,
	USB_CTRL_DATA_IN,
	USB_CTRL_DATA_IN_ZLP,
	USB_CTRL_DATA_OUT,
	USB_CTRL_STATUS_IN,
	USB_CTRL_STATUS_OUT,
} UsbCoreCtrlState_t;

static UsbCoreCfg_t s_CoreCfg;
static UsbClass *s_CoreObject[USB_CORE_CLASS_MAXCNT];
static int s_CoreObjectCnt;
// Endpoint to class index, [0] OUT and [1] IN. Ownership masks are fixed once
// a class registers and may not overlap. They route endpoint-recipient
// requests; data endpoint events go directly to their registered callbacks.
// Minus one means no class owns that endpoint.
static int8_t s_CoreEpClass[2][16];

static bool s_CoreInitialized;
static bool s_CoreStarted;
static bool s_CoreSuspended;
static bool s_RemoteWakeup;
static uint8_t s_Address;
static uint8_t s_PendingAddress;
static bool s_AddressPending;
static uint8_t s_Configuration;
static uint8_t s_NumInterfaces;
static uint8_t s_Alternate[USB_CORE_INTRF_MAXCNT];
static uint16_t s_HaltIn;
static uint16_t s_HaltOut;

static UsbCoreCtrlState_t s_CtrlState;
static UsbSetupData_t s_Setup;
static int s_ActiveClass;
static uint8_t *s_CtrlData;
static uint16_t s_CtrlDataLen;
static uint16_t s_CtrlActual;
static bool s_CtrlNeedZlp;
static uint8_t s_CtrlReply[2];
static UsbDevDesc_t s_CoreDeviceDesc;
static UsbDevQualDesc_t s_CoreQualifierDesc;
static uint8_t s_CoreConfigDesc[USB_CONFIG_DESC_MAXLEN];
static uint8_t s_CoreStringDesc[USB_CORE_STRING_DESC_MAXLEN];

static bool UsbCoreValidateDescriptorFragment(const UsbDeviceClass *pClass,
											 const uint8_t *pDesc,
											 uint16_t Length);

static uint8_t UsbCoreRecipient(const UsbSetupData_t *pSetup)
{
	return pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT;
}

static bool UsbCoreDirIn(const UsbSetupData_t *pSetup)
{
	return (pSetup->bmRequestType & USB_REQTYPE_DIRHOST) != 0;
}

static bool UsbCoreValidEp0Mps(uint8_t Mps)
{
	return Mps == 8U || Mps == 16U || Mps == 32U || Mps == 64U;
}

static const uint8_t *UsbCoreGetDescriptor(uint8_t Type, uint8_t Index,
										uint16_t LangId, uint16_t *pLength)
{
	return UsbGetDescriptor(s_UsbDevNo, Type, Index, LangId,
		UsbGetSpeed(s_UsbDevNo), pLength);
}

static uint8_t UsbDescMaxPower(const UsbCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->bSelfPowered)
	{
		return 0U;
	}

	const uint32_t units = ((uint32_t)pCfg->MaxPower + 1U) / 2U;
	return units > 255U ? 255U : (uint8_t)units;
}

static const uint8_t *UsbDescDevice(int DevNo, uint8_t Index,
									uint16_t *pLength)
{
	const UsbCfg_t *pCfg = UsbGetCfg(DevNo);
	if (pCfg == nullptr || pLength == nullptr || Index != 0U)
	{
		return nullptr;
	}

	memset(&s_CoreDeviceDesc, 0, sizeof(s_CoreDeviceDesc));
	s_CoreDeviceDesc.bLength = sizeof(s_CoreDeviceDesc);
	s_CoreDeviceDesc.bDescriptorType = USB_DESCTYPE_DEVICE;
	s_CoreDeviceDesc.bcdUSB = 0x0200U;
	s_CoreDeviceDesc.bDeviceClass = pCfg->DeviceClass;
	s_CoreDeviceDesc.bDeviceSubClass = pCfg->DeviceSubClass;
	s_CoreDeviceDesc.bDeviceProtocol = pCfg->DeviceProtocol;
	s_CoreDeviceDesc.bMaxPacketSize = USB_PKT_MAXLEN(DevNo, CONTROL);
	s_CoreDeviceDesc.idVendor = pCfg->Vid;
	s_CoreDeviceDesc.idProduct = pCfg->Pid;
	s_CoreDeviceDesc.bcdDevice = pCfg->DevVer;
	s_CoreDeviceDesc.iManufacturer = pCfg->pManufacturer != nullptr ?
		USB_CORE_STR_MANUFACTURER : 0U;
	s_CoreDeviceDesc.iProduct = pCfg->pProduct != nullptr ?
		USB_CORE_STR_PRODUCT : 0U;
	s_CoreDeviceDesc.iSerialNumber = UsbGetSerial(DevNo) != nullptr ?
		USB_CORE_STR_SERIAL : 0U;
	s_CoreDeviceDesc.bNumConfigurations = 1U;

	*pLength = sizeof(s_CoreDeviceDesc);
	return reinterpret_cast<const uint8_t *>(&s_CoreDeviceDesc);
}

static const uint8_t *UsbDescQualifier(int DevNo, uint8_t Index,
									   uint16_t *pLength)
{
	const UsbCfg_t *pCfg = UsbGetCfg(DevNo);
	if (pCfg == nullptr || pLength == nullptr || Index != 0U ||
		!USB_HIGHSPEED_CAPABLE(DevNo))
	{
		return nullptr;
	}

	memset(&s_CoreQualifierDesc, 0, sizeof(s_CoreQualifierDesc));
	s_CoreQualifierDesc.bLength = sizeof(s_CoreQualifierDesc);
	s_CoreQualifierDesc.bDescriptorType = USB_DESCTYPE_DEVICE_QUALIFIER;
	s_CoreQualifierDesc.bcdUSB = 0x0200U;
	s_CoreQualifierDesc.bDeviceClass = pCfg->DeviceClass;
	s_CoreQualifierDesc.bDeviceSubClass = pCfg->DeviceSubClass;
	s_CoreQualifierDesc.bDeviceProtocol = pCfg->DeviceProtocol;
	s_CoreQualifierDesc.bMaxPacketSize0 = USB_PKT_MAXLEN(DevNo, CONTROL);
	s_CoreQualifierDesc.bNumConfigurations = 1U;

	*pLength = sizeof(s_CoreQualifierDesc);
	return reinterpret_cast<const uint8_t *>(&s_CoreQualifierDesc);
}

static const uint8_t *UsbDescString(int DevNo, uint8_t Index,
								 uint16_t LangId, uint16_t *pLength)
{
	if (pLength == nullptr)
	{
		return nullptr;
	}

	if (Index == 0U)
	{
		s_CoreStringDesc[0] = 4U;
		s_CoreStringDesc[1] = USB_DESCTYPE_STRING;
		s_CoreStringDesc[2] = 0x09U;
		s_CoreStringDesc[3] = 0x04U;
		*pLength = 4U;
		return s_CoreStringDesc;
	}

	if (LangId != 0U && LangId != 0x0409U)
	{
		return nullptr;
	}

	const UsbCfg_t *pCfg = UsbGetCfg(DevNo);
	if (pCfg == nullptr)
	{
		return nullptr;
	}

	const char *pString = nullptr;
	switch (Index)
	{
		case USB_CORE_STR_MANUFACTURER:
			pString = pCfg->pManufacturer;
			break;
		case USB_CORE_STR_PRODUCT:
			pString = pCfg->pProduct;
			break;
		case USB_CORE_STR_SERIAL:
			pString = UsbGetSerial(DevNo);
			break;
		case USB_CORE_STR_FUNCTION:
			pString = pCfg->pFuncName;
			break;
		default:
			return nullptr;
	}

	if (pString == nullptr)
	{
		return nullptr;
	}

	size_t length = strlen(pString);
	const size_t maxLength = (sizeof(s_CoreStringDesc) - 2U) / 2U;
	if (length > maxLength)
	{
		length = maxLength;
	}

	s_CoreStringDesc[0] = (uint8_t)(2U + length * 2U);
	s_CoreStringDesc[1] = USB_DESCTYPE_STRING;
	for (size_t i = 0; i < length; i++)
	{
		s_CoreStringDesc[2U + i * 2U] = (uint8_t)pString[i];
		s_CoreStringDesc[3U + i * 2U] = 0U;
	}

	*pLength = s_CoreStringDesc[0];
	return s_CoreStringDesc;
}

static const uint8_t *UsbDescConfiguration(int DevNo, uint8_t Index,
										UsbSpeed_t Speed, bool OtherSpeed,
										uint16_t *pLength)
{
	const UsbCfg_t *pCfg = UsbGetCfg(DevNo);
	if (pCfg == nullptr || pLength == nullptr || Index != 0U ||
		(OtherSpeed && !USB_HIGHSPEED_CAPABLE(DevNo)))
	{
		return nullptr;
	}

	if (OtherSpeed)
	{
		Speed = Speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH;
	}

	uint16_t totalLength = sizeof(UsbCfgDesc_t);
	uint8_t interfaceCount = 0U;
	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		const UsbDeviceClass *pClass =
			static_cast<const UsbDeviceClass *>(s_CoreObject[i]);
		const uint16_t fragmentLength = pClass->DescriptorLength(Speed);
		const uint8_t *pFragment = pClass->Descriptor(Speed);
		if (pFragment == nullptr || fragmentLength == 0U ||
			!UsbCoreValidateDescriptorFragment(pClass, pFragment,
				fragmentLength) ||
			(uint32_t)totalLength + fragmentLength > sizeof(s_CoreConfigDesc))
		{
			return nullptr;
		}
		totalLength = (uint16_t)(totalLength + fragmentLength);
		const uint8_t last = (uint8_t)(pClass->FirstInterface() +
			pClass->InterfaceCount());
		if (last > interfaceCount)
		{
			interfaceCount = last;
		}
	}

	if (s_CoreObjectCnt == 0 || interfaceCount == 0U)
	{
		return nullptr;
	}

	UsbCfgDesc_t config = {};
	config.bLength = sizeof(config);
	config.bDescriptorType = OtherSpeed ?
		USB_DESCTYPE_OSC : USB_DESCTYPE_CONFIGURATION;
	config.wTotalLength = totalLength;
	config.bNumInterfaces = interfaceCount;
	config.bConfigurationValue = 1U;
	config.bmAttributes = USB_CONFATT_RESERVED;
	if (pCfg->bSelfPowered)
	{
		config.bmAttributes |= USB_CONFATT_SELF_POWERED;
	}
	if (pCfg->bRemoteWakeup)
	{
		config.bmAttributes |= USB_CONFATT_REMOTE_WAKEUP;
	}
	config.bMaxPower = UsbDescMaxPower(pCfg);

	memcpy(s_CoreConfigDesc, &config, sizeof(config));
	uint16_t offset = sizeof(config);
	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		const UsbDeviceClass *pClass =
			static_cast<const UsbDeviceClass *>(s_CoreObject[i]);
		const uint16_t fragmentLength = pClass->DescriptorLength(Speed);
		memcpy(&s_CoreConfigDesc[offset], pClass->Descriptor(Speed),
			fragmentLength);
		offset = (uint16_t)(offset + fragmentLength);
	}

	*pLength = totalLength;
	return s_CoreConfigDesc;
}

static bool UsbCoreValidateConfigDescriptor(const uint8_t *pDesc,
											 uint16_t ProviderLength,
											 uint8_t DescType,
											 uint16_t *pLength)
{
	if (pDesc == nullptr || pLength == nullptr ||
		ProviderLength < USBD_CORE_CONFIG_DESC_LEN ||
		pDesc[0] != USBD_CORE_CONFIG_DESC_LEN ||
		pDesc[1] != DescType)
	{
		return false;
	}

	const uint16_t totalLength = (uint16_t)pDesc[2] |
		((uint16_t)pDesc[3] << 8);
	if (totalLength < USBD_CORE_CONFIG_DESC_LEN ||
		totalLength > ProviderLength)
	{
		return false;
	}

	uint16_t ofs = 0;
	while (ofs < totalLength)
	{
		if ((uint16_t)(ofs + 2U) > totalLength)
		{
			return false;
		}

		const uint8_t dlen = pDesc[ofs];
		if (dlen < 2U || (uint16_t)(ofs + dlen) > totalLength)
		{
			return false;
		}

		ofs = (uint16_t)(ofs + dlen);
	}

	*pLength = totalLength;
	return true;
}

static const uint8_t *UsbCoreGetConfigDescriptor(uint8_t Type, uint8_t Index,
												uint16_t *pLength)
{
	if (pLength == nullptr ||
		(Type != USB_DESCTYPE_CONFIGURATION && Type != USB_DESCTYPE_OSC))
	{
		return nullptr;
	}

	uint16_t providerLength;
	const uint8_t *pDesc = UsbCoreGetDescriptor(Type, Index, 0,
												&providerLength);

	return UsbCoreValidateConfigDescriptor(pDesc, providerLength, Type,
													 pLength) ? pDesc : nullptr;
}

static const uint8_t *UsbCoreGetConfigByIndex(uint8_t Index,
										uint16_t *pLength)
{
	return UsbCoreGetConfigDescriptor(USB_DESCTYPE_CONFIGURATION, Index,
												   pLength);
}

static uint8_t UsbCoreConfigurationCount(void)
{
	uint16_t len;
	const uint8_t *pDesc = UsbCoreGetDescriptor(USB_DESCTYPE_DEVICE,
												0, 0, &len);

	if (pDesc == nullptr || len < USBD_CORE_DEVICE_DESC_LEN ||
		pDesc[0] < USBD_CORE_DEVICE_DESC_LEN ||
		pDesc[1] != USB_DESCTYPE_DEVICE)
	{
		return 0;
	}

	return pDesc[17];
}

static const uint8_t *UsbCoreGetConfigByValue(uint8_t Value,
										uint16_t *pLength)
{
	const uint8_t count = UsbCoreConfigurationCount();

	for (uint8_t i = 0; i < count; i++)
	{
		uint16_t len;
		const uint8_t *pDesc = UsbCoreGetConfigByIndex(i, &len);

		if (pDesc != nullptr && pDesc[5] == Value)
		{
			*pLength = len;
			return pDesc;
		}
	}

	return nullptr;
}

static const uint8_t *UsbCoreActiveConfig(uint16_t *pLength)
{
	if (s_Configuration != 0)
	{
		return UsbCoreGetConfigByValue(s_Configuration, pLength);
	}

	return UsbCoreGetConfigByIndex(0, pLength);
}

static bool UsbCoreInterfaceExists(uint8_t InterfaceNo)
{
	uint16_t len;
	const uint8_t *pDesc = UsbCoreActiveConfig(&len);

	if (pDesc == nullptr)
	{
		return false;
	}

	uint16_t ofs = 0;
	while ((uint16_t)(ofs + 2U) <= len)
	{
		const uint8_t dlen = pDesc[ofs];
		const uint8_t type = pDesc[ofs + 1U];

		if (dlen < 2U || (uint16_t)(ofs + dlen) > len)
		{
			break;
		}

		if (type == USB_DESCTYPE_INTERFACE && dlen >= sizeof(UsbIntrfDesc_t) &&
			pDesc[ofs + 2U] == InterfaceNo)
		{
			return true;
		}

		ofs = (uint16_t)(ofs + dlen);
	}

	return false;
}

static bool UsbCoreInterfaceAlternateExists(uint8_t InterfaceNo,
												 uint8_t Alternate)
{
	uint16_t len;
	const uint8_t *pDesc = UsbCoreActiveConfig(&len);

	if (pDesc == nullptr)
	{
		return false;
	}

	uint16_t ofs = 0;
	while ((uint16_t)(ofs + 2U) <= len)
	{
		const uint8_t dlen = pDesc[ofs];
		const uint8_t type = pDesc[ofs + 1U];

		if (dlen < 2U || (uint16_t)(ofs + dlen) > len)
		{
			break;
		}

		if (type == USB_DESCTYPE_INTERFACE && dlen >= sizeof(UsbIntrfDesc_t) &&
			pDesc[ofs + 2U] == InterfaceNo &&
			pDesc[ofs + 3U] == Alternate)
		{
			return true;
		}

		ofs = (uint16_t)(ofs + dlen);
	}

	return false;
}

static bool UsbCoreInterfaceEndpointMasks(uint8_t InterfaceNo,
										   uint8_t Alternate,
										   uint16_t *pInMask,
										   uint16_t *pOutMask)
{
	if (pInMask == nullptr || pOutMask == nullptr)
	{
		return false;
	}

	*pInMask = 0;
	*pOutMask = 0;

	uint16_t len;
	const uint8_t *pDesc = UsbCoreActiveConfig(&len);
	if (pDesc == nullptr)
	{
		return false;
	}

	bool targetInterface = false;
	bool found = false;
	uint16_t ofs = 0;

	while ((uint16_t)(ofs + 2U) <= len)
	{
		const uint8_t dlen = pDesc[ofs];
		const uint8_t type = pDesc[ofs + 1U];

		if (dlen < 2U || (uint16_t)(ofs + dlen) > len)
		{
			break;
		}

		if (type == USB_DESCTYPE_INTERFACE)
		{
			if (dlen >= sizeof(UsbIntrfDesc_t))
			{
				targetInterface =
					pDesc[ofs + 2U] == InterfaceNo &&
					pDesc[ofs + 3U] == Alternate;
				found = found || targetInterface;
			}
			else
			{
				targetInterface = false;
			}
		}
		else if (targetInterface && type == USB_DESCTYPE_ENDPOINT &&
				 dlen >= sizeof(UsbEndPointDesc_t))
		{
			const uint8_t epAddr = pDesc[ofs + 2U];
			const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);

			if (epNum != 0U && (epAddr & 0x70U) == 0U)
			{
				uint16_t *pMask = USB_ENDPADDR_IS_IN(epAddr) ?
					pInMask : pOutMask;
				*pMask |= (uint16_t)(1U << epNum);
			}
		}

		ofs = (uint16_t)(ofs + dlen);
	}

	return found;
}

static void UsbCoreClearInterfaceHalt(uint8_t InterfaceNo,
									  uint8_t OldAlternate,
									  uint8_t NewAlternate)
{
	uint16_t oldIn = 0;
	uint16_t oldOut = 0;
	uint16_t newIn = 0;
	uint16_t newOut = 0;

	(void)UsbCoreInterfaceEndpointMasks(InterfaceNo, OldAlternate,
										&oldIn, &oldOut);
	(void)UsbCoreInterfaceEndpointMasks(InterfaceNo, NewAlternate,
										&newIn, &newOut);

	s_HaltIn &= (uint16_t)~(oldIn | newIn);
	s_HaltOut &= (uint16_t)~(oldOut | newOut);
}

static bool UsbCoreEndpointExists(uint8_t EpAddr)
{
	if (USB_ENDPADDR_NUM(EpAddr) == 0)
	{
		return true;
	}

	uint16_t len;
	const uint8_t *pDesc = UsbCoreActiveConfig(&len);

	if (pDesc == nullptr)
	{
		return false;
	}

	bool activeInterface = false;
	uint16_t ofs = 0;
	while ((uint16_t)(ofs + 2U) <= len)
	{
		const uint8_t dlen = pDesc[ofs];
		const uint8_t type = pDesc[ofs + 1U];

		if (dlen < 2U || (uint16_t)(ofs + dlen) > len)
		{
			break;
		}

		if (type == USB_DESCTYPE_INTERFACE)
		{
			if (dlen >= sizeof(UsbIntrfDesc_t))
			{
				const uint8_t interfaceNo = pDesc[ofs + 2U];
				const uint8_t alternate = pDesc[ofs + 3U];
				activeInterface =
					interfaceNo < USB_CORE_INTRF_MAXCNT &&
					s_Alternate[interfaceNo] == alternate;
			}
			else
			{
				activeInterface = false;
			}
		}
		else if (activeInterface && type == USB_DESCTYPE_ENDPOINT &&
				 dlen >= sizeof(UsbEndPointDesc_t) &&
				 pDesc[ofs + 2U] == EpAddr)
		{
			return true;
		}

		ofs = (uint16_t)(ofs + dlen);
	}

	return false;
}

static int UsbCoreFindClass(uint8_t InterfaceNo)
{
	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		const UsbDeviceClass *pClass =
			static_cast<const UsbDeviceClass *>(s_CoreObject[i]);
		const uint8_t first = pClass->FirstInterface();
		const uint8_t count = pClass->InterfaceCount();

		if (count != 0 && InterfaceNo >= first &&
			InterfaceNo < (uint8_t)(first + count))
		{
			return i;
		}
	}

	return -1;
}

static int UsbCoreFindEndpointClass(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0)
	{
		return -1;
	}

	if (epNum >= 16)
	{
		return -1;
	}

	return s_CoreEpClass[USB_ENDPADDR_IS_IN(EpAddr) ? 1 : 0][epNum];
}

static void UsbCoreResetControl(void)
{
	s_CtrlState = USB_CTRL_IDLE;
	s_ActiveClass = -1;
	s_CtrlData = nullptr;
	s_CtrlDataLen = 0;
	s_CtrlActual = 0;
	s_CtrlNeedZlp = false;
	s_PendingAddress = 0;
	s_AddressPending = false;
}

static void UsbCoreAbortControl(void);
static bool UsbCoreHandleClassRequest(void);

static void UsbCoreStallControl(void)
{
	UsbCoreAbortControl();
	UsbCtrlrEpStall(s_UsbDevNo, 0);
}

static bool UsbCoreInvokeActive(UsbCtrlStage_t Stage,
								 uint16_t Length)
{
	uint8_t *pData = s_CtrlData;
	uint16_t len = Length;
	if (s_ActiveClass < 0 || s_ActiveClass >= s_CoreObjectCnt)
	{
		return true;
	}

	return static_cast<UsbDeviceClass *>(s_CoreObject[s_ActiveClass])->
		Control(&s_Setup, Stage, &pData, &len);
}

static void UsbCoreAbortControl(void)
{
	if (s_ActiveClass >= 0 && s_ActiveClass < s_CoreObjectCnt)
	{
		(void)UsbCoreInvokeActive(USB_CTRL_ABORT, s_CtrlActual);
	}

	UsbCoreResetControl();
}

static bool UsbCoreStartStatus(void)
{
	const uint8_t epAddr = UsbCoreDirIn(&s_Setup) ?
		USB_ENDPADDR_DIR_OUT : USB_ENDPADDR_DIR_IN;

	s_CtrlState = USB_ENDPADDR_IS_IN(epAddr) ?
		USB_CTRL_STATUS_IN : USB_CTRL_STATUS_OUT;

	if (!UsbCtrlrEp0Xfer(s_UsbDevNo, epAddr, nullptr, 0))
	{
		UsbCoreStallControl();
		return false;
	}

	return true;
}

static bool UsbCoreStartIn(const uint8_t *pData, uint16_t Available)
{
	if (s_Setup.wLength == 0)
	{
		s_CtrlData = const_cast<uint8_t *>(pData);
		s_CtrlDataLen = 0;
		s_CtrlActual = 0;
		return UsbCoreStartStatus();
	}

	const uint16_t sendLen = Available < s_Setup.wLength ?
		Available : s_Setup.wLength;

	if (sendLen > 0 && pData == nullptr)
	{
		return false;
	}

	s_CtrlData = const_cast<uint8_t *>(pData);
	s_CtrlDataLen = sendLen;
	s_CtrlActual = 0;
	s_CtrlNeedZlp =
		sendLen > 0 && sendLen < s_Setup.wLength &&
		(sendLen % s_CoreCfg.Ep0Mps) == 0;
	s_CtrlState = USB_CTRL_DATA_IN;

	if (!UsbCtrlrEp0Xfer(s_UsbDevNo, USB_ENDPADDR_DIR_IN, s_CtrlData, sendLen))
	{
		return false;
	}

	return true;
}

static bool UsbCoreStartOut(uint8_t *pData, uint16_t Capacity)
{
	if (s_Setup.wLength == 0)
	{
		s_CtrlData = pData;
		s_CtrlDataLen = 0;
		s_CtrlActual = 0;
		return UsbCoreStartStatus();
	}

	if (pData == nullptr || Capacity < s_Setup.wLength)
	{
		return false;
	}

	s_CtrlData = pData;
	s_CtrlDataLen = s_Setup.wLength;
	s_CtrlActual = 0;
	s_CtrlState = USB_CTRL_DATA_OUT;

	if (!UsbCtrlrEp0Xfer(s_UsbDevNo, USB_ENDPADDR_DIR_OUT, pData, s_Setup.wLength))
	{
		return false;
	}

	return true;
}

static bool UsbCoreConfigRemoteWakeupCapable(void)
{
	uint16_t len;
	const uint8_t *pDesc = UsbCoreActiveConfig(&len);

	return pDesc != nullptr && len >= USBD_CORE_CONFIG_DESC_LEN &&
		(pDesc[7] & USB_CONFATT_REMOTE_WAKEUP) != 0;
}

static bool UsbCoreConfigSelfPowered(void)
{
	uint16_t len;
	const uint8_t *pDesc = UsbCoreActiveConfig(&len);

	return pDesc != nullptr && len >= USBD_CORE_CONFIG_DESC_LEN &&
		(pDesc[7] & USB_CONFATT_SELF_POWERED) != 0;
}

static void UsbCoreClearEndpointState(void)
{
	s_HaltIn = 0;
	s_HaltOut = 0;
	s_RemoteWakeup = false;
	memset(s_Alternate, 0, sizeof(s_Alternate));
}

static bool UsbCoreSelectConfig(int Index, uint8_t Configuration)
{
	return static_cast<UsbDeviceClass *>(s_CoreObject[Index])->
		SelectConfig(Configuration);
}

static void UsbCoreUnconfigureClasses(void)
{
	if (s_Configuration == 0)
	{
		return;
	}

	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		(void)UsbCoreSelectConfig(i, 0);
	}
}

static bool UsbCoreApplyConfiguration(uint8_t Configuration)
{
	uint16_t descLen = 0;
	const uint8_t *pConfigDesc = nullptr;

	if (Configuration != 0)
	{
		pConfigDesc = UsbCoreGetConfigByValue(Configuration, &descLen);
		if (pConfigDesc == nullptr || descLen < USBD_CORE_CONFIG_DESC_LEN)
		{
			return false;
		}
	}

	UsbCoreUnconfigureClasses();
	UsbCtrlrEpCloseAll(s_UsbDevNo);
	s_Configuration = 0;
	s_NumInterfaces = 0;
	UsbCoreClearEndpointState();

	if (Configuration == 0)
	{
		return true;
	}

	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		if (!UsbCoreSelectConfig(i, Configuration))
		{
			for (int n = 0; n <= i; n++)
			{
				(void)UsbCoreSelectConfig(n, 0);
			}
			UsbCtrlrEpCloseAll(s_UsbDevNo);
			return false;
		}
	}

	s_Configuration = Configuration;
	s_NumInterfaces = pConfigDesc[4];

	return true;
}

static void UsbCoreSetEndpointHalt(uint8_t EpAddr, bool Halt)
{
	const uint16_t bit = (uint16_t)(1U << USB_ENDPADDR_NUM(EpAddr));
	uint16_t *pMask = USB_ENDPADDR_IS_IN(EpAddr) ? &s_HaltIn : &s_HaltOut;

	if (Halt)
	{
		UsbCtrlrEpStall(s_UsbDevNo, EpAddr);
		*pMask |= bit;
	}
	else
	{
		UsbCtrlrEpClearStall(s_UsbDevNo, EpAddr);
		*pMask &= (uint16_t)~bit;
	}
}

static bool UsbCoreEndpointHalted(uint8_t EpAddr)
{
	const uint16_t bit = (uint16_t)(1U << USB_ENDPADDR_NUM(EpAddr));
	const uint16_t mask = USB_ENDPADDR_IS_IN(EpAddr) ? s_HaltIn : s_HaltOut;

	return (mask & bit) != 0;
}

static bool UsbCoreHandleGetDescriptor(void)
{
	if (!UsbCoreDirIn(&s_Setup) ||
		UsbCoreRecipient(&s_Setup) != USB_REQTYPE_DEVICE)
	{
		return false;
	}

	const uint8_t type = (uint8_t)(s_Setup.wValue >> 8);
	const uint8_t index = (uint8_t)s_Setup.wValue;

	if (type != USB_DESCTYPE_STRING && s_Setup.wIndex != 0U)
	{
		return false;
	}

	uint16_t len;
	const uint8_t *pDesc;
	if (type == USB_DESCTYPE_CONFIGURATION || type == USB_DESCTYPE_OSC)
	{
		pDesc = UsbCoreGetConfigDescriptor(type, index, &len);
	}
	else
	{
		pDesc = UsbCoreGetDescriptor(type, index, s_Setup.wIndex, &len);
	}

	if (pDesc == nullptr)
	{
		return false;
	}

	if (type == USB_DESCTYPE_DEVICE && len >= 8U &&
		UsbCoreValidEp0Mps(pDesc[7]))
	{
		// Keep the control-transfer termination calculation aligned with the
		// descriptor advertised to the host.
		s_CoreCfg.Ep0Mps = pDesc[7];
	}

	return UsbCoreStartIn(pDesc, len);
}

static bool UsbCoreHandleGetStatus(void)
{
	if (!UsbCoreDirIn(&s_Setup) || s_Setup.wValue != 0 ||
		s_Setup.wLength != 2)
	{
		return false;
	}

	uint16_t status = 0;
	const uint8_t recipient = UsbCoreRecipient(&s_Setup);

	switch (recipient)
	{
		case USB_REQTYPE_DEVICE:
			if (s_Setup.wIndex != 0)
			{
				return false;
			}
			if (UsbCoreConfigSelfPowered())
			{
				status |= USB_DEVSTATUS_SELF_POWERED;
			}
			if (s_RemoteWakeup)
			{
				status |= USB_DEVSTATUS_REMOTE_WAKEUP;
			}
			break;

		case USB_REQTYPE_INTERFACE:
			if (s_Configuration == 0 || s_Setup.wIndex > 0xFFU ||
				!UsbCoreInterfaceExists((uint8_t)s_Setup.wIndex))
			{
				return false;
			}
			break;

		case USB_REQTYPE_ENDPOINT:
		{
			const uint8_t epAddr = (uint8_t)s_Setup.wIndex;
			if ((s_Setup.wIndex & 0xFF00U) != 0 ||
				(epAddr & 0x70U) != 0U ||
				(USB_ENDPADDR_NUM(epAddr) != 0U && s_Configuration == 0U) ||
				!UsbCoreEndpointExists(epAddr))
			{
				return false;
			}
			if (UsbCoreEndpointHalted(epAddr))
			{
				status = USB_ENDPSTATUS_HALT;
			}
			break;
		}

		default:
			return false;
	}

	s_CtrlReply[0] = (uint8_t)status;
	s_CtrlReply[1] = (uint8_t)(status >> 8);
	return UsbCoreStartIn(s_CtrlReply, 2);
}

static bool UsbCoreHandleFeature(bool Set)
{
	if (UsbCoreDirIn(&s_Setup) || s_Setup.wLength != 0)
	{
		return false;
	}

	const uint8_t recipient = UsbCoreRecipient(&s_Setup);

	if (recipient == USB_REQTYPE_DEVICE &&
		s_Setup.wValue == USB_FEATSEL_DEVICE_REMOTE_WAKEUP &&
		s_Setup.wIndex == 0 && s_Configuration != 0 &&
		UsbCoreConfigRemoteWakeupCapable())
	{
		s_RemoteWakeup = Set;
		return UsbCoreStartStatus();
	}

	if (recipient == USB_REQTYPE_ENDPOINT &&
		s_Setup.wValue == USB_FEATSEL_ENDPOINT_HALT &&
		(s_Setup.wIndex & 0xFF00U) == 0)
	{
		const uint8_t epAddr = (uint8_t)s_Setup.wIndex;

		if ((epAddr & 0x70U) != 0U || USB_ENDPADDR_NUM(epAddr) == 0 ||
			s_Configuration == 0 || !UsbCoreEndpointExists(epAddr))
		{
			return false;
		}

		UsbCoreSetEndpointHalt(epAddr, Set);
		return UsbCoreStartStatus();
	}

	return false;
}

static bool UsbCoreHandleSetConfiguration(void)
{
	if (UsbCoreDirIn(&s_Setup) ||
		UsbCoreRecipient(&s_Setup) != USB_REQTYPE_DEVICE ||
		s_Setup.wIndex != 0 || s_Setup.wLength != 0 ||
		s_Setup.wValue > 0xFFU || s_Address == 0)
	{
		return false;
	}

	if (!UsbCoreApplyConfiguration((uint8_t)s_Setup.wValue))
	{
		return false;
	}

	return UsbCoreStartStatus();
}

static bool UsbCoreHandleGetInterface(void)
{
	if (!UsbCoreDirIn(&s_Setup) ||
		UsbCoreRecipient(&s_Setup) != USB_REQTYPE_INTERFACE ||
		s_Setup.wValue != 0 || s_Setup.wLength != 1 ||
		s_Setup.wIndex > 0xFFU || s_Configuration == 0)
	{
		return false;
	}

	const uint8_t interfaceNo = (uint8_t)s_Setup.wIndex;
	if (interfaceNo >= USB_CORE_INTRF_MAXCNT ||
		!UsbCoreInterfaceExists(interfaceNo))
	{
		return false;
	}

	s_CtrlReply[0] = s_Alternate[interfaceNo];
	return UsbCoreStartIn(s_CtrlReply, 1);
}

static bool UsbCoreHandleSetInterface(void)
{
	if (UsbCoreDirIn(&s_Setup) ||
		UsbCoreRecipient(&s_Setup) != USB_REQTYPE_INTERFACE ||
		s_Setup.wLength != 0 || s_Setup.wIndex > 0xFFU ||
		s_Setup.wValue > 0xFFU || s_Configuration == 0)
	{
		return false;
	}

	const uint8_t interfaceNo = (uint8_t)s_Setup.wIndex;
	const uint8_t alternate = (uint8_t)s_Setup.wValue;

	if (interfaceNo >= USB_CORE_INTRF_MAXCNT ||
		!UsbCoreInterfaceAlternateExists(interfaceNo, alternate))
	{
		return false;
	}

	const int cls = UsbCoreFindClass(interfaceNo);
	if (cls < 0)
	{
		return false;
	}

	const uint8_t oldAlternate = s_Alternate[interfaceNo];
	const bool selected =
		static_cast<UsbDeviceClass *>(s_CoreObject[cls])->
			SelectInterface(interfaceNo, alternate);
	if (!selected)
	{
		return false;
	}

	UsbCoreClearInterfaceHalt(interfaceNo, oldAlternate, alternate);
	s_Alternate[interfaceNo] = alternate;
	return UsbCoreStartStatus();
}

static bool UsbCoreHandleStandard(void)
{
	switch (s_Setup.bRequest)
	{
		case USB_REQ_GET_STATUS:
			return UsbCoreHandleGetStatus();

		case USB_REQ_CLEAR_FEATURE:
			return UsbCoreHandleFeature(false);

		case USB_REQ_SET_FEATURE:
			return UsbCoreHandleFeature(true);

		case USB_REQ_SET_ADDRESS:
			if (UsbCoreDirIn(&s_Setup) ||
				UsbCoreRecipient(&s_Setup) != USB_REQTYPE_DEVICE ||
				s_Setup.wIndex != 0 || s_Setup.wLength != 0 ||
				s_Setup.wValue > 127U || s_Configuration != 0)
			{
				return false;
			}
			s_PendingAddress = (uint8_t)s_Setup.wValue;
			s_AddressPending = true;
			UsbCtrlrSetAddress(s_UsbDevNo, s_PendingAddress);
			return UsbCoreStartStatus();

		case USB_REQ_GET_DESCRIPTOR:
			if (UsbCoreRecipient(&s_Setup) == USB_REQTYPE_DEVICE)
			{
				return UsbCoreHandleGetDescriptor();
			}
			return UsbCoreRecipient(&s_Setup) == USB_REQTYPE_INTERFACE &&
				UsbCoreHandleClassRequest();

		case USB_REQ_GET_CONFIGURATION:
			if (!UsbCoreDirIn(&s_Setup) ||
				UsbCoreRecipient(&s_Setup) != USB_REQTYPE_DEVICE ||
				s_Setup.wValue != 0 || s_Setup.wIndex != 0 ||
				s_Setup.wLength != 1)
			{
				return false;
			}
			s_CtrlReply[0] = s_Configuration;
			return UsbCoreStartIn(s_CtrlReply, 1);

		case USB_REQ_SET_CONFIGURATION:
			return UsbCoreHandleSetConfiguration();

		case USB_REQ_GET_INTERFACE:
			return UsbCoreHandleGetInterface();

		case USB_REQ_SET_INTERFACE:
			return UsbCoreHandleSetInterface();

		default:
			return false;
	}
}

static bool UsbCoreCallClassSetup(int Index)
{
	uint8_t *pData = nullptr;
	uint16_t len = 0;
	const bool handled =
		static_cast<UsbDeviceClass *>(s_CoreObject[Index])->
			Control(&s_Setup, USB_CTRL_SETUP, &pData, &len);
	if (!handled)
	{
		return false;
	}

	s_ActiveClass = Index;

	if (s_Setup.wLength == 0)
	{
		s_CtrlData = pData;
		s_CtrlDataLen = 0;
		s_CtrlActual = 0;
		return UsbCoreStartStatus();
	}

	if (UsbCoreDirIn(&s_Setup))
	{
		return UsbCoreStartIn(pData, len);
	}

	return UsbCoreStartOut(pData, len);
}

static bool UsbCoreHandleClassRequest(void)
{
	const uint8_t recipient = UsbCoreRecipient(&s_Setup);

	if (recipient == USB_REQTYPE_INTERFACE)
	{
		if (s_Configuration == 0 || s_Setup.wIndex > 0xFFU)
		{
			return false;
		}

		const uint8_t interfaceNo = (uint8_t)s_Setup.wIndex;
		if (interfaceNo >= USB_CORE_INTRF_MAXCNT ||
			!UsbCoreInterfaceExists(interfaceNo))
		{
			return false;
		}

		const int cls = UsbCoreFindClass(interfaceNo);
		return cls >= 0 && UsbCoreCallClassSetup(cls);
	}

	if (recipient == USB_REQTYPE_ENDPOINT)
	{
		if (s_Configuration == 0 || (s_Setup.wIndex & 0xFF00U) != 0)
		{
			return false;
		}

		const uint8_t epAddr = (uint8_t)s_Setup.wIndex;
		if ((epAddr & 0x70U) != 0U || USB_ENDPADDR_NUM(epAddr) == 0U ||
			!UsbCoreEndpointExists(epAddr))
		{
			return false;
		}

		const int cls = UsbCoreFindEndpointClass(epAddr);
		return cls >= 0 && UsbCoreCallClassSetup(cls);
	}

	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		if (UsbCoreCallClassSetup(i))
		{
			return true;
		}
	}

	return false;
}

static void UsbCoreHandleSetup(const UsbSetupData_t *pSetup)
{
	if (pSetup == nullptr)
	{
		return;
	}

	UsbCoreAbortControl();
	memcpy(&s_Setup, pSetup, sizeof(s_Setup));

	const uint8_t type = s_Setup.bmRequestType & USB_REQTYPE_MASK_TYPE;
	bool handled = false;

	if (type == USB_REQTYPE_STANDARD)
	{
		handled = UsbCoreHandleStandard();
	}
	else if (type == USB_REQTYPE_CLASS || type == USB_REQTYPE_VEND)
	{
		handled = UsbCoreHandleClassRequest();
	}

	if (!handled)
	{
		UsbCoreStallControl();
	}
}

static void UsbCoreHandleCtrlXfer(const UsbCtrlrXferEvt_t *pXfer)
{
	if (pXfer == nullptr || pXfer->Result != USB_CTRLR_XFER_SUCCESS)
	{
		UsbCoreStallControl();
		return;
	}

	switch (s_CtrlState)
	{
		case USB_CTRL_DATA_IN:
			s_CtrlActual = pXfer->Length;
			if (!UsbCoreInvokeActive(USB_CTRL_DATA, s_CtrlActual))
			{
				UsbCoreStallControl();
				return;
			}

			if (s_CtrlNeedZlp)
			{
				s_CtrlNeedZlp = false;
				s_CtrlState = USB_CTRL_DATA_IN_ZLP;
				if (!UsbCtrlrEp0Xfer(s_UsbDevNo, USB_ENDPADDR_DIR_IN, nullptr, 0))
				{
					UsbCoreStallControl();
				}
				return;
			}

			(void)UsbCoreStartStatus();
			break;

		case USB_CTRL_DATA_IN_ZLP:
			(void)UsbCoreStartStatus();
			break;

		case USB_CTRL_DATA_OUT:
			s_CtrlActual = pXfer->Length;
			if (!UsbCoreInvokeActive(USB_CTRL_DATA, s_CtrlActual))
			{
				UsbCoreStallControl();
				return;
			}
			(void)UsbCoreStartStatus();
			break;

		case USB_CTRL_STATUS_IN:
		case USB_CTRL_STATUS_OUT:
			if (s_AddressPending)
			{
				s_Address = s_PendingAddress;
			}
			(void)UsbCoreInvokeActive(USB_CTRL_COMPLETE,
									   s_CtrlActual);
			UsbCoreResetControl();
			break;

		case USB_CTRL_IDLE:
		default:
			break;
	}
}

static void UsbCoreNotifyReset(void)
{
	UsbCoreUnconfigureClasses();

	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		s_CoreObject[i]->Reset();
	}
}

static void UsbCoreResetDeviceState(bool NotifyClasses)
{
	UsbCoreAbortControl();

	if (NotifyClasses)
	{
		UsbCoreNotifyReset();
	}

	s_Address = 0;
	s_Configuration = 0;
	s_NumInterfaces = 0;
	s_CoreSuspended = false;
	UsbCoreClearEndpointState();
}

static void UsbCoreCtrlrEvent(int, const UsbCtrlrEvt_t *pEvt, void *)
{
	if (pEvt == nullptr)
	{
		return;
	}

	switch (pEvt->Type)
	{
		case USB_CTRLR_EVT_RESET:
			UsbCoreResetDeviceState(true);
			break;

		case USB_CTRLR_EVT_SETUP:
			UsbCoreHandleSetup(&pEvt->Setup);
			break;

		case USB_CTRLR_EVT_XFER_CMPL:
			if (USB_ENDPADDR_NUM(pEvt->Xfer.EpAddr) == 0)
			{
				UsbCoreHandleCtrlXfer(&pEvt->Xfer);
			}
			break;

		case USB_CTRLR_EVT_SUSPEND:
			s_CoreSuspended = true;
			break;

		case USB_CTRLR_EVT_RESUME:
			s_CoreSuspended = false;
			break;

		case USB_CTRLR_EVT_ADDRESS:
			s_Address = pEvt->Address;
			break;

		default:
			break;
	}
}

static bool UsbCoreInit(const UsbCoreCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	memcpy(&s_CoreCfg, pCfg, sizeof(s_CoreCfg));
	if (s_CoreCfg.Ep0Mps == 0)
	{
		s_CoreCfg.Ep0Mps = USBD_CORE_EP0_MPS_DEFAULT;
	}
	else if (!UsbCoreValidEp0Mps(s_CoreCfg.Ep0Mps))
	{
		return false;
	}

	memset(s_CoreObject, 0, sizeof(s_CoreObject));
	// Minus one is no owner. Zero would claim class zero owns every
	// endpoint, so this cannot be left to static initialization.
	memset(s_CoreEpClass, -1, sizeof(s_CoreEpClass));
	s_CoreObjectCnt = 0;
	s_CoreInitialized = false;
	s_CoreStarted = false;
	UsbCoreResetDeviceState(false);

	s_CoreInitialized = true;
	return true;
}

static bool UsbCoreRegisterObject(UsbDeviceClass *pClass,
								  uint8_t FirstInterface,
								  uint8_t InterfaceCount,
								  uint16_t EpInMask,
								  uint16_t EpOutMask)
{
	if (!s_CoreInitialized || pClass == nullptr ||
		s_CoreStarted ||
		s_CoreObjectCnt >= USB_CORE_CLASS_MAXCNT)
	{
		return false;
	}

	if (((EpInMask | EpOutMask) & 1U) != 0)
	{
		return false;
	}

	if (InterfaceCount != 0)
	{
		const uint16_t last = (uint16_t)FirstInterface +
			(uint16_t)InterfaceCount;
		if (last > USB_CORE_INTRF_MAXCNT)
		{
			return false;
		}

		for (int i = 0; i < s_CoreObjectCnt; i++)
		{
			const UsbDeviceClass *pRegistered =
				static_cast<const UsbDeviceClass *>(s_CoreObject[i]);
			const uint16_t firstA = FirstInterface;
			const uint16_t lastA = last;
			const uint16_t firstB = pRegistered->FirstInterface();
			const uint16_t lastB =
				firstB + pRegistered->InterfaceCount();

			if (pRegistered->InterfaceCount() != 0 &&
				firstA < lastB && firstB < lastA)
			{
				return false;
			}
		}
	}

	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		const UsbDeviceClass *pRegistered =
			static_cast<const UsbDeviceClass *>(s_CoreObject[i]);
		if (pRegistered == pClass ||
			(EpInMask & pRegistered->EpInMask()) != 0 ||
			(EpOutMask & pRegistered->EpOutMask()) != 0)
		{
			return false;
		}
	}

	s_CoreObject[s_CoreObjectCnt] = pClass;

	for (int ep = 1; ep < 16; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if ((EpInMask & bit) != 0)
		{
			s_CoreEpClass[1][ep] = (int8_t)s_CoreObjectCnt;
		}
		if ((EpOutMask & bit) != 0)
		{
			s_CoreEpClass[0][ep] = (int8_t)s_CoreObjectCnt;
		}
	}

	s_CoreObjectCnt++;
	return true;
}

static void UsbCoreStart(void)
{
	if (!s_CoreInitialized || s_CoreStarted)
	{
		return;
	}

	UsbCtrlrIntEnable(s_UsbDevNo);
	UsbCtrlrConnect(s_UsbDevNo);
	s_CoreStarted = true;
}

static void UsbCoreStop(void)
{
	if (!s_CoreStarted)
	{
		return;
	}

	UsbCtrlrDisconnect(s_UsbDevNo);
	UsbCtrlrIntDisable(s_UsbDevNo);
	UsbCtrlrEpCloseAll(s_UsbDevNo);
	UsbCoreResetDeviceState(true);
	s_CoreStarted = false;
}

static bool UsbCoreRemoteWakeup(void)
{
	if (!s_CoreStarted || !s_CoreSuspended || !s_RemoteWakeup)
	{
		return false;
	}

	UsbCtrlrRemoteWakeup(s_UsbDevNo);
	return true;
}

static bool UsbCoreConfigured(void)
{
	return s_CoreStarted && s_Configuration != 0;
}

static bool UsbCoreSuspended(void)
{
	return s_CoreStarted && s_CoreSuspended;
}

static uint8_t UsbCoreAddress(void)
{
	return s_Address;
}

static uint8_t UsbCoreConfiguration(void)
{
	return s_Configuration;
}

static uint8_t UsbCoreAlternate(uint8_t InterfaceNo)
{
	return InterfaceNo < USB_CORE_INTRF_MAXCNT ?
		s_Alternate[InterfaceNo] : 0;
}

static bool UsbCoreRemoteWakeupEnabled(void)
{
	return s_RemoteWakeup;
}

static bool UsbCoreValidateDescriptorFragment(const UsbDeviceClass *pClass,
											 const uint8_t *pDesc,
											 uint16_t Length)
{
	if (pClass == nullptr || pDesc == nullptr || Length == 0U ||
		pClass->InterfaceCount() == 0U)
	{
		return false;
	}

	const uint8_t first = pClass->FirstInterface();
	const uint8_t last = (uint8_t)(first + pClass->InterfaceCount());
	uint16_t offset = 0U;
	bool hasInterface = false;
	while (offset < Length)
	{
		if ((uint16_t)(Length - offset) < 2U)
		{
			return false;
		}

		const uint8_t descLength = pDesc[offset];
		const uint8_t descType = pDesc[offset + 1U];
		if (descLength < 2U || (uint16_t)(offset + descLength) > Length ||
			descType == USB_DESCTYPE_DEVICE ||
			descType == USB_DESCTYPE_CONFIGURATION ||
			descType == USB_DESCTYPE_STRING ||
			descType == USB_DESCTYPE_DEVICE_QUALIFIER ||
			descType == USB_DESCTYPE_OSC)
		{
			return false;
		}

		if (descType == USB_DESCTYPE_INTERFACE)
		{
			if (descLength < sizeof(UsbIntrfDesc_t))
			{
				return false;
			}
			const uint8_t interfaceNo = pDesc[offset + 2U];
			if (interfaceNo < first || interfaceNo >= last)
			{
				return false;
			}
			hasInterface = true;
		}
		else if (descType == USB_DESCTYPE_ENDPOINT)
		{
			if (descLength < sizeof(UsbEndPointDesc_t))
			{
				return false;
			}
			const uint8_t epAddr = pDesc[offset + 2U];
			const uint8_t epNo = USB_ENDPADDR_NUM(epAddr);
			const uint16_t mask = USB_ENDPADDR_IS_IN(epAddr) ?
				pClass->EpInMask() : pClass->EpOutMask();
			if (epNo == 0U || epNo >= 16U ||
				(mask & (uint16_t)(1U << epNo)) == 0U)
			{
				return false;
			}
		}

		offset = (uint16_t)(offset + descLength);
	}

	return hasInterface && offset == Length;
}

//
// Application entry points.
//

#define USB_SERIAL_MAXLEN			33	//!< 32 hexadecimal characters and a terminator

static UsbCfg_t s_UsbDevCfg;
static char s_UsbDevSerial[USB_SERIAL_MAXLEN];
static bool s_UsbDevInitialized;
static bool s_UsbDevStarted;

static bool UsbDevInit(const UsbCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->Vid == 0 || pCfg->Pid == 0)
	{
		return false;
	}

	s_UsbDevInitialized = false;
	s_UsbDevStarted = false;
	memcpy(&s_UsbDevCfg, pCfg, sizeof(s_UsbDevCfg));

	if (s_UsbDevCfg.MaxPower == 0)
	{
		s_UsbDevCfg.MaxPower = 100;
	}

	if (s_UsbDevCfg.pSerial != nullptr)
	{
		strncpy(s_UsbDevSerial, s_UsbDevCfg.pSerial,
				sizeof(s_UsbDevSerial) - 1U);
		s_UsbDevSerial[sizeof(s_UsbDevSerial) - 1U] = '\0';
	}
	else
	{
		UsbCtrlrGetSerial(s_UsbDevNo, s_UsbDevSerial,
						  sizeof(s_UsbDevSerial));
	}
	s_UsbDevCfg.pSerial = s_UsbDevSerial;

	UsbCtrlrCfg_t ctrlrCfg = {};
	ctrlrCfg.IntPrio = s_UsbDevCfg.IntPrio;
	ctrlrCfg.bLowPowerSuspend = s_UsbDevCfg.bLowPowerSuspend;
	ctrlrCfg.EvtHandler = UsbCoreCtrlrEvent;
	ctrlrCfg.pContext = nullptr;

	if (!UsbCtrlrInit(s_UsbDevNo, &ctrlrCfg))
	{
		return false;
	}

	UsbCoreCfg_t coreCfg = {};
	coreCfg.Ep0Mps = USB_PKT_MAXLEN(s_UsbDevNo, CONTROL);

	if (!UsbCoreInit(&coreCfg))
	{
		return false;
	}

	s_UsbDevInitialized = true;
	return true;
}


static bool UsbDevEnable(void)
{
	if (!s_UsbDevInitialized)
	{
		return false;
	}

	if (s_UsbDevStarted)
	{
		return true;
	}

	uint16_t descriptorLength = 0U;
	if (UsbGetDescriptor(s_UsbDevNo, USB_DESCTYPE_CONFIGURATION, 0U, 0U,
		UsbGetSpeed(s_UsbDevNo), &descriptorLength) == nullptr)
	{
		return false;
	}

	// One call. Power, clock and PHY come up and endpoint zero is prepared.
	if (!UsbCtrlrStart(s_UsbDevNo))
	{
		return false;
	}

	UsbCoreStart();
	s_UsbDevStarted = true;

	return true;
}

static void UsbDevDisable(void)
{
	if (!s_UsbDevStarted)
	{
		return;
	}

	UsbCoreStop();
	UsbCtrlrStop(s_UsbDevNo);
	s_UsbDevStarted = false;
}

static void UsbDevProcess(void)
{
	if (!s_UsbDevInitialized)
	{
		return;
	}

	UsbCtrlrProcess(s_UsbDevNo);

	if (!s_UsbDevStarted)
	{
		//
		// Retry on the level, not only on the attach edge. A board already on
		// a cable at reset never produces an edge, so an Enable that failed
		// during start up would be the only attempt ever made and the port
		// would stay down with nothing to show for it. Enable is cheap while
		// there is no bus power, because UsbCtrlrStart answers false immediately.
		//

		if (UsbCtrlrVbusDetected(s_UsbDevNo))
		{
			(void)UsbDevEnable();
		}

		if (!s_UsbDevStarted)
		{
			return;
		}
	}

	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		s_CoreObject[i]->Process();
	}
}

static bool UsbDevMounted(void)
{
	return s_UsbDevStarted && UsbCoreConfigured();
}

static bool UsbDevSuspended(void)
{
	return s_UsbDevStarted && UsbCoreSuspended();
}

static const UsbCfg_t *UsbDevGetCfg(void)
{
	return s_UsbDevInitialized ? &s_UsbDevCfg : nullptr;
}

static const char *UsbDevGetSerial(void)
{
	return s_UsbDevSerial;
}

//
// Entry points declared in usb.h. UsbInit is the only initialization an
// application makes; the controller, the protocol engine and the identity all
// come up inside it.
//

bool UsbInit(const UsbCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->DevNo < 0 || pCfg->DevNo >= USB_CTRLR_CNT)
	{
		return false;
	}

	// Host and OTG need a dual role controller. The current targets are device
	// only, so reject the other roles rather than pretend to support them.
	if (pCfg->Mode != USB_MODE_DEVICE)
	{
		return false;
	}

	s_UsbDevNo = pCfg->DevNo;
	s_UsbVbusLast = false;

	return UsbDevInit(pCfg);
}

bool UsbClassRegister(int DevNo, UsbDeviceClass *pClass,
					  uint8_t FirstInterface, uint8_t InterfaceCount,
					  uint16_t EpInMask, uint16_t EpOutMask)
{
	if (DevNo != s_UsbDevNo ||
		!UsbCoreRegisterObject(pClass, FirstInterface, InterfaceCount,
			EpInMask, EpOutMask))
	{
		return false;
	}

	pClass->vFirstInterface = FirstInterface;
	pClass->vInterfaceCount = InterfaceCount;
	pClass->vEpInMask = EpInMask;
	pClass->vEpOutMask = EpOutMask;
	pClass->vFsDescriptor = nullptr;
	pClass->vHsDescriptor = nullptr;
	pClass->vFsDescriptorLength = 0U;
	pClass->vHsDescriptorLength = 0U;
	return true;
}

bool UsbDescriptorRegister(int DevNo, UsbDeviceClass *pClass,
						   const void *pFsDescriptor,
						   uint16_t FsDescriptorLength,
						   const void *pHsDescriptor,
						   uint16_t HsDescriptorLength)
{
	if (DevNo != s_UsbDevNo || pClass == nullptr || s_CoreStarted ||
		!UsbCoreValidateDescriptorFragment(pClass,
			static_cast<const uint8_t *>(pFsDescriptor), FsDescriptorLength) ||
		(USB_HIGHSPEED_CAPABLE(DevNo) &&
		 !UsbCoreValidateDescriptorFragment(pClass,
			static_cast<const uint8_t *>(pHsDescriptor), HsDescriptorLength)))
	{
		return false;
	}

	bool registered = false;
	for (int i = 0; i < s_CoreObjectCnt; i++)
	{
		if (s_CoreObject[i] == pClass)
		{
			registered = true;
			break;
		}
	}
	if (!registered || pClass->vFsDescriptor != nullptr ||
		pClass->vHsDescriptor != nullptr)
	{
		return false;
	}

	pClass->vFsDescriptor = static_cast<const uint8_t *>(pFsDescriptor);
	pClass->vFsDescriptorLength = FsDescriptorLength;
	if (USB_HIGHSPEED_CAPABLE(DevNo))
	{
		pClass->vHsDescriptor = static_cast<const uint8_t *>(pHsDescriptor);
		pClass->vHsDescriptorLength = HsDescriptorLength;
	}
	else
	{
		pClass->vHsDescriptor = pClass->vFsDescriptor;
		pClass->vHsDescriptorLength = pClass->vFsDescriptorLength;
	}
	return true;
}

const uint8_t *UsbGetDescriptor(int DevNo, uint8_t Type, uint8_t Index,
								 uint16_t LangId, UsbSpeed_t Speed,
								 uint16_t *pLength)
{
	if (pLength == nullptr || DevNo != s_UsbDevNo ||
		UsbGetCfg(DevNo) == nullptr)
	{
		return nullptr;
	}

	*pLength = 0U;
	switch (Type)
	{
		case USB_DESCTYPE_DEVICE:
			return UsbDescDevice(DevNo, Index, pLength);
		case USB_DESCTYPE_CONFIGURATION:
			return UsbDescConfiguration(DevNo, Index, Speed, false, pLength);
		case USB_DESCTYPE_STRING:
			return UsbDescString(DevNo, Index, LangId, pLength);
		case USB_DESCTYPE_DEVICE_QUALIFIER:
			return UsbDescQualifier(DevNo, Index, pLength);
		case USB_DESCTYPE_OSC:
			return UsbDescConfiguration(DevNo, Index, Speed, true, pLength);
		default:
			return nullptr;
	}
}

bool UsbEnable(int DevNo)
{
	return DevNo == s_UsbDevNo && UsbDevEnable();
}

void UsbDisable(int DevNo)
{
	if (DevNo == s_UsbDevNo)
	{
		UsbDevDisable();
	}
}

void UsbProcess(int DevNo)
{
	if (DevNo != s_UsbDevNo)
	{
		return;
	}

	UsbDevProcess();

	// Cable events are derived here rather than reported by the port, which
	// only exposes the level. UsbDevProcess has already polled the controller,
	// so this edge is against a fresh reading.
	const bool vbus = UsbCtrlrVbusDetected(s_UsbDevNo);

	if (vbus != s_UsbVbusLast)
	{
		s_UsbVbusLast = vbus;

		if (s_UsbDevCfg.EvtHandler != nullptr)
		{
			s_UsbDevCfg.EvtHandler(s_UsbDevNo,
								   vbus ? USB_EVT_ATTACHED : USB_EVT_DETACHED);
		}
	}
}

UsbSpeed_t UsbGetSpeed(int DevNo)
{
	return UsbCtrlrHighSpeed(DevNo) ? USB_SPEED_HIGH : USB_SPEED_FULL;
}

bool UsbConfigured(int DevNo)
{
	return DevNo == s_UsbDevNo && UsbDevMounted();
}

bool UsbSuspended(int DevNo)
{
	return DevNo == s_UsbDevNo && UsbDevSuspended();
}

bool UsbRemoteWakeupEnabled(int DevNo)
{
	return DevNo == s_UsbDevNo && UsbCoreRemoteWakeupEnabled();
}

bool UsbRemoteWakeup(int DevNo)
{
	return DevNo == s_UsbDevNo && UsbCoreRemoteWakeup();
}

const UsbCfg_t *UsbGetCfg(int DevNo)
{
	return DevNo == s_UsbDevNo ? UsbDevGetCfg() : nullptr;
}

const char *UsbGetSerial(int DevNo)
{
	return DevNo == s_UsbDevNo ? UsbDevGetSerial() : nullptr;
}

uint8_t UsbGetAddress(int DevNo)
{
	return DevNo == s_UsbDevNo ? UsbCoreAddress() : 0;
}

uint8_t UsbGetConfiguration(int DevNo)
{
	return DevNo == s_UsbDevNo ? UsbCoreConfiguration() : 0;
}

uint8_t UsbGetAlternate(int DevNo, uint8_t InterfaceNo)
{
	return DevNo == s_UsbDevNo ? UsbCoreAlternate(InterfaceNo) : 0;
}

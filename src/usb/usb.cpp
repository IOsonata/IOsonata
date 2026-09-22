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

#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "usb/usb.h"


#define USB_CORE_CLASS_MAXCNT \
	(USB_EPIN_CNT(0) > USB_EPOUT_CNT(0) ? \
	 USB_EPIN_CNT(0) : USB_EPOUT_CNT(0))
#define USB_CORE_INTRF_MAXCNT		16

#define USBD_CORE_EP0_MPS_DEFAULT		64U
#define USBD_CORE_DEVICE_DESC_LEN		((uint16_t)sizeof(UsbDevDesc_t))
#define USBD_CORE_CONFIG_DESC_LEN		((uint16_t)sizeof(UsbCfgDesc_t))
#define USB_CORE_STRING_DESC_MAXLEN		66U
#define USB_CORE_STR_MANUFACTURER		1U
#define USB_CORE_STR_PRODUCT			2U
#define USB_CORE_STR_SERIAL			3U
#define USB_CORE_STR_FUNCTION			4U

#define USB_SERIAL_MAXLEN			33	//!< 32 hexadecimal characters and a terminator

static_assert(USB_CORE_CLASS_MAXCNT > 0 && USB_CORE_CLASS_MAXCNT <= 16,
	"USB endpoint count must fit the 16-bit endpoint ownership masks");

/// Chapter 9 settings. Built by UsbInit from UsbCfg_t and usb_ctrlr.h, never
/// supplied by an application, which is why it is no longer in a header.
typedef struct __Usb_Core_Config {
	uint8_t Ep0Mps;					//!< EP0 max packet size
} UsbCoreCfg_t;

typedef enum __Usbd_Core_Ctrl_State {
	USB_CTRL_IDLE,
	USB_CTRL_DATA_IN,
	USB_CTRL_DATA_IN_ZLP,
	USB_CTRL_DATA_OUT,
	USB_CTRL_STATUS_IN,
	USB_CTRL_STATUS_OUT,
} UsbCoreCtrlState_t;

static void UsbCoreAbortControl(void);
static bool UsbCoreHandleClassRequest(void);

// One state block: every function then addresses its fields from a single
// literal base instead of one literal per file-scope object.
static struct
{
	int DevNo;					//!< Controller this instance drives
	bool VbusLast;				//!< Bus power at the previous UsbProcess pass
	bool Initialized;
	bool Started;
	bool Suspended;
	bool RemoteWakeup;
	uint8_t Address;
	uint8_t PendingAddress;
	bool AddressPending;
	uint8_t Configuration;
	uint8_t NumInterfaces;
	uint16_t HaltIn;
	uint16_t HaltOut;
	UsbCoreCtrlState_t CtrlState;
	int ActiveClass;
	uint8_t *CtrlData;
	uint16_t CtrlDataLen;
	uint16_t CtrlActual;			//!< IN bytes accepted; OUT bytes received
	bool CtrlNeedZlp;
	uint8_t CtrlReply[2];
	UsbCoreCfg_t Cfg;
	int ObjectCnt;
	UsbClass *Object[USB_CORE_CLASS_MAXCNT];
	// Endpoint to class index, [0] OUT and [1] IN. Ownership masks are
	// fixed once a class registers and may not overlap. They route
	// endpoint-recipient requests; data endpoint events go directly to
	// their registered callbacks. Minus one means no class owns that
	// endpoint.
	int8_t EpClass[2][16];
	uint8_t Alternate[USB_CORE_INTRF_MAXCNT];
	UsbSetupData_t Setup;
	UsbDevDesc_t DeviceDesc;
	UsbDevQualDesc_t QualifierDesc;
	uint8_t ConfigDesc[USB_CONFIG_DESC_MAXLEN];
	uint8_t StringDesc[USB_CORE_STRING_DESC_MAXLEN];
} s_Core;

static UsbCfg_t s_UsbDevCfg;
static char s_UsbDevSerial[USB_SERIAL_MAXLEN];
static bool s_UsbDevInitialized;
static bool s_UsbDevStarted;

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
	return UsbGetDescriptor(s_Core.DevNo, Type, Index, LangId,
		UsbGetSpeed(s_Core.DevNo), pLength);
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

	memset(&s_Core.DeviceDesc, 0, sizeof(s_Core.DeviceDesc));
	s_Core.DeviceDesc.bLength = sizeof(s_Core.DeviceDesc);
	s_Core.DeviceDesc.bDescriptorType = USB_DESCTYPE_DEVICE;
	s_Core.DeviceDesc.bcdUSB = 0x0200U;
	s_Core.DeviceDesc.bDeviceClass = pCfg->DeviceClass;
	s_Core.DeviceDesc.bDeviceSubClass = pCfg->DeviceSubClass;
	s_Core.DeviceDesc.bDeviceProtocol = pCfg->DeviceProtocol;
	s_Core.DeviceDesc.bMaxPacketSize = USB_CTRLR_PKT_LEN_MAX(DevNo, CONTROL);
	s_Core.DeviceDesc.idVendor = pCfg->Vid;
	s_Core.DeviceDesc.idProduct = pCfg->Pid;
	s_Core.DeviceDesc.bcdDevice = pCfg->DevVer;
	s_Core.DeviceDesc.iManufacturer = pCfg->pManufacturer != nullptr ?
		USB_CORE_STR_MANUFACTURER : 0U;
	s_Core.DeviceDesc.iProduct = pCfg->pProduct != nullptr ?
		USB_CORE_STR_PRODUCT : 0U;
	s_Core.DeviceDesc.iSerialNumber = UsbGetSerial(DevNo) != nullptr ?
		USB_CORE_STR_SERIAL : 0U;
	s_Core.DeviceDesc.bNumConfigurations = 1U;

	*pLength = sizeof(s_Core.DeviceDesc);
	return reinterpret_cast<const uint8_t *>(&s_Core.DeviceDesc);
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

	memset(&s_Core.QualifierDesc, 0, sizeof(s_Core.QualifierDesc));
	s_Core.QualifierDesc.bLength = sizeof(s_Core.QualifierDesc);
	s_Core.QualifierDesc.bDescriptorType = USB_DESCTYPE_DEVICE_QUALIFIER;
	s_Core.QualifierDesc.bcdUSB = 0x0200U;
	s_Core.QualifierDesc.bDeviceClass = pCfg->DeviceClass;
	s_Core.QualifierDesc.bDeviceSubClass = pCfg->DeviceSubClass;
	s_Core.QualifierDesc.bDeviceProtocol = pCfg->DeviceProtocol;
	s_Core.QualifierDesc.bMaxPacketSize0 = USB_CTRLR_PKT_LEN_MAX(DevNo, CONTROL);
	s_Core.QualifierDesc.bNumConfigurations = 1U;

	*pLength = sizeof(s_Core.QualifierDesc);
	return reinterpret_cast<const uint8_t *>(&s_Core.QualifierDesc);
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
		s_Core.StringDesc[0] = 4U;
		s_Core.StringDesc[1] = USB_DESCTYPE_STRING;
		s_Core.StringDesc[2] = 0x09U;
		s_Core.StringDesc[3] = 0x04U;
		*pLength = 4U;
		return s_Core.StringDesc;
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
	const size_t maxLength = (sizeof(s_Core.StringDesc) - 2U) / 2U;
	if (length > maxLength)
	{
		length = maxLength;
	}

	s_Core.StringDesc[0] = (uint8_t)(2U + length * 2U);
	s_Core.StringDesc[1] = USB_DESCTYPE_STRING;
	for (size_t i = 0; i < length; i++)
	{
		s_Core.StringDesc[2U + i * 2U] = (uint8_t)pString[i];
		s_Core.StringDesc[3U + i * 2U] = 0U;
	}

	*pLength = s_Core.StringDesc[0];
	return s_Core.StringDesc;
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
	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		const UsbDeviceClass *pClass =
			static_cast<const UsbDeviceClass *>(s_Core.Object[i]);
		const uint16_t fragmentLength = pClass->DescriptorLength(Speed);
		const uint8_t *pFragment = pClass->Descriptor(Speed);
		if (pFragment == nullptr || fragmentLength == 0U ||
			(uint32_t)totalLength + fragmentLength > sizeof(s_Core.ConfigDesc))
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

	if (s_Core.ObjectCnt == 0 || interfaceCount == 0U)
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

	memcpy(s_Core.ConfigDesc, &config, sizeof(config));
	uint16_t offset = sizeof(config);
	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		const UsbDeviceClass *pClass =
			static_cast<const UsbDeviceClass *>(s_Core.Object[i]);
		const uint16_t fragmentLength = pClass->DescriptorLength(Speed);
		memcpy(&s_Core.ConfigDesc[offset], pClass->Descriptor(Speed),
			fragmentLength);
		offset = (uint16_t)(offset + fragmentLength);
	}

	*pLength = totalLength;
	return s_Core.ConfigDesc;
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
	if (s_Core.Configuration != 0)
	{
		return UsbCoreGetConfigByValue(s_Core.Configuration, pLength);
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

	s_Core.HaltIn &= (uint16_t)~(oldIn | newIn);
	s_Core.HaltOut &= (uint16_t)~(oldOut | newOut);
}

static bool UsbCoreEndpointExists(uint8_t EpNo, bool bIn)
{
	if (EpNo == 0U)
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
					s_Core.Alternate[interfaceNo] == alternate;
			}
			else
			{
				activeInterface = false;
			}
		}
		else if (activeInterface && type == USB_DESCTYPE_ENDPOINT &&
				 dlen >= sizeof(UsbEndPointDesc_t))
		{
			const uint8_t epAddr = pDesc[ofs + 2U];
			if (USB_ENDPADDR_NUM(epAddr) == EpNo &&
				(bool)USB_ENDPADDR_IS_IN(epAddr) == bIn)
			{
				return true;
			}
		}

		ofs = (uint16_t)(ofs + dlen);
	}

	return false;
}

static int UsbCoreFindClass(uint8_t InterfaceNo)
{
	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		const UsbDeviceClass *pClass =
			static_cast<const UsbDeviceClass *>(s_Core.Object[i]);
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

static int UsbCoreFindEndpointClass(uint8_t EpNo, bool bIn)
{
	if (EpNo == 0U || EpNo >= 16U)
	{
		return -1;
	}

	return s_Core.EpClass[bIn ? 1 : 0][EpNo];
}

static void UsbCoreResetControl(void)
{
	s_Core.CtrlState = USB_CTRL_IDLE;
	s_Core.ActiveClass = -1;
	s_Core.CtrlData = nullptr;
	s_Core.CtrlDataLen = 0;
	s_Core.CtrlActual = 0;
	s_Core.CtrlNeedZlp = false;
	s_Core.PendingAddress = 0;
	s_Core.AddressPending = false;
}

static void UsbCoreStallControl(void)
{
	UsbCoreAbortControl();
	UsbCtrlrEpStall(s_Core.DevNo, 0U, false);
}

static bool UsbCoreInvokeActive(UsbCtrlStage_t Stage,
								 uint16_t Length)
{
	uint8_t *pData = s_Core.CtrlData;
	uint16_t len = Length;
	if (s_Core.ActiveClass < 0 || s_Core.ActiveClass >= s_Core.ObjectCnt)
	{
		return true;
	}

	return static_cast<UsbDeviceClass *>(s_Core.Object[s_Core.ActiveClass])->
		Control(&s_Core.Setup, Stage, &pData, &len);
}

static void UsbCoreAbortControl(void)
{
	if (s_Core.ActiveClass >= 0 && s_Core.ActiveClass < s_Core.ObjectCnt)
	{
		(void)UsbCoreInvokeActive(USB_CTRL_ABORT, s_Core.CtrlActual);
	}

	UsbCoreResetControl();
}

static bool UsbCoreStartStatus(void)
{
	const uint8_t epAddr = UsbCoreDirIn(&s_Core.Setup) ?
		USB_ENDPADDR_DIR_OUT : USB_ENDPADDR_DIR_IN;

	s_Core.CtrlState = USB_ENDPADDR_IS_IN(epAddr) ?
		USB_CTRL_STATUS_IN : USB_CTRL_STATUS_OUT;

	if (!UsbCtrlrEp0Status(s_Core.DevNo, epAddr))
	{
		UsbCoreStallControl();
		return false;
	}

	return true;
}

// Publish the accepted offset before an interrupt can complete this chunk.
static bool UsbCoreSendIn(void)
{
	const uint32_t state = DisableInterrupt();
	const uint16_t remaining = s_Core.CtrlDataLen - s_Core.CtrlActual;
	uint8_t *pData = s_Core.CtrlData;
	if (pData != nullptr)
		pData += s_Core.CtrlActual;
	const int copied = UsbCtrlrEp0Send(s_Core.DevNo, pData, remaining);
	const bool accepted = copied > 0 || copied == remaining;
	if (accepted)
		s_Core.CtrlActual += copied;
	EnableInterrupt(state);
	return accepted;
}

static bool UsbCoreStartIn(const uint8_t *pData, uint16_t Available)
{
	if (s_Core.Setup.wLength == 0)
	{
		s_Core.CtrlData = const_cast<uint8_t *>(pData);
		s_Core.CtrlDataLen = 0;
		s_Core.CtrlActual = 0;
		return UsbCoreStartStatus();
	}

	const uint16_t sendLen = Available < s_Core.Setup.wLength ?
		Available : s_Core.Setup.wLength;

	if (sendLen > 0 && pData == nullptr)
	{
		return false;
	}

	s_Core.CtrlData = const_cast<uint8_t *>(pData);
	s_Core.CtrlDataLen = sendLen;
	s_Core.CtrlActual = 0;
	s_Core.CtrlNeedZlp =
		sendLen > 0 && sendLen < s_Core.Setup.wLength &&
		(sendLen % s_Core.Cfg.Ep0Mps) == 0;
	s_Core.CtrlState = USB_CTRL_DATA_IN;

	return UsbCoreSendIn();
}

static bool UsbCoreStartOut(uint8_t *pData, uint16_t Capacity)
{
	if (s_Core.Setup.wLength == 0)
	{
		s_Core.CtrlData = pData;
		s_Core.CtrlDataLen = 0;
		s_Core.CtrlActual = 0;
		return UsbCoreStartStatus();
	}

	if (pData == nullptr || Capacity < s_Core.Setup.wLength)
	{
		return false;
	}

	s_Core.CtrlData = pData;
	s_Core.CtrlDataLen = s_Core.Setup.wLength;
	s_Core.CtrlActual = 0;
	s_Core.CtrlState = USB_CTRL_DATA_OUT;

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
	s_Core.HaltIn = 0;
	s_Core.HaltOut = 0;
	s_Core.RemoteWakeup = false;
	memset(s_Core.Alternate, 0, sizeof(s_Core.Alternate));
}

static bool UsbCoreSelectConfig(int Index, uint8_t Configuration)
{
	return static_cast<UsbDeviceClass *>(s_Core.Object[Index])->
		SelectConfig(Configuration);
}

static void UsbCoreUnconfigureClasses(void)
{
	if (s_Core.Configuration == 0)
	{
		return;
	}

	for (int i = 0; i < s_Core.ObjectCnt; i++)
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
	UsbCtrlrEpCloseAll(s_Core.DevNo);
	s_Core.Configuration = 0;
	s_Core.NumInterfaces = 0;
	UsbCoreClearEndpointState();

	if (Configuration == 0)
	{
		return true;
	}

	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		if (!UsbCoreSelectConfig(i, Configuration))
		{
			for (int n = 0; n <= i; n++)
			{
				(void)UsbCoreSelectConfig(n, 0);
			}
			UsbCtrlrEpCloseAll(s_Core.DevNo);
			return false;
		}
	}

	s_Core.Configuration = Configuration;
	s_Core.NumInterfaces = pConfigDesc[4];

	return true;
}

static void UsbCoreSetEndpointHalt(uint8_t EpNo, bool bIn, bool Halt)
{
	const uint16_t bit = (uint16_t)(1U << EpNo);
	uint16_t *pMask = bIn ? &s_Core.HaltIn : &s_Core.HaltOut;

	if (Halt)
	{
		UsbCtrlrEpStall(s_Core.DevNo, EpNo, bIn);
		*pMask |= bit;
	}
	else
	{
		UsbCtrlrEpClearStall(s_Core.DevNo, EpNo, bIn);
		*pMask &= (uint16_t)~bit;
	}
}

static bool UsbCoreEndpointHalted(uint8_t EpNo, bool bIn)
{
	const uint16_t bit = (uint16_t)(1U << EpNo);
	const uint16_t mask = bIn ? s_Core.HaltIn : s_Core.HaltOut;

	return (mask & bit) != 0;
}

static bool UsbCoreHandleGetDescriptor(void)
{
	if (!UsbCoreDirIn(&s_Core.Setup) ||
		UsbCoreRecipient(&s_Core.Setup) != USB_REQTYPE_DEVICE)
	{
		return false;
	}

	const uint8_t type = (uint8_t)(s_Core.Setup.wValue >> 8);
	const uint8_t index = (uint8_t)s_Core.Setup.wValue;

	if (type != USB_DESCTYPE_STRING && s_Core.Setup.wIndex != 0U)
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
		pDesc = UsbCoreGetDescriptor(type, index, s_Core.Setup.wIndex, &len);
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
		s_Core.Cfg.Ep0Mps = pDesc[7];
	}

	return UsbCoreStartIn(pDesc, len);
}

static bool UsbCoreHandleGetStatus(void)
{
	if (!UsbCoreDirIn(&s_Core.Setup) || s_Core.Setup.wValue != 0 ||
		s_Core.Setup.wLength != 2)
	{
		return false;
	}

	uint16_t status = 0;
	const uint8_t recipient = UsbCoreRecipient(&s_Core.Setup);

	switch (recipient)
	{
		case USB_REQTYPE_DEVICE:
			if (s_Core.Setup.wIndex != 0)
			{
				return false;
			}
			if (UsbCoreConfigSelfPowered())
			{
				status |= USB_DEVSTATUS_SELF_POWERED;
			}
			if (s_Core.RemoteWakeup)
			{
				status |= USB_DEVSTATUS_REMOTE_WAKEUP;
			}
			break;

		case USB_REQTYPE_INTERFACE:
			if (s_Core.Configuration == 0 || s_Core.Setup.wIndex > 0xFFU ||
				!UsbCoreInterfaceExists((uint8_t)s_Core.Setup.wIndex))
			{
				return false;
			}
			break;

		case USB_REQTYPE_ENDPOINT:
		{
			const uint8_t epAddr = (uint8_t)s_Core.Setup.wIndex;
			const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
			const bool in = USB_ENDPADDR_IS_IN(epAddr);
			if ((s_Core.Setup.wIndex & 0xFF00U) != 0 ||
				(epAddr & 0x70U) != 0U ||
				(epNum != 0U && s_Core.Configuration == 0U) ||
				!UsbCoreEndpointExists(epNum, in))
			{
				return false;
			}
			if (UsbCoreEndpointHalted(epNum, in))
			{
				status = USB_ENDPSTATUS_HALT;
			}
			break;
		}

		default:
			return false;
	}

	s_Core.CtrlReply[0] = (uint8_t)status;
	s_Core.CtrlReply[1] = (uint8_t)(status >> 8);
	return UsbCoreStartIn(s_Core.CtrlReply, 2);
}

static bool UsbCoreHandleFeature(bool Set)
{
	if (UsbCoreDirIn(&s_Core.Setup) || s_Core.Setup.wLength != 0)
	{
		return false;
	}

	const uint8_t recipient = UsbCoreRecipient(&s_Core.Setup);

	if (recipient == USB_REQTYPE_DEVICE &&
		s_Core.Setup.wValue == USB_FEATSEL_DEVICE_REMOTE_WAKEUP &&
		s_Core.Setup.wIndex == 0 && s_Core.Configuration != 0 &&
		UsbCoreConfigRemoteWakeupCapable())
	{
		s_Core.RemoteWakeup = Set;
		return UsbCoreStartStatus();
	}

	if (recipient == USB_REQTYPE_ENDPOINT &&
		s_Core.Setup.wValue == USB_FEATSEL_ENDPOINT_HALT &&
		(s_Core.Setup.wIndex & 0xFF00U) == 0)
	{
		const uint8_t epAddr = (uint8_t)s_Core.Setup.wIndex;
		const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
		const bool in = USB_ENDPADDR_IS_IN(epAddr);

		if ((epAddr & 0x70U) != 0U || epNum == 0U ||
			s_Core.Configuration == 0 || !UsbCoreEndpointExists(epNum, in))
		{
			return false;
		}

		UsbCoreSetEndpointHalt(epNum, in, Set);
		return UsbCoreStartStatus();
	}

	return false;
}

static bool UsbCoreHandleSetConfiguration(void)
{
	if (UsbCoreDirIn(&s_Core.Setup) ||
		UsbCoreRecipient(&s_Core.Setup) != USB_REQTYPE_DEVICE ||
		s_Core.Setup.wIndex != 0 || s_Core.Setup.wLength != 0 ||
		s_Core.Setup.wValue > 0xFFU || s_Core.Address == 0)
	{
		return false;
	}

	if (!UsbCoreApplyConfiguration((uint8_t)s_Core.Setup.wValue))
	{
		return false;
	}

	return UsbCoreStartStatus();
}

static bool UsbCoreHandleGetInterface(void)
{
	if (!UsbCoreDirIn(&s_Core.Setup) ||
		UsbCoreRecipient(&s_Core.Setup) != USB_REQTYPE_INTERFACE ||
		s_Core.Setup.wValue != 0 || s_Core.Setup.wLength != 1 ||
		s_Core.Setup.wIndex > 0xFFU || s_Core.Configuration == 0)
	{
		return false;
	}

	const uint8_t interfaceNo = (uint8_t)s_Core.Setup.wIndex;
	if (interfaceNo >= USB_CORE_INTRF_MAXCNT ||
		!UsbCoreInterfaceExists(interfaceNo))
	{
		return false;
	}

	s_Core.CtrlReply[0] = s_Core.Alternate[interfaceNo];
	return UsbCoreStartIn(s_Core.CtrlReply, 1);
}

static bool UsbCoreHandleSetInterface(void)
{
	if (UsbCoreDirIn(&s_Core.Setup) ||
		UsbCoreRecipient(&s_Core.Setup) != USB_REQTYPE_INTERFACE ||
		s_Core.Setup.wLength != 0 || s_Core.Setup.wIndex > 0xFFU ||
		s_Core.Setup.wValue > 0xFFU || s_Core.Configuration == 0)
	{
		return false;
	}

	const uint8_t interfaceNo = (uint8_t)s_Core.Setup.wIndex;
	const uint8_t alternate = (uint8_t)s_Core.Setup.wValue;

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

	const uint8_t oldAlternate = s_Core.Alternate[interfaceNo];
	const bool selected =
		static_cast<UsbDeviceClass *>(s_Core.Object[cls])->
			SelectInterface(interfaceNo, alternate);
	if (!selected)
	{
		return false;
	}

	UsbCoreClearInterfaceHalt(interfaceNo, oldAlternate, alternate);
	s_Core.Alternate[interfaceNo] = alternate;
	return UsbCoreStartStatus();
}

static bool UsbCoreHandleStandard(void)
{
	switch (s_Core.Setup.bRequest)
	{
		case USB_REQ_GET_STATUS:
			return UsbCoreHandleGetStatus();

		case USB_REQ_CLEAR_FEATURE:
			return UsbCoreHandleFeature(false);

		case USB_REQ_SET_FEATURE:
			return UsbCoreHandleFeature(true);

		case USB_REQ_SET_ADDRESS:
			if (UsbCoreDirIn(&s_Core.Setup) ||
				UsbCoreRecipient(&s_Core.Setup) != USB_REQTYPE_DEVICE ||
				s_Core.Setup.wIndex != 0 || s_Core.Setup.wLength != 0 ||
				s_Core.Setup.wValue > 127U || s_Core.Configuration != 0)
			{
				return false;
			}
			s_Core.PendingAddress = (uint8_t)s_Core.Setup.wValue;
			s_Core.AddressPending = true;
			UsbCtrlrSetAddress(s_Core.DevNo, s_Core.PendingAddress);
			return UsbCoreStartStatus();

		case USB_REQ_GET_DESCRIPTOR:
			if (UsbCoreRecipient(&s_Core.Setup) == USB_REQTYPE_DEVICE)
			{
				return UsbCoreHandleGetDescriptor();
			}
			return UsbCoreRecipient(&s_Core.Setup) == USB_REQTYPE_INTERFACE &&
				UsbCoreHandleClassRequest();

		case USB_REQ_GET_CONFIGURATION:
			if (!UsbCoreDirIn(&s_Core.Setup) ||
				UsbCoreRecipient(&s_Core.Setup) != USB_REQTYPE_DEVICE ||
				s_Core.Setup.wValue != 0 || s_Core.Setup.wIndex != 0 ||
				s_Core.Setup.wLength != 1)
			{
				return false;
			}
			s_Core.CtrlReply[0] = s_Core.Configuration;
			return UsbCoreStartIn(s_Core.CtrlReply, 1);

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
		static_cast<UsbDeviceClass *>(s_Core.Object[Index])->
			Control(&s_Core.Setup, USB_CTRL_SETUP, &pData, &len);
	if (!handled)
	{
		return false;
	}

	s_Core.ActiveClass = Index;

	if (s_Core.Setup.wLength == 0)
	{
		s_Core.CtrlData = pData;
		s_Core.CtrlDataLen = 0;
		s_Core.CtrlActual = 0;
		return UsbCoreStartStatus();
	}

	if (UsbCoreDirIn(&s_Core.Setup))
	{
		return UsbCoreStartIn(pData, len);
	}

	return UsbCoreStartOut(pData, len);
}

static bool UsbCoreHandleClassRequest(void)
{
	const uint8_t recipient = UsbCoreRecipient(&s_Core.Setup);

	if (recipient == USB_REQTYPE_INTERFACE)
	{
		if (s_Core.Configuration == 0 || s_Core.Setup.wIndex > 0xFFU)
		{
			return false;
		}

		const uint8_t interfaceNo = (uint8_t)s_Core.Setup.wIndex;
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
		if (s_Core.Configuration == 0 || (s_Core.Setup.wIndex & 0xFF00U) != 0)
		{
			return false;
		}

		const uint8_t epAddr = (uint8_t)s_Core.Setup.wIndex;
		const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
		const bool in = USB_ENDPADDR_IS_IN(epAddr);
		if ((epAddr & 0x70U) != 0U || epNum == 0U ||
			!UsbCoreEndpointExists(epNum, in))
		{
			return false;
		}

		const int cls = UsbCoreFindEndpointClass(epNum, in);
		return cls >= 0 && UsbCoreCallClassSetup(cls);
	}

	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		if (UsbCoreCallClassSetup(i))
		{
			return true;
		}
	}

	return false;
}

static void UsbDevProcessSetup(const UsbSetupData_t *pSetup)
{
	if (pSetup == nullptr)
	{
		return;
	}

	UsbCoreAbortControl();
	memcpy(&s_Core.Setup, pSetup, sizeof(s_Core.Setup));

	const uint8_t type = s_Core.Setup.bmRequestType & USB_REQTYPE_MASK_TYPE;
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

static void UsbCoreProcessEp0Complete(const UsbCtrlrXferEvt_t *pXfer)
{
	if (pXfer == nullptr || pXfer->Result != USB_CTRLR_XFER_SUCCESS)
	{
		UsbCoreStallControl();
		return;
	}

	switch (s_Core.CtrlState)
	{
		case USB_CTRL_DATA_OUT:
			if (pXfer->Length > s_Core.CtrlDataLen - s_Core.CtrlActual)
			{
				UsbCoreStallControl();
				return;
			}
			memcpy(s_Core.CtrlData + s_Core.CtrlActual, pXfer->pBuffer, pXfer->Length);
			s_Core.CtrlActual += pXfer->Length;
			if (s_Core.CtrlActual < s_Core.CtrlDataLen &&
				pXfer->Length == s_Core.Cfg.Ep0Mps)
				return;
			break;
		case USB_CTRL_DATA_IN:
			if (s_Core.CtrlActual < s_Core.CtrlDataLen)
			{
				if (!UsbCoreSendIn())
					UsbCoreStallControl();
				return;
			}
			break;

		case USB_CTRL_DATA_IN_ZLP:
			(void)UsbCoreStartStatus();
			return;

		case USB_CTRL_STATUS_IN:
		case USB_CTRL_STATUS_OUT:
			if (s_Core.AddressPending)
			{
				s_Core.Address = s_Core.PendingAddress;
			}
			(void)UsbCoreInvokeActive(USB_CTRL_COMPLETE,
									   s_Core.CtrlActual);
			UsbCoreResetControl();
			return;

		case USB_CTRL_IDLE:
		default:
			return;
	}

	if (!UsbCoreInvokeActive(USB_CTRL_DATA, s_Core.CtrlActual))
	{
		UsbCoreStallControl();
		return;
	}

	if (s_Core.CtrlNeedZlp)
	{
		s_Core.CtrlNeedZlp = false;
		s_Core.CtrlState = USB_CTRL_DATA_IN_ZLP;
		if (UsbCtrlrEp0Send(s_Core.DevNo, nullptr, 0) < 0)
		{
			UsbCoreStallControl();
		}
		return;
	}

	(void)UsbCoreStartStatus();
}

static void UsbCoreNotifyReset(void)
{
	UsbCoreUnconfigureClasses();

	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		s_Core.Object[i]->Reset();
	}
}

static void UsbCoreNotifyDetach(void)
{
	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		static_cast<UsbDeviceClass *>(s_Core.Object[i])->Detach();
	}
}

static void UsbCoreResetDeviceState(bool NotifyClasses)
{
	UsbCoreAbortControl();

	if (NotifyClasses)
	{
		UsbCoreNotifyReset();
	}

	s_Core.Address = 0;
	s_Core.Configuration = 0;
	s_Core.NumInterfaces = 0;
	s_Core.Suspended = false;
	UsbCoreClearEndpointState();
}

void UsbDevProcessEvent(int DevNo, const UsbCtrlrEvt_t *pEvt)
{
	(void)DevNo;
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
			UsbDevProcessSetup(&pEvt->Setup);
			break;

		case USB_CTRLR_EVT_XFER_CMPL:
			if (USB_ENDPADDR_NUM(pEvt->Xfer.EpAddr) == 0)
			{
				UsbCoreProcessEp0Complete(&pEvt->Xfer);
			}
			break;

		case USB_CTRLR_EVT_SUSPEND:
			s_Core.Suspended = true;
			break;

		case USB_CTRLR_EVT_RESUME:
			s_Core.Suspended = false;
			break;

		case USB_CTRLR_EVT_ADDRESS:
			s_Core.Address = pEvt->Address;
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

	memcpy(&s_Core.Cfg, pCfg, sizeof(s_Core.Cfg));
	if (s_Core.Cfg.Ep0Mps == 0)
	{
		s_Core.Cfg.Ep0Mps = USBD_CORE_EP0_MPS_DEFAULT;
	}
	else if (!UsbCoreValidEp0Mps(s_Core.Cfg.Ep0Mps))
	{
		return false;
	}

	memset(s_Core.Object, 0, sizeof(s_Core.Object));
	// Minus one is no owner. Zero would claim class zero owns every
	// endpoint, so this cannot be left to static initialization.
	memset(s_Core.EpClass, -1, sizeof(s_Core.EpClass));
	s_Core.ObjectCnt = 0;
	s_Core.Initialized = false;
	s_Core.Started = false;
	UsbCoreResetDeviceState(false);

	s_Core.Initialized = true;
	return true;
}

static bool UsbCoreRegisterObject(UsbDeviceClass *pClass,
								  uint8_t FirstInterface,
								  uint8_t InterfaceCount,
								  uint16_t EpInMask,
								  uint16_t EpOutMask)
{
	if (!s_Core.Initialized || pClass == nullptr ||
		s_Core.Started ||
		s_Core.ObjectCnt >= USB_CORE_CLASS_MAXCNT)
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

		for (int i = 0; i < s_Core.ObjectCnt; i++)
		{
			const UsbDeviceClass *pRegistered =
				static_cast<const UsbDeviceClass *>(s_Core.Object[i]);
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

	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		const UsbDeviceClass *pRegistered =
			static_cast<const UsbDeviceClass *>(s_Core.Object[i]);
		if (pRegistered == pClass ||
			(EpInMask & pRegistered->EpInMask()) != 0 ||
			(EpOutMask & pRegistered->EpOutMask()) != 0)
		{
			return false;
		}
	}

	s_Core.Object[s_Core.ObjectCnt] = pClass;

	for (int ep = 1; ep < 16; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if ((EpInMask & bit) != 0)
		{
			s_Core.EpClass[1][ep] = (int8_t)s_Core.ObjectCnt;
		}
		if ((EpOutMask & bit) != 0)
		{
			s_Core.EpClass[0][ep] = (int8_t)s_Core.ObjectCnt;
		}
	}

	s_Core.ObjectCnt++;
	return true;
}

static void UsbCoreStart(void)
{
	if (!s_Core.Initialized || s_Core.Started)
	{
		return;
	}

	UsbCtrlrIntEnable(s_Core.DevNo);
	UsbCtrlrConnect(s_Core.DevNo);
	s_Core.Started = true;
}

static void UsbCoreStop(void)
{
	if (!s_Core.Started)
	{
		return;
	}

	UsbCtrlrDisconnect(s_Core.DevNo);
	UsbCtrlrIntDisable(s_Core.DevNo);
	UsbCtrlrEpCloseAll(s_Core.DevNo);
	UsbCoreResetDeviceState(true);
	s_Core.Started = false;
}

static bool UsbCoreRemoteWakeup(void)
{
	if (!s_Core.Started || !s_Core.Suspended || !s_Core.RemoteWakeup)
	{
		return false;
	}

	UsbCtrlrRemoteWakeup(s_Core.DevNo);
	return true;
}

static bool UsbCoreConfigured(void)
{
	return s_Core.Started && s_Core.Configuration != 0;
}

static bool UsbCoreSuspended(void)
{
	return s_Core.Started && s_Core.Suspended;
}

static uint8_t UsbCoreAddress(void)
{
	return s_Core.Address;
}

static uint8_t UsbCoreConfiguration(void)
{
	return s_Core.Configuration;
}

static uint8_t UsbCoreAlternate(uint8_t InterfaceNo)
{
	return InterfaceNo < USB_CORE_INTRF_MAXCNT ?
		s_Core.Alternate[InterfaceNo] : 0;
}

static bool UsbCoreRemoteWakeupEnabled(void)
{
	return s_Core.RemoteWakeup;
}

//
// Application entry points.
//

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
		UsbCtrlrGetSerial(s_Core.DevNo, s_UsbDevSerial,
						  sizeof(s_UsbDevSerial));
	}
	s_UsbDevCfg.pSerial = s_UsbDevSerial;

	UsbCtrlrCfg_t ctrlrCfg = {};
	ctrlrCfg.IntPrio = s_UsbDevCfg.IntPrio;
	ctrlrCfg.bLowPowerSuspend = s_UsbDevCfg.bLowPowerSuspend;

	if (!UsbCtrlrInit(s_Core.DevNo, &ctrlrCfg))
	{
		return false;
	}

	UsbCoreCfg_t coreCfg = {};
	coreCfg.Ep0Mps = USB_CTRLR_PKT_LEN_MAX(s_Core.DevNo, CONTROL);

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
	if (UsbGetDescriptor(s_Core.DevNo, USB_DESCTYPE_CONFIGURATION, 0U, 0U,
		UsbGetSpeed(s_Core.DevNo), &descriptorLength) == nullptr)
	{
		return false;
	}

	// One call. Power, clock and PHY come up and endpoint zero is prepared.
	if (!UsbCtrlrStart(s_Core.DevNo))
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
	UsbCtrlrStop(s_Core.DevNo);
	s_UsbDevStarted = false;
}

static void UsbDevProcess(void)
{
	if (!s_UsbDevInitialized)
	{
		return;
	}

	UsbCtrlrProcess(s_Core.DevNo);

	if (!s_UsbDevStarted)
	{
		//
		// Retry on the level, not only on the attach edge. A board already on
		// a cable at reset never produces an edge, so an Enable that failed
		// during start up would be the only attempt ever made and the port
		// would stay down with nothing to show for it. Enable is cheap while
		// there is no bus power, because UsbCtrlrStart answers false immediately.
		//

		if (UsbCtrlrVbusDetected(s_Core.DevNo))
		{
			(void)UsbDevEnable();
		}

		if (!s_UsbDevStarted)
		{
			return;
		}
	}

	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		s_Core.Object[i]->Process();
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

	if (!AppEvtHandlerInit(nullptr, 0U))
	{
		return false;
	}

	s_Core.DevNo = pCfg->DevNo;
	s_Core.VbusLast = false;

	return UsbDevInit(pCfg);
}

bool UsbClassRegister(int DevNo, UsbDeviceClass *pClass,
					  uint8_t FirstInterface, uint8_t InterfaceCount,
					  uint16_t EpInMask, uint16_t EpOutMask)
{
	if (DevNo != s_Core.DevNo ||
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
	if (DevNo != s_Core.DevNo || pClass == nullptr || s_Core.Started ||
		pFsDescriptor == nullptr || FsDescriptorLength == 0U ||
		(USB_HIGHSPEED_CAPABLE(DevNo) &&
		 (pHsDescriptor == nullptr || HsDescriptorLength == 0U)))
	{
		return false;
	}

	bool registered = false;
	for (int i = 0; i < s_Core.ObjectCnt; i++)
	{
		if (s_Core.Object[i] == pClass)
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

// Weak so a flash-constrained application can supply a strong replacement
// that returns prebuilt static descriptors. When overridden, this default
// and the runtime configuration assembly it drives (including s_Core.ConfigDesc)
// are dropped by the linker's unused-section removal. A replacement owns every
// descriptor type it is asked for (device, configuration, string) and, in the
// static case, owns interface and endpoint numbering.
__attribute__((weak))
const uint8_t *UsbGetDescriptor(int DevNo, uint8_t Type, uint8_t Index,
								 uint16_t LangId, UsbSpeed_t Speed,
								 uint16_t *pLength)
{
	if (pLength == nullptr || DevNo != s_Core.DevNo ||
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
	return DevNo == s_Core.DevNo && UsbDevEnable();
}

void UsbDisable(int DevNo)
{
	if (DevNo == s_Core.DevNo)
	{
		UsbDevDisable();
	}
}

void UsbProcess(int DevNo)
{
	if (DevNo != s_Core.DevNo)
	{
		return;
	}

	UsbDevProcess();

	// Cable events are derived here rather than reported by the port, which
	// only exposes the level. UsbDevProcess has already polled the controller,
	// so this edge is against a fresh reading.
	const bool vbus = UsbCtrlrVbusDetected(s_Core.DevNo);

	if (vbus != s_Core.VbusLast)
	{
		s_Core.VbusLast = vbus;
		if (!vbus)
		{
			UsbCoreNotifyDetach();
		}

		if (s_UsbDevCfg.EvtHandler != nullptr)
		{
			s_UsbDevCfg.EvtHandler(s_Core.DevNo,
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
	return DevNo == s_Core.DevNo && UsbDevMounted();
}

bool UsbSuspended(int DevNo)
{
	return DevNo == s_Core.DevNo && UsbDevSuspended();
}

bool UsbRemoteWakeupEnabled(int DevNo)
{
	return DevNo == s_Core.DevNo && UsbCoreRemoteWakeupEnabled();
}

bool UsbRemoteWakeup(int DevNo)
{
	return DevNo == s_Core.DevNo && UsbCoreRemoteWakeup();
}

bool UsbEpSetHalt(int DevNo, uint8_t EpNo, bool bIn, bool Halt)
{
	if (DevNo != s_Core.DevNo || EpNo == 0U || EpNo >= 16U ||
		!UsbCoreEndpointExists(EpNo, bIn))
	{
		return false;
	}

	UsbCoreSetEndpointHalt(EpNo, bIn, Halt);
	return true;
}

bool UsbEpHalted(int DevNo, uint8_t EpNo, bool bIn)
{
	return DevNo == s_Core.DevNo && EpNo != 0U && EpNo < 16U &&
		UsbCoreEndpointExists(EpNo, bIn) &&
		UsbCoreEndpointHalted(EpNo, bIn);
}

const UsbCfg_t *UsbGetCfg(int DevNo)
{
	return DevNo == s_Core.DevNo ? UsbDevGetCfg() : nullptr;
}

const char *UsbGetSerial(int DevNo)
{
	return DevNo == s_Core.DevNo ? UsbDevGetSerial() : nullptr;
}

uint8_t UsbGetAddress(int DevNo)
{
	return DevNo == s_Core.DevNo ? UsbCoreAddress() : 0;
}

uint8_t UsbGetConfiguration(int DevNo)
{
	return DevNo == s_Core.DevNo ? UsbCoreConfiguration() : 0;
}

uint8_t UsbGetAlternate(int DevNo, uint8_t InterfaceNo)
{
	return DevNo == s_Core.DevNo ? UsbCoreAlternate(InterfaceNo) : 0;
}

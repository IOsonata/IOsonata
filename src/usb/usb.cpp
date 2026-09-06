/**-------------------------------------------------------------------------
@file	usb.cpp

@brief	Generic USB layer.

Identity, Chapter 9 and function dispatch in one place. usb_dev.cpp held the
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
#include "usb/usbd_cdc.h"

/// Controller this instance drives, set by UsbInit.
static int s_UsbDevNo = 0;

/// Bus power level at the previous UsbProcess pass, for edge reporting.
static bool s_UsbVbusLast = false;

#define USB_CORE_FUNC_MAXCNT		8
#define USB_CORE_INTRF_MAXCNT		16

/// Chapter 9 settings. Built by UsbInit from UsbCfg_t and usb_ctrlr.h, never
/// supplied by an application, which is why it is no longer in a header.
typedef struct __Usb_Core_Config {
	UsbDescHandler_t DescHandler;	//!< Device descriptor provider
	void *pDescContext;				//!< Descriptor callback context
	UsbSpeed_t Speed;				//!< Current bus speed for descriptors
	uint8_t Ep0Mps;					//!< EP0 max packet size
} UsbCoreCfg_t;

#define USBD_CORE_EP0_MPS_DEFAULT		64U
#define USBD_CORE_DEVICE_DESC_LEN		((uint16_t)sizeof(UsbDevDesc_t))
#define USBD_CORE_CONFIG_DESC_LEN		((uint16_t)sizeof(UsbCfgDesc_t))

typedef enum __Usbd_Core_Ctrl_State {
	USB_CTRL_IDLE,
	USB_CTRL_DATA_IN,
	USB_CTRL_DATA_IN_ZLP,
	USB_CTRL_DATA_OUT,
	USB_CTRL_STATUS_IN,
	USB_CTRL_STATUS_OUT,
} UsbCoreCtrlState_t;

static UsbCoreCfg_t s_CoreCfg;
static UsbFuncCfg_t s_CoreFunc[USB_CORE_FUNC_MAXCNT];
static int s_CoreFuncCnt;
// Endpoint to function index, [0] OUT and [1] IN. Ownership masks are fixed
// once a function registers and may not overlap, so this answer never changes
// after registration. Every transfer completion needs it, and a completion
// runs in the USB interrupt, which is time the application is not filling the
// Tx CFifo. Minus one means no function owns that endpoint.
static int8_t s_CoreEpFunc[2][16];

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
static int s_ActiveFunc;
static uint8_t *s_CtrlData;
static uint16_t s_CtrlDataLen;
static uint16_t s_CtrlActual;
static bool s_CtrlNeedZlp;
static uint8_t s_CtrlReply[2];

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
	if (pLength == nullptr || s_CoreCfg.DescHandler == nullptr)
	{
		return nullptr;
	}

	*pLength = 0;
	return s_CoreCfg.DescHandler(Type, Index, LangId, s_CoreCfg.Speed,
								 pLength, s_CoreCfg.pDescContext);
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

static int UsbCoreFindFunction(uint8_t InterfaceNo)
{
	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		const uint8_t first = s_CoreFunc[i].FirstInterface;
		const uint8_t count = s_CoreFunc[i].InterfaceCount;

		if (count != 0 && InterfaceNo >= first &&
			InterfaceNo < (uint8_t)(first + count))
		{
			return i;
		}
	}

	return -1;
}

static int UsbCoreFindEndpointFunction(uint8_t EpAddr)
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

	return s_CoreEpFunc[USB_ENDPADDR_IS_IN(EpAddr) ? 1 : 0][epNum];
}

static void UsbCoreResetControl(void)
{
	s_CtrlState = USB_CTRL_IDLE;
	s_ActiveFunc = -1;
	s_CtrlData = nullptr;
	s_CtrlDataLen = 0;
	s_CtrlActual = 0;
	s_CtrlNeedZlp = false;
	s_PendingAddress = 0;
	s_AddressPending = false;
}

static void UsbCoreAbortControl(void);

static void UsbCoreStallControl(void)
{
	UsbCoreAbortControl();
	UsbCtrlrEpStall(s_UsbDevNo, 0);
}

static bool UsbCoreInvokeActive(UsbCtrlStage_t Stage,
								 uint16_t Length)
{
	if (s_ActiveFunc < 0 || s_ActiveFunc >= s_CoreFuncCnt)
	{
		return true;
	}

	UsbRequestHandler_t handler =
		s_CoreFunc[s_ActiveFunc].RequestHandler;

	if (handler == nullptr)
	{
		return true;
	}

	uint8_t *pData = s_CtrlData;
	uint16_t len = Length;

	return handler(&s_Setup, Stage, &pData, &len,
				   s_CoreFunc[s_ActiveFunc].pContext);
}

static void UsbCoreAbortControl(void)
{
	if (s_ActiveFunc >= 0 && s_ActiveFunc < s_CoreFuncCnt)
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

static bool UsbCoreWantSof(void)
{
	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if (s_CoreFunc[i].SofHandler != nullptr)
		{
			return true;
		}
	}

	return false;
}

static void UsbCoreUnconfigureFunctions(void)
{
	if (s_Configuration == 0)
	{
		return;
	}

	UsbCtrlrSofEnable(s_UsbDevNo, false);

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if (s_CoreFunc[i].ConfigHandler != nullptr)
		{
			(void)s_CoreFunc[i].ConfigHandler(0,
									   s_CoreFunc[i].pContext);
		}
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

	UsbCoreUnconfigureFunctions();
	UsbCtrlrEpCloseAll(s_UsbDevNo);
	s_Configuration = 0;
	s_NumInterfaces = 0;
	UsbCoreClearEndpointState();

	if (Configuration == 0)
	{
		return true;
	}

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if (s_CoreFunc[i].ConfigHandler != nullptr &&
			!s_CoreFunc[i].ConfigHandler(Configuration,
									  s_CoreFunc[i].pContext))
		{
			for (int n = 0; n <= i; n++)
			{
				if (s_CoreFunc[n].ConfigHandler != nullptr)
				{
					(void)s_CoreFunc[n].ConfigHandler(0,
										   s_CoreFunc[n].pContext);
				}
			}
			UsbCtrlrEpCloseAll(s_UsbDevNo);
			return false;
		}
	}

	s_Configuration = Configuration;
	s_NumInterfaces = pConfigDesc[4];

	// Controllers drop SOF on bus reset, so enable it per configuration
	if (UsbCoreWantSof())
	{
		UsbCtrlrSofEnable(s_UsbDevNo, true);
	}

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

	const int func = UsbCoreFindFunction(interfaceNo);
	if (func < 0 || s_CoreFunc[func].SetInterfaceHandler == nullptr)
	{
		return false;
	}

	const uint8_t oldAlternate = s_Alternate[interfaceNo];
	if (!s_CoreFunc[func].SetInterfaceHandler(interfaceNo, alternate,
											 s_CoreFunc[func].pContext))
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
			return UsbCoreHandleGetDescriptor();

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

static bool UsbCoreCallFunctionSetup(int Index)
{
	UsbRequestHandler_t handler = s_CoreFunc[Index].RequestHandler;
	if (handler == nullptr)
	{
		return false;
	}

	uint8_t *pData = nullptr;
	uint16_t len = 0;

	if (!handler(&s_Setup, USB_CTRL_SETUP, &pData, &len,
				 s_CoreFunc[Index].pContext))
	{
		return false;
	}

	s_ActiveFunc = Index;

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

static bool UsbCoreHandleFunctionRequest(void)
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

		const int func = UsbCoreFindFunction(interfaceNo);
		return func >= 0 && UsbCoreCallFunctionSetup(func);
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

		const int func = UsbCoreFindEndpointFunction(epAddr);
		return func >= 0 && UsbCoreCallFunctionSetup(func);
	}

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if (UsbCoreCallFunctionSetup(i))
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
		handled = UsbCoreHandleFunctionRequest();
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
	UsbCoreUnconfigureFunctions();

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if (s_CoreFunc[i].ResetHandler != nullptr)
		{
			s_CoreFunc[i].ResetHandler(s_CoreFunc[i].pContext);
		}
	}
}

static void UsbCoreResetDeviceState(bool NotifyFunctions)
{
	UsbCoreAbortControl();

	if (NotifyFunctions)
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
			else
			{
				const int func = UsbCoreFindEndpointFunction(pEvt->Xfer.EpAddr);
				if (func >= 0 && s_CoreFunc[func].XferHandler != nullptr)
				{
					s_CoreFunc[func].XferHandler(pEvt->Xfer.EpAddr,
										 pEvt->Xfer.Length,
										 pEvt->Xfer.Result,
										 s_CoreFunc[func].pContext);
				}
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

		case USB_CTRLR_EVT_SOF:
			if (s_Configuration != 0)
			{
				for (int i = 0; i < s_CoreFuncCnt; i++)
				{
					if (s_CoreFunc[i].SofHandler != nullptr)
					{
						s_CoreFunc[i].SofHandler(pEvt->FrameNo,
												 s_CoreFunc[i].pContext);
					}
				}
			}
			break;

		default:
			break;
	}
}

static bool UsbCoreInit(const UsbCoreCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->DescHandler == nullptr)
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

	memset(s_CoreFunc, 0, sizeof(s_CoreFunc));
	// Minus one is no owner. Zero would claim function zero owns every
	// endpoint, so this cannot be left to static initialization.
	memset(s_CoreEpFunc, -1, sizeof(s_CoreEpFunc));
	s_CoreFuncCnt = 0;
	s_CoreInitialized = false;
	s_CoreStarted = false;
	UsbCoreResetDeviceState(false);

	s_CoreInitialized = true;
	return true;
}

static bool UsbCoreRegisterFunction(const UsbFuncCfg_t *pCfg)
{
	if (!s_CoreInitialized || pCfg == nullptr || s_CoreStarted ||
		s_CoreFuncCnt >= USB_CORE_FUNC_MAXCNT)
	{
		return false;
	}

	if (((pCfg->EpInMask | pCfg->EpOutMask) & 1U) != 0 ||
		((pCfg->EpInMask | pCfg->EpOutMask) != 0 && pCfg->XferHandler == nullptr))
	{
		return false;
	}

	if (pCfg->InterfaceCount != 0)
	{
		const uint16_t last = (uint16_t)pCfg->FirstInterface +
			(uint16_t)pCfg->InterfaceCount;
		if (last > USB_CORE_INTRF_MAXCNT)
		{
			return false;
		}

		for (int i = 0; i < s_CoreFuncCnt; i++)
		{
			const uint16_t firstA = pCfg->FirstInterface;
			const uint16_t lastA = last;
			const uint16_t firstB = s_CoreFunc[i].FirstInterface;
			const uint16_t lastB = firstB + s_CoreFunc[i].InterfaceCount;

			if (s_CoreFunc[i].InterfaceCount != 0 &&
				firstA < lastB && firstB < lastA)
			{
				return false;
			}
		}
	}

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if ((pCfg->EpInMask & s_CoreFunc[i].EpInMask) != 0 ||
			(pCfg->EpOutMask & s_CoreFunc[i].EpOutMask) != 0)
		{
			return false;
		}
	}

	memcpy(&s_CoreFunc[s_CoreFuncCnt], pCfg, sizeof(*pCfg));

	for (int ep = 1; ep < 16; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if ((pCfg->EpInMask & bit) != 0)
		{
			s_CoreEpFunc[1][ep] = (int8_t)s_CoreFuncCnt;
		}
		if ((pCfg->EpOutMask & bit) != 0)
		{
			s_CoreEpFunc[0][ep] = (int8_t)s_CoreFuncCnt;
		}
	}

	s_CoreFuncCnt++;
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

//
// Application entry points.
//

#define USB_SERIAL_MAXLEN			33	//!< 32 hexadecimal characters and a terminator

static UsbCfg_t s_UsbDevCfg;
static char s_UsbDevSerial[USB_SERIAL_MAXLEN];
static bool s_UsbDevInitialized;
static bool s_UsbDevStarted;

static int UsbDevMaxCdcCount(void)
{
	// The CDC topology uses endpoint 1 for notification and endpoint 2 for
	// bulk on the first function, then advances both by two. Endpoint counts
	// include EP0, so the smaller direction decides how many fit.
	const int maxIn = ((int)USB_EPIN_CNT(0) - 1) / 2;
	const int maxOut = ((int)USB_EPOUT_CNT(0) - 1) / 2;
	const int maxCdc = maxIn < maxOut ? maxIn : maxOut;

	return maxCdc < USBD_CDC_FUNC_MAXCNT ? maxCdc : USBD_CDC_FUNC_MAXCNT;
}

static const uint8_t *UsbDevDescHandler(uint8_t DescType,
									uint8_t DescIndex,
									uint16_t LangId,
									UsbSpeed_t,
									uint16_t *pLength,
									void *pContext)
{
	const UsbSpeed_t speed = UsbCtrlrHighSpeed(s_UsbDevNo) ?
		USB_SPEED_HIGH : USB_SPEED_FULL;

	return UsbdCdcDescHandler(DescType, DescIndex, LangId, speed,
							 pLength, pContext);
}


static bool UsbDevInit(const UsbCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->Vid == 0 || pCfg->Pid == 0)
	{
		return false;
	}

	const int maxCdc = UsbDevMaxCdcCount();
	if (maxCdc < 1)
	{
		return false;
	}

	s_UsbDevInitialized = false;
	s_UsbDevStarted = false;
	memcpy(&s_UsbDevCfg, pCfg, sizeof(s_UsbDevCfg));

	if (s_UsbDevCfg.NbCdc < 1)
	{
		s_UsbDevCfg.NbCdc = 1;
	}

	if (s_UsbDevCfg.NbCdc > maxCdc)
	{
		s_UsbDevCfg.NbCdc = maxCdc;
	}

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

	// A supplied handler replaces the built in builder. That is how a device
	// that is not CDC, or one with a composite descriptor set of its own,
	// gets its descriptors out.
	UsbCoreCfg_t coreCfg = {};
	if (s_UsbDevCfg.DescHandler != nullptr)
	{
		coreCfg.DescHandler = s_UsbDevCfg.DescHandler;
		coreCfg.pDescContext = s_UsbDevCfg.pDescContext;
	}
	else
	{
		coreCfg.DescHandler = UsbDevDescHandler;
		coreCfg.pDescContext = nullptr;
	}
	// Speed and endpoint zero packet size come from usb_ctrlr.h now. The
	// capability record that used to carry them is gone.
	coreCfg.Speed = USB_HIGHSPEED_CAPABLE(0) ? USB_SPEED_HIGH : USB_SPEED_FULL;
	coreCfg.Ep0Mps = USB_PKT_MAXLEN(0, CONTROL);

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

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if (s_CoreFunc[i].ProcessHandler != nullptr)
		{
			s_CoreFunc[i].ProcessHandler(s_CoreFunc[i].pContext);
		}
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

	s_UsbDevNo = pCfg->DevNo;
	s_UsbVbusLast = false;

	return UsbDevInit(pCfg);
}

bool UsbRegisterFunc(int DevNo, const UsbFuncCfg_t *pCfg)
{
	return DevNo == s_UsbDevNo && UsbCoreRegisterFunction(pCfg);
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

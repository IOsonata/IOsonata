/**-------------------------------------------------------------------------
@file	usbd_core.cpp

@brief	Native IOsonata USB device protocol core implementation.

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

#include "usb/usbd_core.h"

#define USBD_CORE_EP0_MPS_DEFAULT		64U
#define USBD_CORE_DEVICE_DESC_LEN		((uint16_t)sizeof(UsbDevDesc_t))
#define USBD_CORE_CONFIG_DESC_LEN		((uint16_t)sizeof(UsbCfgDesc_t))

typedef enum __Usbd_Core_Ctrl_State {
	USBD_CORE_CTRL_IDLE,
	USBD_CORE_CTRL_DATA_IN,
	USBD_CORE_CTRL_DATA_IN_ZLP,
	USBD_CORE_CTRL_DATA_OUT,
	USBD_CORE_CTRL_STATUS_IN,
	USBD_CORE_CTRL_STATUS_OUT,
} UsbdCoreCtrlState_t;

static UsbdCoreCfg_t s_CoreCfg;
static UsbdCoreFuncCfg_t s_CoreFunc[USBD_CORE_FUNC_MAXCNT];
static int s_CoreFuncCnt;

static bool s_CoreInitialized;
static bool s_CoreStarted;
static bool s_CoreSuspended;
static bool s_RemoteWakeup;
static uint8_t s_Address;
static uint8_t s_PendingAddress;
static bool s_AddressPending;
static uint8_t s_Configuration;
static uint8_t s_NumInterfaces;
static uint8_t s_Alternate[USBD_CORE_INTRF_MAXCNT];
static uint16_t s_HaltIn;
static uint16_t s_HaltOut;

static UsbdCoreCtrlState_t s_CtrlState;
static UsbSetupData_t s_Setup;
static int s_ActiveFunc;
static uint8_t *s_CtrlData;
static uint16_t s_CtrlDataLen;
static uint16_t s_CtrlActual;
static bool s_CtrlNeedZlp;
static uint8_t s_CtrlReply[2];

static uint8_t UsbdCoreRecipient(const UsbSetupData_t *pSetup)
{
	return pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT;
}

static bool UsbdCoreDirIn(const UsbSetupData_t *pSetup)
{
	return (pSetup->bmRequestType & USB_REQTYPE_DIRHOST) != 0;
}

static bool UsbdCoreValidEp0Mps(uint8_t Mps)
{
	return Mps == 8U || Mps == 16U || Mps == 32U || Mps == 64U;
}

static const uint8_t *UsbdCoreGetDescriptor(uint8_t Type, uint8_t Index,
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

static bool UsbdCoreValidateConfigDescriptor(const uint8_t *pDesc,
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

static const uint8_t *UsbdCoreGetConfigDescriptor(uint8_t Type, uint8_t Index,
												uint16_t *pLength)
{
	if (pLength == nullptr ||
		(Type != USB_DESCTYPE_CONFIGURATION && Type != USB_DESCTYPE_OSC))
	{
		return nullptr;
	}

	uint16_t providerLength;
	const uint8_t *pDesc = UsbdCoreGetDescriptor(Type, Index, 0,
												&providerLength);

	return UsbdCoreValidateConfigDescriptor(pDesc, providerLength, Type,
													 pLength) ? pDesc : nullptr;
}

static const uint8_t *UsbdCoreGetConfigByIndex(uint8_t Index,
										uint16_t *pLength)
{
	return UsbdCoreGetConfigDescriptor(USB_DESCTYPE_CONFIGURATION, Index,
												   pLength);
}

static uint8_t UsbdCoreConfigurationCount(void)
{
	uint16_t len;
	const uint8_t *pDesc = UsbdCoreGetDescriptor(USB_DESCTYPE_DEVICE,
												0, 0, &len);

	if (pDesc == nullptr || len < USBD_CORE_DEVICE_DESC_LEN ||
		pDesc[0] < USBD_CORE_DEVICE_DESC_LEN ||
		pDesc[1] != USB_DESCTYPE_DEVICE)
	{
		return 0;
	}

	return pDesc[17];
}

static const uint8_t *UsbdCoreGetConfigByValue(uint8_t Value,
										uint16_t *pLength)
{
	const uint8_t count = UsbdCoreConfigurationCount();

	for (uint8_t i = 0; i < count; i++)
	{
		uint16_t len;
		const uint8_t *pDesc = UsbdCoreGetConfigByIndex(i, &len);

		if (pDesc != nullptr && pDesc[5] == Value)
		{
			*pLength = len;
			return pDesc;
		}
	}

	return nullptr;
}

static const uint8_t *UsbdCoreActiveConfig(uint16_t *pLength)
{
	if (s_Configuration != 0)
	{
		return UsbdCoreGetConfigByValue(s_Configuration, pLength);
	}

	return UsbdCoreGetConfigByIndex(0, pLength);
}

static bool UsbdCoreInterfaceExists(uint8_t InterfaceNo)
{
	uint16_t len;
	const uint8_t *pDesc = UsbdCoreActiveConfig(&len);

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

static bool UsbdCoreEndpointExists(uint8_t EpAddr)
{
	if (USB_ENDPADDR_NUM(EpAddr) == 0)
	{
		return true;
	}

	uint16_t len;
	const uint8_t *pDesc = UsbdCoreActiveConfig(&len);

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

		if (type == USB_DESCTYPE_ENDPOINT && dlen >= sizeof(UsbEndPointDesc_t) &&
			pDesc[ofs + 2U] == EpAddr)
		{
			return true;
		}

		ofs = (uint16_t)(ofs + dlen);
	}

	return false;
}

static int UsbdCoreFindFunction(uint8_t InterfaceNo)
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

static int UsbdCoreFindEndpointFunction(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0)
	{
		return -1;
	}

	const uint16_t bit = (uint16_t)(1U << epNum);

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		const uint16_t mask = USB_ENDPADDR_IS_IN(EpAddr) ?
			s_CoreFunc[i].EpInMask : s_CoreFunc[i].EpOutMask;

		if ((mask & bit) != 0)
		{
			return i;
		}
	}

	return -1;
}

static void UsbdCoreResetControl(void)
{
	s_CtrlState = USBD_CORE_CTRL_IDLE;
	s_ActiveFunc = -1;
	s_CtrlData = nullptr;
	s_CtrlDataLen = 0;
	s_CtrlActual = 0;
	s_CtrlNeedZlp = false;
	s_PendingAddress = 0;
	s_AddressPending = false;
}

static void UsbdCoreAbortControl(void);

static void UsbdCoreStallControl(void)
{
	UsbdCoreAbortControl();
	UsbdCtrlrEpStall(0);
}

static bool UsbdCoreInvokeActive(UsbdCoreCtrlStage_t Stage,
								 uint16_t Length)
{
	if (s_ActiveFunc < 0 || s_ActiveFunc >= s_CoreFuncCnt)
	{
		return true;
	}

	UsbdCoreRequestHandler_t handler =
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

static void UsbdCoreAbortControl(void)
{
	if (s_ActiveFunc >= 0 && s_ActiveFunc < s_CoreFuncCnt)
	{
		(void)UsbdCoreInvokeActive(USBD_CORE_CTRL_ABORT, s_CtrlActual);
	}

	UsbdCoreResetControl();
}

static bool UsbdCoreStartStatus(void)
{
	const uint8_t epAddr = UsbdCoreDirIn(&s_Setup) ?
		USB_ENDPADDR_DIR_OUT : USB_ENDPADDR_DIR_IN;

	s_CtrlState = USB_ENDPADDR_IS_IN(epAddr) ?
		USBD_CORE_CTRL_STATUS_IN : USBD_CORE_CTRL_STATUS_OUT;

	if (!UsbdCtrlrEpXfer(epAddr, nullptr, 0))
	{
		UsbdCoreStallControl();
		return false;
	}

	return true;
}

static bool UsbdCoreStartIn(const uint8_t *pData, uint16_t Available)
{
	if (s_Setup.wLength == 0)
	{
		s_CtrlData = const_cast<uint8_t *>(pData);
		s_CtrlDataLen = 0;
		s_CtrlActual = 0;
		return UsbdCoreStartStatus();
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
	s_CtrlState = USBD_CORE_CTRL_DATA_IN;

	if (!UsbdCtrlrEpXfer(USB_ENDPADDR_DIR_IN, s_CtrlData, sendLen))
	{
		return false;
	}

	return true;
}

static bool UsbdCoreStartOut(uint8_t *pData, uint16_t Capacity)
{
	if (s_Setup.wLength == 0)
	{
		s_CtrlData = pData;
		s_CtrlDataLen = 0;
		s_CtrlActual = 0;
		return UsbdCoreStartStatus();
	}

	if (pData == nullptr || Capacity < s_Setup.wLength)
	{
		return false;
	}

	s_CtrlData = pData;
	s_CtrlDataLen = s_Setup.wLength;
	s_CtrlActual = 0;
	s_CtrlState = USBD_CORE_CTRL_DATA_OUT;

	if (!UsbdCtrlrEpXfer(USB_ENDPADDR_DIR_OUT, pData, s_Setup.wLength))
	{
		return false;
	}

	return true;
}

static bool UsbdCoreConfigRemoteWakeupCapable(void)
{
	uint16_t len;
	const uint8_t *pDesc = UsbdCoreActiveConfig(&len);

	return pDesc != nullptr && len >= USBD_CORE_CONFIG_DESC_LEN &&
		(pDesc[7] & USB_CONFATT_REMOTE_WAKEUP) != 0;
}

static bool UsbdCoreConfigSelfPowered(void)
{
	uint16_t len;
	const uint8_t *pDesc = UsbdCoreActiveConfig(&len);

	return pDesc != nullptr && len >= USBD_CORE_CONFIG_DESC_LEN &&
		(pDesc[7] & USB_CONFATT_SELF_POWERED) != 0;
}

static void UsbdCoreClearEndpointState(void)
{
	s_HaltIn = 0;
	s_HaltOut = 0;
	s_RemoteWakeup = false;
	memset(s_Alternate, 0, sizeof(s_Alternate));
}

static bool UsbdCoreWantSof(void)
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

static void UsbdCoreUnconfigureFunctions(void)
{
	if (s_Configuration == 0)
	{
		return;
	}

	UsbdCtrlrSofEnable(false);

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if (s_CoreFunc[i].ConfigHandler != nullptr)
		{
			(void)s_CoreFunc[i].ConfigHandler(0,
									   s_CoreFunc[i].pContext);
		}
	}
}

static bool UsbdCoreApplyConfiguration(uint8_t Configuration)
{
	uint16_t descLen = 0;
	const uint8_t *pConfigDesc = nullptr;

	if (Configuration != 0)
	{
		pConfigDesc = UsbdCoreGetConfigByValue(Configuration, &descLen);
		if (pConfigDesc == nullptr || descLen < USBD_CORE_CONFIG_DESC_LEN)
		{
			return false;
		}
	}

	UsbdCoreUnconfigureFunctions();
	UsbdCtrlrEpCloseAll();
	s_Configuration = 0;
	s_NumInterfaces = 0;
	UsbdCoreClearEndpointState();

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
			UsbdCtrlrEpCloseAll();
			return false;
		}
	}

	s_Configuration = Configuration;
	s_NumInterfaces = pConfigDesc[4];

	// Controllers drop SOF on bus reset, so enable it per configuration
	if (UsbdCoreWantSof())
	{
		UsbdCtrlrSofEnable(true);
	}

	return true;
}

static void UsbdCoreSetEndpointHalt(uint8_t EpAddr, bool Halt)
{
	const uint16_t bit = (uint16_t)(1U << USB_ENDPADDR_NUM(EpAddr));
	uint16_t *pMask = USB_ENDPADDR_IS_IN(EpAddr) ? &s_HaltIn : &s_HaltOut;

	if (Halt)
	{
		UsbdCtrlrEpStall(EpAddr);
		*pMask |= bit;
	}
	else
	{
		UsbdCtrlrEpClearStall(EpAddr);
		*pMask &= (uint16_t)~bit;
	}
}

static bool UsbdCoreEndpointHalted(uint8_t EpAddr)
{
	const uint16_t bit = (uint16_t)(1U << USB_ENDPADDR_NUM(EpAddr));
	const uint16_t mask = USB_ENDPADDR_IS_IN(EpAddr) ? s_HaltIn : s_HaltOut;

	return (mask & bit) != 0;
}

static bool UsbdCoreHandleGetDescriptor(void)
{
	if (!UsbdCoreDirIn(&s_Setup) ||
		UsbdCoreRecipient(&s_Setup) != USB_REQTYPE_DEVICE)
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
		pDesc = UsbdCoreGetConfigDescriptor(type, index, &len);
	}
	else
	{
		pDesc = UsbdCoreGetDescriptor(type, index, s_Setup.wIndex, &len);
	}

	if (pDesc == nullptr)
	{
		return false;
	}

	if (type == USB_DESCTYPE_DEVICE && len >= 8U &&
		UsbdCoreValidEp0Mps(pDesc[7]))
	{
		// Keep the control-transfer termination calculation aligned with the
		// descriptor advertised to the host.
		s_CoreCfg.Ep0Mps = pDesc[7];
	}

	return UsbdCoreStartIn(pDesc, len);
}

static bool UsbdCoreHandleGetStatus(void)
{
	if (!UsbdCoreDirIn(&s_Setup) || s_Setup.wValue != 0 ||
		s_Setup.wLength != 2)
	{
		return false;
	}

	uint16_t status = 0;
	const uint8_t recipient = UsbdCoreRecipient(&s_Setup);

	switch (recipient)
	{
		case USB_REQTYPE_DEVICE:
			if (s_Setup.wIndex != 0)
			{
				return false;
			}
			if (UsbdCoreConfigSelfPowered())
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
				!UsbdCoreInterfaceExists((uint8_t)s_Setup.wIndex))
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
				!UsbdCoreEndpointExists(epAddr))
			{
				return false;
			}
			if (UsbdCoreEndpointHalted(epAddr))
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
	return UsbdCoreStartIn(s_CtrlReply, 2);
}

static bool UsbdCoreHandleFeature(bool Set)
{
	if (UsbdCoreDirIn(&s_Setup) || s_Setup.wLength != 0)
	{
		return false;
	}

	const uint8_t recipient = UsbdCoreRecipient(&s_Setup);

	if (recipient == USB_REQTYPE_DEVICE &&
		s_Setup.wValue == USB_FEATSEL_DEVICE_REMOTE_WAKEUP &&
		s_Setup.wIndex == 0 && s_Configuration != 0 &&
		UsbdCoreConfigRemoteWakeupCapable())
	{
		s_RemoteWakeup = Set;
		return UsbdCoreStartStatus();
	}

	if (recipient == USB_REQTYPE_ENDPOINT &&
		s_Setup.wValue == USB_FEATSEL_ENDPOINT_HALT &&
		(s_Setup.wIndex & 0xFF00U) == 0)
	{
		const uint8_t epAddr = (uint8_t)s_Setup.wIndex;

		if ((epAddr & 0x70U) != 0U || USB_ENDPADDR_NUM(epAddr) == 0 ||
			s_Configuration == 0 || !UsbdCoreEndpointExists(epAddr))
		{
			return false;
		}

		UsbdCoreSetEndpointHalt(epAddr, Set);
		return UsbdCoreStartStatus();
	}

	return false;
}

static bool UsbdCoreHandleSetConfiguration(void)
{
	if (UsbdCoreDirIn(&s_Setup) ||
		UsbdCoreRecipient(&s_Setup) != USB_REQTYPE_DEVICE ||
		s_Setup.wIndex != 0 || s_Setup.wLength != 0 ||
		s_Setup.wValue > 0xFFU || s_Address == 0)
	{
		return false;
	}

	if (!UsbdCoreApplyConfiguration((uint8_t)s_Setup.wValue))
	{
		return false;
	}

	return UsbdCoreStartStatus();
}

static bool UsbdCoreHandleGetInterface(void)
{
	if (!UsbdCoreDirIn(&s_Setup) ||
		UsbdCoreRecipient(&s_Setup) != USB_REQTYPE_INTERFACE ||
		s_Setup.wValue != 0 || s_Setup.wLength != 1 ||
		s_Setup.wIndex > 0xFFU || s_Configuration == 0)
	{
		return false;
	}

	const uint8_t interfaceNo = (uint8_t)s_Setup.wIndex;
	if (interfaceNo >= USBD_CORE_INTRF_MAXCNT ||
		!UsbdCoreInterfaceExists(interfaceNo))
	{
		return false;
	}

	s_CtrlReply[0] = s_Alternate[interfaceNo];
	return UsbdCoreStartIn(s_CtrlReply, 1);
}

static bool UsbdCoreHandleSetInterface(void)
{
	if (UsbdCoreDirIn(&s_Setup) ||
		UsbdCoreRecipient(&s_Setup) != USB_REQTYPE_INTERFACE ||
		s_Setup.wLength != 0 || s_Setup.wIndex > 0xFFU ||
		s_Setup.wValue > 0xFFU || s_Configuration == 0)
	{
		return false;
	}

	const uint8_t interfaceNo = (uint8_t)s_Setup.wIndex;
	const uint8_t alternate = (uint8_t)s_Setup.wValue;

	if (interfaceNo >= USBD_CORE_INTRF_MAXCNT ||
		!UsbdCoreInterfaceExists(interfaceNo))
	{
		return false;
	}

	const int func = UsbdCoreFindFunction(interfaceNo);
	if (func >= 0 && s_CoreFunc[func].SetInterfaceHandler != nullptr)
	{
		if (!s_CoreFunc[func].SetInterfaceHandler(interfaceNo, alternate,
											 s_CoreFunc[func].pContext))
		{
			return false;
		}
	}
	else if (alternate != 0)
	{
		return false;
	}

	s_Alternate[interfaceNo] = alternate;
	return UsbdCoreStartStatus();
}

static bool UsbdCoreHandleStandard(void)
{
	switch (s_Setup.bRequest)
	{
		case USB_REQ_GET_STATUS:
			return UsbdCoreHandleGetStatus();

		case USB_REQ_CLEAR_FEATURE:
			return UsbdCoreHandleFeature(false);

		case USB_REQ_SET_FEATURE:
			return UsbdCoreHandleFeature(true);

		case USB_REQ_SET_ADDRESS:
			if (UsbdCoreDirIn(&s_Setup) ||
				UsbdCoreRecipient(&s_Setup) != USB_REQTYPE_DEVICE ||
				s_Setup.wIndex != 0 || s_Setup.wLength != 0 ||
				s_Setup.wValue > 127U || s_Configuration != 0)
			{
				return false;
			}
			s_PendingAddress = (uint8_t)s_Setup.wValue;
			s_AddressPending = true;
			UsbdCtrlrSetAddress(s_PendingAddress);
			return UsbdCoreStartStatus();

		case USB_REQ_GET_DESCRIPTOR:
			return UsbdCoreHandleGetDescriptor();

		case USB_REQ_GET_CONFIGURATION:
			if (!UsbdCoreDirIn(&s_Setup) ||
				UsbdCoreRecipient(&s_Setup) != USB_REQTYPE_DEVICE ||
				s_Setup.wValue != 0 || s_Setup.wIndex != 0 ||
				s_Setup.wLength != 1)
			{
				return false;
			}
			s_CtrlReply[0] = s_Configuration;
			return UsbdCoreStartIn(s_CtrlReply, 1);

		case USB_REQ_SET_CONFIGURATION:
			return UsbdCoreHandleSetConfiguration();

		case USB_REQ_GET_INTERFACE:
			return UsbdCoreHandleGetInterface();

		case USB_REQ_SET_INTERFACE:
			return UsbdCoreHandleSetInterface();

		default:
			return false;
	}
}

static bool UsbdCoreCallFunctionSetup(int Index)
{
	UsbdCoreRequestHandler_t handler = s_CoreFunc[Index].RequestHandler;
	if (handler == nullptr)
	{
		return false;
	}

	uint8_t *pData = nullptr;
	uint16_t len = 0;

	if (!handler(&s_Setup, USBD_CORE_CTRL_SETUP, &pData, &len,
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
		return UsbdCoreStartStatus();
	}

	if (UsbdCoreDirIn(&s_Setup))
	{
		return UsbdCoreStartIn(pData, len);
	}

	return UsbdCoreStartOut(pData, len);
}

static bool UsbdCoreHandleFunctionRequest(void)
{
	const uint8_t recipient = UsbdCoreRecipient(&s_Setup);

	if (recipient == USB_REQTYPE_INTERFACE && s_Setup.wIndex <= 0xFFU)
	{
		const int func = UsbdCoreFindFunction((uint8_t)s_Setup.wIndex);
		return func >= 0 && UsbdCoreCallFunctionSetup(func);
	}

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if (UsbdCoreCallFunctionSetup(i))
		{
			return true;
		}
	}

	return false;
}

static void UsbdCoreHandleSetup(const UsbSetupData_t *pSetup)
{
	if (pSetup == nullptr)
	{
		return;
	}

	UsbdCoreAbortControl();
	memcpy(&s_Setup, pSetup, sizeof(s_Setup));

	const uint8_t type = s_Setup.bmRequestType & USB_REQTYPE_MASK_TYPE;
	bool handled = false;

	if (type == USB_REQTYPE_STANDARD)
	{
		handled = UsbdCoreHandleStandard();
	}
	else if (type == USB_REQTYPE_CLASS || type == USB_REQTYPE_VEND)
	{
		handled = UsbdCoreHandleFunctionRequest();
	}

	if (!handled)
	{
		UsbdCoreStallControl();
	}
}

static void UsbdCoreHandleCtrlXfer(const UsbdCtrlrXferEvt_t *pXfer)
{
	if (pXfer == nullptr || pXfer->Result != USBD_CTRLR_XFER_SUCCESS)
	{
		UsbdCoreStallControl();
		return;
	}

	switch (s_CtrlState)
	{
		case USBD_CORE_CTRL_DATA_IN:
			s_CtrlActual = pXfer->Length;
			if (!UsbdCoreInvokeActive(USBD_CORE_CTRL_DATA, s_CtrlActual))
			{
				UsbdCoreStallControl();
				return;
			}

			if (s_CtrlNeedZlp)
			{
				s_CtrlNeedZlp = false;
				s_CtrlState = USBD_CORE_CTRL_DATA_IN_ZLP;
				if (!UsbdCtrlrEpXfer(USB_ENDPADDR_DIR_IN, nullptr, 0))
				{
					UsbdCoreStallControl();
				}
				return;
			}

			(void)UsbdCoreStartStatus();
			break;

		case USBD_CORE_CTRL_DATA_IN_ZLP:
			(void)UsbdCoreStartStatus();
			break;

		case USBD_CORE_CTRL_DATA_OUT:
			s_CtrlActual = pXfer->Length;
			if (!UsbdCoreInvokeActive(USBD_CORE_CTRL_DATA, s_CtrlActual))
			{
				UsbdCoreStallControl();
				return;
			}
			(void)UsbdCoreStartStatus();
			break;

		case USBD_CORE_CTRL_STATUS_IN:
		case USBD_CORE_CTRL_STATUS_OUT:
			if (s_AddressPending)
			{
				s_Address = s_PendingAddress;
			}
			(void)UsbdCoreInvokeActive(USBD_CORE_CTRL_COMPLETE,
									   s_CtrlActual);
			UsbdCoreResetControl();
			break;

		case USBD_CORE_CTRL_IDLE:
		default:
			break;
	}
}

static void UsbdCoreNotifyReset(void)
{
	UsbdCoreUnconfigureFunctions();

	for (int i = 0; i < s_CoreFuncCnt; i++)
	{
		if (s_CoreFunc[i].ResetHandler != nullptr)
		{
			s_CoreFunc[i].ResetHandler(s_CoreFunc[i].pContext);
		}
	}
}

static void UsbdCoreResetDeviceState(bool NotifyFunctions)
{
	UsbdCoreAbortControl();

	if (NotifyFunctions)
	{
		UsbdCoreNotifyReset();
	}

	s_Address = 0;
	s_Configuration = 0;
	s_NumInterfaces = 0;
	s_CoreSuspended = false;
	UsbdCoreClearEndpointState();
}

static void UsbdCoreCtrlrEvent(const UsbdCtrlrEvt_t *pEvt, void *)
{
	if (pEvt == nullptr)
	{
		return;
	}

	switch (pEvt->Type)
	{
		case USBD_CTRLR_EVT_RESET:
			UsbdCoreResetDeviceState(true);
			break;

		case USBD_CTRLR_EVT_SETUP:
			UsbdCoreHandleSetup(&pEvt->Setup);
			break;

		case USBD_CTRLR_EVT_XFER_CMPL:
			if (USB_ENDPADDR_NUM(pEvt->Xfer.EpAddr) == 0)
			{
				UsbdCoreHandleCtrlXfer(&pEvt->Xfer);
			}
			else
			{
				const int func = UsbdCoreFindEndpointFunction(pEvt->Xfer.EpAddr);
				if (func >= 0 && s_CoreFunc[func].XferHandler != nullptr)
				{
					s_CoreFunc[func].XferHandler(pEvt->Xfer.EpAddr,
										 pEvt->Xfer.Length,
										 pEvt->Xfer.Result,
										 s_CoreFunc[func].pContext);
				}
			}
			break;

		case USBD_CTRLR_EVT_SUSPEND:
			s_CoreSuspended = true;
			break;

		case USBD_CTRLR_EVT_RESUME:
			s_CoreSuspended = false;
			break;

		case USBD_CTRLR_EVT_ADDRESS:
			s_Address = pEvt->Address;
			break;

		case USBD_CTRLR_EVT_SOF:
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

bool UsbdCoreInit(const UsbdCoreCfg_t *pCfg)
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
	else if (!UsbdCoreValidEp0Mps(s_CoreCfg.Ep0Mps))
	{
		return false;
	}

	memset(s_CoreFunc, 0, sizeof(s_CoreFunc));
	s_CoreFuncCnt = 0;
	s_CoreInitialized = false;
	s_CoreStarted = false;
	UsbdCoreResetDeviceState(false);

	if (!UsbdCtrlrInit(UsbdCoreCtrlrEvent, nullptr))
	{
		return false;
	}

	s_CoreInitialized = true;
	return true;
}

bool UsbdCoreRegisterFunction(const UsbdCoreFuncCfg_t *pCfg)
{
	if (!s_CoreInitialized || pCfg == nullptr || s_CoreStarted ||
		s_CoreFuncCnt >= USBD_CORE_FUNC_MAXCNT)
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
		if (last > USBD_CORE_INTRF_MAXCNT)
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
	s_CoreFuncCnt++;
	return true;
}

void UsbdCoreStart(void)
{
	if (!s_CoreInitialized || s_CoreStarted)
	{
		return;
	}

	UsbdCtrlrIntEnable();
	UsbdCtrlrConnect();
	s_CoreStarted = true;
}

void UsbdCoreStop(void)
{
	if (!s_CoreStarted)
	{
		return;
	}

	UsbdCtrlrDisconnect();
	UsbdCtrlrIntDisable();
	UsbdCtrlrEpCloseAll();
	UsbdCoreResetDeviceState(true);
	s_CoreStarted = false;
}

bool UsbdCoreRemoteWakeup(void)
{
	if (!s_CoreStarted || !s_CoreSuspended || !s_RemoteWakeup)
	{
		return false;
	}

	UsbdCtrlrRemoteWakeup();
	return true;
}

bool UsbdCoreConfigured(void)
{
	return s_CoreStarted && s_Configuration != 0;
}

bool UsbdCoreSuspended(void)
{
	return s_CoreStarted && s_CoreSuspended;
}

uint8_t UsbdCoreAddress(void)
{
	return s_Address;
}

uint8_t UsbdCoreConfiguration(void)
{
	return s_Configuration;
}

uint8_t UsbdCoreAlternate(uint8_t InterfaceNo)
{
	return InterfaceNo < USBD_CORE_INTRF_MAXCNT ?
		s_Alternate[InterfaceNo] : 0;
}

bool UsbdCoreRemoteWakeupEnabled(void)
{
	return s_RemoteWakeup;
}

/**-------------------------------------------------------------------------
@file	usbd_hci.cpp

@brief	USB Bluetooth HCI device function implementation.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

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
#include <limits.h>
#include <string.h>

#include "coredev/interrupt.h"
#include "usb_func.h"
#include "usb/usbd_hci.h"

#ifndef USB_ISO_EPIN_MASK
#define USB_ISO_EPIN_MASK(CtrlrNo) \
	((uint16_t)(((1UL << USB_EPIN_CNT(CtrlrNo)) - 1UL) & ~1UL))
#endif
#ifndef USB_ISO_EPOUT_MASK
#define USB_ISO_EPOUT_MASK(CtrlrNo) \
	((uint16_t)(((1UL << USB_EPOUT_CNT(CtrlrNo)) - 1UL) & ~1UL))
#endif

static uint8_t *UsbdHciCommandBuffer(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->CommandBuffer);
}

static uint8_t *UsbdHciAclRxBuffer(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->AclRxBuffer);
}

static uint8_t *UsbdHciAclRxTransfer(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->AclRxTransfer);
}

static uint8_t *UsbdHciAclTxTransfer(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->AclTxTransfer);
}

static UsbPkt_t *UsbdHciAclTxPacket(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<UsbPkt_t *>(pHci->AclTxPacket);
}

static uint8_t *UsbdHciEventTxBuffer(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->EventTxBuffer);
}

static uint8_t *UsbdHciEventTxTransfer(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->EventTxTransfer);
}

static uint8_t *UsbdHciScoRxBuffer(UsbdHciDev_t *pHci, uint8_t Index)
{
	return reinterpret_cast<uint8_t *>(pHci->ScoRxBuffer[Index]);
}

static uint8_t *UsbdHciScoRxTransfer(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->ScoRxTransfer);
}

static uint8_t *UsbdHciScoTxBuffer(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->ScoTxBuffer);
}

static uint8_t *UsbdHciScoTxTransfer(UsbdHciDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->ScoTxTransfer);
}

static const uint8_t s_UsbdHciScoMps[USBD_HCI_SCO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};

static uint8_t UsbdHciScoMps(uint8_t Alt)
{
	return Alt > 0U && Alt <= USBD_HCI_SCO_ALT_COUNT ?
		s_UsbdHciScoMps[Alt - 1U] : 0U;
}

static uint8_t UsbdHciFirstEndpoint(uint16_t Mask)
{
	for (uint8_t ep = 1U; ep < 16U; ep++)
	{
		if ((Mask & (uint16_t)(1U << ep)) != 0U)
		{
			return ep;
		}
	}
	return 0U;
}

static uint16_t UsbdHciReadLe16(const uint8_t *pData)
{
	return (uint16_t)pData[0] | ((uint16_t)pData[1] << 8);
}

static size_t UsbdHciPacketLength(UsbdHciPacketType_t Type,
								  const uint8_t *pPacket, size_t Available)
{
	if (pPacket == nullptr)
	{
		return 0U;
	}

	switch (Type)
	{
		case USBD_HCI_PACKET_COMMAND:
			return Available >= USBD_HCI_COMMAND_HEADER_SIZE ?
				USBD_HCI_COMMAND_HEADER_SIZE + (size_t)pPacket[2] : 0U;

		case USBD_HCI_PACKET_EVENT:
			return Available >= USBD_HCI_EVENT_HEADER_SIZE ?
				USBD_HCI_EVENT_HEADER_SIZE + (size_t)pPacket[1] : 0U;

		case USBD_HCI_PACKET_ACL:
			return Available >= USBD_HCI_ACL_HEADER_SIZE ?
				USBD_HCI_ACL_HEADER_SIZE +
				(size_t)UsbdHciReadLe16(&pPacket[2]) : 0U;

		case USBD_HCI_PACKET_SCO:
			return Available >= USBD_HCI_SCO_HEADER_SIZE ?
				USBD_HCI_SCO_HEADER_SIZE + (size_t)pPacket[2] : 0U;

		default:
			return 0U;
	}
}

static uint16_t UsbdHciEventMps(const UsbdHciDev_t *pHci)
{
	return UsbCtrlrHighSpeed(pHci->DevNo) ? pHci->EventHsMps : pHci->EventFsMps;
}

static uint16_t UsbdHciAclMps(const UsbdHciDev_t *pHci)
{
	return UsbCtrlrHighSpeed(pHci->DevNo) ? pHci->AclHsMps : pHci->AclFsMps;
}

static uint8_t UsbdHciEventInterval(const UsbdHciDev_t *pHci)
{
	return UsbCtrlrHighSpeed(pHci->DevNo) ?
		pHci->EventHsInterval : pHci->EventFsInterval;
}

static bool UsbdHciOpenEndpoint(UsbdHciDev_t *pHci, uint8_t EpAddr,
								 uint8_t TransferType, uint16_t Mps,
								 uint8_t Interval)
{
	UsbEndPointDesc_t desc = {};
	desc.bLength = sizeof(desc);
	desc.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.bEndpointAddress = EpAddr;
	desc.bmAttributes = TransferType;
	desc.wMaxPacketSize = Mps;
	desc.bInterval = Interval;

	return UsbCtrlrEpOpen(pHci->DevNo, &desc);
}

static void UsbdHciCloseEndpoints(UsbdHciDev_t *pHci)
{
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->EventEpNo));
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIROUT(pHci->AclEpNo));
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->AclEpNo));
}

static void UsbdHciClearSco(UsbdHciDev_t *pHci)
{
	pHci->ScoAlt = 0U;
	pHci->ScoTxActive = false;
	pHci->ScoRxBuildIndex = 0U;
	pHci->ScoRxPendingIndex = USBD_HCI_SCO_BUFFER_NONE;
	pHci->ScoRxReadIndex = USBD_HCI_SCO_BUFFER_NONE;
	pHci->ScoRxLength = 0U;
	pHci->ScoRxExpected = 0U;
	pHci->ScoTxLength = 0U;
	pHci->ScoTxOffset = 0U;
	pHci->ScoTxChunkLength = 0U;
	memset(pHci->ScoRxPacketLength, 0, sizeof(pHci->ScoRxPacketLength));
}

static void UsbdHciCloseScoEndpoints(UsbdHciDev_t *pHci)
{
	const uint8_t oldAlt = pHci->ScoAlt;
	UsbdHciClearSco(pHci);
	if (oldAlt != 0U)
	{
		UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIROUT(pHci->ScoEpNo));
		UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->ScoEpNo));
	}
}

static void UsbdHciClearTransport(UsbdHciDev_t *pHci)
{
	pHci->RxType = USBD_HCI_PACKET_NONE;
	pHci->TxType = USBD_HCI_PACKET_NONE;
	pHci->CommandPending = false;
	pHci->AclRxPending = false;
	pHci->EventTxActive = false;
	pHci->EventTxNeedZlp = false;
	pHci->EventTxZlp = false;
	pHci->CommandLength = 0U;
	pHci->AclRxLength = 0U;
	pHci->AclRxExpected = 0U;
	pHci->EventTxLength = 0U;
	pHci->EventTxOffset = 0U;
	pHci->EventTxChunkLength = 0U;
	UsbdHciClearSco(pHci);
}

static void UsbdHciUnconfigure(UsbdHciDev_t *pHci)
{
	pHci->Configured = false;
	UsbIntrfUnconfigure(pHci->pAcl);
	UsbdHciClearTransport(pHci);
}

static bool UsbdHciConfig(uint8_t Configuration, void *pContext)
{
	UsbdHciDev_t *pHci = static_cast<UsbdHciDev_t *>(pContext);

	if (pHci == nullptr)
	{
		return false;
	}

	UsbdHciUnconfigure(pHci);
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != USBD_HCI_CONFIG_VALUE)
	{
		return false;
	}

	const uint16_t eventMps = UsbdHciEventMps(pHci);
	const uint16_t aclMps = UsbdHciAclMps(pHci);
	const uint8_t eventInterval = UsbdHciEventInterval(pHci);

	if (!UsbdHciOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->EventEpNo),
							 USB_ENDPATT_TRANS_INT, eventMps, eventInterval) ||
		!UsbdHciOpenEndpoint(pHci, USB_ENDPADDR_DIROUT(pHci->AclEpNo),
							 USB_ENDPATT_TRANS_BULK, aclMps, 0U) ||
		!UsbdHciOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->AclEpNo),
							 USB_ENDPATT_TRANS_BULK, aclMps, 0U) ||
		!UsbIntrfConfigure(pHci->pAcl, aclMps))
	{
		UsbdHciUnconfigure(pHci);
		UsbdHciCloseEndpoints(pHci);
		return false;
	}

	pHci->Configured = true;
	return true;
}

static bool UsbdHciSetInterface(uint8_t InterfaceNo, uint8_t Alt,
								void *pContext)
{
	UsbdHciDev_t *pHci = static_cast<UsbdHciDev_t *>(pContext);
	if (pHci == nullptr || !pHci->Configured)
	{
		return false;
	}
	if (InterfaceNo == (uint8_t)pHci->HciItfNo)
	{
		return Alt == 0U;
	}
	if (InterfaceNo != (uint8_t)pHci->SyncItfNo ||
		Alt > USBD_HCI_SCO_ALT_COUNT || (!pHci->ScoEnabled && Alt != 0U))
	{
		return false;
	}

	UsbdHciCloseScoEndpoints(pHci);
	if (Alt == 0U)
	{
		return true;
	}

	const uint16_t mps = UsbdHciScoMps(Alt);
	const uint8_t interval = UsbCtrlrHighSpeed(pHci->DevNo) ?
		USBD_HCI_SCO_HS_INTERVAL : USBD_HCI_SCO_FS_INTERVAL;
	if (!UsbdHciOpenEndpoint(pHci, USB_ENDPADDR_DIROUT(pHci->ScoEpNo),
			USB_ENDPATT_TRANS_ISO, mps, interval) ||
		!UsbdHciOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->ScoEpNo),
			USB_ENDPATT_TRANS_ISO, mps, interval))
	{
		UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIROUT(pHci->ScoEpNo));
		UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->ScoEpNo));
		return false;
	}

	pHci->ScoAlt = Alt;
	if (!UsbCtrlrEpRxArm(pHci->DevNo, pHci->ScoEpNo))
	{
		UsbdHciCloseScoEndpoints(pHci);
		return false;
	}
	return true;
}

static bool UsbdHciRequestValid(const UsbdHciDev_t *pHci,
								const UsbSetupData_t *pSetup)
{
	if (!pHci->Configured ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) != USB_REQTYPE_DIRDEV ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE) != USB_REQTYPE_CLASS ||
		pSetup->bRequest != 0U || pSetup->wValue != 0U ||
		pSetup->wLength == 0U ||
		pSetup->wLength > USBD_HCI_COMMAND_MAX_SIZE)
	{
		return false;
	}

	const uint8_t recipient =
		pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT;
	if (recipient == USB_REQTYPE_INTERFACE)
	{
		return (pSetup->wIndex & 0xFF00U) == 0U &&
			(uint8_t)pSetup->wIndex == (uint8_t)pHci->HciItfNo;
	}

	return recipient == USB_REQTYPE_DEVICE;
}

static bool UsbdHciRequest(const UsbSetupData_t *pSetup,
						   UsbCtrlStage_t Stage, uint8_t **ppData,
						   uint16_t *pLength, void *pContext)
{
	UsbdHciDev_t *pHci = static_cast<UsbdHciDev_t *>(pContext);
	if (pHci == nullptr || pSetup == nullptr || pLength == nullptr ||
		!UsbdHciRequestValid(pHci, pSetup))
	{
		return false;
	}

	if (Stage == USB_CTRL_SETUP)
	{
		if (ppData == nullptr || pHci->CommandPending)
		{
			return false;
		}
		*ppData = UsbdHciCommandBuffer(pHci);
		*pLength = pSetup->wLength;
		return true;
	}

	if (Stage == USB_CTRL_DATA)
	{
		return *pLength == pSetup->wLength &&
			UsbdHciPacketLength(USBD_HCI_PACKET_COMMAND,
				UsbdHciCommandBuffer(pHci), *pLength) == *pLength;
	}

	if (Stage == USB_CTRL_COMPLETE)
	{
		pHci->CommandLength = pSetup->wLength;
		pHci->CommandPending = true;
		if (pHci->pAcl->DevIntrf.EvtCB != nullptr)
		{
			pHci->pAcl->DevIntrf.EvtCB(&pHci->pAcl->DevIntrf,
								 DEVINTRF_EVT_RX_DATA, nullptr,
								 pHci->CommandLength);
		}
		return true;
	}

	if (Stage == USB_CTRL_ABORT)
	{
		pHci->CommandLength = 0U;
		return true;
	}

	return false;
}

static void UsbdHciClearEventTx(UsbdHciDev_t *pHci)
{
	pHci->EventTxActive = false;
	pHci->EventTxNeedZlp = false;
	pHci->EventTxZlp = false;
	pHci->EventTxLength = 0U;
	pHci->EventTxOffset = 0U;
	pHci->EventTxChunkLength = 0U;
}

static void UsbdHciEventTxFailure(UsbdHciDev_t *pHci, uint16_t Length)
{
	UsbdHciClearEventTx(pHci);
	if (pHci->pAcl->DevIntrf.EvtCB != nullptr)
	{
		pHci->pAcl->DevIntrf.EvtCB(&pHci->pAcl->DevIntrf,
								 DEVINTRF_EVT_TX_TIMEOUT, nullptr, Length);
	}
}

static bool UsbdHciSendEventChunk(UsbdHciDev_t *pHci)
{
	const uint16_t remaining =
		(uint16_t)(pHci->EventTxLength - pHci->EventTxOffset);
	const uint16_t mps = UsbdHciEventMps(pHci);
	pHci->EventTxChunkLength = remaining < mps ? remaining : mps;
	pHci->EventTxZlp = false;
	memcpy(UsbdHciEventTxTransfer(pHci),
		&UsbdHciEventTxBuffer(pHci)[pHci->EventTxOffset],
		pHci->EventTxChunkLength);
	return UsbCtrlrEpSend(pHci->DevNo, pHci->EventEpNo,
		pHci->EventTxChunkLength);
}

static bool UsbdHciSendEventZlp(UsbdHciDev_t *pHci)
{
	pHci->EventTxChunkLength = 0U;
	pHci->EventTxZlp = true;
	return UsbCtrlrEpSend(pHci->DevNo, pHci->EventEpNo, 0U);
}

static void UsbdHciEventComplete(uint8_t, uint16_t Length,
								 UsbCtrlrXferResult_t Result, void *pContext)
{
	UsbdHciDev_t *pHci = static_cast<UsbdHciDev_t *>(pContext);
	if (pHci == nullptr || !pHci->EventTxActive)
	{
		return;
	}

	const uint16_t expected = pHci->EventTxChunkLength;
	if (Result != USB_CTRLR_XFER_SUCCESS || Length != expected)
	{
		UsbdHciEventTxFailure(pHci, Length);
		return;
	}

	if (!pHci->EventTxZlp)
	{
		pHci->EventTxOffset = (uint16_t)(pHci->EventTxOffset +
			pHci->EventTxChunkLength);
		if (pHci->EventTxOffset < pHci->EventTxLength)
		{
			if (UsbdHciSendEventChunk(pHci))
			{
				return;
			}
			UsbdHciEventTxFailure(pHci, 0U);
			return;
		}

		if (pHci->EventTxNeedZlp)
		{
			if (UsbdHciSendEventZlp(pHci))
			{
				return;
			}
			UsbdHciEventTxFailure(pHci, 0U);
			return;
		}
	}

	UsbdHciClearEventTx(pHci);
	if (pHci->pAcl->DevIntrf.EvtCB != nullptr)
	{
		pHci->pAcl->DevIntrf.EvtCB(&pHci->pAcl->DevIntrf,
							 DEVINTRF_EVT_TX_READY, nullptr, 0);
	}
}

static uint8_t UsbdHciScoFreeBuffer(const UsbdHciDev_t *pHci,
									uint8_t Pending)
{
	for (uint8_t i = 0U; i < USBD_HCI_SCO_RX_BUFFER_COUNT; i++)
	{
		if (i != Pending && i != pHci->ScoRxReadIndex)
		{
			return i;
		}
	}
	return 0U;
}

static void UsbdHciScoPublish(UsbdHciDev_t *pHci)
{
	const uint32_t state = DisableInterrupt();
	const uint8_t complete = pHci->ScoRxBuildIndex;
	pHci->ScoRxPacketLength[complete] = pHci->ScoRxExpected;
	pHci->ScoRxPendingIndex = complete;
	pHci->ScoRxBuildIndex = UsbdHciScoFreeBuffer(pHci, complete);
	pHci->ScoRxPacketLength[pHci->ScoRxBuildIndex] = 0U;
	pHci->ScoRxLength = 0U;
	pHci->ScoRxExpected = 0U;
	EnableInterrupt(state);

	if (pHci->pAcl->DevIntrf.EvtCB != nullptr)
	{
		pHci->pAcl->DevIntrf.EvtCB(&pHci->pAcl->DevIntrf,
			DEVINTRF_EVT_RX_DATA, nullptr,
			pHci->ScoRxPacketLength[complete]);
	}
}

static void UsbdHciScoReceiveComplete(UsbdHciDev_t *pHci, uint16_t Length,
									  UsbCtrlrXferResult_t Result)
{
	if (pHci->ScoAlt == 0U)
	{
		return;
	}

	const uint16_t mps = UsbdHciScoMps(pHci->ScoAlt);
	if (Result != USB_CTRLR_XFER_SUCCESS || Length > mps ||
		(size_t)pHci->ScoRxLength + Length > USBD_HCI_SCO_MAX_SIZE)
	{
		pHci->ScoRxLength = 0U;
		pHci->ScoRxExpected = 0U;
	}
	else if (Length != 0U)
	{
		memcpy(&UsbdHciScoRxBuffer(pHci, pHci->ScoRxBuildIndex)
			[pHci->ScoRxLength], UsbdHciScoRxTransfer(pHci), Length);
		pHci->ScoRxLength = (uint16_t)(pHci->ScoRxLength + Length);

		if (pHci->ScoRxExpected == 0U &&
			pHci->ScoRxLength >= USBD_HCI_SCO_HEADER_SIZE)
		{
			pHci->ScoRxExpected = (uint16_t)UsbdHciPacketLength(
				USBD_HCI_PACKET_SCO,
				UsbdHciScoRxBuffer(pHci, pHci->ScoRxBuildIndex),
				pHci->ScoRxLength);
		}

		if (pHci->ScoRxExpected != 0U &&
			pHci->ScoRxLength == pHci->ScoRxExpected)
		{
			UsbdHciScoPublish(pHci);
		}
		else if ((pHci->ScoRxExpected != 0U &&
				  pHci->ScoRxLength > pHci->ScoRxExpected) || Length < mps)
		{
			pHci->ScoRxLength = 0U;
			pHci->ScoRxExpected = 0U;
		}
	}
	else if (pHci->ScoRxLength != 0U)
	{
		pHci->ScoRxLength = 0U;
		pHci->ScoRxExpected = 0U;
	}

	(void)UsbCtrlrEpRxArm(pHci->DevNo, pHci->ScoEpNo);
}

static bool UsbdHciSendScoChunk(UsbdHciDev_t *pHci)
{
	const uint16_t remaining =
		(uint16_t)(pHci->ScoTxLength - pHci->ScoTxOffset);
	const uint16_t mps = UsbdHciScoMps(pHci->ScoAlt);
	pHci->ScoTxChunkLength = remaining < mps ? remaining : mps;
	memcpy(UsbdHciScoTxTransfer(pHci),
		&UsbdHciScoTxBuffer(pHci)[pHci->ScoTxOffset],
		pHci->ScoTxChunkLength);
	return UsbCtrlrEpSend(pHci->DevNo, pHci->ScoEpNo,
		pHci->ScoTxChunkLength);
}

static void UsbdHciScoSendComplete(UsbdHciDev_t *pHci, uint16_t Length,
								   UsbCtrlrXferResult_t Result)
{
	if (!pHci->ScoTxActive)
	{
		return;
	}
	if (Result != USB_CTRLR_XFER_SUCCESS ||
		Length != pHci->ScoTxChunkLength)
	{
		pHci->ScoTxActive = false;
		if (pHci->pAcl->DevIntrf.EvtCB != nullptr)
		{
			pHci->pAcl->DevIntrf.EvtCB(&pHci->pAcl->DevIntrf,
				DEVINTRF_EVT_TX_TIMEOUT, nullptr, Length);
		}
		return;
	}

	pHci->ScoTxOffset = (uint16_t)(pHci->ScoTxOffset + Length);
	if (pHci->ScoTxOffset < pHci->ScoTxLength)
	{
		if (!UsbdHciSendScoChunk(pHci))
		{
			UsbdHciScoSendComplete(pHci, 0U, USB_CTRLR_XFER_FAILED);
		}
		return;
	}

	pHci->ScoTxActive = false;
	pHci->ScoTxLength = 0U;
	pHci->ScoTxOffset = 0U;
	pHci->ScoTxChunkLength = 0U;
	if (pHci->pAcl->DevIntrf.EvtCB != nullptr)
	{
		pHci->pAcl->DevIntrf.EvtCB(&pHci->pAcl->DevIntrf,
			DEVINTRF_EVT_TX_READY, nullptr, 0);
	}
}

static void UsbdHciXfer(uint8_t EpAddr, uint16_t Length,
						UsbCtrlrXferResult_t Result, void *pContext)
{
	UsbdHciDev_t *pHci = static_cast<UsbdHciDev_t *>(pContext);
	if (pHci == nullptr)
	{
		return;
	}

	if (USB_ENDPADDR_NUM(EpAddr) == pHci->AclEpNo)
	{
		UsbIntrfXferComplete(pHci->pAcl, EpAddr, Length, Result);
	}
	else if (EpAddr == USB_ENDPADDR_DIRIN(pHci->EventEpNo))
	{
		UsbdHciEventComplete(EpAddr, Length, Result, pHci);
	}
	else if (USB_ENDPADDR_NUM(EpAddr) == pHci->ScoEpNo)
	{
		if (USB_ENDPADDR_IS_IN(EpAddr))
		{
			UsbdHciScoSendComplete(pHci, Length, Result);
		}
		else
		{
			UsbdHciScoReceiveComplete(pHci, Length, Result);
		}
	}
}

static void UsbdHciReset(void *pContext)
{
	UsbdHciDev_t *pHci = static_cast<UsbdHciDev_t *>(pContext);
	if (pHci != nullptr)
	{
		UsbdHciUnconfigure(pHci);
	}
}

static UsbdHciDev_t *UsbdHciFromDev(DevIntrf_t *pDev)
{
	if (pDev == nullptr || pDev->pDevData == nullptr)
	{
		return nullptr;
	}
	UsbDevIntrf_t *pAcl = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	return static_cast<UsbdHciDev_t *>(pAcl->pClassContext);
}

static bool UsbdHciDevStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	UsbdHciDev_t *pHci = UsbdHciFromDev(pDev);
	if (pHci == nullptr || !pHci->Configured ||
		(DevAddr != USBD_HCI_PACKET_COMMAND &&
		 DevAddr != USBD_HCI_PACKET_ACL &&
		 (DevAddr != USBD_HCI_PACKET_SCO || pHci->ScoAlt == 0U)))
	{
		return false;
	}

	pHci->RxType = (UsbdHciPacketType_t)DevAddr;
	return true;
}

static void UsbdHciDropPhysicalAcl(UsbdHciDev_t *pHci)
{
	const uint32_t state = DisableInterrupt();
	(void)CFifoGet(pHci->pAcl->hRxFifo);
	if (pHci->pAcl->Mps != 0U && CFifoAvail(pHci->pAcl->hRxFifo) > 0)
	{
		(void)UsbCtrlrEpRxArm(pHci->DevNo, pHci->AclEpNo);
	}
	EnableInterrupt(state);
}

static bool UsbdHciConsumeAcl(UsbdHciDev_t *pHci)
{
	UsbPkt_t *pPacket =
		reinterpret_cast<UsbPkt_t *>(CFifoPeek(pHci->pAcl->hRxFifo));
	if (pPacket == nullptr)
	{
		return false;
	}

	const uint16_t length = pPacket->Hdr.Length;
	if (length == 0U || length > pHci->pAcl->Mps ||
		(size_t)pHci->AclRxLength + length > USBD_HCI_PACKET_MAX_SIZE)
	{
		UsbdHciDropPhysicalAcl(pHci);
		pHci->AclRxLength = 0U;
		pHci->AclRxExpected = 0U;
		return true;
	}

	const int count = pHci->AclRxData(&pHci->pAcl->DevIntrf,
		&UsbdHciAclRxBuffer(pHci)[pHci->AclRxLength], length);
	if (count != length)
	{
		pHci->AclRxLength = 0U;
		pHci->AclRxExpected = 0U;
		return count > 0;
	}

	pHci->AclRxLength = (uint16_t)(pHci->AclRxLength + length);
	if (pHci->AclRxExpected == 0U &&
		pHci->AclRxLength >= USBD_HCI_ACL_HEADER_SIZE)
	{
		const size_t expected = UsbdHciPacketLength(USBD_HCI_PACKET_ACL,
			UsbdHciAclRxBuffer(pHci), pHci->AclRxLength);
		if (expected > USBD_HCI_PACKET_MAX_SIZE)
		{
			pHci->AclRxLength = 0U;
			return true;
		}
		pHci->AclRxExpected = (uint16_t)expected;
	}

	if (pHci->AclRxExpected != 0U &&
		pHci->AclRxLength == pHci->AclRxExpected)
	{
		pHci->AclRxPending = true;
		return true;
	}

	if ((pHci->AclRxExpected != 0U &&
		 pHci->AclRxLength > pHci->AclRxExpected) ||
		(length < pHci->pAcl->Mps))
	{
		pHci->AclRxLength = 0U;
		pHci->AclRxExpected = 0U;
	}

	return true;
}

static int UsbdHciDevRxData(DevIntrf_t * const pDev, uint8_t *pBuffer,
							 int BufferLen)
{
	UsbdHciDev_t *pHci = UsbdHciFromDev(pDev);
	if (pHci == nullptr || pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	if (pHci->RxType == USBD_HCI_PACKET_COMMAND && pHci->CommandPending)
	{
		if (BufferLen < pHci->CommandLength)
		{
			return 0;
		}
		memcpy(pBuffer, UsbdHciCommandBuffer(pHci), pHci->CommandLength);
		const int count = pHci->CommandLength;
		pHci->CommandPending = false;
		pHci->CommandLength = 0U;
		return count;
	}
	if (pHci->RxType == USBD_HCI_PACKET_SCO && pHci->ScoAlt != 0U)
	{
		const uint32_t state = DisableInterrupt();
		const uint8_t index = pHci->ScoRxPendingIndex;
		const uint16_t length = index < USBD_HCI_SCO_RX_BUFFER_COUNT ?
			pHci->ScoRxPacketLength[index] : 0U;
		if (index >= USBD_HCI_SCO_RX_BUFFER_COUNT || BufferLen < length)
		{
			EnableInterrupt(state);
			return 0;
		}
		pHci->ScoRxPendingIndex = USBD_HCI_SCO_BUFFER_NONE;
		pHci->ScoRxReadIndex = index;
		EnableInterrupt(state);

		memcpy(pBuffer, UsbdHciScoRxBuffer(pHci, index), length);
		const uint32_t doneState = DisableInterrupt();
		pHci->ScoRxReadIndex = USBD_HCI_SCO_BUFFER_NONE;
		EnableInterrupt(doneState);
		return length;
	}

	if (pHci->RxType != USBD_HCI_PACKET_ACL)
	{
		return 0;
	}

	while (!pHci->AclRxPending && UsbdHciConsumeAcl(pHci))
	{
	}
	if (!pHci->AclRxPending || BufferLen < pHci->AclRxExpected)
	{
		return 0;
	}

	memcpy(pBuffer, UsbdHciAclRxBuffer(pHci), pHci->AclRxExpected);
	const int count = pHci->AclRxExpected;
	pHci->AclRxPending = false;
	pHci->AclRxLength = 0U;
	pHci->AclRxExpected = 0U;
	return count;
}

static bool UsbdHciDevStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	UsbdHciDev_t *pHci = UsbdHciFromDev(pDev);
	if (pHci == nullptr || !pHci->Configured ||
		(DevAddr != USBD_HCI_PACKET_EVENT &&
		 DevAddr != USBD_HCI_PACKET_ACL &&
		 (DevAddr != USBD_HCI_PACKET_SCO || pHci->ScoAlt == 0U)))
	{
		return false;
	}

	pHci->TxType = (UsbdHciPacketType_t)DevAddr;
	return true;
}

static int UsbdHciQueueAcl(UsbdHciDev_t *pHci, const uint8_t *pData,
						   int DataLen)
{
	const uint16_t mps = pHci->pAcl->Mps;
	const size_t packetCount = ((size_t)DataLen + mps - 1U) / mps;
	const bool needZlp = ((unsigned)DataLen % mps) == 0U;
	const size_t blocks = packetCount + (needZlp ? 1U : 0U);
	if (blocks > INT_MAX ||
		!UsbIntrfRequestToSend(pHci->pAcl,
			(int)(blocks * USBD_HCI_ACL_PKT_BLKSIZE)))
	{
		return 0;
	}

	UsbPkt_t *pPacket = UsbdHciAclTxPacket(pHci);
	pPacket->Hdr.Reserved = 0U;
	int offset = 0;
	while (offset < DataLen)
	{
		const int remaining = DataLen - offset;
		const uint16_t length = (uint16_t)(remaining < mps ? remaining : mps);
		pPacket->Hdr.Length = length;
		memcpy(pPacket->Data, &pData[offset], length);
		if (pHci->AclTxData(&pHci->pAcl->DevIntrf,
			reinterpret_cast<uint8_t *>(pPacket),
			USBD_HCI_ACL_PKT_BLKSIZE) != (int)USBD_HCI_ACL_PKT_BLKSIZE)
		{
			return 0;
		}
		offset += length;
	}

	if (needZlp)
	{
		pPacket->Hdr.Length = 0U;
		if (pHci->AclTxData(&pHci->pAcl->DevIntrf,
			reinterpret_cast<uint8_t *>(pPacket),
			USBD_HCI_ACL_PKT_BLKSIZE) != (int)USBD_HCI_ACL_PKT_BLKSIZE)
		{
			return 0;
		}
	}

	return DataLen;
}

static int UsbdHciSendEvent(UsbdHciDev_t *pHci, const uint8_t *pData,
							int DataLen)
{
	if (pHci->EventTxActive)
	{
		return 0;
	}

	memcpy(UsbdHciEventTxBuffer(pHci), pData, DataLen);
	pHci->EventTxLength = (uint16_t)DataLen;
	pHci->EventTxOffset = 0U;
	pHci->EventTxNeedZlp =
		((unsigned)DataLen % UsbdHciEventMps(pHci)) == 0U;
	pHci->EventTxZlp = false;
	pHci->EventTxActive = true;
	if (!UsbdHciSendEventChunk(pHci))
	{
		UsbdHciClearEventTx(pHci);
		return 0;
	}

	return DataLen;
}

static int UsbdHciSendSco(UsbdHciDev_t *pHci, const uint8_t *pData,
						  int DataLen)
{
	if (pHci->ScoAlt == 0U || pHci->ScoTxActive)
	{
		return 0;
	}

	memcpy(UsbdHciScoTxBuffer(pHci), pData, DataLen);
	pHci->ScoTxLength = (uint16_t)DataLen;
	pHci->ScoTxOffset = 0U;
	pHci->ScoTxActive = true;
	if (!UsbdHciSendScoChunk(pHci))
	{
		pHci->ScoTxActive = false;
		return 0;
	}
	return DataLen;
}

static int UsbdHciDevTxData(DevIntrf_t * const pDev, const uint8_t *pData,
							 int DataLen)
{
	UsbdHciDev_t *pHci = UsbdHciFromDev(pDev);
	if (pHci == nullptr || pData == nullptr || DataLen <= 0 ||
		!pHci->Configured ||
		UsbdHciPacketLength(pHci->TxType, pData, DataLen) !=
			(size_t)DataLen)
	{
		return 0;
	}

	if (pHci->TxType == USBD_HCI_PACKET_EVENT &&
		DataLen <= (int)USBD_HCI_EVENT_MAX_SIZE)
	{
		return UsbdHciSendEvent(pHci, pData, DataLen);
	}
	if (pHci->TxType == USBD_HCI_PACKET_ACL &&
		DataLen <= (int)USBD_HCI_PACKET_MAX_SIZE)
	{
		return UsbdHciQueueAcl(pHci, pData, DataLen);
	}
	if (pHci->TxType == USBD_HCI_PACKET_SCO &&
		DataLen <= (int)USBD_HCI_SCO_MAX_SIZE)
	{
		return UsbdHciSendSco(pHci, pData, DataLen);
	}

	return 0;
}

static int UsbdHciDevTxSrData(DevIntrf_t * const pDev,
							   const uint8_t *pData, int DataLen)
{
	return UsbdHciDevTxData(pDev, pData, DataLen);
}

static void UsbdHciDevReset(DevIntrf_t * const pDev)
{
	UsbdHciDev_t *pHci = UsbdHciFromDev(pDev);
	if (pHci != nullptr)
	{
		UsbdHciUnconfigure(pHci);
	}
}

static void *UsbdHciDevGetHandle(DevIntrf_t * const pDev)
{
	return UsbdHciFromDev(pDev);
}

static void UsbdHciInitDevIntrf(UsbdHciDev_t *pHci)
{
	DevIntrf_t *pDev = &pHci->pAcl->DevIntrf;
	pHci->AclRxData = pDev->RxData;
	pHci->AclTxData = pDev->TxData;
	pHci->pAcl->pClassContext = pHci;
	pDev->StartRx = UsbdHciDevStartRx;
	pDev->RxData = UsbdHciDevRxData;
	pDev->StartTx = UsbdHciDevStartTx;
	pDev->TxData = UsbdHciDevTxData;
	pDev->TxSrData = UsbdHciDevTxSrData;
	pDev->Reset = UsbdHciDevReset;
	pDev->GetHandle = UsbdHciDevGetHandle;
}

bool UsbdHciRequestToSend(UsbdHciDev_t *pHci, int NbBytes)
{
	if (pHci == nullptr || !pHci->Configured || NbBytes <= 0)
	{
		return false;
	}
	if (pHci->TxType == USBD_HCI_PACKET_EVENT)
	{
		return !pHci->EventTxActive &&
			NbBytes <= (int)USBD_HCI_EVENT_MAX_SIZE;
	}
	if (pHci->TxType == USBD_HCI_PACKET_SCO)
	{
		return pHci->ScoAlt != 0U && !pHci->ScoTxActive &&
			NbBytes <= (int)USBD_HCI_SCO_MAX_SIZE;
	}
	if (pHci->TxType != USBD_HCI_PACKET_ACL ||
		NbBytes > (int)USBD_HCI_PACKET_MAX_SIZE)
	{
		return false;
	}

	const uint16_t mps = pHci->pAcl->Mps;
	const int packets = (NbBytes + mps - 1) / mps;
	const int blocks = packets + ((NbBytes % mps) == 0 ? 1 : 0);
	return UsbIntrfRequestToSend(pHci->pAcl,
		blocks * (int)USBD_HCI_ACL_PKT_BLKSIZE);
}

bool UsbdHciMakeDesc(UsbdHciDesc_t *pDesc, const UsbdHciDev_t *pHci,
					 UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pHci == nullptr ||
		pHci->HciItfNo < 0 || pHci->HciItfNo > UINT8_MAX - 1 ||
		pHci->SyncItfNo != pHci->HciItfNo + 1 ||
		pHci->EventEpNo == 0U || pHci->EventEpNo > 15U ||
		pHci->AclEpNo == 0U || pHci->AclEpNo > 15U)
	{
		return false;
	}

	const uint16_t eventMps = Speed == USB_SPEED_HIGH ?
		pHci->EventHsMps : pHci->EventFsMps;
	const uint16_t aclMps = Speed == USB_SPEED_HIGH ?
		pHci->AclHsMps : pHci->AclFsMps;
	const uint8_t eventInterval = Speed == USB_SPEED_HIGH ?
		pHci->EventHsInterval : pHci->EventFsInterval;

	if (eventMps == 0U || eventMps > USB_PKT_MAXLEN(0, INT) ||
		aclMps == 0U || aclMps > USB_PKT_MAXLEN(0, BULK) ||
		eventInterval == 0U)
	{
		return false;
	}

	memset(pDesc, 0, sizeof(*pDesc));

	pDesc->Association.bLength = sizeof(pDesc->Association);
	pDesc->Association.bDescriptorType = USB_DESCTYPE_IA;
	pDesc->Association.bFirstInterface = (uint8_t)pHci->HciItfNo;
	pDesc->Association.bInterfaceCount = 2U;
	pDesc->Association.bFunctionClass = USB_INTRFCLASS_WIRELESS;
	pDesc->Association.bFunctionSubClass = USBD_HCI_SUBCLASS_RF;
	pDesc->Association.bFunctionProtocol = USBD_HCI_PROTOCOL_BT;
	pDesc->Association.iFunction = pHci->InterfaceString;

	pDesc->Hci.bLength = sizeof(pDesc->Hci);
	pDesc->Hci.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Hci.bInterfaceNumber = (uint8_t)pHci->HciItfNo;
	pDesc->Hci.bAlternateSetting = 0U;
	pDesc->Hci.bNumEndpoints = 3U;
	pDesc->Hci.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
	pDesc->Hci.bInterfaceSubClass = USBD_HCI_SUBCLASS_RF;
	pDesc->Hci.bInterfaceProtocol = USBD_HCI_PROTOCOL_BT;
	pDesc->Hci.iInterface = pHci->InterfaceString;

	pDesc->EventIn.bLength = sizeof(pDesc->EventIn);
	pDesc->EventIn.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	pDesc->EventIn.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->EventEpNo);
	pDesc->EventIn.bmAttributes = USB_ENDPATT_TRANS_INT;
	pDesc->EventIn.wMaxPacketSize = eventMps;
	pDesc->EventIn.bInterval = eventInterval;

	pDesc->AclOut.bLength = sizeof(pDesc->AclOut);
	pDesc->AclOut.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	pDesc->AclOut.bEndpointAddress = USB_ENDPADDR_DIROUT(pHci->AclEpNo);
	pDesc->AclOut.bmAttributes = USB_ENDPATT_TRANS_BULK;
	pDesc->AclOut.wMaxPacketSize = aclMps;
	pDesc->AclOut.bInterval = 0U;

	pDesc->AclIn = pDesc->AclOut;
	pDesc->AclIn.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->AclEpNo);

	pDesc->Sync.bLength = sizeof(pDesc->Sync);
	pDesc->Sync.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Sync.bInterfaceNumber = (uint8_t)pHci->SyncItfNo;
	pDesc->Sync.bAlternateSetting = 0U;
	pDesc->Sync.bNumEndpoints = 0U;
	pDesc->Sync.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
	pDesc->Sync.bInterfaceSubClass = USBD_HCI_SUBCLASS_RF;
	pDesc->Sync.bInterfaceProtocol = USBD_HCI_PROTOCOL_BT;
	pDesc->Sync.iInterface = pHci->InterfaceString;

	return true;
}

bool UsbdHciMakeScoDesc(UsbdHciScoDesc_t *pDesc,
						const UsbdHciDev_t *pHci, UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pHci == nullptr || !pHci->ScoEnabled ||
		pHci->ScoEpNo == 0U || pHci->ScoEpNo > 15U ||
		!UsbdHciMakeDesc(&pDesc->Legacy, pHci, Speed))
	{
		return false;
	}

	const uint8_t interval = Speed == USB_SPEED_HIGH ?
		USBD_HCI_SCO_HS_INTERVAL : USBD_HCI_SCO_FS_INTERVAL;
	for (uint8_t i = 0U; i < USBD_HCI_SCO_ALT_COUNT; i++)
	{
		UsbdHciScoAltDesc_t *pAlt = &pDesc->Alt[i];
		memset(pAlt, 0, sizeof(*pAlt));
		pAlt->Interface.bLength = sizeof(pAlt->Interface);
		pAlt->Interface.bDescriptorType = USB_DESCTYPE_INTERFACE;
		pAlt->Interface.bInterfaceNumber = (uint8_t)pHci->SyncItfNo;
		pAlt->Interface.bAlternateSetting = (uint8_t)(i + 1U);
		pAlt->Interface.bNumEndpoints = 2U;
		pAlt->Interface.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
		pAlt->Interface.bInterfaceSubClass = USBD_HCI_SUBCLASS_RF;
		pAlt->Interface.bInterfaceProtocol = USBD_HCI_PROTOCOL_BT;
		pAlt->Interface.iInterface = pHci->InterfaceString;

		pAlt->Out.bLength = sizeof(pAlt->Out);
		pAlt->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		pAlt->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pHci->ScoEpNo);
		pAlt->Out.bmAttributes = USB_ENDPATT_TRANS_ISO;
		pAlt->Out.wMaxPacketSize = s_UsbdHciScoMps[i];
		pAlt->Out.bInterval = interval;

		pAlt->In = pAlt->Out;
		pAlt->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->ScoEpNo);
	}
	return true;
}

bool UsbdHciInit(UsbdHciDev_t * const pHci,
				 UsbDevIntrf_t * const pAcl,
				 const UsbdHciCfg_t *pCfg)
{
	if (pHci == nullptr || pAcl == nullptr || pCfg == nullptr ||
		UsbGetCfg(pCfg->DevNo) == nullptr ||
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0)
	{
		return false;
	}

	memset(pHci, 0, sizeof(*pHci));
	pHci->pAcl = pAcl;
	pHci->DevNo = pCfg->DevNo;
	pHci->InterfaceString = pCfg->InterfaceString;
	pHci->EventFsMps = pCfg->EventFsMps != 0U ?
		pCfg->EventFsMps : USBD_HCI_EVENT_FS_MPS;
	pHci->EventHsMps = pCfg->EventHsMps != 0U ?
		pCfg->EventHsMps : USBD_HCI_EVENT_HS_MPS;
	pHci->AclFsMps = pCfg->AclFsMps != 0U ?
		pCfg->AclFsMps : USBD_HCI_ACL_FS_MPS;
	pHci->AclHsMps = pCfg->AclHsMps != 0U ?
		pCfg->AclHsMps : USBD_HCI_ACL_HS_MPS;
	pHci->ScoEnabled = pCfg->bSco;
	pHci->EventFsInterval = pCfg->EventFsInterval != 0U ?
		pCfg->EventFsInterval : USBD_HCI_EVENT_FS_INTERVAL;
	pHci->EventHsInterval = pCfg->EventHsInterval != 0U ?
		pCfg->EventHsInterval : USBD_HCI_EVENT_HS_INTERVAL;

	const uint16_t isoMask = (uint16_t)(USB_ISO_EPIN_MASK(pHci->DevNo) &
		USB_ISO_EPOUT_MASK(pHci->DevNo));
	const uint8_t scoEp = UsbdHciFirstEndpoint(isoMask);
	if (pHci->EventFsMps > USB_PKT_MAXLEN(0, INT) ||
		pHci->AclFsMps > USBD_HCI_ACL_MAX_MPS ||
		(pHci->ScoEnabled && (!USB_ISO_SUPPORTED(pHci->DevNo) ||
		 scoEp == 0U || USB_PKT_MAXLEN(pHci->DevNo, ISO) <
			USBD_HCI_SCO_MAX_MPS)) ||
		(USB_HIGHSPEED_CAPABLE(0) &&
		 (pHci->EventHsMps > USB_PKT_MAXLEN(0, INT) ||
		  pHci->AclHsMps > USBD_HCI_ACL_MAX_MPS)))
	{
		return false;
	}

	UsbFuncCfg_t coreCfg = {};
	coreCfg.RequestHandler = UsbdHciRequest;
	coreCfg.ConfigHandler = UsbdHciConfig;
	coreCfg.SetInterfaceHandler = UsbdHciSetInterface;
	coreCfg.XferHandler = UsbdHciXfer;
	coreCfg.ResetHandler = UsbdHciReset;
	coreCfg.pContext = pHci;

	UsbFuncReq_t req = {};
	req.InterfaceCount = 2U;
	req.BidirectionalCount = 1U;
	req.InCount = 1U;
	if (pHci->ScoEnabled)
	{
		req.FixedInMask = (uint16_t)(1U << scoEp);
		req.FixedOutMask = (uint16_t)(1U << scoEp);
	}

	UsbFuncAlloc_t alloc = {};
	if (!UsbRegisterFuncAuto(pHci->DevNo, &req, &coreCfg, &alloc))
	{
		return false;
	}

	pHci->HciItfNo = alloc.FirstInterface;
	pHci->SyncItfNo = alloc.FirstInterface + 1U;
	pHci->EventEpNo = alloc.In[0];
	pHci->AclEpNo = alloc.Bidirectional[0];
	pHci->ScoEpNo = pHci->ScoEnabled ? scoEp : 0U;

	UsbIntrfCfg_t dataCfg = {};
	dataCfg.bBlocking = pCfg->bBlocking;
	dataCfg.RxFifoMemSize = pCfg->RxFifoMemSize;
	dataCfg.pRxFifoMem = pCfg->pRxFifoMem;
	dataCfg.TxFifoMemSize = pCfg->TxFifoMemSize;
	dataCfg.pTxFifoMem = pCfg->pTxFifoMem;
	dataCfg.TxFifoBlkSize = USBD_HCI_ACL_PKT_BLKSIZE;
	dataCfg.DevNo = pHci->DevNo;
	dataCfg.EvtCB = pCfg->EvtCB;
	dataCfg.EpNo = pHci->AclEpNo;
	dataCfg.BufferSize = sizeof(pHci->AclRxTransfer);
	dataCfg.pRxBuffer = UsbdHciAclRxTransfer(pHci);
	dataCfg.pTxBuffer = UsbdHciAclTxTransfer(pHci);
	if (!UsbIntrfInit(pAcl, &dataCfg) ||
		!UsbCtrlrEpRegister(pHci->DevNo,
			USB_ENDPADDR_DIRIN(pHci->EventEpNo),
			UsbdHciEventTxTransfer(pHci), UsbdHciEventComplete, pHci) ||
		(pHci->ScoEnabled &&
		 (!UsbCtrlrEpRegister(pHci->DevNo,
			USB_ENDPADDR_DIROUT(pHci->ScoEpNo),
			UsbdHciScoRxTransfer(pHci), UsbdHciXfer, pHci) ||
		  !UsbCtrlrEpRegister(pHci->DevNo,
			USB_ENDPADDR_DIRIN(pHci->ScoEpNo),
			UsbdHciScoTxTransfer(pHci), UsbdHciXfer, pHci))))
	{
		return false;
	}

	UsbdHciInitDevIntrf(pHci);
	return true;
}

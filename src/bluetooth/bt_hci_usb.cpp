/**-------------------------------------------------------------------------
@file	bt_hci_usb.cpp

@brief	Bluetooth HCI USB transport implementation.

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
#include "usb/usbd_epalloc.h"
#include "bluetooth/bt_hci_usb.h"

#ifndef USB_ISO_EPIN_MASK
#define USB_ISO_EPIN_MASK(CtrlrNo) \
	((uint16_t)(((1UL << USB_EPIN_CNT(CtrlrNo)) - 1UL) & ~1UL))
#endif
#ifndef USB_ISO_EPOUT_MASK
#define USB_ISO_EPOUT_MASK(CtrlrNo) \
	((uint16_t)(((1UL << USB_EPOUT_CNT(CtrlrNo)) - 1UL) & ~1UL))
#endif

#define BT_HCI_USB_HISTORICAL_COMMAND_REQUEST	0xE0U

static uint8_t *BtHciUsbCommandBuffer(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->CommandBuffer);
}

static uint8_t *BtHciUsbAclRxBuffer(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->AclRxBuffer);
}

static uint8_t *BtHciUsbAclRxTransfer(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->AclRxTransfer);
}

static uint8_t *BtHciUsbAclTxTransfer(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->AclTxTransfer);
}

static UsbPkt_t *BtHciUsbAclTxPacket(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<UsbPkt_t *>(pHci->AclTxPacket);
}

static uint8_t *BtHciUsbEventTxBuffer(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->EventTxBuffer);
}

static uint8_t *BtHciUsbEventTxTransfer(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->EventTxTransfer);
}

static uint8_t *BtHciUsbScoRxBuffer(BtHciUsbDev_t *pHci, uint8_t Index)
{
	return reinterpret_cast<uint8_t *>(pHci->ScoRxBuffer[Index]);
}

static uint8_t *BtHciUsbScoTxBuffer(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->ScoTxBuffer);
}

static const uint8_t s_BtHciUsbScoMps[BT_HCI_USB_SCO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};

static uint8_t BtHciUsbScoMps(uint8_t Alt)
{
	return Alt > 0U && Alt <= BT_HCI_USB_SCO_ALT_COUNT ?
		s_BtHciUsbScoMps[Alt - 1U] : 0U;
}

static int BtHciUsbNotify(BtHciUsbDev_t *pHci, DEVINTRF_EVT Event,
						  int Length)
{
	return pHci->EvtCB != nullptr ?
		pHci->EvtCB(&pHci->IntrfData.DevIntrf, Event, nullptr, Length) : 0;
}

static uint16_t BtHciUsbReadLe16(const uint8_t *pData)
{
	return (uint16_t)pData[0] | ((uint16_t)pData[1] << 8);
}

static size_t BtHciUsbPacketLength(BtHciUsbPacketType_t Type,
								  const uint8_t *pPacket, size_t Available)
{
	if (pPacket == nullptr)
	{
		return 0U;
	}

	switch (Type)
	{
		case BT_HCI_USB_PACKET_COMMAND:
			return Available >= BT_HCI_USB_COMMAND_HEADER_SIZE ?
				BT_HCI_USB_COMMAND_HEADER_SIZE + (size_t)pPacket[2] : 0U;

		case BT_HCI_USB_PACKET_EVENT:
			return Available >= BT_HCI_USB_EVENT_HEADER_SIZE ?
				BT_HCI_USB_EVENT_HEADER_SIZE + (size_t)pPacket[1] : 0U;

		case BT_HCI_USB_PACKET_ACL:
			return Available >= BT_HCI_USB_ACL_HEADER_SIZE ?
				BT_HCI_USB_ACL_HEADER_SIZE +
				(size_t)BtHciUsbReadLe16(&pPacket[2]) : 0U;

		case BT_HCI_USB_PACKET_SCO:
			return Available >= BT_HCI_USB_SCO_HEADER_SIZE ?
				BT_HCI_USB_SCO_HEADER_SIZE + (size_t)pPacket[2] : 0U;

		case BT_HCI_USB_PACKET_ISO:
			return Available >= BT_HCI_USB_ISO_HEADER_SIZE ?
				BT_HCI_USB_ISO_HEADER_SIZE +
				(size_t)(BtHciUsbReadLe16(&pPacket[2]) & 0x3FFFU) : 0U;

		default:
			return 0U;
	}
}

static uint16_t BtHciUsbEventMps(const BtHciUsbDev_t *pHci)
{
	return UsbCtrlrHighSpeed(pHci->DevNo) ? pHci->EventHsMps : pHci->EventFsMps;
}

static uint16_t BtHciUsbAclMps(const BtHciUsbDev_t *pHci)
{
	return UsbCtrlrHighSpeed(pHci->DevNo) ? pHci->AclHsMps : pHci->AclFsMps;
}

static uint8_t BtHciUsbEventInterval(const BtHciUsbDev_t *pHci)
{
	return UsbCtrlrHighSpeed(pHci->DevNo) ?
		pHci->EventHsInterval : pHci->EventFsInterval;
}

static bool BtHciUsbOpenEndpoint(BtHciUsbDev_t *pHci, uint8_t EpAddr,
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

static void BtHciUsbCloseEndpoints(BtHciUsbDev_t *pHci)
{
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->EventEpNo));
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIROUT(pHci->AclEpNo));
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->AclEpNo));
}

static void BtHciUsbClearSco(BtHciUsbDev_t *pHci)
{
	pHci->ScoAlt = 0U;
	pHci->ScoTxActive = false;
	pHci->ScoRxBuildIndex = 0U;
	pHci->ScoRxPendingIndex = BT_HCI_USB_SCO_BUFFER_NONE;
	pHci->ScoRxReadIndex = BT_HCI_USB_SCO_BUFFER_NONE;
	pHci->ScoRxLength = 0U;
	pHci->ScoRxExpected = 0U;
	pHci->ScoTxLength = 0U;
	pHci->ScoTxOffset = 0U;
	pHci->ScoTxChunkLength = 0U;
	memset(pHci->ScoRxPacketLength, 0, sizeof(pHci->ScoRxPacketLength));
}

static void BtHciUsbCloseScoEndpoints(BtHciUsbDev_t *pHci)
{
	UsbIsoIntrfClose(&pHci->ScoIso);
	BtHciUsbClearSco(pHci);
}

static void BtHciUsbClearTransport(BtHciUsbDev_t *pHci)
{
	pHci->RxType = BT_HCI_USB_PACKET_NONE;
	pHci->TxType = BT_HCI_USB_PACKET_NONE;
	pHci->BulkRxType = BT_HCI_USB_PACKET_NONE;
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
	BtHciUsbClearSco(pHci);
}

static void BtHciUsbClearBulkRx(BtHciUsbDev_t *pHci)
{
	pHci->BulkRxType = BT_HCI_USB_PACKET_NONE;
	pHci->AclRxPending = false;
	pHci->AclRxLength = 0U;
	pHci->AclRxExpected = 0U;
}

static void BtHciUsbClearBulkTransport(BtHciUsbDev_t *pHci)
{
	pHci->RxType = BT_HCI_USB_PACKET_NONE;
	pHci->TxType = BT_HCI_USB_PACKET_NONE;
	BtHciUsbClearBulkRx(pHci);
}

static void BtHciUsbClearEventTx(BtHciUsbDev_t *pHci);

static bool BtHciUsbResetBulkTransport(BtHciUsbDev_t *pHci)
{
	const uint16_t mps = BtHciUsbAclMps(pHci);
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIROUT(pHci->AclEpNo));
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->AclEpNo));
	UsbIntrfUnconfigure(&pHci->IntrfData);
	BtHciUsbClearBulkTransport(pHci);

	if (!UsbIntrfConfigure(&pHci->IntrfData, mps))
	{
		return false;
	}

	if (!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->AclEpNo),
			USB_ENDPATT_TRANS_BULK, mps, 0U) ||
		!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIROUT(pHci->AclEpNo),
			USB_ENDPATT_TRANS_BULK, mps, 0U))
	{
		UsbCtrlrEpClose(pHci->DevNo,
			USB_ENDPADDR_DIROUT(pHci->AclEpNo));
		UsbCtrlrEpClose(pHci->DevNo,
			USB_ENDPADDR_DIRIN(pHci->AclEpNo));
		UsbIntrfUnconfigure(&pHci->IntrfData);
		return false;
	}
	return true;
}

static bool BtHciUsbOpenHciAlt(BtHciUsbDev_t *pHci, uint8_t Alt)
{
	if (!BtHciUsbResetBulkTransport(pHci))
	{
		return false;
	}

	pHci->CommandPending = false;
	pHci->CommandLength = 0U;
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->EventEpNo));
	BtHciUsbClearEventTx(pHci);

	if (Alt == 0U &&
		!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->EventEpNo),
			USB_ENDPATT_TRANS_INT, BtHciUsbEventMps(pHci),
			BtHciUsbEventInterval(pHci)))
	{
		return false;
	}

	pHci->HciAlt = Alt;
	pHci->BulkSerialization = Alt == 1U;
	return true;
}

static void BtHciUsbUnconfigure(BtHciUsbDev_t *pHci)
{
	pHci->Configured = false;
	pHci->HciAlt = 0U;
	pHci->BulkSerialization = false;
	UsbIntrfUnconfigure(&pHci->IntrfData);
	if (pHci->ScoEnabled)
	{
		UsbIsoIntrfClose(&pHci->ScoIso);
	}
	BtHciUsbClearTransport(pHci);
}

static bool BtHciUsbConfig(uint8_t Configuration, void *pContext)
{
	BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);

	if (pHci == nullptr)
	{
		return false;
	}

	BtHciUsbUnconfigure(pHci);
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != BT_HCI_USB_CONFIG_VALUE)
	{
		return false;
	}

	const uint16_t eventMps = BtHciUsbEventMps(pHci);
	const uint16_t aclMps = BtHciUsbAclMps(pHci);
	const uint8_t eventInterval = BtHciUsbEventInterval(pHci);

	if (!UsbIntrfConfigure(&pHci->IntrfData, aclMps))
	{
		return false;
	}

	if (!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->EventEpNo),
							 USB_ENDPATT_TRANS_INT, eventMps, eventInterval) ||
		!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->AclEpNo),
							 USB_ENDPATT_TRANS_BULK, aclMps, 0U) ||
		!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIROUT(pHci->AclEpNo),
							 USB_ENDPATT_TRANS_BULK, aclMps, 0U))
	{
		BtHciUsbUnconfigure(pHci);
		BtHciUsbCloseEndpoints(pHci);
		return false;
	}

	pHci->Configured = true;
	pHci->HciAlt = 0U;
	pHci->BulkSerialization = false;
	return true;
}

static bool BtHciUsbSetInterface(uint8_t InterfaceNo, uint8_t Alt,
								void *pContext)
{
	BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);
	if (pHci == nullptr || !pHci->Configured)
	{
		return false;
	}
	if (InterfaceNo == (uint8_t)pHci->HciItfNo)
	{
		if (Alt > 1U || (Alt == 1U &&
			!pHci->BulkSerializationSupported))
		{
			return false;
		}
		if (Alt == pHci->HciAlt)
		{
			return true;
		}
		if (pHci->ScoAlt != 0U || pHci->EventTxActive)
		{
			return false;
		}

		const uint8_t oldAlt = pHci->HciAlt;
		if (BtHciUsbOpenHciAlt(pHci, Alt))
		{
			return true;
		}

		(void)BtHciUsbOpenHciAlt(pHci, oldAlt);
		return false;
	}
	if (InterfaceNo != (uint8_t)pHci->SyncItfNo ||
		Alt > BT_HCI_USB_SCO_ALT_COUNT ||
		(!pHci->ScoEnabled && Alt != 0U))
	{
		return false;
	}
	if (Alt == pHci->ScoAlt)
	{
		return true;
	}
	if (pHci->BulkSerialization)
	{
		return false;
	}

	const uint8_t oldAlt = pHci->ScoAlt;
	BtHciUsbCloseScoEndpoints(pHci);
	if (Alt == 0U)
	{
		return true;
	}

	const uint8_t interval = UsbCtrlrHighSpeed(pHci->DevNo) ?
		BT_HCI_USB_SCO_HS_INTERVAL : BT_HCI_USB_SCO_FS_INTERVAL;
	if (UsbIsoIntrfOpen(&pHci->ScoIso, BtHciUsbScoMps(Alt), interval))
	{
		pHci->ScoAlt = Alt;
		return true;
	}

	if (oldAlt != 0U &&
		UsbIsoIntrfOpen(&pHci->ScoIso, BtHciUsbScoMps(oldAlt), interval))
	{
		pHci->ScoAlt = oldAlt;
	}
	return false;
}

static bool BtHciUsbRequestValid(const BtHciUsbDev_t *pHci,
								const UsbSetupData_t *pSetup)
{
	if (!pHci->Configured || pHci->BulkSerialization ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) != USB_REQTYPE_DIRDEV ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE) != USB_REQTYPE_CLASS ||
		(pSetup->bRequest != 0U &&
		 pSetup->bRequest != BT_HCI_USB_HISTORICAL_COMMAND_REQUEST) ||
		pSetup->wLength < BT_HCI_USB_COMMAND_HEADER_SIZE ||
		pSetup->wLength > BT_HCI_USB_COMMAND_MAX_SIZE)
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

static bool BtHciUsbRequest(const UsbSetupData_t *pSetup,
						   UsbCtrlStage_t Stage, uint8_t **ppData,
						   uint16_t *pLength, void *pContext)
{
	BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);
	if (pHci == nullptr || pSetup == nullptr || pLength == nullptr ||
		!BtHciUsbRequestValid(pHci, pSetup))
	{
		return false;
	}

	if (Stage == USB_CTRL_SETUP)
	{
		if (ppData == nullptr || pHci->CommandPending)
		{
			return false;
		}
		*ppData = BtHciUsbCommandBuffer(pHci);
		*pLength = pSetup->wLength;
		return true;
	}

	if (Stage == USB_CTRL_DATA)
	{
		return *pLength == pSetup->wLength &&
			BtHciUsbPacketLength(BT_HCI_USB_PACKET_COMMAND,
				BtHciUsbCommandBuffer(pHci), *pLength) == *pLength;
	}

	if (Stage == USB_CTRL_COMPLETE)
	{
		pHci->CommandLength = pSetup->wLength;
		pHci->CommandPending = true;
		(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_RX_DATA,
			pHci->CommandLength);
		return true;
	}

	if (Stage == USB_CTRL_ABORT)
	{
		pHci->CommandLength = 0U;
		return true;
	}

	return false;
}

static void BtHciUsbClearEventTx(BtHciUsbDev_t *pHci)
{
	pHci->EventTxActive = false;
	pHci->EventTxNeedZlp = false;
	pHci->EventTxZlp = false;
	pHci->EventTxLength = 0U;
	pHci->EventTxOffset = 0U;
	pHci->EventTxChunkLength = 0U;
}

static void BtHciUsbEventTxFailure(BtHciUsbDev_t *pHci, uint16_t Length)
{
	BtHciUsbClearEventTx(pHci);
	(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_TX_TIMEOUT, Length);
}

static bool BtHciUsbSendEventChunk(BtHciUsbDev_t *pHci)
{
	const uint16_t remaining =
		(uint16_t)(pHci->EventTxLength - pHci->EventTxOffset);
	const uint16_t mps = BtHciUsbEventMps(pHci);
	pHci->EventTxChunkLength = remaining < mps ? remaining : mps;
	pHci->EventTxZlp = false;
	memcpy(BtHciUsbEventTxTransfer(pHci),
		&BtHciUsbEventTxBuffer(pHci)[pHci->EventTxOffset],
		pHci->EventTxChunkLength);
	return UsbCtrlrEpXfer(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->EventEpNo),
		pHci->EventTxChunkLength);
}

static bool BtHciUsbSendEventZlp(BtHciUsbDev_t *pHci)
{
	pHci->EventTxChunkLength = 0U;
	pHci->EventTxZlp = true;
	return UsbCtrlrEpXfer(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->EventEpNo), 0U);
}

static void BtHciUsbEventComplete(uint8_t, UsbCtrlrEvtType_t Event,
								 uint16_t Length,
								 UsbCtrlrXferResult_t Result, void *pContext)
{
	BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);
	if (pHci == nullptr || Event != USB_CTRLR_EVT_XFER_CMPL ||
		!pHci->EventTxActive)
	{
		return;
	}

	const uint16_t expected = pHci->EventTxChunkLength;
	if (Result != USB_CTRLR_XFER_SUCCESS || Length != expected)
	{
		BtHciUsbEventTxFailure(pHci, Length);
		return;
	}

	if (!pHci->EventTxZlp)
	{
		pHci->EventTxOffset = (uint16_t)(pHci->EventTxOffset +
			pHci->EventTxChunkLength);
		if (pHci->EventTxOffset < pHci->EventTxLength)
		{
			if (BtHciUsbSendEventChunk(pHci))
			{
				return;
			}
			BtHciUsbEventTxFailure(pHci, 0U);
			return;
		}

		if (pHci->EventTxNeedZlp)
		{
			if (BtHciUsbSendEventZlp(pHci))
			{
				return;
			}
			BtHciUsbEventTxFailure(pHci, 0U);
			return;
		}
	}

	BtHciUsbClearEventTx(pHci);
	(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_TX_READY, 0);
}

static uint8_t BtHciUsbScoFreeBuffer(const BtHciUsbDev_t *pHci,
									uint8_t Pending)
{
	for (uint8_t i = 0U; i < BT_HCI_USB_SCO_RX_BUFFER_COUNT; i++)
	{
		if (i != Pending && i != pHci->ScoRxReadIndex)
		{
			return i;
		}
	}
	return 0U;
}

static void BtHciUsbScoPublish(BtHciUsbDev_t *pHci)
{
	const uint32_t state = DisableInterrupt();
	const uint8_t complete = pHci->ScoRxBuildIndex;
	const uint16_t length = pHci->ScoRxExpected;

	if (pHci->ScoRxPendingIndex < BT_HCI_USB_SCO_RX_BUFFER_COUNT)
	{
		pHci->ScoRxPacketLength[complete] = 0U;
		pHci->ScoRxLength = 0U;
		pHci->ScoRxExpected = 0U;
		EnableInterrupt(state);
		(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_RX_FIFO_FULL, length);
		return;
	}

	pHci->ScoRxPacketLength[complete] = length;
	pHci->ScoRxPendingIndex = complete;
	pHci->ScoRxBuildIndex = BtHciUsbScoFreeBuffer(pHci, complete);
	pHci->ScoRxPacketLength[pHci->ScoRxBuildIndex] = 0U;
	pHci->ScoRxLength = 0U;
	pHci->ScoRxExpected = 0U;
	EnableInterrupt(state);

	(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_RX_DATA, length);
}

static void BtHciUsbScoReceiveFrame(UsbIsoIntrf_t *, const uint8_t *pData,
									uint16_t Length,
									UsbCtrlrXferResult_t Result,
									void *pContext)
{
	BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);
	if (pHci == nullptr || pHci->ScoAlt == 0U)
	{
		return;
	}

	const uint16_t mps = BtHciUsbScoMps(pHci->ScoAlt);
	if (Result != USB_CTRLR_XFER_SUCCESS || Length > mps ||
		(size_t)pHci->ScoRxLength + Length > BT_HCI_USB_SCO_MAX_SIZE)
	{
		pHci->ScoRxLength = 0U;
		pHci->ScoRxExpected = 0U;
	}
	else if (Length != 0U)
	{
		memcpy(&BtHciUsbScoRxBuffer(pHci, pHci->ScoRxBuildIndex)
			[pHci->ScoRxLength], pData, Length);
		pHci->ScoRxLength = (uint16_t)(pHci->ScoRxLength + Length);

		if (pHci->ScoRxExpected == 0U &&
			pHci->ScoRxLength >= BT_HCI_USB_SCO_HEADER_SIZE)
		{
			pHci->ScoRxExpected = (uint16_t)BtHciUsbPacketLength(
				BT_HCI_USB_PACKET_SCO,
				BtHciUsbScoRxBuffer(pHci, pHci->ScoRxBuildIndex),
				pHci->ScoRxLength);
		}

		if (pHci->ScoRxExpected != 0U &&
			pHci->ScoRxLength == pHci->ScoRxExpected)
		{
			BtHciUsbScoPublish(pHci);
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
}

static bool BtHciUsbSendScoChunk(BtHciUsbDev_t *pHci)
{
	const uint16_t remaining =
		(uint16_t)(pHci->ScoTxLength - pHci->ScoTxOffset);
	const uint16_t mps = BtHciUsbScoMps(pHci->ScoAlt);
	pHci->ScoTxChunkLength = remaining < mps ? remaining : mps;
	return UsbIsoIntrfSendFrame(&pHci->ScoIso,
		&BtHciUsbScoTxBuffer(pHci)[pHci->ScoTxOffset],
		pHci->ScoTxChunkLength);
}

static void BtHciUsbScoSendFrameComplete(UsbIsoIntrf_t *, uint16_t Length,
										UsbCtrlrXferResult_t Result,
										void *pContext)
{
	BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);
	if (pHci == nullptr || !pHci->ScoTxActive)
	{
		return;
	}
	if (Result != USB_CTRLR_XFER_SUCCESS ||
		Length != pHci->ScoTxChunkLength)
	{
		pHci->ScoTxActive = false;
		(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_TX_TIMEOUT, Length);
		return;
	}

	pHci->ScoTxOffset = (uint16_t)(pHci->ScoTxOffset + Length);
	if (pHci->ScoTxOffset < pHci->ScoTxLength)
	{
		if (!BtHciUsbSendScoChunk(pHci))
		{
			pHci->ScoTxActive = false;
			(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_TX_TIMEOUT, 0);
		}
		return;
	}

	pHci->ScoTxActive = false;
	pHci->ScoTxLength = 0U;
	pHci->ScoTxOffset = 0U;
	pHci->ScoTxChunkLength = 0U;
	(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_TX_READY, 0);
}

static void BtHciUsbReset(void *pContext)
{
	BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);
	if (pHci != nullptr)
	{
		BtHciUsbUnconfigure(pHci);
		if (pHci->ScoEnabled)
		{
			UsbIsoIntrfReset(&pHci->ScoIso);
		}
	}
}

static BtHciUsbDev_t *BtHciUsbFromDev(DevIntrf_t *pDev)
{
	if (pDev == nullptr || pDev->pDevData == nullptr)
	{
		return nullptr;
	}
	UsbDevIntrf_t *pAcl = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	return static_cast<BtHciUsbDev_t *>(pAcl->pClassContext);
}

static bool BtHciUsbDevStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	BtHciUsbDev_t *pHci = BtHciUsbFromDev(pDev);
	if (pHci == nullptr || !pHci->Configured)
	{
		return false;
	}
	if (pHci->BulkSerialization)
	{
		if (DevAddr != BT_HCI_USB_PACKET_COMMAND &&
			DevAddr != BT_HCI_USB_PACKET_ACL &&
			DevAddr != BT_HCI_USB_PACKET_SCO &&
			DevAddr != BT_HCI_USB_PACKET_ISO)
		{
			return false;
		}
	}
	else if (DevAddr != BT_HCI_USB_PACKET_COMMAND &&
		DevAddr != BT_HCI_USB_PACKET_ACL &&
		(DevAddr != BT_HCI_USB_PACKET_SCO || pHci->ScoAlt == 0U))
	{
		return false;
	}

	pHci->RxType = (BtHciUsbPacketType_t)DevAddr;
	return true;
}

static void BtHciUsbDropPhysicalAcl(BtHciUsbDev_t *pHci)
{
	bool rearm = false;
	const uint32_t state = DisableInterrupt();
	if (CFifoGet(pHci->IntrfData.hRxFifo) != nullptr &&
		pHci->IntrfData.RxPending)
	{
		pHci->IntrfData.RxPending = false;
		rearm = true;
	}
	EnableInterrupt(state);

	if (rearm)
	{
		(void)UsbCtrlrEpXfer(pHci->DevNo,
			USB_ENDPADDR_DIROUT(pHci->AclEpNo), pHci->IntrfData.Mps);
	}
}

static bool BtHciUsbBulkRxTypeValid(BtHciUsbPacketType_t Type)
{
	return Type == BT_HCI_USB_PACKET_COMMAND || Type == BT_HCI_USB_PACKET_ACL ||
		Type == BT_HCI_USB_PACKET_SCO || Type == BT_HCI_USB_PACKET_ISO;
}

static bool BtHciUsbConsumeAcl(BtHciUsbDev_t *pHci)
{
	UsbPkt_t *pPacket =
		reinterpret_cast<UsbPkt_t *>(CFifoPeek(pHci->IntrfData.hRxFifo));
	if (pPacket == nullptr)
	{
		return false;
	}

	const uint16_t length = pPacket->Hdr.Length;
	if (length == 0U || length > pHci->IntrfData.Mps ||
		(size_t)pHci->AclRxLength + length >
			BT_HCI_USB_PACKET_MAX_SIZE + (pHci->BulkSerialization ? 1U : 0U))
	{
		BtHciUsbDropPhysicalAcl(pHci);
		BtHciUsbClearBulkRx(pHci);
		return true;
	}

	const int count = pHci->AclRxData(&pHci->IntrfData.DevIntrf,
		&BtHciUsbAclRxBuffer(pHci)[pHci->AclRxLength], length);
	if (count != length)
	{
		BtHciUsbClearBulkRx(pHci);
		return count > 0;
	}

	pHci->AclRxLength = (uint16_t)(pHci->AclRxLength + length);
	const uint8_t prefix = pHci->BulkSerialization ? 1U : 0U;
	if (pHci->BulkSerialization && pHci->AclRxLength != 0U &&
		pHci->BulkRxType == BT_HCI_USB_PACKET_NONE)
	{
		pHci->BulkRxType =
			(BtHciUsbPacketType_t)BtHciUsbAclRxBuffer(pHci)[0];
		if (!BtHciUsbBulkRxTypeValid(pHci->BulkRxType))
		{
			pHci->AclRxLength = 0U;
			pHci->BulkRxType = BT_HCI_USB_PACKET_NONE;
			return true;
		}
	}
	else if (!pHci->BulkSerialization)
	{
		pHci->BulkRxType = BT_HCI_USB_PACKET_ACL;
	}

	const BtHciUsbPacketType_t type = pHci->BulkRxType;
	const uint8_t headerSize = type == BT_HCI_USB_PACKET_COMMAND ?
		BT_HCI_USB_COMMAND_HEADER_SIZE : type == BT_HCI_USB_PACKET_EVENT ?
		BT_HCI_USB_EVENT_HEADER_SIZE : type == BT_HCI_USB_PACKET_SCO ?
		BT_HCI_USB_SCO_HEADER_SIZE : BT_HCI_USB_ACL_HEADER_SIZE;
	if (pHci->AclRxExpected == 0U && type != BT_HCI_USB_PACKET_NONE &&
		pHci->AclRxLength >= prefix + headerSize)
	{
		const size_t expected = prefix + BtHciUsbPacketLength(type,
			&BtHciUsbAclRxBuffer(pHci)[prefix], pHci->AclRxLength - prefix);
		if (expected > BT_HCI_USB_PACKET_MAX_SIZE + prefix)
		{
			pHci->AclRxLength = 0U;
			pHci->BulkRxType = BT_HCI_USB_PACKET_NONE;
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
		(length < pHci->IntrfData.Mps))
	{
		pHci->AclRxLength = 0U;
		pHci->AclRxExpected = 0U;
		pHci->BulkRxType = BT_HCI_USB_PACKET_NONE;
	}

	return true;
}

static int BtHciUsbAclEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
							uint8_t *, int Length)
{
	BtHciUsbDev_t *pHci = BtHciUsbFromDev(pDev);
	if (pHci == nullptr)
	{
		return 0;
	}
	if (Event == DEVINTRF_EVT_RX_FIFO_FULL)
	{
		return 0;
	}
	if (Event != DEVINTRF_EVT_RX_DATA)
	{
		return BtHciUsbNotify(pHci, Event, Length);
	}
	if (pHci->AclRxPending)
	{
		return 0;
	}

	int processed = 0;
	do
	{
		while (!pHci->AclRxPending && BtHciUsbConsumeAcl(pHci))
		{
		}
		if (!pHci->AclRxPending)
		{
			break;
		}

		const uint8_t prefix = pHci->BulkSerialization ? 1U : 0U;
		processed = BtHciUsbNotify(pHci, DEVINTRF_EVT_RX_DATA,
			(int)(pHci->AclRxExpected - prefix));
	}
	while (!pHci->AclRxPending);

	return processed;
}

static int BtHciUsbDevRxData(DevIntrf_t * const pDev, uint8_t *pBuffer,
							 int BufferLen)
{
	BtHciUsbDev_t *pHci = BtHciUsbFromDev(pDev);
	if (pHci == nullptr || pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	if (!pHci->BulkSerialization &&
		pHci->RxType == BT_HCI_USB_PACKET_COMMAND && pHci->CommandPending)
	{
		if (BufferLen < pHci->CommandLength)
		{
			return 0;
		}
		memcpy(pBuffer, BtHciUsbCommandBuffer(pHci), pHci->CommandLength);
		const int count = pHci->CommandLength;
		pHci->CommandPending = false;
		pHci->CommandLength = 0U;
		return count;
	}
	if (!pHci->BulkSerialization &&
		pHci->RxType == BT_HCI_USB_PACKET_SCO && pHci->ScoAlt != 0U)
	{
		const uint32_t state = DisableInterrupt();
		const uint8_t index = pHci->ScoRxPendingIndex;
		const uint16_t length = index < BT_HCI_USB_SCO_RX_BUFFER_COUNT ?
			pHci->ScoRxPacketLength[index] : 0U;
		if (index >= BT_HCI_USB_SCO_RX_BUFFER_COUNT || BufferLen < length)
		{
			EnableInterrupt(state);
			return 0;
		}
		pHci->ScoRxPendingIndex = BT_HCI_USB_SCO_BUFFER_NONE;
		pHci->ScoRxReadIndex = index;
		EnableInterrupt(state);

		memcpy(pBuffer, BtHciUsbScoRxBuffer(pHci, index), length);
		const uint32_t doneState = DisableInterrupt();
		pHci->ScoRxReadIndex = BT_HCI_USB_SCO_BUFFER_NONE;
		EnableInterrupt(doneState);
		return length;
	}

	if (!pHci->BulkSerialization && pHci->RxType != BT_HCI_USB_PACKET_ACL)
	{
		return 0;
	}

	while (!pHci->AclRxPending && BtHciUsbConsumeAcl(pHci))
	{
	}
	const uint8_t prefix = pHci->BulkSerialization ? 1U : 0U;
	const uint16_t length = (uint16_t)(pHci->AclRxExpected - prefix);
	if (!pHci->AclRxPending || pHci->BulkRxType != pHci->RxType ||
		BufferLen < length)
	{
		return 0;
	}

	memcpy(pBuffer, &BtHciUsbAclRxBuffer(pHci)[prefix], length);
	const int count = length;
	pHci->AclRxPending = false;
	pHci->AclRxLength = 0U;
	pHci->AclRxExpected = 0U;
	pHci->BulkRxType = BT_HCI_USB_PACKET_NONE;
	return count;
}

static bool BtHciUsbDevStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	BtHciUsbDev_t *pHci = BtHciUsbFromDev(pDev);
	if (pHci == nullptr || !pHci->Configured)
	{
		return false;
	}
	if (pHci->BulkSerialization)
	{
		if (DevAddr != BT_HCI_USB_PACKET_EVENT &&
			DevAddr != BT_HCI_USB_PACKET_ACL &&
			DevAddr != BT_HCI_USB_PACKET_SCO &&
			DevAddr != BT_HCI_USB_PACKET_ISO)
		{
			return false;
		}
	}
	else if (DevAddr != BT_HCI_USB_PACKET_EVENT &&
		DevAddr != BT_HCI_USB_PACKET_ACL &&
		(DevAddr != BT_HCI_USB_PACKET_SCO || pHci->ScoAlt == 0U))
	{
		return false;
	}

	pHci->TxType = (BtHciUsbPacketType_t)DevAddr;
	return true;
}

static int BtHciUsbQueueAcl(BtHciUsbDev_t *pHci, const uint8_t *pData,
						   int DataLen)
{
	const uint16_t mps = pHci->IntrfData.Mps;
	const uint8_t prefix = pHci->BulkSerialization ? 1U : 0U;
	const size_t wireLength = (size_t)DataLen + prefix;
	const size_t packetCount = (wireLength + mps - 1U) / mps;
	const bool needZlp = (wireLength % mps) == 0U;
	const size_t blocks = packetCount + (needZlp ? 1U : 0U);
	if (blocks > INT_MAX ||
		!UsbIntrfRequestToSend(&pHci->IntrfData,
			(int)(blocks * BT_HCI_USB_ACL_PKT_BLKSIZE)))
	{
		return 0;
	}

	UsbPkt_t *pPacket = BtHciUsbAclTxPacket(pHci);
	pPacket->Hdr.Reserved = 0U;
	size_t wireOffset = 0U;
	while (wireOffset < wireLength)
	{
		const size_t remaining = wireLength - wireOffset;
		const uint16_t length = (uint16_t)(remaining < mps ? remaining : mps);
		pPacket->Hdr.Length = length;
		uint16_t copied = 0U;
		if (prefix != 0U && wireOffset == 0U)
		{
			pPacket->Data[0] = (uint8_t)pHci->TxType;
			copied = 1U;
		}
		const size_t dataOffset = wireOffset + copied - prefix;
		memcpy(&pPacket->Data[copied], &pData[dataOffset], length - copied);
		if (pHci->AclTxData(&pHci->IntrfData.DevIntrf,
			reinterpret_cast<uint8_t *>(pPacket),
			BT_HCI_USB_ACL_PKT_BLKSIZE) != (int)BT_HCI_USB_ACL_PKT_BLKSIZE)
		{
			return 0;
		}
		wireOffset += length;
	}

	if (needZlp)
	{
		pPacket->Hdr.Length = 0U;
		if (pHci->AclTxData(&pHci->IntrfData.DevIntrf,
			reinterpret_cast<uint8_t *>(pPacket),
			BT_HCI_USB_ACL_PKT_BLKSIZE) != (int)BT_HCI_USB_ACL_PKT_BLKSIZE)
		{
			return 0;
		}
	}

	return DataLen;
}

static int BtHciUsbSendEvent(BtHciUsbDev_t *pHci, const uint8_t *pData,
							int DataLen)
{
	if (pHci->EventTxActive)
	{
		return 0;
	}

	memcpy(BtHciUsbEventTxBuffer(pHci), pData, DataLen);
	pHci->EventTxLength = (uint16_t)DataLen;
	pHci->EventTxOffset = 0U;
	pHci->EventTxNeedZlp =
		((unsigned)DataLen % BtHciUsbEventMps(pHci)) == 0U;
	pHci->EventTxZlp = false;
	pHci->EventTxActive = true;
	if (!BtHciUsbSendEventChunk(pHci))
	{
		BtHciUsbClearEventTx(pHci);
		return 0;
	}

	return DataLen;
}

static int BtHciUsbSendSco(BtHciUsbDev_t *pHci, const uint8_t *pData,
						  int DataLen)
{
	if (pHci->ScoAlt == 0U || pHci->ScoTxActive ||
		!UsbIsoIntrfTxReady(&pHci->ScoIso))
	{
		return 0;
	}

	memcpy(BtHciUsbScoTxBuffer(pHci), pData, DataLen);
	pHci->ScoTxLength = (uint16_t)DataLen;
	pHci->ScoTxOffset = 0U;
	pHci->ScoTxActive = true;
	if (!BtHciUsbSendScoChunk(pHci))
	{
		pHci->ScoTxActive = false;
		return 0;
	}
	return DataLen;
}

static int BtHciUsbDevTxData(DevIntrf_t * const pDev, const uint8_t *pData,
							 int DataLen)
{
	BtHciUsbDev_t *pHci = BtHciUsbFromDev(pDev);
	if (pHci == nullptr || pData == nullptr || DataLen <= 0 ||
		!pHci->Configured ||
		BtHciUsbPacketLength(pHci->TxType, pData, DataLen) !=
			(size_t)DataLen)
	{
		return 0;
	}
	if (pHci->BulkSerialization &&
		DataLen <= (int)BT_HCI_USB_PACKET_MAX_SIZE)
	{
		return BtHciUsbQueueAcl(pHci, pData, DataLen);
	}

	if (pHci->TxType == BT_HCI_USB_PACKET_EVENT &&
		DataLen <= (int)BT_HCI_USB_EVENT_MAX_SIZE)
	{
		return BtHciUsbSendEvent(pHci, pData, DataLen);
	}
	if (pHci->TxType == BT_HCI_USB_PACKET_ACL &&
		DataLen <= (int)BT_HCI_USB_PACKET_MAX_SIZE)
	{
		return BtHciUsbQueueAcl(pHci, pData, DataLen);
	}
	if (pHci->TxType == BT_HCI_USB_PACKET_SCO &&
		DataLen <= (int)BT_HCI_USB_SCO_MAX_SIZE)
	{
		return BtHciUsbSendSco(pHci, pData, DataLen);
	}

	return 0;
}

static int BtHciUsbDevTxSrData(DevIntrf_t * const pDev,
							   const uint8_t *pData, int DataLen)
{
	return BtHciUsbDevTxData(pDev, pData, DataLen);
}

static void BtHciUsbDevReset(DevIntrf_t * const pDev)
{
	BtHciUsbDev_t *pHci = BtHciUsbFromDev(pDev);
	if (pHci != nullptr)
	{
		BtHciUsbUnconfigure(pHci);
		if (pHci->ScoEnabled)
		{
			UsbIsoIntrfReset(&pHci->ScoIso);
		}
	}
}

static void *BtHciUsbDevGetHandle(DevIntrf_t * const pDev)
{
	return BtHciUsbFromDev(pDev);
}

static void BtHciUsbInitDevIntrf(BtHciUsbDev_t *pHci)
{
	DevIntrf_t *pDev = &pHci->IntrfData.DevIntrf;
	pHci->AclRxData = pDev->RxData;
	pHci->AclTxData = pDev->TxData;
	pHci->IntrfData.pClassContext = pHci;
	pDev->StartRx = BtHciUsbDevStartRx;
	pDev->RxData = BtHciUsbDevRxData;
	pDev->StartTx = BtHciUsbDevStartTx;
	pDev->TxData = BtHciUsbDevTxData;
	pDev->TxSrData = BtHciUsbDevTxSrData;
	pDev->Reset = BtHciUsbDevReset;
	pDev->GetHandle = BtHciUsbDevGetHandle;
}

bool BtHciUsbRequestToSend(BtHciUsbDev_t *pHci, int NbBytes)
{
	if (pHci == nullptr || !pHci->Configured || NbBytes <= 0)
	{
		return false;
	}
	if (pHci->BulkSerialization)
	{
		if ((pHci->TxType != BT_HCI_USB_PACKET_EVENT &&
			 pHci->TxType != BT_HCI_USB_PACKET_ACL &&
			 pHci->TxType != BT_HCI_USB_PACKET_SCO &&
			 pHci->TxType != BT_HCI_USB_PACKET_ISO) ||
			NbBytes > (int)BT_HCI_USB_PACKET_MAX_SIZE)
		{
			return false;
		}
		const int wireBytes = NbBytes + 1;
		const uint16_t mps = pHci->IntrfData.Mps;
		const int packets = (wireBytes + mps - 1) / mps;
		const int blocks = packets + ((wireBytes % mps) == 0 ? 1 : 0);
		return UsbIntrfRequestToSend(&pHci->IntrfData,
			blocks * (int)BT_HCI_USB_ACL_PKT_BLKSIZE);
	}
	if (pHci->TxType == BT_HCI_USB_PACKET_EVENT)
	{
		return !pHci->EventTxActive &&
			NbBytes <= (int)BT_HCI_USB_EVENT_MAX_SIZE;
	}
	if (pHci->TxType == BT_HCI_USB_PACKET_SCO)
	{
		return pHci->ScoAlt != 0U && !pHci->ScoTxActive &&
			UsbIsoIntrfTxReady(&pHci->ScoIso) &&
			NbBytes <= (int)BT_HCI_USB_SCO_MAX_SIZE;
	}
	if (pHci->TxType != BT_HCI_USB_PACKET_ACL ||
		NbBytes > (int)BT_HCI_USB_PACKET_MAX_SIZE)
	{
		return false;
	}

	const uint16_t mps = pHci->IntrfData.Mps;
	const int packets = (NbBytes + mps - 1) / mps;
	const int blocks = packets + ((NbBytes % mps) == 0 ? 1 : 0);
	return UsbIntrfRequestToSend(&pHci->IntrfData,
		blocks * (int)BT_HCI_USB_ACL_PKT_BLKSIZE);
}

static bool BtHciUsbMakeDesc(BtHciUsbDesc_t *pDesc, const BtHciUsbDev_t *pHci,
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

	if (eventMps == 0U || eventMps > USB_PKT_MAXLEN(pHci->DevNo, INT) ||
		aclMps == 0U || aclMps > USB_PKT_MAXLEN(pHci->DevNo, BULK) ||
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
	pDesc->Association.bFunctionSubClass = BT_HCI_USB_SUBCLASS_RF;
	pDesc->Association.bFunctionProtocol = BT_HCI_USB_PROTOCOL_BT;
	pDesc->Association.iFunction = pHci->InterfaceString;

	pDesc->Hci.bLength = sizeof(pDesc->Hci);
	pDesc->Hci.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Hci.bInterfaceNumber = (uint8_t)pHci->HciItfNo;
	pDesc->Hci.bAlternateSetting = 0U;
	pDesc->Hci.bNumEndpoints = 3U;
	pDesc->Hci.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
	pDesc->Hci.bInterfaceSubClass = BT_HCI_USB_SUBCLASS_RF;
	pDesc->Hci.bInterfaceProtocol = BT_HCI_USB_PROTOCOL_BT;
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
	pDesc->Sync.bInterfaceSubClass = BT_HCI_USB_SUBCLASS_RF;
	pDesc->Sync.bInterfaceProtocol = BT_HCI_USB_PROTOCOL_BT;
	pDesc->Sync.iInterface = pHci->InterfaceString;

	return true;
}

static bool BtHciUsbMakeScoDesc(BtHciUsbScoDesc_t *pDesc,
						const BtHciUsbDev_t *pHci, UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pHci == nullptr || !pHci->ScoEnabled ||
		pHci->ScoEpNo == 0U || pHci->ScoEpNo > 15U ||
		!BtHciUsbMakeDesc(&pDesc->Legacy, pHci, Speed))
	{
		return false;
	}

	const uint8_t interval = Speed == USB_SPEED_HIGH ?
		BT_HCI_USB_SCO_HS_INTERVAL : BT_HCI_USB_SCO_FS_INTERVAL;
	for (uint8_t i = 0U; i < BT_HCI_USB_SCO_ALT_COUNT; i++)
	{
		BtHciUsbScoAltDesc_t *pAlt = &pDesc->Alt[i];
		memset(pAlt, 0, sizeof(*pAlt));
		pAlt->Interface.bLength = sizeof(pAlt->Interface);
		pAlt->Interface.bDescriptorType = USB_DESCTYPE_INTERFACE;
		pAlt->Interface.bInterfaceNumber = (uint8_t)pHci->SyncItfNo;
		pAlt->Interface.bAlternateSetting = (uint8_t)(i + 1U);
		pAlt->Interface.bNumEndpoints = 2U;
		pAlt->Interface.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
		pAlt->Interface.bInterfaceSubClass = BT_HCI_USB_SUBCLASS_RF;
		pAlt->Interface.bInterfaceProtocol = BT_HCI_USB_PROTOCOL_BT;
		pAlt->Interface.iInterface = pHci->InterfaceString;

		pAlt->Out.bLength = sizeof(pAlt->Out);
		pAlt->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		pAlt->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pHci->ScoEpNo);
		pAlt->Out.bmAttributes = USB_ENDPATT_TRANS_ISO;
		pAlt->Out.wMaxPacketSize = s_BtHciUsbScoMps[i];
		pAlt->Out.bInterval = interval;

		pAlt->In = pAlt->Out;
		pAlt->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->ScoEpNo);
	}
	return true;
}

static void BtHciUsbMakeSerialAlt(BtHciUsbSerialAltDesc_t *pAlt,
								 const BtHciUsbDesc_t *pLegacy)
{
	memset(pAlt, 0, sizeof(*pAlt));
	pAlt->Interface = pLegacy->Hci;
	pAlt->Interface.bAlternateSetting = 1U;
	pAlt->Interface.bNumEndpoints = 2U;
	pAlt->Out = pLegacy->AclOut;
	pAlt->In = pLegacy->AclIn;
}

static bool BtHciUsbMakeSerialDesc(BtHciUsbSerialDesc_t *pDesc,
						   const BtHciUsbDev_t *pHci, UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pHci == nullptr ||
		!pHci->BulkSerializationSupported)
	{
		return false;
	}

	BtHciUsbDesc_t legacy = {};
	if (!BtHciUsbMakeDesc(&legacy, pHci, Speed))
	{
		return false;
	}

	memset(pDesc, 0, sizeof(*pDesc));
	pDesc->Association = legacy.Association;
	pDesc->Hci = legacy.Hci;
	pDesc->EventIn = legacy.EventIn;
	pDesc->AclOut = legacy.AclOut;
	pDesc->AclIn = legacy.AclIn;
	BtHciUsbMakeSerialAlt(&pDesc->Serialized, &legacy);
	pDesc->Sync = legacy.Sync;
	return true;
}

static bool BtHciUsbMakeFullDesc(BtHciUsbFullDesc_t *pDesc,
						 const BtHciUsbDev_t *pHci, UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pHci == nullptr || !pHci->ScoEnabled ||
		!BtHciUsbMakeSerialDesc(&pDesc->Base, pHci, Speed))
	{
		return false;
	}

	BtHciUsbScoDesc_t sco = {};
	if (!BtHciUsbMakeScoDesc(&sco, pHci, Speed))
	{
		return false;
	}
	memcpy(pDesc->Alt, sco.Alt, sizeof(pDesc->Alt));
	return true;
}

static bool BtHciUsbInitInternal(BtHciUsbDev_t * const pHci,
								 const BtHciUsbCfg_t *pCfg,
								 UsbDeviceClass *pClass)
{
	if (pHci == nullptr || pCfg == nullptr ||
		UsbGetCfg(pCfg->DevNo) == nullptr ||
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0)
	{
		return false;
	}

	const unsigned descCount =
		(pCfg->pDesc != nullptr ? 1U : 0U) +
		(pCfg->pScoDesc != nullptr ? 1U : 0U) +
		(pCfg->pSerialDesc != nullptr ? 1U : 0U) +
		(pCfg->pFullDesc != nullptr ? 1U : 0U);
	if (descCount > 1U ||
		(pCfg->pDesc != nullptr &&
			(pCfg->bSco || pCfg->bBulkSerialization)) ||
		(pCfg->pScoDesc != nullptr &&
			(!pCfg->bSco || pCfg->bBulkSerialization)) ||
		(pCfg->pSerialDesc != nullptr &&
			(pCfg->bSco || !pCfg->bBulkSerialization)) ||
		(pCfg->pFullDesc != nullptr &&
			(!pCfg->bSco || !pCfg->bBulkSerialization)))
	{
		return false;
	}

	memset(pHci, 0, sizeof(*pHci));
	pHci->EvtCB = pCfg->EvtCB;
	pHci->DevNo = pCfg->DevNo;
	pHci->InterfaceString = pCfg->InterfaceString;
	pHci->EventFsMps = pCfg->EventFsMps != 0U ?
		pCfg->EventFsMps : BT_HCI_USB_EVENT_FS_MPS;
	pHci->EventHsMps = pCfg->EventHsMps != 0U ?
		pCfg->EventHsMps : BT_HCI_USB_EVENT_HS_MPS;
	pHci->AclFsMps = pCfg->AclFsMps != 0U ?
		pCfg->AclFsMps : BT_HCI_USB_ACL_FS_MPS;
	pHci->AclHsMps = pCfg->AclHsMps != 0U ?
		pCfg->AclHsMps : BT_HCI_USB_ACL_HS_MPS;
	pHci->ScoEnabled = pCfg->bSco;
	pHci->BulkSerializationSupported = pCfg->bBulkSerialization;
	pHci->EventFsInterval = pCfg->EventFsInterval != 0U ?
		pCfg->EventFsInterval : BT_HCI_USB_EVENT_FS_INTERVAL;
	pHci->EventHsInterval = pCfg->EventHsInterval != 0U ?
		pCfg->EventHsInterval : BT_HCI_USB_EVENT_HS_INTERVAL;

	const uint16_t isoMask = (uint16_t)(USB_ISO_EPIN_MASK(pHci->DevNo) &
		USB_ISO_EPOUT_MASK(pHci->DevNo));
	if (pHci->EventFsMps > USB_PKT_MAXLEN(pHci->DevNo, INT) ||
		pHci->AclFsMps > USB_PKT_MAXLEN(pHci->DevNo, BULK) ||
		pHci->AclFsMps > BT_HCI_USB_ACL_MAX_MPS ||
		(pHci->ScoEnabled && (!USB_ISO_SUPPORTED(pHci->DevNo) ||
		 isoMask == 0U || USB_PKT_MAXLEN(pHci->DevNo, ISO) <
			BT_HCI_USB_SCO_MAX_MPS)) ||
		(USB_HIGHSPEED_CAPABLE(pHci->DevNo) &&
		 (pHci->EventHsMps > USB_PKT_MAXLEN(pHci->DevNo, INT) ||
		  pHci->AclHsMps > USB_PKT_MAXLEN(pHci->DevNo, BULK) ||
		  pHci->AclHsMps > BT_HCI_USB_ACL_MAX_MPS)))
	{
		return false;
	}

	UsbdClassCfg_t coreCfg = {};
	coreCfg.RequestHandler = pClass == nullptr ? BtHciUsbRequest : nullptr;
	coreCfg.ConfigHandler = BtHciUsbConfig;
	coreCfg.SetInterfaceHandler =
		pClass == nullptr ? BtHciUsbSetInterface : nullptr;
	coreCfg.ResetHandler = BtHciUsbReset;
	coreCfg.pContext = pHci;

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 2U;
	req.BidirectionalCount = 1U;
	req.InCount = 1U;

	UsbdEpAllocRes_t alloc = {};
	uint8_t scoEp = 0U;
	bool registered = false;
	if (!pHci->ScoEnabled)
	{
		registered = UsbdEpAlloc(pHci->DevNo, &req, &coreCfg, &alloc);
	}
	else
	{
		for (uint8_t ep = 1U; ep < 16U; ep++)
		{
			const uint16_t bit = (uint16_t)(1U << ep);
			if ((isoMask & bit) == 0U)
			{
				continue;
			}

			req.FixedInMask = bit;
			req.FixedOutMask = bit;
			if (UsbdEpAlloc(pHci->DevNo, &req, &coreCfg, &alloc))
			{
				scoEp = ep;
				registered = true;
				break;
			}
		}
	}
	if (!registered)
	{
		return false;
	}

	pHci->HciItfNo = alloc.FirstInterface;
	pHci->SyncItfNo = alloc.FirstInterface + 1U;
	pHci->EventEpNo = alloc.In[0];
	pHci->AclEpNo = alloc.Bidirectional[0];
	pHci->ScoEpNo = scoEp;

	UsbIntrfCfg_t dataCfg = {};
	dataCfg.bBlocking = pCfg->bBlocking;
	dataCfg.RxFifoMemSize = pCfg->RxFifoMemSize;
	dataCfg.pRxFifoMem = pCfg->pRxFifoMem;
	dataCfg.TxFifoMemSize = pCfg->TxFifoMemSize;
	dataCfg.pTxFifoMem = pCfg->pTxFifoMem;
	dataCfg.TxFifoBlkSize = BT_HCI_USB_ACL_PKT_BLKSIZE;
	dataCfg.DevNo = pHci->DevNo;
	dataCfg.EvtCB = BtHciUsbAclEvent;
	dataCfg.EpNo = pHci->AclEpNo;
	dataCfg.BufferSize = sizeof(pHci->AclRxTransfer);
	dataCfg.pRxBuffer = BtHciUsbAclRxTransfer(pHci);
	dataCfg.pTxBuffer = BtHciUsbAclTxTransfer(pHci);

	UsbIsoIntrfCfg_t isoCfg = {};
	isoCfg.DevNo = pHci->DevNo;
	isoCfg.EpNo = pHci->ScoEpNo;
	isoCfg.RxHandler = BtHciUsbScoReceiveFrame;
	isoCfg.TxHandler = BtHciUsbScoSendFrameComplete;
	isoCfg.pContext = pHci;

	if (!UsbIntrfInit(&pHci->IntrfData, &dataCfg) ||
		!UsbCtrlrEpRegister(pHci->DevNo,
			USB_ENDPADDR_DIRIN(pHci->EventEpNo),
			BtHciUsbEventTxTransfer(pHci), false, BtHciUsbEventComplete, pHci) ||
		(pHci->ScoEnabled && !UsbIsoIntrfInit(&pHci->ScoIso, &isoCfg)))
	{
		return false;
	}

	BtHciUsbInitDevIntrf(pHci);

	// The interface and endpoint numbers are known now, so build the descriptor
	// fragment into the application buffer that matches the selected layout. The
	// controller speed selects the MPS. A buffer that does not match the
	// bSco/bBulkSerialization flags is rejected by its builder.
	const UsbSpeed_t speed = USB_HIGHSPEED_CAPABLE(pHci->DevNo) ?
		USB_SPEED_HIGH : USB_SPEED_FULL;
	if ((pCfg->pDesc != nullptr &&
			!BtHciUsbMakeDesc(pCfg->pDesc, pHci, speed)) ||
		(pCfg->pScoDesc != nullptr &&
			!BtHciUsbMakeScoDesc(pCfg->pScoDesc, pHci, speed)) ||
		(pCfg->pSerialDesc != nullptr &&
			!BtHciUsbMakeSerialDesc(pCfg->pSerialDesc, pHci, speed)) ||
		(pCfg->pFullDesc != nullptr &&
			!BtHciUsbMakeFullDesc(pCfg->pFullDesc, pHci, speed)))
	{
		return false;
	}

	return pClass == nullptr || UsbClassRegister(pHci->DevNo, pClass);
}

bool BtHciUsbInit(BtHciUsbDev_t * const pHci,
				 const BtHciUsbCfg_t *pCfg)
{
	return BtHciUsbInitInternal(pHci, pCfg, nullptr);
}

bool BtHciUsb::Init(const BtHciUsbCfg_t &Cfg)
{
	return BtHciUsbInitInternal(&vBtHciUsb, &Cfg, this);
}

bool BtHciUsb::Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
						uint8_t **ppData, uint16_t *pLength)
{
	return BtHciUsbRequest(pSetup, Stage, ppData, pLength, &vBtHciUsb);
}

bool BtHciUsb::SelectInterface(uint8_t InterfaceNo, uint8_t Option)
{
	return BtHciUsbSetInterface(InterfaceNo, Option, &vBtHciUsb);
}

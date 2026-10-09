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
#include <string.h>

#include "coredev/interrupt.h"
#include "usb/usbd_epalloc.h"
#include "bluetooth/bt_hci_usb.h"

// Out of line, like CDC/HID, so static instances use zero-filled storage
// instead of a flash initializer containing the object and its packet buffers.
BtHciUsb::BtHciUsb() = default;

#ifndef USB_ISO_EPIN_MASK
#define USB_ISO_EPIN_MASK(CtrlrNo) \
	((uint16_t)(((1UL << USB_EPIN_CNT(CtrlrNo)) - 1UL) & ~1UL))
#endif
#ifndef USB_ISO_EPOUT_MASK
#define USB_ISO_EPOUT_MASK(CtrlrNo) \
	((uint16_t)(((1UL << USB_EPOUT_CNT(CtrlrNo)) - 1UL) & ~1UL))
#endif

#define BT_HCI_USB_HISTORICAL_COMMAND_REQUEST	0xE0U

static void BtHciUsbClearEventTx(BtHciUsbDev_t *pHci);
static void BtHciUsbKickTx(BtHciUsbDev_t *pHci);
static void BtHciUsbBulkComplete(UsbCtrlrEvtType_t Event,
	uint16_t Length, void *pContext);
static const UsbPkt_t *BtHciUsbTxHead(BtHciUsbDev_t *pHci);

static const uint8_t s_BtHciUsbScoMps[BT_HCI_USB_SCO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};
// Largest SCO frame any alternate carries; RxData buffer size.
#define BT_HCI_USB_SCO_FRAME_MAX		63U

static uint8_t *BtHciUsbCommandBuffer(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->CommandBuffer);
}

static uint8_t *BtHciUsbAclRxBuffer(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<uint8_t *>(pHci->AclRxBuffer);
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

static uint8_t BtHciUsbScoMps(uint8_t Alt)
{
	return Alt > 0U && Alt <= BT_HCI_USB_SCO_ALT_COUNT ?
		s_BtHciUsbScoMps[Alt - 1U] : 0U;
}

static int BtHciUsbNotify(BtHciUsbDev_t *pHci, DEVINTRF_EVT Event,
						  int Length)
{
	return pHci->EvtCB != nullptr ?
		pHci->EvtCB(&pHci->pData->DevIntrf, Event, nullptr, Length) : 0;
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

static bool BtHciUsbOpenEndpoint(BtHciUsbDev_t *pHci, uint8_t EpNo,
								 bool bIn, uint8_t TransferType,
								 uint16_t MaxPacketSize)
{
	return UsbCtrlrEpOpenData(pHci->DevNo, EpNo, bIn,
		TransferType, MaxPacketSize);
}

static void BtHciUsbCloseEndpoints(BtHciUsbDev_t *pHci)
{
	UsbCtrlrEpClose(pHci->DevNo, pHci->EventEpNo, true);
	UsbCtrlrEpClose(pHci->DevNo, pHci->AclEpNo, false);
	UsbCtrlrEpClose(pHci->DevNo, pHci->AclEpNo, true);
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
	UsbIsoIntrfClose(pHci->pScoIso);
	const UsbPkt_t *pHead = BtHciUsbTxHead(pHci);
	if (!pHci->BulkSerialization && pHead != nullptr &&
		pHead->Hdr.Reserved == BT_HCI_USB_PACKET_SCO)
	{
		pHci->BulkTxBusy = false;
		pHci->BulkTxOffset = 0U;
	}
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
	pHci->EventTxBusy = false;
	pHci->BulkTxBusy = false;
	pHci->BulkTxOffset = 0U;
	pHci->BulkTxChunkLength = 0U;
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

static bool BtHciUsbResetBulkTransport(BtHciUsbDev_t *pHci)
{
	const uint16_t mps = BtHciUsbAclMps(pHci);
	UsbCtrlrEpClose(pHci->DevNo, pHci->AclEpNo, false);
	UsbCtrlrEpClose(pHci->DevNo, pHci->AclEpNo, true);
	// A mode switch discards the old OUT framing, but retains complete HCI
	// packets awaiting IN delivery. Configure/Unconfigure would flush TX.
	while (CFifoGet(pHci->pData->hRxFifo) != nullptr)
	{
	}
	pHci->pData->RxPending = 0U;
	BtHciUsbClearBulkTransport(pHci);
	pHci->BulkTxBusy = false;
	pHci->BulkTxOffset = 0U;
	pHci->BulkTxChunkLength = 0U;

	if (!BtHciUsbOpenEndpoint(pHci, pHci->AclEpNo, true,
			USB_ENDPATT_TRANS_BULK, mps) ||
		!BtHciUsbOpenEndpoint(pHci, pHci->AclEpNo, false,
			USB_ENDPATT_TRANS_BULK, mps))
	{
		UsbCtrlrEpClose(pHci->DevNo, pHci->AclEpNo, false);
		UsbCtrlrEpClose(pHci->DevNo, pHci->AclEpNo, true);
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
	UsbCtrlrEpClose(pHci->DevNo, pHci->EventEpNo, true);
	pHci->EventTxBusy = false;
	pHci->EventTxOffset = 0U;
	pHci->EventTxZlp = false;

	if (Alt == 0U &&
		!BtHciUsbOpenEndpoint(pHci, pHci->EventEpNo, true,
			USB_ENDPATT_TRANS_INT, BtHciUsbEventMps(pHci)))
	{
		return false;
	}

	pHci->HciAlt = Alt;
	pHci->BulkSerialization = Alt == 1U;
	const unsigned length = pHci->EventTxLength + (Alt == 1U ? 1U : 0U);
	pHci->EventTxNeedZlp = (length % (Alt == 1U ?
		BtHciUsbAclMps(pHci) : BtHciUsbEventMps(pHci))) == 0U;
	return true;
}

static void BtHciUsbUnconfigure(BtHciUsbDev_t *pHci)
{
	pHci->Configured = false;
	pHci->HciAlt = 0U;
	pHci->BulkSerialization = false;
	UsbIntrfUnconfigure(pHci->pData);
	if (pHci->ScoEnabled)
	{
		UsbIsoIntrfClose(pHci->pScoIso);
	}
	BtHciUsbClearTransport(pHci);
}

static bool BtHciUsbConfig(BtHciUsbDev_t *pHci, uint8_t Configuration)
{
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

	if (!UsbIntrfConfigure(pHci->pData, aclMps))
	{
		return false;
	}

	if (!BtHciUsbOpenEndpoint(pHci, pHci->EventEpNo, true,
							 USB_ENDPATT_TRANS_INT, eventMps) ||
		!BtHciUsbOpenEndpoint(pHci, pHci->AclEpNo, true,
							 USB_ENDPATT_TRANS_BULK, aclMps) ||
		!BtHciUsbOpenEndpoint(pHci, pHci->AclEpNo, false,
							 USB_ENDPATT_TRANS_BULK, aclMps))
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

static bool BtHciUsbSetInterface(BtHciUsbDev_t *pHci,
								uint8_t InterfaceNo, uint8_t Alt)
{
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
		if (Alt != pHci->HciAlt && pHci->ScoAlt != 0U)
		{
			return false;
		}

		const uint8_t oldAlt = pHci->HciAlt;
		const uint32_t state = DisableInterrupt();
		// SET_INTERFACE resets endpoint state even when the alternate is
		// unchanged. Reopen to reset hardware toggles and restart retained
		// packets from their headers when the host opens a new session.
		// Close may synchronously cancel an endpoint on some controllers.
		// Keep callbacks and writers out until the new endpoints are ready.
		pHci->Configured = false;
		const bool selected = BtHciUsbOpenHciAlt(pHci, Alt);
		pHci->Configured = selected || BtHciUsbOpenHciAlt(pHci, oldAlt);
		BtHciUsbKickTx(pHci);
		EnableInterrupt(state);
		return selected;
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
	if (UsbIsoIntrfOpen(pHci->pScoIso, BtHciUsbScoMps(Alt), interval))
	{
		pHci->ScoAlt = Alt;
		BtHciUsbKickTx(pHci);
		return true;
	}

	if (oldAlt != 0U &&
		UsbIsoIntrfOpen(pHci->pScoIso, BtHciUsbScoMps(oldAlt), interval))
	{
		pHci->ScoAlt = oldAlt;
		BtHciUsbKickTx(pHci);
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
						   uint16_t *pLength, BtHciUsbDev_t *pHci)
{
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
	pHci->EventTxBusy = false;
	pHci->EventTxNeedZlp = false;
	pHci->EventTxZlp = false;
	pHci->EventTxLength = 0U;
	pHci->EventTxOffset = 0U;
	pHci->EventTxChunkLength = 0U;
}

static void BtHciUsbEventTxFailure(BtHciUsbDev_t *pHci, uint16_t Length)
{
	BtHciUsbClearEventTx(pHci);
	BtHciUsbKickTx(pHci);
	(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_TX_TIMEOUT, Length);
}

static bool BtHciUsbSendEventChunk(BtHciUsbDev_t *pHci)
{
	const uint8_t prefix = pHci->BulkSerialization ? 1U : 0U;
	const uint16_t remaining =
		(uint16_t)(pHci->EventTxLength + prefix - pHci->EventTxOffset);
	const uint16_t mps = prefix ? BtHciUsbAclMps(pHci) : BtHciUsbEventMps(pHci);
	pHci->EventTxChunkLength = remaining < mps ? remaining : mps;
	pHci->EventTxZlp = false;
	const uint8_t copied = prefix && pHci->EventTxOffset == 0U ? 1U : 0U;
	if (copied)
		BtHciUsbEventTxTransfer(pHci)[0] = BT_HCI_USB_PACKET_EVENT;
	memcpy(&BtHciUsbEventTxTransfer(pHci)[copied],
		&BtHciUsbEventTxBuffer(pHci)[pHci->EventTxOffset + copied - prefix],
		pHci->EventTxChunkLength - copied);
	pHci->EventTxBusy = UsbCtrlrEpSend(pHci->DevNo,
		prefix ? pHci->AclEpNo : pHci->EventEpNo,
		BtHciUsbEventTxTransfer(pHci),
		pHci->EventTxChunkLength);
	return pHci->EventTxBusy;
}

static bool BtHciUsbSendEventZlp(BtHciUsbDev_t *pHci)
{
	pHci->EventTxChunkLength = 0U;
	pHci->EventTxZlp = true;
	pHci->EventTxBusy = UsbCtrlrEpSend(pHci->DevNo,
		pHci->BulkSerialization ? pHci->AclEpNo : pHci->EventEpNo,
		BtHciUsbEventTxTransfer(pHci), 0U);
	return pHci->EventTxBusy;
}

static void BtHciUsbEventComplete(UsbCtrlrEvtType_t Event,
								 uint16_t Length, void *pContext)
{
	BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);
	if (pHci == nullptr || !pHci->Configured ||
		(Event != USB_CTRLR_EVT_XFER_CMPL && Event != USB_CTRLR_EVT_XFER_FAILED))
	{
		return;
	}
	if (!pHci->EventTxBusy)
	{
		const UsbPkt_t *pHead = BtHciUsbTxHead(pHci);
		if (!pHci->BulkSerialization && pHead != nullptr &&
			pHead->Hdr.Reserved == BT_HCI_USB_PACKET_EVENT)
			BtHciUsbBulkComplete(Event, Length, pContext);
		return;
	}
	pHci->EventTxBusy = false;

	const uint16_t expected = pHci->EventTxChunkLength;
	if (Event == USB_CTRLR_EVT_XFER_FAILED || Length != expected)
	{
		BtHciUsbEventTxFailure(pHci, Length);
		return;
	}

	if (!pHci->EventTxZlp)
	{
		pHci->EventTxOffset = (uint16_t)(pHci->EventTxOffset +
			pHci->EventTxChunkLength);
		if (pHci->EventTxOffset < pHci->EventTxLength +
			(pHci->BulkSerialization ? 1U : 0U))
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
	BtHciUsbKickTx(pHci);
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

// One received SCO frame, pulled from the ISO interface's RX FIFO.
static void BtHciUsbScoReceiveFrame(BtHciUsbDev_t *pHci, const uint8_t *pData,
									uint16_t Length)
{
	if (pHci->ScoAlt == 0U)
	{
		return;
	}

	const uint16_t mps = BtHciUsbScoMps(pHci->ScoAlt);
	if (Length > mps ||
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
	return UsbIsoIntrfSendFrame(pHci->pScoIso,
		&BtHciUsbScoTxBuffer(pHci)[pHci->ScoTxOffset],
		pHci->ScoTxChunkLength);
}

// One SCO frame left the device (Failed: the controller gave up on it).
static void BtHciUsbScoSendFrameComplete(BtHciUsbDev_t *pHci, uint16_t Length,
										bool Failed)
{
	const UsbPkt_t *pHead = BtHciUsbTxHead(pHci);
	if (!pHci->BulkSerialization && pHci->BulkTxBusy && pHead != nullptr &&
		pHead->Hdr.Reserved == BT_HCI_USB_PACKET_SCO)
	{
		BtHciUsbBulkComplete(Failed ? USB_CTRLR_EVT_XFER_FAILED :
			USB_CTRLR_EVT_XFER_CMPL, Length, pHci);
		return;
	}
	if (!pHci->ScoTxActive)
	{
		return;
	}
	if (Failed || Length != pHci->ScoTxChunkLength)
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
	BtHciUsbKickTx(pHci);
	(void)BtHciUsbNotify(pHci, DEVINTRF_EVT_TX_READY, 0);
}

// Event callback of the SCO ISO interface, the DeviceIntrf model: received
// frames are pulled with RxData when RX_DATA is raised; each sent frame is
// reported with TX_READY or TX_FIFO_EMPTY, a failed one with TX_TIMEOUT.
static int BtHciUsbScoEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
							uint8_t *, int Length)
{
	BtHciUsbDev_t *pHci =
		static_cast<BtHciUsbDev_t *>(UsbIsoIntrfContext(pDev));
	if (pHci == nullptr)
	{
		return 0;
	}

	switch (Event)
	{
		case DEVINTRF_EVT_RX_DATA:
		{
			uint8_t frame[BT_HCI_USB_SCO_FRAME_MAX];
			int total = 0;
			int len;
			while ((len = DeviceIntrfRxData(pDev, frame, sizeof(frame))) > 0)
			{
				BtHciUsbScoReceiveFrame(pHci, frame, (uint16_t)len);
				total += len;
			}
			return total;
		}

		case DEVINTRF_EVT_TX_READY:
		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			BtHciUsbScoSendFrameComplete(pHci, (uint16_t)Length, false);
			return Length;

		case DEVINTRF_EVT_TX_TIMEOUT:
			BtHciUsbScoSendFrameComplete(pHci, (uint16_t)Length, true);
			return 0;

		default:
			return 0;
	}
}

static void BtHciUsbReset(BtHciUsbDev_t *pHci)
{
	if (pHci != nullptr)
	{
		BtHciUsbUnconfigure(pHci);
		if (pHci->ScoEnabled)
		{
			UsbIsoIntrfReset(pHci->pScoIso);
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

static bool BtHciUsbBulkRxTypeValid(BtHciUsbPacketType_t Type)
{
	return Type == BT_HCI_USB_PACKET_COMMAND || Type == BT_HCI_USB_PACKET_ACL ||
		Type == BT_HCI_USB_PACKET_SCO || Type == BT_HCI_USB_PACKET_ISO;
}

static bool BtHciUsbConsumeAcl(BtHciUsbDev_t *pHci)
{
	UsbPkt_t *pPacket =
		reinterpret_cast<UsbPkt_t *>(CFifoPeek(pHci->pData->hRxFifo));
	if (pPacket == nullptr)
	{
		return false;
	}

	const uint16_t length = pPacket->Hdr.Length;
	if (length == 0U || length > pHci->pData->Mps ||
		(size_t)pHci->AclRxLength + length >
			BT_HCI_USB_PACKET_MAX_SIZE + (pHci->BulkSerialization ? 1U : 0U))
	{
		// Consume through UsbIntrf so freeing a slot also restarts withheld
		// OUT DMA. Zero capacity consumes exactly one ZLP.
		const int capacity = length < sizeof(pHci->AclRxBuffer) ?
			length : sizeof(pHci->AclRxBuffer);
		const int count = pHci->AclRxData(&pHci->pData->DevIntrf,
			BtHciUsbAclRxBuffer(pHci), capacity);
		BtHciUsbClearBulkRx(pHci);
		return count == length;
	}

	const int count = pHci->AclRxData(&pHci->pData->DevIntrf,
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
		(length < pHci->pData->Mps))
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

// TX FIFO blocks hold complete HCI packets without USB framing. The first
// block stores total length and packet type; continuation blocks use only
// Data. Nothing is released until the last transaction (including ZLP) has
// completed, so an alternate switch can restart the whole HCI packet.
static const UsbPkt_t *BtHciUsbTxHead(BtHciUsbDev_t *pHci)
{
	return reinterpret_cast<const UsbPkt_t *>(CFifoPeek(pHci->pData->hTxFifo));
}

static bool BtHciUsbHasQueuedType(BtHciUsbDev_t *pHci, BtHciUsbPacketType_t Type)
{
	CFifo_t cursor = *pHci->pData->hTxFifo;
	const UsbPkt_t *pPacket;
	while ((pPacket = reinterpret_cast<const UsbPkt_t *>(CFifoGet(&cursor))) != nullptr)
	{
		if (pPacket->Hdr.Reserved == Type)
			return true;
	}
	return false;
}

static void BtHciUsbCopyTx(BtHciUsbDev_t *pHci, uint8_t *pDest,
						 unsigned Offset, unsigned Length)
{
	// Traverse a read-only snapshot; only completion consumes the real FIFO.
	// Callers exclude interrupts while inspecting and copying queued data.
	CFifo_t cursor = *pHci->pData->hTxFifo;
	while (Offset >= BT_HCI_USB_ACL_MAX_MPS)
	{
		(void)CFifoGet(&cursor);
		Offset -= BT_HCI_USB_ACL_MAX_MPS;
	}
	while (Length != 0U)
	{
		const UsbPkt_t *pPacket = reinterpret_cast<const UsbPkt_t *>(CFifoGet(&cursor));
		const unsigned room = BT_HCI_USB_ACL_MAX_MPS - Offset;
		const unsigned count = Length < room ? Length : room;
		memcpy(pDest, &pPacket->Data[Offset], count);
		pDest += count;
		Length -= count;
		Offset = 0U;
	}
}

static uint16_t BtHciUsbQueuedMps(BtHciUsbDev_t *pHci, uint16_t Type)
{
	if (!pHci->BulkSerialization)
	{
		if (Type == BT_HCI_USB_PACKET_EVENT)
			return BtHciUsbEventMps(pHci);
		if (Type == BT_HCI_USB_PACKET_SCO)
			return BtHciUsbScoMps(pHci->ScoAlt);
	}
	return BtHciUsbAclMps(pHci);
}

static void BtHciUsbKickTx(BtHciUsbDev_t *pHci)
{
	const uint32_t state = DisableInterrupt();
	if (!pHci->Configured)
	{
		EnableInterrupt(state);
		return;
	}

	// The independent legacy event buffer survives a switch too. Send it
	// first on bulk IN before resuming the queued packets in serialized mode.
	if (pHci->EventTxActive && !pHci->EventTxBusy &&
		(!pHci->BulkSerialization || !pHci->BulkTxBusy))
		(void)BtHciUsbSendEventChunk(pHci);

	const UsbPkt_t *pHead = BtHciUsbTxHead(pHci);
	atomic_store_explicit(&pHci->pData->DevIntrf.bTxReady,
		pHead == nullptr && !pHci->EventTxActive, memory_order_release);
	if (pHead == nullptr || pHci->BulkTxBusy ||
		(pHci->BulkSerialization && pHci->EventTxActive))
	{
		EnableInterrupt(state);
		return;
	}

	const uint16_t type = pHead->Hdr.Reserved;
	const bool event = !pHci->BulkSerialization && type == BT_HCI_USB_PACKET_EVENT;
	const bool sco = !pHci->BulkSerialization && type == BT_HCI_USB_PACKET_SCO;
	if ((event && pHci->EventTxActive) ||
		(sco && (pHci->ScoAlt == 0U || pHci->ScoTxActive)) ||
		(!pHci->BulkSerialization && type == BT_HCI_USB_PACKET_ISO))
	{
		// ISO has no legacy endpoint. Retain it until serialized mode returns;
		// retained SCO waits for the host to select a synchronous alternate.
		EnableInterrupt(state);
		return;
	}

	const unsigned prefix = pHci->BulkSerialization ? 1U : 0U;
	const unsigned remaining = pHead->Hdr.Length + prefix - pHci->BulkTxOffset;
	const uint16_t mps = BtHciUsbQueuedMps(pHci, type);
	pHci->BulkTxChunkLength = remaining < mps ? remaining : mps;
	uint8_t *pBuffer = event ? BtHciUsbEventTxTransfer(pHci) :
		BtHciUsbAclTxPacket(pHci)->Data;
	const unsigned copied = prefix && pHci->BulkTxOffset == 0U ? 1U : 0U;
	if (copied)
		pBuffer[0] = (uint8_t)type;
	BtHciUsbCopyTx(pHci, &pBuffer[copied],
		pHci->BulkTxOffset + copied - prefix, pHci->BulkTxChunkLength - copied);
	pHci->BulkTxBusy = sco ?
		UsbIsoIntrfSendFrame(pHci->pScoIso, pBuffer, pHci->BulkTxChunkLength) :
		UsbCtrlrEpSend(pHci->DevNo, event ? pHci->EventEpNo : pHci->AclEpNo,
			pBuffer, pHci->BulkTxChunkLength);
	EnableInterrupt(state);
}

static void BtHciUsbBulkComplete(UsbCtrlrEvtType_t Event,
	uint16_t Length, void *pContext)
{
	BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);
	if (!pHci->Configured ||
		(Event != USB_CTRLR_EVT_XFER_CMPL && Event != USB_CTRLR_EVT_XFER_FAILED))
		return;
	if (pHci->BulkSerialization && pHci->EventTxBusy)
	{
		BtHciUsbEventComplete(Event, Length, pContext);
		return;
	}
	if (!pHci->BulkTxBusy)
		return;

	const uint32_t state = DisableInterrupt();
	const UsbPkt_t *pHead = BtHciUsbTxHead(pHci);
	const uint16_t total = pHead->Hdr.Length;
	const uint16_t type = pHead->Hdr.Reserved;
	const bool failed = Event == USB_CTRLR_EVT_XFER_FAILED ||
		Length != pHci->BulkTxChunkLength;
	pHci->BulkTxBusy = false;
	pHci->BulkTxOffset += Length;
	const unsigned wireLength = total + (pHci->BulkSerialization ? 1U : 0U);
	const bool sco = !pHci->BulkSerialization && type == BT_HCI_USB_PACKET_SCO;
	if (!failed && (pHci->BulkTxOffset < wireLength ||
		(!sco && Length == BtHciUsbQueuedMps(pHci, type))))
	{
		BtHciUsbKickTx(pHci);
		EnableInterrupt(state);
		return;
	}

	unsigned blocks = (total + BT_HCI_USB_ACL_MAX_MPS - 1U) / BT_HCI_USB_ACL_MAX_MPS;
	while (blocks-- != 0U)
		(void)CFifoGet(pHci->pData->hTxFifo);
	pHci->BulkTxOffset = 0U;
	pHci->BulkTxChunkLength = 0U;
	BtHciUsbKickTx(pHci);
	const bool empty = BtHciUsbTxHead(pHci) == nullptr;
	EnableInterrupt(state);
	if (failed || empty)
		(void)BtHciUsbNotify(pHci, failed ? DEVINTRF_EVT_TX_TIMEOUT :
			DEVINTRF_EVT_TX_FIFO_EMPTY, failed ? total : 0);
}

static int BtHciUsbQueueAcl(BtHciUsbDev_t *pHci, const uint8_t *pData,
						   int DataLen)
{
	const unsigned blocks = (DataLen + BT_HCI_USB_ACL_MAX_MPS - 1U) /
		BT_HCI_USB_ACL_MAX_MPS;
	// Whole-packet admission also applies to a nonblocking FIFO. Overwriting
	// even one old block would corrupt an HCI packet already in flight.
	if (CFifoAvail(pHci->pData->hTxFifo) < (int)blocks)
	{
		BtHciUsbKickTx(pHci);
		return 0;
	}

	unsigned offset = 0U;
	while (offset < (unsigned)DataLen)
	{
		UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(CFifoPut(pHci->pData->hTxFifo));
		pPacket->Hdr.Length = offset == 0U ? DataLen : 0U;
		pPacket->Hdr.Reserved = offset == 0U ? pHci->TxType : BT_HCI_USB_PACKET_NONE;
		const unsigned remaining = DataLen - offset;
		const unsigned count = remaining < BT_HCI_USB_ACL_MAX_MPS ?
			remaining : BT_HCI_USB_ACL_MAX_MPS;
		memcpy(pPacket->Data, &pData[offset], count);
		offset += count;
	}
	BtHciUsbKickTx(pHci);
	return DataLen;
}

static int BtHciUsbSendEvent(BtHciUsbDev_t *pHci, const uint8_t *pData,
							int DataLen)
{
	// A newly submitted legacy event must not overtake older events retained
	// in the serialized queue, even while an ACL occupies its head.
	if (pHci->EventTxActive || BtHciUsbHasQueuedType(pHci, BT_HCI_USB_PACKET_EVENT))
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
		BtHciUsbHasQueuedType(pHci, BT_HCI_USB_PACKET_SCO) ||
		!UsbIsoIntrfTxReady(pHci->pScoIso))
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

static int BtHciUsbTxDataLocked(DevIntrf_t * const pDev, const uint8_t *pData,
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

static int BtHciUsbDevTxData(DevIntrf_t * const pDev, const uint8_t *pData,
							 int DataLen)
{
	const uint32_t state = DisableInterrupt();
	const int count = BtHciUsbTxDataLocked(pDev, pData, DataLen);
	EnableInterrupt(state);
	return count;
}

static void BtHciUsbDevReset(DevIntrf_t * const pDev)
{
	BtHciUsbDev_t *pHci = BtHciUsbFromDev(pDev);
	BtHciUsbReset(pHci);
}

static void *BtHciUsbDevGetHandle(DevIntrf_t * const pDev)
{
	return BtHciUsbFromDev(pDev);
}

static void BtHciUsbInitDevIntrf(BtHciUsbDev_t *pHci)
{
	DevIntrf_t *pDev = &pHci->pData->DevIntrf;
	pHci->AclRxData = pDev->RxData;
	pHci->AclTxData = pDev->TxData;
	pHci->pData->pClassContext = pHci;
	pDev->StartRx = BtHciUsbDevStartRx;
	pDev->RxData = BtHciUsbDevRxData;
	pDev->StartTx = BtHciUsbDevStartTx;
	pDev->TxData = BtHciUsbDevTxData;
	pDev->TxSrData = BtHciUsbDevTxData;
	pDev->Reset = BtHciUsbDevReset;
	pDev->GetHandle = BtHciUsbDevGetHandle;
}

static constexpr BtHciUsbDesc_t BtHciUsbLegacyTemplate(void)
{
	BtHciUsbDesc_t desc = {};

	desc.Association.bLength = sizeof(desc.Association);
	desc.Association.bDescriptorType = USB_DESCTYPE_IA;
	desc.Association.bInterfaceCount = 2U;
	desc.Association.bFunctionClass = USB_INTRFCLASS_WIRELESS;
	desc.Association.bFunctionSubClass = BT_HCI_USB_SUBCLASS_RF;
	desc.Association.bFunctionProtocol = BT_HCI_USB_PROTOCOL_BT;

	desc.Hci.bLength = sizeof(desc.Hci);
	desc.Hci.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Hci.bNumEndpoints = 3U;
	desc.Hci.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
	desc.Hci.bInterfaceSubClass = BT_HCI_USB_SUBCLASS_RF;
	desc.Hci.bInterfaceProtocol = BT_HCI_USB_PROTOCOL_BT;

	desc.EventIn.bLength = sizeof(desc.EventIn);
	desc.EventIn.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.EventIn.bmAttributes = USB_ENDPATT_TRANS_INT;

	desc.AclOut.bLength = sizeof(desc.AclOut);
	desc.AclOut.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.AclOut.bmAttributes = USB_ENDPATT_TRANS_BULK;
	desc.AclIn = desc.AclOut;

	desc.Sync.bLength = sizeof(desc.Sync);
	desc.Sync.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Sync.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
	desc.Sync.bInterfaceSubClass = BT_HCI_USB_SUBCLASS_RF;
	desc.Sync.bInterfaceProtocol = BT_HCI_USB_PROTOCOL_BT;
	return desc;
}

static constexpr BtHciUsbSerialDesc_t BtHciUsbSerialTemplate(void)
{
	const BtHciUsbDesc_t legacy = BtHciUsbLegacyTemplate();
	BtHciUsbSerialDesc_t desc = {};
	desc.Association = legacy.Association;
	desc.Hci = legacy.Hci;
	desc.EventIn = legacy.EventIn;
	desc.AclOut = legacy.AclOut;
	desc.AclIn = legacy.AclIn;
	desc.Serialized.Interface = legacy.Hci;
	desc.Serialized.Interface.bAlternateSetting = 1U;
	desc.Serialized.Interface.bNumEndpoints = 2U;
	desc.Serialized.Out = legacy.AclOut;
	desc.Serialized.In = legacy.AclIn;
	desc.Sync = legacy.Sync;
	return desc;
}

static constexpr BtHciUsbScoAltDesc_t BtHciUsbScoAltTemplate(unsigned Index)
{
	BtHciUsbScoAltDesc_t desc = {};
	desc.Interface.bLength = sizeof(desc.Interface);
	desc.Interface.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Interface.bAlternateSetting = (uint8_t)(Index + 1U);
	desc.Interface.bNumEndpoints = 2U;
	desc.Interface.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
	desc.Interface.bInterfaceSubClass = BT_HCI_USB_SUBCLASS_RF;
	desc.Interface.bInterfaceProtocol = BT_HCI_USB_PROTOCOL_BT;
	desc.Out.bLength = sizeof(desc.Out);
	desc.Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.Out.bmAttributes = USB_ENDPATT_TRANS_ISO;
	desc.Out.wMaxPacketSize = Index == 0U ? 9U :
		Index == 1U ? 17U : Index == 2U ? 25U :
		Index == 3U ? 33U : Index == 4U ? 49U : 63U;
	desc.In = desc.Out;
	return desc;
}

static constexpr BtHciUsbScoDesc_t BtHciUsbScoTemplate(void)
{
	BtHciUsbScoDesc_t desc = {};
	desc.Legacy = BtHciUsbLegacyTemplate();
	for (unsigned i = 0U; i < BT_HCI_USB_SCO_ALT_COUNT; i++)
	{
		desc.Alt[i] = BtHciUsbScoAltTemplate(i);
	}
	return desc;
}

static constexpr BtHciUsbFullDesc_t BtHciUsbFullTemplate(void)
{
	BtHciUsbFullDesc_t desc = {};
	desc.Base = BtHciUsbSerialTemplate();
	for (unsigned i = 0U; i < BT_HCI_USB_SCO_ALT_COUNT; i++)
	{
		desc.Alt[i] = BtHciUsbScoAltTemplate(i);
	}
	return desc;
}

static constexpr BtHciUsbScoDesc_t s_BtHciUsbScoDescTemplate =
	BtHciUsbScoTemplate();
static constexpr BtHciUsbFullDesc_t s_BtHciUsbFullDescTemplate =
	BtHciUsbFullTemplate();

static inline __attribute__((always_inline))
void BtHciUsbPatchBase(UsbInrtfAssDesc_t &Association,
					   UsbIntrfDesc_t &Hci,
					   UsbEndPointDesc_t &EventIn,
					   UsbEndPointDesc_t &AclOut,
					   UsbEndPointDesc_t &AclIn,
					   UsbIntrfDesc_t &Sync,
					   const BtHciUsbDev_t *pHci,
					   UsbSpeed_t Speed)
{
	const uint16_t eventMps = Speed == USB_SPEED_HIGH ?
		pHci->EventHsMps : pHci->EventFsMps;
	const uint16_t aclMps = Speed == USB_SPEED_HIGH ?
		pHci->AclHsMps : pHci->AclFsMps;
	const uint8_t eventInterval = Speed == USB_SPEED_HIGH ?
		pHci->EventHsInterval : pHci->EventFsInterval;

	Association.bFirstInterface = (uint8_t)pHci->HciItfNo;
	Association.iFunction = pHci->InterfaceString;
	Hci.bInterfaceNumber = (uint8_t)pHci->HciItfNo;
	Hci.iInterface = pHci->InterfaceString;
	EventIn.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->EventEpNo);
	EventIn.wMaxPacketSize = eventMps;
	EventIn.bInterval = eventInterval;
	AclOut.bEndpointAddress = USB_ENDPADDR_DIROUT(pHci->AclEpNo);
	AclOut.wMaxPacketSize = aclMps;
	AclIn.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->AclEpNo);
	AclIn.wMaxPacketSize = aclMps;
	Sync.bInterfaceNumber = (uint8_t)pHci->SyncItfNo;
	Sync.iInterface = pHci->InterfaceString;
}

static inline __attribute__((always_inline))
void BtHciUsbPatchSerial(BtHciUsbSerialDesc_t *pDesc,
						 const BtHciUsbDev_t *pHci,
						 UsbSpeed_t Speed)
{
	BtHciUsbPatchBase(pDesc->Association, pDesc->Hci, pDesc->EventIn,
		pDesc->AclOut, pDesc->AclIn, pDesc->Sync, pHci, Speed);
	const uint16_t aclMps = Speed == USB_SPEED_HIGH ?
		pHci->AclHsMps : pHci->AclFsMps;
	pDesc->Serialized.Interface.bInterfaceNumber = (uint8_t)pHci->HciItfNo;
	pDesc->Serialized.Interface.iInterface = pHci->InterfaceString;
	pDesc->Serialized.Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pHci->AclEpNo);
	pDesc->Serialized.Out.wMaxPacketSize = aclMps;
	pDesc->Serialized.In.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->AclEpNo);
	pDesc->Serialized.In.wMaxPacketSize = aclMps;
}

static inline __attribute__((always_inline))
void BtHciUsbPatchSco(BtHciUsbScoAltDesc_t *pAlt,
					  const BtHciUsbDev_t *pHci, UsbSpeed_t Speed)
{
	const uint8_t interval = Speed == USB_SPEED_HIGH ?
		BT_HCI_USB_SCO_HS_INTERVAL : BT_HCI_USB_SCO_FS_INTERVAL;
	for (unsigned i = 0U; i < BT_HCI_USB_SCO_ALT_COUNT; i++)
	{
		pAlt[i].Interface.bInterfaceNumber = (uint8_t)pHci->SyncItfNo;
		pAlt[i].Interface.iInterface = pHci->InterfaceString;
		pAlt[i].Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pHci->ScoEpNo);
		pAlt[i].Out.bInterval = interval;
		pAlt[i].In.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->ScoEpNo);
		pAlt[i].In.bInterval = interval;
	}
}

static void BtHciUsbPatchRegistered(const UsbDeviceClass *pClass,
									uint8_t *pData, UsbSpeed_t Speed)
{
	const BtHciUsbDev_t *pHci = *static_cast<const BtHciUsb *>(pClass);
	if (pHci->BulkSerializationSupported)
	{
		BtHciUsbSerialDesc_t *pDesc;
		if (pHci->ScoEnabled)
		{
			BtHciUsbFullDesc_t *pFull =
				reinterpret_cast<BtHciUsbFullDesc_t *>(pData);
			pDesc = &pFull->Base;
			BtHciUsbPatchSco(pFull->Alt, pHci, Speed);
		}
		else
		{
			pDesc = reinterpret_cast<BtHciUsbSerialDesc_t *>(pData);
		}
		BtHciUsbPatchSerial(pDesc, pHci, Speed);
		return;
	}

	BtHciUsbDesc_t *pDesc;
	if (pHci->ScoEnabled)
	{
		BtHciUsbScoDesc_t *pSco =
			reinterpret_cast<BtHciUsbScoDesc_t *>(pData);
		pDesc = &pSco->Legacy;
		BtHciUsbPatchSco(pSco->Alt, pHci, Speed);
	}
	else
	{
		pDesc = reinterpret_cast<BtHciUsbDesc_t *>(pData);
	}
	BtHciUsbPatchBase(pDesc->Association, pDesc->Hci, pDesc->EventIn,
		pDesc->AclOut, pDesc->AclIn, pDesc->Sync, pHci, Speed);
}

static bool BtHciUsbInitInternal(BtHciUsbDev_t * const pHci,
								 UsbDevIntrf_t *pData, UsbIsoIntrf *pSco,
								 const BtHciUsbCfg_t *pCfg,
								 UsbDeviceClass *pClass)
{
	if (pHci == nullptr || pCfg == nullptr || pClass == nullptr ||
		UsbGetCfg(pCfg->DevNo) == nullptr ||
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0)
	{
		return false;
	}

	memset(pHci, 0, sizeof(*pHci));
	pHci->pData = pData;
	pHci->pScoIso = *pSco;
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
	if (pHci->EventFsMps > USB_CTRLR_PKT_LEN_MAX(pHci->DevNo, INT) ||
		pHci->AclFsMps > USB_CTRLR_PKT_LEN_MAX(pHci->DevNo, BULK) ||
		pHci->AclFsMps > BT_HCI_USB_ACL_MAX_MPS ||
		(pHci->ScoEnabled && (!USB_ISO_SUPPORTED(pHci->DevNo) ||
		 isoMask == 0U || USB_CTRLR_PKT_LEN_MAX(pHci->DevNo, ISO) <
			BT_HCI_USB_SCO_MAX_MPS)) ||
		(USB_HIGHSPEED_CAPABLE(pHci->DevNo) &&
		 (pHci->EventHsMps > USB_CTRLR_PKT_LEN_MAX(pHci->DevNo, INT) ||
		  pHci->AclHsMps > USB_CTRLR_PKT_LEN_MAX(pHci->DevNo, BULK) ||
		  pHci->AclHsMps > BT_HCI_USB_ACL_MAX_MPS)))
	{
		return false;
	}

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 2U;
	req.BidirectionalCount = 1U;
	req.InCount = 1U;

	UsbdEpAllocRes_t alloc = {};
	uint8_t scoEp = 0U;
	bool registered = false;
	if (!pHci->ScoEnabled)
	{
		registered = UsbdEpAlloc(pHci->DevNo, &req, pClass, &alloc);
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
			if (UsbdEpAlloc(pHci->DevNo, &req, pClass, &alloc))
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
	dataCfg.BufferSize = BT_HCI_USB_ACL_MAX_MPS;

	UsbIsoIntrfCfg_t isoCfg = {};
	isoCfg.DevNo = pHci->DevNo;
	isoCfg.EpNo = pHci->ScoEpNo;
	isoCfg.EvtCB = BtHciUsbScoEvent;
	isoCfg.pContext = pHci;

	if (!UsbIntrfInit(pHci->pData, &dataCfg) ||
		(pHci->ScoEnabled && !pSco->Init(isoCfg)))
	{
		return false;
	}
	UsbCtrlrEpBind(pHci->DevNo, pHci->EventEpNo, true, false,
		BtHciUsbEventComplete, pHci);
	// HCI retains complete packets across alternate switches. The OUT path
	// stays with UsbIntrf; the packet-aware IN completion belongs here.
	UsbCtrlrEpBind(pHci->DevNo, pHci->AclEpNo, true, true,
		BtHciUsbBulkComplete, pHci);

	BtHciUsbInitDevIntrf(pHci);

	const void *pTemplate;
	uint16_t descLength;
	if (pHci->BulkSerializationSupported)
	{
		pTemplate = &s_BtHciUsbFullDescTemplate;
		descLength = pHci->ScoEnabled ?
			sizeof(BtHciUsbFullDesc_t) : sizeof(BtHciUsbSerialDesc_t);
	}
	else
	{
		pTemplate = &s_BtHciUsbScoDescTemplate;
		descLength = pHci->ScoEnabled ?
			sizeof(BtHciUsbScoDesc_t) : sizeof(BtHciUsbDesc_t);
	}

	return UsbDescRegister(pHci->DevNo, pClass,
		pTemplate, descLength, BtHciUsbPatchRegistered);
}

bool BtHciUsb::Init(const BtHciUsbCfg_t &Cfg)
{
	return BtHciUsbInitInternal(&vBtHciUsb, &vUsbDevIntrf, &vScoIso,
		&Cfg, this);
}

bool BtHciUsb::Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
						uint8_t **ppData, uint16_t *pLength)
{
	return BtHciUsbRequest(pSetup, Stage, ppData, pLength, &vBtHciUsb);
}

bool BtHciUsb::SelectConfig(uint8_t ConfigValue)
{
	return BtHciUsbConfig(&vBtHciUsb, ConfigValue);
}

bool BtHciUsb::SelectInterface(uint8_t InterfaceNo, uint8_t Option)
{
	return BtHciUsbSetInterface(&vBtHciUsb, InterfaceNo, Option);
}

void BtHciUsb::Reset()
{
	BtHciUsbReset(&vBtHciUsb);
}



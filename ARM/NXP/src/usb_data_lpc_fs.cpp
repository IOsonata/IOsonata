/**-------------------------------------------------------------------------
@file usb_data_lpc_fs.c
@brief Private shared LPC USB device (IP3511) bulk and interrupt endpoint
transfers, full and high speed.

The UsbCtrlr functions take the controller of DevNo from the context the port
gives (LpcUsbFsDataContext). A data endpoint moves one packet of its maximum
packet size at a time through its buffer in the USB SRAM.

@author Hoang Nguyen Hoan
@date Oct. 13, 2026
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
#include "usb_data_lpc_fs.h"

#define LPCUSB_ACTIVE (1UL << 31)
#define LPCUSB_DISABLED (1UL << 30)
#define LPCUSB_STALL (1UL << 29)
#define LPCUSB_TOGGLE_RESET (1UL << 28)
#define LPCUSB_RESET (1UL << 26)
#define LPCUSB_INTONNAK_AI (1UL << 13)
#define LPCUSB_FS_PKT_MAX 64U
#define LPCUSB_HS_BULK_PKT 512U

static void LpcUsbFsDataHalt(LpcUsbFsData_t *pData, unsigned Physical, bool Halt);

// USB.15: stop the IN NAK interrupts once no data IN endpoint waits for one
static void LpcUsbFsDataNakDone(LpcUsbFsData_t *pData)
{
	if (!pData->pEp0->bHsErrata)
		return;
	const unsigned count = 2U * pData->pEp0->NbEp;
	for (unsigned physical = 3U; physical < count; physical += 2U)
	{
		if (pData->Endpoint[physical - 2U].NakWait)
			return;
	}
	LpcUsbFsEp0Command(pData->pEp0, LPCUSB_INTONNAK_AI, 0U, 0U);
}

uint32_t LpcUsbFsDataMask(LpcUsbFsData_t *pData)
{
	return ((1U << (2U * pData->pEp0->NbEp)) - 1U) & ~3U;
}

void LpcUsbFsDataInit(LpcUsbFsData_t *pData)
{
	memset(pData->Endpoint, 0, sizeof(pData->Endpoint));
	pData->RxReady = 0U;
}

// The data context of DevNo with EpNo a data endpoint of it, else NULL
static LpcUsbFsData_t *LpcUsbFsDataEp(int DevNo, uint8_t EpNo)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataContext(DevNo);
	return pData == NULL || EpNo == 0U || EpNo >= pData->pEp0->NbEp ? NULL : pData;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataContext(DevNo);
	if (pData == NULL || EpNo >= pData->pEp0->NbEp || *pData->pClosing)
		return;
	const uint32_t state = DisableInterrupt();
	if (EpNo != 0U)
	{
		LpcUsbFsDataHalt(pData, 2U * EpNo + bIn, true);
		EnableInterrupt(state);
		return;
	}
	LpcUsbFsEp0Stall(pData->pEp0);
	EnableInterrupt(state);
}

// The controller owns only aligned USB SRAM. Caller payloads can have any
// alignment; a successful submission remains owned until its callback.
bool LpcUsbFsDataCurrent(LpcUsbFsData_t *pData)
{
	return *pData->pPrepared && pData->pEp0->Enabled && !*pData->pClosing && pData->pEp0->Ready() &&
		(pData->pEp0->pReg->DEVCMDSTAT & LPCUSB_RESET) == 0U;
}

static void LpcUsbFsDataNotify(LpcUsbFsData_t *pData, unsigned Physical, UsbCtrlrEvtType_t Event, uint16_t Length)
{
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[Physical - 2U];
	if (ep->Handler != NULL)
		ep->Handler(Event, Length, ep->pContext);
}

// Called only after retirement. Block all reentrant submissions while returning
// buffers to their owners; keep registrations across reset and Stop/Start.
void LpcUsbFsDataForget(LpcUsbFsData_t *pData, uint32_t Mask)
{
	const bool closing = *pData->pClosing;
	const unsigned count = 2U * pData->pEp0->NbEp;
	*pData->pClosing = true;
	for (unsigned physical = 2U; physical < count; ++physical)
	{
		if ((Mask & (1U << physical)) == 0U)
			continue;
		LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[physical - 2U];
		const bool busy = ep->Busy;
		ep->Busy = false;
		ep->Halted = false;
		ep->NakWait = false;
		ep->Mps = 0U;
		ep->pBuffer = NULL;
		++ep->Generation;
		pData->RxReady &= ~(1U << physical);
		pData->pEp0->pList[2U * physical] = LPCUSB_DISABLED;
		if (busy)
			LpcUsbFsDataNotify(pData, physical, USB_CTRLR_EVT_CANCEL, 0U);
	}
	LpcUsbFsDataNakDone(pData);
	*pData->pClosing = closing;
}

void UsbCtrlrEpBind(int DevNo, uint8_t EpNo, bool bIn, bool bBlocking,
	UsbCtrlrEpHandler_t Handler, void *pContext)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataEp(DevNo, EpNo);
	(void)bBlocking;
	if (pData == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	const unsigned physical = 2U * EpNo + bIn;
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[physical - 2U];
	// Never replace the owner of an accepted transfer.
	if (!*pData->pClosing && !ep->Busy)
	{
		ep->Handler = Handler;
		ep->pContext = pContext;
		++ep->Generation;
		if (LpcUsbFsDataCurrent(pData) && !bIn && ep->Mps != 0U && !ep->Halted && Handler != NULL)
		{
			pData->RxReady |= 1U << physical;
			NVIC_SetPendingIRQ(pData->Irq);
		}
	}
	EnableInterrupt(state);
}

// The packet sizes of the bus speed. Full speed: bulk 8, 16, 32 or 64,
// interrupt 1 to 64. High speed: bulk 512, interrupt 1 to the buffer size.
bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpNo, bool bIn, uint8_t Type, uint16_t MaxPacketSize)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataEp(DevNo, EpNo);
	if (pData == NULL || (Type != BULK && Type != INT))
		return false;
	const bool hs = pData->pEp0->bHighSpeed;
	const uint16_t limit = hs ? pData->pEp0->BufSize : LPCUSB_FS_PKT_MAX;
	if (MaxPacketSize == 0U || MaxPacketSize > limit ||
		(Type == BULK && (hs ? MaxPacketSize != LPCUSB_HS_BULK_PKT :
		 (MaxPacketSize != 8U && MaxPacketSize != 16U && MaxPacketSize != 32U && MaxPacketSize != 64U))))
		return false;
	const uint32_t state = DisableInterrupt();
	const unsigned physical = 2U * EpNo + bIn;
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[physical - 2U];
	if (!LpcUsbFsDataCurrent(pData) || ep->Mps != 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	ep->Mps = MaxPacketSize;
	ep->Busy = false;
	ep->Halted = false;
	ep->pBuffer = NULL;
	++ep->Generation;
	pData->pEp0->pList[2U * physical] = LpcUsbFsEp0BufferAddress(pData->pEp0, physical) | LPCUSB_TOGGLE_RESET;
	__DMB();
	pData->pEp0->pReg->INTSTAT = 1U << physical;
	pData->pEp0->pReg->INTEN |= 1U << physical;
	if (!bIn)
	{
		// Initial destination request runs in IRQ after class configuration.
		pData->RxReady |= 1U << physical;
		NVIC_SetPendingIRQ(pData->Irq);
	}
	EnableInterrupt(state);
	return true;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == NULL || (pDesc->bEndpointAddress & 0x70U) != 0U)
		return false;
	return UsbCtrlrEpOpenData(DevNo, pDesc->bEndpointAddress & 15U,
		(pDesc->bEndpointAddress & 0x80U) != 0U, pDesc->bmAttributes & 3U, pDesc->wMaxPacketSize);
}

static void LpcUsbFsDataClose(LpcUsbFsData_t *pData, uint32_t Mask)
{
	if (!*pData->pPrepared || *pData->pClosing)
		return;
	if (!LpcUsbFsEp0Retire(pData->pEp0, Mask))
	{
		pData->pEp0->Fault();
		return;
	}
	pData->pEp0->pReg->INTEN &= ~Mask;
	pData->pEp0->pReg->INTSTAT = Mask;
	LpcUsbFsDataForget(pData, Mask);
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpNo, bool bIn)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataEp(DevNo, EpNo);
	if (pData == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	LpcUsbFsDataClose(pData, 1U << (2U * EpNo + bIn));
	EnableInterrupt(state);
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataContext(DevNo);
	if (pData == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	LpcUsbFsDataClose(pData, LpcUsbFsDataMask(pData));
	EnableInterrupt(state);
}

// An IN after an OUT of 4 + 16 x N bytes waits for an IN NAK (USB.15): the
// list word is ready, without ACTIVE, until LpcUsbFsDataInterrupt sees it.
static void LpcUsbFsDataArm(LpcUsbFsData_t *pData, unsigned Physical)
{
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[Physical - 2U];
	const uint32_t toggle = pData->pEp0->pList[2U * Physical] & LPCUSB_TOGGLE_RESET;
	const bool wait = (Physical & 1U) != 0U && pData->pEp0->bHsErrata && pData->pEp0->OutQuirk;
	pData->pEp0->pReg->INTSTAT = 1U << Physical;
	__DMB();
	pData->pEp0->pList[2U * Physical] = LpcUsbFsEp0Entry(pData->pEp0, Physical, ep->Length) |
		toggle | (wait ? 0U : LPCUSB_ACTIVE);
	if (wait)
	{
		ep->NakWait = true;
		LpcUsbFsEp0Command(pData->pEp0, 0U, LPCUSB_INTONNAK_AI, 0U);
	}
}

bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Capacity)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataEp(DevNo, EpNo);
	if (pData == NULL || pBuffer == NULL)
		return false;
	const uint32_t state = DisableInterrupt();
	const unsigned physical = 2U * EpNo;
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[physical - 2U];
	if (!LpcUsbFsDataCurrent(pData) || ep->Mps == 0U || ep->Busy || Capacity < ep->Mps)
	{
		EnableInterrupt(state);
		return false;
	}
	ep->pBuffer = pBuffer;
	ep->Length = ep->Mps;
	ep->Busy = true;
	pData->RxReady &= ~(1U << physical);
	if (!ep->Halted)
		LpcUsbFsDataArm(pData, physical);
	EnableInterrupt(state);
	return true;
}

// Stage at most one packet. The rest stays in caller-owned memory until its
// turn; no extra SRAM or transfer-sized copy is needed in interrupt context.
static void LpcUsbFsDataStageIn(LpcUsbFsData_t *pData, unsigned Physical)
{
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[Physical - 2U];
	ep->Length = ep->Remaining < ep->Mps ? ep->Remaining : ep->Mps;
	if (ep->Length != 0U)
		memcpy(LpcUsbFsEp0Packet(pData->pEp0, Physical), ep->pBuffer, ep->Length);
}

bool UsbCtrlrEpSend(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Length)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataEp(DevNo, EpNo);
	if (pData == NULL || (Length != 0U && pBuffer == NULL))
		return false;
	const uint32_t state = DisableInterrupt();
	const unsigned physical = 2U * EpNo + 1U;
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[physical - 2U];
	if (!LpcUsbFsDataCurrent(pData) || ep->Mps == 0U || ep->Busy)
	{
		EnableInterrupt(state);
		return false;
	}
	ep->pBuffer = pBuffer;
	ep->Remaining = Length;
	ep->Transferred = 0U;
	ep->Busy = true;
	LpcUsbFsDataStageIn(pData, physical);
	if (!ep->Halted)
		LpcUsbFsDataArm(pData, physical);
	EnableInterrupt(state);
	return true;
}

static void LpcUsbFsDataComplete(LpcUsbFsData_t *pData, unsigned Physical, bool Failed)
{
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[Physical - 2U];
	const bool in = (Physical & 1U) != 0U;
	const uint32_t generation = ep->Generation;
	__DMB();
	// USB.2: the byte count after an IN packet is not correct
	const uint16_t remaining = in && pData->pEp0->bHsErrata ? 0U :
		LpcUsbFsEp0Length(pData->pEp0, pData->pEp0->pList[2U * Physical]);
	const bool failed = Failed || remaining > ep->Length || (in && remaining != 0U);
	if (!failed)
		LpcUsbFsEp0Quirk(pData->pEp0, in, (uint16_t)(ep->Length - remaining));
	if (in && !failed)
	{
		ep->Transferred += ep->Length;
		ep->Remaining -= ep->Length;
		if (ep->Remaining != 0U)
		{
			ep->pBuffer += ep->Length;
			LpcUsbFsDataStageIn(pData, Physical);
			// A halt can observe a completed packet before its IRQ. Keep
			// the next packet queued until clear, without an early callback.
			if (!ep->Halted)
				LpcUsbFsDataArm(pData, Physical);
			return;
		}
	}
	const uint16_t length = in ? ep->Transferred : (failed ? 0 : ep->Length - remaining);
	if (!in && !failed && length != 0U)
		memcpy(ep->pBuffer, LpcUsbFsEp0Packet(pData->pEp0, Physical), length);
	ep->Busy = false;
	ep->pBuffer = NULL;
	pData->pEp0->pList[2U * Physical] = LpcUsbFsEp0BufferAddress(pData->pEp0, Physical) |
		(ep->Halted ? LPCUSB_STALL : 0U);
	LpcUsbFsDataNotify(pData, Physical, failed ? USB_CTRLR_EVT_XFER_FAILED : USB_CTRLR_EVT_XFER_CMPL, length);
	if (LpcUsbFsDataCurrent(pData) && !in && ep->Generation == generation &&
		ep->Mps != 0U && !ep->Busy && !ep->Halted)
		LpcUsbFsDataNotify(pData, Physical, USB_CTRLR_EVT_DRDY, 0U);
}

// Halt retires an active packet and reports failure, rather than replaying a
// packet whose ACK may have raced EPSKIP (including a zero-length IN packet).
// A completed packet advances the transfer; any remaining packets wait until
// CLEAR_FEATURE. Submissions made while halted also stay queued in software.
static void LpcUsbFsDataHalt(LpcUsbFsData_t *pData, unsigned Physical, bool Halt)
{
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[Physical - 2U];
	if (!LpcUsbFsDataCurrent(pData) || ep->Mps == 0U || (Halt && ep->Halted))
		return;
	const bool wasHalted = ep->Halted;
	// An IN waiting for its NAK (USB.15) was not sent
	const bool active = (pData->pEp0->pList[2U * Physical] & LPCUSB_ACTIVE) != 0U || ep->NakWait;
	if (!LpcUsbFsEp0Retire(pData->pEp0, 1U << Physical))
	{
		pData->pEp0->Fault();
		return;
	}
	if (ep->NakWait)
	{
		ep->NakWait = false;
		LpcUsbFsDataNakDone(pData);
	}
	ep->Halted = true;
	pData->pEp0->pReg->INTSTAT = 1U << Physical;
	++ep->Generation;
	const uint32_t generation = ep->Generation;
	if (ep->Busy && !wasHalted)
		LpcUsbFsDataComplete(pData, Physical, active);
	if (!LpcUsbFsDataCurrent(pData) || ep->Generation != generation || ep->Mps == 0U)
		return;
	ep->Halted = Halt;
	pData->pEp0->pList[2U * Physical] = LpcUsbFsEp0BufferAddress(pData->pEp0, Physical) |
		(Halt ? LPCUSB_STALL : LPCUSB_TOGGLE_RESET);
	__DMB();
	if (!Halt && ep->Busy)
		LpcUsbFsDataArm(pData, Physical);
	else if (!Halt && (Physical & 1U) == 0U)
	{
		pData->RxReady |= 1U << Physical;
		NVIC_SetPendingIRQ(pData->Irq);
	}
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool bIn)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataEp(DevNo, EpNo);
	if (pData == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	LpcUsbFsDataHalt(pData, 2U * EpNo + bIn, false);
	EnableInterrupt(state);
}

void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool bIn, UsbCtrlrEvtType_t Event, uint16_t Value)
{
	LpcUsbFsData_t *pData = LpcUsbFsDataEp(DevNo, EpNo);
	if (pData == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	const unsigned physical = 2U * EpNo + bIn;
	if (LpcUsbFsDataCurrent(pData) && pData->Endpoint[physical - 2U].Mps != 0U)
		LpcUsbFsDataNotify(pData, physical, Event, Value);
	EnableInterrupt(state);
}

void LpcUsbFsDataInterrupt(LpcUsbFsData_t *pData, unsigned Physical)
{
	LpcUsbFsDataEndpoint_t *ep = &pData->Endpoint[Physical - 2U];
	const uint32_t mask = 1U << Physical;
	if ((pData->pEp0->pReg->INTSTAT & pData->pEp0->pReg->INTEN & mask) != 0U)
	{
		pData->pEp0->pReg->INTSTAT = mask;
		if (ep->NakWait)
		{
			// The IN token NAKed (USB.15): the packet goes with the next one
			ep->NakWait = false;
			pData->pEp0->OutQuirk = false;
			LpcUsbFsDataNakDone(pData);
			if (ep->Mps != 0U && ep->Busy && !ep->Halted)
				LpcUsbFsDataArm(pData, Physical);
		}
		else if (ep->Mps != 0U && ep->Busy && !ep->Halted &&
			(pData->pEp0->pList[2U * Physical] & LPCUSB_ACTIVE) == 0U)
			LpcUsbFsDataComplete(pData, Physical, false);
	}
	if (LpcUsbFsDataCurrent(pData) && (pData->RxReady & mask) != 0U)
	{
		pData->RxReady &= ~mask;
		if (ep->Mps != 0U && !ep->Busy && !ep->Halted)
			LpcUsbFsDataNotify(pData, Physical, USB_CTRLR_EVT_DRDY, 0U);
	}
}

bool UsbCtrlrIsoInit(int DevNo)
{
	(void)DevNo;
	return false;
}

bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn, uint16_t MaxPacketSize)
{
	(void)DevNo;
	(void)EpNo;
	(void)bIn;
	(void)MaxPacketSize;
	return false;
}

bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Length)
{
	(void)DevNo;
	(void)EpNo;
	(void)pBuffer;
	(void)Length;
	return false;
}

uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;
	if (ppData != NULL)
		*ppData = NULL;
	return 0U;
}


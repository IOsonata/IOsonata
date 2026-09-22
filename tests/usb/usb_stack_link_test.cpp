/**-------------------------------------------------------------------------
@file	usb_stack_link_test.cpp

@brief	Brings the whole USB device stack up against a no-op controller.

Links usb.cpp, usb_intrf.cpp, usbd_cdc.cpp and usbd_cdc_desc.cpp together and
runs the sequence an application runs: UsbInit, two UsbdCdc initializations,
UsbEnable and UsbProcess. It also verifies that the class registration assigns
the established dual-CDC endpoint layout without placement fields in either
application configuration.

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
#include <stdio.h>
#include <string.h>

#include "usb/usb.h"
#include "usb/usbd_cdc.h"
#include <type_traits>
#include "usb/usbd_bulk.h"
#include "usb/usbd_hid.h"
#include "usb/usbd_msc.h"
#include "bluetooth/bt_hci_usb.h"

static_assert(std::is_base_of<DeviceIntrf, UsbIntrf>::value, "USB transport root");
static_assert(std::is_base_of<UsbIntrf, UsbdCdc>::value, "CDC transport");
static_assert(std::is_base_of<UsbIntrf, UsbdBulk>::value, "Bulk transport");
static_assert(std::is_base_of<UsbIntrf, UsbdMsc>::value, "MSC transport");
static_assert(std::is_base_of<UsbIntrf, BtHciUsb>::value, "HCI transport");
static_assert(std::is_base_of<UsbIntrf, UsbIsoIntrf>::value, "ISO transport");
static_assert(std::is_base_of<UsbIntrf, UsbIntIntrf>::value, "Interrupt transport");
static_assert(std::is_base_of<UsbIntIntrf, UsbdHid>::value, "HID interrupt transport");


static uint8_t s_RegisteredEp[6];
static int s_RegisteredEpCount;
static int s_EpOpenCount;
static int s_Ep0EventCount;
static UsbCtrlrEpHandler_t s_EpHandler[16];
static void *s_EpContext[16];
static unsigned s_EpSendCount[8];
static uint8_t s_LastEp0Addr;
static uint16_t s_LastEp0Length;

bool UsbCtrlrInit(int, const UsbCtrlrCfg_t *pCfg)
{
	return pCfg != nullptr;
}
bool UsbCtrlrStart(int) { return true; }
void UsbCtrlrStop(int) {}
void UsbCtrlrProcess(int) {}
bool UsbCtrlrVbusDetected(int) { return true; }
bool UsbCtrlrHighSpeed(int) { return false; }
void UsbCtrlrIntEnable(int) {}
void UsbCtrlrIntDisable(int) {}
void UsbCtrlrConnect(int) {}
void UsbCtrlrDisconnect(int) {}
void UsbCtrlrRemoteWakeup(int) {}
void UsbCtrlrSofEnable(int, bool) {}
void UsbCtrlrSetAddress(int, uint8_t) {}
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *)
{
	s_EpOpenCount++;
	return true;
}
void UsbCtrlrEpClose(int, uint8_t, bool) {}
void UsbCtrlrEpCloseAll(int) {}
void UsbCtrlrEpAlloc(int, uint8_t EpNo, bool bIn, uint8_t *, bool,
						UsbCtrlrEpHandler_t Handler, void *pContext)
{
	if (s_RegisteredEpCount < (int)sizeof(s_RegisteredEp))
	{
		s_RegisteredEp[s_RegisteredEpCount++] = (uint8_t)(EpNo |
			(bIn ? USB_ENDPADDR_DIR_IN : 0U));
	}
	const unsigned index = EpNo + (bIn ? 8U : 0U);
	s_EpHandler[index] = Handler;
	s_EpContext[index] = pContext;
}
bool UsbCtrlrEpSend(int, uint8_t EpNo, uint8_t *, uint16_t)
{
	s_EpSendCount[EpNo]++;
	return true;
}
static bool RecordEp0(uint8_t EpAddr, uint16_t Length)
{
	s_Ep0EventCount++;
	s_LastEp0Addr = EpAddr;
	s_LastEp0Length = Length;
	return true;
}
int UsbCtrlrEp0Send(int, uint8_t *, int Length)
{
	return RecordEp0(USB_ENDPADDR_DIR_IN, Length) ? Length : -1;
}
bool UsbCtrlrEp0Status(int, uint8_t EpAddr)
{
	return RecordEp0(EpAddr, 0);
}
void UsbCtrlrEpStall(int, uint8_t, bool) {}
void UsbCtrlrEpClearStall(int, uint8_t, bool) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }

#define RX_MEM_SIZE USB_INTRF_RXMEM_SIZE(4, USB_CTRLR_PKT_LEN_MAX(0, BULK))
#define TX_MEM_SIZE CFIFO_MEMSIZE(1024)

alignas(4) static uint8_t s_RxMem0[RX_MEM_SIZE];
alignas(4) static uint8_t s_TxMem0[TX_MEM_SIZE];
alignas(4) static uint8_t s_RxMem1[RX_MEM_SIZE];
alignas(4) static uint8_t s_TxMem1[TX_MEM_SIZE];
static UsbdCdc s_Cdc0;
static UsbdCdc s_Cdc1;

static void Setup(uint8_t Request, uint16_t Value)
{
	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_SETUP;
	evt.Setup.bmRequestType = USB_REQTYPE_DIRDEV | USB_REQTYPE_STANDARD |
		USB_REQTYPE_DEVICE;
	evt.Setup.bRequest = Request;
	evt.Setup.wValue = Value;
	UsbDevProcessEvent(0, &evt);
}

static void CompleteEp0In(void)
{
	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_XFER_CMPL;
	evt.Xfer.EpAddr = USB_ENDPADDR_DIRIN(0);
	evt.Xfer.Result = USB_CTRLR_XFER_SUCCESS;
	UsbDevProcessEvent(0, &evt);
}

static void SetControlLineState(uint8_t InterfaceNo, uint16_t State)
{
	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_SETUP;
	evt.Setup.bmRequestType = USB_REQTYPE_DIRDEV | USB_REQTYPE_CLASS |
		USB_REQTYPE_INTERFACE;
	evt.Setup.bRequest = USB_CDC_REQ_SET_CTRL_LINE_STATE;
	evt.Setup.wValue = State;
	evt.Setup.wIndex = InterfaceNo;
	UsbDevProcessEvent(0, &evt);
}

static UsbdCdcCfg_t CdcCfg(uint8_t *pRx, int RxSize,
						   uint8_t *pTx, int TxSize)
{
	UsbdCdcCfg_t cfg = {};
	cfg.bBlocking = true;
	cfg.RxFifoMemSize = RxSize;
	cfg.pRxFifoMem = pRx;
	cfg.TxFifoMemSize = TxSize;
	cfg.pTxFifoMem = pTx;
	cfg.DevNo = 0;
	return cfg;
}

int main(void)
{
	UsbCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.Vid = 0x1209;
	cfg.Pid = 1;
	cfg.DeviceClass = USB_DEVCLASS_MISC;
	cfg.DeviceSubClass = 2U;
	cfg.DeviceProtocol = 1U;
	cfg.bRemoteWakeup = true;
	if (!UsbInit(&cfg))
	{
		printf("UsbInit failed\n");
		return 1;
	}

	const UsbdCdcCfg_t cdc0 = CdcCfg(s_RxMem0, sizeof(s_RxMem0),
									  s_TxMem0, sizeof(s_TxMem0));
	const UsbdCdcCfg_t cdc1 = CdcCfg(s_RxMem1, sizeof(s_RxMem1),
									  s_TxMem1, sizeof(s_TxMem1));

	if (!s_Cdc0.Init(cdc0) || !s_Cdc1.Init(cdc1))
	{
		printf("UsbdCdc::Init failed\n");
		return 2;
	}
	if (s_Cdc0.Data() == nullptr || s_Cdc1.Data() == nullptr)
	{
		printf("UsbdCdc data binding failed\n");
		return 5;
	}
	uint16_t descriptorLength = 0U;
	const uint8_t *pDescriptor = UsbGetDescriptor(0,
		USB_DESCTYPE_CONFIGURATION, 0U, 0U, USB_SPEED_FULL,
		&descriptorLength);
	if (pDescriptor == nullptr ||
		descriptorLength != sizeof(UsbCfgDesc_t) + 2U * sizeof(UsbdCdcDesc_t) ||
		pDescriptor[4] != 4U ||
		pDescriptor[1] != USB_DESCTYPE_CONFIGURATION)
	{
		printf("CDC descriptor composition failed\n");
		return 8;
	}
	const UsbdCdcDesc_t *pCdcDesc = reinterpret_cast<const UsbdCdcDesc_t *>(
		pDescriptor + sizeof(UsbCfgDesc_t));
	if (pCdcDesc[0].Association.bFirstInterface != 0U ||
		pCdcDesc[1].Association.bFirstInterface != 2U)
	{
		printf("CDC descriptor order failed\n");
		return 9;
	}
	// C++ CDC objects register through the common UsbClass object array.
	// Registering the same object a second time must be rejected.
	if (UsbClassRegister(0, &s_Cdc0, 0, 0, 0, 0))
	{
		printf("UsbdCdc class object was not registered\n");
		return 7;
	}

	// UsbIntrf registers data OUT then data IN, followed by the class
	// notification IN endpoint. No application configuration supplied any
	// of these numbers.
	const uint8_t expected[] = {
		USB_ENDPADDR_DIROUT(2U), USB_ENDPADDR_DIRIN(2U),
		USB_ENDPADDR_DIRIN(1U),
		USB_ENDPADDR_DIROUT(4U), USB_ENDPADDR_DIRIN(4U),
		USB_ENDPADDR_DIRIN(3U),
	};
	if (s_RegisteredEpCount != (int)sizeof(expected) ||
		memcmp(s_RegisteredEp, expected, sizeof(expected)) != 0)
	{
		printf("CDC automatic endpoint placement failed\n");
		return 6;
	}

	if (!UsbEnable(0))
	{
		printf("UsbEnable failed\n");
		return 3;
	}

	UsbdCdcDev_t *pCdc0 = s_Cdc0;
	UsbIntrf *pTransport = &s_Cdc0;
	DeviceIntrf *pDevice = pTransport;
	if (pTransport->Data() != &pCdc0->pData->DevIntrf ||
		static_cast<DevIntrf_t *>(*pDevice) != s_Cdc0.Data())
	{
		printf("CDC does not share the UsbIntrf endpoint state\n");
		return 14;
	}
	pCdc0->LineCoding.dwDTERate = 9600U;
	UsbCtrlrEvt_t reset = {};
	reset.Type = USB_CTRLR_EVT_RESET;
	UsbDevProcessEvent(0, &reset);
	if (pCdc0->LineCoding.dwDTERate != 115200U)
	{
		printf("UsbdCdc virtual reset was not dispatched\n");
		return 8;
	}

	Setup(USB_REQ_SET_ADDRESS, 5U);
	if (s_Ep0EventCount != 1 || s_LastEp0Addr != USB_ENDPADDR_DIRIN(0) ||
		s_LastEp0Length != 0U || UsbGetAddress(0) != 0U)
	{
		printf("CDC SET_ADDRESS setup failed\n");
		return 9;
	}
	CompleteEp0In();
	if (UsbGetAddress(0) != 5U)
	{
		printf("CDC SET_ADDRESS completion failed\n");
		return 10;
	}

	Setup(USB_REQ_SET_CONFIGURATION, 1U);
	if (s_Ep0EventCount != 2 || s_LastEp0Addr != USB_ENDPADDR_DIRIN(0) ||
		s_LastEp0Length != 0U || !UsbConfigured(0) || s_EpOpenCount != 6 ||
		pCdc0->pData->Mps != USB_CTRLR_PKT_LEN_MAX(0, BULK) ||
		((UsbdCdcDev_t *)s_Cdc1)->pData->Mps != USB_CTRLR_PKT_LEN_MAX(0, BULK))
	{
		printf("C++ CDC configuration was not applied\n");
		return 11;
	}
	CompleteEp0In();

	// CDC notification owns one transfer buffer. Multiple state changes while
	// the initial notification is in flight must coalesce, not queue that
	// same buffer more than once.
	if (s_EpSendCount[pCdc0->NotifyEpNo] != 1U)
	{
		printf("CDC initial notification was not submitted exactly once\n");
		return 15;
	}
	UsbdCdcSetSerialState(pCdc0, 1U);
	UsbdCdcSetSerialState(pCdc0, 2U);
	if (s_EpSendCount[pCdc0->NotifyEpNo] != 1U)
	{
		printf("CDC notification buffer was submitted while already active\n");
		return 16;
	}
	const unsigned notifyIndex = pCdc0->NotifyEpNo + 8U;
	s_EpHandler[notifyIndex](USB_CTRLR_EVT_XFER_CMPL,
		USBD_CDC_NOTIFY_LEN, s_EpContext[notifyIndex]);
	if (s_EpSendCount[pCdc0->NotifyEpNo] != 2U)
	{
		printf("CDC pending notification was not chained after completion\n");
		return 17;
	}
	s_EpHandler[notifyIndex](USB_CTRLR_EVT_XFER_CMPL,
		USBD_CDC_NOTIFY_LEN, s_EpContext[notifyIndex]);
	if (s_EpSendCount[pCdc0->NotifyEpNo] != 2U)
	{
		printf("CDC notification completion duplicated a transfer\n");
		return 18;
	}

	SetControlLineState(pCdc0->CtrlIfNo, USB_CDC_CTRL_LINE_STATE_DTR);
	if (s_Ep0EventCount != 3 || s_LastEp0Addr != USB_ENDPADDR_DIRIN(0) ||
		s_LastEp0Length != 0U || s_Cdc0.IsPortOpen())
	{
		printf("C++ CDC Control setup was not dispatched\n");
		return 12;
	}
	CompleteEp0In();
	if (!s_Cdc0.IsPortOpen())
	{
		printf("C++ CDC Control completion was not dispatched\n");
		return 13;
	}

	UsbProcess(0);
	printf("UsbInit, dual UsbdCdc Init, UsbEnable, UsbProcess all completed\n");
	return UsbConfigured(0) ? 0 : 4;
}

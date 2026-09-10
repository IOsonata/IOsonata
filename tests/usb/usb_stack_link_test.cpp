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

static uint8_t s_RegisteredEp[6];
static int s_RegisteredEpCount;
static UsbCtrlrEvtHandler_t s_CoreHandler;
static void *s_CoreContext;
static int s_EpOpenCount;
static int s_Ep0XferCount;
static uint8_t s_LastEp0Addr;
static uint16_t s_LastEp0Length;

bool UsbCtrlrInit(int, const UsbCtrlrCfg_t *pCfg)
{
	s_CoreHandler = pCfg->EvtHandler;
	s_CoreContext = pCfg->pContext;
	return true;
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
void UsbCtrlrEpClose(int, uint8_t) {}
void UsbCtrlrEpCloseAll(int) {}
bool UsbCtrlrEpRegister(int, uint8_t EpAddr, uint8_t *, bool,
						UsbCtrlrEpHandler_t, void *)
{
	if (s_RegisteredEpCount >= (int)sizeof(s_RegisteredEp))
	{
		return false;
	}

	s_RegisteredEp[s_RegisteredEpCount++] = EpAddr;
	return true;
}
bool UsbCtrlrEpXfer(int, uint8_t, uint16_t) { return true; }
bool UsbCtrlrEp0Xfer(int, uint8_t EpAddr, uint8_t *, uint16_t Length)
{
	s_Ep0XferCount++;
	s_LastEp0Addr = EpAddr;
	s_LastEp0Length = Length;
	return true;
}
void UsbCtrlrEpStall(int, uint8_t) {}
void UsbCtrlrEpClearStall(int, uint8_t) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }

#define RX_MEM_SIZE USB_INTRF_RXMEM_SIZE(4, USB_PKT_MAXLEN(0, BULK))
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
	s_CoreHandler(0, &evt, s_CoreContext);
}

static void CompleteEp0In(void)
{
	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_XFER_CMPL;
	evt.Xfer.EpAddr = USB_ENDPADDR_DIRIN(0);
	evt.Xfer.Result = USB_CTRLR_XFER_SUCCESS;
	s_CoreHandler(0, &evt, s_CoreContext);
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
	s_CoreHandler(0, &evt, s_CoreContext);
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
	cfg.NbCdc = 2;
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
	// C++ CDC objects register through the common UsbClass object array.
	// Registering the same object a second time must be rejected.
	UsbdClassCfg_t duplicate = {};
	if (UsbClassRegister(0, &duplicate, &s_Cdc0))
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
	pCdc0->LineCoding.dwDTERate = 9600U;
	UsbCtrlrEvt_t reset = {};
	reset.Type = USB_CTRLR_EVT_RESET;
	s_CoreHandler(0, &reset, s_CoreContext);
	if (pCdc0->LineCoding.dwDTERate != 115200U)
	{
		printf("UsbdCdc virtual reset was not dispatched\n");
		return 8;
	}

	Setup(USB_REQ_SET_ADDRESS, 5U);
	if (s_Ep0XferCount != 1 || s_LastEp0Addr != USB_ENDPADDR_DIRIN(0) ||
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
	if (s_Ep0XferCount != 2 || s_LastEp0Addr != USB_ENDPADDR_DIRIN(0) ||
		s_LastEp0Length != 0U || !UsbConfigured(0) || s_EpOpenCount != 6 ||
		pCdc0->IntrfData.Mps != USB_PKT_MAXLEN(0, BULK) ||
		((UsbdCdcDev_t *)s_Cdc1)->IntrfData.Mps != USB_PKT_MAXLEN(0, BULK))
	{
		printf("C++ CDC configuration was not applied\n");
		return 11;
	}
	CompleteEp0In();

	SetControlLineState(pCdc0->CtrlIfNo, USB_CDC_CTRL_LINE_STATE_DTR);
	if (s_Ep0XferCount != 3 || s_LastEp0Addr != USB_ENDPADDR_DIRIN(0) ||
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

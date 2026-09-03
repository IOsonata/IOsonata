/**-------------------------------------------------------------------------
@file	usb_stack_link_test.cpp

@brief	Brings the whole USB device stack up against a no-op controller.

Links usb.cpp, usb_intrf.cpp, usbd_cdc.cpp and usbd_cdc_desc.cpp together and
runs the sequence an application runs: UsbInit, UsbdCdcInit, UsbEnable,
UsbProcess. It catches what the unit tests cannot, a configuration field the
class layer forgets to fill in, because it is the only test that calls the
class layer the way an application does.

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
bool UsbCtrlrInit(int, const UsbCtrlrCfg_t *) { return true; }
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
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *) { return true; }
void UsbCtrlrEpClose(int, uint8_t) {}
void UsbCtrlrEpCloseAll(int) {}
bool UsbCtrlrEpXfer(int, uint8_t, uint8_t *, uint16_t) { return true; }
bool UsbCtrlrEpBusy(int, uint8_t) { return false; }
void UsbCtrlrEpStall(int, uint8_t) {}
void UsbCtrlrEpClearStall(int, uint8_t) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }
alignas(4) static uint8_t s_RxMem[USB_INTRF_RXMEM_SIZE(4, USB_PKT_MAXLEN(0, BULK))];
alignas(4) static uint8_t s_TxMem[CFIFO_MEMSIZE(1024)];
static UsbdCdc s_Cdc;

int main(void)
{
	UsbCfg_t cfg = {};
	cfg.DevNo = 0; cfg.Vid = 0x1209; cfg.Pid = 1; cfg.NbCdc = 1;
	if (!UsbInit(&cfg)) { printf("UsbInit failed\n"); return 1; }

	// The example does this next, and it is what actually exercises the
	// class layer. Skipping it is how a zero TxFifoBlkSize got shipped.
	UsbdCdcCfg_t cdc = {};
	cdc.bBlocking = true;
	cdc.RxFifoMemSize = (int)sizeof(s_RxMem);
	cdc.pRxFifoMem = s_RxMem;
	cdc.TxFifoMemSize = (int)sizeof(s_TxMem);
	cdc.pTxFifoMem = s_TxMem;
	cdc.ItfNo = 0;
	cdc.DevNo = 0;
	if (!s_Cdc.Init(cdc)) { printf("UsbdCdc::Init failed\n"); return 2; }
	if (s_Cdc.Data() == nullptr) { printf("UsbdCdc data binding failed\n"); return 5; }

	if (!UsbEnable(0)) { printf("UsbEnable failed\n"); return 3; }
	UsbProcess(0);
	printf("UsbInit, UsbdCdcInit, UsbEnable, UsbProcess all completed\n");
	return UsbConfigured(0) ? 4 : 0;
}

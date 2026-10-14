/**-------------------------------------------------------------------------
@file	usb_cdc_loopback.cpp

@brief	LPC1769 USB CDC loopback example.

Uses the generic CDC class and AppRun. Set the oscillator and USB wiring
in board.h before use. Hardware validation is pending.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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
#include "app_evt_handler.h"
#include "coredev/system_core_clock.h"
#include "usb/usbd_cdc.h"
#include "board.h"

#define CDC_RX_SIZE USB_INTRF_RXMEM_SIZE(4, USBD_CDC_BULK_FS_MPS)
#define CDC_TX_SIZE CFIFO_MEMSIZE(1024)

McuOsc_t g_McuOsc = MCUOSC;
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
alignas(4) static uint8_t s_RxMemory[CDC_RX_SIZE];
alignas(4) static uint8_t s_TxMemory[CDC_TX_SIZE];
static uint8_t s_Buffer[USBD_CDC_BULK_FS_MPS];
static int s_Pending;
static int s_Offset;
static UsbdCdc s_Cdc;
static const IOPinCfg_t s_Pins[] = USB_PINS_MAP;
static const UsbdCdcCfg_t s_CdcConfig = {
	.DevNo = 0,
	.bBlocking = false,
	.RxFifoMemSize = sizeof(s_RxMemory),
	.pRxFifoMem = s_RxMemory,
	.TxFifoMemSize = sizeof(s_TxMemory),
	.pTxFifoMem = s_TxMemory,
};
static const UsbCfg_t s_Config = {
	.DevNo = 0,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0001,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "LPC1769 CDC Loopback",
	.pSerial = nullptr,
	.pFuncName = "IOsonata CDC",
	.IntPrio = 6,
	.pIOPinMap = s_Pins,
	.NbIOPins = sizeof(s_Pins) / sizeof(s_Pins[0]),
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2,
	.DeviceProtocol = 1,
	.bRemoteWakeup = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
};

bool AppCheckStatus(void)
{
	UsbCheckStatus();
	if (!s_Cdc.IsPortOpen())
	{
		s_Pending = 0;
		s_Offset = 0;
		return !AppEvtHandlerPending();
	}
	if (s_Pending == 0)
	{
		s_Pending = s_Cdc.Rx(0, s_Buffer, sizeof(s_Buffer));
		s_Offset = 0;
		if (s_Pending <= 0)
		{
			s_Pending = 0;
			return !AppEvtHandlerPending();
		}
	}
	const int sent = s_Cdc.Tx(0, &s_Buffer[s_Offset], s_Pending);
	if (sent > 0)
	{
		s_Offset += sent;
		s_Pending -= sent;
	}
	// Keep draining queued RX data and retry partial nonblocking writes.
	return false;
}

int main(void)
{
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_Config) || !s_Cdc.Init(s_CdcConfig) || !UsbEnable(0))
		return 1;
	AppRun();
}

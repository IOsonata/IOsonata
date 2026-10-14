/**-------------------------------------------------------------------------
@example	uart_usb_bridge.cpp


@brief	UART to USB CDC bridge

Demo code using IOsonata library to connect a UART to the host through a USB
CDC port. The device appears on the host as a serial port. Bytes the host
writes to the port go out on the UART TX pin, bytes received on the UART RX
pin go to the host.

The bridge is transparent: nothing is added to the data in either direction.
The UART rate follows the rate the host sets on the port. The frame stays 8N1,
the UART is not reconfigured for data bits, parity or stop bits.

Data received on the UART before the host opens the port stays in the UART
receive FIFO and goes to the host when the port opens, so a banner printed at
power up by the other side is not lost. Past the FIFO size, new bytes are
dropped until the host reads.

The UART and its pins are defined in board.h of the target project:
UART_DEVNO, UART_PINS and optionally UART_RATE (start rate, 115200 by default)
and UART_FLOWCTRL (UART_FLWCTRL_NONE by default). With UART_FLWCTRL_HW,
UART_PINS lists RX, TX, CTS and RTS in that order.

Nothing here is specific to an MCU. The USB device controller, its clock and
its cable detect are behind UsbInit and friends, and one port file answers
them per MCU family, so the same source builds for every target that has a
USB device controller.

USB RX/TX packet progress is interrupt driven. Deferred endpoint work, attach,
detach and class housekeeping are queued by the USB stack in the application
event queue, which the main loop runs with AppEvtHandlerExec.

@author	Hoang Nguyen Hoan
@date	Oct. 9, 2026

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

#include <stdint.h>

#include "cfifo.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "coredev/uart.h"
#include "usb/usb.h"
#include "usb/usbd_cdc.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#if !defined(UART_DEVNO) || !defined(UART_PINS)
#error "Define UART_DEVNO and UART_PINS in board.h"
#endif

#ifndef UART_RATE
#define UART_RATE				115200
#endif

#ifndef UART_FLOWCTRL
#define UART_FLOWCTRL			UART_FLWCTRL_NONE
#endif

#define USB_DEVNO				0

// One USB packet per transfer in each direction
#define BRIDGE_BUFF_SIZE		USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK)

// The application owns queued RX/TX memory. UsbdCdc owns the controller
// transfer buffers and copies completed OUT packets into this RX packet FIFO.
#define CDC_RXFIFO_PKTCNT		4
#define CDC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK))
#define CDC_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(1024)

// The UART receive FIFO also holds what arrives before the host opens the
// port.
#define UART_RXFIFO_MEMSIZE		CFIFO_MEMSIZE(1024)
#define UART_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(512)

alignas(4) static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE];
alignas(4) static uint8_t s_UartRxFifoMem[UART_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_UartTxFifoMem[UART_TXFIFO_MEMSIZE];

static const IOPinCfg_t s_UartPins[] = UART_PINS;

// Blocking FIFOs: a full FIFO refuses new bytes rather than overwriting the
// ones not sent yet.
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_RATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLOWCTRL,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = UART_RXFIFO_MEMSIZE,
	.pRxMem = s_UartRxFifoMem,
	.TxMemSize = UART_TXFIFO_MEMSIZE,
	.pTxMem = s_UartTxFifoMem,
	.bDMAMode = true,
};

// USB CDC configuration. No event handler: the bridge does not write
// anything of its own to the port.
static const UsbdCdcCfg_t s_CdcCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = CDC_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_CdcTxFifoMem,
	.EvtCB = nullptr,
};

#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

// Application event queue memory, replaces the 4 event library default. The
// USB controller port queues its deferred endpoint events there.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

// USB device configuration.
//
// 0x1209 is the pid.codes vendor id, which exists for open hardware and is
// what the other USB demos in this tree use. Put your own vendor and product
// id here before shipping anything : a duplicate pair makes the host reuse a
// driver and a saved COM port from somebody else's board.
static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0001,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata UART USB Bridge",
	.pSerial = nullptr,			// Taken from the MCU unique id
	.pFuncName = "IOsonata UART",
	.IntPrio = 6,
#ifdef USB_PINS
	.pIOPinMap = s_UsbPins,
	.NbIOPins = sizeof(s_UsbPins) / sizeof(IOPinCfg_t),
#else
	.pIOPinMap = nullptr,
	.NbIOPins = 0,
#endif
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2U,
	.DeviceProtocol = 1U,
	.bSelfPowered = false,
	.bRemoteWakeup = true,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

UART g_Uart;
UsbdCdc g_Cdc;

// Data waiting to go out on one side. Kept until the receiving side takes all
// of it, so a full FIFO delays the data rather than dropping it.
typedef struct {
	uint8_t Buff[BRIDGE_BUFF_SIZE];
	int Len;
	int Offset;
} BridgeBuff_t;

static BridgeBuff_t s_HostToUart;
static BridgeBuff_t s_UartToHost;

// Rate the UART was last set to from the host line coding
static uint32_t s_UartRate = UART_RATE;

// The host sets the line coding when it opens or reconfigures the port. Only
// the rate is applied.
static void BridgeLineCoding(void)
{
	const UsbCdcLineCoding_t *pLc = g_Cdc.LineCoding();

	if (pLc != nullptr && pLc->dwDTERate != 0U && pLc->dwDTERate != s_UartRate)
	{
		s_UartRate = pLc->dwDTERate;
		g_Uart.Rate(s_UartRate);
	}
}

static void BridgeHostToUart(void)
{
	BridgeBuff_t *p = &s_HostToUart;

	if (p->Len <= 0)
	{
		p->Len = g_Cdc.Rx(0, p->Buff, sizeof(p->Buff));
		p->Offset = 0;
	}

	if (p->Len > 0)
	{
		int n = g_Uart.Tx(&p->Buff[p->Offset], p->Len);

		if (n > 0)
		{
			p->Offset += n;
			p->Len -= n;
		}
	}
}

static void BridgeUartToHost(void)
{
	BridgeBuff_t *p = &s_UartToHost;

	// Nothing is read from the UART until the host opens the port, the
	// receive FIFO keeps it meanwhile.
	if (g_Cdc.IsPortOpen() == false)
	{
		return;
	}

	if (p->Len <= 0)
	{
		p->Len = g_Uart.Rx(p->Buff, sizeof(p->Buff));
		p->Offset = 0;
	}

	if (p->Len > 0)
	{
		int n = g_Cdc.Tx(0, &p->Buff[p->Offset], p->Len);

		if (n > 0)
		{
			p->Offset += n;
			p->Len -= n;
		}
	}
}

int main()
{
	if (g_Uart.Init(s_UartCfg) == false)
	{
		return -1;
	}

	if (AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) == false ||
		UsbInit(&s_UsbCfg) == false)
	{
		return -1;
	}

	if (g_Cdc.Init(s_CdcCfg) == false)
	{
		return -1;
	}

	// A board on a battery starts with no cable in it, so this failing is
	// not an error. UsbProcess notices the attach and comes back to it.
	UsbEnable(USB_DEVNO);

	while (1)
	{
		if (AppEvtHandlerExec() == false)
		{
			UsbCheckStatus();
		}

		BridgeLineCoding();
		BridgeHostToUart();
		BridgeUartToHost();
	}

	return 0;
}

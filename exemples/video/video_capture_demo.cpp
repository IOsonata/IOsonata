/**-------------------------------------------------------------------------
@example	video_capture_demo.cpp

@brief	Video capture to the console UART

Captures JPEG frames from the video device that board.h selects and sends
each frame on the console UART to Python/video_rx.py, which writes the frames
to JPEG files. board.h selects the device by its name:

	ARDUCAM_MEGA	ArduCAM Mega SPI camera module (2MP, 3MP or 5MP)

and gives the SPI bus with SPI_DEVNO and SPI_PINS (SCK, MISO, MOSI, CS), the
console UART with UART_DEVNO and the UART_RX_ and UART_TX_ pins. board.h can
change VIDEO_DEMO_WIDTH, VIDEO_DEMO_HEIGHT (640x480 by default) and
VIDEO_DEMO_SPI_RATE (8 MHz by default).

Frame on the UART : the 4 characters VFRM, the frame length in bytes 32 bits
little endian, then the frame data. Everything else on the UART is text. The
Release build prints the text on the UART, the Debug build on semihosting.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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

#include "coredev/uart.h"
#include "coredev/spi.h"
#include "coredev/interrupt.h"
#include "stddev.h"
#include "video/cam_arducam_mega.h"
#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE			1000000
#endif

#ifndef VIDEO_DEMO_WIDTH
#define VIDEO_DEMO_WIDTH		640
#endif

#ifndef VIDEO_DEMO_HEIGHT
#define VIDEO_DEMO_HEIGHT		480
#endif

#ifndef VIDEO_DEMO_SPI_RATE
#define VIDEO_DEMO_SPI_RATE		8000000
#endif

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

// Frame data read per SPI burst
#define VIDEO_DEMO_BUFSIZE		1024

// Frames between two status lines
#define VIDEO_DEMO_REPORT		10

static const uint8_t s_FrameTag[4] = { 'V', 'F', 'R', 'M' };

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(VIDEO_DEMO_BUFSIZE)];

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCallback = NULL,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
	.bIrDAMode = false,
	.bIrDAInvert = false,
	.bIrDAFixPulse = false,
	.IrDAPulseDiv = 0,
	.Duplex = UART_DUPLEX_FULL,
	.Mode = UART_MODE_UART,
};

static UART s_Uart;

static const IOPinCfg_t s_SpiPins[] = SPI_PINS;

// SPI mode 0, 0 shifted out while reading as the vendor library does
static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = VIDEO_DEMO_SPI_RATE,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_LOW,
	.DummyByte = 0,
	.EvtCB = NULL,
};

static SPI s_Spi;

static const VideoDevCfg_t s_VideoCfg = {
	.DevAddr = 0,
	.OpMode = VIDEO_OPMODE_CONTINUOUS,
	.OutFmt = { VIDEO_FMT_JPEG, VIDEO_DEMO_WIDTH, VIDEO_DEMO_HEIGHT, 0 },
	.InFmt = { VIDEO_FMT_NONE, 0, 0, 0 },
	.EvtHandler = NULL,
};

#if defined(ARDUCAM_MEGA)
static CamArducamMega s_Video;
static const char s_VideoName[] = "ArduCAM Mega";
#else
#error "board.h selects no video device"
#endif

static uint8_t s_Buff[VIDEO_DEMO_BUFSIZE];

// The UART takes what fits in its FIFO, send until all is taken
static void UartSend(const uint8_t *pData, uint32_t Len)
{
	while (Len > 0)
	{
		int n = s_Uart.Tx((uint8_t*)pData, Len);

		if (n > 0)
		{
			pData += n;
			Len -= n;
		}
	}
}

int main()
{
	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1);
	}

#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif

	printf("\r\nVideo capture, %s, SPI %d\r\n", s_VideoName, SPI_DEVNO);

	if (s_Spi.Init(s_SpiCfg) == false)
	{
		printf("SPI init failed\r\n");

		return 1;
	}

	if (s_Video.Init(s_VideoCfg, &s_Spi) == false)
	{
		printf("%s: not found\r\n", s_VideoName);

		return 1;
	}

	uint64_t id = s_Video.DeviceID();
	const VideoFmt_t &fmt = s_Video.Format(VIDEO_STREAM_OUT);

	printf("%s: sensor 0x%02X, controller version 0x%02X, date 20%02u-%02u-%02u\r\n", s_VideoName,
		   (unsigned)(id & 0xFF), (unsigned)((id >> 8) & 0xFF), (unsigned)((id >> 16) & 0xFF),
		   (unsigned)((id >> 24) & 0xFF), (unsigned)((id >> 32) & 0xFF));
	printf("JPEG %ux%u, continuous\r\n", fmt.Width, fmt.Height);

	if (s_Video.Start() == false)
	{
		printf("Start failed\r\n");

		return 1;
	}

	uint32_t count = 0;
	uint32_t total = 0;

	while (1)
	{
		if (s_Video.UpdateData())
		{
			uint32_t len = s_Video.FrameLength();
			uint8_t hdr[8];

			memcpy(hdr, s_FrameTag, sizeof(s_FrameTag));
			hdr[4] = len & 0xFF;
			hdr[5] = (len >> 8) & 0xFF;
			hdr[6] = (len >> 16) & 0xFF;
			hdr[7] = (len >> 24) & 0xFF;
			UartSend(hdr, sizeof(hdr));

			int n;
			uint32_t sent = 0;

			while ((n = s_Video.ReadFrame(s_Buff, sizeof(s_Buff))) > 0)
			{
				UartSend(s_Buff, n);
				sent += n;
			}

			count++;
			total += sent;

			if (sent != len)
			{
				printf("\r\nFrame %lu: %lu of %lu bytes read\r\n", (unsigned long)count, (unsigned long)sent,
					   (unsigned long)len);
			}
			else if ((count % VIDEO_DEMO_REPORT) == 0)
			{
				printf("\r\n%lu frames, %lu bytes\r\n", (unsigned long)count, (unsigned long)total);
			}
		}
	}

	return 0;
}

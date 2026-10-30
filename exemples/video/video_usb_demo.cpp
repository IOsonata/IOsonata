/**-------------------------------------------------------------------------
@example	video_usb_demo.cpp

@brief	Video over USB CDC

Streams the JPEG frames of the video device that board.h selects on a USB
CDC port. Python/video_view.py --port shows them on the PC. board.h selects
the device by its name:

	ARDUCAM_MEGA	ArduCAM Mega SPI camera module (2MP, 3MP or 5MP)

and gives the SPI bus with SPI_DEVNO and SPI_PINS (SCK, MISO, MOSI, CS).
board.h can change VIDEO_DEMO_WIDTH, VIDEO_DEMO_HEIGHT (640x480 by default)
and VIDEO_DEMO_SPI_RATE (8 MHz by default). The SPI rate in effect is
lowered when it is above the maximum of the device.

Capture runs while the port is open (DTR set). A status line is sent first,
then each frame : the 4 characters VFRM, the frame length in bytes 32 bits
little endian, then the frame data.

The frame is read from the device in parts while the CDC Tx FIFO drains, from
AppCheckStatus. The USB work runs from the application event queue through
AppRun.

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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cfifo.h"
#include "app_evt_handler.h"
#include "usb/usb.h"
#include "usb/usbd_cdc.h"
#include "coredev/iopincfg.h"
#include "coredev/spi.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "video/cam_arducam_mega.h"
#include "board.h"

#ifndef VIDEO_DEMO_WIDTH
#define VIDEO_DEMO_WIDTH		640
#endif

#ifndef VIDEO_DEMO_HEIGHT
#define VIDEO_DEMO_HEIGHT		480
#endif

#ifndef VIDEO_DEMO_SPI_RATE
#define VIDEO_DEMO_SPI_RATE		8000000
#endif

// board.h can select another controller
#ifndef USB_DEVNO
#define USB_DEVNO				0
#endif

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

// Frame data read per SPI burst
#define VIDEO_DEMO_BUFSIZE		1024

#if defined(ARDUCAM_MEGA)
#define VIDEO_DEMO_SPI_RATE_MAX	ARDUCAM_MEGA_SPI_RATE_MAX
static CamArducamMega s_Video;
static const char s_VideoName[] = "ArduCAM Mega";
#else
#error "board.h selects no video device"
#endif

#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

static const IOPinCfg_t s_SpiPins[] = SPI_PINS;

// SPI mode 0, 0 shifted out while reading as the vendor library does
static int VideoSpiEvent(DevIntrf_t *, DEVINTRF_EVT Evt, uint8_t *, int Len)
{
	s_Video.IntrfEvent(Evt, Len);
	return 0;
}

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
	.EvtCB = VideoSpiEvent,
};

static SPI s_Spi;

static const VideoDevCfg_t s_VideoCfg = {
	.DevAddr = 0,
	.OpMode = VIDEO_OPMODE_CONTINUOUS,
	.OutFmt = { VIDEO_FMT_JPEG, VIDEO_DEMO_WIDTH, VIDEO_DEMO_HEIGHT, 0 },
	.InFmt = { VIDEO_FMT_NONE, 0, 0, 0 },
	.EvtHandler = NULL,
};

// CDC FIFO memory belongs to the application. The Tx side holds a couple of
// frame parts so the next SPI burst overlaps the USB transfer.
#define CDC_RXFIFO_PKTCNT		4
#define CDC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK))
#define CDC_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(4096)

alignas(4) static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE];

static const UsbdCdcCfg_t s_CdcCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = CDC_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_CdcTxFifoMem,
	.EvtCB = NULL,
};

// Application event queue memory, replaces the 4 event library default. The
// USB controller port queues its deferred endpoint events there.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

// 0x1209 is the pid.codes vendor id, which exists for open hardware. Put your
// own vendor and product id here before shipping anything.
static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0002,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata Video",
	.pSerial = NULL,
	.pFuncName = "IOsonata CDC",
	.IntPrio = 6,
#ifdef USB_PINS
	.pIOPinMap = s_UsbPins,
	.NbIOPins = sizeof(s_UsbPins) / sizeof(IOPinCfg_t),
#else
	.pIOPinMap = NULL,
	.NbIOPins = 0,
#endif
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2U,
	.DeviceProtocol = 1U,
	.bSelfPowered = false,
#ifdef USB_REMOTE_WAKEUP
	.bRemoteWakeup = USB_REMOTE_WAKEUP,
#else
	.bRemoteWakeup = true,
#endif
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = NULL,
};

UsbdCdc g_Cdc;

static const uint8_t s_FrameTag[4] = { 'V', 'F', 'R', 'M' };

static bool s_bVideoOk = false;		// Device found
static bool s_bSession = false;		// Port open, status line sent
static bool s_bRun = false;			// Capture running
static bool s_bFrame = false;		// A frame is being sent
static uint8_t s_Buff[VIDEO_DEMO_BUFSIZE];
static int s_Pending = 0;			// Bytes of s_Buff not taken by the CDC yet
static int s_Offset = 0;

// Next part of the stream into s_Buff : frame header, frame data
static bool VideoFill()
{
	if (s_bFrame)
	{
		int n = s_Video.ReadFrame(s_Buff, sizeof(s_Buff));

		if (n > 0)
		{
			s_Pending = n;
			s_Offset = 0;

			return true;
		}
		s_bFrame = false;
	}

	if (s_Video.UpdateData() == false)
	{
		return false;
	}

	uint32_t len = s_Video.FrameLength();

	memcpy(s_Buff, s_FrameTag, sizeof(s_FrameTag));
	s_Buff[4] = len & 0xFF;
	s_Buff[5] = (len >> 8) & 0xFF;
	s_Buff[6] = (len >> 16) & 0xFF;
	s_Buff[7] = (len >> 24) & 0xFF;
	s_Pending = 8;
	s_Offset = 0;
	s_bFrame = true;

	return true;
}

// Port just opened : status line first, then start the capture
static void VideoOpen()
{
	if (s_bVideoOk)
	{
		const VideoFmt_t &fmt = s_Video.Format(VIDEO_STREAM_OUT);

		s_Pending = snprintf((char*)s_Buff, sizeof(s_Buff),
							 "Video USB, %s sensor 0x%02X, JPEG %ux%u, SPI %lu Hz\r\n",
							 s_VideoName, (unsigned)(s_Video.DeviceID() & 0xFF),
							 fmt.Width, fmt.Height, (unsigned long)s_Spi.Rate());
		s_bRun = s_Video.Start();
	}
	else
	{
		s_Pending = snprintf((char*)s_Buff, sizeof(s_Buff), "Video USB, %s not found\r\n", s_VideoName);
	}
	s_Offset = 0;
	s_bFrame = false;
	s_bSession = true;
}

bool AppCheckStatus(void)
{
	UsbCheckStatus();

	if (g_Cdc.IsPortOpen() == false)
	{
		if (s_bRun)
		{
			s_Video.Stop();
			s_bRun = false;
		}
		s_bSession = false;
		s_Pending = 0;
		s_bFrame = false;

		return !AppEvtHandlerPending();
	}

	if (s_bSession == false)
	{
		VideoOpen();
	}

	while (true)
	{
		if (s_Pending == 0 && (s_bRun == false || VideoFill() == false))
		{
			break;
		}

		int n = g_Cdc.Tx(0, &s_Buff[s_Offset], s_Pending);

		if (n <= 0)
		{
			// Tx FIFO full, the next loop continues
			break;
		}
		s_Offset += n;
		s_Pending -= n;
	}

	// Stay active while the port is open
	return s_bRun == false && s_Pending == 0 && !AppEvtHandlerPending();
}

int main()
{
	if (s_Spi.Init(s_SpiCfg) == true)
	{
		// The rate in effect is the closest the SPI gives, keep it within the device maximum
		while (s_Spi.Rate() > VIDEO_DEMO_SPI_RATE_MAX)
		{
			uint32_t r = s_Spi.Rate();

			if (s_Spi.Rate(r / 2) >= r)
			{
				break;
			}
		}

		s_bVideoOk = s_Video.Init(s_VideoCfg, &s_Spi);
	}

	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg) || !g_Cdc.Init(s_CdcCfg))
	{
		return -1;
	}
	(void)UsbEnable(USB_DEVNO);
	AppRun();

	return 0;
}

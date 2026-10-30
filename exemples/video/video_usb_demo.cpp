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
typedef struct {
	uint8_t Data[VIDEO_DEMO_BUFSIZE];
	int Len;
	int Offset;
} VideoBuffer_t;

static VideoBuffer_t s_Buff[2];
static int s_TxIdx = 0;
static int s_FillIdx = 0;
static int s_ReadIdx = -1;
static int s_ReadHeader = 0;


// Only foreground code changes buffer ownership. The SPI callback records
// completion in the camera; BtIntrf/CDC copies each accepted transmit portion.
static void VideoFill()
{
	VideoBuffer_t *buff;
	int header = 0;
	int n;
	uint32_t len;

	if (s_ReadIdx >= 0)
	{
		n = s_Video.ReadFrameComplete();
		if (n < 0)
		{
			return;
		}
		buff = &s_Buff[s_ReadIdx];
		s_ReadIdx = -1;
		if (n == 0)
		{
			s_Video.Stop();
			s_bRun = false;
			s_bFrame = false;
			return;
		}
		buff->Len = s_ReadHeader + n;
		s_ReadHeader = 0;
		buff->Offset = 0;
		s_FillIdx ^= 1;
		s_bFrame = s_Video.FrameLength() != 0;
	}
	buff = &s_Buff[s_FillIdx];
	if (!s_bRun || buff->Len != 0)
	{
		return;
	}
	if (!s_bFrame)
	{
		if (!s_Video.UpdateData())
		{
			return;
		}
		len = s_Video.FrameLength();
		memcpy(buff->Data, s_FrameTag, sizeof(s_FrameTag));
		buff->Data[4] = len & 0xFF;
		buff->Data[5] = (len >> 8) & 0xFF;
		buff->Data[6] = (len >> 16) & 0xFF;
		buff->Data[7] = (len >> 24) & 0xFF;
		s_ReadHeader = 8;
		s_bFrame = true;
	}
	header = s_ReadHeader;
	n = s_Video.ReadFrameAsync(buff->Data + header, sizeof(buff->Data) - header);
	if (n < 0)
	{
		s_ReadIdx = s_FillIdx;
		s_ReadHeader = header;
	}
	else if (n > 0)
	{
		buff->Len = header + n;
		s_ReadHeader = 0;
		buff->Offset = 0;
		s_FillIdx ^= 1;
		s_bFrame = s_Video.FrameLength() != 0;
	}
}

// Port just opened : status line first, then start the capture
static void VideoOpen()
{
	VideoBuffer_t *buff = &s_Buff[0];

	for (int i = 0; i < 2; i++)
	{
		s_Buff[i].Len = 0;
		s_Buff[i].Offset = 0;
	}
	s_TxIdx = 0;
	s_FillIdx = 1;
	s_ReadHeader = 0;
	if (s_bVideoOk)
	{
		const VideoFmt_t &fmt = s_Video.Format(VIDEO_STREAM_OUT);

		buff->Len = snprintf((char*)buff->Data, sizeof(buff->Data),
							 "Video USB, %s sensor 0x%02X, JPEG %ux%u, SPI %lu Hz\r\n",
							 s_VideoName, (unsigned)(s_Video.DeviceID() & 0xFF),
							 fmt.Width, fmt.Height, (unsigned long)s_Spi.Rate());
		s_bRun = s_Video.Start();
	}
	else
	{
		buff->Len = snprintf((char*)buff->Data, sizeof(buff->Data), "Video USB, %s not found\r\n", s_VideoName);
	}
	buff->Offset = 0;
	s_bFrame = false;
	s_bSession = true;
}

bool AppCheckStatus(void)
{
	bool blocked = false;
	VideoBuffer_t *buff;
	int n;

	UsbCheckStatus();

	if (g_Cdc.IsPortOpen() == false)
	{
		s_Video.Stop();
		s_bRun = false;
		s_bSession = false;
		s_bFrame = false;
		// DMA still owns its buffer after disconnect. Collect its completion
		// before clearing storage or allowing a new streaming session.
		if (s_ReadIdx >= 0 && s_Video.ReadFrameComplete() < 0)
		{
			return !AppEvtHandlerPending();
		}
		s_ReadIdx = -1;
		for (int i = 0; i < 2; i++)
		{
			s_Buff[i].Len = 0;
			s_Buff[i].Offset = 0;
		}
		return !AppEvtHandlerPending();
	}

	if (!s_bSession)
	{
		if (s_ReadIdx >= 0 && s_Video.ReadFrameComplete() < 0)
		{
			return !AppEvtHandlerPending();
		}
		s_ReadIdx = -1;
		VideoOpen();
	}

	while (true)
	{
		// Start the next SPI burst before submitting the preceding buffer.
		VideoFill();
		buff = &s_Buff[s_TxIdx];
		if (buff->Len == 0)
		{
			break;
		}
		n = g_Cdc.Tx(0, buff->Data + buff->Offset, buff->Len);
		if (n <= 0)
		{
			blocked = true;
			break;
		}
		buff->Offset += n;
		buff->Len -= n;
		if (buff->Len == 0)
		{
			buff->Offset = 0;
			s_TxIdx ^= 1;
		}
	}

	// SPI and transport interrupts wake the loop. Camera capture readiness
	// still needs polling whenever no SPI read or transmit is blocking us.
	return (blocked || s_ReadIdx >= 0 || !s_bRun) && !AppEvtHandlerPending();
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

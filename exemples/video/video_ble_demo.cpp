/**-------------------------------------------------------------------------
@example	video_ble_demo.cpp

@brief	Video over BLE

Streams the JPEG frames of the video device that board.h selects over the
BlueIO UART service (BtIntrf). Python/video_view.py --ble shows them on the
PC. board.h selects the device by its name:

	ARDUCAM_MEGA	ArduCAM Mega SPI camera module (2MP, 3MP or 5MP)

and gives the SPI bus with SPI_DEVNO and SPI_PINS (SCK, MISO, MOSI, CS).
board.h can change VIDEO_DEMO_WIDTH, VIDEO_DEMO_HEIGHT (320x240 by default)
and VIDEO_DEMO_SPI_RATE (8 MHz by default). The SPI rate in effect is
lowered when it is above the maximum of the device.

BLE moves about 150 KB/s at best with the 2M PHY. When the PC enables the
notification, the demo asks for the 2M PHY with BtGapSetPhy, the PC accepts
it or not. 320x240 JPEG gives about 10 to 15 frames per second, 640x480
about 3.

Capture runs while the PC is connected with the notification enabled. A status
line is sent first, then each frame : the 4 characters VFRM, the frame length
in bytes 32 bits little endian, then the frame data.

The frame is read from the device in parts while the BtIntrf Tx FIFO drains,
from AppCheckStatus.

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

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_intrf.h"
#include "bluetooth/blueio_blesrvc.h"
#include "app_evt_handler.h"
#include "coredev/iopincfg.h"
#include "coredev/spi.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "video/cam_arducam_mega.h"
#include "board.h"

#ifndef BLUEIO_CONNECT_LED_PORT
#include "blueio_board.h"
#endif

#ifndef VIDEO_DEMO_WIDTH
#define VIDEO_DEMO_WIDTH		320
#endif

#ifndef VIDEO_DEMO_HEIGHT
#define VIDEO_DEMO_HEIGHT		240
#endif

#ifndef VIDEO_DEMO_SPI_RATE
#define VIDEO_DEMO_SPI_RATE		8000000
#endif

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#if defined(ARDUCAM_MEGA)
#define VIDEO_DEMO_SPI_RATE_MAX	ARDUCAM_MEGA_SPI_RATE_MAX
static CamArducamMega s_Video;
static const char s_VideoName[] = "ArduCAM Mega";
#else
#error "board.h selects no video device"
#endif

#define DEVICE_NAME						"BleVideo"			//!< Advertised name, video_view.py looks for it

#define PACKET_SIZE						244					//!< Notification payload with the 247 MTU

#define MANUFACTURER_NAME				"I-SYST inc."
#define MODEL_NAME						"IOsonata Video"

#define APP_ADV_INTERVAL				64					//!< msec
#define APP_ADV_TIMEOUT					0					//!< No timeout

#define MIN_CONN_INTERVAL				7.5					//!< msec
#define MAX_CONN_INTERVAL				40					//!< msec

#define BLESRV_READ_CHAR_IDX			0
#define BLESRV_WRITE_CHAR_IDX			1

// Whole notification payloads per SPI burst; avoid a short packet each burst.
#define VIDEO_DEMO_BUFSIZE				(PACKET_SIZE * 4)

void ReadCharSetNotif(BtGattChar_t *pChar, bool bEnable, uint16_t ConnHdl);

static const char s_RxCharDescString[] = "Video data characteristic";
static const char s_TxCharDescString[] = "Command characteristic";

uint8_t g_ManData[8];

static BtGattChar_t s_VideoChars[] = {
	// Read + Notify, the frames
	BT_CHAR(BLUEIO_UUID_UART_RX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
			s_RxCharDescString,
			.SetNotifCB = ReadCharSetNotif),
	// Write + Write Without Response, not used by this demo
	BT_CHAR(BLUEIO_UUID_UART_TX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_WRITE_WORESP,
			s_TxCharDescString),
};

uint8_t g_LWrBuffer[512];

// GAP, GATT, the video characteristics and DIS exceed the default 2 KB pool.
alignas(BtAttDBEntry_t) static uint8_t s_AttDBMem[3072];

const BtAttDBMemCfg_t g_BtAttDBMemCfg = {
	s_AttDBMem, sizeof(s_AttDBMem)
};

static bool s_bServiceOk = false;

BtGattSrvc_t g_VideoBleSrvc = BT_SRVC_CUSTOM(BLUEIO_UUID_BASE,
											 BLUEIO_UUID_UART_SERVICE,
											 s_VideoChars);

static const BtAppDevInfo_t s_VideoBleDevDesc {
	MODEL_NAME,				// Model name
	MANUFACTURER_NAME,		// Manufacturer name
	"",						// Serial number string
	"0.0",					// Firmware version string
	"0.0",					// Hardware version string
};

static const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.PeriphDevMax = 0,
	.CentralDevMax = 1,
	.pDevName = DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.ProductId = 1,
	.ProductVer = 0,
	.pDevInfo = &s_VideoBleDevDesc,
	.pAdvManData = g_ManData,
	.AdvManDataLen = sizeof(g_ManData),
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BTGAP_SECTYPE_NONE,
	.SecExchg = BTAPP_SECEXCHG_NONE,
	.pAdvUuid = NULL,
	.AdvInterval = APP_ADV_INTERVAL,
	.AdvTimeout = APP_ADV_TIMEOUT,
	.AdvSlowInterval = 0,
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT,
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,
	.TxPower = 0,
	.MaxMtu = 247,
	.pLongWrPoolMem = g_LWrBuffer,
	.LongWrPoolMemSize = sizeof(g_LWrBuffer),
};

#define BLEINTRF_RXFIFOSIZE				BTINTRF_CFIFO_TOTAL_MEMSIZE(2, PACKET_SIZE)
#define BLEINTRF_TXFIFOSIZE				BTINTRF_CFIFO_TOTAL_MEMSIZE(16, PACKET_SIZE)

alignas(4) static uint8_t s_BleIntrfRxFifo[BLEINTRF_RXFIFOSIZE];
alignas(4) static uint8_t s_BleIntrfTxFifo[BLEINTRF_TXFIFOSIZE];

static const BtIntrfCfg_t s_BleIntrfCfg = {
	.pSrvc = &g_VideoBleSrvc,
	.RxCharIdx = BLESRV_WRITE_CHAR_IDX,
	.TxCharIdx = BLESRV_READ_CHAR_IDX,
	.PacketSize = PACKET_SIZE,
	.bBlocking = true,
	.RxFifoMemSize = BLEINTRF_RXFIFOSIZE,
	.pRxFifoMem = s_BleIntrfRxFifo,
	.TxFifoMemSize = BLEINTRF_TXFIFOSIZE,
	.pTxFifoMem = s_BleIntrfTxFifo,
	.EvtCB = NULL,
};

BtIntrf g_BtIntrf;

static const IOPinCfg_t s_SpiPins[] = SPI_PINS;

// SPI mode 0, interrupt-driven DMA, 0 shifted out as the vendor library does
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
	.bDmaEn = true,
	.bIntEn = true,
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

static const uint8_t s_FrameTag[4] = { 'V', 'F', 'R', 'M' };

static volatile bool s_bNotif = false;	// Notification enabled by the PC
static volatile uint16_t s_ConnHdl = BT_CONN_HDL_INVALID;	// Connection of the notification
static bool s_bVideoOk = false;			// Device found
static bool s_bSession = false;			// Streaming session, status line sent
static bool s_bRun = false;				// Capture running
static bool s_bFrame = false;			// A frame is being sent
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


void ReadCharSetNotif(BtGattChar_t *pChar, bool bEnable, uint16_t ConnHdl)
{
	(void)pChar;

	s_ConnHdl = ConnHdl;
	s_bNotif = bEnable;
}

void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
}

void BtAppInitUserServices()
{
	s_bServiceOk = BtGattSrvcAdd(&g_VideoBleSrvc);
}

void BtAppInitUserData()
{
}

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

// Notification just enabled : 2M PHY request, status line first, then start
// the capture
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
	bool phy = BtGapSetPhy(s_ConnHdl, BT_GAP_PHY_2MBITS, BT_GAP_PHY_2MBITS, BT_GAP_PHY_OPT_NONE);

	if (s_bVideoOk)
	{
		const VideoFmt_t &fmt = s_Video.Format(VIDEO_STREAM_OUT);

		buff->Len = snprintf((char*)buff->Data, sizeof(buff->Data),
							 "Video BLE, %s sensor 0x%02X, JPEG %ux%u, SPI %lu Hz, 2M PHY %s\r\n",
							 s_VideoName, (unsigned)(s_Video.DeviceID() & 0xFF),
							 fmt.Width, fmt.Height, (unsigned long)s_Spi.Rate(),
							 phy ? "requested" : "request failed");
		s_bRun = s_Video.Start();
	}
	else
	{
		buff->Len = snprintf((char*)buff->Data, sizeof(buff->Data), "Video BLE, %s not found\r\n", s_VideoName);
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

	BtAppCheckStatus();

	if (isConnected() == false || s_bNotif == false)
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
		if (!g_BtIntrf.RequestToSend(1))
		{
			blocked = true;
			break;
		}
		n = g_BtIntrf.Tx(0, buff->Data + buff->Offset, buff->Len);
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

	if (BtAppInit(&s_BleAppCfg) == false || s_bServiceOk == false ||
		g_BtIntrf.Init(s_BleIntrfCfg) == false)
	{
		BtAdvStop();

		return -1;
	}

	AppRun();

	return 0;
}

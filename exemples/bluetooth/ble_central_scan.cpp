/**-------------------------------------------------------------------------
@example	ble_central_scan.cpp

@brief	BLE Central Scan demo

This application demo shows Central mode scanning for peripheral devices

@author	Hoang Nguyen Hoan
@date	Feb. 4, 2017

@license

MIT License

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#include "app_evt_handler.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/blueio_blesrvc.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"

#include "board.h"

#ifdef BLE_SCAN_DISPLAY
#include "coredev/spi.h"
#include "coredev/interrupt.h"
#include "display/display_ssd1315.h"
#include "display/display_hx8357.h"
#include "display/display_ili9341.h"
#include "display/display_st77xx.h"
#endif

#ifndef UART_DEVNO
#define UART_DEVNO 0
#endif

#ifndef BLUEIO_CONNECT_LED_PORT
#include "blueio_board.h"
#endif

//#define DUMP_MAN_SPEC_DATA

// BLE
#define DEVICE_NAME "UARTCentral"

#define MANUFACTURER_NAME "I-SYST inc."
#define MODEL_NAME "IMM-NRF5x"
#define MANUFACTURER_ID ISYST_BLUETOOTH_ID
#define ORG_UNIQUE_ID ISYST_BLUETOOTH_ID

#define MIN_CONN_INTERVAL 10 // in msec
#define MAX_CONN_INTERVAL 40 // in msec

#define SCAN_INTERVAL 1000 // in msec
#define SCAN_WINDOW 100	   // in msec
#define SCAN_TIMEOUT 0	   // 0 = never

#define TARGET_BRIDGE_DEV_NAME "BlueIO832Mini"

// UART
#define BLE_MTU_SIZE 512 // in bytes
#define PACKET_SIZE 256
#define UART_MAX_DATA_LEN (PACKET_SIZE * 4)
#define UARTFIFOSIZE CFIFO_MEMSIZE(UART_MAX_DATA_LEN)

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx);

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_CENTRAL,
	.PeriphDevMax = 1,				// Max peripheral devices we connect to as central
	.CentralDevMax = 0,				// Max central devices we serve as peripheral
	.pDevName = DEVICE_NAME,		// Device name
	.VendorId = ISYST_BLUETOOTH_ID, // PnP Bluetooth/USB vendor id
	.ProductId = 1,					// PnP Product ID
	.ProductVer = 0,				// Pnp prod version
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT, // Led port nuber
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,	// Led pin number
	.ConnLedActLevel = 0,
	.TxPower = 0, // Tx power
	.MaxMtu = BLE_MTU_SIZE,
};

static IOPinCfg_t s_UartPins[] = {
	{ UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },	   // RX
	{ UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },	   // TX
	{ UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },  // CTS
	{ UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, // RTS
};

/// UART operation mode config
alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];

UARTCfg_t g_UartCfg = {
	.DevNo = UART_DEVNO,								 // Device number zero based
	.pIOPinMap = s_UartPins,							 // UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t), // Total number of UART pins used
	.Rate = 1000000,									 // Baudrate
	.DataBits = 8,										 // Data bits
	.Parity = UART_PARITY_NONE,							 // Parity
	.StopBits = 1,										 // Stop bit
	.FlowControl = UART_FLWCTRL_NONE,					 // Flow control
	.bIntMode = true,									 // Interrupt mode
	.IntPrio = 6,										 //APP_IRQ_PRIORITY_LOW,			// Interrupt priority
	.EvtCallback = UartEvthandler,						 // UART event handler
	.bFifoBlocking = true,								 // Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
};

// UART object instance
UART g_Uart;

int g_DelayCnt = 0;

static BtGapScanCfg_t s_ScanParams = {
	.Type = BTSCAN_TYPE_ACTIVE,
	.Param = {
		.Interval = SCAN_INTERVAL,
		.Duration = SCAN_WINDOW,
		.Timeout = SCAN_TIMEOUT,
	},
	.BaseUid = BLUEIO_UUID_BASE,
	.ServUid = BLUEIO_UUID_UART_SERVICE, //s_UartBleSrvAdvUuid,
};

uint8_t g_ScanBuff[BT_GAP_SCAN_BUFFER_SIZE_DEFAULT]; //BLE_GAP_SCAN_BUFFER_EXTENDED_MAX];

#ifdef BLE_SCAN_DISPLAY
// The display is optional; existing UART-only projects keep their setup.
// Four recent devices, fixed storage. The timer queues work; only the
// application event handler sends display data.
typedef struct __Scan_Device
{
	bool Used;
	uint8_t AddrType, Addr[6];
	char Name[15];
	int8_t Rssi;
	uint32_t Seen;
} ScanDevice_t;

alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
static const IOPinCfg_t s_DisplayPins[] = TFT_PINS;
static const IOPinCfg_t s_DisplaySpiPins[] = SPI_PINS;
static const SPICfg_t s_DisplaySpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_DisplaySpiPins,
	.NbIOPins = sizeof(s_DisplaySpiPins) / sizeof(s_DisplaySpiPins[0]),
	.Rate = SPI_RATE,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = DISPLAY_SPI_CLKPOL,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = DISPLAY_SPI_DMA_ENABLE,
	.bIntEn = false,
	.IntPrio = 6,
	.EvtCB = nullptr
};
static DisplayCfg_t s_DisplayCfg = {
	.DevAddr = 0,
	.pPins = s_DisplayPins,
	.NbPins = sizeof(s_DisplayPins) / sizeof(s_DisplayPins[0]),
	.Stride = DISPLAY_STRIDE,
	.Width = DISPLAY_WIDTH,
	.Height = DISPLAY_HEIGHT,
	.PixelSize = DISPLAY_PIXEL_SIZE,
	.Orient = DISPLAY_ORIENTATION
};
static const TimerCfg_t s_DisplayTimerCfg = {
	.DevNo = TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000,
	.IntPrio = 7,
	.EvtHandler = nullptr,
	.bTickInt = false
};

static ScanDevice_t s_Devices[4];
static SPI s_DisplaySpi;
static DISPLAY_DRIVER s_Display;
static Timer s_DisplayTimer;
static bool s_DisplayReady;
static volatile bool s_DisplayQueued;

static unsigned SignalWidth(int Rssi)
{
	if (Rssi == 127 || Rssi <= -100)
	{
		return 0;
	}
	if (Rssi >= -20)
	{
		return 128;
	}
	return (Rssi + 100) * 128 / 80;
}

static void ScanDisplayReport(int8_t Rssi, uint8_t AddrType, const uint8_t *Addr,
							  size_t AdvLen, uint8_t *pAdvData)
{
	if (!s_DisplayReady || !Addr)
	{
		return;
	}
	char name[15] = {};
	if (AdvLen && pAdvData)
	{
		BtAdvDataGetDevName(pAdvData, AdvLen, name, sizeof(name));
		for (char *p = name; *p; p++)
		{
			if ((uint8_t)*p < 32 || (uint8_t)*p > 126)
			{
				*p = '?';
			}
		}
	}
	uint32_t now = s_DisplayTimer.mSecond();
	uint32_t state = DisableInterrupt();
	int slot = -1;
	for (int i = 0; i < 4; i++)
	{
		if (s_Devices[i].Used && s_Devices[i].AddrType == AddrType &&
			memcmp(s_Devices[i].Addr, Addr, 6) == 0)
		{
			slot = i;
			break;
		}
	}
	if (slot < 0)
	{
		slot = 0;
		for (int i = 0; i < 4; i++)
		{
			if (!s_Devices[i].Used)
			{
				slot = i;
				break;
			}
			if (now - s_Devices[i].Seen > now - s_Devices[slot].Seen)
			{
				slot = i;
			}
		}
		s_Devices[slot] = {};
	}
	ScanDevice_t &d = s_Devices[slot];
	d.Used = true;
	d.AddrType = AddrType;
	memcpy(d.Addr, Addr, 6);
	d.Rssi = Rssi;
	d.Seen = now;
	// A scan response with no name must not erase a previously received name.
	if (name[0])
	{
		memcpy(d.Name, name, sizeof(name));
	}
	EnableInterrupt(state);
}

static void ScanDisplayRefresh(uint32_t, void *)
{
	s_DisplayQueued = false;
	if (!s_DisplayReady)
	{
		return;
	}
	ScanDevice_t devices[4];
	uint32_t now = s_DisplayTimer.mSecond();
	uint32_t state = DisableInterrupt();
	for (auto &d : s_Devices)
	{
		if (d.Used && now - d.Seen >= 10000)
		{
			d.Used = false;
		}
	}
	memcpy(devices, s_Devices, sizeof(devices));
	EnableInterrupt(state);
	s_Display.Clear();
	s_Display.SetCursor(0, 0);
	s_Display.printf("BLE scan / RSSI dBm");
	unsigned row = 0;
	for (const auto &d : devices)
	{
		if (!d.Used)
		{
			continue;
		}
		char addr[15];
		snprintf(addr, sizeof(addr), "%02X%02X%02X%02X%02X%02X",
				 d.Addr[5], d.Addr[4], d.Addr[3], d.Addr[2], d.Addr[1], d.Addr[0]);
		s_Display.SetCursor(0, 12 + row * 12);
		if (d.Rssi == 127)
		{
			s_Display.printf("%-14.14s   --", d.Name[0] ? d.Name : addr);
		}
		else
		{
			s_Display.printf("%-14.14s %4d", d.Name[0] ? d.Name : addr, d.Rssi);
		}
		unsigned width = SignalWidth(d.Rssi);
		if (width)
		{
			s_Display.Fill(0, 20 + row * 12, width, 3, DISPLAY_PIXEL_SIZE == 1 ? 1 : 0xFFFF);
		}
		row++;
	}
	if (!row)
	{
		s_Display.SetCursor(0, 24);
		s_Display.printf("Waiting for devices");
	}
}

static void ScanDisplayTick(TimerDev_t *, int, void *)
{
	if (s_DisplayReady && !s_DisplayQueued)
	{
		s_DisplayQueued = true;
		if (!AppEvtHandlerQue(0, nullptr, ScanDisplayRefresh))
		{
			s_DisplayQueued = false;
		}
	}
}

static bool ScanDisplayInit()
{
	if (!s_DisplaySpi.Init(s_DisplaySpiCfg) || !s_Display.Init(s_DisplayCfg, &s_DisplaySpi) ||
		s_Display.Width() < 128 || s_Display.Height() < 64 || !s_Display.SetFont(&iFontSystem5x7))
	{
		return false;
	}
	s_Display.Backlight(true);
	if (!s_DisplayTimer.Init(s_DisplayTimerCfg))
	{
		s_Display.printf("Timer init failed");
		return false;
	}
	s_DisplayReady = true;
	ScanDisplayRefresh(0, nullptr);
	if (!s_DisplayTimer.EnableTimerTrigger(0, uint32_t(250), TIMER_TRIG_TYPE_CONTINUOUS, ScanDisplayTick, nullptr))
	{
		s_DisplayReady = false;
		s_DisplayTimer.Disable();
		s_Display.Clear();
		s_Display.printf("Timer trigger failed");
		return false;
	}
	return true;
}

static void ScanDisplayError(const char *Message)
{
	if (!s_DisplayReady)
	{
		return;
	}
	s_DisplayReady = false;
	s_DisplayTimer.Disable();
	s_Display.Clear();
	s_Display.printf("%s", Message);
}
#endif

bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *pAdvData)
{
#ifdef BLE_SCAN_DISPLAY
	ScanDisplayReport(Rssi, AddrType, Addr, AdvLen, pAdvData);
#endif
	if (AdvLen > 0)
	{
		char name[32];
		size_t l = BtAdvDataGetDevName(pAdvData, AdvLen, name, 32);

		if (l > 0)
		{
			// BtAdvDataGetDevName returns characters copied and terminates name.
			g_Uart.printf("%02x %02x %02x %02x %02x %02x : RSSI = %d, ",
						  Addr[0], Addr[1], Addr[2],
						  Addr[3], Addr[4], Addr[5], Rssi);
			g_Uart.printf("%s\r\n", name);
		}
		else
		{
			//			g_Uart.printf("(No Name)\r\n", name);
		}
#ifdef DUMP_MAN_SPEC_DATA
		uint8_t buff[256];
		l = BtAdvDataGetManData(pAdvData, AdvLen, buff, 256);
		if (l > 0)
		{
			g_Uart.printf("Len = %d - ", l);
			for (int i = 0; i < l; i++)
			{
				g_Uart.printf("%02x ", buff[i]);
			}

			g_Uart.printf("\r\n");
		}
#endif
	}
	else
	{
		g_Uart.printf("(No data)\r\n");
	}

	return true;
}

bool HardwareInit()
{
	if (!g_Uart.Init(g_UartCfg))
	{
		return false;
	}

	g_Uart.printf("BLE Central Scan Demo\r\n");
	g_Uart.printf("UART Configuration: %d, Flow Control (%s), Parity (%s)\r\n",
				  g_UartCfg.Rate,
				  (g_UartCfg.FlowControl == UART_FLWCTRL_NONE) ? "No" : "Yes",
				  (g_UartCfg.Parity == UART_PARITY_NONE) ? "No" : "Yes");

#ifdef BLE_SCAN_DISPLAY
	if (!ScanDisplayInit())
	{
		return false;
	}
#endif
	return true;
}

void BtAppInitUserData()
{
}

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	//	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
#ifdef BLE_SCAN_DISPLAY
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)))
	{
		return 1;
	}
#endif
	if (!HardwareInit())
	{
		return 1;
	}
	if (!BtAppInit(&s_BleAppCfg))
	{
		g_Uart.printf("BLE initialization failed\r\n");
#ifdef BLE_SCAN_DISPLAY
		ScanDisplayError("BLE init failed");
#endif
		return 1;
	}
	if (!BtAppScanInit(&s_ScanParams))
	{
		g_Uart.printf("BLE scan initialization failed\r\n");
#ifdef BLE_SCAN_DISPLAY
		ScanDisplayError("Scan init failed");
#endif
		return 1;
	}

	BtAppScan();

	AppRun();

	return 0;
}

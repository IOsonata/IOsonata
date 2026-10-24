/**-------------------------------------------------------------------------
@example	ble_security_demo.cpp

@brief	BLE security association-model demo over UART/BLE

This application demo shows UART Rx/Tx over BLE custom service using EHAL library.
For evaluating power consumption of the UART, the button 1 is used to enable/disable it.
The IOsonata Bluetooth library handles pairing, bonding and key storage.
Only application-specific user interaction is supplied through BtSmp callbacks.

@author	Hoang Nguyen Hoan
@date	Feb. 4, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/blueio_blesrvc.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "syslog.h"

#include "board.h"
#ifdef BLE_STATUS_DISPLAY
#include "coredev/spi.h"
#include "display/display_ssd1315.h"
#include "display/display_hx8357.h"
#include "display/display_ili9341.h"
#include "display/display_st77xx.h"
#endif

// Configure a security policy in the application board.h. The library owns
// SMP, key exchange, bonding, persistence and security events.
// Confirmation uses buttons or UART input; pairing remains in the library.
#ifndef BLE_SECURITY_TYPE
#define BLE_SECURITY_TYPE BTGAP_SECTYPE_STATICKEY_NO_MITM
#endif
#ifndef BLE_SECURITY_EXCHG
#define BLE_SECURITY_EXCHG BTAPP_SECEXCHG_NONE
#endif

#ifndef UART_DEVNO
#define UART_DEVNO 0
#endif

#ifndef BUT2_INT
#define BUT2_INT 1
#endif

#ifndef BUT1_INT
#define BUT1_INT 0
#endif

#ifndef DEVICE_NAME
#define DEVICE_NAME "BLESec"
#endif

#define PACKET_SIZE 20

#define MANUFACTURER_NAME "I-SYST inc."
#define MODEL_NAME "Generic"

#define MANUFACTURER_ID ISYST_BLUETOOTH_ID
#define ORG_UNIQUE_ID ISYST_BLUETOOTH_ID

#define APP_ADV_INTERVAL 64 // in msec

#define APP_ADV_TIMEOUT 0 // in msec

#define MIN_CONN_INTERVAL 10 // in msec
#define MAX_CONN_INTERVAL 40 // in msec

#define BLE_UART_UUID_BASE BLUEIO_UUID_BASE

#define BLE_UART_UUID_SERVICE BLUEIO_UUID_UART_SERVICE //!< BlueIO default service
#define BLE_UART_UUID_TX_CHAR BLUEIO_UUID_UART_TX_CHAR //!< Data characteristic
#define BLE_UART_UUID_RX_CHAR BLUEIO_UUID_UART_RX_CHAR //!< Command control characteristic

void UartTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

static const BtUuidArr_t s_AdvUuid = {
	.BaseIdx = 1,
	.Type = BT_UUID_TYPE_16,
	.Count = 1,
	.Uuid16 = {
		BLE_UART_UUID_SERVICE,
	}
};

static const char s_RxCharDescString[] = {
	"UART Rx characteristic",
};

static const char s_TxCharDescString[] = {
	"UART Tx characteristic",
};

uint8_t g_ManData[8];

/// Characteristic definitions
BtGattChar_t g_UartChars[] = {
	// Read + Notify (server-pushed value, peer can also subscribe)
	BT_CHAR(BLE_UART_UUID_RX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
			s_RxCharDescString),
	// Write Without Response (command sink, callback consumes)
	BT_CHAR(BLE_UART_UUID_TX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_WRITE_WORESP,
			s_TxCharDescString,
			.WrCB = UartTxSrvcCallback),
};

uint8_t g_LWrBuffer[512];

/// Service definition
BtGattSrvc_t g_UartBleSrvc = BT_SRVC_CUSTOM(BLE_UART_UUID_BASE,
											BLE_UART_UUID_SERVICE,
											g_UartChars,
											.SecType = BLE_SECURITY_TYPE);

const BtAppDevInfo_t s_UartBleDevDesc = {
	MODEL_NAME,		   // Model name
	MANUFACTURER_NAME, // Manufacturer name
	"123",			   // Serial number string
	"0.0",			   // Firmware version string
	"0.0",			   // Hardware version string
};

uint8_t g_AdvLong[] = "1234567890abcdefghijklmnopqrstuvwxyz`!@#$%^&*()_+\0";

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.PeriphDevMax = 0,				// Max peripheral devices we connect to as central
	.CentralDevMax = 2,				// Max central devices we serve as peripheral; keep advertising after the first
	.pDevName = DEVICE_NAME,		// Device name
	.VendorId = ISYST_BLUETOOTH_ID, // PnP Bluetooth/USB vendor id
	.ProductId = 1,					// PnP Product ID
	.ProductVer = 0,				// Pnp prod version
	.Appearance = 0,
	.pDevInfo = &s_UartBleDevDesc,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData), // Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLE_SECURITY_TYPE,
	.SecExchg = BLE_SECURITY_EXCHG,
	.bCompleteUuidList = false,
	.pAdvUuid = &s_AdvUuid,			 // Service uuids to advertise
	.AdvInterval = APP_ADV_INTERVAL, // Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,	 // Advertising timeout in sec
	.AdvSlowInterval = 0,			 // Slow advertising interval, if > 0, fallback to
									 // slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT, // Led port nuber
	.ConnLedPin = CONNECT_LED_PIN,	 // Led pin number
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0, // Tx power
	// .SDEvtHandler removed for compatibility with older BtAppCfg_t definitions
	.pLongWrPoolMem = g_LWrBuffer, // Long-write reassembly pool (split across peer slots)
	.LongWrPoolMemSize = sizeof(g_LWrBuffer),
};

// Board IO tables and UART state, grouped ahead of the first function.
static const IOPinCfg_t s_LedPins[] = LED_PINS;

static int s_NbLedPins = sizeof(s_LedPins) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_ButPins[] = BUTTON_PINS;

static int s_NbButPins = sizeof(s_ButPins) / sizeof(IOPinCfg_t);

int g_DelayCnt = 0;
volatile bool g_bUartState = false;
int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define UARTFIFOSIZE CFIFO_MEMSIZE(256)

static uint8_t s_UartRxFifo[UARTFIFOSIZE];
static uint8_t s_UartTxFifo[UARTFIFOSIZE];

/// UART pins definitions
static IOPinCfg_t s_UartPins[] = UART_PINS;

/// UART configuration
const UARTCfg_t g_UartCfg = {
	.DevNo = UART_DEVNO,								 // Device number zero based
	.pIOPinMap = s_UartPins,							 // UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t), // Total number of UART pins used
	.Rate = 1000000,									 // Baudrate
	.DataBits = 8,										 // Data bits
	.Parity = UART_PARITY_NONE,							 // Parity
	.StopBits = 1,										 // Stop bit
	.FlowControl = UART_FLWCTRL_NONE,					 // Flow control
	.bIntMode = true,									 // Interrupt mode
	.IntPrio = IRQ_PRIO_LOW,							 // Interrupt priority
	.EvtCallback = UartEvthandler,						 // UART event handler
	.bFifoBlocking = true,								 // Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
};

/// UART object instance
UART g_Uart;

// SysLog store. With the UART attached at init, each record goes out as it
// is logged. 16 records of 128 bytes, non blocking, so a burst the UART
// cannot keep up with drops the oldest lines rather than the newest.
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = false
};

#ifdef TIMER_DEVNO
static const TimerCfg_t s_TimerCfg = {
	.DevNo = TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
#ifdef BLE_STATUS_DISPLAY
	.Freq = 1000000,
#else
	.Freq = 0,
#endif
	.IntPrio = 6,
};

Timer g_Timer;
#endif

// Pending stays set through queue refusal until the callback runs.
static volatile bool s_bUartRxPending = false;
static volatile bool s_bUartRxTimeout = false;
static volatile bool s_bSysLogFlushPending = false;

void UartRxChedHandler(uint32_t Evt, void *pCtx);
static void SysLogFlushEvt(uint32_t Evt, void *pCtx);

static uint8_t s_UartRxBuff[PACKET_SIZE];
static int s_UartRxBuffLen = 0;

#if defined(BLE_SECURITY_BUTTON_CONFIRM) || defined(BLE_SECURITY_UART_CONFIRM)
// User-interface state only. The IOsonata library retains the pairing
// transaction while awaiting the decision.
static volatile uint8_t s_PairDecision = 0; // 0=pending, 1=accept, 2=reject
static volatile bool s_PairAwaiting = false;
static uint16_t s_PairConn = 0;
#ifdef BLE_STATUS_DISPLAY
static uint32_t s_PairNumber;
#endif

#endif

#ifdef BLE_STATUS_DISPLAY
// Display work stays in AppEvt context. The existing timer also timestamps
// SysLog; Bluetooth keeps its own low-power timers.
#ifndef TIMER_DEVNO
#error BLE_STATUS_DISPLAY requires a free TIMER_DEVNO
#endif
typedef struct __Display_Link
{
	uint16_t Hdl;
	uint8_t Addr[6];
} DisplayLink_t;

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
static SPI s_DisplaySpi;
static DISPLAY_DRIVER s_Display;
static bool s_DisplayReady, s_BleReady;
static volatile bool s_DisplayQueued;

#endif

static void UartRxQue(void)
{
	uint32_t state = DisableInterrupt();

	if (s_bUartRxPending == false)
	{
		s_bUartRxPending = true;
		(void)AppEvtHandlerQue(0, nullptr, UartRxChedHandler);
	}
	EnableInterrupt(state);
}

static void SysLogFlushQue(void)
{
	uint32_t state = DisableInterrupt();

	if (s_bSysLogFlushPending == false)
	{
		s_bSysLogFlushPending = true;
		(void)AppEvtHandlerQue(0, nullptr, SysLogFlushEvt);
	}
	EnableInterrupt(state);
}

void UartTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len)
{
	g_Uart.Tx(pData, Len);
}

void BtAppPeriphEvtHandler(uint32_t Evt, void *const pCtx)
{
	//BtGattEvtHandler(Evt, pCtx);
}

void BtAppInitUserServices()
{
	bool res;
	res = BtGattSrvcAdd(&g_UartBleSrvc);
}

#if defined(BLE_SECURITY_BUTTON_CONFIRM) || defined(BLE_SECURITY_UART_CONFIRM)
void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	if (s_PairAwaiting && s_PairConn == ConnHdl)
	{
		s_PairAwaiting = false;
		s_PairDecision = 0;
	}
}

int BtAppPairConfirm(uint16_t ConnHdl, uint32_t Number)
{
	if (s_PairAwaiting)
	{
		return 0;
	}
	s_PairConn = ConnHdl;
#ifdef BLE_STATUS_DISPLAY
	s_PairNumber = Number;
#endif
	s_PairDecision = 0;
	s_PairAwaiting = true;
#ifdef BLE_SECURITY_UART_CONFIRM
	SysLogPrintf(SysLogGet(), "Compare %06u on both devices: y=accept, n=reject\r\n",
				 (unsigned)Number);
#else
	SysLogPrintf(SysLogGet(), "Compare %06u on both devices: BUT1=accept, BUT2=reject\r\n",
				 (unsigned)Number);
#endif
	return -1; // Defer; no blocking and no protocol reply in this callback.
}
#endif

#ifdef BLE_STATUS_DISPLAY
static const char *SecurityText(bool Known, const BtConnSec_t &Sec)
{
	if (!Known)
	{
		return "unknown";
	}
	switch (Sec.Level)
	{
		case BT_GAP_SEC_LEVEL_NONE:
			return "open L1";
		case BT_GAP_SEC_LEVEL_ENC_UNAUTH:
			return "enc L2";
		case BT_GAP_SEC_LEVEL_ENC_AUTH:
			return "MITM L3";
		case BT_GAP_SEC_LEVEL_LESC_AUTH:
			return "LESC MITM L4";
		default:
			return "unknown";
	}
}

static void StatusDisplayRefresh(uint32_t, void *)
{
	s_DisplayQueued = false;
	if (!s_DisplayReady)
	{
		return;
	}

	DisplayLink_t links[2];

	unsigned count = 0, shown = 0;
	bool awaiting = false;
	uint32_t number = 0;
	uint32_t irq = DisableInterrupt();
	for (uint16_t i = 0; i < BtPeerCount(); i++)
	{
		BtDevice_t *p = BtPeerSlot(i);
		if (!p || p->Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			continue;
		}
		count++;
		if (shown < 2)
		{
			links[shown].Hdl = p->Conn.Hdl;
			memcpy(links[shown++].Addr, p->Conn.PeerAddr, 6);
		}
	}
#if defined(BLE_SECURITY_BUTTON_CONFIRM) || defined(BLE_SECURITY_UART_CONFIRM)
	awaiting = s_PairAwaiting;
	number = s_PairNumber;
#endif
	EnableInterrupt(irq);
	s_Display.Clear();
	s_Display.SetCursor(0, 0);
	s_Display.printf("BLE status: %.9s", DEVICE_NAME);
	s_Display.SetCursor(0, 8);
	s_Display.printf("Links: %u", count);
	if (!shown)
	{
		s_Display.SetCursor(0, 24);
		s_Display.printf("%s", s_BleReady ? "Waiting for central" : "Starting BLE");
	}
	for (unsigned i = 0; i < shown; i++)
	{
		BtConnSec_t sec = {};
		bool known = BtGapConnSecGet(links[i].Hdl, &sec);
		s_Display.SetCursor(0, 16 + i * 16);
		s_Display.printf("H%04X %s", links[i].Hdl, SecurityText(known, sec));
		const uint8_t *a = links[i].Addr;
		s_Display.SetCursor(0, 24 + i * 16);
		s_Display.printf("%02X%02X%02X%02X%02X%02X", a[5], a[4], a[3], a[2], a[1], a[0]);
	}
	s_Display.SetCursor(0, 48);
	if (awaiting)
	{
		s_Display.printf("Compare %06u", (unsigned)number);
		s_Display.SetCursor(0, 56);
#ifdef BLE_SECURITY_UART_CONFIRM
		s_Display.printf("UART y=yes n=no");
#else
		s_Display.printf("Buttons: yes/no");
#endif
	}
	else
	{
		s_Display.printf("UART: 1000000 baud");
	}
}

static void StatusDisplayTick(TimerDev_t *, int, void *)
{
	if (s_DisplayReady && !s_DisplayQueued)
	{
		s_DisplayQueued = true;
		if (!AppEvtHandlerQue(0, nullptr, StatusDisplayRefresh))
		{
			s_DisplayQueued = false;
		}
	}
}

static bool StatusDisplayInit()
{
	if (!s_DisplaySpi.Init(s_DisplaySpiCfg) || !s_Display.Init(s_DisplayCfg, &s_DisplaySpi) ||
		s_Display.Width() < 128 || s_Display.Height() < 64 || !s_Display.SetFont(&iFontSystem5x7))
	{
		return false;
	}
	s_Display.Backlight(true);
	s_DisplayReady = true;
	StatusDisplayRefresh(0, nullptr);
	if (!g_Timer.EnableTimerTrigger(0, uint32_t(250), TIMER_TRIG_TYPE_CONTINUOUS, StatusDisplayTick, nullptr))
	{
		s_DisplayReady = false;
		s_Display.Clear();
		s_Display.printf("Timer trigger failed");
		return false;
	}
	return true;
}

static void StatusDisplayError(const char *Message)
{
	if (!s_DisplayReady)
	{
		return;
	}
	s_DisplayReady = false;
	g_Timer.DisableTimerTrigger(0);
	s_Display.Clear();
	s_Display.printf("%s", Message);
}
#endif

void ButEvent(int IntNo, void *pCtx)
{
#ifdef BLE_SECURITY_BUTTON_CONFIRM
	if (s_PairAwaiting)
	{
		if (IntNo == BUT1_INT)
		{
			s_PairDecision = 1;
		}
		else if (IntNo == BUT2_INT)
		{
			s_PairDecision = 2;
		}
		return;
	}
#endif
	if (IntNo == BUT1_INT)
	{
		if (g_bUartState == false)
		{
			g_Uart.Enable();
			g_bUartState = true;
		}
		else
		{
			g_Uart.Disable();
			g_bUartState = false;
		}
	}
}

bool HardwareInit()
{
	g_bUartState = g_Uart.Init(g_UartCfg);
	if (!g_bUartState)
	{
		return false;
	}

#ifdef TIMER_DEVNO
	// Free running timer for the millisecond stamp on each log line.
	if (!g_Timer.Init(s_TimerCfg))
	{
		return false;
	}
#endif

	// Route SysLog to the same UART so the SMP/ATT stack traces (SMP_TRACE,
	// DEBUG_PRINTF) appear here alongside the application output. Without this
	// the stack pairs/runs silently and no trace is seen.
#ifdef TIMER_DEVNO
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)g_Uart, 0,
			   (TimerDev_t *)g_Timer, 0);
#else
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)g_Uart, 0,
			   nullptr, 0);
#endif

	IOPinCfg(s_LedPins, s_NbLedPins);

	for (int i = 0; i < s_NbLedPins; i++)
	{
		IOPinSet(s_LedPins[i].PortNo, s_LedPins[i].PinNo);
	}

	IOPinCfg(s_ButPins, s_NbButPins);

	IOPinEnableInterrupt(BUT1_INT, IRQ_PRIO_LOW, s_ButPins[0].PortNo,
						 s_ButPins[0].PinNo, IOPINSENSE_LOW_TRANSITION, ButEvent, NULL);
#ifdef BLE_SECURITY_BUTTON_CONFIRM
	IOPinEnableInterrupt(BUT2_INT, IRQ_PRIO_LOW, s_ButPins[1].PortNo,
						 s_ButPins[1].PinNo, IOPINSENSE_LOW_TRANSITION, ButEvent, NULL);
#endif
	return true;
}

void BtAppInitUserData()
{
	// Optional by linkage: open-link users may use UartBleDemo instead.
	// Non-NONE security is implemented entirely in the Bluetooth library.
	(void)BtAppSecInit();
}

void BtAppEvtConnected(uint16_t ConnHdl)
{
	(void)ConnHdl;
	// Retry a frame held while the BLE link had no notification recipient.
	if (s_UartRxBuffLen > 0)
	{
		UartRxQue();
	}
}

void UartRxChedHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_bUartRxPending = false;
	if (s_UartRxBuffLen == 0)
	{
		s_UartRxBuffLen = g_Uart.Rx(s_UartRxBuff, sizeof(s_UartRxBuff));
	}
#ifdef BLE_SECURITY_UART_CONFIRM
	if (s_PairAwaiting)
	{
		for (int i = 0; i < s_UartRxBuffLen; i++)
		{
			if (s_UartRxBuff[i] == 'y' || s_UartRxBuff[i] == 'Y')
			{
				s_PairDecision = 1;
				break;
			}
			if (s_UartRxBuff[i] == 'n' || s_UartRxBuff[i] == 'N')
			{
				s_PairDecision = 2;
				break;
			}
		}
		s_UartRxBuffLen = 0;
		return;
	}
#endif
	if (s_UartRxBuffLen > 0 &&
		BtAppNotify(&g_UartChars[0], s_UartRxBuff, (uint16_t)s_UartRxBuffLen))
	{
		s_UartRxBuffLen = 0;
	}
}

static void SysLogFlushEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_bSysLogFlushPending = false;
	(void)SysLogFlush(SysLogGet());
}

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
			s_bUartRxTimeout = true;
			UartRxQue();
			break;
		case UART_EVT_RXDATA:
			UartRxQue();
			break;
		case UART_EVT_TXREADY:
			if (CFifoPeek(SysLogGet()->hFifo) != nullptr)
			{
				SysLogFlushQue();
			}
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

#ifdef NET_UART_CORE_NO
	// Assign the other core's console before Bluetooth starts that core.
	if (!IOPinAssignCore(NET_UART_PORT, NET_UART_RX_PIN, NET_UART_CORE_NO) ||
		!IOPinAssignCore(NET_UART_PORT, NET_UART_TX_PIN, NET_UART_CORE_NO))
	{
		return 1;
	}
#endif

#ifdef BLE_STATUS_DISPLAY
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)))
	{
		return 1;
	}
#endif
	if (!HardwareInit())
	{
		return 1;
	}
#ifdef BLE_STATUS_DISPLAY
	if (!StatusDisplayInit())
	{
		SysLogPrintf(SysLogGet(), "Display initialization failed\r\n");
		return 1;
	}
#endif

	SysLogPrintf(SysLogGet(), "BLE security demo\r\n");

	//g_Uart.Disable();

	if (!BtAppInit(&s_BleAppCfg))
	{
		SysLogPrintf(SysLogGet(), "BtAppInit failed\r\n");
#ifdef BLE_STATUS_DISPLAY
		StatusDisplayError("BLE init failed");
#endif
		while (true)
		{
			__NOP();
		}
	}

#ifdef BLE_STATUS_DISPLAY
	s_BleReady = true;
#endif
	AppRun();

	// AppRun is not expected to return. Keep embedded startup from falling
	// through newlib exit if it ever does.
	SysLogPrintf(SysLogGet(), "AppRun returned\r\n");
	while (true)
	{
		__NOP();
	}
}

bool AppCheckStatus(void)
{
	BtAppCheckStatus();
#if defined(BLE_SECURITY_BUTTON_CONFIRM) || defined(BLE_SECURITY_UART_CONFIRM)
	uint32_t irq = DisableInterrupt();
	uint8_t decision = s_PairDecision;
	uint16_t conn = s_PairConn;
	if (decision != 0)
	{
		s_PairDecision = 0;
		s_PairAwaiting = false;
	}
	EnableInterrupt(irq);
	if (decision != 0)
	{
		SysLogPrintf(SysLogGet(), "Pairing numeric comparison %s (hdl=%u)\r\n",
					 decision == 1 ? "accepted" : "rejected", (unsigned)conn);
		BtAppPairDecision(conn, decision == 1);
#ifdef BLE_SECURITY_DISCONNECT_ON_REJECT
		// S132 may defer its SMP Pairing Failed until the central transmits
		// DHKey Check. Terminate this test connection on explicit user rejection
		// instead of leaving the central's confirmation UI to time out.
		if (decision == 2)
		{
			BtAppDisconnectConn(conn);
		}
#endif
	}
#endif

	// A pending callback on an empty queue was refused. Keep the check and
	// retry together so an interrupt cannot queue the same callback between them.
	uint32_t state = DisableInterrupt();

	if (AppEvtHandlerPending() == false)
	{
		if (s_bUartRxPending)
		{
			(void)AppEvtHandlerQue(0, nullptr, UartRxChedHandler);
		}
		if (s_bSysLogFlushPending)
		{
			(void)AppEvtHandlerQue(0, nullptr, SysLogFlushEvt);
		}
	}
	const bool idle = s_bUartRxPending == false &&
					  s_bSysLogFlushPending == false &&
					  AppEvtHandlerPending() == false;
	EnableInterrupt(state);
	return idle;
}

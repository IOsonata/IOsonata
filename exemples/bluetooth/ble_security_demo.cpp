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
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "syslog.h"

#include "board.h"

// Configure a security policy in the application board.h. The library owns
// SMP, key exchange, bonding, persistence and security events.
// No console commands or application pairing state machine are required.
#ifndef BLE_SECURITY_TYPE
#define BLE_SECURITY_TYPE BTGAP_SECTYPE_STATICKEY_NO_MITM
#endif
#ifndef BLE_SECURITY_EXCHG
#define BLE_SECURITY_EXCHG BTAPP_SECEXCHG_NONE
#endif

#ifndef UART_DEVNO
#define UART_DEVNO					0
#endif

#ifndef BUT2_INT
#define BUT2_INT 1
#endif

#ifndef BUT1_INT
#define BUT1_INT					0
#endif

#define DEVICE_NAME					"BLESecurityDemo"

#define PACKET_SIZE					20

#define MANUFACTURER_NAME			"I-SYST inc."
#define MODEL_NAME					"Generic"

#define MANUFACTURER_ID				ISYST_BLUETOOTH_ID
#define ORG_UNIQUE_ID				ISYST_BLUETOOTH_ID

#define APP_ADV_INTERVAL			64	// in msec

#define APP_ADV_TIMEOUT				0	// in msec

#define MIN_CONN_INTERVAL			10	// in msec
#define MAX_CONN_INTERVAL			40	// in msec

#define BLE_UART_UUID_BASE			BLUEIO_UUID_BASE

#define BLE_UART_UUID_SERVICE		BLUEIO_UUID_UART_SERVICE		//!< BlueIO default service
#define BLE_UART_UUID_TX_CHAR		BLUEIO_UUID_UART_TX_CHAR		//!< Data characteristic
#define BLE_UART_UUID_RX_CHAR		BLUEIO_UUID_UART_RX_CHAR		//!< Command control characteristic

void UartTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

static const BtUuidArr_t s_AdvUuid = {
	.BaseIdx = 1,
	.Type = BT_UUID_TYPE_16,
	.Count = 1,
	.Uuid16 = {BLE_UART_UUID_SERVICE,}
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
	MODEL_NAME,       		// Model name
	MANUFACTURER_NAME,		// Manufacturer name
	"123",					// Serial number string
	"0.0",					// Firmware version string
	"0.0",					// Hardware version string
};

uint8_t g_AdvLong[] = "1234567890abcdefghijklmnopqrstuvwxyz`!@#$%^&*()_+\0";

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.PeriphDevMax = 0, 				// Max peripheral devices we connect to as central
	.CentralDevMax = 2, 				// Max central devices we serve as peripheral; keep advertising after the first
	.pDevName = DEVICE_NAME,			// Device name
	.VendorId = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.Appearance = 0,
	.pDevInfo = &s_UartBleDevDesc,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLE_SECURITY_TYPE,
	.SecExchg = BLE_SECURITY_EXCHG,
	.bCompleteUuidList = false,
	.pAdvUuid = &s_AdvUuid,      			// Service uuids to advertise
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = CONNECT_LED_PIN,// Led pin number
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0,						// Tx power
	// .SDEvtHandler removed for compatibility with older BtAppCfg_t definitions
	.pLongWrPoolMem = g_LWrBuffer,		// Long-write reassembly pool (split across peer slots)
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

#define UARTFIFOSIZE				CFIFO_MEMSIZE(256)

static uint8_t s_UartRxFifo[UARTFIFOSIZE];
static uint8_t s_UartTxFifo[UARTFIFOSIZE];

/// UART pins definitions
static IOPinCfg_t s_UartPins[] = UART_PINS;

/// UART configuration
const UARTCfg_t g_UartCfg = {
	.DevNo = UART_DEVNO,				// Device number zero based
	.pIOPinMap = s_UartPins,				// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),	// Total number of UART pins used
	.Rate = 115200,						// Baudrate
	.DataBits = 8,						// Data bits
	.Parity = UART_PARITY_NONE,			// Parity
	.StopBits = 1,						// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,	// Flow control
	.bIntMode = true,					// Interrupt mode
	.IntPrio = IRQ_PRIO_LOW,			// Interrupt priority
	.EvtCallback = UartEvthandler,		// UART event handler
	.bFifoBlocking = true,				// Blocking FIFO
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

// Pending stays set through queue refusal until the callback runs.
static volatile bool s_bUartRxPending = false;
static volatile bool s_bUartRxTimeout = false;
static volatile bool s_bSysLogFlushPending = false;

void UartRxChedHandler(uint32_t Evt, void *pCtx);
static void SysLogFlushEvt(uint32_t Evt, void *pCtx);

static uint8_t s_UartRxBuff[PACKET_SIZE];
static int s_UartRxBuffLen = 0;

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

void BtAppPeriphEvtHandler(uint32_t Evt, void * const pCtx)
{
	//BtGattEvtHandler(Evt, pCtx);
}

void BtAppInitUserServices()
{
	bool res;
	res = BtGattSrvcAdd(&g_UartBleSrvc);
}

#ifdef BLE_SECURITY_BUTTON_CONFIRM
// User-interface state only. The IOsonata library retains the pairing
// transaction while awaiting the decision.
static volatile uint8_t s_PairDecision = 0; // 0=pending, 1=accept, 2=reject
static volatile bool s_PairAwaiting = false;
static uint16_t s_PairConn = 0;

int BtAppPairConfirm(uint16_t ConnHdl, uint32_t Number)
{
	s_PairConn = ConnHdl;
	s_PairDecision = 0;
	s_PairAwaiting = true;
	SysLogPrintf(SysLogGet(), "Compare %06u on both devices: BUT1=accept, BUT2=reject\r\n",
			(unsigned)Number);
	return -1; // Defer; no blocking and no protocol reply in this callback.
}
#endif

void ButEvent(int IntNo, void *pCtx)
{
#ifdef BLE_SECURITY_BUTTON_CONFIRM
	if (s_PairAwaiting)
	{
		if (IntNo == BUT1_INT) s_PairDecision = 1;
		else if (IntNo == BUT2_INT) s_PairDecision = 2;
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

void HardwareInit()
{
	g_bUartState = g_Uart.Init(g_UartCfg);

	// Route SysLog to the same UART so the SMP/ATT stack traces (SMP_TRACE,
	// DEBUG_PRINTF) appear here alongside the application output. Without this
	// the stack pairs/runs silently and no trace is seen.
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)g_Uart, 0,
			   nullptr, 0);

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
	HardwareInit();

	SysLogPrintf(SysLogGet(), "BLE security demo\r\n");

	//g_Uart.Disable();

	if (!BtAppInit(&s_BleAppCfg))
	{
		SysLogPrintf(SysLogGet(), "BtAppInit failed\r\n");
		while (true)
		{
			__NOP();
		}
	}

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
#ifdef BLE_SECURITY_BUTTON_CONFIRM
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

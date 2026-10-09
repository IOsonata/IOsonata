/**-------------------------------------------------------------------------
@example	uart_ble.cpp

@brief	Uart BLE demo

This application demo shows UART Rx/Tx over BLE custom service using EHAL library.
For evaluating power consumption of the UART, the button 1 is used to enable/disable it.
This example also demonstrates passkey paring mode.

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
#include "bluetooth/bt_gatt.h"
#include "bluetooth/blueio_blesrvc.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "syslog.h"

#include "board.h"

#ifndef UART_DEVNO
#define UART_DEVNO					0
#endif

#ifndef BUT1_INT
#define BUT1_INT					0
#endif

#define DEVICE_NAME					"UARTDemo"

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
											g_UartChars);

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
	.SecType = BTGAP_SECTYPE_NONE,		// Open UART/BLE bridge
	.SecExchg = BTAPP_SECEXCHG_NONE,
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
static volatile bool s_bSysLogFlushPending = false;

// UART data is forwarded by the application; the Bluetooth library owns security.
void UartRxHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_bUartRxPending = false;
	int len = g_Uart.Rx(s_UartRxBuff, sizeof(s_UartRxBuff));
	if (len > 0)
	{
		(void)BtAppNotify(&g_UartChars[0], s_UartRxBuff, (uint16_t)len);
	}
}

#if 0
uint32_t BleSrvcCharNotify(BtGattSrvc_t *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen)
{
	BtGattCharNotify(&pSrvc->pCharArray[Idx], pData, DataLen);

	return 0;
}
#endif

// SysLog sends a record when it is logged. A record the UART had no room for
// waits in the store, so the UART ready event queues one flush to send it.
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

	SysLogPrintf(SysLogGet(), "UART over BLE\r\n");

	
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

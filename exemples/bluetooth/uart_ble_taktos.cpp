/**-------------------------------------------------------------------------
@example    uart_ble_taktos.cpp


@brief  UART over BLE with TaktOS demo

This application demo shows UART Rx/Tx over BLE custom service using EHAL library.
For evaluating power consumption of the UART, the button 1 is used to enable/disable it.
This example also demonstrates passkey paring mode.

@author Hoang Nguyen Hoan
@date   Feb. 4, 2017

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

#include "istddef.h"

#include "TaktOS.h"
#include "TaktOSThread.h"
#include "TaktOSSem.h"
#include "TaktOSQueue.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/blueio_blesrvc.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"

#include "board.h"

#define DEVICE_NAME                     "UARTTaktOS"                            //!< Name of device. Will be included in the advertising data.

#define MANUFACTURER_NAME               "I-SYST inc."                       //!< Manufacturer. Will be passed to Device Information Service.

#define MODEL_NAME                      "I-SYST-BLE"                            //!< Model number. Will be passed to Device Information Service.

#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID                               //!< Manufacturer ID, part of System ID. Will be passed to Device Information Service.
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID                               //!< Organizational Unique ID, part of System ID. Will be passed to Device Information Service.

#define APP_ADV_INTERVAL                300//MSEC_TO_UNITS(300, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         //!< The advertising timeout (in units of seconds).

#define MIN_CONN_INTERVAL               10//MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               40//MSEC_TO_UNITS(40, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

// The BLE task runs the work of the stack queued with BtEvtQue, the RX task
// only moves UART bytes to a notification.
#define BLE_TAKTOS_THREAD_STACK         1024u
#define RX_TAKTOS_THREAD_STACK          512u

// One notification holds one characteristic value
#define UART_BLE_PACKET_SIZE            20

#ifndef TAKTOS_APP_TICK_HZ
#define TAKTOS_APP_TICK_HZ              1000u
#endif

#ifndef TAKTOS_APP_CORE_CLOCK_HZ
// Default to the CMSIS-standard runtime value populated by SystemInit().
// Override at build time (-DTAKTOS_APP_CORE_CLOCK_HZ=<Hz>) when a compile-time
// constant is preferred.
#define TAKTOS_APP_CORE_CLOCK_HZ        SystemCoreClock
#endif

void UartTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

static hTaktOSThread_t s_BleTask = NULL;
static hTaktOSThread_t s_RxTask = NULL;

static TaktOSSem_t s_RxEvtSem;

// Bluetooth work: one message per BtEvtQue call, run by the BLE task
typedef struct {
	uint32_t EvtId;
	void *pCtx;
	BtEvtQueHandler_t Handler;
} BleWork_t;

#define BLE_WORK_QUE_SIZE				16u

static uint8_t g_BleWorkQueMem[BLE_WORK_QUE_SIZE * sizeof(BleWork_t)] TAKT_ALIGNED(4);
static TaktOSQueue_t s_BleWorkQue;

static uint8_t g_BleTaskMem[TAKTOS_THREAD_MEM_SIZE(BLE_TAKTOS_THREAD_STACK)] TAKT_ALIGNED(4);
static uint8_t g_RxTaskMem[TAKTOS_THREAD_MEM_SIZE(RX_TAKTOS_THREAD_STACK)] TAKT_ALIGNED(4);

//static const ble_uuid_t s_AdvUuids[] = {
//	{BLUEIO_UUID_UART_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
//};

static const char s_RxCharDescString[] = {
	"UART Rx characteristic",
};

static const char s_TxCharDescString[] = {
	"UART Tx characteristic",
};

uint8_t g_ManData[8];

BtGattChar_t g_UartChars[] = {
	// Read + Notify (server-pushed)
	BT_CHAR(BLUEIO_UUID_UART_RX_CHAR, 20,
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
			s_RxCharDescString),
	// Write Without Response (peer sink, callback consumes)
	BT_CHAR(BLUEIO_UUID_UART_TX_CHAR, 20,
			BT_GATT_CHAR_PROP_WRITE_WORESP,
			s_TxCharDescString,
			.WrCB = UartTxSrvcCallback),
};

uint8_t g_LWrBuffer[512];

BtGattSrvc_t g_UartBleSrvc = BT_SRVC_CUSTOM(BLUEIO_UUID_BASE,
											BLUEIO_UUID_UART_SERVICE,
											g_UartChars);

const BtAppDevInfo_t s_UartBleDevDesc {
	MODEL_NAME,           // Model name
	MANUFACTURER_NAME,          // Manufacturer name
	"",                     // Serial number string
	"0.0",                  // Firmware version string
	"0.0",                  // Hardware version string
};

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.PeriphDevMax = 0, 				// Max peripheral devices we connect to as central
	.CentralDevMax = 1, 				// Max central devices we serve as peripheral
	.pDevName = DEVICE_NAME,			// Device name
	.VendorId = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.pDevInfo = &s_UartBleDevDesc,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BTGAP_SECTYPE_NONE,//BLEAPP_SECTYPE_STATICKEY_MITM,//BLEAPP_SECTYPE_NONE,    // Secure connection type
	.SecExchg = BTAPP_SECEXCHG_NONE,	// Security key exchange
	.pAdvUuid = NULL,      			// Service uuids to advertise
	//.NbAdvUuid = 0, 					// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = 0,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = CONNECT_LED_PIN,// Led pin number
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0,						// Tx power
	.pLongWrPoolMem = g_LWrBuffer,		// Long-write reassembly pool (split across peer slots)
	.LongWrPoolMemSize = sizeof(g_LWrBuffer),
};

int UartEvtHandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// UART configuration data

static IOPinCfg_t s_UartPins[] = UART_PINS;

const UARTCfg_t g_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 1000000,			// Rate
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,					// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,//TAKTOS_PRIORITY_NORMAL,
	.EvtCallback = UartEvtHandler,
	.bFifoBlocking = true,				// fifo blocking mode
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = 0,
	.pTxMem = NULL,
	.bDMAMode = true,
};

// UART object instance
UART g_Uart;

static const IOPINCFG s_LedPins[] = LED_PINS;

static int s_NbLedPins = sizeof(s_LedPins) / sizeof(IOPINCFG);

static const IOPINCFG s_ButPins[] = BUTTON_PINS;

static int s_NbButPins = sizeof(s_ButPins) / sizeof(IOPINCFG);

int g_DelayCnt = 0;
volatile bool g_bUartState = false;

// Local error codes for fatal traps
#define APP_ERR_INVALID_PARAM    (-1)
#define APP_ERR_NO_MEM           (-2)

// ADD a generic local fatal handler (no vendor SDK dependency)
static void AppFatalError(int err)
{
	(void)err;
	while (1)
	{
		// trap here
	}
}
void UartTxSrvcCallback(BtGattChar_t *pBlueIOSvc, uint8_t *pData, int Offset, int Len)
{
	g_Uart.Tx(pData, Len);
}

void BtAppPeriphEvtHandler(uint32_t Evt, void * const pCtx)
{
	// The port has already dispatched this event to the GATT services.
	(void)Evt;
	(void)pCtx;
}

void BtAppInitUserServices()
{
	// uint32_t       err_code;

	bool res = BtGattSrvcAdd(&g_UartBleSrvc);
	if (res == false)
	{
		while(1);
	}
//    APP_ERROR_CHECK(err_code);
}

void ButEvent(int IntNo, void *pCtx)
{
	if (IntNo == 0)
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
	g_Uart.Init(g_UartCfg);

	g_Uart.printf("UartBleTaktOS demo\r\n");

	IOPinCfg(s_ButPins, s_NbButPins);

	IOPinCfg(s_LedPins, s_NbLedPins);

	for (int i = 0; i < s_NbLedPins; i++)
	{
		IOPinSet(s_LedPins[i].PortNo, s_LedPins[i].PinNo);
	}

	IOPinCfg(s_ButPins, s_NbButPins);

	IOPinEnableInterrupt(0, IRQ_PRIO_LOW, s_ButPins[0].PortNo, s_ButPins[0].PinNo, IOPINSENSE_LOW_TRANSITION, ButEvent, NULL);
}

// UART interrupt: wake the RX task. A full binary semaphore already records
// the wake, so any number of events before the take cost nothing.
int UartEvtHandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	(void)pDev;
	(void)pBuffer;
	(void)BufferLen;

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			(void)TaktOSSemGive(&s_RxEvtSem, false);
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return 0;
}

// RTOS bridge: link-time override of BtEvtQue, see bluetooth/bt_app.h. The
// Bluetooth stack calls it, often from interrupt context, for every piece of
// work that must run outside the interrupt. Each one becomes a message to the
// BLE task, which runs it. The application event queue is not used.
bool BtEvtQue(uint32_t EvtId, void *pCtx, BtEvtQueHandler_t Handler)
{
	const BleWork_t work = { EvtId, pCtx, Handler };

	return TaktOSQueueSend(&s_BleWorkQue, &work, false, 0) == TAKTOS_OK;
}

// UART to BLE. The characteristic takes UART_BLE_PACKET_SIZE bytes per
// notification, so the UART FIFO is drained in packets. A packet the stack
// refuses, its notification queue being full, is kept and offered again: the
// ports report no notification done event to the application, so the retry
// waits one tick instead of the next UART event, which may never come.
static void RxTask(void * pvParameter)
{
	(void)pvParameter;

	uint8_t buff[UART_BLE_PACKET_SIZE];
	int pending = 0;

	while (1)
	{
		(void)TaktOSSemTake(&s_RxEvtSem, true, pending > 0 ? 1u : TAKTOS_WAIT_FOREVER);

		while (true)
		{
			if (pending == 0)
			{
				pending = g_Uart.Rx(buff, sizeof(buff));
				if (pending <= 0)
				{
					pending = 0;
					break;
				}
			}
			if (BtAppNotify(&g_UartBleSrvc.pCharArray[0], buff, (uint16_t)pending) == false)
			{
				// Not connected, notification off, or the stack queue full
				break;
			}
			pending = 0;
		}
	}
}

void BtAppInitUserData()
{
	// Init user data
}


// BLE task: runs the Bluetooth work as its messages arrive, forever.
static void BleTask(void * pvParameter)
{
	(void)pvParameter;

	while (1)
	{
		BleWork_t work;

		if (TaktOSQueueReceive(&s_BleWorkQue, &work, false, 0) != TAKTOS_OK)
		{
			// Queue empty: queue again the work it refused, then wait
			BtAppCheckStatus();
			if (TaktOSQueueReceive(&s_BleWorkQue, &work, true, TAKTOS_WAIT_FOREVER) != TAKTOS_OK)
			{
				continue;
			}
		}
		work.Handler(work.EvtId, work.pCtx);
	}
}

void TaktOSAppInit()
{
	if (TaktOSSemInit(&s_RxEvtSem, 0u, 1u) != TAKTOS_OK)
	{
		AppFatalError(APP_ERR_INVALID_PARAM);
	}

	TaktOSCfg_t cfg = {
		.KernClockHz = TAKTOS_APP_CORE_CLOCK_HZ,
		.TickHz = TAKTOS_APP_TICK_HZ,
	};

	if (TaktOSInit(&cfg) != TAKTOS_OK)
	{
		AppFatalError(APP_ERR_INVALID_PARAM);
	}

	s_BleTask = TaktOSThreadCreate(g_BleTaskMem,
								   sizeof(g_BleTaskMem),
								   BleTask,
								   NULL,
								   TAKTOS_PRIORITY_HIGH);
	if (s_BleTask == NULL)
	{
		AppFatalError(APP_ERR_NO_MEM);
	}

	s_RxTask = TaktOSThreadCreate(g_RxTaskMem,
								  sizeof(g_RxTaskMem),
								  RxTask,
								  NULL,
								  TAKTOS_PRIORITY_NORMAL);
	if (s_RxTask == NULL)
	{
		AppFatalError(APP_ERR_NO_MEM);
	}
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	HardwareInit();

	// The Bluetooth work queue exists before BtAppInit queues its first work
	if (TaktOSQueueInit(&s_BleWorkQue, g_BleWorkQueMem, sizeof(BleWork_t),
						BLE_WORK_QUE_SIZE) != TAKTOS_OK)
	{
		AppFatalError(APP_ERR_INVALID_PARAM);
	}

	BtAppInit(&s_BleAppCfg);//, true);

	TaktOSAppInit();

	TaktOSStart();

	return 0;
}

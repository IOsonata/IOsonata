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

#include "istddef.h"
#include "bluetooth/bt_app.h"
//#include "bluetooth/bt_gap.h"
#include "bluetooth/blueio_blesrvc.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "stddev.h"
#include "board.h"

//#define DUMP_MAN_SPEC_DATA

// BLE
#define DEVICE_NAME             "UARTCentral"

#define MANUFACTURER_NAME       "I-SYST inc."
#define MODEL_NAME              "IMM-NRF5x"
#define MANUFACTURER_ID         ISYST_BLUETOOTH_ID
#define ORG_UNIQUE_ID           ISYST_BLUETOOTH_ID

#define MIN_CONN_INTERVAL       10 // in msec
#define MAX_CONN_INTERVAL       40 // in msec

#define SCAN_INTERVAL           1000	// in msec
#define SCAN_WINDOW             100 	// in msec
#define SCAN_TIMEOUT            0		// 0 = never

#define TARGET_BRIDGE_DEV_NAME	"BlueIO832Mini"

// UART
#define BLE_MTU_SIZE			512		// in bytes
#define PACKET_SIZE				256
#define UART_MAX_DATA_LEN  		(PACKET_SIZE*4)
#define UARTFIFOSIZE			CFIFO_MEMSIZE(UART_MAX_DATA_LEN)

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx);

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_CENTRAL,
	.CentLinkCount = 1, 							// Number of central link
	.PeriLinkCount = 0, 							// Number of peripheral link
	.pDevName = DEVICE_NAME,                // Device name
	.VendorId = ISYST_BLUETOOTH_ID,     	// PnP Bluetooth/USB vendor id
	.ProductId = 1,                      	// PnP Product ID
	.ProductVer = 0,							// Pnp prod version
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT,    // Led port nuber
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,     // Led pin number
	.ConnLedActLevel = 0,
	.TxPower = 0,							// Tx power
	.MaxMtu = BLE_MTU_SIZE,
};

static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

/// UART operation mode config
alignas(4) uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) uint8_t s_UartTxFifo[UARTFIFOSIZE];

UARTCfg_t g_UartCfg = {
	.DevNo = 0,									// Device number zero based
	.pIOPinMap = s_UartPins,					// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),					// Total number of UART pins used
	.Rate = 115200,								// Baudrate
	.DataBits = 8,								// Data bits
	.Parity = UART_PARITY_NONE,			// Parity
	.StopBits = 1,								// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,	// Flow control
	.bIntMode = true,							// Interrupt mode
	.IntPrio = 6,//APP_IRQ_PRIORITY_LOW,			// Interrupt priority
	.EvtCallback = UartEvthandler,			// UART event handler
	.bFifoBlocking = true,						// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
};

// UART object instance
UART g_Uart;

int g_DelayCnt = 0;

static BtGapScanCfg_t const g_ScanParams = {
	.Type = BTSCAN_TYPE_ACTIVE,
	.Param = {
		.Interval = SCAN_INTERVAL,
		.Duration = SCAN_WINDOW,
		.Timeout = SCAN_TIMEOUT,
	},
	.BaseUid = BLUEIO_UUID_BASE,
	.ServUid = BLUEIO_UUID_UART_SERVICE,//s_UartBleSrvAdvUuid,
};

uint8_t g_ScanBuff[BT_GAP_SCAN_BUFFER_SIZE_DEFAULT];//BLE_GAP_SCAN_BUFFER_EXTENDED_MAX];

bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *pAdvData)
{
//	g_Uart.printf("BtAppScanReport : Rssi = %d\r\n", Rssi);
	if (AdvLen > 0)
	{
		char name[32];
		size_t l = BtAdvDataGetDevName(pAdvData, AdvLen, name, 32);

		if (l > 0)
		{
			name[l-1] = 0;
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

void HardwareInit()
{
	g_Uart.Init(g_UartCfg);

	g_Uart.printf("BLE Central Scan Demo\r\n");
	g_Uart.printf("UART Configuration: %d, Flow Control (%s), Parity (%s)\r\n",
			g_UartCfg.Rate,
			(g_UartCfg.FlowControl == UART_FLWCTRL_NONE) ? "No" : "Yes",
			(g_UartCfg.Parity == UART_PARITY_NONE) ? "No" : "Yes");

}

void BtAppInitUserData()
{
}

void UartRxChedHandler(void * p_event_data, uint16_t event_size)
{
	// TODO: Use CFIFO for this function for avoiding dropped data packets
	uint8_t buff[PACKET_SIZE];

	int l = g_Uart.Rx(buff, PACKET_SIZE);
	if (l > 0)
	{
	//	if (g_ConnectedDev.ConnHdl != BLE_CONN_HANDLE_INVALID && g_BleTxCharHdl != BLE_CONN_HANDLE_INVALID)
		{
	//		BtAppWrite(g_ConnectedDev.ConnHdl, g_BleTxCharHdl, buff, l);
		}
	}
}

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
//	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
		//	app_sched_event_put(NULL, 0, UartRxChedHandler);
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
    HardwareInit();

    BtAppInit(&s_BleAppCfg);

    BtAppScanInit((BtGapScanCfg_t*)&g_ScanParams);

    BtAppScan();

    BtAppRun();

	return 0;
}


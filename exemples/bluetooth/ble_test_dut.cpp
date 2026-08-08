/**-------------------------------------------------------------------------
@example	ble_test_dut.cpp

@brief	Generic IOsonata Bluetooth over-the-air test DUT.

One application source is used unchanged for the supported IOsonata targets.
The active target port owns all controller/vendor details. The application
provides only a deterministic peripheral, Numeric Comparison security, a small
GATT endpoint, and a UART control/trace channel for an external test harness.

@author	Hoang Nguyen Hoan
@date	Aug. 8, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/

#include <string.h>

#include "istddef.h"
#include "app_evt_handler.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "syslog.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/blueio_blesrvc.h"
#include "bluetooth/bt_appearance.h"

#include "board.h"

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

#define DUT_DEVICE_NAME			"IOsonataDUT"
#define DUT_MANUFACTURER_NAME	"I-SYST inc."
#define DUT_MODEL_NAME			"BLE Test DUT"

#define DUT_ADV_INTERVAL_MS		64
#define DUT_CONN_INTERVAL_MIN_MS	10
#define DUT_CONN_INTERVAL_MAX_MS	40
#define DUT_GATT_VALUE_MAX		32

#define DUT_UART_FIFO_SIZE		CFIFO_MEMSIZE(256)
#define DUT_UART_LINE_MAX		80

#define DUT_UUID_BASE			BLUEIO_UUID_BASE
#define DUT_UUID_SERVICE		BLUEIO_UUID_UART_SERVICE
#define DUT_UUID_VALUE_CHAR		BLUEIO_UUID_UART_RX_CHAR

static uint8_t s_UartRxFifo[DUT_UART_FIFO_SIZE];
static uint8_t s_UartTxFifo[DUT_UART_FIFO_SIZE];
static IOPinCfg_t s_UartPins[] = UART_PINS;

static int UartEvtHandler(UARTDev_t *pDev, UART_EVT EvtId,
	uint8_t *pBuffer, int BufferLen);
static void DutUartHandler(uint32_t Evt, void *pCtx);
static void DutValueWrite(BtGattChar_t *pChar, uint8_t *pData,
	int Offset, int Len);

static const UARTCfg_t s_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCallback = UartEvtHandler,
	.bFifoBlocking = true,
	.RxMemSize = DUT_UART_FIFO_SIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = DUT_UART_FIFO_SIZE,
	.pTxMem = s_UartTxFifo,
};

UART g_Uart;

static const char s_ValueDesc[] = "BLE DUT value";
static uint8_t s_Value[DUT_GATT_VALUE_MAX];

BtGattChar_t g_DutChars[] = {
	BT_CHAR(DUT_UUID_VALUE_CHAR, DUT_GATT_VALUE_MAX,
		BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE |
		BT_GATT_CHAR_PROP_NOTIFY,
		s_ValueDesc,
		.WrCB = DutValueWrite),
};

BtGattSrvc_t g_DutService =
	BT_SRVC_CUSTOM(DUT_UUID_BASE, DUT_UUID_SERVICE, g_DutChars);

static const BtUuidArr_t s_AdvUuid = {
	.BaseIdx = 1,
	.Type = BT_UUID_TYPE_16,
	.Count = 1,
	.Uuid16 = { DUT_UUID_SERVICE, }
};

static const BtAppDevInfo_t s_DevInfo = {
	DUT_MODEL_NAME,
	DUT_MANUFACTURER_NAME,
	"001",
	"0.1",
	"0.1",
};

// Stable marker used by the HciController Python test to identify the DUT by
// advertising contents rather than by its rotating address.
static const uint8_t s_AdvMarker[] = { 'D', 'U', 'T', 1 };

static uint8_t s_LongWriteMem[512];

static const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.CentLinkCount = 0,
	.PeriLinkCount = 1,
	.pDevName = DUT_DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.ProductId = 1,
	.ProductVer = 1,
	.Appearance = BT_APPEAR_UNKNOWN_GENERIC,
	.pDevInfo = &s_DevInfo,
	.pAdvManData = s_AdvMarker,
	.AdvManDataLen = sizeof(s_AdvMarker),
	.pSrManData = nullptr,
	.SrManDataLen = 0,
	.SecType = BTGAP_SECTYPE_LESC_MITM,
	.SecExchg = BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO,
	.bCompleteUuidList = true,
	.pAdvUuid = &s_AdvUuid,
	.AdvInterval = DUT_ADV_INTERVAL_MS,
	.AdvTimeout = 0,
	.AdvSlowInterval = 0,
	.ConnIntervalMin = DUT_CONN_INTERVAL_MIN_MS,
	.ConnIntervalMax = DUT_CONN_INTERVAL_MAX_MS,
	.ConnLedPort = -1,
	.ConnLedPin = -1,
	.TxPower = 0,
	.pLongWrPoolMem = s_LongWriteMem,
	.LongWrPoolMemSize = sizeof(s_LongWriteMem),
};

static volatile bool s_NumCompPending = false;
static uint16_t s_NumCompConnHdl = BT_CONN_HDL_INVALID;

static void PrintAddr(const uint8_t Addr[6])
{
	for (int i = 5; i >= 0; i--)
	{
		g_Uart.printf("%02X", Addr[i]);
		if (i != 0)
		{
			g_Uart.printf(":");
		}
	}
}

static void PrintStatus(void)
{
	uint8_t addrType = 0;
	uint8_t addr[6] = {};

	BtSmpLocalAddrGet(&addrType, addr);

	g_Uart.printf("DUT STATUS init=%d connected=%d id_type=%u id=",
		BtInitialized() ? 1 : 0, BtConnected() ? 1 : 0,
		(unsigned)addrType);
	PrintAddr(addr);
	g_Uart.printf("\r\n");
}

static void PrintLink(uint16_t ConnHdl)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		g_Uart.printf("DUT LINK hdl=%u unavailable\r\n", (unsigned)ConnHdl);
		return;
	}

	g_Uart.printf("DUT LINK hdl=%u role=%u peer_type=%u peer=",
		(unsigned)ConnHdl, (unsigned)pPeer->Conn.Role,
		(unsigned)pPeer->Conn.PeerAddrType);
	PrintAddr(pPeer->Conn.PeerAddr);
	g_Uart.printf(" own_type=%u own=", (unsigned)pPeer->Conn.OwnAddrType);
	PrintAddr(pPeer->Conn.OwnAddr);
	g_Uart.printf("\r\n");
}

static void PrintHelp(void)
{
	g_Uart.printf("DUT COMMANDS status | adv start | adv stop | "
		"disconnect | bond clear | y | n | help\r\n");
}

static void HandleCommand(const char *pLine)
{
	if (strcmp(pLine, "status") == 0)
	{
		PrintStatus();
		return;
	}

	if (strcmp(pLine, "adv start") == 0)
	{
		BtAppAdvStart();
		g_Uart.printf("DUT ADV_START requested\r\n");
		return;
	}

	if (strcmp(pLine, "adv stop") == 0)
	{
		BtAppAdvStop();
		g_Uart.printf("DUT ADV_STOP requested\r\n");
		return;
	}

	if (strcmp(pLine, "disconnect") == 0)
	{
		int count = BtAppDisconnectAll();
		g_Uart.printf("DUT DISCONNECT requested=%d\r\n", count);
		return;
	}

	if (strcmp(pLine, "bond clear") == 0)
	{
		BtSmpBondClearAll();
		g_Uart.printf("DUT BOND_CLEAR requested\r\n");
		return;
	}

	if ((strcmp(pLine, "y") == 0 || strcmp(pLine, "yes") == 0) &&
		s_NumCompPending)
	{
		uint16_t hdl = s_NumCompConnHdl;
		s_NumCompPending = false;
		s_NumCompConnHdl = BT_CONN_HDL_INVALID;
		BtSmpNumericComparisonReply(hdl, true);
		g_Uart.printf("DUT NUMCOMP_REPLY hdl=%u accept=1\r\n",
			(unsigned)hdl);
		return;
	}

	if ((strcmp(pLine, "n") == 0 || strcmp(pLine, "no") == 0) &&
		s_NumCompPending)
	{
		uint16_t hdl = s_NumCompConnHdl;
		s_NumCompPending = false;
		s_NumCompConnHdl = BT_CONN_HDL_INVALID;
		BtSmpNumericComparisonReply(hdl, false);
		g_Uart.printf("DUT NUMCOMP_REPLY hdl=%u accept=0\r\n",
			(unsigned)hdl);
		return;
	}

	if (strcmp(pLine, "help") == 0)
	{
		PrintHelp();
		return;
	}

	g_Uart.printf("DUT ERROR unknown_command=%s\r\n", pLine);
}

static void DutUartHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	static char line[DUT_UART_LINE_MAX];
	static size_t len = 0;
	uint8_t c;

	while (g_Uart.Rx(&c, 1) == 1)
	{
		if (c == '\r' || c == '\n')
		{
			if (len != 0)
			{
				line[len] = 0;
				HandleCommand(line);
				len = 0;
			}
			continue;
		}

		if (len + 1 < sizeof(line))
		{
			line[len++] = (char)c;
		}
		else
		{
			len = 0;
			g_Uart.printf("DUT ERROR command_too_long\r\n");
		}
	}
}

static int UartEvtHandler(UARTDev_t *pDev, UART_EVT EvtId,
	uint8_t *pBuffer, int BufferLen)
{
	(void)pDev;
	(void)pBuffer;
	(void)BufferLen;

	if (EvtId == UART_EVT_RXDATA || EvtId == UART_EVT_RXTIMEOUT)
	{
		AppEvtHandlerQue(0, nullptr, DutUartHandler);
	}

	return 0;
}

static void DutValueWrite(BtGattChar_t *pChar, uint8_t *pData,
	int Offset, int Len)
{
	(void)Offset;

	int copyLen = Len;
	if (copyLen > DUT_GATT_VALUE_MAX)
	{
		copyLen = DUT_GATT_VALUE_MAX;
	}
	if (copyLen < 0 || (copyLen > 0 && pData == nullptr))
	{
		return;
	}

	if (copyLen > 0)
	{
		memcpy(s_Value, pData, (size_t)copyLen);
	}
	BtGattCharSetValue(pChar, s_Value, copyLen);

	g_Uart.printf("DUT GATT_WRITE len=%d data=", copyLen);
	for (int i = 0; i < copyLen; i++)
	{
		g_Uart.printf("%02X", s_Value[i]);
	}
	g_Uart.printf("\r\n");

	if (isBtGattCharNotifyEnabled(pChar))
	{
		bool sent = BtAppNotify(pChar, s_Value, (uint16_t)copyLen);
		g_Uart.printf("DUT GATT_NOTIFY sent=%d\r\n", sent ? 1 : 0);
	}
}

void BtAppInitUserData(void)
{
	BtSmpAuthConfig(BT_SMP_IOCAPS_DISPLAY_YESNO,
		BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM);
}

void BtAppInitUserServices(void)
{
	if (BtGattSrvcAdd(&g_DutService) == false)
	{
		g_Uart.printf("DUT ERROR service_add\r\n");
		return;
	}

	BtGattCharSetValue(&g_DutChars[0], s_Value, sizeof(s_Value));
}

void BtAppEvtConnected(uint16_t ConnHdl)
{
	g_Uart.printf("DUT CONNECTED hdl=%u\r\n", (unsigned)ConnHdl);
	PrintLink(ConnHdl);
}

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	if (s_NumCompPending && s_NumCompConnHdl == ConnHdl)
	{
		s_NumCompPending = false;
		s_NumCompConnHdl = BT_CONN_HDL_INVALID;
	}
	g_Uart.printf("DUT DISCONNECTED hdl=%u\r\n", (unsigned)ConnHdl);
}

void BtAppEvtSecured(uint16_t ConnHdl)
{
	g_Uart.printf("DUT SECURED hdl=%u\r\n", (unsigned)ConnHdl);
	PrintLink(ConnHdl);
}

void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
}

void BtSmpNumericComparison(uint16_t ConnHdl, uint32_t Value)
{
	s_NumCompPending = true;
	s_NumCompConnHdl = ConnHdl;

	g_Uart.printf("DUT NUMCOMP hdl=%u value=%06u\r\n",
		(unsigned)ConnHdl, (unsigned)Value);
}

static void HardwareInit(void)
{
	g_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), (DevIntrf_t*)g_Uart, 0, nullptr, 0);
}

int main(void)
{
	HardwareInit();

	g_Uart.printf("\r\nDUT BOOT name=%s\r\n", DUT_DEVICE_NAME);
	g_Uart.printf("DUT SECURITY method=numcomp bonding=1 mitm=1 lesc=1\r\n");

	if (BtAppInit(&s_BleAppCfg) == false)
	{
		g_Uart.printf("DUT ERROR bt_init\r\n");
		while (true)
		{
		}
	}

	PrintStatus();
	PrintHelp();
	g_Uart.printf("DUT READY\r\n");

	BtAppRun();

	g_Uart.printf("DUT ERROR bt_run_returned\r\n");
	while (true)
	{
	}
}

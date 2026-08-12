/**-------------------------------------------------------------------------
@example	ble_test_dut.cpp

@brief	Generic IOsonata Bluetooth over-the-air test DUT.

A Bluetooth LE device that puts the IOsonata Bluetooth stack on the air so an
external tester can check the implementation against the specification. It is
not an HCI controller and it exposes no HCI interface: the UART below is a
control and trace channel for the test operator, and every command on it names
a GAP, GATT, SMP or advertising procedure rather than an HCI opcode. What the
tester measures is the stack's behaviour on the radio.

One application source is used unchanged for the supported IOsonata targets.
The active target port owns all vendor details. The application provides a
deterministic peripheral, Numeric Comparison security, a small GATT endpoint,
and the advertising and periodic advertising procedures a tester drives.

The UART channel is line based in both directions at 115200 8N1 with no flow
control. Every line the DUT prints starts with "DUT " and a tag, so a harness
can match on the tag and parse the key=value fields that follow. A command
that cannot run answers with a single DUT ERROR line naming the reason, never
with silence, so a harness can tell a refusal from a lost line.

The over-the-air side is driven from a PC by the HciController dongle running
tests/hardware/hci_dut_compliance_test.py. That script owns the peer side of
every exchange the commands below set up: discovery, connection, ATT, PHY,
data length, periodic advertising synchronisation and PAwR responses.

The DUT is an advertiser and a peripheral. It does not scan, so it is never
the synchronising side of a periodic advertising train; the dongle is. The
periodic advertising and PAwR commands therefore configure and run trains for
the dongle to synchronise with, and the response reports the dongle produces
come back through the PAwR response callback.

A procedure the stack cannot bring up on this target is refused at the command
that asks for it rather than removed from the build, so one binary answers the
same set of commands everywhere and the tester learns what is available by
asking for it. Nothing here reports the local controller's HCI capability
bitmaps; that would describe the silicon under the stack rather than the stack
being tested, and it would let a tester skip a procedure the stack should have
been asked to perform.

@author	Hoang Nguyen Hoan
@date	Aug. 8, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/

#include <stdlib.h>
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
#define DUT_UART_LINE_MAX		160

// Manufacturer specific advertising payload the "adv ext" command grows to.
// Above 254 octets the advertising data no longer fits one HCI fragment, so a
// harness asking for the top of this range exercises the fragmenting writer
// and the reassembly a scanner has to do.
#define DUT_ADV_EXT_MAX			300

// Periodic advertising payload the DUT keeps. Far below what a train can hold,
// which is deliberate: the point is to prove the train runs and the payload
// arrives intact, not to fill a controller's periodic advertising buffer.
#define DUT_PERIODIC_DATA_MAX	64

// PAwR shape. A subevent count this small keeps the subevent buffers inside
// the RAM budget of the smallest supported target while still giving a
// scanner more than one subevent to choose between.
#define DUT_PAWR_SUBEVENT_CNT	4
#define DUT_PAWR_SUBEVENT_DATA	32
#define DUT_PAWR_RSP_SLOT_CNT	4

// PAwR timing. The subevent interval has to leave room for the response slot
// delay plus every response slot, so the three are picked together.
#define DUT_PAWR_INTERVAL_MIN	0x0050
#define DUT_PAWR_INTERVAL_MAX	0x0060
#define DUT_PAWR_SUBEVT_INTERVAL	0x20
#define DUT_PAWR_RSP_SLOT_DELAY	0x08
#define DUT_PAWR_RSP_SLOT_SPACING	0x10

#define DUT_PERIODIC_INTERVAL_MIN	0x0050
#define DUT_PERIODIC_INTERVAL_MAX	0x0060

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
	nullptr,
};

// Stable marker used by the HciController Python test to identify the DUT by
// advertising contents rather than by its rotating address. The "adv ext"
// command keeps these four octets at the front of the enlarged payload so the
// same match works on a legacy and on an extended advertising report.
static const uint8_t s_AdvMarker[] = { 'D', 'U', 'T', 1 };

static uint8_t s_AdvExtData[DUT_ADV_EXT_MAX];

static uint8_t s_LongWriteMem[512];

// Advertising configuration is rebuilt whenever the harness changes the
// advertising payload size or the coding selection, both of which are read at
// advertising init. Everything else stays as the boot value.
static BtAppCfg_t s_BleAppCfg = {
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

static uint8_t s_PeriodicData[DUT_PERIODIC_DATA_MAX];
static uint16_t s_PeriodicLen = 0;
static bool s_PeriodicReady = false;

static uint8_t s_PawrData[DUT_PAWR_SUBEVENT_CNT][DUT_PAWR_SUBEVENT_DATA];
static uint8_t s_PawrLen[DUT_PAWR_SUBEVENT_CNT];
static bool s_PawrReady = false;
static bool s_PawrReqTraced = false;

// ***
// UART line helpers
//

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

static void PrintHex(const uint8_t *pData, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		g_Uart.printf("%02X", pData[i]);
	}
}

// Match a command word and hand back what follows it with the separating
// spaces removed. An exact match hands back an empty argument, so one helper
// serves both a bare command and a command with operands.
static bool DutMatch(const char *pLine, const char *pCmd, const char **ppArg)
{
	size_t len = strlen(pCmd);

	if (strncmp(pLine, pCmd, len) != 0)
	{
		return false;
	}

	const char *p = pLine + len;

	if (*p != 0 && *p != ' ')
	{
		return false;
	}

	while (*p == ' ')
	{
		p++;
	}

	if (ppArg != nullptr)
	{
		*ppArg = p;
	}

	return true;
}

// Read one unsigned decimal or 0x prefixed value and step past it. Returns
// false when the text at pArg is not a number, which every caller treats as a
// malformed command rather than as a zero.
static bool DutParseUint(const char **ppArg, uint32_t *pVal)
{
	const char *p = *ppArg;
	char *end = nullptr;
	unsigned long v;

	if (*p == 0)
	{
		return false;
	}

	v = strtoul(p, &end, 0);
	if (end == p)
	{
		return false;
	}

	while (*end == ' ' || *end == ',')
	{
		end++;
	}

	*ppArg = end;
	*pVal = (uint32_t)v;

	return true;
}

static int DutHexDigit(char c)
{
	if (c >= '0' && c <= '9')
	{
		return c - '0';
	}
	if (c >= 'a' && c <= 'f')
	{
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F')
	{
		return c - 'A' + 10;
	}

	return -1;
}

// Read an even length run of hex digits into pBuff. An odd count or a digit
// out of range fails the whole run, so a truncated line never lands as a
// shorter payload that a harness would then compare against.
static bool DutParseHex(const char *pArg, uint8_t *pBuff, size_t BuffLen,
	size_t *pLen)
{
	size_t len = 0;

	while (*pArg != 0 && *pArg != ' ')
	{
		int hi = DutHexDigit(pArg[0]);
		int lo = (pArg[1] != 0) ? DutHexDigit(pArg[1]) : -1;

		if (hi < 0 || lo < 0)
		{
			return false;
		}

		if (len >= BuffLen)
		{
			return false;
		}

		pBuff[len++] = (uint8_t)((hi << 4) | lo);
		pArg += 2;
	}

	*pLen = len;

	return true;
}

// ***
// Stack access
//

// The link the commands below act on. This peripheral holds one at a time, so
// there is nothing to choose between.
static uint16_t DutActiveConnHdl(void)
{
	BtDevice_t *pPeer = BtPeerGetActive();

	return (pPeer != nullptr) ? pPeer->Conn.Hdl : BT_CONN_HDL_INVALID;
}

// ***
// Command implementations
//

static void PrintStatus(void)
{
	uint8_t addrType = 0;
	uint8_t addr[6] = {};

	BtSmpLocalAddrGet(&addrType, addr);

	g_Uart.printf("DUT STATUS init=%d connected=%d adv_man_len=%u "
		"periodic=%d pawr=%d id_type=%u id=",
		BtInitialized() ? 1 : 0, BtConnected() ? 1 : 0,
		(unsigned)s_BleAppCfg.AdvManDataLen,
		BtAdvPeriodicIsRunning() ? 1 : 0, s_PawrReady ? 1 : 0,
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

static void PrintSecLevels(void)
{
	uint8_t value[BT_GAP_SEC_LEVEL_VALUE_MAX];
	size_t len = BtGapGetSecurityLevels(value, sizeof(value));

	if (len == 0 || len > sizeof(value))
	{
		g_Uart.printf("DUT ERROR sec_levels unavailable\r\n");
		return;
	}

	g_Uart.printf("DUT SEC_LEVELS count=%u value=", (unsigned)(len / 2));
	PrintHex(value, len);
	g_Uart.printf("\r\n");
}

// Rebuild advertising with whatever payload and coding selection are set now.
// Both are read at advertising init, so a change to either has to go back
// through init rather than through a data update on a running advertiser.
static bool DutAdvRebuild(void)
{
	BtAdvStop();

	if (BtAppAdvInit(&s_BleAppCfg) == false)
	{
		return false;
	}

	BtAdvStart();

	return true;
}

// Grow the manufacturer specific advertising payload to Len octets, keeping
// the marker at the front and filling the rest with a counting pattern a
// scanner can verify without being told the contents.
static void DutAdvExt(uint32_t Len)
{
	if (Len > DUT_ADV_EXT_MAX)
	{
		g_Uart.printf("DUT ERROR adv_ext len=%lu max=%u\r\n",
			(unsigned long)Len, (unsigned)DUT_ADV_EXT_MAX);
		return;
	}

	if (Len != 0 && Len < sizeof(s_AdvMarker))
	{
		g_Uart.printf("DUT ERROR adv_ext len=%lu min=%u\r\n",
			(unsigned long)Len, (unsigned)sizeof(s_AdvMarker));
		return;
	}

	// A tester walks down a ladder of sizes until one is accepted, so a size
	// the stack turns down has to leave the advertiser as it was rather than
	// pointing at a payload that is not on the air.
	const uint8_t *pPrevData = s_BleAppCfg.pAdvManData;
	int prevLen = s_BleAppCfg.AdvManDataLen;

	if (Len == 0)
	{
		s_BleAppCfg.pAdvManData = s_AdvMarker;
		s_BleAppCfg.AdvManDataLen = sizeof(s_AdvMarker);
	}
	else
	{
		memcpy(s_AdvExtData, s_AdvMarker, sizeof(s_AdvMarker));
		for (uint32_t i = sizeof(s_AdvMarker); i < Len; i++)
		{
			s_AdvExtData[i] = (uint8_t)i;
		}
		s_BleAppCfg.pAdvManData = s_AdvExtData;
		s_BleAppCfg.AdvManDataLen = (int)Len;
	}

	if (DutAdvRebuild() == false)
	{
		s_BleAppCfg.pAdvManData = pPrevData;
		s_BleAppCfg.AdvManDataLen = prevLen;
		DutAdvRebuild();

		g_Uart.printf("DUT ERROR adv_ext init_failed len=%lu kept=%d\r\n",
			(unsigned long)Len, prevLen);
		return;
	}

	g_Uart.printf("DUT ADV_EXT man_len=%u\r\n",
		(unsigned)s_BleAppCfg.AdvManDataLen);
}

static void DutCoding(uint32_t Prim, uint32_t Sec)
{
	uint8_t prevPrim = 0;
	uint8_t prevSec = 0;
	BtAdvCodingSelectionGet(&prevPrim, &prevSec);

	if (BtAdvCodingSelectionSet((uint8_t)Prim, (uint8_t)Sec) == false)
	{
		g_Uart.printf("DUT ERROR coding out_of_range prim=%lu sec=%lu\r\n",
			(unsigned long)Prim, (unsigned long)Sec);
		return;
	}

	// Same reason as the advertising payload: a coding the stack cannot bring
	// up leaves the advertiser on the coding it already had.
	if (DutAdvRebuild() == false)
	{
		BtAdvCodingSelectionSet(prevPrim, prevSec);
		DutAdvRebuild();

		g_Uart.printf("DUT ERROR coding init_failed kept_prim=%u kept_sec=%u\r\n",
			(unsigned)prevPrim, (unsigned)prevSec);
		return;
	}

	uint8_t prim = 0;
	uint8_t sec = 0;
	BtAdvCodingSelectionGet(&prim, &sec);

	g_Uart.printf("DUT CODING prim=%u sec=%u\r\n",
		(unsigned)prim, (unsigned)sec);
}

static void DutPhyRead(void)
{
	uint16_t hdl = DutActiveConnHdl();
	if (hdl == BT_CONN_HDL_INVALID)
	{
		g_Uart.printf("DUT ERROR phy no_link\r\n");
		return;
	}

	uint8_t tx = 0;
	uint8_t rx = 0;

	if (BtGapReadPhy(hdl, &tx, &rx) == false)
	{
		g_Uart.printf("DUT ERROR phy read_failed hdl=%u\r\n", (unsigned)hdl);
		return;
	}

	g_Uart.printf("DUT PHY hdl=%u tx=%u rx=%u\r\n",
		(unsigned)hdl, (unsigned)tx, (unsigned)rx);
}

// A PHY change is negotiated with the peer, so an accepted command means the
// request went out. The harness reads the result back with "phy read" once the
// peer has had time to answer.
static void DutPhySet(uint32_t Tx, uint32_t Rx, uint32_t Opt)
{
	uint16_t hdl = DutActiveConnHdl();
	if (hdl == BT_CONN_HDL_INVALID)
	{
		g_Uart.printf("DUT ERROR phy no_link\r\n");
		return;
	}

	if (BtGapSetPhy(hdl, (uint8_t)Tx, (uint8_t)Rx, (uint16_t)Opt) == false)
	{
		g_Uart.printf("DUT ERROR phy set_failed tx=%lu rx=%lu opt=%lu\r\n",
			(unsigned long)Tx, (unsigned long)Rx, (unsigned long)Opt);
		return;
	}

	g_Uart.printf("DUT PHY_SET hdl=%u tx=%lu rx=%lu opt=%lu\r\n",
		(unsigned)hdl, (unsigned long)Tx, (unsigned long)Rx,
		(unsigned long)Opt);
}

static void DutDataLen(uint32_t Octets, uint32_t Time)
{
	uint16_t hdl = DutActiveConnHdl();
	if (hdl == BT_CONN_HDL_INVALID)
	{
		g_Uart.printf("DUT ERROR datalen no_link\r\n");
		return;
	}

	if (BtGapSetDataLength(hdl, (uint16_t)Octets, (uint16_t)Time) == false)
	{
		g_Uart.printf("DUT ERROR datalen set_failed octets=%lu time=%lu\r\n",
			(unsigned long)Octets, (unsigned long)Time);
		return;
	}

	g_Uart.printf("DUT DATALEN hdl=%u octets=%lu time=%lu\r\n",
		(unsigned)hdl, (unsigned long)Octets, (unsigned long)Time);
}

static void DutNotify(const char *pArg)
{
	uint8_t data[DUT_GATT_VALUE_MAX];
	size_t len = 0;

	if (DutParseHex(pArg, data, sizeof(data), &len) == false || len == 0)
	{
		g_Uart.printf("DUT ERROR notify bad_hex\r\n");
		return;
	}

	memcpy(s_Value, data, len);
	BtGattCharSetValue(&g_DutChars[0], s_Value, (int)len);

	if (isBtGattCharNotifyEnabled(&g_DutChars[0]) == false)
	{
		g_Uart.printf("DUT ERROR notify not_enabled\r\n");
		return;
	}

	bool sent = BtAppNotify(&g_DutChars[0], s_Value, (uint16_t)len);

	g_Uart.printf("DUT GATT_NOTIFY sent=%d len=%u data=",
		sent ? 1 : 0, (unsigned)len);
	PrintHex(s_Value, len);
	g_Uart.printf("\r\n");
}

// ***
// Periodic advertising
//

static void DutPeriodicInit(void)
{
	BtAdvPeriodicCfg_t cfg = {};
	cfg.IntervalMin = DUT_PERIODIC_INTERVAL_MIN;
	cfg.IntervalMax = DUT_PERIODIC_INTERVAL_MAX;
	cfg.OwnAddrType = BTADDR_TYPE_RAND;
	cfg.Sid = 0;
	cfg.IncludeTxPower = true;

	if (BtAdvPeriodicInit(&cfg) == false)
	{
		g_Uart.printf("DUT ERROR padv init_failed\r\n");
		s_PeriodicReady = false;
		return;
	}

	// A fresh train replaces whatever a previous PAwR run configured, so the
	// subevent buffers stop being the ones a data request would answer from.
	s_PawrReady = false;
	s_PeriodicReady = true;

	g_Uart.printf("DUT PADV_INIT sid=%u interval_min=%u interval_max=%u\r\n",
		(unsigned)cfg.Sid, (unsigned)cfg.IntervalMin,
		(unsigned)cfg.IntervalMax);
}

static void DutPeriodicData(const char *pArg)
{
	uint8_t data[DUT_PERIODIC_DATA_MAX];
	size_t len = 0;

	if (DutParseHex(pArg, data, sizeof(data), &len) == false)
	{
		g_Uart.printf("DUT ERROR padv_data bad_hex max=%u\r\n",
			(unsigned)DUT_PERIODIC_DATA_MAX);
		return;
	}

	if (s_PeriodicReady == false)
	{
		g_Uart.printf("DUT ERROR padv_data not_initialised\r\n");
		return;
	}

	if (BtAdvPeriodicDataSet(data, len) == false)
	{
		g_Uart.printf("DUT ERROR padv_data set_failed len=%u\r\n",
			(unsigned)len);
		return;
	}

	memcpy(s_PeriodicData, data, len);
	s_PeriodicLen = (uint16_t)len;

	g_Uart.printf("DUT PADV_DATA len=%u data=", (unsigned)len);
	PrintHex(s_PeriodicData, s_PeriodicLen);
	g_Uart.printf("\r\n");
}

static void DutPeriodicStart(void)
{
	if (s_PeriodicReady == false)
	{
		g_Uart.printf("DUT ERROR padv not_initialised\r\n");
		return;
	}

	if (BtAdvPeriodicStart() == false)
	{
		g_Uart.printf("DUT ERROR padv start_failed\r\n");
		return;
	}

	g_Uart.printf("DUT PADV running=1\r\n");
}

static void DutPeriodicStop(void)
{
	if (BtAdvPeriodicStop() == false)
	{
		g_Uart.printf("DUT ERROR padv stop_failed\r\n");
		return;
	}

	g_Uart.printf("DUT PADV running=0\r\n");
}

// ***
// Periodic advertising with responses
//

static void DutPawrInit(void)
{
	BtAdvPawrCfg_t cfg = {};
	cfg.IntervalMin = DUT_PAWR_INTERVAL_MIN;
	cfg.IntervalMax = DUT_PAWR_INTERVAL_MAX;
	cfg.OwnAddrType = BTADDR_TYPE_RAND;
	cfg.Sid = 0;
	cfg.IncludeTxPower = true;
	cfg.NumSubevents = DUT_PAWR_SUBEVENT_CNT;
	cfg.SubeventInterval = DUT_PAWR_SUBEVT_INTERVAL;
	cfg.ResponseSlotDelay = DUT_PAWR_RSP_SLOT_DELAY;
	cfg.ResponseSlotSpacing = DUT_PAWR_RSP_SLOT_SPACING;
	cfg.NumResponseSlots = DUT_PAWR_RSP_SLOT_CNT;

	if (BtAdvPawrInit(&cfg) == false)
	{
		g_Uart.printf("DUT ERROR pawr init_failed\r\n");
		s_PawrReady = false;
		return;
	}

	// Give every subevent a payload that names itself, so a scanner reading
	// one subevent can tell which it landed in without being told.
	for (uint8_t i = 0; i < DUT_PAWR_SUBEVENT_CNT; i++)
	{
		s_PawrData[i][0] = 'S';
		s_PawrData[i][1] = i;
		s_PawrLen[i] = 2;
	}

	s_PeriodicReady = false;
	s_PawrReady = true;
	s_PawrReqTraced = false;

	g_Uart.printf("DUT PAWR_INIT subevents=%u subevt_interval=%u "
		"rsp_delay=%u rsp_spacing=%u rsp_slots=%u\r\n",
		(unsigned)cfg.NumSubevents, (unsigned)cfg.SubeventInterval,
		(unsigned)cfg.ResponseSlotDelay, (unsigned)cfg.ResponseSlotSpacing,
		(unsigned)cfg.NumResponseSlots);
}

static void DutPawrData(uint32_t Subevent, const char *pArg)
{
	uint8_t data[DUT_PAWR_SUBEVENT_DATA];
	size_t len = 0;

	if (Subevent >= DUT_PAWR_SUBEVENT_CNT)
	{
		g_Uart.printf("DUT ERROR pawr_data subevent=%lu max=%u\r\n",
			(unsigned long)Subevent, (unsigned)(DUT_PAWR_SUBEVENT_CNT - 1));
		return;
	}

	if (DutParseHex(pArg, data, sizeof(data), &len) == false)
	{
		g_Uart.printf("DUT ERROR pawr_data bad_hex max=%u\r\n",
			(unsigned)DUT_PAWR_SUBEVENT_DATA);
		return;
	}

	if (s_PawrReady == false)
	{
		g_Uart.printf("DUT ERROR pawr_data not_initialised\r\n");
		return;
	}

	memcpy(s_PawrData[Subevent], data, len);
	s_PawrLen[Subevent] = (uint8_t)len;

	g_Uart.printf("DUT PAWR_DATA subevent=%lu len=%u data=",
		(unsigned long)Subevent, (unsigned)len);
	PrintHex(s_PawrData[Subevent], s_PawrLen[Subevent]);
	g_Uart.printf("\r\n");
}

static void DutPawrStart(void)
{
	if (s_PawrReady == false)
	{
		g_Uart.printf("DUT ERROR pawr not_initialised\r\n");
		return;
	}

	if (BtAdvPeriodicStart() == false)
	{
		g_Uart.printf("DUT ERROR pawr start_failed\r\n");
		return;
	}

	g_Uart.printf("DUT PAWR running=1\r\n");
}

static void DutPawrStop(void)
{
	if (BtAdvPeriodicStop() == false)
	{
		g_Uart.printf("DUT ERROR pawr stop_failed\r\n");
		return;
	}

	g_Uart.printf("DUT PAWR running=0\r\n");
}

// ***
// Command dispatch
//

static void PrintHelp(void)
{
	g_Uart.printf("DUT COMMANDS status | sec levels | help\r\n");
	g_Uart.printf("DUT COMMANDS adv start | adv stop | adv ext <len> | "
		"coding <prim> <sec>\r\n");
	g_Uart.printf("DUT COMMANDS disconnect | bond clear | y | n\r\n");
	g_Uart.printf("DUT COMMANDS phy read | phy set <tx> <rx> [opt] | "
		"datalen <octets> <time> | notify <hex>\r\n");
	g_Uart.printf("DUT COMMANDS padv init | padv data <hex> | padv start | "
		"padv stop\r\n");
	g_Uart.printf("DUT COMMANDS pawr init | pawr data <subevent> <hex> | "
		"pawr start | pawr stop\r\n");
}

static void HandleCommand(const char *pLine)
{
	const char *arg = nullptr;
	uint32_t a = 0;
	uint32_t b = 0;
	uint32_t c = 0;

	if (DutMatch(pLine, "status", &arg) && *arg == 0)
	{
		PrintStatus();
		return;
	}

	if (DutMatch(pLine, "sec levels", &arg) && *arg == 0)
	{
		PrintSecLevels();
		return;
	}

	if (DutMatch(pLine, "adv start", &arg) && *arg == 0)
	{
		BtAdvStart();
		g_Uart.printf("DUT ADV_START requested\r\n");
		return;
	}

	if (DutMatch(pLine, "adv stop", &arg) && *arg == 0)
	{
		BtAdvStop();
		g_Uart.printf("DUT ADV_STOP requested\r\n");
		return;
	}

	if (DutMatch(pLine, "adv ext", &arg))
	{
		if (DutParseUint(&arg, &a) == false)
		{
			g_Uart.printf("DUT ERROR adv_ext missing_len\r\n");
			return;
		}
		DutAdvExt(a);
		return;
	}

	if (DutMatch(pLine, "coding", &arg))
	{
		if (DutParseUint(&arg, &a) == false ||
			DutParseUint(&arg, &b) == false)
		{
			g_Uart.printf("DUT ERROR coding missing_option\r\n");
			return;
		}
		DutCoding(a, b);
		return;
	}

	if (DutMatch(pLine, "disconnect", &arg) && *arg == 0)
	{
		int count = BtAppDisconnectAll();
		g_Uart.printf("DUT DISCONNECT requested=%d\r\n", count);
		return;
	}

	if (DutMatch(pLine, "bond clear", &arg) && *arg == 0)
	{
		BtSmpBondClearAll();
		g_Uart.printf("DUT BOND_CLEAR requested\r\n");
		return;
	}

	if (DutMatch(pLine, "phy read", &arg) && *arg == 0)
	{
		DutPhyRead();
		return;
	}

	if (DutMatch(pLine, "phy set", &arg))
	{
		if (DutParseUint(&arg, &a) == false ||
			DutParseUint(&arg, &b) == false)
		{
			g_Uart.printf("DUT ERROR phy missing_phy\r\n");
			return;
		}
		if (DutParseUint(&arg, &c) == false)
		{
			c = BT_GAP_PHY_CODED_ANY;
		}
		DutPhySet(a, b, c);
		return;
	}

	if (DutMatch(pLine, "datalen", &arg))
	{
		if (DutParseUint(&arg, &a) == false ||
			DutParseUint(&arg, &b) == false)
		{
			g_Uart.printf("DUT ERROR datalen missing_value\r\n");
			return;
		}
		DutDataLen(a, b);
		return;
	}

	if (DutMatch(pLine, "notify", &arg))
	{
		DutNotify(arg);
		return;
	}

	if (DutMatch(pLine, "padv init", &arg) && *arg == 0)
	{
		DutPeriodicInit();
		return;
	}

	if (DutMatch(pLine, "padv data", &arg))
	{
		DutPeriodicData(arg);
		return;
	}

	if (DutMatch(pLine, "padv start", &arg) && *arg == 0)
	{
		DutPeriodicStart();
		return;
	}

	if (DutMatch(pLine, "padv stop", &arg) && *arg == 0)
	{
		DutPeriodicStop();
		return;
	}

	if (DutMatch(pLine, "pawr init", &arg) && *arg == 0)
	{
		DutPawrInit();
		return;
	}

	if (DutMatch(pLine, "pawr data", &arg))
	{
		if (DutParseUint(&arg, &a) == false)
		{
			g_Uart.printf("DUT ERROR pawr_data missing_subevent\r\n");
			return;
		}
		DutPawrData(a, arg);
		return;
	}

	if (DutMatch(pLine, "pawr start", &arg) && *arg == 0)
	{
		DutPawrStart();
		return;
	}

	if (DutMatch(pLine, "pawr stop", &arg) && *arg == 0)
	{
		DutPawrStop();
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
	PrintHex(s_Value, (size_t)copyLen);
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

// The controller asks for the subevents it is about to advertise. Answering
// every one of them from the retained buffers keeps the train advertising the
// same payload each rotation, which is what lets the harness compare a
// subevent report against a value it set earlier.
void BtAdvPawrSubeventDataRequest(uint8_t AdvHandle, uint8_t SubeventStart,
	uint8_t SubeventDataCount)
{
	(void)AdvHandle;

	uint8_t filled = 0;

	for (uint8_t i = 0; i < SubeventDataCount; i++)
	{
		uint8_t subevent = (uint8_t)((SubeventStart + i) %
			DUT_PAWR_SUBEVENT_CNT);

		if (BtAdvPawrSubeventDataSet(subevent, 0, DUT_PAWR_RSP_SLOT_CNT,
			s_PawrData[subevent], s_PawrLen[subevent]))
		{
			filled++;
		}
	}

	// The controller asks again every periodic advertising event, so tracing
	// each one would fill the UART with lines that all say the same thing and
	// slow the very handler they describe. One line proves the train is
	// asking, and a later line means the answer stopped being complete.
	if (s_PawrReqTraced == false || filled != SubeventDataCount)
	{
		s_PawrReqTraced = true;
		g_Uart.printf("DUT PAWR_REQ start=%u count=%u filled=%u\r\n",
			(unsigned)SubeventStart, (unsigned)SubeventDataCount,
			(unsigned)filled);
	}
}

void BtAdvPawrResponse(uint8_t AdvHandle, uint8_t Subevent,
	uint8_t ResponseSlot, int8_t Rssi, size_t Len, const uint8_t *pData)
{
	(void)AdvHandle;

	g_Uart.printf("DUT PAWR_RSP subevent=%u slot=%u rssi=%d len=%u data=",
		(unsigned)Subevent, (unsigned)ResponseSlot, (int)Rssi,
		(unsigned)Len);
	PrintHex(pData, Len);
	g_Uart.printf("\r\n");
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
	PrintSecLevels();
	PrintHelp();
	g_Uart.printf("DUT READY\r\n");

	BtAppRun();

	g_Uart.printf("DUT ERROR bt_run_returned\r\n");
	while (true)
	{
	}
}

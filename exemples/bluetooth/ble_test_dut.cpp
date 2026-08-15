/**-------------------------------------------------------------------------
@example	ble_test_dut.cpp

@brief	Runtime test DUT for the IOsonata Bluetooth stack.

A Bluetooth LE device that puts the stack on the air so an external tester can
watch it run. This is a runtime test, not a unit test: the host and compliance
suites already check parsing, error codes and malformed input off target. What
cannot be checked there is whether a feature actually works on silicon, with a
radio, against a real peer. That is what this answers, feature by feature.

The application drives the stack through its ordinary public API and adds
nothing to it. Every command below names a GAP, GATT, SMP or advertising
procedure an application would use, and the command line interface belongs to
this file rather than to the library.

That interface is one line based command language over a DeviceIntrf, and two
interfaces carry it. The UART reaches the DUT whether or not anything is
connected, which is what a reset, an advertising change or the start of a
periodic advertising train needs. The BlueIO UART service takes the same
language over the air through BtIntrf, so a tester holding a connection drives
the DUT without a second cable to it.

Nothing is added to the attribute table to make that work. The service is the
one uart_ble.cpp exposes, a byte pipe that does not care what the bytes mean,
so the table under test is the table a product would ship.

Every line the DUT prints starts with "DUT " and a tag, so a harness matches on
the tag and parses the key=value fields that follow. A command that cannot run
answers with a single DUT ERROR line naming the reason, never with silence, so
a harness can tell a refusal from a lost line. Reports always reach the UART,
and reach a connected client as well, so the UART holds a complete trace
whichever interface asked.

The over-the-air side is driven from a PC through a Bluetooth dongle that owns
the peer side of every exchange these commands set up: discovery, connection,
ATT, PHY, data length, periodic advertising synchronisation and PAwR responses.
The IOsonata HciController firmware is one such dongle.

The DUT is an advertiser and a peripheral. It does not scan, so it is never the
synchronising side of a periodic advertising train; the dongle is. The periodic
advertising and PAwR commands therefore configure and run trains for the dongle
to synchronise with, and the response reports the dongle produces come back
through the response report callback.

A procedure the stack cannot bring up on this target is refused at the command
that asks for it rather than removed from the build, so one binary answers the
same set of commands everywhere and the tester learns what is available by
asking for it. Nothing here reports the local controller's HCI capability
bitmaps; that would describe the silicon under the stack rather than the stack
being tested, and it would let a tester skip a procedure the stack should have
been asked to perform.

@author	Hoang Nguyen Hoan
@date	Aug. 15, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "istddef.h"
#include "idelay.h"
#include "app_evt_handler.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "syslog.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_ead.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_intrf.h"
#include "bluetooth/bt_padv.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_psync.h"
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

#define DUT_UART_FIFO_SIZE		CFIFO_MEMSIZE(256)

/// Longest command line either interface accepts.
#define DUT_CMD_LINE_MAX		160

/// Longest report line. Enough for the periodic advertising data echo, which
/// is the longest thing any command below prints, at two hex digits an octet.
#define DUT_OUT_LINE_MAX		200

// The advertising set the stack configures. bt_adv_hci.cpp builds handle 0 and
// no other, and the periodic advertising train attaches to that set, so this
// is what every periodic advertising command names.
#define DUT_ADV_HDL				0

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

// Session key and initialization vector of an Encrypted Advertising Data key,
// given to the "ead key" command as one run of hex digits.
#define DUT_EAD_KEY_HEX_LEN		((BTEAD_KEY_LEN + BTEAD_IV_LEN) * 2)

// Advertising modes the "adv mode" command selects between.
#define DUT_ADV_MODE_CONN		0	//!< Connectable, the boot mode
#define DUT_ADV_MODE_SCAN		1	//!< Scannable, not connectable
#define DUT_ADV_MODE_NONCONN	2	//!< Neither, a plain broadcaster

// The command and report vocabulary this build speaks, in the boot banner and
// in "status". A harness checks it before running anything: a DUT older than
// the harness answers ERROR unknown_command to a command that was added later,
// which reads as a stack defect rather than as a version mismatch. Raised
// whenever a command is added or a report changes shape.
#define DUT_PROTOCOL_VERSION	1

// Scan response payload the scannable mode advertises. Its only job is to make
// the set scannable, and a scanner that receives it has proof the scan request
// and response exchange happened rather than a single advertising PDU.
static const uint8_t s_ScanRspMarker[] = { 'D', 'U', 'T', 'S' };

// Stable marker used by the dongle side test to identify the DUT by
// advertising contents rather than by an address that rotates. The "adv ext"
// command keeps these four octets at the front of the enlarged payload so the
// same match works on a legacy and on an extended advertising report.
//
// With Encrypted Advertising Data armed the marker is inside the ciphertext,
// so only a scanner holding the key can match on it. That is the point of the
// feature, and a harness testing it identifies the DUT by the address it was
// told instead.
static const uint8_t s_AdvMarker[] = { 'D', 'U', 'T', 1 };

// ***
// The command interface
//
// One language over two DeviceIntrf. The UART is always there; the BlueIO UART
// service appears when a client connects. BtIntrf turns that service into a
// DeviceIntrf, so neither the parser nor the report path knows which one it is
// talking to.
//

static uint8_t s_UartRxFifo[DUT_UART_FIFO_SIZE];
static uint8_t s_UartTxFifo[DUT_UART_FIFO_SIZE];
static IOPinCfg_t s_UartPins[] = UART_PINS;

static int UartEvtHandler(UARTDev_t *pDev, UART_EVT EvtId,
	uint8_t *pBuffer, int BufferLen);
static int DutBleIntrfEvtHandler(DevIntrf_t *pDev, DEVINTRF_EVT EvtId,
	uint8_t *pBuffer, int BufferLen);
static void DutUartHandler(uint32_t Evt, void *pCtx);
static void DutBleHandler(uint32_t Evt, void *pCtx);
static void HandleCommand(const char *pLine);

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

// The BlueIO UART service, exactly as uart_ble.cpp exposes it. The receive
// characteristic notifies the client and the transmit characteristic takes
// what the client writes, which is the naming this service has always used;
// BtIntrf is handed the indexes rather than the names.
static const char s_RxCharDesc[] = "DUT report";
static const char s_TxCharDesc[] = "DUT command";

static uint8_t s_RxCharValue[DUT_OUT_LINE_MAX];
static uint8_t s_TxCharValue[DUT_CMD_LINE_MAX];

#define DUT_CHAR_IDX_RX			0	//!< Reports out, notified
#define DUT_CHAR_IDX_TX			1	//!< Commands in, written

BtGattChar_t g_DutChars[] = {
	BT_CHAR(BLUEIO_UUID_UART_RX_CHAR, DUT_OUT_LINE_MAX,
		BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
		s_RxCharDesc),
	BT_CHAR(BLUEIO_UUID_UART_TX_CHAR, DUT_CMD_LINE_MAX,
		BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_WRITE_WORESP,
		s_TxCharDesc),
};

BtGattSrvc_t g_DutService = BT_SRVC_CUSTOM(BLUEIO_UUID_BASE,
	BLUEIO_UUID_UART_SERVICE, g_DutChars);

#define DUT_INTRF_PACKET_MAX	64
#define DUT_INTRF_FIFO_MEMSIZE	\
		BTINTRF_CFIFO_TOTAL_MEMSIZE(8, DUT_INTRF_PACKET_MAX)

alignas(4) static uint8_t s_BtIntrfRxFifo[DUT_INTRF_FIFO_MEMSIZE];
alignas(4) static uint8_t s_BtIntrfTxFifo[DUT_INTRF_FIFO_MEMSIZE];

static const BtIntrfCfg_t s_BtIntrfCfg = {
	.pSrvc = &g_DutService,
	.RxCharIdx = DUT_CHAR_IDX_TX,
	.TxCharIdx = DUT_CHAR_IDX_RX,
	.PacketSize = 0,
	// Dropping a report is better than stalling the stack that produced it.
	.bBlocking = false,
	.RxFifoMemSize = DUT_INTRF_FIFO_MEMSIZE,
	.pRxFifoMem = s_BtIntrfRxFifo,
	.TxFifoMemSize = DUT_INTRF_FIFO_MEMSIZE,
	.pTxFifoMem = s_BtIntrfTxFifo,
	.EvtCB = DutBleIntrfEvtHandler,
};

static BtIntrf g_BtIntrf;
static bool s_BtIntrfReady = false;

static const BtUuidArr_t s_AdvUuid = {
	.BaseIdx = 1,
	.Type = BT_UUID_TYPE_16,
	.Count = 1,
	.Uuid16 = { BLUEIO_UUID_UART_SERVICE, }
};

static const BtAppDevInfo_t s_DevInfo = {
	DUT_MODEL_NAME,
	DUT_MANUFACTURER_NAME,
	"001",
	"0.1",
	"0.1",
};

static uint8_t s_AdvExtData[DUT_ADV_EXT_MAX];
static uint8_t s_LongWriteMem[512];

// Advertising configuration is rebuilt whenever the harness changes the
// advertising payload size, the advertising mode or the coding selection, all
// of which are read at advertising init. Everything else stays as the boot
// value.
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

// The advertising coding selection in force. The stack takes both options at
// advertising init and keeps no reader for them, so the values a harness set
// are held here to be reported back and to be restored when a rebuild fails.
static uint8_t s_CodingPrim = BTADV_PHY_OPT_NONE;
static uint8_t s_CodingSec = BTADV_PHY_OPT_NONE;
static bool s_CodingFeature = false;

// Advertising mode the "adv mode" command last selected. The role decides
// whether the set is connectable and the scan response data decides whether it
// is scannable, so the mode is held here rather than read back from the two.
static uint8_t s_AdvMode = DUT_ADV_MODE_CONN;

// ***
// Report lines
//
// A report is built whole and emitted once. Building it first is what lets one
// report reach both interfaces, and what makes a notification carry a line
// rather than whatever fragment a print call happened to produce.
//

static char s_OutLine[DUT_OUT_LINE_MAX];
static int s_OutLen = 0;

static void DutOut(const char *pFmt, ...)
{
	if (s_OutLen >= (int)sizeof(s_OutLine) - 1)
	{
		return;
	}

	va_list args;
	va_start(args, pFmt);
	int n = vsnprintf(&s_OutLine[s_OutLen], sizeof(s_OutLine) - s_OutLen,
					  pFmt, args);
	va_end(args);

	if (n < 0)
	{
		return;
	}

	// vsnprintf answers what it would have written, so a truncated line has to
	// stop at the buffer rather than at that number.
	s_OutLen += n;
	if (s_OutLen > (int)sizeof(s_OutLine) - 1)
	{
		s_OutLen = (int)sizeof(s_OutLine) - 1;
	}
}

static void DutOutHex(const uint8_t *pData, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		DutOut("%02X", pData[i]);
	}
}

static void DutOutAddr(const uint8_t Addr[6])
{
	for (int i = 5; i >= 0; i--)
	{
		DutOut("%02X", Addr[i]);
		if (i != 0)
		{
			DutOut(":");
		}
	}
}

static void DutOutEnd(void)
{
	s_OutLine[s_OutLen] = '\n';
	int len = s_OutLen + 1;

	g_Uart.Tx((uint8_t*)s_OutLine, len);

	// BtIntrf holds the line in its own FIFO and notifies it in whatever the
	// negotiated ATT_MTU allows, so nothing here has to know the MTU. A client
	// that has not subscribed simply never sees it, and the UART copy above is
	// the trace either way.
	if (s_BtIntrfReady && BtConnected())
	{
		g_BtIntrf.Tx(0, (uint8_t*)s_OutLine, len);
	}

	s_OutLen = 0;
	s_OutLine[0] = '\0';
}

// ***
// Command lines
//
// One assembler per interface, because a partial line arriving on the UART and
// a partial line arriving over the air are two different lines.
//

typedef struct __Dut_Line {
	char Buf[DUT_CMD_LINE_MAX];
	size_t Len;
} DutLine_t;

static DutLine_t s_UartLine;
static DutLine_t s_BleLine;

static void DutLineFeed(DutLine_t *pLine, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++)
	{
		char c = (char)pData[i];

		if (c == '\r' || c == '\n')
		{
			if (pLine->Len != 0)
			{
				pLine->Buf[pLine->Len] = '\0';
				pLine->Len = 0;
				HandleCommand(pLine->Buf);
			}
			continue;
		}

		if (pLine->Len + 1 < sizeof(pLine->Buf))
		{
			pLine->Buf[pLine->Len++] = c;
		}
		else
		{
			pLine->Len = 0;
			DutOut("DUT ERROR command_too_long");
			DutOutEnd();
		}
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

	DutOut("DUT STATUS proto=%u init=%d connected=%d adv_mode=%u "
		"adv_man_len=%u periodic=%d pawr=%d ead=%d id_type=%u id=",
		(unsigned)DUT_PROTOCOL_VERSION,
		BtInitialized() ? 1 : 0, BtConnected() ? 1 : 0,
		(unsigned)s_AdvMode, (unsigned)s_BleAppCfg.AdvManDataLen,
		BtPadvIsEnabled(DUT_ADV_HDL) ? 1 : 0, s_PawrReady ? 1 : 0,
		BtAdvEadIsArmed() ? 1 : 0, (unsigned)addrType);
	DutOutAddr(addr);
	DutOutEnd();
}

static void PrintLink(uint16_t ConnHdl)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		DutOut("DUT LINK hdl=%u unavailable", (unsigned)ConnHdl);
		DutOutEnd();
		return;
	}

	DutOut("DUT LINK hdl=%u role=%u mtu=%u peer_type=%u peer=",
		(unsigned)ConnHdl, (unsigned)pPeer->Conn.Role,
		(unsigned)pPeer->Conn.MaxMtu, (unsigned)pPeer->Conn.PeerAddrType);
	DutOutAddr(pPeer->Conn.PeerAddr);
	DutOut(" own_type=%u own=", (unsigned)pPeer->Conn.OwnAddrType);
	DutOutAddr(pPeer->Conn.OwnAddr);
	DutOut(" sec_level=%u key_size=%u sec_flags=0x%02X bonded=%d",
		(unsigned)pPeer->Conn.Sec.Level, (unsigned)pPeer->Conn.Sec.KeySize,
		(unsigned)pPeer->Conn.Sec.Flags, BtSmpBonded(ConnHdl) ? 1 : 0);
	DutOutEnd();
}

// Rebuild advertising with whatever payload, mode, coding selection and
// encryption are set now. All four are read at advertising init, so a change to
// any of them has to go back through init rather than through a data update on
// a running advertiser.
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
		DutOut("DUT ERROR adv_ext len=%lu max=%u",
			(unsigned long)Len, (unsigned)DUT_ADV_EXT_MAX);
		DutOutEnd();
		return;
	}

	if (Len != 0 && Len < sizeof(s_AdvMarker))
	{
		DutOut("DUT ERROR adv_ext len=%lu min=%u",
			(unsigned long)Len, (unsigned)sizeof(s_AdvMarker));
		DutOutEnd();
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

		DutOut("DUT ERROR adv_ext init_failed len=%lu kept=%d",
			(unsigned long)Len, prevLen);
		DutOutEnd();
		return;
	}

	DutOut("DUT ADV_EXT man_len=%u", (unsigned)s_BleAppCfg.AdvManDataLen);
	DutOutEnd();
}

// Connectable, scannable or neither. The role decides the first and the
// presence of scan response data decides the second, so both move together
// here and the advertiser is rebuilt, since an advertising set cannot change
// its event properties while it is running.
static void DutAdvMode(const char *pArg)
{
	uint8_t mode;

	if (DutMatch(pArg, "conn", nullptr))
	{
		mode = DUT_ADV_MODE_CONN;
	}
	else if (DutMatch(pArg, "scan", nullptr))
	{
		mode = DUT_ADV_MODE_SCAN;
	}
	else if (DutMatch(pArg, "nonconn", nullptr))
	{
		mode = DUT_ADV_MODE_NONCONN;
	}
	else
	{
		DutOut("DUT ERROR adv_mode bad_mode");
		DutOutEnd();
		return;
	}

	uint8_t prevMode = s_AdvMode;
	BTAPP_ROLE prevRole = s_BleAppCfg.Role;
	const uint8_t *pPrevSr = s_BleAppCfg.pSrManData;
	int prevSrLen = s_BleAppCfg.SrManDataLen;

	s_AdvMode = mode;
	s_BleAppCfg.Role = (mode == DUT_ADV_MODE_CONN) ?
					   BTAPP_ROLE_PERIPHERAL : BTAPP_ROLE_BROADCASTER;
	s_BleAppCfg.pSrManData = (mode == DUT_ADV_MODE_SCAN) ?
							 s_ScanRspMarker : nullptr;
	s_BleAppCfg.SrManDataLen = (mode == DUT_ADV_MODE_SCAN) ?
							   (int)sizeof(s_ScanRspMarker) : 0;

	if (DutAdvRebuild() == false)
	{
		s_AdvMode = prevMode;
		s_BleAppCfg.Role = prevRole;
		s_BleAppCfg.pSrManData = pPrevSr;
		s_BleAppCfg.SrManDataLen = prevSrLen;
		DutAdvRebuild();

		DutOut("DUT ERROR adv_mode init_failed kept=%u", (unsigned)prevMode);
		DutOutEnd();
		return;
	}

	DutOut("DUT ADV_MODE mode=%u connectable=%d scannable=%d",
		(unsigned)s_AdvMode, (mode == DUT_ADV_MODE_CONN) ? 1 : 0,
		(mode == DUT_ADV_MODE_SCAN) ? 1 : 0);
	DutOutEnd();
}

// Advertising Coding Selection, Core 5.4 Vol 1 Part C 13.1. The host feature
// bit has to be claimed before a coding means anything, and the controller
// refuses to change it while it holds a connection, so the bit is claimed at
// boot and only the two options move from here.
static void DutCoding(uint32_t Prim, uint32_t Sec)
{
	uint8_t prevPrim = s_CodingPrim;
	uint8_t prevSec = s_CodingSec;

	if (BtAdvCodingSet((uint8_t)Prim, (uint8_t)Sec) == false)
	{
		DutOut("DUT ERROR coding out_of_range prim=%lu sec=%lu",
			(unsigned long)Prim, (unsigned long)Sec);
		DutOutEnd();
		return;
	}

	s_CodingPrim = (uint8_t)Prim;
	s_CodingSec = (uint8_t)Sec;

	// Same reason as the advertising payload: a coding the stack cannot bring
	// up leaves the advertiser on the coding it already had.
	if (DutAdvRebuild() == false)
	{
		s_CodingPrim = prevPrim;
		s_CodingSec = prevSec;
		BtAdvCodingSet(prevPrim, prevSec);
		DutAdvRebuild();

		DutOut("DUT ERROR coding init_failed kept_prim=%u kept_sec=%u",
			(unsigned)prevPrim, (unsigned)prevSec);
		DutOutEnd();
		return;
	}

	DutOut("DUT CODING prim=%u sec=%u feature=%d",
		(unsigned)s_CodingPrim, (unsigned)s_CodingSec,
		s_CodingFeature ? 1 : 0);
	DutOutEnd();
}

// Install Encrypted Advertising Data key material and rebuild advertising so
// the next packet on air is encrypted. The argument is the session key followed
// by the initialization vector, in the order Core 5.4 Vol 3 Part C 12.6 Table
// 12.10 lists them and the order the Core Specification Supplement prints its
// sample data.
static void DutEadKey(const char *pArg)
{
	uint8_t raw[BTEAD_KEY_LEN + BTEAD_IV_LEN];
	size_t len = 0;

	if (DutParseHex(pArg, raw, sizeof(raw), &len) == false ||
		len != sizeof(raw))
	{
		DutOut("DUT ERROR ead_key bad_hex digits=%u",
			(unsigned)DUT_EAD_KEY_HEX_LEN);
		DutOutEnd();
		return;
	}

	BtEadKey_t key;
	memcpy(key.SessionKey, raw, BTEAD_KEY_LEN);

	// The IV goes into the CCM nonce least significant octet first, which is
	// the reverse of how it is written. The session key is not reversed.
	for (size_t i = 0; i < BTEAD_IV_LEN; i++)
	{
		key.Iv[i] = raw[BTEAD_KEY_LEN + BTEAD_IV_LEN - 1 - i];
	}

	if (BtAdvEadKeySet(&key) == false)
	{
		DutOut("DUT ERROR ead_key rejected");
		DutOutEnd();
		return;
	}

	if (DutAdvRebuild() == false)
	{
		// The most likely cause is a missing engine or a payload that no
		// longer fits once the randomizer and MIC are added. Either way the
		// advertiser has to come back up unencrypted rather than stay dark.
		BtAdvEadKeySet(nullptr);
		DutAdvRebuild();

		DutOut("DUT ERROR ead_key init_failed disarmed=1");
		DutOutEnd();
		return;
	}

	DutOut("DUT EAD armed=1 overhead=%u", (unsigned)(BTEAD_OVERHEAD + 2));
	DutOutEnd();
}

static void DutEadOff(void)
{
	BtAdvEadKeySet(nullptr);

	if (DutAdvRebuild() == false)
	{
		DutOut("DUT ERROR ead_off init_failed");
		DutOutEnd();
		return;
	}

	DutOut("DUT EAD armed=0");
	DutOutEnd();
}

// Known answer test against the sample data of the Core Specification
// Supplement Part A 2.3. A tester runs this before trusting anything the DUT
// puts on air encrypted: a byte order fault shows up here rather than as a
// dongle that cannot read the advertising data.
static void DutEadSelfTest(void)
{
	int res = BtEadSelfTest();

	if (res != 0)
	{
		DutOut("DUT ERROR ead_selftest res=%d", res);
		DutOutEnd();
		return;
	}

	DutOut("DUT EAD_SELFTEST pass=1");
	DutOutEnd();
}

// Restart the board so a run starts from a known state. A harness that has to
// power cycle by hand between runs is not automated, and reaching the same
// state by sending the commands that undo the previous run tests the undo path
// rather than the run that follows it.
//
// The line goes out before the reset and the transmit path is given time to
// drain, because a harness that sees nothing cannot tell a board that reset
// from one that hung.
static void DutReset(void)
{
	DutOut("DUT RESET requested");
	DutOutEnd();
	msDelay(50);

	NVIC_SystemReset();
}

// Peripheral initiated security, Core 5.4 Vol 3 Part H 2.4.6. The peripheral
// asks and the central decides what to do about it, so an accepted command
// means the request went out and nothing more. What follows is either pairing
// or encryption with a stored key, and both arrive as their own reports.
static void DutSecurityRequest(void)
{
	uint16_t hdl = DutActiveConnHdl();
	if (hdl == BT_CONN_HDL_INVALID)
	{
		DutOut("DUT ERROR sec_request no_link");
		DutOutEnd();
		return;
	}

	BtSmpRequestSecurity(hdl);

	DutOut("DUT SEC_REQUEST hdl=%u", (unsigned)hdl);
	DutOutEnd();
}

static void DutPhyRead(void)
{
	uint16_t hdl = DutActiveConnHdl();
	if (hdl == BT_CONN_HDL_INVALID)
	{
		DutOut("DUT ERROR phy no_link");
		DutOutEnd();
		return;
	}

	uint8_t tx = 0;
	uint8_t rx = 0;

	if (BtGapReadPhy(hdl, &tx, &rx) == false)
	{
		DutOut("DUT ERROR phy read_failed hdl=%u", (unsigned)hdl);
		DutOutEnd();
		return;
	}

	DutOut("DUT PHY hdl=%u tx=%u rx=%u",
		(unsigned)hdl, (unsigned)tx, (unsigned)rx);
	DutOutEnd();
}

// A PHY change is negotiated with the peer, so an accepted command means the
// request went out. The harness reads the result back with "phy read" once the
// peer has had time to answer.
static void DutPhySet(uint32_t Tx, uint32_t Rx, uint32_t Opt)
{
	uint16_t hdl = DutActiveConnHdl();
	if (hdl == BT_CONN_HDL_INVALID)
	{
		DutOut("DUT ERROR phy no_link");
		DutOutEnd();
		return;
	}

	if (BtGapSetPhy(hdl, (uint8_t)Tx, (uint8_t)Rx, (uint16_t)Opt) == false)
	{
		DutOut("DUT ERROR phy set_failed tx=%lu rx=%lu opt=%lu",
			(unsigned long)Tx, (unsigned long)Rx, (unsigned long)Opt);
		DutOutEnd();
		return;
	}

	DutOut("DUT PHY_SET hdl=%u tx=%lu rx=%lu opt=%lu",
		(unsigned)hdl, (unsigned long)Tx, (unsigned long)Rx,
		(unsigned long)Opt);
	DutOutEnd();
}

static void DutDataLen(uint32_t Octets, uint32_t Time)
{
	uint16_t hdl = DutActiveConnHdl();
	if (hdl == BT_CONN_HDL_INVALID)
	{
		DutOut("DUT ERROR datalen no_link");
		DutOutEnd();
		return;
	}

	if (BtGapSetDataLength(hdl, (uint16_t)Octets, (uint16_t)Time) == false)
	{
		DutOut("DUT ERROR datalen set_failed octets=%lu time=%lu",
			(unsigned long)Octets, (unsigned long)Time);
		DutOutEnd();
		return;
	}

	DutOut("DUT DATALEN hdl=%u octets=%lu time=%lu",
		(unsigned)hdl, (unsigned long)Octets, (unsigned long)Time);
	DutOutEnd();
}

// ***
// Periodic advertising
//

static void DutPeriodicInit(void)
{
	BtPadvCfg_t cfg = {};
	cfg.AdvHdl = DUT_ADV_HDL;
	cfg.IntervalMin = DUT_PERIODIC_INTERVAL_MIN;
	cfg.IntervalMax = DUT_PERIODIC_INTERVAL_MAX;
	cfg.Properties = BTPADV_PROP_TXPWR;
	cfg.NbSubevents = 0;

	if (BtPadvInit(&cfg) == false)
	{
		DutOut("DUT ERROR padv init_failed");
		DutOutEnd();
		s_PeriodicReady = false;
		return;
	}

	// A fresh train replaces whatever a previous PAwR run configured, so the
	// subevent buffers stop being the ones a data request would answer from.
	s_PawrReady = false;
	s_PeriodicReady = true;

	DutOut("DUT PADV_INIT hdl=%u interval_min=%u interval_max=%u",
		(unsigned)cfg.AdvHdl, (unsigned)cfg.IntervalMin,
		(unsigned)cfg.IntervalMax);
	DutOutEnd();
}

static void DutPeriodicData(const char *pArg)
{
	uint8_t data[DUT_PERIODIC_DATA_MAX];
	size_t len = 0;

	if (DutParseHex(pArg, data, sizeof(data), &len) == false)
	{
		DutOut("DUT ERROR padv_data bad_hex max=%u",
			(unsigned)DUT_PERIODIC_DATA_MAX);
		DutOutEnd();
		return;
	}

	if (s_PeriodicReady == false)
	{
		DutOut("DUT ERROR padv_data not_initialised");
		DutOutEnd();
		return;
	}

	if (BtPadvDataSet(DUT_ADV_HDL, data, len) == false)
	{
		DutOut("DUT ERROR padv_data set_failed len=%u", (unsigned)len);
		DutOutEnd();
		return;
	}

	memcpy(s_PeriodicData, data, len);
	s_PeriodicLen = (uint16_t)len;

	DutOut("DUT PADV_DATA len=%u data=", (unsigned)len);
	DutOutHex(s_PeriodicData, s_PeriodicLen);
	DutOutEnd();
}

static void DutPeriodicStart(void)
{
	if (s_PeriodicReady == false)
	{
		DutOut("DUT ERROR padv not_initialised");
		DutOutEnd();
		return;
	}

	if (BtPadvStart(DUT_ADV_HDL) == false)
	{
		DutOut("DUT ERROR padv start_failed");
		DutOutEnd();
		return;
	}

	DutOut("DUT PADV running=1");
	DutOutEnd();
}

static void DutPeriodicStop(void)
{
	if (BtPadvStop(DUT_ADV_HDL) == false)
	{
		DutOut("DUT ERROR padv stop_failed");
		DutOutEnd();
		return;
	}

	DutOut("DUT PADV running=0");
	DutOutEnd();
}

// ***
// Periodic advertising with responses
//

static void DutPawrInit(void)
{
	BtPadvCfg_t cfg = {};
	cfg.AdvHdl = DUT_ADV_HDL;
	cfg.IntervalMin = DUT_PAWR_INTERVAL_MIN;
	cfg.IntervalMax = DUT_PAWR_INTERVAL_MAX;
	cfg.Properties = BTPADV_PROP_TXPWR;
	cfg.NbSubevents = DUT_PAWR_SUBEVENT_CNT;
	cfg.SubeventInterval = DUT_PAWR_SUBEVT_INTERVAL;
	cfg.RspSlotDelay = DUT_PAWR_RSP_SLOT_DELAY;
	cfg.RspSlotSpacing = DUT_PAWR_RSP_SLOT_SPACING;
	cfg.NbRspSlots = DUT_PAWR_RSP_SLOT_CNT;

	if (BtPadvInit(&cfg) == false)
	{
		DutOut("DUT ERROR pawr init_failed");
		DutOutEnd();
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

	DutOut("DUT PAWR_INIT subevents=%u subevt_interval=%u rsp_delay=%u "
		"rsp_spacing=%u rsp_slots=%u",
		(unsigned)cfg.NbSubevents, (unsigned)cfg.SubeventInterval,
		(unsigned)cfg.RspSlotDelay, (unsigned)cfg.RspSlotSpacing,
		(unsigned)cfg.NbRspSlots);
	DutOutEnd();
}

static void DutPawrData(uint32_t Subevent, const char *pArg)
{
	uint8_t data[DUT_PAWR_SUBEVENT_DATA];
	size_t len = 0;

	if (Subevent >= DUT_PAWR_SUBEVENT_CNT)
	{
		DutOut("DUT ERROR pawr_data subevent=%lu max=%u",
			(unsigned long)Subevent, (unsigned)(DUT_PAWR_SUBEVENT_CNT - 1));
		DutOutEnd();
		return;
	}

	if (DutParseHex(pArg, data, sizeof(data), &len) == false)
	{
		DutOut("DUT ERROR pawr_data bad_hex max=%u",
			(unsigned)DUT_PAWR_SUBEVENT_DATA);
		DutOutEnd();
		return;
	}

	if (s_PawrReady == false)
	{
		DutOut("DUT ERROR pawr_data not_initialised");
		DutOutEnd();
		return;
	}

	memcpy(s_PawrData[Subevent], data, len);
	s_PawrLen[Subevent] = (uint8_t)len;

	DutOut("DUT PAWR_DATA subevent=%lu len=%u data=",
		(unsigned long)Subevent, (unsigned)len);
	DutOutHex(s_PawrData[Subevent], s_PawrLen[Subevent]);
	DutOutEnd();
}

static void DutPawrStart(void)
{
	if (s_PawrReady == false)
	{
		DutOut("DUT ERROR pawr not_initialised");
		DutOutEnd();
		return;
	}

	if (BtPadvStart(DUT_ADV_HDL) == false)
	{
		DutOut("DUT ERROR pawr start_failed");
		DutOutEnd();
		return;
	}

	DutOut("DUT PAWR running=1");
	DutOutEnd();
}

static void DutPawrStop(void)
{
	if (BtPadvStop(DUT_ADV_HDL) == false)
	{
		DutOut("DUT ERROR pawr stop_failed");
		DutOutEnd();
		return;
	}

	DutOut("DUT PAWR running=0");
	DutOutEnd();
}

// ***
// Command dispatch
//

static void PrintHelp(void)
{
	DutOut("DUT COMMANDS status | link | reset | help");
	DutOutEnd();
	DutOut("DUT COMMANDS adv start | adv stop | adv ext <len> | "
		"adv mode <conn|scan|nonconn> | coding <prim> <sec>");
	DutOutEnd();
	DutOut("DUT COMMANDS disconnect | bond clear | sec request | y | n");
	DutOutEnd();
	DutOut("DUT COMMANDS phy read | phy set <tx> <rx> [opt] | "
		"datalen <octets> <time>");
	DutOutEnd();
	DutOut("DUT COMMANDS padv init | padv data <hex> | padv start | padv stop");
	DutOutEnd();
	DutOut("DUT COMMANDS pawr init | pawr data <subevent> <hex> | "
		"pawr start | pawr stop");
	DutOutEnd();
	DutOut("DUT COMMANDS ead key <%u hex digits> | ead off | ead selftest",
		(unsigned)DUT_EAD_KEY_HEX_LEN);
	DutOutEnd();
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

	if (DutMatch(pLine, "link", &arg) && *arg == 0)
	{
		uint16_t hdl = DutActiveConnHdl();
		if (hdl == BT_CONN_HDL_INVALID)
		{
			DutOut("DUT ERROR link no_link");
			DutOutEnd();
			return;
		}
		PrintLink(hdl);
		return;
	}

	if (DutMatch(pLine, "sec request", &arg) && *arg == 0)
	{
		DutSecurityRequest();
		return;
	}

	if (DutMatch(pLine, "reset", &arg) && *arg == 0)
	{
		DutReset();
		return;
	}

	if (DutMatch(pLine, "adv start", &arg) && *arg == 0)
	{
		BtAdvStart();
		DutOut("DUT ADV_START requested");
		DutOutEnd();
		return;
	}

	if (DutMatch(pLine, "adv stop", &arg) && *arg == 0)
	{
		BtAdvStop();
		DutOut("DUT ADV_STOP requested");
		DutOutEnd();
		return;
	}

	if (DutMatch(pLine, "adv ext", &arg))
	{
		if (DutParseUint(&arg, &a) == false)
		{
			DutOut("DUT ERROR adv_ext missing_len");
			DutOutEnd();
			return;
		}
		DutAdvExt(a);
		return;
	}

	if (DutMatch(pLine, "adv mode", &arg))
	{
		DutAdvMode(arg);
		return;
	}

	if (DutMatch(pLine, "coding", &arg))
	{
		if (DutParseUint(&arg, &a) == false ||
			DutParseUint(&arg, &b) == false)
		{
			DutOut("DUT ERROR coding missing_option");
			DutOutEnd();
			return;
		}
		DutCoding(a, b);
		return;
	}

	if (DutMatch(pLine, "ead key", &arg))
	{
		DutEadKey(arg);
		return;
	}

	if (DutMatch(pLine, "ead off", &arg) && *arg == 0)
	{
		DutEadOff();
		return;
	}

	if (DutMatch(pLine, "ead selftest", &arg) && *arg == 0)
	{
		DutEadSelfTest();
		return;
	}

	if (DutMatch(pLine, "disconnect", &arg) && *arg == 0)
	{
		int count = BtAppDisconnectAll();
		DutOut("DUT DISCONNECT requested=%d", count);
		DutOutEnd();
		return;
	}

	if (DutMatch(pLine, "bond clear", &arg) && *arg == 0)
	{
		BtSmpBondClearAll();
		DutOut("DUT BOND_CLEAR requested");
		DutOutEnd();
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
			DutOut("DUT ERROR phy missing_phy");
			DutOutEnd();
			return;
		}
		if (DutParseUint(&arg, &c) == false)
		{
			c = BT_GAP_PHY_OPT_NONE;
		}
		DutPhySet(a, b, c);
		return;
	}

	if (DutMatch(pLine, "datalen", &arg))
	{
		if (DutParseUint(&arg, &a) == false ||
			DutParseUint(&arg, &b) == false)
		{
			DutOut("DUT ERROR datalen missing_value");
			DutOutEnd();
			return;
		}
		DutDataLen(a, b);
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
			DutOut("DUT ERROR pawr_data missing_subevent");
			DutOutEnd();
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
		DutOut("DUT NUMCOMP_REPLY hdl=%u accept=1", (unsigned)hdl);
		DutOutEnd();
		return;
	}

	if ((strcmp(pLine, "n") == 0 || strcmp(pLine, "no") == 0) &&
		s_NumCompPending)
	{
		uint16_t hdl = s_NumCompConnHdl;
		s_NumCompPending = false;
		s_NumCompConnHdl = BT_CONN_HDL_INVALID;
		BtSmpNumericComparisonReply(hdl, false);
		DutOut("DUT NUMCOMP_REPLY hdl=%u accept=0", (unsigned)hdl);
		DutOutEnd();
		return;
	}

	if (strcmp(pLine, "help") == 0)
	{
		PrintHelp();
		return;
	}

	DutOut("DUT ERROR unknown_command=%s", pLine);
	DutOutEnd();
}

// ***
// Interface plumbing
//
// Both interfaces defer to the application event handler rather than running a
// command in the callback that delivered it. On the BLE side that callback is
// inside the stack's handling of an incoming write, and answering from there
// would have one link receiving and transmitting inside a single call.
//

static void DutUartHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	uint8_t buf[64];
	int len;

	while ((len = g_Uart.Rx(buf, sizeof(buf))) > 0)
	{
		DutLineFeed(&s_UartLine, buf, len);
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

static void DutBleHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	uint8_t buf[DUT_INTRF_PACKET_MAX];
	int len;

	while ((len = g_BtIntrf.Rx(0, buf, sizeof(buf))) > 0)
	{
		DutLineFeed(&s_BleLine, buf, len);
	}
}

static int DutBleIntrfEvtHandler(DevIntrf_t *pDev, DEVINTRF_EVT EvtId,
	uint8_t *pBuffer, int BufferLen)
{
	(void)pDev;
	(void)pBuffer;
	(void)BufferLen;

	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		AppEvtHandlerQue(0, nullptr, DutBleHandler);
	}

	return 0;
}

// ***
// Stack callbacks
//

void BtAppInitUserData(void)
{
	BtSmpAuthConfig(BT_SMP_IOCAPS_DISPLAY_YESNO,
		BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM);
}

void BtAppInitUserServices(void)
{
	if (BtGattSrvcAdd(&g_DutService) == false)
	{
		DutOut("DUT ERROR service_add");
		DutOutEnd();
		return;
	}

	BtGattCharSetValue(&g_DutChars[DUT_CHAR_IDX_RX], s_RxCharValue, 0);
	BtGattCharSetValue(&g_DutChars[DUT_CHAR_IDX_TX], s_TxCharValue, 0);

	// BtIntrf installs its own write and transmit complete callbacks on the
	// two characteristics, so it goes after the service is registered.
	s_BtIntrfReady = g_BtIntrf.Init(s_BtIntrfCfg);
	if (s_BtIntrfReady == false)
	{
		DutOut("DUT ERROR intrf_init");
		DutOutEnd();
	}
}

void BtAppEvtConnected(uint16_t ConnHdl)
{
	// A fresh link starts a fresh command line. Half a line left by a client
	// that disconnected mid write would otherwise join the next client's first
	// command.
	s_BleLine.Len = 0;

	DutOut("DUT CONNECTED hdl=%u", (unsigned)ConnHdl);
	DutOutEnd();
	PrintLink(ConnHdl);
}

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	if (s_NumCompPending && s_NumCompConnHdl == ConnHdl)
	{
		s_NumCompPending = false;
		s_NumCompConnHdl = BT_CONN_HDL_INVALID;
	}
	s_BleLine.Len = 0;

	DutOut("DUT DISCONNECTED hdl=%u", (unsigned)ConnHdl);
	DutOutEnd();
}

void BtAppEvtSecured(uint16_t ConnHdl)
{
	DutOut("DUT SECURED hdl=%u", (unsigned)ConnHdl);
	DutOutEnd();
	PrintLink(ConnHdl);
}

void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
}

// Numeric Comparison, Core 5.4 Vol 3 Part H 2.3.5.6.4. The value is reported
// and the answer comes back as a command, so a harness confirms it the same
// way a person would.
void BtSmpNumericComparison(uint16_t ConnHdl, uint32_t Value)
{
	s_NumCompPending = true;
	s_NumCompConnHdl = ConnHdl;

	DutOut("DUT NUMCOMP hdl=%u value=%06u", (unsigned)ConnHdl,
		(unsigned)Value);
	DutOutEnd();
}

// The controller asks for the subevents it is about to advertise. Answering
// every one of them from the retained buffers keeps the train advertising the
// same payload each rotation, which is what lets the harness compare a
// subevent report against a value it set earlier.
//
// The requested subevents wrap at the subevent count, so the set is not simply
// Start to Start + Count and BtPadvSubeventOfRequest resolves it. They go out
// in one command because the whole request fits one: the DUT train has four
// subevents and the command takes fifteen.
void BtPadvSubeventDataRequest(uint8_t AdvHdl, uint8_t Start, uint8_t Count)
{
	if (s_PawrReady == false || Count == 0)
	{
		return;
	}

	if (Count > DUT_PAWR_SUBEVENT_CNT)
	{
		Count = DUT_PAWR_SUBEVENT_CNT;
	}

	BtPadvSubeventData_t sub[DUT_PAWR_SUBEVENT_CNT];

	for (uint8_t i = 0; i < Count; i++)
	{
		uint8_t subevent = BtPadvSubeventOfRequest(Start, i,
			DUT_PAWR_SUBEVENT_CNT);

		sub[i].Subevent = subevent;
		sub[i].RspSlotStart = 0;
		sub[i].RspSlotCount = DUT_PAWR_RSP_SLOT_CNT;
		sub[i].DataLen = s_PawrLen[subevent];
		sub[i].pData = s_PawrData[subevent];
	}

	bool ok = BtPadvSubeventDataSet(AdvHdl, sub, Count);

	// The controller asks again every periodic advertising event, so tracing
	// each one would fill both interfaces with lines that all say the same
	// thing and slow the very handler they describe. One line proves the train
	// is asking, and a later line means the answer stopped being complete.
	if (s_PawrReqTraced == false || ok == false)
	{
		s_PawrReqTraced = true;
		DutOut("DUT PAWR_REQ start=%u count=%u filled=%u",
			(unsigned)Start, (unsigned)Count, ok ? (unsigned)Count : 0u);
		DutOutEnd();
	}
}

void BtPsyncResponseReport(uint8_t AdvHdl, uint8_t Subevent, uint8_t RspSlot,
	int8_t TxPwr, int8_t Rssi, const uint8_t *pData, uint16_t Len)
{
	(void)AdvHdl;

	DutOut("DUT PAWR_RSP subevent=%u slot=%u txpwr=%d rssi=%d len=%u data=",
		(unsigned)Subevent, (unsigned)RspSlot, (int)TxPwr, (int)Rssi,
		(unsigned)Len);
	DutOutHex(pData, Len);
	DutOutEnd();
}

// A subevent that never went out means the responders had nothing to answer,
// so silence in it says nothing about them. A harness that treated it as a
// missing response would be blaming the wrong side.
void BtPsyncSubeventNotSent(uint8_t AdvHdl, uint8_t Subevent)
{
	(void)AdvHdl;

	DutOut("DUT PAWR_NOT_SENT subevent=%u", (unsigned)Subevent);
	DutOutEnd();
}

static void HardwareInit(void)
{
	g_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), (DevIntrf_t*)g_Uart, 0, nullptr, 0);
}

int main(void)
{
	HardwareInit();

	DutOut("DUT BOOT name=%s proto=%u", DUT_DEVICE_NAME,
		(unsigned)DUT_PROTOCOL_VERSION);
	DutOutEnd();
	DutOut("DUT SECURITY method=numcomp bonding=1 mitm=1 lesc=1");
	DutOutEnd();

	if (BtAppInit(&s_BleAppCfg) == false)
	{
		DutOut("DUT ERROR bt_init");
		DutOutEnd();
		while (true)
		{
		}
	}

	// Advertising Coding Selection is claimed once here. Core Vol 4 Part E
	// 7.8.115 has the controller refuse the host feature bit while it holds a
	// connection, so a "coding" command arriving later could not claim it.
	s_CodingFeature = BtAdvCodingSelectionEnable(true);

	PrintStatus();
	DutOut("DUT CODING_FEATURE claimed=%d", s_CodingFeature ? 1 : 0);
	DutOutEnd();
	PrintHelp();
	DutOut("DUT READY");
	DutOutEnd();

	BtAppRun();

	DutOut("DUT ERROR bt_run_returned");
	DutOutEnd();
	while (true)
	{
	}
}

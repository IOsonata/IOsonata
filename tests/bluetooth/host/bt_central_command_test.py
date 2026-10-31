#!/usr/bin/env python3
"""Exercise split console commands through the production central RX handler."""
import os
from pathlib import Path
import subprocess
import tempfile
from extract_bt_app_init import extract_function

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT / 'exemples/bluetooth/uart_ble_central.cpp').read_text()
code = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include "cfifo.h"
#define BLE_SC_METHOD 0
#define BLE_SC_NONE 0
#define CENTRAL_PRBS_CHECK 1
#define PACKET_SIZE 128
#define UART_DEVNO 0
struct UARTDev_t
{
	unsigned TxDropCnt;
	hCFifo_t hTxFifo;
} uart;

enum UART_EVT
{
	UART_EVT_RXTIMEOUT,
	UART_EVT_RXDATA,
	UART_EVT_TXREADY,
	UART_EVT_LINESTATE
};

struct BtDevice_t
{
	struct
	{
		uint16_t MaxMtu;
	} Conn;
} peer;

struct
{
	struct
	{
		uint16_t Hdl;
	} Conn;
} g_ConnectedDev;

static bool s_bUartRxPending;
static int g_UartRxExtBuffLen;
static uint8_t g_UartRxExtBuff[256];
static hCFifo_t g_UartRx2BleFifo;
static unsigned g_DropCnt;
static unsigned rxQueued;
static unsigned txQueued;
static unsigned bleReports;
static unsigned streamReports;
static unsigned s_BleToUartReceived;
static unsigned s_BleToUartAccepted;
static unsigned s_BleToUartDropped;
static unsigned s_BleNotifications;
static unsigned s_BleFirstRxMs;
static unsigned s_BlePrbsBreaks;
static uint16_t s_BleNotifyMinLen;
static uint16_t s_BleNotifyMaxLen;

struct
{
	const char *input = "";

	int Rx(uint8_t *data, int capacity)
	{
		int count = strlen(input);

		assert(count <= capacity);
		memcpy(data, input, count);
		input = "";
		return count;
	}

	void printf(const char *format, ...)
	{
		va_list args;
		char buffer[80];
		int length;

		va_start(args, format);
		length = vsnprintf(buffer, sizeof(buffer), format, args);
		va_end(args);
		assert(length >= 0 && length < int(sizeof(buffer)));

		if (strstr(format, "BLE:"))
		{
			++bleReports;
		}
		if (strstr(format, "Stream:"))
		{
			++streamReports;
		}
	}
} g_Uart;

static unsigned DisableInterrupt()
{
	return 0;
}

static void EnableInterrupt(unsigned)
{
}

static unsigned BtGattMsTick()
{
	return 0;
}

static BtDevice_t *BtPeerFindByHdl(uint16_t)
{
	return &peer;
}

static const UARTDev_t *UARTGetInstance(int)
{
	return &uart;
}

static bool UartBleSecTryCommand(const uint8_t *, int)
{
	return false;
}

static bool UartBleOobTryCommand(const uint8_t *, int)
{
	return false;
}

static void BleTxQue()
{
	++txQueued;
}

static void UartRxQue()
{
	++rxQueued;
}

'''
for signature in [r'^static void UartBlePrintStats\(',
                  r'^void UartRxSchedHandler\([^;]+\)(?=\n\{)',
                  r'^int nRFUartEvthandler\([^;]+\)(?=\n\{)']:
    code += extract_function(source, signature) + '\n'
code += r'''
int main()
{
	alignas(8) uint8_t mem[CFIFO_MEMSIZE(512)];
	alignas(8) uint8_t forward[CFIFO_MEMSIZE(512)];
	const char *command = "stats\n";
	const char *p;
	char prefix[6];
	unsigned split;

	s_BleNotifications = UINT32_MAX;
	s_BleToUartReceived = UINT32_MAX;
	s_BleToUartAccepted = UINT32_MAX;
	s_BleToUartDropped = UINT32_MAX;
	s_BlePrbsBreaks = UINT32_MAX;
	s_BleNotifyMinLen = UINT16_MAX;
	s_BleNotifyMaxLen = UINT16_MAX;
	s_BleFirstRxMs = 1;
	peer.Conn.MaxMtu = UINT16_MAX;
	uart.TxDropCnt = UINT32_MAX;

	g_UartRx2BleFifo = CFifoInit(mem, sizeof(mem), 1, true);
	uart.hTxFifo = CFifoInit(forward, sizeof(forward), 1, true);
	// Every split of the command, with an empty read between fragments.
	for (split = 1; split <= 5; ++split)
	{
		memset(prefix, 0, sizeof(prefix));
		memcpy(prefix, command, split);
		g_Uart.input = prefix;
		UartRxSchedHandler(0, nullptr);
		UartRxSchedHandler(0, nullptr);
		assert(g_UartRxExtBuffLen == int(split));
		assert(CFifoUsed(g_UartRx2BleFifo) == 0 && txQueued == 0);
		assert(rxQueued == split - 1); // Partial command must not spin in main.
		nRFUartEvthandler(&uart, UART_EVT_RXDATA, nullptr, 0);
		assert(rxQueued == split); // Further RX must wake the partial command.
		g_Uart.input = command + split;
		UartRxSchedHandler(0, nullptr);
		assert(g_UartRxExtBuffLen == 0);
		assert(bleReports == split && streamReports == split);
	}
	// A prefix that turns out to be ordinary payload is still forwarded.
	g_Uart.input = "st";
	UartRxSchedHandler(0, nullptr);
	g_Uart.input = "uff\n";
	UartRxSchedHandler(0, nullptr);
	assert(CFifoUsed(g_UartRx2BleFifo) == 6 && txQueued == 1);
	for (p = "stuff\n"; *p; ++p)
	{
		assert(*CFifoGet(g_UartRx2BleFifo) == *p);
	}
}
'''
with tempfile.TemporaryDirectory(prefix='central-command-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    binary = Path(temp) / 'test'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O2',
                    '-Wall', '-Wextra', '-fsanitize=undefined',
                    '-fno-sanitize-recover=all', '-I' + str(ROOT / 'include'),
                    str(cpp), str(ROOT / 'src/cfifo.c'), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('PASS central split stats command, RX wakeup and ordinary payload')

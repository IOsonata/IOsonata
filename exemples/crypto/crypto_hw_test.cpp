/**-------------------------------------------------------------------------
@example	crypto_hw_test.cpp

@brief	Runner of the crypto hardware tests on the console UART

The crypto hardware acceptance tests print on stdout. This runner sends
stdout to SysLog, whose output is the console UART of board.h, and runs them
one after the other:

	RngTest			rng_test.cpp, the hardware random generator engine
	Cc3xxEcdhTest	cc3xx_ecdh_test.cpp, P-256 on the CryptoCell, when
					CRYPTO_HW_TEST_CC3XX is 1

Build the test sources with RNG_TEST_NO_MAIN and CC3XX_ECDH_TEST_NO_MAIN so
this file gives main.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "syslog.h"

#include "board.h"

#ifndef CRYPTO_HW_TEST_CC3XX
#define CRYPTO_HW_TEST_CC3XX	0
#endif

bool RngTest(void);
#if CRYPTO_HW_TEST_CC3XX
bool Cc3xxEcdhTest(void);
#endif

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

// Longest SysLog record, the stdout writes are cut to it
#define CRYPTO_HW_TEST_LOG_RECLEN	128

static int CryptoHwTestLogWrite(void * const pDevObj, int Handle, uint8_t *pBuff, size_t Len);

static UART s_Uart;

// SysLog on the console UART, a record goes out as soon as it is logged.
// 16 records. A record the UART does not take in full stays in the store
// and the rest goes out at the next log call. bBlocking keeps the oldest
// records when the store is full.
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, CRYPTO_HW_TEST_LOG_RECLEN)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = CRYPTO_HW_TEST_LOG_RECLEN,
	.bBlocking = true,
};

// stdout of the tests, into SysLog
static StdDev_t s_LogStdDev = {
	"SYSLOG",
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	CryptoHwTestLogWrite,
	nullptr,
};

static int CryptoHwTestLogWrite(void * const pDevObj, int Handle, uint8_t *pBuff, size_t Len)
{
	(void)pDevObj;
	(void)Handle;

	size_t done = 0;

	while (done < Len)
	{
		size_t n = Len - done;

		if (n > CRYPTO_HW_TEST_LOG_RECLEN - 1)
		{
			n = CRYPTO_HW_TEST_LOG_RECLEN - 1;
		}
		SysLogPrintf(SysLogGet(), "%.*s", (int)n, (const char *)&pBuff[done]);
		done += n;
	}

	return (int)Len;
}

int main()
{
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
	InstallBlkDev(&s_LogStdDev, STDOUT_FILENO);

	printf("Crypto hardware tests\r\n");

	bool res = RngTest();

#if CRYPTO_HW_TEST_CC3XX
	res = Cc3xxEcdhTest() && res;
#endif

	printf("\r\nCrypto hardware tests %s\r\n", res ? "PASS" : "FAIL");
	fflush(stdout);

	while (1)
	{
		__WFE();
	}

	return 0;
}

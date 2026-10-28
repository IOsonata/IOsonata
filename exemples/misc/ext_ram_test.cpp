/**-------------------------------------------------------------------------
@example	ext_ram_test.cpp

@brief	External SDRAM test

Sets up the SDRAM of the board (coredev/sdram.h), then checks: the data
lines (a walking one), the address lines (one address at each power of
two), the byte lanes (byte and half word writes), then the whole memory
written with a value made from the address and read back, a second time
inverted. Prints the result of each check and ends with "ALL PASS". board.h
gives the SDRAM and its pins:

	SDRAM_PINS			pin map of the memory controller
	SDRAM_DEVNO			optional, chip select, 0
	SDRAM_RATE			SDRAM clock, Hz
	SDRAM_BUS_WIDTH		data bus, bits
	SDRAM_DEV_WIDTH		data width of one part, bits
	SDRAM_NB_BANKS		banks of a part
	SDRAM_ROW_BITS		row address bits of a part
	SDRAM_COL_BITS		column address bits of a part
	SDRAM_CAS_LATENCY	CAS latency, clocks
	SDRAM_TMRD_CLK		load mode register to active, clocks
	SDRAM_TWR_CLK		write recovery, clock part, added to SDRAM_TWR_NS
	SDRAM_REFRESH_NS	refresh period / rows
	SDRAM_TRCD_NS, SDRAM_TRP_NS, SDRAM_TRAS_NS, SDRAM_TRC_NS, SDRAM_TRFC_NS,
	SDRAM_TRRD_NS, SDRAM_TWR_NS, SDRAM_TAPR_NS, SDRAM_TSREX_NS,
	SDRAM_TXSR_NS		timings of the data sheet, ns

With UART_RETARGET and the UART pins, the Release build prints on the UART
at 1000000 baud 8N1, the Debug build uses semihosting.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#include <stdio.h>
#include <stdint.h>

#include "coredev/sdram.h"
#include "coredev/uart.h"
#include "stddev.h"

#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE		1000000
#endif

#ifdef UART_RETARGET
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCallback = NULL,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};

static UART s_Uart;
#endif

#ifndef SDRAM_DEVNO
#define SDRAM_DEVNO			0
#endif

static const IOPinCfg_t s_SdramPins[] = SDRAM_PINS;

static const SdramCfg_t s_SdramCfg = {
	.DevNo = SDRAM_DEVNO,
	.pIOPinMap = s_SdramPins,
	.NbIOPins = sizeof(s_SdramPins) / sizeof(IOPinCfg_t),
	.Rate = SDRAM_RATE,
	.BusWidth = SDRAM_BUS_WIDTH,
	.DevWidth = SDRAM_DEV_WIDTH,
	.NbBanks = SDRAM_NB_BANKS,
	.RowBits = SDRAM_ROW_BITS,
	.ColBits = SDRAM_COL_BITS,
	.CasLatency = SDRAM_CAS_LATENCY,
	.TwrClk = SDRAM_TWR_CLK,
	.TmrdClk = SDRAM_TMRD_CLK,
	.RefreshNs = SDRAM_REFRESH_NS,
	.TrcdNs = SDRAM_TRCD_NS,
	.TrpNs = SDRAM_TRP_NS,
	.TrasNs = SDRAM_TRAS_NS,
	.TrcNs = SDRAM_TRC_NS,
	.TrfcNs = SDRAM_TRFC_NS,
	.TrrdNs = SDRAM_TRRD_NS,
	.TwrNs = SDRAM_TWR_NS,
	.TaprNs = SDRAM_TAPR_NS,
	.TsrexNs = SDRAM_TSREX_NS,
	.TxsrNs = SDRAM_TXSR_NS,
};

static Sdram s_Sdram;

#define EXT_RAM_MAX_REPORT	8

static volatile uint32_t *s_pRam = NULL;
static uint32_t s_Words = 0;
static int s_Fail = 0;
static int s_Errors = 0;

static void Check(bool Ok, const char *pName)
{
	printf("%s : %s\r\n", Ok ? "PASS" : "FAIL", pName);
	if (Ok == false)
	{
		s_Fail++;
	}
}

static void Error(uint32_t Idx, uint32_t Expect, uint32_t Read)
{
	if (s_Errors < EXT_RAM_MAX_REPORT)
	{
		printf("  0x%08lX: wrote 0x%08lX, read 0x%08lX\r\n", (unsigned long)(uintptr_t)(s_pRam + Idx),
			   (unsigned long)Expect, (unsigned long)Read);
	}
	s_Errors++;
}

// A value of each word that differs from its neighbours and from the words
// at a power of two distance
static uint32_t Pattern(uint32_t Idx)
{
	uint32_t v = Idx * 0x9E3779B1UL + 0x7F4A7C15UL;

	return v ^ (v >> 15) ^ (Idx << 7);
}

// A one walking through the 32 data lines, on the first word
static bool DataLines()
{
	int err = s_Errors;

	for (int i = 0; i < 32; i++)
	{
		uint32_t v = 1UL << i;

		s_pRam[0] = v;
		s_pRam[1] = ~v;			// the bus changes between the write and the read
		uint32_t r = s_pRam[0];
		if (r != v)
		{
			Error(0, v, r);
		}
	}

	return err == s_Errors;
}

// One word at each power of two: a stuck or shorted address line makes two
// of them the same word
static bool AddrLines()
{
	int err = s_Errors;
	uint32_t i, j;

	for (i = 1; i < s_Words; i <<= 1)
	{
		s_pRam[i] = 0xAAAAAAAAUL;
	}
	s_pRam[0] = 0x55555555UL;
	for (i = 1; i < s_Words; i <<= 1)
	{
		if (s_pRam[i] != 0xAAAAAAAAUL)
		{
			Error(i, 0xAAAAAAAAUL, s_pRam[i]);
		}
	}
	for (i = 1; i < s_Words; i <<= 1)
	{
		s_pRam[i] = 0x55555555UL;
		for (j = 1; j < s_Words; j <<= 1)
		{
			if (j != i && s_pRam[j] != 0xAAAAAAAAUL)
			{
				Error(j, 0xAAAAAAAAUL, s_pRam[j]);
			}
		}
		if (s_pRam[0] != 0x55555555UL)
		{
			Error(0, 0x55555555UL, s_pRam[0]);
		}
		s_pRam[i] = 0xAAAAAAAAUL;
	}

	return err == s_Errors;
}

// Byte and half word writes reach their own lanes only
static bool ByteLanes()
{
	volatile uint8_t *b = (volatile uint8_t *)s_pRam;
	volatile uint16_t *h = (volatile uint16_t *)s_pRam;
	int err = s_Errors;

	s_pRam[0] = 0;
	s_pRam[1] = 0xFFFFFFFFUL;
	b[0] = 0x11;
	b[1] = 0x22;
	b[2] = 0x33;
	b[3] = 0x44;
	if (s_pRam[0] != 0x44332211UL)
	{
		Error(0, 0x44332211UL, s_pRam[0]);
	}
	h[1] = 0xBEEF;
	if (s_pRam[0] != 0xBEEF2211UL)
	{
		Error(0, 0xBEEF2211UL, s_pRam[0]);
	}
	b[5] = 0x5A;
	if (s_pRam[1] != 0xFFFF5AFFUL || b[0] != 0x11 || h[1] != 0xBEEF)
	{
		Error(1, 0xFFFF5AFFUL, s_pRam[1]);
	}

	return err == s_Errors;
}

// The whole memory, written first then read
static bool Whole(uint32_t Invert)
{
	int err = s_Errors;
	uint32_t i;

	for (i = 0; i < s_Words; i++)
	{
		s_pRam[i] = Pattern(i) ^ Invert;
	}
	for (i = 0; i < s_Words; i++)
	{
		uint32_t r = s_pRam[i];
		uint32_t v = Pattern(i) ^ Invert;

		if (r != v)
		{
			Error(i, v, r);
		}
	}

	return err == s_Errors;
}

int main()
{
#ifdef UART_RETARGET
	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1);
	}

#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
#endif

	printf("\r\nExternal SDRAM test\r\n");

	if (s_Sdram.Init(s_SdramCfg) == false)
	{
		printf("Init failed\r\n");

		return 1;
	}
	s_pRam = (volatile uint32_t *)s_Sdram.Base();
	s_Words = s_Sdram.Size() / 4UL;
	printf("%lu KB at 0x%08lX, clock %lu Hz\r\n", (unsigned long)(s_Sdram.Size() / 1024UL),
		   (unsigned long)(uintptr_t)s_Sdram.Base(), (unsigned long)s_Sdram.Rate());

	Check(DataLines(), "data lines");
	Check(AddrLines(), "address lines");
	Check(ByteLanes(), "byte lanes");
	Check(Whole(0), "whole memory");
	Check(Whole(0xFFFFFFFFUL), "whole memory, inverted");

	printf("\r\n%s, %d errors\r\n", s_Fail == 0 ? "ALL PASS" : "FAILURES", s_Errors);

	return s_Fail == 0 ? 0 : 1;
}

/**-------------------------------------------------------------------------
@example	nvm_mcu_nrf52_demo.cpp

@brief	NvmMcu demo on nRF52 internal flash.

		Runs the checks that only real memory can answer: that a page erases
		to all ones, that a write survives a reset, that programming clears
		bits and never sets them back, and that a write crossing a page
		boundary lands correctly.

		The operations are chosen from the build settings. A build defining
		SOFTDEVICE_PRESENT, or one of the SoftDevice names, submits the access
		to the SoftDevice; anything else drives the memory controller directly.
		Driving it directly while a SoftDevice runs does not work, which is why
		this is not left to a hand edit.

		On the SoftDevice path the demo drains SoC events itself, which is
		correct here because nothing else in this program consumes them. An
		application with its own event dispatch passes events to NvmMcuSdSocEvt
		instead.

		The region is worked out from the device at run time and placed below
		the pages a bond store would take, so no part specific address is
		needed. It must still hold nothing else: check it against the
		application image end in the map file. The demo prints the region it is
		about to use, so read that line before letting it erase.

@author	Hoang Nguyen Hoan
@date	July 23, 2026

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
#include <string.h>

#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "stddev.h"
#include "idelay.h"
#include "board.h"

// Taken from the build settings. A build with a SoftDevice must submit the
// access to it; driving the memory controller directly while a SoftDevice runs
// does not work. Define this yourself only to force one path for a test.
#ifndef NVM_MCU_DEMO_USE_SOFTDEVICE
  #if defined(SOFTDEVICE_PRESENT) || defined(S112) || defined(S113) || \
	  defined(S132) || defined(S140)
	#define NVM_MCU_DEMO_USE_SOFTDEVICE	1
  #else
	#define NVM_MCU_DEMO_USE_SOFTDEVICE	0
  #endif
#endif

#if NVM_MCU_DEMO_USE_SOFTDEVICE
#include "nrf_soc.h"
#include "nvm_mcu_sd.h"
#else
#include "nvm_mcu_nvmc.h"
#endif

// Pages the demo uses. The region is worked out from the device at run time
// so no part specific address is needed, and it sits below the pages a bond
// store would take. Define NVM_MCU_DEMO_REGION_ADDR to place it yourself.
#ifndef NVM_MCU_DEMO_REGION_PAGES
#define NVM_MCU_DEMO_REGION_PAGES		2
#endif

// Pages at the top of the memory the demo stays away from.
#ifndef NVM_MCU_DEMO_TOP_RESERVE_PAGES
#define NVM_MCU_DEMO_TOP_RESERVE_PAGES	3
#endif

// Written at the region start so a reset can show the data survived.
#define NVM_MCU_DEMO_MAGIC				0x4E564D31UL	// "NVM1"

static const IOPinCfg_t s_UartPins[] = UART_PINS;

static const UARTCfg_t s_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	//.EvtCallback = nRFUartEvthandler,
	.bFifoBlocking = true,
#ifdef UARTFIFOSIZE
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,//FIFOSIZE,
	.pTxMem = s_UartTxFifo,//g_TxBuff,
#else
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = 0,
	.pTxMem = NULL,
#endif
	.bDMAMode = true,
};

UART g_Uart;

static int s_Fail = 0;

// Read the memory directly, bypassing the driver. If this disagrees with what
// the driver returns, the driver is not working on the memory it claims to.
static uint32_t RawWord(uintptr_t Addr)
{
	return *(volatile uint32_t *)Addr;
}

static void Check(bool Cond, const char *pMsg)
{
	if (Cond)
	{
		printf("  ok   : %s\r\n", pMsg);
	}
	else
	{
		s_Fail++;
		printf("  FAIL : %s\r\n", pMsg);
	}
}

#if NVM_MCU_DEMO_USE_SOFTDEVICE
// Nothing else in this program consumes SoC events, so drain them here and
// hand every one to the NVM operations.
static void DemoSdIdle(void)
{
	uint32_t evt;

	while (sd_evt_get(&evt) == NRF_SUCCESS)
	{
		NvmMcuSdSocEvt(evt);
	}
}
#endif

// Report whether the whole region reads erased.
static bool RegionIsErased(NvmIO &Mem)
{
	uint32_t buf[64];
	uint64_t off = 0;

	while (off < Mem.Size())
	{
		uint32_t len = sizeof(buf);
		if (len > Mem.Size() - off)
		{
			len = (uint32_t)(Mem.Size() - off);
		}
		if (Mem.Read(off, buf, len) != (int)len)
		{
			return false;
		}
		for (uint32_t i = 0; i < len / 4; i++)
		{
			if (buf[i] != 0xFFFFFFFFUL)
			{
				return false;
			}
		}
		off += len;
	}

	return true;
}

static void RunDemo(NvmIO &Mem, uintptr_t RegionAddr)
{
	uint32_t page = Mem.EraseSize();

	printf("region  : 0x%08lX size %lu, page %lu, write unit %lu\r\n",
		   (unsigned long)RegionAddr,
		   (unsigned long)Mem.Size(), (unsigned long)page,
		   (unsigned long)Mem.WriteGran());

	// 1. What is already there. A matching magic means the previous run's
	//    data survived the reset, which is the point of the memory.
	//    The raw read is the reference: it is what the memory actually holds.
	printf("raw     : [0x%08lX] = 0x%08lX 0x%08lX before anything is touched\r\n",
		   (unsigned long)RegionAddr,
		   (unsigned long)RawWord(RegionAddr),
		   (unsigned long)RawWord(RegionAddr + 4));

	uint32_t head[2] = { 0, 0 };
	Mem.Read(0, head, sizeof(head));
	printf("driver  : reads 0x%08lX 0x%08lX\r\n",
		   (unsigned long)head[0], (unsigned long)head[1]);
	Check(head[0] == RawWord(RegionAddr),
		  "driver reads the same memory as a direct read");
	if (head[0] == NVM_MCU_DEMO_MAGIC)
	{
		printf("persist : magic found, run count %lu\r\n",
			   (unsigned long)head[1]);
	}
	else
	{
		printf("persist : no magic yet, first run on this region\r\n");
	}
	uint32_t runcount = (head[0] == NVM_MCU_DEMO_MAGIC) ? head[1] + 1 : 1;

	// 2. Erase, then the whole region must read all ones.
	printf("erase   : erasing %lu pages, the core stalls meanwhile\r\n",
		   (unsigned long)(Mem.Size() / page));
	Check(Mem.Erase(0, (uint32_t)Mem.Size()) == 0, "erase returned success");
	Check(RegionIsErased(Mem), "whole region reads all ones after erase");

	// 3. A write that crosses a page boundary.
	uint32_t pattern[32], readback[32];
	for (int i = 0; i < 32; i++) { pattern[i] = 0xA5000000UL | (uint32_t)i; }
	memset(readback, 0, sizeof(readback));

	uint64_t cross = page - (16 * 4);		// half before, half after
	Check(Mem.Write(cross, pattern, sizeof(pattern)) == (int)sizeof(pattern),
		  "write across a page boundary");
	Check(Mem.Read(cross, readback, sizeof(readback)) == (int)sizeof(readback),
		  "read it back");
	Check(memcmp(pattern, readback, sizeof(pattern)) == 0,
		  "data matches across the boundary");

	// 4. Programming clears bits and cannot set them back without an erase.
	uint32_t half = 0x0000FFFFUL;
	uint32_t ones = 0xFFFFFFFFUL;
	uint32_t rd = 0;
	Check(Mem.Write(0x100, &half, 4) == 4, "program a word");
	Mem.Read(0x100, &rd, 4);
	Check(rd == half, "word holds what was programmed");
	Check(Mem.Write(0x100, &ones, 4) == 4, "program all ones over it");
	Mem.Read(0x100, &rd, 4);
	Check(rd == half, "bits did not come back without an erase");

	// 5. The driver rejects what the memory cannot do.
	Check(Mem.Write(1, &ones, 4) == -EINVAL, "unaligned write rejected");
	Check(Mem.Write(0, &ones, 3) == -EINVAL, "partial word write rejected");
	Check(Mem.Erase(1, page) == -EINVAL, "unaligned erase rejected");
	Check(Mem.Read(Mem.Size(), &rd, 4) == -EINVAL, "read past the end rejected");

	// 6. Leave the magic and the run count behind for the next reset.
	Check(Mem.Erase(0, page) == 0, "erase the first page");
	uint32_t stamp[2] = { NVM_MCU_DEMO_MAGIC, runcount };
	Check(Mem.Write(0, stamp, sizeof(stamp)) == (int)sizeof(stamp),
		  "write the magic and run count");
	memset(head, 0, sizeof(head));
	Mem.Read(0, head, sizeof(head));
	Check(head[0] == NVM_MCU_DEMO_MAGIC && head[1] == runcount,
		  "stamp reads back through the driver");
	printf("raw     : [0x%08lX] = 0x%08lX 0x%08lX after the stamp\r\n",
		   (unsigned long)RegionAddr,
		   (unsigned long)RawWord(RegionAddr),
		   (unsigned long)RawWord(RegionAddr + 4));
	Check(RawWord(RegionAddr) == NVM_MCU_DEMO_MAGIC,
		  "the magic is in the memory, not only in the driver view");

	printf("\r\n%s : reset the board, the run count must increase\r\n",
		   s_Fail == 0 ? "ALL PASS" : "FAILURES");
}

int main(void)
{
	g_Uart.Init(s_UartCfg);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

	printf("\r\nNvmMcu demo on nRF52 internal memory\r\n");

	NvmCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));

#if NVM_MCU_DEMO_USE_SOFTDEVICE
	NvmMcuSdCfg(cfg);
	NvmMcuSdSetIdle(DemoSdIdle, 5000);
	const NvmMcuOp_t &op = NvmMcuSdOp();
	printf("mode    : access submitted to the SoftDevice\r\n");
#else
	NvmMcuNvmcCfg(cfg);
	const NvmMcuOp_t &op = NvmMcuNvmcOp();
	printf("mode    : memory controller driven directly\r\n");
#endif

	printf("device  : %lu bytes, page %lu\r\n",
		   (unsigned long)cfg.TotalSize, (unsigned long)cfg.EraseSize);
	// Place the region below the pages a bond store would take, unless the
	// build named an address.
#ifdef NVM_MCU_DEMO_REGION_ADDR
	uintptr_t regionaddr = NVM_MCU_DEMO_REGION_ADDR;
#else
	uintptr_t regionaddr = (uintptr_t)(cfg.TotalSize
			- (uint64_t)cfg.EraseSize * (NVM_MCU_DEMO_TOP_RESERVE_PAGES
										 + NVM_MCU_DEMO_REGION_PAGES));
#endif

	printf("cfg     : base 0x%08lX, region 0x%08lX, %u pages\r\n",
		   (unsigned long)cfg.BaseAddr, (unsigned long)regionaddr,
		   NVM_MCU_DEMO_REGION_PAGES);

	NvmMcu mem;
	uint64_t regionsize = (uint64_t)cfg.EraseSize * NVM_MCU_DEMO_REGION_PAGES;

	if (mem.Init(cfg, op, regionaddr, regionsize) == false)
	{
		printf("Init failed, check the region against the device size\r\n");
		while (true) { msDelay(1000); }
	}

	RunDemo(mem, regionaddr);

	while (true)
	{
		msDelay(1000);
	}
}

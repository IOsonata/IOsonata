/**-------------------------------------------------------------------------
@file	nvm_demo_checks.h

@brief	NVM test setup and verification shared by the standalone and radio tests.

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
#ifndef __NVM_DEMO_CHECKS_H__
#define __NVM_DEMO_CHECKS_H__

#include <stdio.h>
#include <string.h>

#include "istddef.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "idelay.h"
#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE		1000000
#endif

#include "storage/nvm.h"

// What is underneath: 0 the MCU internal memory, 1 a SPI NOR flash
// (MX25R3235F), 2 an I2C EEPROM (CAT24C32). Everything below the
// construction blocks is shared.
#ifndef NVM_DEMO_MEDIUM
#define NVM_DEMO_MEDIUM			0
#endif

#if NVM_DEMO_MEDIUM == 0
#include "storage/nvm_mcu_intrf.h"
#include "storage/nvm_region.h"
#elif NVM_DEMO_MEDIUM == 1
#include "coredev/spi.h"
#include "storage/flash.h"		// for the FLASH_CMD_READ/WRITE opcodes
#elif NVM_DEMO_MEDIUM == 2
#include "coredev/i2c.h"
#endif

#include "stddev.h"

// Completion mode. 0 the calls finish in the call, 1 the driver reports
// through an event and IsBusy/Sync drive the operation. The second is the one
// a stack sits on top of: a write started from an event handler must not block
// it, so the operation has to make progress from somewhere else.
#ifndef NVM_DEMO_ASYNC
#define NVM_DEMO_ASYNC			1
#endif


#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

// Two scratch pages the checks use, plus one for the stamp.
#ifndef NVM_DEMO_SCRATCH_PAGES
#define NVM_DEMO_SCRATCH_PAGES	2
#endif

#define NVM_DEMO_REGION_PAGES	(NVM_DEMO_SCRATCH_PAGES + 1)

// Destructive MCU test region, separate from NVM0 bond storage.
#ifndef NVM_DEMO_REGION_NO
#define NVM_DEMO_REGION_NO		1
#endif

// Pages at the top of the memory the demo stays away from.
#ifndef NVM_DEMO_TOP_RESERVE_PAGES
#define NVM_DEMO_TOP_RESERVE_PAGES	3
#endif

#define NVM_DEMO_MAGIC			0x4E564D31UL	// "NVM1"

// UART instance of the output, 0 when board.h does not say
#ifndef NVM_DEMO_UART_DEVNO
#define NVM_DEMO_UART_DEVNO		0
#endif

static const IOPinCfg_t s_UartPins[] = UART_PINS;

// TX FIFO large enough for the burst of check results printed back to back;
// the driver's built in fallback is only 32 bytes.
#define UART_TXFIFO_MEMSIZE			CFIFO_MEMSIZE(2048)

alignas(4) static uint8_t s_UartTxFifoMem[UART_TXFIFO_MEMSIZE];

static const UARTCfg_t s_UartCfg = {
	.DevNo = NVM_DEMO_UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = UART_TXFIFO_MEMSIZE,
	.pTxMem = s_UartTxFifoMem,
	.bDMAMode = true,
};

UART g_Uart;

#if NVM_DEMO_MEDIUM == 0

static const char s_MediumName[] = "internal memory";

static NvmMcuIntrf s_MemIntrf;

#elif NVM_DEMO_MEDIUM == 1

static const char s_MediumName[] = "SPI NOR flash";

static const IOPinCfg_t s_SpiPins[] = SPI_PINS_CFG;

// SPI clock, the board can set it
#ifndef NVM_DEMO_SPI_RATE
#define NVM_DEMO_SPI_RATE		8000000
#endif

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = NVM_DEMO_SPI_RATE,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,
	// Polling: the driver runs each bus transfer to completion and paces the
	// memory operation itself. Interrupt mode here with no callback left the
	// transfer with nothing to finish it.
	.bIntEn = false,
	.IntPrio = 6,
	.EvtCB = NULL
};

static SPI s_MemIntrf;

#ifdef NVM_DEMO_CHIP_CFG
// The part of the board, from board.h
static const NvmCfg_t s_ChipCfg = NVM_DEMO_CHIP_CFG;
#else
// MX25R3235F : 32 Mbits NOR flash, 4K sector, 256 byte page. From the
// device database in nvm.h; another part is another entry here.
static const NvmCfg_t s_ChipCfg = {
	.DevNo = 0,
	.TotalSize = 32 * 1024 * 1024 / 8,
	.EraseSize = 4 * 1024,
	.PageSize = 256,
	.AddrSize = 3,
	.WrProtMask = 0x3C,				// BP0..BP3
	.WrProtPin = { -1, -1, },		// no pin on this part
};
#endif

#elif NVM_DEMO_MEDIUM == 2

static const char s_MediumName[] = "I2C EEPROM";

static const IOPinCfg_t s_I2cPins[] = I2C_PINS_CFG;

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 100000,
	.MaxRetry = 5,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = {0,},
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 6,
	.EvtCB = NULL
};

static I2C s_MemIntrf;

// CAT24C32 : 32 Kbits I2C EEPROM, 2 byte address, 32 byte page, 5 ms write.
// No erase: a command value of 0 means the medium does not have it.
static const NvmCfg_t s_ChipCfg = {
	.DevNo = 0x50,					// I2C device address
	.TotalSize = 32 * 1024 / 8,
	.EraseSize = 0,					// overwrites directly
	.PageSize = 32,
	.AddrSize = 2,
	.WrProtPin = { -1, -1, },		// no pin on this part
	.WriteDelayUs = 5000,			// Twr
};

#endif

static Nvm s_Nvm;

static uintptr_t s_RegionAddr = 0;
static int s_Fail = 0;

// What the driver reported, and how often. One event per operation, so these
// counts are checked against the operations the demo issued.
static volatile uint32_t s_EvtWrite;
static volatile uint32_t s_EvtErase;
static volatile uint32_t s_EvtError;
static volatile int s_EvtLastRes;
static volatile uint64_t s_EvtLastOff;
static volatile uint32_t s_EvtLastLen;

static uint8_t s_Big[512];
static uint8_t s_RBack[512];

// The interface reports a finished transfer here. Nvm is told, and nothing
// else on this interface needs it. The interface does not install this
// itself: the callback belongs to whoever created the interface.
static int MemIntrfEvtCB(DevIntrf_t * const, DEVINTRF_EVT EvtId, uint8_t *, int)
{
	s_Nvm.IntrfEvent(EvtId);

	return 0;
}

static void NvmDemoEvtHandler(Nvm *pMem, NVM_EVT Evt, uint64_t Off,
							  uint32_t Len, int Res)
{
	(void)pMem;

	s_EvtLastRes = Res;
	s_EvtLastOff = Off;
	s_EvtLastLen = Len;

	switch (Evt)
	{
		case NVM_EVT_WRITE_DONE:
			s_EvtWrite = s_EvtWrite + 1;
			break;

		case NVM_EVT_ERASE_DONE:
			s_EvtErase = s_EvtErase + 1;
			break;

		case NVM_EVT_ERROR:
			s_EvtError = s_EvtError + 1;
			break;

		default:
			break;
	}
}

// The checks read back what they wrote, so an operation has to be finished
// before the next line looks at it. Sync drains one in flight and returns at
// once when there is none, so the checks read the same in both modes and the
// demo needs no second version of them.
static int DemoWrite(Nvm &Mem, uint64_t Off, const void *pData, uint32_t Len)
{
	int r = Mem.Write(Off, pData, Len);
	if (r != (int)Len)
	{
		printf("write   : offset 0x%lX, requested %lu bytes, Write returned %d\r\n",
			   (unsigned long)Off, (unsigned long)Len, r);
	}
	if (r < 0)
	{
		return r;
	}

	int sr = Mem.Sync();
	if (sr < 0)
	{
		printf("write   : offset 0x%lX, Sync returned %d\r\n", (unsigned long)Off, sr);
	}
	return sr < 0 ? sr : r;
}

static int DemoErase(Nvm &Mem, uint64_t Off, uint32_t Len)
{
	int r = Mem.Erase(Off, Len);
	if (r < 0)
	{
		return r;
	}

	int sr = Mem.Sync();

	return sr < 0 ? sr : r;
}

#if NVM_DEMO_MEDIUM == 0
// Read the memory directly, bypassing the driver. If this disagrees with what
// the driver returns, the driver is not working on the memory it claims to.
// Only the internal memory is mapped; a chip on a bus has no address to read.
static uint32_t RawWord(uintptr_t Addr)
{
	return *(volatile uint32_t *)Addr;
}
#endif

// The erase unit is the natural granule for the region. A medium with no
// erase has none, so a fixed granule stands in for it.
static uint32_t UnitSize(Nvm &Mem)
{
	uint32_t u = Mem.EraseSize();

	return u != 0 ? u : 256;
}

static bool Check(bool Cond, const char *pMsg)
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
	return Cond;
}

// Report whether the scratch pages read erased.
static bool ScratchIsErased(Nvm &Mem)
{
	uint32_t buf[64];
	uint64_t end = (uint64_t)UnitSize(Mem) * NVM_DEMO_SCRATCH_PAGES;
	uint64_t off = 0;

	while (off < end)
	{
		uint32_t len = sizeof(buf);
		if (len > end - off)
		{
			len = (uint32_t)(end - off);
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

// The stamp lives on the page after the scratch, so the checks never erase it.
static void StampCheck(Nvm &Mem)
{
	uintptr_t RegionAddr = (uintptr_t)(Mem.BaseAddress() + Mem.RegionOffset());
	uint32_t page = UnitSize(Mem);
	uint64_t off = (uint64_t)page * NVM_DEMO_SCRATCH_PAGES;
	uintptr_t raw = RegionAddr + (uintptr_t)off;
	uint32_t stamp[2] = { 0, 0 };

	if (Mem.Read(off, stamp, sizeof(stamp)) != (int)sizeof(stamp))
	{
		Check(false, "read previous boot stamp");
		return;
	}

#if NVM_DEMO_MEDIUM == 0
	printf("raw     : [0x%08lX] = 0x%08lX 0x%08lX before anything is touched\r\n",
							  (unsigned long)raw, (unsigned long)RawWord(raw),
							  (unsigned long)RawWord(raw + 4));

	Check(stamp[0] == RawWord(raw), "driver reads the same memory as a direct read");
#else
	(void)raw;
#endif

	uint32_t boots = 1;

	if (stamp[0] == NVM_DEMO_MAGIC)
	{
		boots = stamp[1] + 1;
		printf("persist : stamp found, previous boot %lu, data survived\r\n",
								  (unsigned long)stamp[1]);
	}
	else
	{
		printf("persist : no stamp yet, first run on this region\r\n");
	}

	uint32_t ns[2] = { NVM_DEMO_MAGIC, boots };
	uint32_t rb[2] = { 0, 0 };

	if (Mem.EraseSize() != 0)
	{
		// A clears-bits medium needs the erase before the rewrite.
		if (!Check(DemoErase(Mem, off, page) == 0, "erase the stamp page"))
		{
			return;
		}
	}
	if (!Check(DemoWrite(Mem, off, ns, sizeof(ns)) == (int)sizeof(ns), "write the stamp"))
	{
		return;
	}
	int count = Mem.Read(off, rb, sizeof(rb));
	if (!Check(count == (int)sizeof(rb) && rb[0] == NVM_DEMO_MAGIC && rb[1] == boots,
			   "stamp reads back through the driver"))
	{
		return;
	}
#if NVM_DEMO_MEDIUM == 0
	if (!Check(RawWord(raw) == NVM_DEMO_MAGIC, "stamp verified by direct memory read"))
	{
		return;
	}
#endif

	printf("persist : boot %lu stamped, power cycle and it must rise\r\n",
							  (unsigned long)boots);
}

// The checks that need real memory. Uses the scratch pages only.
static void NvmDemoVerify(Nvm &Mem)
{
	uintptr_t RegionAddr = (uintptr_t)(Mem.BaseAddress() + Mem.RegionOffset());
	uint32_t page = UnitSize(Mem);
	uint32_t scratch = page * NVM_DEMO_SCRATCH_PAGES;

	printf("region  : 0x%08lX size %lu, unit %lu, write unit %lu\r\n",
							  (unsigned long)RegionAddr, (unsigned long)Mem.Size(),
							  (unsigned long)page, (unsigned long)Mem.WriteGran());

	StampCheck(Mem);

	if (Mem.EraseSize() != 0)
	{
		// Erase, then the scratch must read all ones.
		Check(DemoErase(Mem, 0, scratch) == 0, "erase the scratch pages");
		Check(ScratchIsErased(Mem), "scratch reads all ones after erase");
	}
	else
	{
		// No erase on this medium: writing all ones puts the scratch in the
		// same known state an erase would.
		alignas(4) uint8_t ff[64];
		bool ok = true;

		memset(ff, 0xFF, sizeof(ff));
		for (uint32_t o = 0; o < scratch; o += sizeof(ff))
		{
			if (DemoWrite(Mem, o, ff, sizeof(ff)) != (int)sizeof(ff))
			{
				ok = false;
				break;
			}
		}
		Check(ok, "write ones over the scratch");
		Check(ScratchIsErased(Mem), "scratch reads all ones after writing ones");
	}

	// A write that crosses a page boundary.
	uint32_t pattern[32], readback[32];
	for (int i = 0; i < 32; i++) { pattern[i] = 0xA5000000UL | (uint32_t)i; }
	memset(readback, 0, sizeof(readback));

	uint64_t cross = page - (16 * 4);
	Check(DemoWrite(Mem, cross, pattern, sizeof(pattern)) == (int)sizeof(pattern),
		  "write across a page boundary");
	Check(Mem.Read(cross, readback, sizeof(readback)) == (int)sizeof(readback),
		  "read it back");
	Check(memcmp(pattern, readback, sizeof(pattern)) == 0,
		  "data matches across the boundary");

	// Programming either clears bits only, as a NOR flash and the nRF52
	// internal memory do, or replaces the word, as a memory that rewrites in
	// place does. Report which this one is rather than assume either.
	uint32_t half = 0x0000FFFFUL;
	uint32_t ones = 0xFFFFFFFFUL;
	uint32_t rd = 0;

	Check(DemoWrite(Mem, 0x100, &half, 4) == 4, "program a word");
	Mem.Read(0x100, &rd, 4);
	Check(rd == half, "word holds what was programmed");
	Check(DemoWrite(Mem, 0x100, &ones, 4) == 4, "program all ones over it");
	Mem.Read(0x100, &rd, 4);

	if (rd == half)
	{
		printf("medium  : clears bits only, an erase is needed to set them\r\n");
	}
	else if (rd == ones)
	{
		printf("medium  : replaces the word, it rewrites in place\r\n");
	}
	else
	{
		Check(false, "programming left the word in neither state");
	}

	// The driver rejects what the memory cannot do. What that is comes from
	// the config: a byte granular medium takes an unaligned write, one with
	// no erase has no erase to misalign.
	if (Mem.WriteGran() > 1)
	{
		Check(DemoWrite(Mem, 1, &ones, 4) == -EINVAL, "unaligned write rejected");
		Check(DemoWrite(Mem, 0, &ones, 3) == -EINVAL, "partial word write rejected");
	}
	if (Mem.EraseSize() != 0)
	{
		Check(DemoErase(Mem, 1, page) == -EINVAL, "unaligned erase rejected");
	}
	Check(Mem.Read(Mem.Size(), &rd, 4) == -EINVAL, "read past the end rejected");

#if NVM_DEMO_ASYNC
	// What the mode is for: the call establishes the operation and returns,
	// and something else drives it. A write bigger than one page needs several
	// steps, so IsBusy has real work to do rather than answering false at once.
	printf("async   : the call returns, IsBusy drives the rest\r\n");

	if (Mem.EraseSize() != 0)
	{
		Check(DemoErase(Mem, 0, scratch) == 0, "async: erase the scratch");
	}

	for (uint32_t i = 0; i < sizeof(s_Big); i++)
	{
		s_Big[i] = (uint8_t)(0xA0 ^ i);
	}

	uint32_t evtBefore = s_EvtWrite;
	uint32_t errBefore = s_EvtError;

	Check(Mem.Write(0, s_Big, sizeof(s_Big)) == (int)sizeof(s_Big),
		  "async: write accepted and returned");

	// Nothing reported yet: the operation was established, not finished.
	Check(s_EvtWrite == evtBefore, "async: no event before the work is done");

	uint32_t steps = 0;
	while (Mem.IsBusy() && steps < 100000UL)
	{
		steps++;
	}

	Check(steps > 0, "async: IsBusy had work to do");
	Check(Mem.IsBusy() == false, "async: the operation ended");
	Check(s_EvtWrite == evtBefore + 1, "async: exactly one event");
	Check(s_EvtError == errBefore, "async: no error reported");
	Check(s_EvtLastRes == 0, "async: the event says it succeeded");
	Check(s_EvtLastOff == 0 && s_EvtLastLen == sizeof(s_Big),
		  "async: the event carried the offset and length asked for");
	Check(Mem.Sync() == 0, "async: nothing left to drain");

	Check(Mem.Read(0, s_RBack, sizeof(s_RBack)) == (int)sizeof(s_RBack) &&
		  memcmp(s_RBack, s_Big, sizeof(s_Big)) == 0,
		  "async: the data landed");

	printf("async   : %lu steps, write evt %lu erase evt %lu err %lu\r\n",
							  (unsigned long)steps, (unsigned long)s_EvtWrite,
							  (unsigned long)s_EvtErase, (unsigned long)s_EvtError);

	// This block filled the scratch, and the cycle that follows expects it
	// erased: programming only clears bits, so writing a slot twice without an
	// erase in between reads back wrong.
	if (Mem.EraseSize() != 0)
	{
		Check(DemoErase(Mem, 0, scratch) == 0, "async: scratch left erased");
	}
#endif

#if NVM_DEMO_MEDIUM == 0
	NvmMcuIntrfStat_t st;
	NvmMcuIntrfGetStat(&st);

	printf("\r\n%s | ops %lu busy %lu evt %lu skipped %lu\r\n",
							  s_Fail == 0 ? "ALL PASS" : "FAILURES",
							  (unsigned long)st.Ops, (unsigned long)st.Busy,
							  (unsigned long)st.Evt, (unsigned long)st.Skipped);

	printf("path    : sd %lu slot %lu direct %lu\r\n",
							  (unsigned long)st.Sd, (unsigned long)st.Slot,
							  (unsigned long)st.Direct);
#else
	printf("\r\n%s\r\n", s_Fail == 0 ? "ALL PASS" : "FAILURES");
#endif
}

// Work out the region and mount it. Shared by both builds.
static bool NvmDemoSetup(void)
{
	NvmCfg_t cfg = {};

#if NVM_DEMO_MEDIUM == 0
	if (NvmMcuCfgInit(cfg, NVM_DEMO_REGION_NO) == false)
	{
		printf("no valid reserved NVM%d test region\r\n",
				NVM_DEMO_REGION_NO);

		return false;
	}
#else
	cfg = s_ChipCfg;
#endif

	uint64_t ceiling = cfg.TotalSize;

	uint32_t unit = cfg.EraseSize != 0 ? cfg.EraseSize : 256;

	printf("NVM size: %lu bytes, unit %lu\r\n",
							  (unsigned long)cfg.TotalSize, (unsigned long)unit);

	// Which completion mode was built, so a log says it rather than the reader
	// working it out from what is missing.
	printf("mode    : %s\r\n",
							  NVM_DEMO_ASYNC ? "interrupt, IsBusy drives the operation"
								 : "polling, the call finishes the operation");

#if NVM_DEMO_MEDIUM == 0
	// Print the selected linker allocation and the other NVM regions.
	for (int i = 0; i < NVM_REGION_MAX; i++)
	{
		uintptr_t ra = NvmRegionAddr(i);
		size_t rs = NvmRegionSize(i);

		if (ra == 0 && rs == 0)
		{
			printf("linker  : NVM%d not declared\r\n", i);
		}
		else
		{
			printf("linker  : NVM%d at %08lX size %08lX\r\n", i,
									  (unsigned long)ra, (unsigned long)rs);
		}
	}

#endif

#if NVM_DEMO_MEDIUM == 0
	// The linker already reserved this storage; no additional guard pages.
	uint64_t below = (uint64_t)unit * NVM_DEMO_REGION_PAGES;
#else
	uint64_t below = (uint64_t)unit * (NVM_DEMO_TOP_RESERVE_PAGES
									   + NVM_DEMO_REGION_PAGES);
#endif

	if (ceiling < below)
	{
		printf("device too small for the region\r\n");
		return false;
	}

	s_RegionAddr = (uintptr_t)(ceiling - below);

	uint64_t regionsize = (uint64_t)unit * NVM_DEMO_REGION_PAGES;

#if NVM_DEMO_MEDIUM == 0
	// The interface's configuration belongs to whoever builds the interface,
	// which is this application: the callback and the transfer mode are both
	// said here, once. The driver reads the mode; it does not set it.
#if NVM_DEMO_ASYNC
	if (s_MemIntrf.Init(MemIntrfEvtCB, true) == false)
#else
	if (s_MemIntrf.Init(MemIntrfEvtCB) == false)
#endif
#elif NVM_DEMO_MEDIUM == 1
	if (s_MemIntrf.Init(s_SpiCfg) == false)
#else
	if (s_MemIntrf.Init(s_I2cCfg) == false)
#endif
	{
		printf("memory interface init failed\r\n");
		return false;
	}

#if NVM_DEMO_ASYNC
	cfg.bIntEn = true;
	cfg.EvtHandler = NvmDemoEvtHandler;
#endif

	if (s_Nvm.Init(cfg, &s_MemIntrf, s_RegionAddr, regionsize) == false)
	{
		printf("Nvm init failed\r\n");
		return false;
	}

#if NVM_DEMO_MEDIUM == 1
	// The commands in use, and the status register with the Quad Enable bit,
	// which a quad command needs. A command of 0 is the single line default.
	printf("commands: read 0x%02X dummy %u, write 0x%02X\r\n",
							  cfg.RdCmd.Cmd != 0 ? cfg.RdCmd.Cmd : 0x03,
							  (unsigned)cfg.RdCmd.DummyCycle,
							  cfg.WrCmd.Cmd != 0 ? cfg.WrCmd.Cmd : 0x02);
	{
		uint8_t sr = s_Nvm.ReadStatus();

		if (cfg.QuadEnable.Reg == 1)
		{
			printf("status  : 0x%02X, quad enable bit %u %s\r\n",
									  (unsigned)sr, (unsigned)cfg.QuadEnable.Bit,
									  (sr >> cfg.QuadEnable.Bit) & 1 ? "set" : "clear");
		}
		else
		{
			printf("status  : 0x%02X, quad enable %s\r\n",
									  (unsigned)sr, cfg.QuadEnable.Reg == 0 ?
									  "not used" : "in status register 2");
		}
	}
#endif

	return true;
}

#endif

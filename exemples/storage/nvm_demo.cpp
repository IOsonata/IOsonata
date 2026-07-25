/**-------------------------------------------------------------------------
@example	nvm_demo.cpp

@brief	Nvm demo, the medium chosen by a define.

		Runs the checks that only real memory can answer: a page erases to all
		ones, a write crossing a page boundary lands correctly, programming
		clears bits and never sets them back, and a stamp survives a power
		cycle.

		One driver serves every byte addressed memory, so one demo does too.
		NVM_DEMO_MEDIUM picks what is underneath: 0 the MCU internal memory
		through NvmIntrf, 1 a SPI NOR flash, 2 an I2C EEPROM. Only the
		construction block below differs; every check is shared, and the
		checks that need an erase condition themselves on the config, not on
		the medium, because that is the driver model: a command value of 0
		means the medium does not have it.

		For the internal memory the interface works out for itself whether
		the access goes to a stack, into a timeslot, or straight at the
		controller, so there is nothing here to select. Above it sits the ordinary Nvm driver, the same one a SPI
		flash or an I2C EEPROM uses. Build with NVM_DEMO_BLE set to 1 to
		run the same checks with a BLE stack up and advertising, which is what
		puts the radio in contention for the memory. Everything above the main
		function is shared between the two.

		The region is three pages placed at the top of the memory the
		application owns, below a few reserved pages. The ceiling comes from
		NvmIntrfCeiling(), because the application area does not always run to
		the end of the device: a stack image or a reserved partition can sit
		at the top, and the target implementation knows where. The first two
		pages are the scratch the checks use, so a write can cross a
		boundary; the third holds the stamp and is never erased by the
		checks. The region must hold nothing else: check it against the
		application image end in the map file. The demo prints the region
		before it erases anything.

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

#include "istddef.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "idelay.h"
#include "board.h"

#include "storage/nvm.h"

// What is underneath: 0 the MCU internal memory, 1 a SPI NOR flash
// (MX25R3235F), 2 an I2C EEPROM (CAT24C32). Everything below the
// construction blocks is shared.
#ifndef NVM_DEMO_MEDIUM
#define NVM_DEMO_MEDIUM			0
#endif

#if NVM_DEMO_MEDIUM == 0
#include "storage/nvm_intrf.h"
#elif NVM_DEMO_MEDIUM == 1
#include "coredev/spi.h"
#include "storage/flash.h"		// for the FLASH_CMD_READ/WRITE opcodes
#elif NVM_DEMO_MEDIUM == 2
#include "coredev/i2c.h"
#endif

// 1 : bring up a BLE stack and repeat the exercise while advertising.
#ifndef NVM_DEMO_BLE
#define NVM_DEMO_BLE			1
#endif

#if NVM_DEMO_BLE
#include "app_evt_handler.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"
#else
#endif

#include "syslog.h"

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

// Two scratch pages the checks use, plus one for the stamp.
#ifndef NVM_DEMO_SCRATCH_PAGES
#define NVM_DEMO_SCRATCH_PAGES	2
#endif

#define NVM_DEMO_REGION_PAGES	(NVM_DEMO_SCRATCH_PAGES + 1)

// Pages at the top of the memory the demo stays away from.
#ifndef NVM_DEMO_TOP_RESERVE_PAGES
#define NVM_DEMO_TOP_RESERVE_PAGES	3
#endif

#define NVM_DEMO_MAGIC			0x4E564D31UL	// "NVM1"

static const IOPinCfg_t s_UartPins[] = UART_PINS;

// TX FIFO large enough for the burst of check results printed back to back;
// the driver's built in fallback is only 32 bytes.
#define UART_TXFIFO_MEMSIZE			CFIFO_MEMSIZE(256)

alignas(4) static uint8_t s_UartTxFifoMem[UART_TXFIFO_MEMSIZE];

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

static NvmIntrf s_MemIntrf;

#elif NVM_DEMO_MEDIUM == 1

static const char s_MediumName[] = "SPI NOR flash";

static const IOPinCfg_t s_SpiPins[] = SPI_PINS_CFG;

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = 8000000,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,
	.bIntEn = true,
	.IntPrio = 6,
	.EvtCB = NULL
};

static SPI s_MemIntrf;

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

static void Check(bool Cond, const char *pMsg)
{
	if (Cond)
	{
		g_Uart.printf("  ok   : %s\r\n", pMsg);
	}
	else
	{
		s_Fail++;
		g_Uart.printf("  FAIL : %s\r\n", pMsg);
	}
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
static void StampCheck(Nvm &Mem, uintptr_t RegionAddr)
{
	uint32_t page = UnitSize(Mem);
	uint64_t off = (uint64_t)page * NVM_DEMO_SCRATCH_PAGES;
	uintptr_t raw = RegionAddr + (uintptr_t)off;
	uint32_t stamp[2] = { 0, 0 };

	Mem.Read(off, stamp, sizeof(stamp));

#if NVM_DEMO_MEDIUM == 0
	g_Uart.printf("raw     : [0x%08lX] = 0x%08lX 0x%08lX before anything is touched\r\n",
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
		g_Uart.printf("persist : stamp found, previous boot %lu, data survived\r\n",
					  (unsigned long)stamp[1]);
	}
	else
	{
		g_Uart.printf("persist : no stamp yet, first run on this region\r\n");
	}

	uint32_t ns[2] = { NVM_DEMO_MAGIC, boots };
	uint32_t rb[2] = { 0, 0 };

	if (Mem.EraseSize() != 0)
	{
		// A clears-bits medium needs the erase before the rewrite.
		Check(Mem.Erase(off, page) == 0, "erase the stamp page");
	}
	Check(Mem.Write(off, ns, sizeof(ns)) == (int)sizeof(ns), "write the stamp");
	Mem.Read(off, rb, sizeof(rb));
	Check(rb[0] == NVM_DEMO_MAGIC && rb[1] == boots,
		  "stamp reads back through the driver");
#if NVM_DEMO_MEDIUM == 0
	Check(RawWord(raw) == NVM_DEMO_MAGIC,
		  "the stamp is in the memory, not only in the driver view");
#endif

	g_Uart.printf("persist : boot %lu stamped, power cycle and it must rise\r\n",
				  (unsigned long)boots);
}

// The checks that need real memory. Uses the scratch pages only.
static void NvmDemoVerify(Nvm &Mem, uintptr_t RegionAddr)
{
	uint32_t page = UnitSize(Mem);
	uint32_t scratch = page * NVM_DEMO_SCRATCH_PAGES;

	g_Uart.printf("region  : 0x%08lX size %lu, unit %lu, write unit %lu\r\n",
				  (unsigned long)RegionAddr, (unsigned long)Mem.Size(),
				  (unsigned long)page, (unsigned long)Mem.WriteGran());

	StampCheck(Mem, RegionAddr);

	if (Mem.EraseSize() != 0)
	{
		// Erase, then the scratch must read all ones.
		Check(Mem.Erase(0, scratch) == 0, "erase the scratch pages");
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
			if (Mem.Write(o, ff, sizeof(ff)) != (int)sizeof(ff))
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
	Check(Mem.Write(cross, pattern, sizeof(pattern)) == (int)sizeof(pattern),
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

	Check(Mem.Write(0x100, &half, 4) == 4, "program a word");
	Mem.Read(0x100, &rd, 4);
	Check(rd == half, "word holds what was programmed");
	Check(Mem.Write(0x100, &ones, 4) == 4, "program all ones over it");
	Mem.Read(0x100, &rd, 4);

	if (rd == half)
	{
		g_Uart.printf("medium  : clears bits only, an erase is needed to set them\r\n");
	}
	else if (rd == ones)
	{
		g_Uart.printf("medium  : replaces the word, it rewrites in place\r\n");
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
		Check(Mem.Write(1, &ones, 4) == -EINVAL, "unaligned write rejected");
		Check(Mem.Write(0, &ones, 3) == -EINVAL, "partial word write rejected");
	}
	if (Mem.EraseSize() != 0)
	{
		Check(Mem.Erase(1, page) == -EINVAL, "unaligned erase rejected");
	}
	Check(Mem.Read(Mem.Size(), &rd, 4) == -EINVAL, "read past the end rejected");

#if NVM_DEMO_MEDIUM == 0
	NvmIntrfStat_t st;
	NvmIntrfGetStat(&st);

	g_Uart.printf("\r\n%s | ops %lu busy %lu evt %lu skipped %lu\r\n",
				  s_Fail == 0 ? "ALL PASS" : "FAILURES",
				  (unsigned long)st.Ops, (unsigned long)st.Busy,
				  (unsigned long)st.Evt, (unsigned long)st.Skipped);
#else
	g_Uart.printf("\r\n%s\r\n", s_Fail == 0 ? "ALL PASS" : "FAILURES");
#endif
}

// Work out the region and mount it. Shared by both builds.
static bool NvmDemoSetup(void)
{
	NvmCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));

#if NVM_DEMO_MEDIUM == 0
	NvmIntrfCfg(cfg);

	// The application area does not always run to the end of the device; the
	// target implementation knows where it ends.
	uint64_t ceiling = NvmIntrfCeiling();
#else
	cfg = s_ChipCfg;

	// A chip is all ours; the region rides at its top.
	uint64_t ceiling = cfg.TotalSize;
#endif

	uint32_t unit = cfg.EraseSize != 0 ? cfg.EraseSize : 256;

	g_Uart.printf("device  : %lu bytes, unit %lu\r\n",
				  (unsigned long)cfg.TotalSize, (unsigned long)unit);

	s_RegionAddr = (uintptr_t)(ceiling
			- (uint64_t)unit * (NVM_DEMO_TOP_RESERVE_PAGES
								 + NVM_DEMO_REGION_PAGES));

	uint64_t regionsize = (uint64_t)unit * NVM_DEMO_REGION_PAGES;

#if NVM_DEMO_MEDIUM == 0
	if (s_MemIntrf.Init() == false)
#elif NVM_DEMO_MEDIUM == 1
	if (s_MemIntrf.Init(s_SpiCfg) == false)
#else
	if (s_MemIntrf.Init(s_I2cCfg) == false)
#endif
	{
		g_Uart.printf("memory interface init failed\r\n");
		return false;
	}

	if (s_Nvm.Init(cfg, &s_MemIntrf, s_RegionAddr, regionsize) == false)
	{
		g_Uart.printf("Nvm init failed\r\n");
		return false;
	}

	return true;
}

#if NVM_DEMO_BLE

// ---------------------------------------------------------------------------
// With a stack up: the same checks once, then a cycle per advertising timeout
// so the radio is contending for the memory.
// ---------------------------------------------------------------------------

#define DEVICE_NAME					"NvmDemo"
#define APP_ADV_INTERVAL_MSEC		50
#define APP_ADV_TIMEOUT_MSEC		1000

#ifndef NVM_DEMO_WRITES_PER_CYCLE
#define NVM_DEMO_WRITES_PER_CYCLE	16
#endif

#ifndef NVM_DEMO_SLOTS
#define NVM_DEMO_SLOTS			64
#endif

static uint32_t g_AdvCnt = 0;
static bool s_Ready = false;
static uint32_t s_Cycles = 0;
static uint32_t s_Writes = 0;
static uint32_t s_Slot = 0;

const BtAppCfg_t s_BtAppCfg = {
	.Role = BTAPP_ROLE_BROADCASTER,
	.CentLinkCount = 0,
	.PeriLinkCount = 1,
	.pDevName = (char*)DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.Appearance = BT_APPEAR_COMPUTER_WEARABLE,
	.pAdvManData = (uint8_t*)&g_AdvCnt,
	.AdvManDataLen = sizeof(g_AdvCnt),
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.AdvInterval = APP_ADV_INTERVAL_MSEC,
	.AdvTimeout = APP_ADV_TIMEOUT_MSEC,
	.TxPower = 0,
};

// Runs from the BtAppRun loop, so blocking here is safe while advertising
// continues.
static void NvmCycleHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	if (s_Ready == false)
	{
		return;
	}

	uint32_t page = UnitSize(s_Nvm);

	for (int i = 0; i < NVM_DEMO_WRITES_PER_CYCLE; i++)
	{
		if (s_Slot >= NVM_DEMO_SLOTS)
		{
			// A rewrite in place medium wraps by overwriting.
			if (s_Nvm.EraseSize() != 0 && s_Nvm.Erase(0, page) != 0)
			{
				s_Fail++;
			}
			s_Slot = 0;
		}

		uint32_t off = s_Slot * 4;
		uint32_t val = NVM_DEMO_MAGIC ^ s_Writes;
		uint32_t rd = 0;

		if (s_Nvm.Write(off, &val, 4) != 4 ||
			s_Nvm.Read(off, &rd, 4) != 4 || rd != val)
		{
			s_Fail++;
		}

		s_Slot++;
		s_Writes++;
	}

	s_Cycles++;

	if ((s_Cycles % 5) == 0)
	{
#if NVM_DEMO_MEDIUM == 0
		NvmIntrfStat_t st;
		NvmIntrfGetStat(&st);

		// evt is the one that matters: a result only arrives as an event
		// while an arbitrating stack is running.
		g_Uart.printf("cycles %lu writes %lu fails %lu | ops %lu busy %lu evt %lu\r\n",
					  (unsigned long)s_Cycles, (unsigned long)s_Writes,
					  (unsigned long)s_Fail, (unsigned long)st.Ops,
					  (unsigned long)st.Busy, (unsigned long)st.Evt);
#else
		g_Uart.printf("cycles %lu writes %lu fails %lu\r\n",
					  (unsigned long)s_Cycles, (unsigned long)s_Writes,
					  (unsigned long)s_Fail);
#endif
	}
}

void BtAppInitUserData()
{
#if NVM_DEMO_MEDIUM == 0
	// The target implementation delivers completions on its own, so the wait
	// needs no help; NULL spends it in a short delay.
	NvmIntrfSetWait(NULL, 5000);
#endif

	if (NvmDemoSetup() == false)
	{
		return;
	}

	NvmDemoVerify(s_Nvm, s_RegionAddr);

	// The checks left the scratch in a known state; start the cycle after it.
	s_Slot = 0;
	s_Writes = 0;
	s_Ready = true;

	g_Uart.printf("advertising now, %d writes per cycle\r\n",
				  NVM_DEMO_WRITES_PER_CYCLE);
}

// Runs inside the stack event dispatch, so it only queues the work.
void BtAppAdvTimeoutHandler()
{
	g_AdvCnt++;

	BtAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);

	AppEvtHandlerQue(0, NULL, NvmCycleHandler);
}

int main()
{
	g_Uart.Init(s_UartCfg);

	g_Uart.printf("\r\nNvm demo on the %s, with a stack up\r\n", s_MediumName);

	SysLogGetInstance()->Init(g_Uart);

	BtAppInit(&s_BtAppCfg);

	BtAppRun();

	return 0;
}

#else

// ---------------------------------------------------------------------------
// No stack: the checks once, then idle.
// ---------------------------------------------------------------------------

int main()
{
	g_Uart.Init(s_UartCfg);

	g_Uart.printf("\r\nNvm demo on the %s\r\n", s_MediumName);

	if (NvmDemoSetup())
	{
		NvmDemoVerify(s_Nvm, s_RegionAddr);
	}

	while (true)
	{
		msDelay(1000);
	}
}

#endif	// NVM_DEMO_BLE

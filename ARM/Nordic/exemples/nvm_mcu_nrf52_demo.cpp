/**-------------------------------------------------------------------------
@example	nvm_mcu_nrf52_demo.cpp

@brief	NvmMcu demo on the nRF52 internal memory.

		Runs the checks that only real memory can answer: a page erases to all
		ones, a write crossing a page boundary lands correctly, programming
		clears bits and never sets them back, and a stamp survives a power
		cycle.

		The driver works out for itself whether the access goes to a
		SoftDevice, into a timeslot, or straight at the controller, so there is
		nothing here to select. Build with NVM_MCU_DEMO_BLE set to 1 to run the
		same checks with a stack up and advertising, which is what puts the
		radio in contention for the memory. Everything above the main function
		is shared between the two.

		The region is three pages worked out from the device at run time and
		placed below the pages a bond store would take. The first two are the
		scratch the checks use, so a write can cross a boundary; the third
		holds the stamp and is never erased by the checks. It must still hold
		nothing else: check it against the application image end in the map
		file. The demo prints the region before it erases anything.

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

#include "nvm_mcu_nrf52.h"

// 1 : bring up a BLE stack and repeat the exercise while advertising.
#ifndef NVM_MCU_DEMO_BLE
#define NVM_MCU_DEMO_BLE			1
#endif

#if NVM_MCU_DEMO_BLE
#include "app_evt_handler.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"
#include "syslog.h"
#include "nrf_sdh_soc.h"
#endif

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

// Two scratch pages the checks use, plus one for the stamp.
#ifndef NVM_MCU_DEMO_SCRATCH_PAGES
#define NVM_MCU_DEMO_SCRATCH_PAGES	2
#endif

#define NVM_MCU_DEMO_REGION_PAGES	(NVM_MCU_DEMO_SCRATCH_PAGES + 1)

// Pages at the top of the memory the demo stays away from.
#ifndef NVM_MCU_DEMO_TOP_RESERVE_PAGES
#define NVM_MCU_DEMO_TOP_RESERVE_PAGES	3
#endif

#define NVM_MCU_DEMO_MAGIC			0x4E564D31UL	// "NVM1"

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
	.IntPrio = 6,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = 0,
	.pTxMem = NULL,
	.bDMAMode = true,
};

UART g_Uart;

static NvmMcu s_Nvm;
static uintptr_t s_RegionAddr = 0;
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
		g_Uart.printf("  ok   : %s\r\n", pMsg);
	}
	else
	{
		s_Fail++;
		g_Uart.printf("  FAIL : %s\r\n", pMsg);
	}
}

// Report whether the scratch pages read erased.
static bool ScratchIsErased(NvmIO &Mem)
{
	uint32_t buf[64];
	uint64_t end = (uint64_t)Mem.EraseSize() * NVM_MCU_DEMO_SCRATCH_PAGES;
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
static void StampCheck(NvmIO &Mem, uintptr_t RegionAddr)
{
	uint32_t page = Mem.EraseSize();
	uint64_t off = (uint64_t)page * NVM_MCU_DEMO_SCRATCH_PAGES;
	uintptr_t raw = RegionAddr + (uintptr_t)off;
	uint32_t stamp[2] = { 0, 0 };

	Mem.Read(off, stamp, sizeof(stamp));

	g_Uart.printf("raw     : [0x%08lX] = 0x%08lX 0x%08lX before anything is touched\r\n",
				  (unsigned long)raw, (unsigned long)RawWord(raw),
				  (unsigned long)RawWord(raw + 4));

	Check(stamp[0] == RawWord(raw), "driver reads the same memory as a direct read");

	uint32_t boots = 1;

	if (stamp[0] == NVM_MCU_DEMO_MAGIC)
	{
		boots = stamp[1] + 1;
		g_Uart.printf("persist : stamp found, previous boot %lu, data survived\r\n",
					  (unsigned long)stamp[1]);
	}
	else
	{
		g_Uart.printf("persist : no stamp yet, first run on this region\r\n");
	}

	uint32_t ns[2] = { NVM_MCU_DEMO_MAGIC, boots };
	uint32_t rb[2] = { 0, 0 };

	Check(Mem.Erase(off, page) == 0, "erase the stamp page");
	Check(Mem.Write(off, ns, sizeof(ns)) == (int)sizeof(ns), "write the stamp");
	Mem.Read(off, rb, sizeof(rb));
	Check(rb[0] == NVM_MCU_DEMO_MAGIC && rb[1] == boots,
		  "stamp reads back through the driver");
	Check(RawWord(raw) == NVM_MCU_DEMO_MAGIC,
		  "the stamp is in the memory, not only in the driver view");

	g_Uart.printf("persist : boot %lu stamped, power cycle and it must rise\r\n",
				  (unsigned long)boots);
}

// The checks that need real memory. Uses the scratch pages only.
static void NvmDemoVerify(NvmIO &Mem, uintptr_t RegionAddr)
{
	uint32_t page = Mem.EraseSize();
	uint32_t scratch = page * NVM_MCU_DEMO_SCRATCH_PAGES;

	g_Uart.printf("region  : 0x%08lX size %lu, page %lu, write unit %lu\r\n",
				  (unsigned long)RegionAddr, (unsigned long)Mem.Size(),
				  (unsigned long)page, (unsigned long)Mem.WriteGran());

	StampCheck(Mem, RegionAddr);

	// Erase, then the scratch must read all ones.
	Check(Mem.Erase(0, scratch) == 0, "erase the scratch pages");
	Check(ScratchIsErased(Mem), "scratch reads all ones after erase");

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

	// Programming clears bits and cannot set them back without an erase.
	uint32_t half = 0x0000FFFFUL;
	uint32_t ones = 0xFFFFFFFFUL;
	uint32_t rd = 0;

	Check(Mem.Write(0x100, &half, 4) == 4, "program a word");
	Mem.Read(0x100, &rd, 4);
	Check(rd == half, "word holds what was programmed");
	Check(Mem.Write(0x100, &ones, 4) == 4, "program all ones over it");
	Mem.Read(0x100, &rd, 4);
	Check(rd == half, "bits did not come back without an erase");

	// The driver rejects what the memory cannot do.
	Check(Mem.Write(1, &ones, 4) == -EINVAL, "unaligned write rejected");
	Check(Mem.Write(0, &ones, 3) == -EINVAL, "partial word write rejected");
	Check(Mem.Erase(1, page) == -EINVAL, "unaligned erase rejected");
	Check(Mem.Read(Mem.Size(), &rd, 4) == -EINVAL, "read past the end rejected");

	NvmMcuNrf52Stat_t st;
	NvmMcuNrf52GetStat(&st);

	g_Uart.printf("\r\n%s | ops %lu busy %lu evt %lu skipped %lu\r\n",
				  s_Fail == 0 ? "ALL PASS" : "FAILURES",
				  (unsigned long)st.Ops, (unsigned long)st.Busy,
				  (unsigned long)st.Evt, (unsigned long)st.Skipped);
}

// Work out the region and mount it. Shared by both builds.
static bool NvmDemoSetup(void)
{
	NvmCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	NvmMcuNrf52Cfg(cfg);

	g_Uart.printf("device  : %lu bytes, page %lu\r\n",
				  (unsigned long)cfg.TotalSize, (unsigned long)cfg.EraseSize);

	s_RegionAddr = (uintptr_t)(cfg.TotalSize
			- (uint64_t)cfg.EraseSize * (NVM_MCU_DEMO_TOP_RESERVE_PAGES
										 + NVM_MCU_DEMO_REGION_PAGES));

	uint64_t regionsize = (uint64_t)cfg.EraseSize * NVM_MCU_DEMO_REGION_PAGES;

	if (s_Nvm.Init(cfg, NvmMcuNrf52Op(), s_RegionAddr, regionsize) == false)
	{
		g_Uart.printf("NvmMcu init failed\r\n");
		return false;
	}

	return true;
}

#if NVM_MCU_DEMO_BLE

// ---------------------------------------------------------------------------
// With a stack up: the same checks once, then a cycle per advertising timeout
// so the radio is contending for the memory.
// ---------------------------------------------------------------------------

#define DEVICE_NAME					"NvmMcu"
#define APP_ADV_INTERVAL_MSEC		50
#define APP_ADV_TIMEOUT_MSEC		1000

#ifndef NVM_MCU_DEMO_WRITES_PER_CYCLE
#define NVM_MCU_DEMO_WRITES_PER_CYCLE	16
#endif

#ifndef NVM_MCU_DEMO_SLOTS
#define NVM_MCU_DEMO_SLOTS			64
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

// SoC events arrive through the SoftDevice handler; hand every one over.
static void NvmDemoSocObserver(uint32_t SysEvt, void *pCtx)
{
	(void)pCtx;

	NvmMcuNrf52SocEvt(SysEvt);
}

NRF_SDH_SOC_OBSERVER(s_NvmDemoSocObs, 0, NvmDemoSocObserver, NULL);

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

	uint32_t page = s_Nvm.EraseSize();

	for (int i = 0; i < NVM_MCU_DEMO_WRITES_PER_CYCLE; i++)
	{
		if (s_Slot >= NVM_MCU_DEMO_SLOTS)
		{
			if (s_Nvm.Erase(0, page) != 0)
			{
				s_Fail++;
			}
			s_Slot = 0;
		}

		uint32_t off = s_Slot * 4;
		uint32_t val = NVM_MCU_DEMO_MAGIC ^ s_Writes;
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
		NvmMcuNrf52Stat_t st;
		NvmMcuNrf52GetStat(&st);

		// evt is the one that matters: a result only arrives as an event while
		// the SoftDevice is running.
		g_Uart.printf("cycles %lu writes %lu fails %lu | ops %lu busy %lu evt %lu\r\n",
					  (unsigned long)s_Cycles, (unsigned long)s_Writes,
					  (unsigned long)s_Fail, (unsigned long)st.Ops,
					  (unsigned long)st.Busy, (unsigned long)st.Evt);
	}
}

void BtAppInitUserData()
{
	// The SoftDevice handler delivers the completion from an interrupt, so the
	// wait needs no help.
	NvmMcuNrf52SetWait(NULL, 5000);

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
				  NVM_MCU_DEMO_WRITES_PER_CYCLE);
}

// Runs inside the SoftDevice event dispatch, so it only queues the work.
void BtAppAdvTimeoutHandler()
{
	g_AdvCnt++;

	BtAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);

	AppEvtHandlerQue(0, NULL, NvmCycleHandler);
}

int main()
{
	g_Uart.Init(s_UartCfg);

	g_Uart.printf("\r\nNvmMcu demo on nRF52 internal memory, with a stack up\r\n");

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

	g_Uart.printf("\r\nNvmMcu demo on nRF52 internal memory\r\n");

	if (NvmDemoSetup())
	{
		NvmDemoVerify(s_Nvm, s_RegionAddr);
	}

	while (true)
	{
		msDelay(1000);
	}
}

#endif	// NVM_MCU_DEMO_BLE

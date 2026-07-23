/**-------------------------------------------------------------------------
@example	nvm_mcu_sd_demo.cpp

@brief	NvmMcu over a full SoftDevice, exercised while the radio is running.

		The standalone NvmMcu demo runs with the SoftDevice present but not
		enabled, so the memory calls finish before they return. This one
		advertises, which means the SoftDevice is enabled and owns the memory:
		a request can be refused while the radio holds it, and the result
		arrives later as a SoC event. That is the path this example is here to
		exercise.

		Two hooks do the work. BtAppInitUserData runs once after the stack is
		up, in thread context, and verifies the region. BtAppAdvTimeoutHandler
		then fires about once a second, but it runs inside the SoftDevice event
		dispatch, so it must not block: it queues the work with
		AppEvtHandlerQue and the cycle runs from the BtAppRun loop instead,
		where blocking is safe and the radio keeps advertising.

		SoC events reach the operations through an observer registered here, so
		nothing polls sd_evt_get behind the SoftDevice handler's back.

		Watch the busy count in the output. A non zero value is the proof that
		requests were refused while the radio held the memory and the retry
		path recovered.

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
#include "app_evt_handler.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "syslog.h"
#include "board.h"

#include "nrf_sdh_soc.h"
#include "nrf_sdm.h"

#include "nvm_mcu_sd.h"

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

#define DEVICE_NAME					"NvmMcuSd"
#define APP_ADV_INTERVAL_MSEC		50
#define APP_ADV_TIMEOUT_MSEC		1000

// Pages the exercise uses, placed below the pages a bond store would take.
#ifndef NVM_MCU_DEMO_REGION_PAGES
#define NVM_MCU_DEMO_REGION_PAGES	2
#endif

#ifndef NVM_MCU_DEMO_TOP_RESERVE_PAGES
#define NVM_MCU_DEMO_TOP_RESERVE_PAGES	3
#endif

// Priority of the SoC observer. The SDK allows 0 to
// NRF_SDH_SOC_OBSERVER_PRIO_LEVELS - 1.
#ifndef NVM_MCU_DEMO_SOC_PRIO
#define NVM_MCU_DEMO_SOC_PRIO		0
#endif

#define NVM_MCU_DEMO_MAGIC			0x4E564D32UL	// "NVM2"

// Words written per cycle, and words used in a page before it is erased
// again. More of both means more time holding the memory, which is what makes
// a refusal by the radio likely enough to observe.
#ifndef NVM_MCU_DEMO_WRITES_PER_CYCLE
#define NVM_MCU_DEMO_WRITES_PER_CYCLE	16
#endif

#ifndef NVM_MCU_DEMO_SLOTS
#define NVM_MCU_DEMO_SLOTS			64
#endif

static uint32_t g_AdvCnt = 0;

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
static bool s_NvmReady = false;
static uintptr_t s_RegionAddr = 0;

// Running tally of the exercise that runs while advertising.
static uint32_t s_Cycles = 0;
static uint32_t s_Writes = 0;
static uint32_t s_Errors = 0;
static uint32_t s_Slot = 0;

// SoC events arrive through the SoftDevice handler. Hand every one to the NVM
// operations; those that are not memory results are ignored there.
static void NvmDemoSocObserver(uint32_t SysEvt, void *pCtx)
{
	(void)pCtx;

	NvmMcuSdSocEvt(SysEvt);
}

NRF_SDH_SOC_OBSERVER(s_NvmDemoSocObs, NVM_MCU_DEMO_SOC_PRIO, NvmDemoSocObserver, NULL);

// One write and read back, and a page erase when the page is full. Runs from
// the BtAppRun loop, so blocking here is safe while advertising continues.
static void NvmCycleHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	if (s_NvmReady == false)
	{
		return;
	}

	uint32_t page = s_Nvm.EraseSize();

	for (int i = 0; i < NVM_MCU_DEMO_WRITES_PER_CYCLE; i++)
	{
		if (s_Slot >= NVM_MCU_DEMO_SLOTS)
		{
			int res = s_Nvm.Erase(0, page);
			if (res != 0)
			{
				s_Errors++;
				g_Uart.printf("erase failed %d\r\n", res);
			}
			s_Slot = 0;
		}

		uint32_t off = s_Slot * 4;
		uint32_t val = NVM_MCU_DEMO_MAGIC ^ s_Writes;
		uint32_t rd = 0;

		int res = s_Nvm.Write(off, &val, 4);
		if (res != 4)
		{
			s_Errors++;
			g_Uart.printf("write failed %d at slot %lu\r\n", res,
						  (unsigned long)s_Slot);
		}
		else if (s_Nvm.Read(off, &rd, 4) != 4 || rd != val)
		{
			s_Errors++;
			g_Uart.printf("read back mismatch at slot %lu\r\n",
						  (unsigned long)s_Slot);
		}

		s_Slot++;
		s_Writes++;
	}

	s_Cycles++;

	if ((s_Cycles % 5) == 0)
	{
		// evt is the count that matters: it is the number of results that
		// arrived as a SoC event, which only happens while the SoftDevice is
		// running. sync counts the ones that finished without an event.
		g_Uart.printf("cycles %lu writes %lu errors %lu | busy %lu evt %lu sync %lu\r\n",
					  (unsigned long)s_Cycles, (unsigned long)s_Writes,
					  (unsigned long)s_Errors,
					  (unsigned long)NvmMcuSdBusyCount(),
					  (unsigned long)NvmMcuSdEvtCount(),
					  (unsigned long)NvmMcuSdSyncCount());
	}
}

// Runs once after the stack is up, in thread context.
void BtAppInitUserData()
{
	NvmCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	NvmMcuSdCfg(cfg);

	// The SoftDevice handler delivers the completion event from an interrupt,
	// so the wait needs no help from an idle callback.
	NvmMcuSdSetIdle(NULL, 5000);

	s_RegionAddr = (uintptr_t)(cfg.TotalSize
			- (uint64_t)cfg.EraseSize * (NVM_MCU_DEMO_TOP_RESERVE_PAGES
										 + NVM_MCU_DEMO_REGION_PAGES));

	uint64_t regionsize = (uint64_t)cfg.EraseSize * NVM_MCU_DEMO_REGION_PAGES;

	g_Uart.printf("region  : 0x%08lX, %u pages of %lu\r\n",
				  (unsigned long)s_RegionAddr, NVM_MCU_DEMO_REGION_PAGES,
				  (unsigned long)cfg.EraseSize);

	if (s_Nvm.Init(cfg, NvmMcuSdOp(), s_RegionAddr, regionsize) == false)
	{
		g_Uart.printf("NvmMcu init failed\r\n");
		return;
	}

	uint32_t page = s_Nvm.EraseSize();

	// The second page holds a boot stamp and is never touched by the exercise,
	// so it shows whether a write made through the SoftDevice really reached
	// the memory and stayed there over a power cycle. The raw read is taken
	// straight from the address, so a stale driver view cannot fake it.
	uint32_t stamp[2] = { 0, 0 };

	s_Nvm.Read(page, stamp, sizeof(stamp));

	g_Uart.printf("raw     : [0x%08lX] = 0x%08lX 0x%08lX\r\n",
				  (unsigned long)(s_RegionAddr + page),
				  (unsigned long)*(volatile uint32_t *)(s_RegionAddr + page),
				  (unsigned long)*(volatile uint32_t *)(s_RegionAddr + page + 4));

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

	// Leave the next stamp behind, through the SoftDevice like everything else.
	uint32_t ns[2] = { NVM_MCU_DEMO_MAGIC, boots };
	uint32_t rb[2] = { 0, 0 };

	if (s_Nvm.Erase(page, page) != 0 ||
		s_Nvm.Write(page, ns, sizeof(ns)) != (int)sizeof(ns))
	{
		g_Uart.printf("stamp write failed\r\n");
		return;
	}

	s_Nvm.Read(page, rb, sizeof(rb));

	if (rb[0] != NVM_MCU_DEMO_MAGIC || rb[1] != boots ||
		*(volatile uint32_t *)(s_RegionAddr + page) != NVM_MCU_DEMO_MAGIC)
	{
		g_Uart.printf("stamp did not read back\r\n");
		return;
	}

	g_Uart.printf("persist : boot %lu stamped, power cycle and it must rise\r\n",
				  (unsigned long)boots);

	// A first pass on the churn page, with the SoftDevice enabled and the radio
	// still idle.
	uint32_t val = NVM_MCU_DEMO_MAGIC;
	uint32_t rd = 0;

	if (s_Nvm.Erase(0, page) != 0)
	{
		g_Uart.printf("initial erase failed\r\n");
		return;
	}
	if (s_Nvm.Write(0, &val, 4) != 4 || s_Nvm.Read(0, &rd, 4) != 4 || rd != val)
	{
		g_Uart.printf("initial write failed\r\n");
		return;
	}

	s_Slot = 1;
	s_Writes = 1;
	s_NvmReady = true;

	uint8_t sden = 0;
	(void)sd_softdevice_is_enabled(&sden);

	g_Uart.printf("initial pass ok, softdevice %s, evt %lu sync %lu\r\n",
				  sden ? "running" : "STOPPED",
				  (unsigned long)NvmMcuSdEvtCount(),
				  (unsigned long)NvmMcuSdSyncCount());
	g_Uart.printf("advertising now, %d writes per cycle, erase every %d\r\n",
				  NVM_MCU_DEMO_WRITES_PER_CYCLE,
				  NVM_MCU_DEMO_SLOTS / NVM_MCU_DEMO_WRITES_PER_CYCLE);
}

// Advertising timeout. This runs inside the SoftDevice event dispatch, so it
// only queues the work; the cycle itself runs from the BtAppRun loop.
void BtAppAdvTimeoutHandler()
{
	g_AdvCnt++;

	BtAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);

	AppEvtHandlerQue(0, NULL, NvmCycleHandler);
}

int main()
{
	g_Uart.Init(s_UartCfg);

	g_Uart.printf("\r\nNvmMcu over SoftDevice, exercised while advertising\r\n");

	SysLogGetInstance()->Init(g_Uart);

	BtAppInit(&s_BtAppCfg);

	BtAppRun();

	return 0;
}

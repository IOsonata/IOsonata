/**-------------------------------------------------------------------------
@example	ble_nvm_test.cpp

@brief	Exercise NVM operations while Bluetooth advertising is active.

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
#include "istddef.h"
#include "stddev.h"
#include "../storage/nvm_demo_checks.h"
#if NVM_DEMO_MEDIUM == 0
#include "storage/nvm_mcu_intrf.h"
#endif
#include "app_evt_handler.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"

#define DEVICE_NAME					"NvmDemo"
#define APP_ADV_INTERVAL_MSEC		50
#define APP_ADV_TIMEOUT_MSEC		1000

#ifndef NVM_DEMO_WRITES_PER_CYCLE
#define NVM_DEMO_WRITES_PER_CYCLE	16
#endif

#ifndef NVM_DEMO_SLOTS
#define NVM_DEMO_SLOTS			64
#endif

// The library default is 4 slots. The async cycle puts the pump back on the
// queue on every visit, thousands of times a cycle, alongside whatever the
// stack posts, so 4 is not enough and a refusal costs a whole cycle.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

static uint32_t s_AdvCnt = 0;
static bool s_Ready = false;
static uint32_t s_Cycles = 0;
static uint32_t s_Writes = 0;
static uint32_t s_Slot = 0;

#if NVM_DEMO_ASYNC
// The async cycle's own state: how many slots of this cycle are left, how many
// times it handed the loop back, and the slot it started so the next visit can
// read it back.
static uint32_t s_Pending = 0;
static uint32_t s_Requeues = 0;
static uint32_t s_Dropped = 0;
static uint32_t s_PendOff = 0;
static uint32_t s_PendVal = 0;
static bool s_PendCheck = false;

// Requeues since the pump last got past IsBusy, and the longest such wait
// the run has seen. The pump goes round about 675 times a second, and one
// write costs about one advertising interval, so the ordinary wait is a few
// tens of visits. Recording the longest and reporting it with the cycle is
// what says whether that ever got worse; printing at the time turned a slow
// write into a burst of lines, and the burst is what pushed the report out
// of the UART.
static uint32_t s_StallRq = 0;
static uint32_t s_StallMax = 0;

// Waits longer than NVM_DEMO_STALL_LONG. A count rather than a line each
// time, for the same reason.
#ifndef NVM_DEMO_STALL_LONG
#define NVM_DEMO_STALL_LONG		200UL
#endif
static uint32_t s_StallLong = 0;

// A wait this long is not slow scheduling, it is an operation that is not
// going to finish. About thirty seconds at the rate above.
#ifndef NVM_DEMO_STALL_STUCK
#define NVM_DEMO_STALL_STUCK	20000UL
#endif

// Skips by NvmCycleHandler. A memory that never finishes and an advertising
// timeout that stopped arriving both show as no output at all otherwise.
static uint32_t s_Skips = 0;
#endif

const BtAppCfg_t s_BtAppCfg = {
	.Role = BTAPP_ROLE_BROADCASTER,
	.PeriphDevMax = 0,
	.CentralDevMax = 1,
	.pDevName = (char*)DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.Appearance = BT_APPEAR_COMPUTER_WEARABLE,
	.pAdvManData = (uint8_t*)&s_AdvCnt,
	.AdvManDataLen = sizeof(s_AdvCnt),
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.AdvInterval = APP_ADV_INTERVAL_MSEC,
	.AdvTimeout = APP_ADV_TIMEOUT_MSEC,
	.TxPower = 0,
};

static void NvmCycleHandler(uint32_t Evt, void *pCtx);
static void NvmCycleReport(void);

#if NVM_DEMO_ASYNC
// The async cycle never waits. It starts one operation and returns, and while
// the memory is still working it puts itself back on the queue, so the loop
// keeps running and the stack keeps its turn. This is the shape a stack event
// handler needs: start the write, leave, finish it later.
//
// It cannot use NvmCycleSlot: that one drains each write so the checks can
// read it back, which would leave nothing in flight and nothing to requeue
// for. The readback happens on the next visit instead, once the memory is
// idle.
static void NvmCyclePump(uint32_t Evt, void *pCtx);
#endif

// ---------------------------------------------------------------------------
// With a stack up: the same checks once, then a cycle per advertising timeout
// so the radio is contending for the memory.
// ---------------------------------------------------------------------------

// One slot: erase and wrap when the scratch is full, then write and read back.
// Shared by both modes so the work being measured is the same.
static void NvmCycleSlot(void)
{
	uint32_t page = NvmDemoUnitSize(g_Nvm);

	if (s_Slot >= NVM_DEMO_SLOTS)
	{
		// A rewrite in place medium wraps by overwriting.
		if (g_Nvm.EraseSize() != 0 && NvmDemoErase(g_Nvm, 0, page) != 0)
		{
			g_NvmFailures++;
		}
		s_Slot = 0;
	}

	uint32_t off = s_Slot * 4;
	uint32_t val = NVM_DEMO_MAGIC ^ s_Writes;
	uint32_t rd = 0;

	if (NvmDemoWrite(g_Nvm, off, &val, 4) != 4 ||
		g_Nvm.Read(off, &rd, 4) != 4 || rd != val)
	{
		g_NvmFailures++;
	}

	s_Slot++;
	s_Writes++;
}

#if NVM_DEMO_ASYNC
// Put the pump back on the queue, or abandon the cycle. A queue slot is not
// guaranteed: the stack posts its own work and the pump asks for one on every
// visit, thousands of times a cycle. Without this the first refusal would end
// cycling for good, because NvmCycleHandler skips while s_Pending is not 0.
static bool NvmCycleRequeue(void)
{
	if (AppEvtHandlerQue(0, NULL, NvmCyclePump))
	{
		return true;
	}

	// Leave nothing half started, so the next advertising timeout begins a
	// clean cycle rather than finding one that never finished.
	s_Pending = 0;
	s_PendCheck = false;
	s_Dropped++;

	return false;
}

static void NvmCyclePump(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	if (g_Nvm.IsBusy())
	{
		s_Requeues++;
		s_StallRq++;

		if (s_StallRq > s_StallMax)
		{
			s_StallMax = s_StallRq;
		}
		if (s_StallRq == NVM_DEMO_STALL_LONG)
		{
			s_StallLong++;
		}

		// Only a wait that is never going to end prints at the time. A slow
		// one is counted above and reported with the cycle instead.
		if ((s_StallRq % NVM_DEMO_STALL_STUCK) == 0)
		{
#if NVM_DEMO_MEDIUM == 0
			NvmMcuIntrfStat_t st;

			NvmMcuIntrfGetStat(&st);

			printf("stuck   : rq %lu ops %lu evt %lu\r\n",
									  (unsigned long)s_StallRq, (unsigned long)st.Ops,
									  (unsigned long)st.Evt);

			// Passed on, dropped with nothing wanting it, dropped with nothing
			// outstanding. Either of the last two means the driver is waiting
			// for a completion that has already been and gone.
			printf("stuck   : rep %lu/%lu/%lu pend %lu\r\n",
									  (unsigned long)st.RepDone,
									  (unsigned long)st.RepNoWant,
									  (unsigned long)st.RepNoPend,
									  (unsigned long)s_Pending);
#else
			printf("stuck   : rq %lu pend %lu\r\n",
									  (unsigned long)s_StallRq, (unsigned long)s_Pending);
#endif
		}

		(void)NvmCycleRequeue();

		return;
	}

	// The memory answered, so this wait is over. How long it was is in
	// s_StallMax and goes out with the cycle report.
	s_StallRq = 0;

	// The memory is idle, so whatever was started has finished.
	if (s_PendCheck)
	{
		uint32_t rd = 0;

		if (g_Nvm.Sync() != 0 ||
			g_Nvm.Read(s_PendOff, &rd, 4) != 4 || rd != s_PendVal)
		{
			g_NvmFailures++;
		}

		s_PendCheck = false;
	}

	if (s_Pending == 0)
	{
		s_Cycles++;

		NvmCycleReport();

		return;
	}

	s_Pending--;

	if (s_Slot >= NVM_DEMO_SLOTS)
	{
		// A rewrite in place medium wraps by overwriting.
		if (g_Nvm.EraseSize() != 0 && g_Nvm.Erase(0, NvmDemoUnitSize(g_Nvm)) != 0)
		{
			g_NvmFailures++;
		}
		s_Slot = 0;

		// The erase was started, not finished. Come back for it.
		(void)NvmCycleRequeue();

		return;
	}

	s_PendOff = s_Slot * 4;
	s_PendVal = NVM_DEMO_MAGIC ^ s_Writes;

	if (g_Nvm.Write(s_PendOff, &s_PendVal, 4) != 4)
	{
		g_NvmFailures++;
	}
	else
	{
		s_PendCheck = true;
	}

	s_Slot++;
	s_Writes++;

	(void)NvmCycleRequeue();
}

#endif

// Runs from AppRun, outside any interrupt, so blocking here is safe while advertising
// continues.
static void NvmCycleHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	if (s_Ready == false)
	{
		return;
	}

#if NVM_DEMO_ASYNC
	if (s_Pending > 0 || g_Nvm.IsBusy())
	{
		// The previous cycle has not finished; skip rather than pile up.
		// Which of the two matters. pend above 0 means the pump is still
		// going round. pend 0 means it stopped and the memory never
		// finished, since nothing else can be true here. rq standing still
		// across two of these says the pump is no longer running at all,
		// which is different again from the timeout no longer arriving,
		// and that one prints nothing.
		s_Skips++;
		if (s_Skips == 3UL || (s_Skips % 10UL) == 0)
		{
			printf("cycle   : skip %lu pend %lu rq %lu\r\n",
									  (unsigned long)s_Skips, (unsigned long)s_Pending,
									  (unsigned long)s_StallRq);
		}

		return;
	}

	s_Skips = 0;
	s_Pending = NVM_DEMO_WRITES_PER_CYCLE;

	(void)NvmCycleRequeue();
#else
	for (int i = 0; i < NVM_DEMO_WRITES_PER_CYCLE; i++)
	{
		NvmCycleSlot();
	}

	s_Cycles++;

	NvmCycleReport();
#endif
}

// Every fifth cycle, so the line does not drown the rest.
static void NvmCycleReport(void)
{
	if ((s_Cycles % 5) == 0)
	{
#if NVM_DEMO_MEDIUM == 0
		NvmMcuIntrfStat_t st;
		NvmMcuIntrfGetStat(&st);

		// With the radio up, direct is the count that must stay still: it
		// means the controller was driven while the stack owned the memory.
#if NVM_DEMO_ASYNC
		// requeues is the point of the mode: that many times the cycle left
		// the loop instead of waiting for the memory.
		// dropped is a cycle abandoned because the event queue was full. It
		// should stay at 0; if it climbs, the queue is too shallow for the
		// rate the pump asks at.
		// Split in two. UARTvprintf formats into SPRT_BUFFER_SIZE, which is
		// 80, so a line past 79 characters loses its tail before the FIFO
		// sees it. The single line this replaced was already over.
		printf("async   : requeues %lu dropped %lu txdrop %lu\r\n",
								  (unsigned long)s_Requeues, (unsigned long)s_Dropped,
								  (unsigned long)((UARTDev_t*)g_Uart)->TxDropCnt);
		printf("async   : write evt %lu erase evt %lu err %lu\r\n",
								  (unsigned long)g_NvmEvtWrite, (unsigned long)g_NvmEvtErase,
								  (unsigned long)g_NvmEvtError);

		// max is the longest the pump ever waited on one operation, in visits
		// of about 1.5 msec each. long is how many waits passed the threshold.
		printf("wait    : max %lu long %lu\r\n",
								  (unsigned long)s_StallMax, (unsigned long)s_StallLong);

#endif
		printf("cycles %lu writes %lu fails %lu\r\n",
								  (unsigned long)s_Cycles, (unsigned long)s_Writes,
								  (unsigned long)g_NvmFailures);
		// rep, not evt. Evt counts only the SoftDevice flash event, so on a
		// timeslot build it stays at 0 while every operation completes
		// normally. RepDone counts what was passed to the driver whichever
		// path the operation took, so ops against rep is the check that
		// reads the same on both.
		printf("intrf   : ops %lu busy %lu rep %lu\r\n",
								  (unsigned long)st.Ops, (unsigned long)st.Busy,
								  (unsigned long)st.RepDone);
		printf("path    : sd %lu slot %lu direct %lu\r\n",
								  (unsigned long)st.Sd, (unsigned long)st.Slot,
								  (unsigned long)st.Direct);
#else
		printf("cycles %lu writes %lu fails %lu\r\n",
								  (unsigned long)s_Cycles, (unsigned long)s_Writes,
								  (unsigned long)g_NvmFailures);
#endif
	}
}

void BtAppInitUserData()
{
	if (NvmDemoSetup() == false)
	{
		return;
	}

	NvmDemoVerify(g_Nvm);

	// The checks left the scratch in a known state; start the cycle after it.
	s_Slot = 0;
	s_Writes = 0;
	s_Ready = true;

	printf("advertising now, %d writes per cycle\r\n",
							  NVM_DEMO_WRITES_PER_CYCLE);
}

// Runs inside the stack event dispatch, so it only queues the work.
void BtAppAdvTimeoutHandler()
{
	s_AdvCnt++;

	BtAppAdvManDataSet((uint8_t*)&s_AdvCnt, sizeof(s_AdvCnt), NULL, 0);

	if (AppEvtHandlerQue(0, NULL, NvmCycleHandler) == false)
	{
		printf("cycle   : event queue refused the cycle\r\n");
	}
}

int main()
{
	// The queue exists before any interrupt can queue an event
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));

	if (!g_Uart.Init(g_NvmDemoUartCfg))
	{
		return 1;
	}
#ifdef NDEBUG
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif
	setvbuf(stdout, NULL, _IONBF, 0);

	printf("\r\nNvm demo on the %s, with a stack up\r\n", g_NvmMediumName);

	BtAppInit(&s_BtAppCfg);

	AppRun();

	return 0;
}

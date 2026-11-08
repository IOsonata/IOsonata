/**-------------------------------------------------------------------------
@file	pei_timer_cc_test.cpp
@brief	Validate independent PEI and interrupt-driven Timer compare channels.

CC0 normally drives a 500 ms PEI/PPI/GPIOTE LED0 task. CC1 normally
generates a 250 ms interrupt and toggles LED1 from its callback.
The Timer ISR reschedules both compares, but never forwards the PEI event.
Elapsed-counter and interrupt-interval results are in g_PeiCcTest.
No UART output or semihosting is used.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 2,
	"PeiTimerCcTest requires two LEDs");

enum {
	PEI_CC_STATUS_SETUP = 0U,
	PEI_CC_STATUS_RUNNING = 1U,
	PEI_CC_STATUS_TIMER_PASS = 2U,
	PEI_CC_STATUS_ERROR = 3U
};

enum {
	PEI_CC_ERROR_INIT = 1U << 0,
	PEI_CC_ERROR_ROUTE = 1U << 1,
	PEI_CC_ERROR_TRIGGER = 1U << 2,
	PEI_CC_ERROR_COUNTER = 1U << 3,
	PEI_CC_ERROR_INTERVAL = 1U << 4
};

typedef struct {
	volatile uint32_t Status;
	volatile uint32_t Errors;
	volatile uint32_t TimerDevNo;
	volatile uint32_t TimerFrequency;
	volatile uint32_t PeiTrigger;
	volatile uint32_t IrqTrigger;
	volatile uint32_t IrqCount;
	volatile uint32_t BadIntervals;
	volatile uint64_t ExpectedIrqTicks;
	volatile uint64_t MinIrqTicks;
	volatile uint64_t MaxIrqTicks;
	volatile uint64_t LastIrqTicks;
	volatile uint64_t LastIrqIntervalTicks;
	volatile uint64_t StartTicks;
	volatile uint64_t ElapsedTicks;
	volatile uint64_t LastCounterTicks;
} PeiCcTestResult_t;

// Status 2 verifies Timer intervals and counter continuity, not the PPI route.
// LED0 is the independent physical indication of PPI/GPIOTE operation.
volatile PeiCcTestResult_t g_PeiCcTest;

static void PeiCcIrqHandler(TimerDev_t *pTimer, int TrigNo, void *pContext)
{
	uint64_t now;
	uint64_t delta;
	uint64_t expected;
	uint64_t tolerance;

	(void)pContext;
	now = TimerGetTickCount(pTimer);
	if (TrigNo != (int)g_PeiCcTest.IrqTrigger)
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_TRIGGER;
	}
	if (g_PeiCcTest.IrqCount != 0U)
	{
		if (now < g_PeiCcTest.LastIrqTicks)
		{
			g_PeiCcTest.Errors |= PEI_CC_ERROR_COUNTER;
		}
		else
		{
			delta = now - g_PeiCcTest.LastIrqTicks;
			g_PeiCcTest.LastIrqIntervalTicks = delta;
			if (g_PeiCcTest.IrqCount == 1U ||
				delta < g_PeiCcTest.MinIrqTicks)
			{
				g_PeiCcTest.MinIrqTicks = delta;
			}
			if (delta > g_PeiCcTest.MaxIrqTicks)
			{
				g_PeiCcTest.MaxIrqTicks = delta;
			}
			expected = g_PeiCcTest.ExpectedIrqTicks;
			tolerance = expected / 5U;
			if (delta < expected - tolerance ||
				delta > expected + tolerance)
			{
				g_PeiCcTest.BadIntervals++;
				g_PeiCcTest.Errors |= PEI_CC_ERROR_INTERVAL;
			}
		}
	}
	g_PeiCcTest.LastIrqTicks = now;
	g_PeiCcTest.IrqCount++;
	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
}

int main(void)
{
	Pei pei;
	Timer timer;
	TimerCfg_t cfg = {};
	TimerDev_t *pTimer;
	PeiEvent_t evt = 0U;
	uint64_t accepted;
	uint64_t now;
	uint64_t previous;
	int peiTrig;
	int irqTrig;
	int i;

	cfg.DevNo = PEI_CC_TEST_TIMER_DEVNO;
	cfg.Freq = PEI_CC_TEST_TIMER_FREQ;
	cfg.IntPrio = IRQ_PRIO_NORMAL;
	g_PeiCcTest.TimerDevNo = cfg.DevNo;
	g_PeiCcTest.PeiTrigger = UINT32_MAX;
	g_PeiCcTest.IrqTrigger = UINT32_MAX;

	if (!timer.Init(cfg))
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_INIT;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}
	if (!timer.Enable())
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_INIT;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}
	if (!timer.ExecuteTask(TIMER_TASK_STOP))
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_INIT;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}
	if (timer.MaxTimerTrigger() < 2)
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_INIT;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}
	pTimer = timer;
	g_PeiCcTest.TimerFrequency = timer.Frequency();
	g_PeiCcTest.ExpectedIrqTicks =
		TimerNanosecondToTick(pTimer,
			(uint64_t)PEI_CC_TEST_ISR_PERIOD_MS * 1000000ULL);

	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Leds[i].PortNo, s_Leds[i].PinNo, s_Leds[i].PinOp,
			s_Leds[i].PinDir, s_Leds[i].Res, s_Leds[i].Type);
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

#if PEI_CC_TEST_ISR_FIRST
	irqTrig = 0;
	g_PeiCcTest.IrqTrigger = (uint32_t)irqTrig;
	accepted = timer.EnableTimerTrigger(irqTrig,
		(uint64_t)PEI_CC_TEST_ISR_PERIOD_MS * 1000000ULL,
		TIMER_TRIG_TYPE_CONTINUOUS, PeiCcIrqHandler, nullptr);
	if (accepted == 0U)
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_TRIGGER;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}
	peiTrig = timer.EnableTimerTrigger(pei, evt,
		PEI_CC_TEST_PEI_PERIOD_MS, TIMER_TRIG_TYPE_CONTINUOUS);
#else
	peiTrig = timer.EnableTimerTrigger(pei, evt,
		PEI_CC_TEST_PEI_PERIOD_MS, TIMER_TRIG_TYPE_CONTINUOUS);
	if (peiTrig < 0)
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_TRIGGER;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}
	irqTrig = peiTrig == 0 ? 1 : 0;
	g_PeiCcTest.IrqTrigger = (uint32_t)irqTrig;
	accepted = timer.EnableTimerTrigger(irqTrig,
		(uint64_t)PEI_CC_TEST_ISR_PERIOD_MS * 1000000ULL,
		TIMER_TRIG_TYPE_CONTINUOUS, PeiCcIrqHandler, nullptr);
	if (accepted == 0U)
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_TRIGGER;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}
#endif
	if (peiTrig < 0)
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_TRIGGER;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}
	g_PeiCcTest.PeiTrigger = (uint32_t)peiTrig;

	if (!IOPinEnableTask(pei, evt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_ROUTE;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}
	if (!pei.Enable(evt))
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_ROUTE;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}

	g_PeiCcTest.StartTicks = timer.TickCount();
	previous = g_PeiCcTest.StartTicks;
	g_PeiCcTest.LastCounterTicks = previous;
	g_PeiCcTest.Status = PEI_CC_STATUS_RUNNING;
	if (!timer.ExecuteTask(TIMER_TASK_START))
	{
		g_PeiCcTest.Errors |= PEI_CC_ERROR_INIT;
		g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		return 1;
	}

	for (;;)
	{
		__WFE();
		now = timer.TickCount();
		if (now < previous)
		{
			g_PeiCcTest.Errors |= PEI_CC_ERROR_COUNTER;
		}
		else
		{
			g_PeiCcTest.ElapsedTicks = now - g_PeiCcTest.StartTicks;
		}
		previous = now;
		g_PeiCcTest.LastCounterTicks = now;
		if (g_PeiCcTest.Errors != 0U)
		{
			g_PeiCcTest.Status = PEI_CC_STATUS_ERROR;
		}
		else if (g_PeiCcTest.IrqCount >= 20U)
		{
			g_PeiCcTest.Status = PEI_CC_STATUS_TIMER_PASS;
		}
	}
}

#!/usr/bin/env python3
"""Exercise WBA deadlines on the ST controller timer API, without hardware."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
WPAN = Path(os.environ.get('STM32_WPAN', str(ROOT.parent / 'stm32-mw-wpan')))
if not (WPAN / 'link_layer/ll_cmd_lib/inc/os_wrapper.h').exists():
    WPAN = ROOT.parent / 'external/stm32-mw-wpan'
source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_wba_platform.cpp').read_text()
start = source.index('enum {\n\tBT_WBA_TMR_FREE')
end = source.index('// Random number generator.', start)
code = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include "os_wrapper.h"
#define SysLogPrintf(...) ((void)0)
#define BT_WBA_TIMER_SLOTS 8U
#define BLE_STATUS_SUCCESS 0
#define BLE_STATUS_FAILED 1
#define BLE_STATUS_INSUFFICIENT_RESOURCES 2
static uint32_t now, irq, period, frequency = 32768;
static unsigned hostCalls, expiredCount;
static uint16_t expired[32];
static bool failArm, refuseQueue, failCreate;
static sw_timer_t timers[2];
static unsigned created;
static uint32_t rawBase;
static bool alternateRatio;
static unsigned depth, alignedCalls;
static uint32_t staleSteps;
static void (*queued)(uint32_t, void *);
static uint32_t BtWbaIrqSave() { uint32_t saved = irq; irq = 1; depth++; return saved; }
static void BtWbaIrqRestore(uint32_t saved) { assert(depth > 0); depth--; irq = saved; }
static bool BtEvtQue(uint32_t, void *, void (*handler)(uint32_t, void *))
{
	if (refuseQueue)
	{
		return false;
	}
	assert(queued == nullptr);
	queued = handler;
	return true;
}
// The aligned read waits up to one sleep timer step for an edge. It must never
// run inside the port critical section, which would hold off the radio IRQ.
static uint32_t ll_intf_get_aligned_us_now(uint32_t *multiplier, uint32_t *divider)
{
	assert(depth == 0);
	alignedCalls++;
	*multiplier = frequency == 32768 ? 15625 : 125;
	*divider = frequency == 32768 ? 512 : 4;
	if (alternateRatio)
	{
		*multiplier *= 3;
		*divider *= 3;
	}
	return rawBase + (uint32_t)(((uint64_t)now * frequency + 999) / 1000);
}
static uint32_t ll_intf_cmn_get_slptmr_value(void)
{
	assert(depth > 0);
	return rawBase + (uint32_t)(((uint64_t)now * frequency + 999) / 1000) - staleSteps;
}
void *os_timer_create(t_timer_callbk callback, os_timer_type type, void *arg)
{
	if (failCreate)
	{
		return nullptr;
	}
	assert(created < 2);
	timers[created].ptimer = callback;
	timers[created].argument = arg;
	timers[created].type = type;
	return &timers[created++];
}
int32_t os_timer_stop(os_timer_id timer)
{
	assert(timer != nullptr);
	if (timer == &timers[0])
	{
		period = 0;
	}
	return 0;
}
int32_t os_timer_start(os_timer_id timer, uint32_t steps)
{
	assert(timer != nullptr && steps <= 32000);
	if (failArm)
	{
		return -1;
	}
	if (timer == &timers[0])
	{
		period = steps / 32;
	}
	return 0;
}
int32_t os_timer_set(os_timer_id timer, uint32_t steps)
{
	return os_timer_stop(timer) | os_timer_start(timer, steps);
}
static void BLEPLATCB_TimerExpiry(uint16_t id)
{
	assert(expiredCount < 32);
	expired[expiredCount++] = id;
}
extern "C" void BleStackCB_Process() { hostCalls++; }
'''
code += source[start:end]
code += r'''
static void Dispatch()
{
	assert(queued != nullptr);
	auto handler = queued;
	queued = nullptr;
	handler(0, nullptr);
}
static void Wake()
{
	assert(period > 0 && period <= 1000);
	now += period;
	period = 0;
	timers[0].ptimer(timers[0].argument);
	Dispatch();
}
static void Retry()
{
	// Same timer retry as BtWbaPlatformRetry, without unrelated LL tasks.
	if (s_BleTimerOwed)
	{
		s_BleTimerOwed = false;
		BtWbaTimerQue();
	}
	Dispatch();
}
static void Reset(uint32_t freq)
{
	memset(s_BleTimer, 0, sizeof(s_BleTimer));
	now = 0;
	created = 0;
	rawBase = 0xfffffff0U;
	s_ClockReady = false;
	depth = 0;
	alignedCalls = 0;
	s_BleTimerId = nullptr;
	s_AppTickTimer = nullptr;
	period = 0;
	frequency = freq;
	expiredCount = 0;
	hostCalls = 0;
	s_BleTimerOwed = false;
	s_BleTimerQueued = false;
	queued = nullptr;
}
int main()
{
	for (uint32_t freq : {32000U, 32768U})
	{
		Reset(freq);
		// Compare fast and fallback conversions with the original formula.
		// Include sub-ms boundaries, raw wraps and the 32-bit ms wrap.
		BtWbaClockStart();
		uint64_t samples[] = {0, 1, 31, 32, 33, 4095, 4096, 4097,
			0xffffffffULL, 0x100000000ULL,
			((uint64_t)0xffffffffU * freq) / 1000};
		for (bool alternate : {false, true})
		{
			alternateRatio = alternate;
			// The port keeps the scale from the last aligned read
			BtWbaClockScaleUpdate();
			for (uint64_t base : samples)
			{
				for (unsigned offset = 0; offset < 128; offset++)
				{
					s_ClockSteps = base + offset;
					uint64_t mult = freq == 32768 ? 15625 : 125;
					uint64_t div = freq == 32768 ? 512 : 4;
					irq = offset & 1;
					assert(BtWbaMsTick() == (uint32_t)((base + offset) * mult / (div * 1000)));
					assert(irq == (offset & 1));
				}
			}
		}
		alternateRatio = false;
		Reset(freq);
		irq = 1;
		failCreate = true;
		assert(BLE_TIMER_Start(1, 30000) == BLE_STATUS_INSUFFICIENT_RESOURCES);
		failCreate = false;
		assert(BLE_TIMER_Start(1, 30000) == 0 && irq == 1);
		// After start, the millisecond clock never waits for a sleep timer edge
		{
			unsigned aligned = alignedCalls;
			uint32_t t0 = BtWbaMsTick();
			now += 7;
			assert(BtWbaMsTick() == t0 + 7 && alignedCalls == aligned);
			// A stale read behind the last one does not move the clock
			staleSteps = 100;
			assert(BtWbaMsTick() == t0 + 7);
			staleSteps = 0;
			now += 3;
			assert(BtWbaMsTick() == t0 + 10);
			now -= 3;
			now -= 7;
			s_ClockSteps = 0;
			s_ClockLast = rawBase;
			BtWbaClockScaleUpdate();
			assert(alignedCalls == aligned + 1 && depth == 0 && irq == 1);
		}
		assert(BLE_TIMER_Start(2, 2000) == 0);
		Wake();
		assert(now == 1000 && expiredCount == 0 && hostCalls == 0);
		Wake();
		assert(now == 2000 && expiredCount == 1 && expired[0] == 2);
		while (now < 30000)
		{
			Wake();
			assert(expiredCount == (now == 30000 ? 2U : 1U));
		}
		assert(expired[1] == 1 && period == 0 && hostCalls == 2);

		assert(BLE_TIMER_Start(3, 30000) == 0);
		Wake();
		assert(BLE_TIMER_Start(3, 5) == 0 && period == 5);
		Wake();
		assert(expiredCount == 3 && expired[2] == 3);
		assert(BLE_TIMER_Start(4, 30000) == 0);
		BLE_TIMER_Stop(4);
		assert(period == 0);

		// Seed the extended time near a millisecond wrap; raw counter wrap
		// is already exercised by every Reset's near-UINT32_MAX rawBase.
		now = 0;
		s_ClockLast = rawBase;
		s_ClockSteps = ((uint64_t)0xfffffff0U * freq + 999) / 1000;
		assert(BLE_TIMER_Start(5, 30000) == 0);
		for (unsigned i = 0; i < 30; i++)
		{
			Wake();
		}
		assert(expiredCount == 4 && expired[3] == 5 && period == 0);

		failArm = true;
		assert(BLE_TIMER_Start(6, 30000) == 0);
		assert(s_BleTimerOwed && period == 0);
		failArm = false;
		Retry();
		assert(period == 1000 && expiredCount == 4);
		BLE_TIMER_Stop(6);

		assert(BLE_TIMER_Start(7, 10) == 0);
		now += period;
		period = 0;
		refuseQueue = true;
		timers[0].ptimer(timers[0].argument);
		assert(s_BleTimerOwed && !s_BleTimerQueued);
		refuseQueue = false;
		Retry();
		assert(expiredCount == 5 && expired[4] == 7 && period == 0);
		assert(created == 1);
		failCreate = true;
		assert(!BtWbaTickStart(BtWbaTimerHandler));
		failCreate = false;
		failArm = true;
		assert(!BtWbaTickStart(BtWbaTimerHandler));
		failArm = false;
		assert(BtWbaTickStart(BtWbaTimerHandler));
		assert(created == 2 && timers[1].type == os_timer_periodic);
		for (unsigned i = 0; i < BT_WBA_TIMER_SLOTS; i++)
		{
			assert(BLE_TIMER_Start(100 + i, 5000) == 0);
		}
		assert(BLE_TIMER_Start(200, 5000) == BLE_STATUS_INSUFFICIENT_RESOURCES);
		assert(created == 2);
		for (unsigned i = 0; i < BT_WBA_TIMER_SLOTS; i++)
		{
			BLE_TIMER_Stop(100 + i);
		}
		assert(period == 0);
	}
}
'''
code = code.replace('#include <cstring>', '#include <cstring>\n#include <initializer_list>')
with tempfile.TemporaryDirectory(prefix='wba-host-timers-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    for opt in ['-O0', '-O2']:
        binary = Path(temp) / 'test'
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', opt,
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I' + str(ROOT / 'include'), '-I' + str(WPAN / 'link_layer/ll_cmd_lib/inc'), str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True, timeout=10)
        print('PASS WBA host timers:', opt, 'long deadlines, restart, cancel, wrap, arm and queue refusal')

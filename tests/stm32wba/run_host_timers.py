#!/usr/bin/env python3
"""Exercise WBA host deadlines against the LPTIM compare range, without hardware."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_wba_platform.cpp').read_text()
start = source.index('enum {\n\tBT_WBA_TMR_FREE')
end = source.index('// Random number generator.', start)
code = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include "coredev/timer.h"
#define BT_WBA_TIMER_SLOTS 8U
#define BLE_STATUS_SUCCESS 0
#define BLE_STATUS_FAILED 1
#define BLE_STATUS_INSUFFICIENT_RESOURCES 2
static uint32_t now, irq, period, frequency = 32768;
static unsigned hostCalls, expiredCount;
static uint16_t expired[32];
static bool failArm, refuseQueue;
static void (*queued)(uint32_t, void *);
static uint32_t BtWbaIrqSave() { uint32_t saved = irq; irq = 1; return saved; }
static void BtWbaIrqRestore(uint32_t saved) { irq = saved; }
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
static uint64_t Ticks(TimerDev_t *) { return now; }
static void Disable(TimerDev_t *, int) { period = 0; }
static uint64_t Arm(TimerDev_t *, int, uint64_t ns, TIMER_TRIG_TYPE,
	TimerTrigEvtHandler_t, void *)
{
	uint64_t counts = (ns * frequency + 500000000ULL) / 1000000000ULL;
	if (failArm || counts <= 2 || counts >= 65536)
	{
		return 0;
	}
	period = ns / 1000000;
	return ns;
}
bool TimerInit(TimerDev_t *timer, const TimerCfg_t *)
{
	timer->Freq = 1000;
	timer->nsPeriod = 1000000;
	timer->GetTickCount = Ticks;
	timer->DisableTrigger = Disable;
	timer->EnableTrigger = Arm;
	return true;
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
	BtWbaTimerHandler(&s_BleTimerDev, TIMER_EVT_TRIGGER(0));
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
		irq = 1;
		assert(BLE_TIMER_Start(1, 30000) == 0 && irq == 1);
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

		now = 0xfffffff0U;
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
		BtWbaTimerHandler(&s_BleTimerDev, TIMER_EVT_TRIGGER(0));
		assert(s_BleTimerOwed && !s_BleTimerQueued);
		refuseQueue = false;
		Retry();
		assert(expiredCount == 5 && expired[4] == 7 && period == 0);
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
                        '-I' + str(ROOT / 'include'), str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True, timeout=10)
        print('PASS WBA host timers:', opt, 'long deadlines, restart, cancel, wrap, arm and queue refusal')

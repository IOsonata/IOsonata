#!/usr/bin/env python3
"""Exercise WBA host and LL task scheduling, with interrupt and queue pressure."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r'^(?:extern "C" )?(?:static )?(?:void|bool) ' +
                      name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


app = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_app_stm32wba.cpp').read_text()
platform = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_wba_platform.cpp').read_text()
code = r'''
#include <cassert>
#include <cstdint>
#include <initializer_list>
#define NDEBUG
#define SysLogPrintf(...) ((void)0)
static uint32_t now, processMs;
static uint32_t irq;
static uint32_t expectedClockMask;
static bool interruptClockRequest;
extern "C" void BleStackCB_Process(void);
static uint32_t BtSmpMsTick()
{
	// Port clock conversion must not inherit an extra queue-state IRQ mask.
	assert(irq == expectedClockMask);
	if (interruptClockRequest)
	{
		interruptClockRequest = false;
		BleStackCB_Process();
	}
	return now;
}
static unsigned hostRuns, taskRuns, count, head, tail;
static bool refuse, interruptRequest, requestWhileRunning;
static int processResult = 1;
#define BT_APP_WBA_EVT_SLOTS 8
static unsigned s_EvtCnt;
static bool s_bEvtFlowBlocked;
static unsigned burstRemaining;
static bool emitEvent;
using Handler = void (*)(uint32_t, void *);
static Handler events[4];
static volatile bool s_bBleHostQueued, s_bBleHostOwed;
static uint32_t s_HostProcessCount;
static void (*s_SeqTask[32])(void);
static volatile uint32_t s_SeqPend;
static volatile bool s_SeqQueued, s_SeqOwed;
extern "C" void BleStackCB_Process(void);
static void BleHostEvt(uint32_t, void *);
static void BtWbaSeqEvt(uint32_t, void *);
static uint32_t __get_PRIMASK() { return irq; }
static void __disable_irq() { irq = 1; }
static void __set_PRIMASK(uint32_t value) { irq = value; }
static uint32_t BtWbaIrqSave()
{
	uint32_t saved = irq;
	irq = 1;
	return saved;
}
static void BtWbaIrqRestore(uint32_t saved) { irq = saved; }
static bool BtEvtQue(uint32_t, void *, Handler handler)
{
	if (interruptRequest)
	{
		interruptRequest = false;
		BleStackCB_Process();
	}
	if (refuse || count == 4)
	{
		return false;
	}
	events[tail++ % 4] = handler;
	count++;
	return true;
}
static int BleStack_Process()
{
	hostRuns++;
	now += processMs;
	if (requestWhileRunning)
	{
		requestWhileRunning = false;
		BleStackCB_Process();
	}
	if (emitEvent)
	{
		assert(s_EvtCnt < BT_APP_WBA_EVT_SLOTS);
		s_EvtCnt++;
	}
	if (burstRemaining)
	{
		burstRemaining--;
		return burstRemaining ? 0 : 1;
	}
	return processResult;
}
static void RunOne()
{
	assert(count);
	Handler handler = events[head++ % 4];
	count--;
	handler(0, nullptr);
}
'''
start = app.index('// Port-wide counters since boot.')
code += app[start:app.index('static BtWbaEvtFlow_t BtAppHciEvtHandler', start)]
for name in ['BleHostEvt', 'BleStackCB_Process']:
    code += function(app, name)
for name in ['BtWbaSeqQue', 'BtWbaSeqEvt', 'UTIL_SEQ_SetTask']:
    code += function(platform, name)
code += r'''
static void LinkTask()
{
	taskRuns++;
	// The real LL background task may request host service and another pass.
	// Host work must run before this task repeats.
	assert(taskRuns == 1 || hostRuns > 0);
	if (taskRuns == 1)
	{
		BleStackCB_Process();
		UTIL_SEQ_SetTask(1, 0);
	}
}
int main()
{
	s_SeqTask[0] = LinkTask;
	UTIL_SEQ_SetTask(1, 0);
	RunOne();
	assert(taskRuns == 1 && count == 2);
	RunOne();
	RunOne();
	assert(hostRuns == 1 && taskRuns == 2 && count == 0);

	// An IRQ during timestamp capture must see the reserved host work.
	interruptClockRequest = true;
	BleStackCB_Process();
	assert(count == 1);
	RunOne();
	assert(count == 0);

	// A request during queue admission coalesces with its reservation.
	interruptRequest = true;
	BleStackCB_Process();
	assert(count == 1);
	requestWhileRunning = true;
	RunOne();
	assert(count == 1);
	RunOne();
	assert(count == 0);

	// A full queue retains work for the status retry, including masked callers.
	irq = 1;
	expectedClockMask = 1;
	refuse = true;
	now = 100;
	BleStackCB_Process();
	assert(s_HostQueueRefused == 1);
	now = 110;
	assert(irq == 1 && s_bBleHostOwed && !s_bBleHostQueued);
	refuse = false;
	BleStackCB_Process();
	assert(irq == 1 && !s_bBleHostOwed && s_bBleHostQueued && count == 1);
	now = 116;
	processMs = 2;
	RunOne();
	assert(!count);
	assert(s_HostWaitMaxMs == 16 && s_HostWaitOver15Ms == 1);
	assert(s_HostWaitOver1Ms == 1 && s_HostRunMaxMs == 2);
	processMs = 0;

	// Coalescing preserves the first timestamp across clock wrap.
	now = 0xfffffffeu;
	BleStackCB_Process();
	now = 1;
	BleStackCB_Process();
	RunOne();
	assert(s_HostWaitMaxMs == 16 && s_HostWaitOver1Ms == 2);
	assert(s_HostWaitOver15Ms == 1 && !count);

	// Four- and eight-event bursts complete in one dispatch.
	for (unsigned size : {4U, 8U})
	{
		unsigned before = hostRuns;
		burstRemaining = size;
		emitEvent = true;
		BleStackCB_Process();
		RunOne();
		assert(hostRuns - before == size && s_EvtCnt == size && !count);
		s_EvtCnt = 0;
	}
	// A partly occupied ring yields before another host call can overflow it.
	s_EvtCnt = 6;
	burstRemaining = 4;
	BleStackCB_Process();
	RunOne();
	assert(s_EvtCnt == 8 && burstRemaining == 2 && count == 1);
	RunOne();
	assert(s_EvtCnt == 8 && burstRemaining == 2 && count == 1);
	s_EvtCnt = 0;
	RunOne();
	assert(s_EvtCnt == 2 && !burstRemaining && !count);
	s_EvtCnt = 0;
	emitEvent = false;

	// Busy host work with no events is bounded, allowing other tasks to run.
	unsigned before = hostRuns;
	processResult = 0;
	BleStackCB_Process();
	RunOne();
	assert(hostRuns - before == 8 && count == 1);
	processResult = 1;
	RunOne();
	assert(!count);

	refuse = true;
	UTIL_SEQ_SetTask(1, 0);
	assert(s_SeqOwed && s_SeqPend == 1 && !s_SeqQueued);
	refuse = false;
	BtWbaSeqQue();
	RunOne();
	assert(!s_SeqOwed && !s_SeqPend && !count);
}
'''
with tempfile.TemporaryDirectory(prefix='wba-scheduling-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    for opt in ['-O0', '-O2']:
        binary = Path(temp) / 'test'
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', opt,
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True, timeout=10)
        print('PASS WBA scheduling:', opt, 'LL yield, host coalescing, queue refusal, IRQ mask')

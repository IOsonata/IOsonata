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
#define NDEBUG
#define SysLogPrintf(...) ((void)0)
static uint32_t BtSmpMsTick() { return 0; }
static uint32_t irq;
static unsigned hostRuns, taskRuns, count, head, tail;
static bool refuse, interruptRequest, requestWhileRunning;
static int processResult = 1;
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
	if (requestWhileRunning)
	{
		requestWhileRunning = false;
		BleStackCB_Process();
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
	refuse = true;
	BleStackCB_Process();
	assert(irq == 1 && s_bBleHostOwed && !s_bBleHostQueued);
	refuse = false;
	BleStackCB_Process();
	assert(irq == 1 && !s_bBleHostOwed && s_bBleHostQueued && count == 1);
	RunOne();
	assert(!count);

	processResult = 0;
	BleStackCB_Process();
	RunOne();
	assert(count == 1);
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

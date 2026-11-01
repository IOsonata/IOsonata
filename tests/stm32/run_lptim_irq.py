#!/usr/bin/env python3
"""Check compare programming with radio-like preemption and trigger cancellation."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/ST/src/timer_lptim_stm32.cpp').read_text()


def function(name):
    match = re.search(r'^static (?:uint64_t|void) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


code = r'''
#include <cassert>
#include <cstdint>
#include "coredev/timer.h"
#define ST_LPTIM_TRIG_MAXCNT 2
#define ST_LPTIM_CNT 1
#define LPTIM_CR_ENABLE 1
struct LPTIM_TypeDef { uint32_t CR, CNT, ICR, CMP[2]; };
'''
start = source.index('typedef struct {\n\tTimerDev_t *pTimer;')
code += source[start:source.index('} StLptDev_t;', start) + len('} StLptDev_t;')]
code += r'''
static StLptDev_t s_StLptDev[1];
static LPTIM_TypeDef regs;
static struct { LPTIM_TypeDef *pReg; int IrqNo; } s_StLptInfo[1] = {{&regs, 0}};
static uint32_t s_StLptCmpOk[2] = {1, 2}, s_StLptCmpIf[2] = {4, 8};
static uint32_t primask, expectedMask;
static bool timerIrq = true, radioIrq = true, writeOk = true, cancel, reenter;
static unsigned writes, preemptions;
static TimerDev_t timer;
static int StLptDevNo(TimerDev_t *p) { return p == &timer ? 0 : -1; }
static uint32_t DisableInterrupt()
{
	uint32_t old = primask;
	primask = 1;
	return old;
}
static void EnableInterrupt(uint32_t old) { primask = old; }
static uint32_t NVIC_GetEnableIRQ(int) { return timerIrq; }
static void NVIC_DisableIRQ(int) { timerIrq = false; }
static void NVIC_EnableIRQ(int) { timerIrq = true; }
static uint32_t StLptCount(LPTIM_TypeDef *reg) { return reg->CNT; }
static volatile uint32_t *StLptCmpReg(LPTIM_TypeDef *reg, int n) { return &reg->CMP[n]; }
static bool StLptWrite(LPTIM_TypeDef *, volatile uint32_t *, uint32_t, uint32_t);
'''
code += function('StLptEnableTrigger')
code += function('StLptDisableTrigger')
code += r'''
static bool StLptWrite(LPTIM_TypeDef *, volatile uint32_t *dst, uint32_t val, uint32_t)
{
	writes++;
	assert(primask == expectedMask && !timerIrq && radioIrq);
	if (!primask)
	{
		preemptions++;
		if (reenter)
		{
			assert(StLptEnableTrigger(&timer, 0, 20000000,
				TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
			assert(StLptEnableTrigger(&timer, 1, 20000000,
				TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
			assert(s_StLptDev[0].CC[0] == 320 && !timerIrq && !primask);
		}
		if (cancel)
		{
			StLptDisableTrigger(&timer, 0);
		}
	}
	*dst = val;
	return writeOk;
}
static uint64_t Arm()
{
	return StLptEnableTrigger(&timer, 0, 10000000,
		TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr);
}
int main()
{
	timer.nsPeriod = 31250;
	regs.CR = LPTIM_CR_ENABLE;
	regs.CNT = 100;
	assert(Arm() == 10000000);
	assert(preemptions == 1 && timerIrq && !primask && regs.CMP[0] == 420);
	assert(s_StLptDev[0].bTrigEn[0] && !s_StLptDev[0].bTrigWrite);

	reenter = true;
	assert(Arm() == 10000000 && writes == 2);
	reenter = false;
	cancel = true;
	assert(Arm() == 0 && !s_StLptDev[0].bTrigEn[0]);
	assert(!s_StLptDev[0].bTrigWrite && timerIrq && !primask);
	cancel = false;

	writeOk = false;
	assert(Arm() == 0 && !s_StLptDev[0].bTrigEn[0]);
	assert(!s_StLptDev[0].bTrigWrite && timerIrq && !primask);
	writeOk = true;
	assert(Arm() == 10000000);

	// An already-disabled timer vector remains disabled.
	timerIrq = false;
	assert(Arm() == 10000000 && !timerIrq && !primask);
	// Never enable interrupts that the caller disabled.
	primask = expectedMask = 1;
	timerIrq = true;
	assert(Arm() == 10000000 && primask == 1 && timerIrq);
	assert(writes == 7 && preemptions == 6);
}
'''
with tempfile.TemporaryDirectory(prefix='lptim-irq-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    for opt in ['-O0', '-O2']:
        binary = Path(temp) / 'test'
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', opt,
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I' + str(ROOT / 'include'), str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True, timeout=10)
        print('PASS LPTIM IRQ:', opt, 'preemption, reentry, cancellation, write failure, saved masks')

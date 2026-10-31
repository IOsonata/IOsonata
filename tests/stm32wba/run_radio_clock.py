#!/usr/bin/env python3
"""Exercise WBA radio sleep-clock startup with delayed oscillator readiness."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_wba_platform.cpp').read_text()
start = source.index('extern "C" void SCM_HSE_WaitUntilReady(void)')
end = source.index('extern "C" void scm_notifyradiostate', start)
code = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include "coredev/system_core_clock.h"
#define RCC_CR_HSEON (1U << 16)
#define RCC_CR_HSERDY (1U << 17)
#define RCC_BDCR1_LSEON 1U
#define RCC_BDCR1_LSERDY 2U
#define RCC_BDCR1_LSEBYP 4U
#define RCC_BDCR1_LSEDRV (3U << 3)
#define RCC_BDCR1_LSEDRV_0 (1U << 3)
#define RCC_BDCR1_LSI1ON (1U << 26)
#define RCC_BDCR1_LSI1RDY (1U << 27)
#define RCC_BDCR1_RADIOSTSEL (3U << 18)
#define RCC_BDCR1_RADIOSTSEL_0 (1U << 18)
#define RCC_BDCR1_RADIOSTSEL_1 (2U << 18)
#define PWR_DBPR_DBP 1U
#define BT_WBA_HSE_POLL_MAX 2000000UL
#define SET_BIT(r,b) ((r) |= (b))
#define CLEAR_BIT(r,b) ((r) &= ~(b))
#define MODIFY_REG(r,c,s) ((r) = ((r) & ~(c)) | (s))
static uint32_t reads, readyAfter;
struct Register
{
	uint32_t value;
	operator uint32_t()
	{
		if (value & RCC_BDCR1_LSEON)
		{
			if (++reads >= readyAfter)
			{
				value |= RCC_BDCR1_LSERDY;
			}
		}
		return value;
	}
	void operator=(uint32_t v) { value = v; }
	void operator|=(uint32_t v) { value |= v; }
	void operator&=(uint32_t v) { value &= v; }
};
static struct { uint32_t CR; Register BDCR1; } rcc;
static struct { uint32_t DBPR; } pwr;
#define RCC (&rcc)
#define PWR (&pwr)
static uint32_t SystemCoreClock = 100000000;
McuOsc_t g_McuOsc;
static const char *clockName;
static void *SysLogGet() { return nullptr; }
static void SysLogPrintf(void *, const char *, const char *name) { clockName = name; }
'''
code += source[start:end]
code += r'''
static void Reset(OSC_TYPE type, uint32_t delay)
{
	rcc.CR = RCC_CR_HSERDY;
	rcc.BDCR1.value = RCC_BDCR1_LSI1RDY;
	pwr.DBPR = 0;
	reads = 0;
	readyAfter = delay;
	g_McuOsc.LowPwrOsc.Type = type;
}
int main()
{
	// The old HSE-sized budget expired before this crystal became ready.
	Reset(OSC_TYPE_XTAL, 3000000);
	rcc.BDCR1.value |= RCC_BDCR1_LSEBYP;
	BtWbaPlatformInit();
	assert((rcc.BDCR1.value & RCC_BDCR1_RADIOSTSEL) == RCC_BDCR1_RADIOSTSEL_0);
	assert((rcc.BDCR1.value & RCC_BDCR1_LSEDRV) == RCC_BDCR1_LSEDRV_0);
	assert(!(rcc.BDCR1.value & RCC_BDCR1_LSEBYP));
	assert(strcmp(clockName, "LSE 32768 Hz") == 0);
	// Preserve drive and bypass of a clock already in use by another timer.
	Reset(OSC_TYPE_XTAL, 1);
	rcc.BDCR1.value |= RCC_BDCR1_LSEON | RCC_BDCR1_LSERDY | RCC_BDCR1_LSEDRV;
	BtWbaPlatformInit();
	assert((rcc.BDCR1.value & RCC_BDCR1_LSEDRV) == RCC_BDCR1_LSEDRV);
	Reset(OSC_TYPE_TCXO, 1);
	BtWbaPlatformInit();
	assert(rcc.BDCR1.value & RCC_BDCR1_LSEBYP);
	// RC boards do not attempt crystal startup.
	Reset(OSC_TYPE_RC, 1);
	BtWbaPlatformInit();
	assert(reads == 0);
	assert((rcc.BDCR1.value & RCC_BDCR1_RADIOSTSEL) == RCC_BDCR1_RADIOSTSEL_1);
	// Preserve the existing failed-crystal fallback and report its actual source.
	Reset(OSC_TYPE_XTAL, UINT32_MAX);
	BtWbaPlatformInit();
	assert(!(rcc.BDCR1.value & RCC_BDCR1_LSEON));
	assert((rcc.BDCR1.value & RCC_BDCR1_RADIOSTSEL) == RCC_BDCR1_RADIOSTSEL_1);
	assert(strcmp(clockName, "LSI1") == 0);
}
'''
with tempfile.TemporaryDirectory(prefix='wba-radio-clock-') as tmp:
    path = Path(tmp)
    (path / 'test.cpp').write_text(code)
    for opt in ('-O0', '-O2'):
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', opt,
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I', str(ROOT / 'include'), str(path / 'test.cpp'),
                        '-o', str(path / 'test')], check=True)
        subprocess.run([str(path / 'test')], check=True)
        print(f'WBA radio clock {opt}: PASS')

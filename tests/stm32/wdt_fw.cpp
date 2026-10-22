/**-------------------------------------------------------------------------
@file	wdt_fw.cpp

@brief	STM32 watchdog driver test firmware

Runs the production watchdog driver, ARM/ST/src/wdt_stm32.cpp, in the
emulator of run_wdt.py, one scenario per run. usDelay (delay_shim/idelay.h)
moves the time of the IWDG model. Test registers at 0x5FFF0000, word index:

	0	read: scenario
	1	check result, bit 31 set when it failed
	2	end, number of checks, no reset expected
	3	time passes, usec
	4	expected reset, earliest time in usec
	5	expected reset, latest time in usec: arms the expectation
	6	expected reset cause: 1 counter at 0, 2 reload outside the window
	7	IWDG base: records it with the features of 8
	8	features: bit 0 window register, bit 1 early wakeup interrupt, bit 2
		SR ONF, bits 8 - 11 largest prescaler value
	9	read: LSI frequency of the model, Hz, after the predivider
	10	read: time, usec
	11	read: bit 0 IWDG running, bit 1 early wakeup interrupt pending
	12	command: 1 started by hardware at reset, 2 started by a boot loader
		(prescaler 256, reload 100, updates in progress)
	13	predivider register address
	14	predivider bit
	15	event handler called
	16	read: divider in effect
	17	read: reload value in effect
	18	read: window value in effect
	19	read: number of key register writes
	20	read: time of the last counter reload, usec

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#include <stdint.h>
#include <stddef.h>

#include "stm32.h"
#include "coredev/wdt.h"
#include "idelay.h"

#if defined(IWDG)
#define TEST_IWDG		IWDG
#elif defined(IWDG1)
#define TEST_IWDG		IWDG1
#endif

#define TEST_PR_MAX		((IWDG_PR_PR >> IWDG_PR_PR_Pos) > 7U ? 8U : 6U)

#ifdef IWDG_EWCR_EWIE
extern "C" void IWDG_IRQHandler(void);
#endif

static volatile uint32_t *const s_Io = (volatile uint32_t *)0x5FFF0000;
static unsigned s_Checks = 0;
#ifdef IWDG_EWCR_EWIE
static int s_Events = 0;
#endif

extern "C" void usDelay(uint32_t cnt)
{
	s_Io[3] = cnt;
}

extern "C" void *memset(void *p, int c, size_t n)
{
	volatile unsigned char *d = (volatile unsigned char *)p;

	while (n--)
	{
		*d++ = c;
	}

	return p;
}

extern "C" void *memcpy(void *p, const void *q, size_t n)
{
	volatile unsigned char *d = (volatile unsigned char *)p;
	const volatile unsigned char *s = (const volatile unsigned char *)q;

	while (n--)
	{
		*d++ = *s++;
	}

	return p;
}

static void Check(bool Ok)
{
	s_Io[1] = ++s_Checks | (Ok ? 0 : 0x80000000U);
}

static uint32_t Now(void)
{
	return s_Io[10];
}

// IWDG_SW option bit: true software watchdog (the reset value of the option
// byte), false hardware watchdog
static void OptionSw(bool bSw)
{
#if defined(FLASH_OPTR_IWDG_SW)
	volatile uint32_t *reg = &FLASH->OPTR;
	uint32_t bit = FLASH_OPTR_IWDG_SW;
#elif defined(FLASH_OBR_IWDG_SW)
	volatile uint32_t *reg = &FLASH->OBR;
	uint32_t bit = FLASH_OBR_IWDG_SW;
#elif defined(FLASH_OPTCR_WDG_SW)
	volatile uint32_t *reg = &FLASH->OPTCR;
	uint32_t bit = FLASH_OPTCR_WDG_SW;
#elif defined(FLASH_OPTSR_IWDG1_SW)
	volatile uint32_t *reg = &FLASH->OPTSR_CUR;
	uint32_t bit = FLASH_OPTSR_IWDG1_SW;
#endif

	*reg = bSw ? *reg | bit : *reg & ~bit;
}

// IWDG debug freeze bit
static bool Frozen(void)
{
#if defined(DBGMCU_APB1_FZ_DBG_IWDG_STOP)
	return DBGMCU->APB1FZ & DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#elif defined(DBGMCU_APB1FZR1_DBG_IWDG_STOP)
	return DBGMCU->APB1FZR1 & DBGMCU_APB1FZR1_DBG_IWDG_STOP;
#elif defined(DBGMCU_APB4FZ1_DBG_IWDG1)
	return DBGMCU->APB4FZ1 & DBGMCU_APB4FZ1_DBG_IWDG1;
#elif defined(DBGMCU_APB1LFZR_DBG_IWDG_STOP)
	return DBGMCU->APB1LFZR & DBGMCU_APB1LFZR_DBG_IWDG_STOP;
#endif
}

static WdtCfg_t Cfg(uint32_t Ms)
{
	WdtCfg_t cfg = { 0, Ms, 0, 1, true, false, 3, nullptr };

	return cfg;
}

// Expected configuration of a timeout: the smallest divider that holds it,
// ticks rounded up. False when no divider holds it.
static bool Expect(uint32_t Ms, uint32_t *pDiv, uint32_t *pTicks, uint32_t *pMs)
{
	uint64_t lsi = s_Io[9];

	for (uint32_t div = 4; div <= (4U << TEST_PR_MAX); div <<= 1)
	{
		uint64_t ticks = ((uint64_t)Ms * lsi + div * 1000ULL - 1) / (div * 1000ULL);

		if (ticks <= 4096)
		{
			*pDiv = div;
			*pTicks = (uint32_t)ticks;
			*pMs = (uint32_t)((ticks * div * 1000ULL + lsi - 1) / lsi);

			return true;
		}
	}

	return false;
}

// Longest timeout, msec
static uint32_t MaxMs(void)
{
	return (uint32_t)(4096ULL * (4U << TEST_PR_MAX) * 1000ULL / s_Io[9]);
}

// Expects the reset a timeout after time T: Ms is the timeout rounded up,
// the reset comes in the last msec before
static void ExpectReset(uint32_t T, uint32_t Ms, uint32_t Cause)
{
	s_Io[6] = Cause;
	s_Io[4] = T + (Ms - 1U) * 1000U + 1U;
	s_Io[5] = T + Ms * 1000U;
}

static void Fail(void)
{
	Check(false);
	while (1)
	{
	}
}

#ifdef IWDG_EWCR_EWIE
static void Event(WdtDev_t * const pDev)
{
	(void)pDev;
	s_Events++;
	s_Io[15] = 1;
}
#endif

// Time passes. With the early wakeup interrupt enabled, in steps of 100 us
// with the interrupt serviced when pending.
static void Run(uint32_t Us)
{
#ifdef IWDG_EWCR_EWIE
	if (NVIC_GetEnableIRQ(IWDG_IRQn))
	{
		for (uint32_t t = 0; t < Us; t += 100)
		{
			usDelay(100);
			if (s_Io[11] & 2)
			{
				IWDG_IRQHandler();
				Check((s_Io[11] & 2) == 0);
			}
		}

		return;
	}
#endif
	usDelay(Us);
}

// Refusals, timeout, freeze, reload channels, then a channel stops
static void Basic(void)
{
	WdtDev_t dev;
	WdtCfg_t cfg = Cfg(2000);
	uint32_t div, ticks, ms;

	Check(!WdtInit(nullptr, &cfg) && !WdtInit(&dev, nullptr));
	cfg.DevNo = 1;
	Check(!WdtInit(&dev, &cfg));
	cfg = Cfg(0);
	Check(!WdtInit(&dev, &cfg));
	cfg = Cfg(2000);
	cfg.bRunSleep = false;
	Check(!WdtInit(&dev, &cfg));
	cfg = Cfg(2000);
	cfg.NbChan = 33;
	Check(!WdtInit(&dev, &cfg));
	cfg = Cfg(MaxMs() + 1);
	Check(!WdtInit(&dev, &cfg));
	cfg = Cfg(2000);
	cfg.msWindow = 2000;
	Check(!WdtInit(&dev, &cfg));
#ifndef IWDG_WINR_WIN
	cfg.msWindow = 100;
	Check(!WdtInit(&dev, &cfg));
#endif
#ifndef IWDG_EWCR_EWIE
	cfg = Cfg(2000);
	cfg.EvtHandler = [](WdtDev_t * const) {};
	Check(!WdtInit(&dev, &cfg));
#endif
	// Nothing started, no key written
	Check(s_Io[19] == 0 && (s_Io[11] & 1) == 0);

	// Longest timeout: 4096 ticks of the largest divider
	cfg = Cfg(MaxMs());
	Check(WdtInit(&dev, &cfg) && Expect(MaxMs(), &div, &ticks, &ms) && div == (4U << TEST_PR_MAX) &&
		  dev.msTimeout == ms);
	cfg = Cfg(1);
	Check(WdtInit(&dev, &cfg) && Expect(1, &div, &ticks, &ms) && dev.msTimeout == ms);

	// 2000 ms, one channel, freeze while halted
	cfg = Cfg(2000);
	cfg.bRunHalt = true;
	Check(WdtInit(&dev, &cfg) && !Frozen());
	cfg.bRunHalt = false;
	Check(WdtInit(&dev, &cfg) && Frozen() && Expect(2000, &div, &ticks, &ms) && dev.msTimeout == ms &&
		  dev.NbChan == 1 && dev.DevNo == 0 && !WdtRunning(&dev));
#ifdef RCC_APB2ENR_DBGMCUEN
	Check(RCC->APB2ENR & RCC_APB2ENR_DBGMCUEN);
#endif
	Check(s_Io[19] == 0 && (s_Io[11] & 1) == 0);

	// Nothing before Init
	WdtDev_t none = {};
	none.NbChan = 1;
	Check(!WdtStart(&none) && !WdtStart(nullptr));
	WdtReload(&none, 0);
	WdtReload(nullptr, 0);
	Check(s_Io[19] == 0);

	// A channel reloaded before the start: the start reloads the IWDG, the
	// channels count from there
	cfg.NbChan = 2;
	Check(WdtInit(&dev, &cfg));
	WdtReload(&dev, 0);
	Check(WdtStart(&dev) && WdtRunning(&dev) && (s_Io[11] & 1));
	Check(s_Io[16] == div && s_Io[17] == ticks - 1);
	uint32_t last = s_Io[20];
	Run(1000);
	WdtReload(&dev, 1);
	Check(s_Io[20] == last);
	WdtReload(&dev, 0);
	Check(s_Io[20] == Now());
	cfg.NbChan = 1;
	Check(WdtInit(&dev, &cfg));
#ifdef IWDG_WINR_WIN
	Check(s_Io[18] == 0xFFF);
#endif
	uint32_t keys = s_Io[19];
	Check(WdtStart(&dev) && s_Io[19] == keys);

	for (int i = 0; i < 4; i++)
	{
		Run(ms * 900);
		WdtReload(&dev, 0);
		Check(s_Io[20] == Now());
	}

	// Init while running: the configuration is kept, the channels change
	uint32_t ms2000 = ms;

	cfg = Cfg(5000);
	cfg.NbChan = 3;
	Check(WdtInit(&dev, &cfg) && dev.msTimeout == ms2000 && dev.NbChan == 3 && s_Io[17] == ticks - 1);
	Check(WdtStart(&dev) && s_Io[17] == ticks - 1);

	// The IWDG reloads once every channel has reloaded
	last = s_Io[20];
	Run(1000);
	WdtReload(&dev, 0);
	WdtReload(&dev, 1);
	WdtReload(&dev, 1);
	Check(s_Io[20] == last);
	keys = s_Io[19];
	WdtReload(&dev, 3);
	WdtReload(&dev, -1);
	Check(s_Io[19] == keys);
	WdtReload(&dev, 2);
	Check(s_Io[20] == Now());
	last = Now();
	Run(1000);
	WdtReload(&dev, 2);
	WdtReload(&dev, 2);
	WdtReload(&dev, 0);
	Check(s_Io[20] == last);
	WdtReload(&dev, 1);
	Check(s_Io[20] == Now());

	// Init again: the channels count from there
	last = Now();
	Run(1000);
	WdtReload(&dev, 0);
	WdtReload(&dev, 1);
	Check(WdtInit(&dev, &cfg));
	WdtReload(&dev, 2);
	Check(s_Io[20] == last);
	WdtReload(&dev, 0);
	WdtReload(&dev, 1);
	Check(s_Io[20] == Now());

	// Channel 1 stops: reset one timeout after the last reload of the IWDG
	ExpectReset(Now(), ms2000, 1);
	for (int i = 0; i < 20; i++)
	{
		Run(ms2000 * 300);
		WdtReload(&dev, 0);
		WdtReload(&dev, 2);
	}
	Fail();
}

#ifdef IWDG_WINR_WIN
// Reloads at the window edge, then one too soon
static void Window(void)
{
	WdtDev_t dev;
	WdtCfg_t cfg = Cfg(5000);
	uint32_t div, ticks, ms;

	// 1001 ms is not a whole number of ticks of the 5000 ms timeout: 500.5
	// ticks of 2 ms at 32 kHz, 625.6 of 1.6 ms at 40 kHz, 578.6 at 37 kHz
	cfg.msWindow = 1001;
	Check(WdtInit(&dev, &cfg) && Expect(5000, &div, &ticks, &ms) && dev.msTimeout == ms && WdtStart(&dev));

	// After the window, then at its edge
	Run(3000000);

	// Window ticks rounded down
	uint32_t win = (uint32_t)(1001ULL * s_Io[9] / (div * 1000ULL));

	Check(s_Io[16] == div && s_Io[17] == ticks - 1 && s_Io[18] == ticks - 1 - win);
	WdtReload(&dev, 0);
	Check(s_Io[20] == Now());
	Run(1001000);
	WdtReload(&dev, 0);
	Check(s_Io[20] == Now());
	Run(3000000);
	WdtReload(&dev, 0);

	// Too soon
	Run(900000);
	s_Io[6] = 2;
	s_Io[4] = Now();
	s_Io[5] = Now();
	WdtReload(&dev, 0);
	Fail();
}
#endif

// IWDG running from the reset: reset configuration kept and reported
static void Hardware(void)
{
#ifndef IWDG_SR_ONF
	OptionSw(false);
#endif
	s_Io[12] = 1;

	WdtDev_t dev;
	WdtCfg_t cfg = Cfg(2000);
	uint32_t ms = (uint32_t)((4096ULL * 4 * 1000 + s_Io[9] - 1) / s_Io[9]);

	Check(WdtRunning(&dev) && WdtInit(&dev, &cfg) && dev.msTimeout == ms && WdtStart(&dev));
	Check(s_Io[16] == 4 && s_Io[17] == 0xFFF);
#ifndef IWDG_EWCR_EWIE
	Check(s_Io[19] == 0);
#endif

	for (int i = 0; i < 3; i++)
	{
		Run(ms * 600);
		WdtReload(&dev, 0);
	}

	ExpectReset(s_Io[20], ms, 1);
	Run(ms * 2000);
	Fail();
}

// Timeout of the boot loader configuration: reload 100, prescaler 7 (256)
// or 9 (1024) aliasing the largest divider
static uint32_t BootLoaderMs(void)
{
	return (uint32_t)((101ULL * (4U << TEST_PR_MAX) * 1000ULL + s_Io[9] - 1) / s_Io[9]);
}

#ifdef IWDG_SR_ONF
// IWDG started by a boot loader, seen (SR ONF): configuration kept
static void BootLoader(void)
{
	s_Io[12] = 2;
	usDelay(10000);

	WdtDev_t dev;
	WdtCfg_t cfg = Cfg(2000);

	Check(WdtRunning(&dev) && WdtInit(&dev, &cfg) && dev.msTimeout == BootLoaderMs() && WdtStart(&dev));
	Check(s_Io[16] == (4U << TEST_PR_MAX) && s_Io[17] == 100);
	WdtReload(&dev, 0);
	ExpectReset(s_Io[20], BootLoaderMs(), 1);
	Run(BootLoaderMs() * 2000);
	Fail();
}
#else
// Hardware watchdog with the configuration of a boot loader: kept
static void HardwareBootLoader(void)
{
	OptionSw(false);
	s_Io[12] = 2;
	usDelay(10000);

	WdtDev_t dev;
	WdtCfg_t cfg = Cfg(2000);

	Check(WdtRunning(&dev) && WdtInit(&dev, &cfg) && dev.msTimeout == BootLoaderMs() && WdtStart(&dev));
	Check(s_Io[16] == (4U << TEST_PR_MAX) && s_Io[17] == 100 && s_Io[19] == 0);
	WdtReload(&dev, 0);
	ExpectReset(s_Io[20], BootLoaderMs(), 1);
	Run(BootLoaderMs() * 2000);
	Fail();
}

// IWDG started by a boot loader, not seen: WdtStart writes the configuration
// once the updates of the boot loader are done
static void BootLoader(void)
{
	s_Io[12] = 2;

	WdtDev_t dev;
	WdtCfg_t cfg = Cfg(2000);
	uint32_t div, ticks, ms;

	Check(!WdtRunning(&dev) && WdtInit(&dev, &cfg) && Expect(2000, &div, &ticks, &ms) && dev.msTimeout == ms);
	Check(WdtStart(&dev) && WdtRunning(&dev));
	Check(s_Io[16] == div && s_Io[17] == ticks - 1);
	ExpectReset(s_Io[20], ms, 1);
	Run(ms * 2000);
	Fail();
}
#endif

#ifdef IWDG_EWCR_EWIE
// Early wakeup interrupt one tick before the reset
static void EarlyWakeup(void)
{
	WdtDev_t dev;
	WdtCfg_t cfg = Cfg(2000);
	uint32_t div, ticks, ms;

	cfg.EvtHandler = Event;
	cfg.IntPrio = 5;
	Check(WdtInit(&dev, &cfg) && Expect(2000, &div, &ticks, &ms) && NVIC_GetEnableIRQ(IWDG_IRQn) &&
		  NVIC_GetPriority(IWDG_IRQn) == 5 && dev.EvtHandler == Event);
	Check(WdtStart(&dev) && TEST_IWDG->EWCR == (IWDG_EWCR_EWIE | 2));

	for (int i = 0; i < 3; i++)
	{
		Run(ms * 900);
		WdtReload(&dev, 0);
	}
	Check(s_Events == 0);

	// No handler: the interrupt is off
	cfg.EvtHandler = nullptr;
	Check(WdtInit(&dev, &cfg) && !NVIC_GetEnableIRQ(IWDG_IRQn) && (TEST_IWDG->EWCR & IWDG_EWCR_EWIE) == 0);
	cfg.EvtHandler = Event;
	Check(WdtInit(&dev, &cfg) && NVIC_GetEnableIRQ(IWDG_IRQn));

	WdtReload(&dev, 0);
	ExpectReset(Now(), ms, 1);
	Run(ms * 2000);
	Fail();
}
#endif

#if defined(RCC_BDCR1_LSI1PREDIV) || defined(RCC_CSR_LSIPREDIV)
// LSI divided by 128, 250 Hz
static void Predivider(void)
{
#if defined(RCC_BDCR1_LSI1PREDIV)
	RCC->BDCR1 |= RCC_BDCR1_LSI1PREDIV;
#else
	RCC->CSR |= RCC_CSR_LSIPREDIV;
#endif

	WdtDev_t dev;
	WdtCfg_t cfg = Cfg(60000);
	uint32_t div, ticks, ms;

	Check(s_Io[9] < 1000 && WdtInit(&dev, &cfg) && Expect(60000, &div, &ticks, &ms) && div == 4 &&
		  dev.msTimeout == ms);
#ifdef IWDG_EWCR_EWIE
	// The interrupt comes at counter 1: refused with 1 tick (1 ms) or 2 ticks
	// (20 ms, 1.25 ticks of 16 ms), allowed with 3 (40 ms)
	WdtCfg_t one = Cfg(1);
	one.EvtHandler = Event;
	Check(!WdtInit(&dev, &one));
	one.msTimeout = 20;
	Check(!WdtInit(&dev, &one));
	one.msTimeout = 40;
	Check(WdtInit(&dev, &one));
#endif
	cfg = Cfg(MaxMs());
	Check(WdtInit(&dev, &cfg) && Expect(MaxMs(), &div, &ticks, &ms) && dev.msTimeout == ms);
	cfg = Cfg(60000);
	Check(WdtInit(&dev, &cfg) && Expect(60000, &div, &ticks, &ms) && WdtStart(&dev) && s_Io[16] == 4 &&
		  s_Io[17] == ticks - 1);
	ExpectReset(s_Io[20], 60000, 1);
	Run(70000000);
	Fail();
}
#endif

extern "C" void ResetHandler()
{
	extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
	uint32_t *s = &_sidata;

	for (uint32_t *d = &_sdata; d < &_edata;)
	{
		*d++ = *s++;
	}
	for (uint32_t *d = &_sbss; d < &_ebss;)
	{
		*d++ = 0;
	}

	uint32_t feat = TEST_PR_MAX << 8;

#ifdef IWDG_WINR_WIN
	feat |= 1;
#endif
#ifdef IWDG_EWCR_EWIE
	feat |= 2;
#endif
#ifdef IWDG_SR_ONF
	feat |= 4;
#endif
	s_Io[8] = feat;
	s_Io[7] = (uint32_t)TEST_IWDG;
#if defined(RCC_BDCR1_LSI1PREDIV)
	s_Io[13] = (uint32_t)&RCC->BDCR1;
	s_Io[14] = RCC_BDCR1_LSI1PREDIV;
#elif defined(RCC_CSR_LSIPREDIV)
	s_Io[13] = (uint32_t)&RCC->CSR;
	s_Io[14] = RCC_CSR_LSIPREDIV;
#endif
	OptionSw(true);

	switch (s_Io[0])
	{
		case 0:
			Basic();
			break;
#ifdef IWDG_WINR_WIN
		case 1:
			Window();
			break;
#endif
		case 2:
			Hardware();
			break;
		case 3:
			BootLoader();
			break;
#ifndef IWDG_SR_ONF
		case 6:
			HardwareBootLoader();
			break;
#endif
#ifdef IWDG_EWCR_EWIE
		case 4:
			EarlyWakeup();
			break;
#endif
#if defined(RCC_BDCR1_LSI1PREDIV) || defined(RCC_CSR_LSIPREDIV)
		case 5:
			Predivider();
			break;
#endif
	}

	s_Io[2] = s_Checks;

	while (1)
	{
	}
}

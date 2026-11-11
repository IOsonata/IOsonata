/**-------------------------------------------------------------------------
@file timer_test.cpp
@brief Exercise the production Timer Group driver and IRQ routing with host MMIO.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <cassert>
#include <cstdint>
#include <map>
#include <sys/mman.h>
#include "esp32xx.h"

#define __INTERRUPT_H__
static uintptr_t s_Masked;
static uintptr_t DisableInterrupt()
{
	uintptr_t previous;

	previous = s_Masked;
	s_Masked = 1;
	return previous;
}
static void EnableInterrupt(uintptr_t Previous)
{
	s_Masked = Previous;
}

static std::map<uintptr_t, uint32_t> s_Regs;
static bool s_Started;
struct Register {
	uintptr_t Address;
	operator uint32_t() const
	{
		return s_Regs[Address];
	}
	void operator=(uint32_t Value)
	{
		uintptr_t base;
		unsigned offset;

		s_Regs[Address] = Value;
		base = Address & ~uintptr_t(0xFFF);
		if (base != ESP32_TIMG0_BASE
#ifdef ESP32_TIMG1_BASE
			&& base != ESP32_TIMG1_BASE
#endif
		)
		{
			return;
		}
		offset = Address - base;
		if (offset == 0x0C)
		{
			s_Regs[Address] = 0;
		}
		if (offset == 0x20)
		{
			s_Regs[base + 4] = s_Regs[base + 0x18];
			s_Regs[base + 8] = s_Regs[base + 0x1C];
		}
		if (offset == 0x7C)
		{
			s_Regs[base + 0x78] &= ~Value;
		}
		if (offset == 0 && (Value & (1U << 31)))
		{
			s_Started = true;
		}
	}
	void operator|=(uint64_t Value)
	{
		*this = uint32_t(*this) | Value;
	}
	void operator&=(uint64_t Value)
	{
		*this = uint32_t(*this) & Value;
	}
};
#define ESP32_REG32(addr) Register{uintptr_t(addr)}
#include "../../RISCV/Espressif/src/timer_esp32.cpp"
#include "../../RISCV/Espressif/src/esp32xx_irq.c"
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
#include "../../RISCV/Espressif/src/Vectors_esp32.c"
#else
static Esp32IrqHandler s_HostHandlers[32];
void Esp32C3SetCpuIrqHandler(uint32_t Cpu, Esp32IrqHandler Handler)
{
	assert(s_Masked);
	s_HostHandlers[Cpu] = Handler;
}
void Esp32C3ClearCpuIrqHandler(uint32_t Cpu)
{
	s_HostHandlers[Cpu] = nullptr;
}
void Esp32C3SetSourceIrqHandler(uint32_t Source, Esp32IrqHandler Handler)
{
	(void)Source;
	(void)Handler;
}
void Esp32C3ClearSourceIrqHandler(uint32_t Source)
{
	(void)Source;
}
void Esp32C3IntMtxDispatch(void) {}
#endif

static unsigned s_Events;
static unsigned s_Triggers;
static bool s_ExpectTrigger;
static void Event(TimerDev_t *pDev, uint32_t Events)
{
	assert(pDev != nullptr);
	assert(Events == TIMER_EVT_TRIGGER(0));
	if (s_ExpectTrigger)
	{
		assert(s_Triggers == s_Events);
	}
	s_Events++;
}
static void Trigger(TimerDev_t *pDev, int TriggerNo, void *pContext)
{
	assert(pDev != nullptr && TriggerNo == 0 && pContext == &s_Triggers);
	s_Triggers++;
}
static void SetCount(unsigned DevNo, uint64_t Count)
{
	s_Regs[s_TimerBases[DevNo] + 4] = uint32_t(Count);
	s_Regs[s_TimerBases[DevNo] + 8] = uint32_t(Count >> 32);
}

int main()
{
	TimerDev_t timers[ESP32_TIMG_COUNT] = {};
	TimerDev_t duplicate = {};
	TimerCfg_t cfg = {};
	uint64_t actual;
	uint64_t ticks;
	uint32_t base;
	uint32_t divider;
	unsigned source;
	int dev;
#if defined(ESP32C5) || defined(ESP32C61)
	void *mapped;

	mapped = mmap(reinterpret_cast<void *>(ESP32_CLIC_BASE), 8192,
		PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	assert(mapped == reinterpret_cast<void *>(ESP32_CLIC_BASE));
#endif
#if defined(ESP32C2)
	assert(ESP32_TIMG_COUNT == 1 && s_TimerSources[0] == 23);
	assert(ESP32_TIMER_APB_HZ == 40000000);
#elif defined(ESP32C3)
	assert(s_TimerSources[0] == 32 && s_TimerSources[1] == 34);
#elif defined(ESP32C5)
	assert(s_TimerSources[0] == 57 && s_TimerSources[1] == 59);
	assert(ESP32_TIMER_PCR_OFFSET == 0x54 && ESP32_TIMER_PCR_SEL == 2);
#elif defined(ESP32C6)
	assert(s_TimerSources[0] == 51 && s_TimerSources[1] == 54);
	assert(ESP32_TIMER_PCR_OFFSET == 0x3C && ESP32_TIMER_PCR_SEL == 1);
#elif defined(ESP32C61)
	assert(s_TimerSources[0] == 46 && s_TimerSources[1] == 49);
	assert(ESP32_TIMER_PCR_OFFSET == 0x40 && ESP32_TIMER_PCR_SEL == 2);
#endif
	for (source = 0; source < ESP32_INTMTX_NUM_SOURCES; source++)
	{
		ESP32_INTMTX_SOURCE_MAP_REG(source) = ESP32_INTMTX_DISABLED_MAP;
	}
	assert(TimerGetHighFreqDevCount() == ESP32_TIMG_COUNT);
	assert(TimerGetLowFreqDevCount() == 0 && TimerGetHighFreqDevNo() == 0);
	assert(!TimerInit(nullptr, &cfg));
	cfg.DevNo = ESP32_TIMG_COUNT;
	assert(!TimerInit(&duplicate, &cfg));
	cfg.Freq = 1000000;
	cfg.EvtHandler = Event;
	for (dev = 0; dev < ESP32_TIMG_COUNT; dev++)
	{
		cfg.DevNo = dev;
		assert(TimerInit(&timers[dev], &cfg));
		assert(!TimerInit(&duplicate, &cfg));
		assert(timers[dev].Freq == 1000000);
		assert(timers[dev].ExecuteTask == nullptr);
		assert(s_Regs[ESP32_INTMTX_BASE + s_TimerSources[dev] * 4] ==
			ESP32_TIMER_CPUINT + dev + ESP32_INTMTX_CPU_OFFSET);
		base = s_TimerBases[dev];
#if defined(ESP32C2) || defined(ESP32C3)
		assert(s_Regs[ESP32_SYSTEM_BASE + 0x10] & (1U << (dev == 0 ? 13 : 15)));
		assert(s_Regs[base + 0xFC] & (1U << 30));
#else
		assert((s_Regs[ESP32_PCR_BASE + ESP32_TIMER_PCR_OFFSET + dev * 12 + 4]
			& (3U << 20)) == (ESP32_TIMER_PCR_SEL << 20));
#endif
		assert(nsTimerEnableTrigger(&timers[dev], 0, 100000000,
			TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr) == 100000000);
		SetCount(dev, 100000);
		s_TimerHandlers[dev]();
		assert(s_Events == unsigned(dev + 1));
		assert(s_Timer[dev].Deadline == 200000);
		SetCount(dev, 550000);
		s_TimerHandlers[dev]();
		assert(s_Timer[dev].Deadline == 600000);
		s_Events--;
		TimerDisable(&timers[dev]);
		s_Started = false;
		assert(TimerSetFrequency(&timers[dev], 2000000) == 2000000);
		assert(!s_Started);
		assert(TimerGetTickCount(&timers[dev]) == 0);
		assert(TimerEnable(&timers[dev]));
		assert(s_Started);
		TimerDisableTrigger(&timers[dev], 0);
		assert(nsTimerEnableTrigger(&timers[dev], 0, 120000000,
			TIMER_TRIG_TYPE_SINGLE, Trigger, &s_Triggers) == 120000000);
		s_ExpectTrigger = true;
		SetCount(dev, 240000);
		s_TimerHandlers[dev]();
		s_TimerHandlers[dev]();
		assert(s_Triggers == unsigned(dev + 1));
		assert(!s_Timer[dev].TriggerActive);
		assert(s_Events == unsigned(dev + 2));
		s_Events--;
		s_ExpectTrigger = false;
		SetCount(dev, (1ULL << 32) + 42);
		assert(TimerGetTickCount(&timers[dev]) == (1ULL << 32) + 42);
	}
	assert(Esp32ClockSelect(UINT32_MAX, &divider) == ESP32_TIMER_APB_HZ / 2);
	assert(divider == 2);
	assert(Esp32ClockSelect(0, &divider) == ESP32_TIMER_APB_HZ / 2);
	assert(Esp32ClockSelect(1, &divider) == ESP32_TIMER_APB_HZ / ESP32_TIMER_MAX_DIV);
	// 80 MHz / 2 = 40 MHz, / 3 = 26.67 MHz: 33 MHz must choose / 3.
#if !defined(ESP32C2)
	assert(Esp32ClockSelect(33000000, &divider) == 26666666);
#endif
	actual = Esp32PeriodToTicks(1000, UINT64_MAX, &ticks);
	assert(actual > UINT64_MAX - 1000000);
	assert(ticks <= ESP32_TIMER_MAX_TICKS);
	TimerDisable(&timers[0]);
	s_Started = false;
	TimerSetFrequency(&timers[0], 1);
	assert((s_Regs[s_TimerBases[0]] & ESP32_T0_DIV_MASK) == 0);
	assert(!s_Started);
	assert(s_Masked == 0);
}

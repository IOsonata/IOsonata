/**-------------------------------------------------------------------------
@file gpio_irq_test.cpp
@brief Exercise GPIO interrupt registration and release with production routing.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <cassert>
#include <cstdint>
#include <map>
#include <sys/mman.h>
#include "esp32xx_gpio.h"

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
struct Register {
	uintptr_t Address;
	operator uint32_t() const
	{
		return s_Regs[Address];
	}
	void operator=(uint32_t Value)
	{
		s_Regs[Address] = Value;
		if (Address == ESP32_GPIO_BASE + ESP32_GPIO_STATUS_W1TC_OFFSET)
		{
			assert(s_Masked);
			s_Regs[ESP32_GPIO_BASE + ESP32_GPIO_STATUS_REG_OFFSET] &= ~Value;
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
#undef ESP32_REG32
#define ESP32_REG32(addr) Register{uintptr_t(addr)}
#include "../../RISCV/Espressif/src/iopincfg_esp32.c"
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

static unsigned s_First;
static unsigned s_Second;
static unsigned s_Other;
static void Other(int IntNo, void *pContext)
{
	assert(IntNo == 7 && pContext == &s_Other);
	s_Other++;
}
static void First(int IntNo, void *pContext)
{
	assert(IntNo == 42 && pContext == &s_First);
	s_First++;
}
static void Second(int IntNo, void *pContext)
{
	assert(IntNo == 42 && pContext == &s_Second);
	s_Second++;
}
static void Edge(unsigned Pin)
{
	uintptr_t state;

	s_Regs[ESP32_GPIO_BASE + ESP32_GPIO_STATUS_REG_OFFSET] |= 1U << Pin;
	state = DisableInterrupt();
	ESP32_GPIO_IRQHandler();
	EnableInterrupt(state);
}
int main()
{
	unsigned source;
	uint32_t before;
#if defined(ESP32C5) || defined(ESP32C61)
	void *mapped;

	mapped = mmap(reinterpret_cast<void *>(ESP32_CLIC_BASE), 8192,
		PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	assert(mapped == reinterpret_cast<void *>(ESP32_CLIC_BASE));
#endif
	for (source = 0; source < ESP32_INTMTX_NUM_SOURCES; source++)
	{
		ESP32_INTMTX_SOURCE_MAP_REG(source) = ESP32_INTMTX_DISABLED_MAP;
	}
	assert(!IOPinEnableInterrupt(42, 2, 1, 4, IOPINSENSE_TOGGLE, First, &s_First));
	assert(!IOPinEnableInterrupt(42, 2, 0, UINT32_MAX, IOPINSENSE_TOGGLE, First, &s_First));
	assert(!IOPinEnableInterrupt(42, 2, 0, 4, (IOPINSENSE)99, First, &s_First));
	assert(!IOPinEnableInterrupt(42, 2, 0, 4, IOPINSENSE_TOGGLE, nullptr, nullptr));
	assert(IOPinEnableInterrupt(42, 2, 0, 4, IOPINSENSE_TOGGLE, First, &s_First));
	assert(s_Regs[ESP32_INTMTX_BASE + 4 * ESP32_GPIO_INTR_SOURCE_ID] ==
		ESP32_GPIO_INTR_CPU_INT_ID + ESP32_INTMTX_CPU_OFFSET);
	Edge(4);
	assert(s_First == 1);
	assert(!IOPinEnableInterrupt(42, 2, 0, 5, IOPINSENSE_TOGGLE, First, &s_First));
	assert(s_GpioIntEnableMask == (1U << 4));
	assert(IOPinEnableInterrupt(42, 2, 0, 4, IOPINSENSE_HIGH_TRANSITION, Second, &s_Second));
	Edge(4);
	assert(s_First == 1 && s_Second == 1);
	IOPinDisableInterrupt(4);
	assert(s_GpioIntEnableMask == (1U << 4));
	assert(IOPinEnableInterrupt(7, 2, 0, 5, IOPINSENSE_TOGGLE, Other, &s_Other));
	IOPinDisableInterrupt(42);
	assert(s_GpioIntEnableMask == (1U << 5));
	Edge(5);
	assert(s_Other == 1);
	IOPinDisableInterrupt(7);
	assert(s_GpioIntEnableMask == 0);
	assert((uint32_t(ESP32_GPIO_PIN_REG32(4)) & ESP32_GPIO_PIN_INT_ENA_Msk) == 0);
	Edge(4);
	assert(s_Second == 1);
	// Disabling by physical pin must also release a custom interrupt number.
	assert(IOPinEnableInterrupt(42, 2, 0, 5, IOPINSENSE_LOW_TRANSITION, First, &s_First));
	IOPinDisable(0, 5);
	assert(s_GpioIntEnableMask == 0);
	assert(IOPinEnableInterrupt(42, 2, 0, 4, IOPINSENSE_TOGGLE, First, &s_First));
	assert(IOPinEnableInterrupt(-1, 2, 0, 4, IOPINSENSE_DISABLE, nullptr, nullptr));
	assert(s_GpioIntEnableMask == 0);
	assert(IOPinAllocateInterrupt(2, 0, 4, IOPINSENSE_TOGGLE, First, &s_First) == 4);
	IOPinDisableInterrupt(4);
	assert(s_GpioIntEnableMask == 0);
	// Re-registration and release preserve an existing CPU interrupt mask.
	s_Masked = 1;
	assert(IOPinEnableInterrupt(42, 2, 0, 4, IOPINSENSE_TOGGLE, First, &s_First));
	assert(s_Masked == 1);
	before = uint32_t(ESP32_GPIO_PIN_REG32(4));
	IOPinDisable(1, 4);
	assert(uint32_t(ESP32_GPIO_PIN_REG32(4)) == before);
	IOPinDisableInterrupt(42);
	assert(s_Masked == 1 && s_GpioIntEnableMask == 0);
}

/**-------------------------------------------------------------------------
@file irq_test.cpp
@brief Execute ESP32 interrupt routing and dispatch against mapped MMIO memory.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <cassert>
#include <cstdint>
#include <sys/mman.h>

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

#include "../../RISCV/Espressif/src/esp32xx_irq.c"
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
#include "../../RISCV/Espressif/src/Vectors_esp32.c"
#else
static Esp32IrqHandler s_CpuHandlers[32];
static Esp32IrqHandler s_SourceHandlers[ESP32_INTMTX_NUM_SOURCES];
void Esp32C3SetCpuIrqHandler(uint32_t Cpu, Esp32IrqHandler Handler)
{
	assert(s_Masked);
	s_CpuHandlers[Cpu] = Handler;
}
void Esp32C3ClearCpuIrqHandler(uint32_t Cpu) { s_CpuHandlers[Cpu] = nullptr; }
void Esp32C3SetSourceIrqHandler(uint32_t Source, Esp32IrqHandler Handler)
{
	assert(s_Masked);
	s_SourceHandlers[Source] = Handler;
}
void Esp32C3ClearSourceIrqHandler(uint32_t Source) { s_SourceHandlers[Source] = nullptr; }
void Esp32C3IntMtxDispatch(void) {}
#endif

static unsigned s_First;
static unsigned s_Second;
static void First() { s_First++; }
static void Second() { s_Second++; }

static void Map(uintptr_t Address, size_t Size)
{
	void *mapped;

	mapped = mmap(reinterpret_cast<void *>(Address), Size, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	assert(mapped == reinterpret_cast<void *>(Address));
}

static bool CpuEnabled(unsigned Cpu)
{
#if defined(ESP32C5) || defined(ESP32C61)
	return CLIC_INT_IE_REG(ESP32_CLIC_CTRL_BASE, Cpu + 16) != 0;
#else
	return (ESP32_CPU_INT_ENABLE_REG & (1U << Cpu)) != 0;
#endif
}

int main()
{
	unsigned source;
	unsigned uartSource;
	unsigned saved;

	Map(ESP32_INTMTX_BASE, 4096);
#if defined(ESP32C6)
	Map(ESP32_INTPRI_BASE, 4096);
#endif
#if defined(ESP32C5) || defined(ESP32C61)
	Map(ESP32_CLIC_BASE, 8192);
#endif
	for (source = 0; source < ESP32_INTMTX_NUM_SOURCES; source++)
	{
		ESP32_INTMTX_SOURCE_MAP_REG(source) = ESP32_INTMTX_DISABLED_MAP;
	}
#if defined(ESP32C2)
	uartSource = 18;
#elif defined(ESP32C3)
	uartSource = 22;
#elif defined(ESP32C5)
	uartSource = 48;
#elif defined(ESP32C6)
	uartSource = 44;
#else
	uartSource = 42;
#endif
	assert(!Esp32EnableSourceIrq(ESP32_INTMTX_NUM_SOURCES, 2, 1, First));
	assert(!Esp32EnableSourceIrq(uartSource, ESP32_INTMTX_DISABLED_CPU, 1, First));
	assert(!Esp32EnableSourceIrq(uartSource, 2, 1, nullptr));
	assert(!s_Masked);
#if defined(ESP32C6)
	assert(!Esp32EnableSourceIrq(uartSource, 3, 1, First));
	assert(!Esp32EnableSourceIrq(uartSource, 4, 1, First));
	assert(!Esp32EnableSourceIrq(uartSource, 7, 1, First));
#endif
	assert(Esp32EnableSourceIrq(uartSource, 2, 99, First));
	assert(ESP32_INTMTX_SOURCE_MAP_REG(uartSource) == 2 + ESP32_INTMTX_CPU_OFFSET);
	assert(CpuEnabled(2));
	assert(s_CpuHandlers[2] == First);
	assert(!s_Masked);
#if defined(ESP32C5) || defined(ESP32C61)
	assert(CLIC_INT_CTL_REG(ESP32_CLIC_CTRL_BASE, 18) == 255);
	assert(CLIC_INT_ATTR_REG(ESP32_CLIC_CTRL_BASE, 18) == 0xC0);
	assert(!CpuEnabled(6));
	assert(ESP32_CLIC_INT_CONFIG_REG == 3);
#else
	assert(ESP32_CPU_INT_PRI_REG(2) == 15);
#endif
	// A direct owner cannot be overwritten by a different peripheral.
	assert(!Esp32EnableSourceIrq(uartSource - 1, 2, 1, Second));
	assert(s_CpuHandlers[2] == First);
	// Wrong CPU must not detach an existing route.
	Esp32DisableSourceIrq(uartSource, 8);
	assert(CpuEnabled(2));
	assert(ESP32_INTMTX_SOURCE_MAP_REG(uartSource) == 2 + ESP32_INTMTX_CPU_OFFSET);
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
	assert(Esp32DispatchCpuIrq(0x80000000U | (2 + ESP32_INTMTX_CPU_OFFSET)));
	assert(s_First == 1);
	assert(!Esp32DispatchCpuIrq(2));
	assert(!Esp32DispatchCpuIrq(0x80000000U | 48));
#endif
	Esp32DisableSourceIrq(uartSource, 2);
	assert(!CpuEnabled(2));
	assert(s_CpuHandlers[2] == nullptr);
	assert(ESP32_INTMTX_SOURCE_MAP_REG(uartSource) == ESP32_INTMTX_DISABLED_MAP);

	// Source zero is included in shared ownership and last-owner detection.
	Esp32InstallIrqHandler(0, 8, First, ESP32_IRQ_SHARED);
	assert(Esp32RouteSourceIrq(0, 8, 1));
	Esp32InstallIrqHandler(uartSource, 8, Second, ESP32_IRQ_SHARED);
	assert(Esp32RouteSourceIrq(uartSource, 8, 1));
	assert(s_CpuHandlers[8] == Esp32C3IntMtxDispatch);
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
	ESP32_INTMTX_INTR_STATUS_0_REG = 1;
	if (uartSource < 32)
	{
		ESP32_INTMTX_INTR_STATUS_0_REG |= 1UL << uartSource;
	}
	else
	{
		ESP32_INTMTX_INTR_STATUS_1_REG = 1UL << (uartSource - 32);
	}
	saved = s_First;
	assert(Esp32DispatchCpuIrq(0x80000000U | (8 + ESP32_INTMTX_CPU_OFFSET)));
	assert(s_First == saved + 1 && s_Second == 1);
#endif
	Esp32DisableSourceIrq(uartSource, 8);
	assert(CpuEnabled(8));
	assert(s_CpuHandlers[8] == Esp32C3IntMtxDispatch);
	Esp32DisableSourceIrq(0, 8);
	assert(!CpuEnabled(8));
	assert(s_CpuHandlers[8] == nullptr);

	// Standalone route/install/disable preserve an existing interrupt mask.
	s_Masked = 1;
	Esp32InstallIrqHandler(uartSource, 2, First, ESP32_IRQ_DIRECT);
	assert(Esp32RouteSourceIrq(uartSource, 2, 0));
	assert(s_Masked == 1);
	Esp32DisableSourceIrq(uartSource, 2);
	assert(s_Masked == 1);
	return 0;
}

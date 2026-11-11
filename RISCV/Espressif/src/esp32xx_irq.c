/**-------------------------------------------------------------------------
@file	esp32xx_irq.c

@brief	Cross-chip peripheral interrupt install for the IOsonata ESP32
        RISC-V family.

INTC (C2 / C3 / C6) and CLIC (C5 / C61) implementations of the API declared in
esp32xx_irq.h.  See the header for the surface and the ownership
semantics.

@author	Nguyen Hoan Hoang
@date	May 9, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "coredev/interrupt.h"
#include "esp32xx_irq.h"

static void Esp32IrqFenceIo(void)
{
#if defined(__riscv)
	__asm volatile("fence iorw, iorw" ::: "memory");
#endif
}

static bool Esp32IrqValid(uint32_t Source, uint32_t Cpu)
{
	if (Source >= ESP32_INTMTX_NUM_SOURCES || Cpu == 0 || Cpu >= 32 ||
		Cpu == ESP32_INTMTX_DISABLED_CPU)
	{
		return false;
	}
#if defined(ESP32C6)
	if (Cpu == 3 || Cpu == 4 || Cpu == 7)
	{
		return false;
	}
#endif
	return true;
}

static bool Esp32IrqSourceShares(uint32_t Cpu, uint32_t Exclude)
{
	uint32_t source;
	uint32_t map;

	map = Cpu + ESP32_INTMTX_CPU_OFFSET;
	for (source = 0; source < ESP32_INTMTX_NUM_SOURCES; source++)
	{
		if (source != Exclude && ESP32_INTMTX_SOURCE_MAP_REG(source) == map)
		{
			return true;
		}
	}
	return false;
}

void Esp32GlobalIrqEnable(void)
{
#if defined(__riscv)
	__asm volatile("csrsi mstatus, 0x8" ::: "memory");
#endif
}

bool Esp32RouteSourceIrq(uint32_t source_id, uint32_t cpu_int_id, uint32_t prio)
{
	uintptr_t state;
#if defined(ESP32C5) || defined(ESP32C61)
	uint32_t slot;
#endif

	if (!Esp32IrqValid(source_id, cpu_int_id))
	{
		return false;
	}
	state = DisableInterrupt();
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
	Esp32InitVectors();
#endif
	if (prio == 0)
	{
		prio = 1;
	}
	ESP32_INTMTX_CLOCK_GATE_REG = 1;
#if defined(ESP32C5) || defined(ESP32C61)
	if (prio > 7)
	{
		prio = 7;
	}
	slot = cpu_int_id + ESP32_CLIC_EXT_INTR_NUM_OFFSET;
	// A CLIC slot belongs to the selected CPU input, not to the source ID.
	ClicEnableSlot(ESP32_CLIC_CTRL_BASE, slot, ESP32_CLIC_INT_COUNT,
		CLIC_TRIG_LEVEL_HIGH, prio, ESP32_CLIC_NLBITS);
#else
	if (prio > 15)
	{
		prio = 15;
	}
	// Keep the disabled matrix input masked even if ROM left it enabled.
	ESP32_CPU_INT_ENABLE_REG &= ~(1UL << ESP32_INTMTX_DISABLED_CPU);
	ESP32_CPU_INT_CLEAR_REG = 1UL << cpu_int_id;
	ESP32_CPU_INT_CLEAR_REG = 0;
	ESP32_CPU_INT_TYPE_REG &= ~(1UL << cpu_int_id);
	ESP32_CPU_INT_PRI_REG(cpu_int_id) = prio;
	ESP32_CPU_INT_THRESH_REG = 0;
	ESP32_CPU_INT_ENABLE_REG |= 1UL << cpu_int_id;
#endif
	ESP32_INTMTX_SOURCE_MAP_REG(source_id) = cpu_int_id + ESP32_INTMTX_CPU_OFFSET;
	Esp32IrqFenceIo();
	EnableInterrupt(state);
	return true;
}

void Esp32InstallIrqHandler(uint32_t source_id, uint32_t cpu_int_id,
	Esp32IrqHandler handler, Esp32IrqMode mode)
{
	uintptr_t state;

	if (!Esp32IrqValid(source_id, cpu_int_id) || handler == 0 ||
		(mode != ESP32_IRQ_DIRECT && mode != ESP32_IRQ_SHARED))
	{
		return;
	}
	state = DisableInterrupt();
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
	Esp32InitVectors();
#endif
	if (mode == ESP32_IRQ_DIRECT)
	{
		if (Esp32C3SetCpuIrqHandler != 0)
		{
			Esp32C3SetCpuIrqHandler(cpu_int_id, handler);
		}
	}
	else if (Esp32C3SetSourceIrqHandler != 0 && Esp32C3SetCpuIrqHandler != 0 &&
		Esp32C3IntMtxDispatch != 0)
	{
		Esp32C3SetSourceIrqHandler(source_id, handler);
		Esp32C3SetCpuIrqHandler(cpu_int_id, Esp32C3IntMtxDispatch);
	}
	Esp32IrqFenceIo();
	EnableInterrupt(state);
}

bool Esp32EnableSourceIrq(uint32_t source_id, uint32_t cpu_int_id,
	uint32_t prio, Esp32IrqHandler handler)
{
	uintptr_t state;
	bool ok;

	if (!Esp32IrqValid(source_id, cpu_int_id) || handler == 0)
	{
		return false;
	}
	state = DisableInterrupt();
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
	Esp32InitVectors();
#endif
	if (Esp32C3SetCpuIrqHandler == 0)
	{
		EnableInterrupt(state);
		return false;
	}
	// Never replace the handler on a CPU input owned by another source.
	if (Esp32IrqSourceShares(cpu_int_id, source_id))
	{
		EnableInterrupt(state);
		return false;
	}
	Esp32InstallIrqHandler(source_id, cpu_int_id, handler, ESP32_IRQ_DIRECT);
	ok = Esp32RouteSourceIrq(source_id, cpu_int_id, prio);
	Esp32IrqFenceIo();
	EnableInterrupt(state);
	if (ok)
	{
		Esp32GlobalIrqEnable();
	}
	return ok;
}

void Esp32DisableSourceIrq(uint32_t source_id, uint32_t cpu_int_id)
{
	uintptr_t state;

	if (!Esp32IrqValid(source_id, cpu_int_id))
	{
		return;
	}
	state = DisableInterrupt();
	if (ESP32_INTMTX_SOURCE_MAP_REG(source_id) != cpu_int_id + ESP32_INTMTX_CPU_OFFSET)
	{
		EnableInterrupt(state);
		return;
	}
	ESP32_INTMTX_SOURCE_MAP_REG(source_id) = ESP32_INTMTX_DISABLED_MAP;
	Esp32IrqFenceIo();
	if (Esp32C3ClearSourceIrqHandler != 0)
	{
		Esp32C3ClearSourceIrqHandler(source_id);
	}
	if (!Esp32IrqSourceShares(cpu_int_id, source_id))
	{
#if defined(ESP32C5) || defined(ESP32C61)
		ClicDisableSlot(ESP32_CLIC_CTRL_BASE,
			cpu_int_id + ESP32_CLIC_EXT_INTR_NUM_OFFSET, ESP32_CLIC_INT_COUNT);
#else
		ESP32_CPU_INT_ENABLE_REG &= ~(1UL << cpu_int_id);
#endif
		if (Esp32C3ClearCpuIrqHandler != 0)
		{
			Esp32C3ClearCpuIrqHandler(cpu_int_id);
		}
	}
	Esp32IrqFenceIo();
	EnableInterrupt(state);
}

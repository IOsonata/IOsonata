/**-------------------------------------------------------------------------
@file Vectors_esp32.c
@brief Shared C2 INTC and production C5/C61 CLIC trap dispatch.

C3/C6 retain their existing vector files. This file is linked with
esp32xx_irq.c on C2/C5/C61. The first IRQ installation initializes the
controller and trap entry with MIE masked; startup may initialize it earlier.
CLIC uses non-vectored slots, so mtvec points to the trap function (MODE=3),
not to a jump table. C2 uses a 256-byte aligned instruction vector table.

@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "esp32xx_irq.h"

#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)

static Esp32IrqHandler volatile s_CpuHandlers[32];
static Esp32IrqHandler volatile s_SourceHandlers[ESP32_INTMTX_NUM_SOURCES];
static volatile uint32_t s_ActiveCpu;
static bool s_VectorsReady;

void RISCV_TrapHandler(void);

#if defined(__riscv) && defined(ESP32C2)
__asm__(
	".pushsection .iram.text.vectors,\"ax\",@progbits\n"
	".option push\n"
	".option norvc\n"
	".balign 256\n"
	".global RISCV_VectorTable\n"
	".type RISCV_VectorTable, @function\n"
	"RISCV_VectorTable:\n"
	".rept 32\n"
	"jal x0, RISCV_TrapHandler\n"
	".endr\n"
	".size RISCV_VectorTable, . - RISCV_VectorTable\n"
	".option pop\n"
	".popsection\n"
);
extern void RISCV_VectorTable(void);
#endif

void Esp32InitVectors(void)
{
	uint32_t source;
#if defined(ESP32C5) || defined(ESP32C61)
	uint32_t slot;
#endif
#if defined(__riscv)
	uintptr_t state;
	uintptr_t vector;
#endif

	if (s_VectorsReady)
	{
		return;
	}
#if defined(__riscv)
	__asm volatile("csrrci %0, mstatus, 8" : "=r"(state) : : "memory");
#endif
	ESP32_INTMTX_CLOCK_GATE_REG = 1;
#if defined(ESP32C2)
	ESP32_CPU_INT_ENABLE_REG = 0;
	ESP32_CPU_INT_TYPE_REG = 0;
	ESP32_CPU_INT_CLEAR_REG = UINT32_MAX;
	ESP32_CPU_INT_CLEAR_REG = 0;
	ESP32_CPU_INT_THRESH_REG = 0;
#else
	for (slot = 0; slot < ESP32_CLIC_INT_COUNT; slot++)
	{
		ClicDisableSlot(ESP32_CLIC_CTRL_BASE, slot, ESP32_CLIC_INT_COUNT);
	}
	ESP32_CLIC_INT_CONFIG_REG = ESP32_CLIC_NLBITS;
#endif
	for (source = 0; source < ESP32_INTMTX_NUM_SOURCES; source++)
	{
		ESP32_INTMTX_SOURCE_MAP_REG(source) = ESP32_INTMTX_DISABLED_MAP;
	}
#if defined(__riscv)
#if defined(ESP32C2)
	vector = (uintptr_t)RISCV_VectorTable | 1U;
#else
	vector = (uintptr_t)RISCV_TrapHandler | 3U;
	// Production C5/C61 have standard mintthresh. Priority zero is masked.
	__asm volatile("csrw 0x347, %0" : : "r"(0x1FU) : "memory");
#endif
	__asm volatile("csrw mtvec, %0" : : "r"(vector) : "memory");
	__asm volatile("fence iorw, iorw" : : : "memory");
#endif
	s_VectorsReady = true;
#if defined(__riscv)
	__asm volatile("csrw mstatus, %0" : : "r"(state) : "memory");
#endif
}

void Esp32C3SetCpuIrqHandler(uint32_t Cpu, Esp32IrqHandler Handler)
{
	if (Cpu < 32)
	{
		s_CpuHandlers[Cpu] = Handler;
	}
}

void Esp32C3ClearCpuIrqHandler(uint32_t Cpu)
{
	Esp32C3SetCpuIrqHandler(Cpu, 0);
}

void Esp32C3SetSourceIrqHandler(uint32_t Source, Esp32IrqHandler Handler)
{
	if (Source < ESP32_INTMTX_NUM_SOURCES)
	{
		s_SourceHandlers[Source] = Handler;
	}
}

void Esp32C3ClearSourceIrqHandler(uint32_t Source)
{
	Esp32C3SetSourceIrqHandler(Source, 0);
}

void Esp32C3IntMtxDispatch(void)
{
	uint32_t status[3];
	uint32_t source;
	Esp32IrqHandler handler;

	status[0] = ESP32_INTMTX_INTR_STATUS_0_REG;
	status[1] = ESP32_INTMTX_INTR_STATUS_1_REG;
	status[2] = 0;
#ifdef ESP32_INTMTX_INTR_STATUS_2_REG
	status[2] = ESP32_INTMTX_INTR_STATUS_2_REG;
#endif
	for (source = 0; source < ESP32_INTMTX_NUM_SOURCES; source++)
	{
		if ((status[source / 32] & (1UL << (source % 32))) == 0 ||
			ESP32_INTMTX_SOURCE_MAP_REG(source) != s_ActiveCpu + ESP32_INTMTX_CPU_OFFSET)
		{
			continue;
		}
		handler = s_SourceHandlers[source];
		if (handler != 0)
		{
			handler();
		}
	}
}

bool Esp32DispatchCpuIrq(uint32_t Cause)
{
	uint32_t cpu;
	uint32_t previous;
	Esp32IrqHandler handler;

	if ((Cause & 0x80000000UL) == 0)
	{
		return false;
	}
	// CLIC mcause also contains previous privilege/priority in higher bits.
	cpu = Cause & 0xFFFU;
	if (cpu < ESP32_INTMTX_CPU_OFFSET || cpu >= ESP32_INTMTX_CPU_OFFSET + 32)
	{
		return false;
	}
	cpu -= ESP32_INTMTX_CPU_OFFSET;
	handler = s_CpuHandlers[cpu];
	if (handler == 0)
	{
		return false;
	}
	previous = s_ActiveCpu;
	s_ActiveCpu = cpu;
	handler();
	s_ActiveCpu = previous;
	return true;
}

#if defined(__riscv)
__attribute__((interrupt("machine"), aligned(64), section(".iram.text"), used))
void RISCV_TrapHandler(void)
{
	uint32_t cause;
	uint32_t epc;
	uint32_t status;

	__asm volatile("csrr %0, mcause" : "=r"(cause));
	__asm volatile("csrr %0, mepc" : "=r"(epc));
	__asm volatile("csrr %0, mstatus" : "=r"(status));
	if (!Esp32DispatchCpuIrq(cause))
	{
		for (;;)
		{
			__asm volatile("nop");
		}
	}
	// Preserve the interrupted context even if a callback enabled nesting.
	// CLIC mret restores the previous interrupt level from mcause.
	__asm volatile("csrci mstatus, 8" : : : "memory");
	__asm volatile("csrw mepc, %0" : : "r"(epc) : "memory");
	__asm volatile("csrw mcause, %0" : : "r"(cause) : "memory");
	__asm volatile("csrw mstatus, %0" : : "r"(status) : "memory");
}
#endif
#endif

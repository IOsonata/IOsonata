/**-------------------------------------------------------------------------
@file system_esp32.c
@brief C2/C5/C61 startup after the generic RISC-V RAM initialization.

Uses the shared SYSTEM/PCR clock helpers and interrupt matrix vectors.
C2 retains the ROM's 80/120 MHz PLL or crystal clock selection. C5/C61
also retain the ROM clock and enable the standard cycle counter for delays.

@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include "esp32xx_irq.h"

#if defined(ESP32C2)
extern void Esp32SystemInit(void);
#elif defined(ESP32C5) || defined(ESP32C61)
extern void Esp32PcrSystemInit(uint32_t CpuHz);
#else
#error "Use the chip-specific startup for this target"
#endif

void SystemInit(void)
{
	__asm volatile("csrci mstatus, 8" : : : "memory");
	Esp32InitVectors();
#if defined(ESP32C2)
	Esp32SystemInit();
#else
	Esp32PcrSystemInit(0);
	// C5/C61 implement mcycle/cycle. Clear only the cycle inhibit bit.
	__asm volatile("csrci 0x320, 1" : : : "memory");
#endif
}

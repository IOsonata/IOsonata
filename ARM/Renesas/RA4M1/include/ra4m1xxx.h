/**-------------------------------------------------------------------------
@file	ra4m1xxx.h

@brief	Renesas RA4M1 Cortex-M4 core and startup definitions.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#ifndef __RA4M1XXX_H__
#define __RA4M1XXX_H__

#include <stdint.h>

/* RA4M1 has 32 configurable ICU event links, not fixed peripheral vectors.
 * Event selection belongs to the interrupt/peripheral driver, as on RE01.
 * This initial header describes the core, not the complete peripheral set.
 */
typedef enum IRQn {
	NonMaskableInt_IRQn = -14,
	HardFault_IRQn      = -13,
	MemoryManagement_IRQn = -12,
	BusFault_IRQn       = -11,
	UsageFault_IRQn     = -10,
	SVCall_IRQn         = -5,
	DebugMonitor_IRQn   = -4,
	PendSV_IRQn         = -2,
	SysTick_IRQn        = -1,
	IEL0_IRQn = 0,
	IEL1_IRQn = 1,
	IEL2_IRQn = 2,
	IEL3_IRQn = 3,
	IEL4_IRQn = 4,
	IEL5_IRQn = 5,
	IEL6_IRQn = 6,
	IEL7_IRQn = 7,
	IEL8_IRQn = 8,
	IEL9_IRQn = 9,
	IEL10_IRQn = 10,
	IEL11_IRQn = 11,
	IEL12_IRQn = 12,
	IEL13_IRQn = 13,
	IEL14_IRQn = 14,
	IEL15_IRQn = 15,
	IEL16_IRQn = 16,
	IEL17_IRQn = 17,
	IEL18_IRQn = 18,
	IEL19_IRQn = 19,
	IEL20_IRQn = 20,
	IEL21_IRQn = 21,
	IEL22_IRQn = 22,
	IEL23_IRQn = 23,
	IEL24_IRQn = 24,
	IEL25_IRQn = 25,
	IEL26_IRQn = 26,
	IEL27_IRQn = 27,
	IEL28_IRQn = 28,
	IEL29_IRQn = 29,
	IEL30_IRQn = 30,
	IEL31_IRQn = 31,
} IRQn_Type;

#define __CM4_REV                 0x0001U
#define __MPU_PRESENT             1U
#define __FPU_PRESENT             1U
#define __NVIC_PRIO_BITS          4U
#define __Vendor_SysTickConfig    0U

#ifdef __GNUC__
#ifndef __PROGRAM_START
#define __PROGRAM_START          /* IOsonata owns ResetEntry, not CMSIS CRT. */
#endif
#endif
#include "core_cm4.h"

#define RA4M1_IRQ_COUNT           32U
#define RA4M1_VECTOR_COUNT        (16U + RA4M1_IRQ_COUNT)
#define RA4M1_FLASH_START         0x00000000UL
#define RA4M1_FLASH_SIZE          0x00040000UL
#define RA4M1_RAM_START           0x20000000UL
#define RA4M1_RAM_SIZE            0x00008000UL

#include "system_ra4m1.h"
#endif /* __RA4M1XXX_H__ */

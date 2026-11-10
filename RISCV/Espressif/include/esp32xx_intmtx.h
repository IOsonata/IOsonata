/**-------------------------------------------------------------------------
@file	esp32xx_intmtx.h

@brief	ESP32 Interrupt Matrix / CPU Interrupt Controller definitions
        for the IOsonata RISC-V family.

This header provides the cross-chip register interface for routing
peripheral interrupt sources to a CPU interrupt level and configuring
that CPU interrupt's enable, type, priority and threshold.  It also
defines the per-chip GPIO interrupt source IDs used by iopincfg_esp32.c.

C2/C3 combine interrupt matrix and INTC registers in one block. C6
separates matrix routing from INTPRI CPU control. C5/C61 also use the
matrix, with each destination selecting a CLIC slot at CPU input + 16.
Their CLIC control registers are defined in esp32xx_clic.h.

Register offsets follow Espressif ESP-IDF v5.5.1 per-chip interrupt_core0,
interrupt_matrix, intpri and clic register headers.

@author	Nguyen Hoan Hoang
@date	May 9, 2026

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
#ifndef __ESP32XX_INTMTX_H__
#define __ESP32XX_INTMTX_H__

#include <stdint.h>

#include "esp32xx.h"

/*---------------------------------------------------------------------------
 * Memory-mapped 32-bit register helper.  Defined here too so this header
 * is usable independently of esp32xx_gpio.h.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_REG32
#define ESP32_REG32(addr)                   (*(volatile uint32_t *)((uintptr_t)(addr)))
#endif

/*---------------------------------------------------------------------------
 * Global constants (architecture limits).
 *---------------------------------------------------------------------------*/
#define ESP32_CPU_INT_COUNT_MAX             32U  //!< CPU INT IDs 1..31 valid; 0 reserved for exceptions.
#define ESP32_CPU_INT_PRIO_MIN              1U
#define ESP32_CPU_INT_PRIO_MAX              15U

/*---------------------------------------------------------------------------
 * Per-chip GPIO interrupt source ID.
 *
 * Verified against IDF interrupts.h periph_interrupt_t enum:
 * C2: 13, C3: 16, C5: 31, C6: 30, C61: 27.
 * The matrix destination CPU input determines the CLIC slot on C5/C61.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_GPIO_INTR_SOURCE_ID
#  if defined(ESP32C2)
#    define ESP32_GPIO_INTR_SOURCE_ID       13U
#  elif defined(ESP32C61)
#    define ESP32_GPIO_INTR_SOURCE_ID       27U
#  elif defined(ESP32C3)
#    define ESP32_GPIO_INTR_SOURCE_ID       16U
#  elif defined(ESP32C5)
#    define ESP32_GPIO_INTR_SOURCE_ID       31U
#  elif defined(ESP32C6)
#    define ESP32_GPIO_INTR_SOURCE_ID       30U
#  endif
#endif

/*---------------------------------------------------------------------------
 * Default CPU interrupt level and priority for GPIO.
 *
 * CPU INT 4 is a sensible default that does not collide with the UART
 * driver's CPU INT 1.  Override per-board if needed.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_GPIO_INTR_CPU_INT_ID
#  if defined(ESP32C6)
#    define ESP32_GPIO_INTR_CPU_INT_ID      6U
#  else
#    define ESP32_GPIO_INTR_CPU_INT_ID      4U
#  endif
#endif

#ifndef ESP32_GPIO_INTR_CPU_INT_PRIO
#  define ESP32_GPIO_INTR_CPU_INT_PRIO      1U
#endif

/*===========================================================================
 * INTC register interface (C3 + C6 only).
 *
 * C5 is intentionally excluded; it uses CLIC.  Code that needs to
 * support C5 should #ifdef on ESP32_CPU_INT_ENABLE_REG and provide an
 * alternative path.
 *===========================================================================*/

#if defined(ESP32C2) || defined(ESP32C3) || defined(ESP32C6) || defined(ESP32C61)

/* Source-map register: route source N to one of the 32 CPU INT lines.
 * Common to C3 and C6: stored at ESP32_INTMTX_BASE + N*4. */
#define ESP32_INTMTX_SOURCE_MAP_REG(src)                                            \
    ESP32_REG32(ESP32_INTMTX_BASE + ((uint32_t)(src) * 4U))

#endif // C3 || C6


/*---------------------------------------------------------------------------
 * C3: routing + CPU-INT control are in a single block at INTMTX_BASE.
 *---------------------------------------------------------------------------*/
#if defined(ESP32C3)

// INTMTX block: source-map regs precede these.
#define ESP32_INTMTX_INTR_STATUS_0_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x0F8U)
#define ESP32_INTMTX_INTR_STATUS_1_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x0FCU)
#define ESP32_INTMTX_CLOCK_GATE_REG         ESP32_REG32(ESP32_INTMTX_BASE + 0x100U)

// CPU-INT control (same block on C3).
#define ESP32_CPU_INT_ENABLE_REG            ESP32_REG32(ESP32_INTMTX_BASE + 0x104U)
#define ESP32_CPU_INT_TYPE_REG              ESP32_REG32(ESP32_INTMTX_BASE + 0x108U)
#define ESP32_CPU_INT_CLEAR_REG             ESP32_REG32(ESP32_INTMTX_BASE + 0x10CU)
#define ESP32_CPU_INT_EIP_STATUS_REG        ESP32_REG32(ESP32_INTMTX_BASE + 0x110U)
#define ESP32_CPU_INT_PRI_REG(n)            ESP32_REG32(ESP32_INTMTX_BASE + 0x114U + ((uint32_t)(n) * 4U))
#define ESP32_CPU_INT_THRESH_REG            ESP32_REG32(ESP32_INTMTX_BASE + 0x194U)

#endif // ESP32C3


/*---------------------------------------------------------------------------
 * C6: routing/status at INTMTX_BASE, CPU interrupt control at PLIC_MX_BASE.
 *---------------------------------------------------------------------------*/
#if defined(ESP32C6)

// INTMTX block: routing and matrix-side status.
#define ESP32_INTMTX_INTR_STATUS_0_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x134U)
#define ESP32_INTMTX_INTR_STATUS_1_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x138U)
#define ESP32_INTMTX_INTR_STATUS_2_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x13CU)
#define ESP32_INTMTX_CLOCK_GATE_REG         ESP32_REG32(ESP32_INTMTX_BASE + 0x140U)

// C6 uses the machine-mode PLIC, not the peripheral INTPRI block.
// ESP-IDF v5.4.2: soc/esp32c6/include/soc/interrupt_reg.h and plic_reg.h.
#define ESP32_CPU_INT_ENABLE_REG            ESP32_REG32(ESP32_PLIC_MX_BASE + 0x000U)
#define ESP32_CPU_INT_TYPE_REG              ESP32_REG32(ESP32_PLIC_MX_BASE + 0x004U)
#define ESP32_CPU_INT_CLEAR_REG             ESP32_REG32(ESP32_PLIC_MX_BASE + 0x008U)
#define ESP32_CPU_INT_EIP_STATUS_REG        ESP32_REG32(ESP32_PLIC_MX_BASE + 0x00CU)
#define ESP32_CPU_INT_PRI_REG(n)            ESP32_REG32(ESP32_PLIC_MX_BASE + 0x010U + ((uint32_t)(n) * 4U))
#define ESP32_CPU_INT_THRESH_REG            ESP32_REG32(ESP32_PLIC_MX_BASE + 0x090U)

#endif // ESP32C6


/*---------------------------------------------------------------------------
 * C5: CLIC architecture.  This header provides only the source-map and
 * matrix status registers for C5; CPU-INT control is via CLIC and lives
 * in a separate esp32xx_clic.h (TBD).
 *---------------------------------------------------------------------------*/
#if defined(ESP32C5)

#define ESP32_INTMTX_SOURCE_MAP_REG(src)                                            \
    ESP32_REG32(ESP32_INTMTX_BASE + ((uint32_t)(src) * 4U))

#define ESP32_INTMTX_INTR_STATUS_0_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x150U)
#define ESP32_INTMTX_INTR_STATUS_1_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x154U)
#define ESP32_INTMTX_INTR_STATUS_2_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x158U)
#define ESP32_INTMTX_CLOCK_GATE_REG         ESP32_REG32(ESP32_INTMTX_BASE + 0x170U)

/* ESP32_CPU_INT_*_REG intentionally undefined on C5.
 * Use CLIC (esp32xx_clic.h) for per-interrupt enable/priority/level. */

#endif // ESP32C5

// Source counts stop before the status/control registers in the matrix.
#if defined(ESP32C2)
#define ESP32_INTMTX_NUM_SOURCES 43U
#define ESP32_INTMTX_INTR_STATUS_0_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x0ACU)
#define ESP32_INTMTX_INTR_STATUS_1_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x0B0U)
#define ESP32_INTMTX_CLOCK_GATE_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x0B4U)
#define ESP32_CPU_INT_ENABLE_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x0B8U)
#define ESP32_CPU_INT_TYPE_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x0BCU)
#define ESP32_CPU_INT_CLEAR_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x0C0U)
#define ESP32_CPU_INT_EIP_STATUS_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x0C4U)
#define ESP32_CPU_INT_PRI_REG(n) ESP32_REG32(ESP32_INTMTX_BASE + 0x0C8U + (uint32_t)(n) * 4U)
#define ESP32_CPU_INT_THRESH_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x148U)
#elif defined(ESP32C3)
#define ESP32_INTMTX_NUM_SOURCES 62U
#elif defined(ESP32C5)
#define ESP32_INTMTX_NUM_SOURCES 84U
#elif defined(ESP32C6)
#define ESP32_INTMTX_NUM_SOURCES 77U
#elif defined(ESP32C61)
#define ESP32_INTMTX_NUM_SOURCES 66U
#define ESP32_INTMTX_INTR_STATUS_0_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x108U)
#define ESP32_INTMTX_INTR_STATUS_1_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x10CU)
#define ESP32_INTMTX_INTR_STATUS_2_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x110U)
#define ESP32_INTMTX_CLOCK_GATE_REG ESP32_REG32(ESP32_INTMTX_BASE + 0x128U)
#endif

// CLIC matrix values include the 16 CPU-local interrupt slots.
#if defined(ESP32C5) || defined(ESP32C61)
#define ESP32_INTMTX_CPU_OFFSET 16U
#define ESP32_INTMTX_DISABLED_CPU 6U
#elif defined(ESP32C6)
#define ESP32_INTMTX_CPU_OFFSET 0U
#define ESP32_INTMTX_DISABLED_CPU 0U
#else
#define ESP32_INTMTX_CPU_OFFSET 0U
#define ESP32_INTMTX_DISABLED_CPU 6U
#endif
#define ESP32_INTMTX_DISABLED_MAP (ESP32_INTMTX_DISABLED_CPU + ESP32_INTMTX_CPU_OFFSET)

#endif // __ESP32XX_INTMTX_H__

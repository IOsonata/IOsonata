/**-------------------------------------------------------------------------
@file	esp32xx_intmtx.h

@brief	ESP32 Interrupt Matrix / CPU Interrupt Controller definitions
        for the IOsonata RISC-V family.

This header provides the cross-chip register interface for routing
peripheral interrupt sources to a CPU interrupt level and configuring
that CPU interrupt's enable, type, priority and threshold.  It also
defines the per-chip GPIO interrupt source IDs used by iopincfg_esp32.c.

Architecture note:

  ESP32-C3 and ESP32-C6 use Espressif's "INTC" interrupt controller:
      - Per-source MAP register routes one of 62..64 sources to one of
        32 CPU interrupt levels.
      - Global per-CPU-INT registers (ENABLE / TYPE / PRI / THRESH)
        control level, edge sensitivity, priority (1..15), and the
        global priority threshold.
      - C3 has both the routing block AND the CPU-INT control block at
        a single INTMTX_BASE.  C6 split them: routing+status remain at
        INTMTX_BASE, CPU-INT control moved to INTPRI_BASE.

  ESP32-C5 uses RISC-V CLIC (Core Local Interrupt Controller):
      - Per-interrupt CLIC_INT_n_IE / IP / CTL registers at
        0x20801000.  No global ENABLE/TYPE/PRI/THRESH.
      - Different programming model entirely; the C3/C6 macros below
        are intentionally NOT defined on C5.  CLIC support belongs in
        a separate esp32xx_clic.h.

Driver code that wants to compile across C3/C5/C6 should guard its
INTC setup on the existence of ESP32_CPU_INT_ENABLE_REG (for example)
and provide a CLIC alternative when that macro is undefined.

Register offsets verified against ESP-IDF v5.3
components/soc/esp32c3/include/soc/interrupt_core0_reg.h and
components/soc/esp32c6/include/soc/{interrupt_matrix_reg.h, intpri_reg.h}.

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
 * Cross-chip base-address fallbacks.
 *
 * Each chip header defines its own ESP32CN_INTMTX_BASE; here we provide
 * the no-prefix common alias if the chip header has not already done it.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_INTMTX_BASE
#  if   defined(ESP32C3)
#    define ESP32_INTMTX_BASE               ESP32C3_INTMTX_BASE
#  elif defined(ESP32C5)
#    define ESP32_INTMTX_BASE               ESP32C5_INTMTX_BASE
#  elif defined(ESP32C6)
#    define ESP32_INTMTX_BASE               ESP32C6_INTMTX_BASE
#  endif
#endif

#ifndef ESP32_INTPRI_BASE
#  if   defined(ESP32C5)
#    define ESP32_INTPRI_BASE               ESP32C5_INTPRI_BASE
#  elif defined(ESP32C6)
#    define ESP32_INTPRI_BASE               ESP32C6_INTPRI_BASE
#  endif
#endif

/* On C3 the CPU-INT control registers share the INTMTX block.  Provide
 * an alias so the same ESP32_CPU_INT_*_REG macros can use a single base
 * variable name. */
#if defined(ESP32C3) && !defined(ESP32_INTPRI_BASE)
#  define ESP32_INTPRI_BASE                 ESP32_INTMTX_BASE
#endif

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
 *   C3:  ETS_GPIO_INTR_SOURCE = 16
 *   C6:  ETS_GPIO_INTR_SOURCE = 30
 *   C5:  ETS_GPIO_INTR_SOURCE = 31  (used by the CLIC install path
 *                                    in esp32xx_irq.c, which adds the
 *                                    CLIC_EXT_INTR_NUM_OFFSET to obtain
 *                                    the actual CLIC slot).
 *---------------------------------------------------------------------------*/
#ifndef ESP32_GPIO_INTR_SOURCE_ID
#  if   defined(ESP32C3)
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
#  define ESP32_GPIO_INTR_CPU_INT_ID        4U
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

#if defined(ESP32C3) || defined(ESP32C6)

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
 * C6: routing+status at INTMTX_BASE, CPU-INT control at INTPRI_BASE.
 *---------------------------------------------------------------------------*/
#if defined(ESP32C6)

// INTMTX block: routing and matrix-side status.
#define ESP32_INTMTX_INTR_STATUS_0_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x134U)
#define ESP32_INTMTX_INTR_STATUS_1_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x138U)
#define ESP32_INTMTX_INTR_STATUS_2_REG      ESP32_REG32(ESP32_INTMTX_BASE + 0x13CU)
#define ESP32_INTMTX_CLOCK_GATE_REG         ESP32_REG32(ESP32_INTMTX_BASE + 0x140U)

// CPU-INT control: separate INTPRI block on C6.
#define ESP32_CPU_INT_ENABLE_REG            ESP32_REG32(ESP32_INTPRI_BASE + 0x000U)
#define ESP32_CPU_INT_TYPE_REG              ESP32_REG32(ESP32_INTPRI_BASE + 0x004U)
#define ESP32_CPU_INT_EIP_STATUS_REG        ESP32_REG32(ESP32_INTPRI_BASE + 0x008U)
#define ESP32_CPU_INT_PRI_REG(n)            ESP32_REG32(ESP32_INTPRI_BASE + 0x00CU + ((uint32_t)(n) * 4U))
#define ESP32_CPU_INT_THRESH_REG            ESP32_REG32(ESP32_INTPRI_BASE + 0x08CU)
#define ESP32_CPU_INT_CLEAR_REG             ESP32_REG32(ESP32_INTPRI_BASE + 0x0A8U)

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

#endif // __ESP32XX_INTMTX_H__

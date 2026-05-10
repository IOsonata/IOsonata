/**-------------------------------------------------------------------------
@file	esp32xx_clic.h

@brief	ESP32 RISC-V CLIC chip-specific configuration.

The CLIC register layout, ATTR/CTL field positions, trigger/mode
values, and the install / disable mechanics are RISC-V standard and
live in the generic <clic.h> at the RISCV/ layer.  This header only
adds the values that vary per chip: control block addresses, total
slot count, NLBITS, and the offset where peripheral interrupts start.

Targets covered by this file:
    ESP32-C5    NLBITS = 3, INT_COUNT = 64, ext-int offset = 16

Future Espressif CLIC chips (ESP32-H2, ESP32-H4, ESP32-P4) plug in
the same way: a new #if block here defining their addresses and
limits.  The CLIC enable/disable code in esp32xx_irq.c does not
change per chip — it just calls ClicEnableSlot() with whichever
ESP32_CLIC_* macros this header exposes.

C3 and C6 use the older INTMTX + global CPU-INT controller;
those definitions are in esp32xx_intmtx.h, not here.

Register addresses verified against ESP-IDF master,
components/soc/esp32c5/include/soc/clic_reg.h.

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

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __ESP32XX_CLIC_H__
#define __ESP32XX_CLIC_H__

#include <stdint.h>

#include "clic.h"           // generic RISC-V CLIC layer
#include "esp32xx.h"

#ifndef ESP32_REG32
#define ESP32_REG32(addr)                       (*(volatile uint32_t *)((uintptr_t)(addr)))
#endif

#if defined(ESP32C5)

/*---------------------------------------------------------------------------
 * ESP32-C5 CLIC base addresses.
 *
 * The CLIC has two register windows: a small global config block at
 * CLIC_BASE (used at boot to set NLBITS and during exception entry to
 * read the active level) and the per-interrupt control array at
 * CLIC_CTRL_BASE.  Both are documented in the ESP32-C5 TRM rev. 0.x,
 * "Core Local Interrupt Controller".
 *---------------------------------------------------------------------------*/
#define ESP32_CLIC_BASE                         0x20800000UL    //!< CLIC global config block.
#define ESP32_CLIC_CTRL_BASE                    0x20801000UL    //!< Per-interrupt 4-byte control blocks.

/*---------------------------------------------------------------------------
 * Chip-specific CLIC limits and configuration.
 *---------------------------------------------------------------------------*/
#define ESP32_CLIC_INT_COUNT                    64U             //!< Total CLIC slots (0..63).
#define ESP32_CLIC_NLBITS                       3U              //!< Level bits in CTL byte (3 level + 5 sub-prio, IDF default).
#define ESP32_CLIC_EXT_INTR_NUM_OFFSET          16U             //!< First peripheral slot (slots 0..15 reserved for standard RISC-V ints).

/*---------------------------------------------------------------------------
 * Global CLIC config register.  The layout (UNLBITS / SNLBITS / MNLBITS
 * fields at their nibble positions) is RISC-V standard, but the address
 * is chip-specific so it stays here.
 *---------------------------------------------------------------------------*/
#define ESP32_CLIC_INT_CONFIG_REG               ESP32_REG32(ESP32_CLIC_BASE + 0x000U)
#define ESP32_CLIC_INT_CONFIG_UNLBITS_Pos       24U
#define ESP32_CLIC_INT_CONFIG_UNLBITS_Msk       (0xFUL << ESP32_CLIC_INT_CONFIG_UNLBITS_Pos)
#define ESP32_CLIC_INT_CONFIG_SNLBITS_Pos       16U
#define ESP32_CLIC_INT_CONFIG_SNLBITS_Msk       (0xFUL << ESP32_CLIC_INT_CONFIG_SNLBITS_Pos)
#define ESP32_CLIC_INT_CONFIG_MNLBITS_Pos       0U
#define ESP32_CLIC_INT_CONFIG_MNLBITS_Msk       (0xFUL << ESP32_CLIC_INT_CONFIG_MNLBITS_Pos)

#endif // ESP32C5

#endif // __ESP32XX_CLIC_H__

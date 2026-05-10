/**-------------------------------------------------------------------------
@file	esp32xx_clic.h

@brief	ESP32 RISC-V CLIC (Core Local Interrupt Controller) register
        definitions for the IOsonata RISC-V family.

Covers the CLIC controller used on ESP32-C5 (and other CLIC-equipped
variants such as H2 / H4 / P4 when added).  C3 and C6 use the older
INTMTX + global CPU-INT controller; their definitions live in
esp32xx_intmtx.h.

CLIC architecture summary:

  * Per-interrupt control instead of global ENABLE/TYPE/PRI/THRESH.
    Each CLIC interrupt slot has its own 4-byte block at
        DR_REG_CLIC_CTRL_BASE + slot * 4
    with byte fields IP / IE / ATTR / CTL.
  * Slots 0..15 reserved for RISC-V standard interrupts (m-software,
    m-timer, m-external, etc.).  Peripheral sources start at slot 16
    (CLIC_EXT_INTR_NUM_OFFSET).
  * Vectoring uses the mtvt CSR (table base) and mtvec mode bits 11
    (CLIC vectored mode), distinct from C3/C6's vectored CLINT mode.

Mapping a peripheral source to a CLIC slot:
    slot = source_id + ESP32_CLIC_EXT_INTR_NUM_OFFSET
where source_id is the same enum value as on C3/C6 (e.g. GPIO = 31 on
C5, so CLIC slot = 47).

Register offsets verified against ESP-IDF master
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
#ifndef __ESP32XX_CLIC_H__
#define __ESP32XX_CLIC_H__

#include <stdint.h>

#include "esp32xx.h"

#if defined(ESP32C5)

#ifndef ESP32_REG32
#define ESP32_REG32(addr)                   (*(volatile uint32_t *)((uintptr_t)(addr)))
#endif
#ifndef ESP32_REG8
#define ESP32_REG8(addr)                    (*(volatile uint8_t  *)((uintptr_t)(addr)))
#endif

/*---------------------------------------------------------------------------
 * CLIC base addresses.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_CLIC_BASE
#  define ESP32_CLIC_BASE                   0x20800000UL    //!< CLIC config block.
#endif
#ifndef ESP32_CLIC_CTRL_BASE
#  define ESP32_CLIC_CTRL_BASE              0x20801000UL    //!< Per-interrupt control block.
#endif

/*---------------------------------------------------------------------------
 * Architecture limits.
 *---------------------------------------------------------------------------*/
#define ESP32_CLIC_INT_COUNT                64U             //!< Total CLIC slots (0..63).
#define ESP32_CLIC_EXT_INTR_NUM_OFFSET      16U             //!< First peripheral slot (slots 0..15 reserved).
#define ESP32_CLIC_NLBITS                   3U              //!< NLBITS configured by IDF: 3 level bits, 5 priority bits per CTL.

/*---------------------------------------------------------------------------
 * Global CLIC config register (mtvt configuration / NLBITS).
 *---------------------------------------------------------------------------*/
#define ESP32_CLIC_INT_CONFIG_REG           ESP32_REG32(ESP32_CLIC_BASE + 0x000U)
#define ESP32_CLIC_INT_CONFIG_UNLBITS_Pos   24U
#define ESP32_CLIC_INT_CONFIG_UNLBITS_Msk   (0xFUL << ESP32_CLIC_INT_CONFIG_UNLBITS_Pos)
#define ESP32_CLIC_INT_CONFIG_SNLBITS_Pos   16U
#define ESP32_CLIC_INT_CONFIG_SNLBITS_Msk   (0xFUL << ESP32_CLIC_INT_CONFIG_SNLBITS_Pos)
#define ESP32_CLIC_INT_CONFIG_MNLBITS_Pos   0U
#define ESP32_CLIC_INT_CONFIG_MNLBITS_Msk   (0xFUL << ESP32_CLIC_INT_CONFIG_MNLBITS_Pos)

/*---------------------------------------------------------------------------
 * Per-interrupt 4-byte block at ESP32_CLIC_CTRL_BASE + slot * 4.
 *
 * Byte 0 : IP   - interrupt pending  (RW; write 1 to clear)
 * Byte 1 : IE   - interrupt enable   (RW)
 * Byte 2 : ATTR - mode / trigger / SHV
 * Byte 3 : CTL  - priority byte
 *---------------------------------------------------------------------------*/
#define ESP32_CLIC_INT_CTRL_REG(slot)       ESP32_REG32(ESP32_CLIC_CTRL_BASE + (uint32_t)(slot) * 4U)
#define ESP32_CLIC_INT_IP_REG(slot)         ESP32_REG8 (ESP32_CLIC_CTRL_BASE + (uint32_t)(slot) * 4U + 0U)
#define ESP32_CLIC_INT_IE_REG(slot)         ESP32_REG8 (ESP32_CLIC_CTRL_BASE + (uint32_t)(slot) * 4U + 1U)
#define ESP32_CLIC_INT_ATTR_REG(slot)       ESP32_REG8 (ESP32_CLIC_CTRL_BASE + (uint32_t)(slot) * 4U + 2U)
#define ESP32_CLIC_INT_CTL_REG(slot)        ESP32_REG8 (ESP32_CLIC_CTRL_BASE + (uint32_t)(slot) * 4U + 3U)

/*---------------------------------------------------------------------------
 * ATTR byte fields (per-interrupt attributes).
 *---------------------------------------------------------------------------*/
#define ESP32_CLIC_INT_ATTR_SHV_Pos         0U              //!< Selective hardware vectoring (1 = vectored).
#define ESP32_CLIC_INT_ATTR_SHV_Msk         (1U << ESP32_CLIC_INT_ATTR_SHV_Pos)
#define ESP32_CLIC_INT_ATTR_TRIG_Pos        1U              //!< Trigger type bits.
#define ESP32_CLIC_INT_ATTR_TRIG_Msk        (3U << ESP32_CLIC_INT_ATTR_TRIG_Pos)
#define ESP32_CLIC_INT_ATTR_TRIG_LEVEL      (0U << ESP32_CLIC_INT_ATTR_TRIG_Pos)
#define ESP32_CLIC_INT_ATTR_TRIG_EDGE_POS   (1U << ESP32_CLIC_INT_ATTR_TRIG_Pos)
#define ESP32_CLIC_INT_ATTR_TRIG_EDGE_NEG   (3U << ESP32_CLIC_INT_ATTR_TRIG_Pos)
#define ESP32_CLIC_INT_ATTR_MODE_Pos        6U              //!< Privilege mode bits.
#define ESP32_CLIC_INT_ATTR_MODE_Msk        (3U << ESP32_CLIC_INT_ATTR_MODE_Pos)
#define ESP32_CLIC_INT_ATTR_MODE_USER       (0U << ESP32_CLIC_INT_ATTR_MODE_Pos)
#define ESP32_CLIC_INT_ATTR_MODE_SUPERVISOR (1U << ESP32_CLIC_INT_ATTR_MODE_Pos)
#define ESP32_CLIC_INT_ATTR_MODE_MACHINE    (3U << ESP32_CLIC_INT_ATTR_MODE_Pos)

/*---------------------------------------------------------------------------
 * CTL byte: priority value.
 *
 * Layout depends on NLBITS (set in CLIC_INT_CONFIG_REG).  With
 * NLBITS = 3 (IDF default): top 3 bits are the level, bottom 5 are
 * the sub-priority within that level.  Higher values are higher priority.
 *
 * For a basic "level N" interrupt, write (N << 5).
 *---------------------------------------------------------------------------*/
#define ESP32_CLIC_INT_CTL_LEVEL_Pos        5U
#define ESP32_CLIC_INT_CTL_LEVEL_Msk        (0xE0U)
#define ESP32_CLIC_INT_CTL_FROM_LEVEL(n)    ((uint8_t)(((n) & 0x7U) << ESP32_CLIC_INT_CTL_LEVEL_Pos))

#endif // ESP32C5

#endif // __ESP32XX_CLIC_H__

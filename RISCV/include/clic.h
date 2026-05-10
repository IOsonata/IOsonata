/**-------------------------------------------------------------------------
@file	clic.h

@brief	Generic RISC-V Core-Local Interrupt Controller (CLIC) layer.

This is the chip-agnostic CLIC interface for the IOsonata RISC-V family.
The CLIC register layout, ATTR/CTL field positions, trigger / mode
values and the standard enable sequence are defined by the RISC-V
"Fast Interrupts" specification and are identical on every compliant
chip.  Only the base address, total slot count, NLBITS and the offset
where peripheral interrupts start are chip-specific; those live in
the chip-level header (e.g. esp32xx_clic.h, gd32vf_clic.h, ...).

CLIC architecture summary:

  - Each CLIC interrupt has a 4-byte control block at
        CLIC_CTRL_BASE + slot * 4
    with byte fields IP / IE / ATTR / CTL.
  - Slots 0..15 are reserved for the standard RISC-V interrupts
    (m-software, m-timer, m-external, ...); peripheral sources start
    at the chip's external interrupt offset (typically 16).
  - Vectoring is selected by bit 11 of mtvec (CLIC vectored mode),
    distinct from the CLINT vectored mode (mtvec bit 0) used by
    non-CLIC RISC-V chips.  The CLIC vector table base lives in mtvt.

Per-chip bring-up files set up mtvec / mtvt / NLBITS once at boot.
This file does not touch them — it only provides the per-slot install
and disable mechanics, which are identical across chips.

Usage:
    #include "esp32xx_clic.h"   // chip header brings in this one and
                                // defines CLIC_BASE / CLIC_CTRL_BASE /
                                // CLIC_INT_COUNT / CLIC_NLBITS

    ClicEnableSlot(ESP32_CLIC_CTRL_BASE, slot,
                   CLIC_TRIG_LEVEL_HIGH, level, ESP32_CLIC_NLBITS);

@author	Nguyen Hoan Hoang
@date	May 10, 2026

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
#ifndef __CLIC_H__
#define __CLIC_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * Per-slot 4-byte control block accessors.
 *
 * `base` is the chip's CLIC control block base address (e.g.
 * ESP32_CLIC_CTRL_BASE on ESP32-C5).  Each slot occupies 4 consecutive
 * bytes at base + slot * 4:
 *     +0  IP    interrupt pending     (RW; write 1 to clear edge sources)
 *     +1  IE    interrupt enable      (RW)
 *     +2  ATTR  mode / trigger / SHV  (RW)
 *     +3  CTL   priority / level      (RW)
 *---------------------------------------------------------------------------*/
#define CLIC_INT_IP_REG(base, slot)         (*(volatile uint8_t *)((uintptr_t)(base) + (uint32_t)(slot) * 4U + 0U))
#define CLIC_INT_IE_REG(base, slot)         (*(volatile uint8_t *)((uintptr_t)(base) + (uint32_t)(slot) * 4U + 1U))
#define CLIC_INT_ATTR_REG(base, slot)       (*(volatile uint8_t *)((uintptr_t)(base) + (uint32_t)(slot) * 4U + 2U))
#define CLIC_INT_CTL_REG(base, slot)        (*(volatile uint8_t *)((uintptr_t)(base) + (uint32_t)(slot) * 4U + 3U))

/*---------------------------------------------------------------------------
 * ATTR byte field positions and masks.
 *---------------------------------------------------------------------------*/
#define CLIC_INT_ATTR_SHV_Pos               0U                          //!< Selective hardware vectoring (1 = vectored).
#define CLIC_INT_ATTR_SHV_Msk               (1U << CLIC_INT_ATTR_SHV_Pos)
#define CLIC_INT_ATTR_TRIG_Pos              1U                          //!< Trigger type bits.
#define CLIC_INT_ATTR_TRIG_Msk              (3U << CLIC_INT_ATTR_TRIG_Pos)
#define CLIC_INT_ATTR_MODE_Pos              6U                          //!< Privilege mode bits.
#define CLIC_INT_ATTR_MODE_Msk              (3U << CLIC_INT_ATTR_MODE_Pos)

/*---------------------------------------------------------------------------
 * Trigger types (RISC-V CLIC spec, ATTR[2:1]).
 *---------------------------------------------------------------------------*/
typedef enum {
    CLIC_TRIG_LEVEL_HIGH = 0,           //!< Level-sensitive, active high (default).
    CLIC_TRIG_EDGE_POS   = 1,           //!< Edge-sensitive, rising.
    CLIC_TRIG_LEVEL_LOW  = 2,           //!< Level-sensitive, active low.
    CLIC_TRIG_EDGE_NEG   = 3,           //!< Edge-sensitive, falling.
} CLIC_TRIGGER;

/*---------------------------------------------------------------------------
 * Privilege mode values (RISC-V CLIC spec, ATTR[7:6]).
 *---------------------------------------------------------------------------*/
typedef enum {
    CLIC_MODE_USER       = 0,
    CLIC_MODE_SUPERVISOR = 1,
    CLIC_MODE_MACHINE    = 3,
} CLIC_MODE;

/*---------------------------------------------------------------------------
 * Build a CTL byte from a level value.
 *
 * NLBITS is set per chip in CLIC_INT_CONFIG.  The CTL byte stores the
 * level in the top NLBITS bits and a sub-priority in the bottom
 * (8 - NLBITS) bits; sub-priority is filled with 1s here so a level-N
 * interrupt has the highest sub-priority within its level.  Higher
 * numeric values mean higher priority.
 *
 *     nlbits = 3 (ESP32-C5):  level << 5,  sub-priority = 0x1F
 *     nlbits = 4:             level << 4,  sub-priority = 0x0F
 *     nlbits = 8:             level,       sub-priority = 0
 *---------------------------------------------------------------------------*/
static inline uint8_t ClicCtlFromLevel(uint8_t level, uint8_t nlbits)
{
    if (nlbits == 0U)        // degenerate config; treat as no level bits
    {
        return 0xFFU;
    }
    if (nlbits >= 8U)
    {
        return level;
    }
    uint32_t lvl_mask = (1U << nlbits) - 1U;
    uint32_t sub_mask = (1U << (8U - nlbits)) - 1U;
    return (uint8_t)((((uint32_t)level & lvl_mask) << (8U - nlbits)) | sub_mask);
}

/*---------------------------------------------------------------------------
 * Build an ATTR byte from mode + trigger.  SHV is left at 0 (non-vectored
 * dispatch via DEF_IRQHandler); pass an explicit attr to ClicEnableSlotRaw
 * if you need to override that.
 *---------------------------------------------------------------------------*/
static inline uint8_t ClicAttrBuild(CLIC_MODE mode, CLIC_TRIGGER trig)
{
    return (uint8_t)((((uint32_t)mode & 3U) << CLIC_INT_ATTR_MODE_Pos)
                   | (((uint32_t)trig & 3U) << CLIC_INT_ATTR_TRIG_Pos));
}

/*---------------------------------------------------------------------------
 * Low-level: write attr + ctl directly, clear pending, enable.
 *
 * Caller is responsible for everything else — slot range check, MIE
 * masking, memory fences, and ROM-hook seeding if the chip needs one.
 * Useful when the caller has already computed ATTR (e.g. wants SHV = 1)
 * and CTL.
 *---------------------------------------------------------------------------*/
static inline void ClicEnableSlotRaw(uintptr_t base, uint32_t slot,
                                     uint8_t attr, uint8_t ctl)
{
    CLIC_INT_ATTR_REG(base, slot) = attr;
    CLIC_INT_CTL_REG(base, slot)  = ctl;
    CLIC_INT_IP_REG(base, slot)   = 0U;     // clear stale pending
    CLIC_INT_IE_REG(base, slot)   = 1U;
}

/*---------------------------------------------------------------------------
 * Convenience wrapper: enable a slot in machine mode with the given
 * trigger and level.  SHV = 0 (non-vectored).  Returns false if the
 * slot is out of range; otherwise returns true and the slot is live.
 *
 * Caller still owns MIE masking around this call.
 *---------------------------------------------------------------------------*/
static inline bool ClicEnableSlot(uintptr_t base, uint32_t slot,
                                  uint32_t int_count,
                                  CLIC_TRIGGER trig,
                                  uint8_t level, uint8_t nlbits)
{
    if (slot >= int_count)
    {
        return false;
    }
    ClicEnableSlotRaw(base, slot,
                      ClicAttrBuild(CLIC_MODE_MACHINE, trig),
                      ClicCtlFromLevel(level, nlbits));
    return true;
}

/*---------------------------------------------------------------------------
 * Disable a slot (mask its IE bit).  Pending state is left untouched.
 *---------------------------------------------------------------------------*/
static inline bool ClicDisableSlot(uintptr_t base, uint32_t slot,
                                   uint32_t int_count)
{
    if (slot >= int_count)
    {
        return false;
    }
    CLIC_INT_IE_REG(base, slot) = 0U;
    return true;
}

/*---------------------------------------------------------------------------
 * Software-trigger a slot (mainly useful for testing the dispatch path).
 * Edge-triggered slots latch IP = 1; level-triggered slots take it as
 * a forced active state until cleared.
 *---------------------------------------------------------------------------*/
static inline void ClicSetPending(uintptr_t base, uint32_t slot)
{
    CLIC_INT_IP_REG(base, slot) = 1U;
}

static inline void ClicClearPending(uintptr_t base, uint32_t slot)
{
    CLIC_INT_IP_REG(base, slot) = 0U;
}

#ifdef __cplusplus
}
#endif

#endif // __CLIC_H__

/**-------------------------------------------------------------------------
@file	esp32xx_irq.c

@brief	Cross-chip peripheral interrupt install for the IOsonata ESP32
        RISC-V family.

INTC implementation for C3 / C6 (single function, since esp32xx_intmtx.h
already abstracts the per-block layout differences).  CLIC implementation
for C5.

See esp32xx_irq.h for the API.

@author	Nguyen Hoan Hoang
@date	May 9, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "esp32xx.h"
#include "esp32xx_intmtx.h"
#include "esp32xx_irq.h"

#if defined(ESP32C5)
#include "esp32xx_clic.h"
#endif


// ===========================================================================
// Atomic mstatus.MIE save/restore + memory fence helpers (RISC-V).
// ===========================================================================
static inline uint32_t Esp32IrqSaveAndDisableMie(void)
{
#if defined(__riscv)
    uint32_t mstatus;
    __asm volatile("csrrc %0, mstatus, %1" : "=r"(mstatus) : "r"(0x8UL) : "memory");
    return mstatus;
#else
    return 0U;
#endif
}

static inline void Esp32IrqRestoreMie(uint32_t saved_mstatus)
{
#if defined(__riscv)
    if ((saved_mstatus & 0x8UL) != 0U)
    {
        __asm volatile("csrsi mstatus, 0x8" ::: "memory");
    }
#else
    (void)saved_mstatus;
#endif
}

static inline void Esp32IrqFenceIo(void)
{
#if defined(__riscv)
    // `fence iorw, iorw` is base RISC-V; required by the C3 TRM between
    // INTC writes and the MIE restore, and harmless on other RISC-V cores.
    __asm volatile("fence iorw, iorw" ::: "memory");
#else
    __asm volatile("" ::: "memory");
#endif
}


// ===========================================================================
// C3 + C6: INTC-based install.
//
// Steps:
//   1. Save MIE, clear MIE
//   2. Enable INTMTX clock gate (no-op if already enabled)
//   3. Optionally install ROM-table hooks (C3 only; weak symbols on C6)
//   4. Map source -> CPU INT
//   5. Configure CPU INT as level type, set priority, threshold = 0
//   6. Enable CPU INT
//   7. Fence
//   8. Restore MIE
// ===========================================================================
#if defined(ESP32C3) || defined(ESP32C6)

bool Esp32EnableSourceIrq(uint32_t source_id,
                          uint32_t cpu_int_id,
                          uint32_t prio,
                          Esp32IrqHandler handler)
{
    if (cpu_int_id == 0U || cpu_int_id >= 32U)
    {
        return false;
    }
    if (prio == 0U)
    {
        prio = 1U;
    }
    if (prio > 15U)
    {
        prio = 15U;
    }

    uint32_t mstatus = Esp32IrqSaveAndDisableMie();

    ESP32_INTMTX_CLOCK_GATE_REG = 1U;

    // Optional ROM-hook seed.  The weak references resolve to NULL when
    // Vectors_esp32_intmtx.c is not linked, so the call sites compile
    // either way.  Address-test syntax is required for weak symbols.
    if (&Esp32C3SetCpuIrqHandler != 0 && handler != 0)
    {
        Esp32C3SetCpuIrqHandler(cpu_int_id, handler);
    }
    if (&Esp32C3SetSourceIrqHandler != 0 && handler != 0)
    {
        Esp32C3SetSourceIrqHandler(source_id, handler);
    }

    ESP32_INTMTX_SOURCE_MAP_REG(source_id) = cpu_int_id;

    ESP32_CPU_INT_CLEAR_REG = (1UL << cpu_int_id);
    ESP32_CPU_INT_CLEAR_REG = 0U;
    ESP32_CPU_INT_TYPE_REG  &= ~(1UL << cpu_int_id);    // 0 = level
    ESP32_CPU_INT_PRI_REG(cpu_int_id) = prio;
    ESP32_CPU_INT_THRESH_REG = 0U;
    ESP32_CPU_INT_ENABLE_REG |= (1UL << cpu_int_id);

    Esp32IrqFenceIo();
    Esp32IrqRestoreMie(mstatus);

    return true;
}

void Esp32DisableSourceIrq(uint32_t source_id, uint32_t cpu_int_id)
{
    (void)source_id;
    if (cpu_int_id == 0U || cpu_int_id >= 32U)
    {
        return;
    }
    uint32_t mstatus = Esp32IrqSaveAndDisableMie();
    ESP32_CPU_INT_ENABLE_REG &= ~(1UL << cpu_int_id);
    Esp32IrqFenceIo();
    Esp32IrqRestoreMie(mstatus);
}

#endif // C3 || C6


// ===========================================================================
// C5: CLIC-based install.
//
// Steps:
//   1. Save MIE, clear MIE
//   2. Compute CLIC slot = source_id + CLIC_EXT_INTR_NUM_OFFSET
//   3. ATTR  : MODE = machine, TRIG = level, SHV = 0 (non-vectored;
//              dispatch goes through DEF_IRQHandler in
//              Vectors_esp32_clic.c, which then jumps to the bound
//              handler)
//   4. CTL   : level << (8 - NLBITS) i.e. (prio << 5) for NLBITS = 3
//   5. IP    : write 1 to clear any pending bit
//   6. IE    : 1 to enable
//   7. Fence
//   8. Restore MIE
//
// `cpu_int_id` is unused on C5; CLIC has per-source slots, not shared
// CPU INT lines.  `handler` is unused unless a ROM-hook installer
// equivalent is later added for C5.
// ===========================================================================
#if defined(ESP32C5)

bool Esp32EnableSourceIrq(uint32_t source_id,
                          uint32_t cpu_int_id,
                          uint32_t prio,
                          Esp32IrqHandler handler)
{
    (void)cpu_int_id;
    (void)handler;

    uint32_t slot = source_id + ESP32_CLIC_EXT_INTR_NUM_OFFSET;
    if (slot >= ESP32_CLIC_INT_COUNT)
    {
        return false;
    }
    if (prio == 0U) prio = 1U;
    if (prio > 7U)  prio = 7U;

    uint32_t mstatus = Esp32IrqSaveAndDisableMie();

    ESP32_CLIC_INT_ATTR_REG(slot) = (uint8_t)(ESP32_CLIC_INT_ATTR_MODE_MACHINE
                                              | ESP32_CLIC_INT_ATTR_TRIG_LEVEL);
    ESP32_CLIC_INT_CTL_REG(slot)  = ESP32_CLIC_INT_CTL_FROM_LEVEL(prio);
    ESP32_CLIC_INT_IP_REG(slot)   = 0U;     // clear stale pending
    ESP32_CLIC_INT_IE_REG(slot)   = 1U;

    Esp32IrqFenceIo();
    Esp32IrqRestoreMie(mstatus);

    return true;
}

void Esp32DisableSourceIrq(uint32_t source_id, uint32_t cpu_int_id)
{
    (void)cpu_int_id;
    uint32_t slot = source_id + ESP32_CLIC_EXT_INTR_NUM_OFFSET;
    if (slot >= ESP32_CLIC_INT_COUNT)
    {
        return;
    }
    uint32_t mstatus = Esp32IrqSaveAndDisableMie();
    ESP32_CLIC_INT_IE_REG(slot) = 0U;
    Esp32IrqFenceIo();
    Esp32IrqRestoreMie(mstatus);
}

#endif // C5

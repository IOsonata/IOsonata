/**-------------------------------------------------------------------------
@file	esp32xx_irq.c

@brief	Cross-chip peripheral interrupt install for the IOsonata ESP32
        RISC-V family.

INTC (C3 / C6) and CLIC (C5) implementations of the API declared in
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

#include "esp32xx.h"
#include "esp32xx_intmtx.h"
#include "esp32xx_irq.h"

#if defined(ESP32C5)
#include "esp32xx_clic.h"
#endif


// ===========================================================================
// Local helpers (mstatus + memory fence).
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

// Restore mstatus.MIE to whatever it was when SaveAndDisableMie() returned.
// Used by the lower-level Route/Install/Disable functions, which must NOT
// silently raise MIE on behalf of the caller.  The convenience wrapper
// Esp32EnableSourceIrq() calls Esp32GlobalIrqEnable() separately to make
// "MIE = 1 after this returns" an explicit, opt-in policy.
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
    // `fence iorw, iorw` is base RISC-V; required between INTC writes and
    // any subsequent enable that depends on them being globally visible.
    __asm volatile("fence iorw, iorw" ::: "memory");
#else
    __asm volatile("" ::: "memory");
#endif
}


// ===========================================================================
// Esp32GlobalIrqEnable - separated MIE enable.
// ===========================================================================
void Esp32GlobalIrqEnable(void)
{
#if defined(__riscv)
    __asm volatile("csrsi mstatus, 0x8" ::: "memory");
#endif
}


// ===========================================================================
// C3 + C6: INTMTX-based implementation.
// ===========================================================================
#if defined(ESP32C3) || defined(ESP32C6)

// Maximum number of INTMTX source slots.  C3 uses 0..62, C6 uses 0..77,
// allocate a small upper bound that covers both.  Used only by the
// disable-side scan that decides whether cpu_int_id can be safely cleared.
#define ESP32_INTMTX_NUM_SOURCES        80U

// Scan all source map registers to see whether any source other than
// `excluding_source` still maps to `cpu_int_id`.  Used to decide if the
// CPU INT enable + CPU-level handler can be cleared in disable.
static bool Esp32IntcSourceShares(uint32_t cpu_int_id, uint32_t excluding_source)
{
    for (uint32_t src = 1U; src < ESP32_INTMTX_NUM_SOURCES; src++)
    {
        if (src == excluding_source)
        {
            continue;
        }
        if (ESP32_INTMTX_SOURCE_MAP_REG(src) == cpu_int_id)
        {
            return true;
        }
    }
    return false;
}

bool Esp32RouteSourceIrq(uint32_t source_id, uint32_t cpu_int_id, uint32_t prio)
{
    if (cpu_int_id == 0U || cpu_int_id >= 32U)
    {
        return false;
    }
    if (source_id >= ESP32_INTMTX_NUM_SOURCES)
    {
        return false;
    }
    if (prio == 0U)        prio = 1U;
    if (prio > 15U)        prio = 15U;

    uint32_t mstatus = Esp32IrqSaveAndDisableMie();

    ESP32_INTMTX_CLOCK_GATE_REG = 1U;

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

void Esp32InstallIrqHandler(uint32_t source_id, uint32_t cpu_int_id,
                            Esp32IrqHandler handler, Esp32IrqMode mode)
{
    if (handler == 0)
    {
        return;
    }

    uint32_t mstatus = Esp32IrqSaveAndDisableMie();

    if (mode == ESP32_IRQ_DIRECT)
    {
        // Single source owns cpu_int_id.  Drop the handler at the CPU-level
        // slot; RISCV_TrapHandler will call it directly and skip the
        // source-status dispatcher.  No source-level entry needed.
        if (&Esp32C3SetCpuIrqHandler != 0)
        {
            Esp32C3SetCpuIrqHandler(cpu_int_id, handler);
        }
    }
    else  // ESP32_IRQ_SHARED
    {
        // Multiple sources share cpu_int_id.  Point the CPU-level slot at
        // the matrix dispatcher (read INTMTX status, fan out per source),
        // and put this source's actual handler at its source-level slot.
        // Doing the source-level write first means the dispatcher cannot
        // observe an incomplete state if the IRQ fires mid-install.
        if (&Esp32C3SetSourceIrqHandler != 0)
        {
            Esp32C3SetSourceIrqHandler(source_id, handler);
        }
        if (&Esp32C3SetCpuIrqHandler != 0 && &Esp32C3IntMtxDispatch != 0)
        {
            Esp32C3SetCpuIrqHandler(cpu_int_id, &Esp32C3IntMtxDispatch);
        }
    }

    Esp32IrqFenceIo();
    Esp32IrqRestoreMie(mstatus);
}

bool Esp32EnableSourceIrq(uint32_t source_id, uint32_t cpu_int_id,
                          uint32_t prio, Esp32IrqHandler handler)
{
    if (!Esp32RouteSourceIrq(source_id, cpu_int_id, prio))
    {
        return false;
    }
    Esp32InstallIrqHandler(source_id, cpu_int_id, handler, ESP32_IRQ_DIRECT);
    Esp32GlobalIrqEnable();
    return true;
}

void Esp32DisableSourceIrq(uint32_t source_id, uint32_t cpu_int_id)
{
    if (cpu_int_id == 0U || cpu_int_id >= 32U)
    {
        return;
    }
    if (source_id >= ESP32_INTMTX_NUM_SOURCES)
    {
        return;
    }

    uint32_t mstatus = Esp32IrqSaveAndDisableMie();

    // Step 1: stop routing this source to cpu_int_id BEFORE touching the
    // CPU-side state.  Any pending edge from this source after this write
    // is silently dropped.
    ESP32_INTMTX_SOURCE_MAP_REG(source_id) = 0U;     // 0 = disabled, per TRM

    // Step 2: drop the source-level handler (cheap, always safe).
    if (&Esp32C3ClearSourceIrqHandler != 0)
    {
        Esp32C3ClearSourceIrqHandler(source_id);
    }

    // Step 3: if no other source still maps to cpu_int_id, the CPU INT
    // line itself is now idle - mask it and clear its CPU-level handler.
    // If other sources are still mapped (SHARED mode), leave both the
    // enable bit and the dispatcher in place so they keep working.
    if (!Esp32IntcSourceShares(cpu_int_id, source_id))
    {
        ESP32_CPU_INT_ENABLE_REG &= ~(1UL << cpu_int_id);
        if (&Esp32C3ClearCpuIrqHandler != 0)
        {
            Esp32C3ClearCpuIrqHandler(cpu_int_id);
        }
    }

    Esp32IrqFenceIo();
    Esp32IrqRestoreMie(mstatus);
}

#endif // C3 || C6


// ===========================================================================
// C5: CLIC-based implementation.
// ===========================================================================
#if defined(ESP32C5)

bool Esp32RouteSourceIrq(uint32_t source_id, uint32_t cpu_int_id, uint32_t prio)
{
    (void)cpu_int_id;

    uint32_t slot = source_id + ESP32_CLIC_EXT_INTR_NUM_OFFSET;

    uint32_t lvl_max = (1U << ESP32_CLIC_NLBITS) - 1U;
    if (prio == 0U)        prio = 1U;
    if (prio > lvl_max)    prio = lvl_max;

    uint32_t mstatus = Esp32IrqSaveAndDisableMie();

    bool ok = ClicEnableSlot((uintptr_t)ESP32_CLIC_CTRL_BASE,
                             slot,
                             ESP32_CLIC_INT_COUNT,
                             CLIC_TRIG_LEVEL_HIGH,
                             (uint8_t)prio,
                             (uint8_t)ESP32_CLIC_NLBITS);

    Esp32IrqFenceIo();
    Esp32IrqRestoreMie(mstatus);
    return ok;
}

// CLIC has per-source slots, no shared CPU INT lines, so handler install
// is not part of the CLIC programming model in IOsonata's non-vectored
// (SHV = 0) configuration: dispatch is via DEF_IRQHandler in
// Vectors_esp32_clic.c, which the linker resolves to the bound driver
// handler symbol.  Keep this entry as a no-op so callers can use the
// same API across chip families.
void Esp32InstallIrqHandler(uint32_t source_id, uint32_t cpu_int_id,
                            Esp32IrqHandler handler, Esp32IrqMode mode)
{
    (void)source_id; (void)cpu_int_id; (void)handler; (void)mode;
}

bool Esp32EnableSourceIrq(uint32_t source_id, uint32_t cpu_int_id,
                          uint32_t prio, Esp32IrqHandler handler)
{
    (void)handler;
    if (!Esp32RouteSourceIrq(source_id, cpu_int_id, prio))
    {
        return false;
    }
    Esp32GlobalIrqEnable();
    return true;
}

void Esp32DisableSourceIrq(uint32_t source_id, uint32_t cpu_int_id)
{
    (void)cpu_int_id;

    uint32_t slot = source_id + ESP32_CLIC_EXT_INTR_NUM_OFFSET;

    uint32_t mstatus = Esp32IrqSaveAndDisableMie();
    (void)ClicDisableSlot((uintptr_t)ESP32_CLIC_CTRL_BASE,
                          slot,
                          ESP32_CLIC_INT_COUNT);
    Esp32IrqFenceIo();
    Esp32IrqRestoreMie(mstatus);
}

#endif // C5

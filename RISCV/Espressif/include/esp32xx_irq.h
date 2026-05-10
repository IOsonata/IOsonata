/**-------------------------------------------------------------------------
@file	esp32xx_irq.h

@brief	Cross-chip peripheral interrupt install helpers for the IOsonata
        ESP32 RISC-V family.

API split:

  Esp32RouteSourceIrq()  - configure the controller (matrix routing on
                           C3/C6, CLIC slot on C5).  Does not touch
                           mstatus.MIE, does not install handlers.
                           Safe to call on multiple sources before
                           interrupts are globally enabled.

  Esp32InstallIrqHandler() - install a handler with explicit ownership
                           of the CPU interrupt input:
                             ESP32_IRQ_DIRECT  (only this source uses
                                                cpu_int_id)
                             ESP32_IRQ_SHARED  (multiple sources share
                                                cpu_int_id; the source-
                                                status dispatcher is
                                                installed at the CPU-
                                                level slot, the actual
                                                handler at the source-
                                                level slot)
                           No-op on C5 (CLIC has per-source slots).

  Esp32GlobalIrqEnable() - set mstatus.MIE = 1.  Caller-controlled, so
                           startup code can decide when interrupts go
                           live.

  Esp32EnableSourceIrq() - convenience wrapper preserving the existing
                           one-shot semantics: Route + Install (DIRECT)
                           + GlobalEnable in one call.  Returns true on
                           success.

  Esp32DisableSourceIrq() - reverse of Esp32EnableSourceIrq:
                             - clears the source map (C3/C6) or CLIC IE (C5)
                             - clears the source-level handler ref
                             - if no other source still maps to cpu_int_id,
                               also clears the CPU-INT enable bit and
                               the CPU-level handler ref

C3/C6 (INTMTX) parameter ranges:
    cpu_int_id : 1..31  (0 reserved for "disabled" per TRM)
    prio       : 1..15

C5 (CLIC) parameter ranges:
    cpu_int_id : ignored (CLIC slot derived from source_id)
    prio       : 1..(2^NLBITS - 1)  -- 1..7 with the IDF default NLBITS = 3

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
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __ESP32XX_IRQ_H__
#define __ESP32XX_IRQ_H__

#include <stdint.h>
#include <stdbool.h>

#include "esp32xx.h"
#include "esp32xx_intmtx.h"
#if defined(ESP32C5)
#include "esp32xx_clic.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Handler signature for the C3/C6 ROM-table dispatch tables.  Each
// stored handler is a no-arg function; the wrapper that calls it is
// expected to capture any per-source state (e.g. UART0 / UART1).
typedef void (*Esp32IrqHandler)(void);

// Ownership mode for Esp32InstallIrqHandler() on C3/C6.  Ignored on C5.
typedef enum {
    ESP32_IRQ_DIRECT = 0,   //!< exclusive owner of cpu_int_id
    ESP32_IRQ_SHARED = 1,   //!< multiple sources share cpu_int_id; dispatcher routes
} Esp32IrqMode;

// Forward declarations for the ROM-table handler installers in
// Vectors_esp32c3.c (C3) and Vectors_esp32c6.c (C6).  Declared weak
// so the call sites compile even when the vector file is not linked
// in, in which case the address-test in the .c file makes them no-ops.
extern void Esp32C3SetCpuIrqHandler   (uint32_t cpu_int_id, Esp32IrqHandler handler) __attribute__((weak));
extern void Esp32C3SetSourceIrqHandler(uint32_t source_id,  Esp32IrqHandler handler) __attribute__((weak));
extern void Esp32C3ClearCpuIrqHandler   (uint32_t cpu_int_id) __attribute__((weak));
extern void Esp32C3ClearSourceIrqHandler(uint32_t source_id)  __attribute__((weak));
extern void Esp32C3IntMtxDispatch       (void)                __attribute__((weak));

/**
 * Configure the interrupt controller for `source_id` so it can fire at
 * the CPU.  Does NOT install per-source/per-CPU handlers and does NOT
 * touch mstatus.MIE.
 *
 * C3/C6 (INTMTX): writes ESP32_INTMTX_SOURCE_MAP_REG[source_id] = cpu_int_id,
 *                 sets CPU INT line `cpu_int_id` to level type, priority
 *                 = prio, threshold = 0, and enables it.
 *
 * C5 (CLIC):      configures CLIC slot = source_id + ESP32_CLIC_EXT_INTR_NUM_OFFSET
 *                 with mode = M, trigger = level-high, level = prio,
 *                 SHV = 0, IE = 1.
 */
bool Esp32RouteSourceIrq(uint32_t source_id, uint32_t cpu_int_id, uint32_t prio);

/**
 * Install a handler with explicit ownership of cpu_int_id.
 *
 * ESP32_IRQ_DIRECT (C3/C6):
 *   CpuHandler[cpu_int_id]  = handler
 *   The CPU dispatcher in RISCV_TrapHandler calls handler() directly.
 *   Use when the source has exclusive ownership of cpu_int_id.
 *
 * ESP32_IRQ_SHARED (C3/C6):
 *   CpuHandler[cpu_int_id]  = Esp32C3IntMtxDispatch  (matrix dispatcher)
 *   SourceHandler[source_id] = handler
 *   The CPU dispatcher routes to the source-status dispatcher, which
 *   reads INTMTX status and calls the per-source handler.  Use when
 *   multiple sources share cpu_int_id.
 *
 * On C5 this is a no-op (CLIC slots are per-source).
 *
 * Caller is responsible for not mixing modes on the same cpu_int_id;
 * a later DIRECT install on a CPU INT that is already in SHARED use
 * silently overwrites the dispatcher pointer and breaks dispatch for
 * the other sources.
 */
void Esp32InstallIrqHandler(uint32_t source_id, uint32_t cpu_int_id,
                            Esp32IrqHandler handler, Esp32IrqMode mode);

/**
 * Globally enable interrupts (set mstatus.MIE = 1).  No-op on non-RISC-V
 * builds.  Separated from the install path so startup code can decide
 * when interrupts go live.
 */
void Esp32GlobalIrqEnable(void);

/**
 * Convenience: Esp32RouteSourceIrq + Esp32InstallIrqHandler(DIRECT) +
 * Esp32GlobalIrqEnable, in that order.  Preserves the prior one-shot
 * semantics for existing call sites.  Returns the result of the
 * routing step (false if cpu_int_id or source_id is out of range).
 */
bool Esp32EnableSourceIrq(uint32_t source_id, uint32_t cpu_int_id,
                          uint32_t prio, Esp32IrqHandler handler);

/**
 * Reverse of Esp32EnableSourceIrq.
 *
 * C3/C6:
 *   - Clears ESP32_INTMTX_SOURCE_MAP_REG[source_id]    (= 0, "disabled")
 *   - Clears the source-level handler ref.
 *   - If no remaining source maps to cpu_int_id, also clears the
 *     CPU INT enable bit and the CPU-level handler ref.
 *
 * C5: clears CLIC IE for the slot derived from source_id.  cpu_int_id
 *     is ignored.
 */
void Esp32DisableSourceIrq(uint32_t source_id, uint32_t cpu_int_id);

#ifdef __cplusplus
}
#endif

#endif // __ESP32XX_IRQ_H__

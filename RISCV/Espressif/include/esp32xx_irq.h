/**-------------------------------------------------------------------------
@file	esp32xx_irq.h

@brief	Cross-chip peripheral interrupt install helpers for the IOsonata
        ESP32 RISC-V family.

Routes a peripheral interrupt source to a CPU interrupt and configures
trigger type, priority, and enable.  Hides the C3/C6 (INTMTX + global
CPU INT controller) vs C5 (CLIC, per-interrupt control) split behind
a single API.

Two functions:

  Esp32EnableSourceIrq(source_id, cpu_int_id, prio, handler)
    Routes peripheral source `source_id` so it can fire at the CPU and
    enables it.  Returns true on success.

      C3, C6 (INTC):
        - Writes ESP32_INTMTX_SOURCE_MAP_REG[source_id] = cpu_int_id
        - Configures CPU INT line `cpu_int_id`: TYPE = level, PRI = prio,
          ENABLE = 1, THRESH = 0
        - Optionally calls `handler` through the ROM-table install hooks
          (Esp32C3SetCpuIrqHandler / SetSourceIrqHandler) when those are
          available.
        - `prio` valid range: 1..15.
        - `cpu_int_id` valid range: 1..31.

      C5 (CLIC):
        - Computes CLIC slot = source_id + ESP32_CLIC_EXT_INTR_NUM_OFFSET
        - Sets ATTR (mode = M, trigger = level, SHV from chosen vectoring),
          CTL (priority byte), IE = 1.
        - `cpu_int_id` is ignored (CLIC does not use a separate CPU INT
          line numbering).
        - `prio` valid range: 1..7 with NLBITS = 3 (IDF default), or
          0..255 if directly programming the CTL byte.

  Esp32DisableSourceIrq(source_id, cpu_int_id)
    Disables the source.  On C3/C6 clears the CPU INT line's enable
    bit.  On C5 clears CLIC IE for the corresponding slot.

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

// Optional ROM-hook installer callback type.  Used on C3 to seed the
// ROM dispatcher; pass NULL if not using that path.
typedef void (*Esp32IrqHandler)(void);

// Forward declarations for the ROM-hook installers (defined in
// Vectors_esp32_intmtx.c on chips that support it; left as weak so
// the call is a no-op when the file is not linked in).
extern void Esp32C3SetCpuIrqHandler(uint32_t cpu_int_id, Esp32IrqHandler handler) __attribute__((weak));
extern void Esp32C3SetSourceIrqHandler(uint32_t source_id, Esp32IrqHandler handler) __attribute__((weak));

// Route a peripheral source through the interrupt controller, configure
// it as a level interrupt with the given priority, and enable it.
//
//   source_id  : INTMTX source number (e.g. ESP32_GPIO_INTR_SOURCE_ID)
//   cpu_int_id : C3/C6 only -- which CPU INT line (1..31) to use.  On
//                C5 this is ignored (CLIC has per-interrupt slots).
//   prio       : priority.  C3/C6: 1..15.  C5: 1..7 (NLBITS = 3) or
//                a direct CTL byte if larger.
//   handler    : optional ROM-table hook.  May be NULL.
//
// Returns true on success.
bool Esp32EnableSourceIrq(uint32_t source_id,
                          uint32_t cpu_int_id,
                          uint32_t prio,
                          Esp32IrqHandler handler);

// Disable a peripheral source's interrupt routing.
//
//   source_id  : INTMTX source number (used on C5 to compute CLIC slot)
//   cpu_int_id : C3/C6 only -- which CPU INT line to clear.  Ignored on C5.
void Esp32DisableSourceIrq(uint32_t source_id, uint32_t cpu_int_id);

#ifdef __cplusplus
}
#endif

#endif // __ESP32XX_IRQ_H__

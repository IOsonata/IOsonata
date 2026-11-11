/**-------------------------------------------------------------------------
@file	esp32xx_irq.h

@brief	Cross-chip peripheral interrupt install helpers for the IOsonata
        ESP32 RISC-V family.

All C-series targets route peripheral sources through a matrix to CPU
inputs. C2/C3/C6 use INTC; C5/C61 add 16 to the CPU input for the CLIC slot.
CPU inputs are 1..31, excluding the disabled input (6 on C2/C3/C5/C61)
and C6 CPU-local inputs 3, 4 and 7. Priorities are 1..15 on INTC, 1..7 on CLIC.

RouteSource configures hardware while preserving MIE. InstallIrqHandler
installs direct or shared dispatch. EnableSource performs both under one
interrupt mask, installs the handler before routing, then enables MIE.
DisableSource disconnects only the specified mapping and keeps a shared
CPU input enabled until its last source is removed.

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
#if defined(ESP32C5) || defined(ESP32C61)
#include "esp32xx_clic.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Handler signature for the CPU and source dispatch tables. Each
// stored handler is a no-arg function; the wrapper that calls it is
// expected to capture any per-source state (e.g. UART0 / UART1).
typedef void (*Esp32IrqHandler)(void);

// Ownership mode for each matrix-routed CPU interrupt.
typedef enum {
	ESP32_IRQ_DIRECT = 0,   //!< exclusive owner of cpu_int_id
	ESP32_IRQ_SHARED = 1,   //!< multiple sources share cpu_int_id; dispatcher routes
} Esp32IrqMode;

// Keep the historical hook names used by C3/C6 vectors. C2/C5/C61
// implement the same hooks in Vectors_esp32.c. A missing vector provider
// makes EnableSource fail; the low-level installer leaves missing hooks alone.
extern void Esp32C3SetCpuIrqHandler   (uint32_t cpu_int_id, Esp32IrqHandler handler) __attribute__((weak));
extern void Esp32C3SetSourceIrqHandler(uint32_t source_id,  Esp32IrqHandler handler) __attribute__((weak));
extern void Esp32C3ClearCpuIrqHandler   (uint32_t cpu_int_id) __attribute__((weak));
extern void Esp32C3ClearSourceIrqHandler(uint32_t source_id)  __attribute__((weak));
extern void Esp32C3IntMtxDispatch       (void)                __attribute__((weak));

// Preserves MIE; the caller must install a handler before enabling a route.
bool Esp32RouteSourceIrq(uint32_t source_id, uint32_t cpu_int_id, uint32_t prio);

// Do not mix direct and shared ownership on the same CPU input.
void Esp32InstallIrqHandler(uint32_t source_id, uint32_t cpu_int_id,
	Esp32IrqHandler handler, Esp32IrqMode mode);

void Esp32GlobalIrqEnable(void);
bool Esp32EnableSourceIrq(uint32_t source_id, uint32_t cpu_int_id,
	uint32_t prio, Esp32IrqHandler handler);
void Esp32DisableSourceIrq(uint32_t source_id, uint32_t cpu_int_id);

#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
// Shared vectors for new targets. IRQ installation calls initialization once,
// with MIE masked. Startup may also call it before enabling any peripherals.
void Esp32InitVectors(void);
bool Esp32DispatchCpuIrq(uint32_t Cause);
#endif

#ifdef __cplusplus
}
#endif

#endif // __ESP32XX_IRQ_H__

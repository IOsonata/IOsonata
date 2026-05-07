/**-------------------------------------------------------------------------
@file	iatomic.c

@brief	GCC __atomic_* libcall implementation for ARM Cortex-M cores
	that lack LDREX/STREX — i.e. Armv6-M (Cortex-M0/M0+/M1).

	Implements the libcalls GCC emits when it cannot lower a C11
	`<stdatomic.h>` operation or a C++11/14/17/20/23 `<atomic>`
	operation directly to a hardware atomic instruction.  When
	the target architecture *does* expose LDREX/STREX (Armv7-M
	on Cortex-M3/M4/M7, Armv8-M Baseline on M23, Armv8-M Mainline
	on M33/M55, etc.) the entire body of this file is guarded out
	by `__ARM_FEATURE_LDREX` and the compiler's inline exclusive-
	monitor sequences are used instead.  We never override the
	compiler-provided implementation.

	ABI followed (per GCC __atomic builtin documentation, which
	mirrors the ISO C11 §7.17 / C++11 §29.6 atomic memory model):

	  TYPE __atomic_load_N        (const volatile void *p, int mo)
	  void __atomic_store_N       (volatile void *p, TYPE v, int mo)
	  TYPE __atomic_exchange_N    (volatile void *p, TYPE v, int mo)
	  bool __atomic_compare_exchange_N (volatile void *p, void *expected,
	                                    TYPE desired, bool weak,
	                                    int success_mo, int failure_mo)
	  TYPE __atomic_fetch_OP_N    (volatile void *p, TYPE v, int mo)
	          -> returns *p as it was BEFORE the operation
	  TYPE __atomic_OP_fetch_N    (volatile void *p, TYPE v, int mo)
	          -> returns *p as it is  AFTER  the operation

	with OP in { add, sub, and, or, xor, nand } and N in { 1, 2, 4 }.

	Plus the byte-scoped flag operations:
	  bool __atomic_test_and_set  (volatile void *p, int mo)
	  void __atomic_clear         (volatile void *p, int mo)

	And the fences and lock-free queries:
	  void __atomic_thread_fence  (int mo)
	  void __atomic_signal_fence  (int mo)
	  bool __atomic_is_lock_free  (unsigned size, const volatile void *p)
	  bool __atomic_always_lock_free (unsigned size, const volatile void *p)

	Memory-order parameters are accepted but always promoted to
	seq_cst — fully ISO-conformant since the standard requires
	*at least* the requested ordering, not exactly the requested
	ordering.  On a single-core in-order Cortex-M with PRIMASK
	masking IRQs across each RMW, every operation observes
	seq_cst with respect to any other code on the same core.

	Critical-section primitive: CMSIS `__get_PRIMASK` /
	`__disable_irq` / `__set_PRIMASK`.  Same role as the
	mstatus.MIE save/restore in the RISC-V port
	(`RISCV/src/iatomic.c`).

@author	Hoang Nguyen Hoan
@date	Mar. 22, 2015 (rewritten May 2026 for ISO C11/C++11 conformance)

@license

MIT License

Copyright (c) 2017, I-SYST inc., all rights reserved

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

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 *  Compile this file's body ONLY on ARM targets that lack LDREX/STREX.
 *  When the architecture exposes the exclusive monitor (Armv7-M on
 *  Cortex-M3/M4/M7, Armv8-M Mainline on M33/M55, etc.), GCC predefines
 *  `__ARM_FEATURE_LDREX` to a bitmask of supported sizes and inlines
 *  every atomic operation via the LDREX/STREX exclusive-monitor pair;
 *  it never emits any `__atomic_*_N` libcall, so we provide nothing.
 *=========================================================================*/

#if defined(__arm__) && !defined(__ARM_FEATURE_LDREX)

#include "cmsis_compiler.h"

/*===========================================================================
 *  Critical-section primitives
 *=========================================================================*/

static inline uint32_t critical_enter(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

static inline void critical_exit(uint32_t saved)
{
    __set_PRIMASK(saved);
}

/*===========================================================================
 *  Per-size template.  Each invocation generates 16 functions for one
 *  width: load, store, exchange, compare_exchange, fetch_{add,sub,and,
 *  or,xor,nand}, {add,sub,and,or,xor,nand}_fetch.
 *=========================================================================*/

#define ATOMIC_LIBCALLS(N, TYPE)                                               \
                                                                               \
TYPE __atomic_load_##N(const volatile void *ptr, int mo)                       \
{                                                                              \
    (void)mo;                                                                  \
    /* Naturally-aligned 1/2/4-byte load is atomic on ARMv6-M — no IRQ mask */ \
    return *(const volatile TYPE *)ptr;                                        \
}                                                                              \
                                                                               \
void __atomic_store_##N(volatile void *ptr, TYPE val, int mo)                  \
{                                                                              \
    (void)mo;                                                                  \
    /* Naturally-aligned 1/2/4-byte store is atomic on ARMv6-M. */             \
    *(volatile TYPE *)ptr = val;                                               \
}                                                                              \
                                                                               \
TYPE __atomic_exchange_##N(volatile void *ptr, TYPE val, int mo)               \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE old = *(volatile TYPE *)ptr;                                          \
    *(volatile TYPE *)ptr = val;                                               \
    critical_exit(s);                                                          \
    return old;                                                                \
}                                                                              \
                                                                               \
bool __atomic_compare_exchange_##N(volatile void *ptr, void *expected,         \
                                   TYPE desired, bool weak,                    \
                                   int s_mo, int f_mo)                         \
{                                                                              \
    (void)weak; (void)s_mo; (void)f_mo;                                        \
    uint32_t s = critical_enter();                                             \
    TYPE cur = *(volatile TYPE *)ptr;                                          \
    bool ok = (cur == *(TYPE *)expected);                                      \
    if (ok) { *(volatile TYPE *)ptr = desired; }                               \
    else    { *(TYPE *)expected     = cur;     }                               \
    critical_exit(s);                                                          \
    return ok;                                                                 \
}                                                                              \
                                                                               \
/* ----- fetch_OP : returns OLD value -------------------------------------- */\
                                                                               \
TYPE __atomic_fetch_add_##N(volatile void *ptr, TYPE val, int mo)              \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE old = *(volatile TYPE *)ptr;                                          \
    *(volatile TYPE *)ptr = (TYPE)(old + val);                                 \
    critical_exit(s);                                                          \
    return old;                                                                \
}                                                                              \
                                                                               \
TYPE __atomic_fetch_sub_##N(volatile void *ptr, TYPE val, int mo)              \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE old = *(volatile TYPE *)ptr;                                          \
    *(volatile TYPE *)ptr = (TYPE)(old - val);                                 \
    critical_exit(s);                                                          \
    return old;                                                                \
}                                                                              \
                                                                               \
TYPE __atomic_fetch_and_##N(volatile void *ptr, TYPE val, int mo)              \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE old = *(volatile TYPE *)ptr;                                          \
    *(volatile TYPE *)ptr = (TYPE)(old & val);                                 \
    critical_exit(s);                                                          \
    return old;                                                                \
}                                                                              \
                                                                               \
TYPE __atomic_fetch_or_##N(volatile void *ptr, TYPE val, int mo)               \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE old = *(volatile TYPE *)ptr;                                          \
    *(volatile TYPE *)ptr = (TYPE)(old | val);                                 \
    critical_exit(s);                                                          \
    return old;                                                                \
}                                                                              \
                                                                               \
TYPE __atomic_fetch_xor_##N(volatile void *ptr, TYPE val, int mo)              \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE old = *(volatile TYPE *)ptr;                                          \
    *(volatile TYPE *)ptr = (TYPE)(old ^ val);                                 \
    critical_exit(s);                                                          \
    return old;                                                                \
}                                                                              \
                                                                               \
TYPE __atomic_fetch_nand_##N(volatile void *ptr, TYPE val, int mo)             \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE old = *(volatile TYPE *)ptr;                                          \
    *(volatile TYPE *)ptr = (TYPE)(~(old & val));                              \
    critical_exit(s);                                                          \
    return old;                                                                \
}                                                                              \
                                                                               \
/* ----- OP_fetch : returns NEW value -------------------------------------- */\
                                                                               \
TYPE __atomic_add_fetch_##N(volatile void *ptr, TYPE val, int mo)              \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE nv = (TYPE)(*(volatile TYPE *)ptr + val);                             \
    *(volatile TYPE *)ptr = nv;                                                \
    critical_exit(s);                                                          \
    return nv;                                                                 \
}                                                                              \
                                                                               \
TYPE __atomic_sub_fetch_##N(volatile void *ptr, TYPE val, int mo)              \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE nv = (TYPE)(*(volatile TYPE *)ptr - val);                             \
    *(volatile TYPE *)ptr = nv;                                                \
    critical_exit(s);                                                          \
    return nv;                                                                 \
}                                                                              \
                                                                               \
TYPE __atomic_and_fetch_##N(volatile void *ptr, TYPE val, int mo)              \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE nv = (TYPE)(*(volatile TYPE *)ptr & val);                             \
    *(volatile TYPE *)ptr = nv;                                                \
    critical_exit(s);                                                          \
    return nv;                                                                 \
}                                                                              \
                                                                               \
TYPE __atomic_or_fetch_##N(volatile void *ptr, TYPE val, int mo)               \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE nv = (TYPE)(*(volatile TYPE *)ptr | val);                             \
    *(volatile TYPE *)ptr = nv;                                                \
    critical_exit(s);                                                          \
    return nv;                                                                 \
}                                                                              \
                                                                               \
TYPE __atomic_xor_fetch_##N(volatile void *ptr, TYPE val, int mo)              \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE nv = (TYPE)(*(volatile TYPE *)ptr ^ val);                             \
    *(volatile TYPE *)ptr = nv;                                                \
    critical_exit(s);                                                          \
    return nv;                                                                 \
}                                                                              \
                                                                               \
TYPE __atomic_nand_fetch_##N(volatile void *ptr, TYPE val, int mo)             \
{                                                                              \
    (void)mo;                                                                  \
    uint32_t s = critical_enter();                                             \
    TYPE nv = (TYPE)(~(*(volatile TYPE *)ptr & val));                          \
    *(volatile TYPE *)ptr = nv;                                                \
    critical_exit(s);                                                          \
    return nv;                                                                 \
}

/* Instantiate for the three sizes ARM commonly emits. */
ATOMIC_LIBCALLS(1, uint8_t)
ATOMIC_LIBCALLS(2, uint16_t)
ATOMIC_LIBCALLS(4, uint32_t)

/*===========================================================================
 *  Byte-scoped flag operations.
 *
 *  Per C11 §7.17.8 and the GCC ABI, atomic_flag::test_and_set / clear
 *  operate on a single byte regardless of the surrounding type's size.
 *=========================================================================*/

bool __atomic_test_and_set(volatile void *ptr, int mo)
{
    (void)mo;
    uint32_t s = critical_enter();
    unsigned char old = *(volatile unsigned char *)ptr;
    *(volatile unsigned char *)ptr = 1U;
    critical_exit(s);
    return old != 0;
}

void __atomic_clear(volatile void *ptr, int mo)
{
    (void)mo;
    /* Aligned byte store is atomic; no IRQ mask required. */
    *(volatile unsigned char *)ptr = 0U;
}

/*===========================================================================
 *  Memory fences.
 *
 *  Single-core in-order Cortex-M with interrupts masked across each RMW
 *  gives us the strongest ordering for free.  A compiler barrier is
 *  enough to defeat C/C++-level reordering; no DSB is required because
 *  there are no other observers (no SMP, no cache coherency to manage).
 *=========================================================================*/

void __atomic_thread_fence(int mo)
{
    (void)mo;
    __asm volatile ("" ::: "memory");
}

void __atomic_signal_fence(int mo)
{
    (void)mo;
    __asm volatile ("" ::: "memory");
}

/*===========================================================================
 *  Lock-free queries.
 *
 *  Returning false ("not lock-free") is the truthful answer when the
 *  hardware can't do these atomically without our software guard.
 *  std::atomic<T> callers can branch on this if they want to choose
 *  a different strategy.
 *=========================================================================*/

bool __atomic_is_lock_free(unsigned int objsize, const volatile void *ptr)
{
    (void)objsize; (void)ptr;
    return false;
}

bool __atomic_always_lock_free(unsigned int objsize, const volatile void *ptr)
{
    (void)objsize; (void)ptr;
    return false;
}

#endif /* __arm__ && !__ARM_FEATURE_LDREX */

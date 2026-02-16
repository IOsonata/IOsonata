/**-------------------------------------------------------------------------
@file	bm_compat_atomic.h

@brief	Zephyr atomic API mapped to C17 <stdatomic.h>

Maps Zephyr's atomic API to standard C17 atomics.  No GCC builtins,
no compiler extensions — pure ISO C17.

Zephyr atomic_t is int-based (32-bit).  C17 provides _Atomic qualifier
and <stdatomic.h> functions that compile to LDREX/STREX on Cortex-M.

C++ note: C++23 made <stdatomic.h> available in C++.  For older C++
standards, use <atomic> with std::atomic<int32_t>.  Since sdk-nrf-bm
is C code, this header targets C17 only.

@author	IOsonata
@date	2025

@license MIT
----------------------------------------------------------------------------*/

#ifndef BM_COMPAT_ATOMIC_H__
#define BM_COMPAT_ATOMIC_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Types
 *
 * Zephyr: atomic_t is a plain int used with atomic_set/get/cas/etc.
 * C17:    _Atomic int32_t with atomic_store/load/compare_exchange.
 *
 * We define atomic_t as _Atomic int32_t so C17 operations apply directly.
 * atomic_val_t is the non-atomic value type for parameters/returns.
 * ========================================================================== */

typedef _Atomic int32_t  atomic_t;
typedef int32_t          atomic_val_t;


/* ==========================================================================
 * Static initializers
 * ========================================================================== */

#define ATOMIC_INIT(val)  (val)

/**
 * Define an atomic bitmap of at least @p num_bits bits.
 * Creates atomic_t array with ceil(num_bits / 32) elements.
 */
#define ATOMIC_DEFINE(name, num_bits) \
	atomic_t name[((num_bits) + 31) / 32]


/* ==========================================================================
 * Load / Store
 * ========================================================================== */

static inline void atomic_set(atomic_t *target, atomic_val_t value)
{
	atomic_store_explicit(target, value, memory_order_seq_cst);
}

static inline atomic_val_t atomic_get(const atomic_t *target)
{
	return atomic_load_explicit((atomic_t *)target, memory_order_seq_cst);
}


/* ==========================================================================
 * Arithmetic
 *
 * Zephyr: atomic_inc/dec return OLD value (pre-operation).
 * C17:    atomic_fetch_add/sub also return OLD value.
 * Direct mapping — no adjustment needed.
 * ========================================================================== */

static inline atomic_val_t atomic_inc(atomic_t *target)
{
	return atomic_fetch_add_explicit(target, 1, memory_order_seq_cst);
}

static inline atomic_val_t atomic_dec(atomic_t *target)
{
	return atomic_fetch_sub_explicit(target, 1, memory_order_seq_cst);
}

static inline atomic_val_t atomic_add(atomic_t *target, atomic_val_t value)
{
	return atomic_fetch_add_explicit(target, value, memory_order_seq_cst);
}

static inline atomic_val_t atomic_sub(atomic_t *target, atomic_val_t value)
{
	return atomic_fetch_sub_explicit(target, value, memory_order_seq_cst);
}


/* ==========================================================================
 * Bitwise (whole-word)
 *
 * All return PREVIOUS value (matches both Zephyr and C17).
 * ========================================================================== */

static inline atomic_val_t atomic_or(atomic_t *target, atomic_val_t value)
{
	return atomic_fetch_or_explicit(target, value, memory_order_seq_cst);
}

static inline atomic_val_t atomic_and(atomic_t *target, atomic_val_t value)
{
	return atomic_fetch_and_explicit(target, value, memory_order_seq_cst);
}

static inline atomic_val_t atomic_xor(atomic_t *target, atomic_val_t value)
{
	return atomic_fetch_xor_explicit(target, value, memory_order_seq_cst);
}


/* ==========================================================================
 * Compare-and-swap
 *
 * Zephyr: bool atomic_cas(target, old_value, new_value)
 * C17:    bool atomic_compare_exchange_strong(target, &expected, desired)
 *         Note: C17 updates expected on failure. Zephyr does not.
 * ========================================================================== */

static inline bool atomic_cas(atomic_t *target,
			      atomic_val_t old_value,
			      atomic_val_t new_value)
{
	/* Local copy — C17 may modify expected on failure */
	atomic_val_t expected = old_value;

	return atomic_compare_exchange_strong_explicit(
		target, &expected, new_value,
		memory_order_seq_cst, memory_order_seq_cst);
}


/* ==========================================================================
 * Bit operations (indexed across atomic_t array)
 *
 * No direct C17 equivalent — built on atomic_fetch_or / atomic_fetch_and.
 * target is treated as array of atomic_t; bit index spans full array.
 * ========================================================================== */

static inline void atomic_set_bit(atomic_t *target, int bit)
{
	atomic_fetch_or_explicit(&target[bit / 32],
				 (int32_t)(1u << (bit % 32)),
				 memory_order_seq_cst);
}

static inline void atomic_clear_bit(atomic_t *target, int bit)
{
	atomic_fetch_and_explicit(&target[bit / 32],
				  (int32_t)~(1u << (bit % 32)),
				  memory_order_seq_cst);
}

static inline bool atomic_test_bit(const atomic_t *target, int bit)
{
	int32_t val = atomic_load_explicit((atomic_t *)&target[bit / 32],
					   memory_order_seq_cst);
	return (val & (int32_t)(1u << (bit % 32))) != 0;
}

/**
 * Atomically test and set bit. Returns PREVIOUS value of the bit.
 */
static inline bool atomic_test_and_set_bit(atomic_t *target, int bit)
{
	int32_t mask = (int32_t)(1u << (bit % 32));
	int32_t old = atomic_fetch_or_explicit(&target[bit / 32], mask,
					       memory_order_seq_cst);
	return (old & mask) != 0;
}

/**
 * Atomically test and clear bit. Returns PREVIOUS value of the bit.
 */
static inline bool atomic_test_and_clear_bit(atomic_t *target, int bit)
{
	int32_t mask = (int32_t)(1u << (bit % 32));
	int32_t old = atomic_fetch_and_explicit(&target[bit / 32], ~mask,
						memory_order_seq_cst);
	return (old & mask) != 0;
}


/* ==========================================================================
 * Clear (set to zero)
 * ========================================================================== */

static inline void atomic_clear(atomic_t *target)
{
	atomic_store_explicit(target, 0, memory_order_seq_cst);
}


#ifdef __cplusplus
}
#endif

#endif /* BM_COMPAT_ATOMIC_H__ */

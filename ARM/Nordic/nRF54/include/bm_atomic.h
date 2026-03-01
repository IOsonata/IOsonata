/**-------------------------------------------------------------------------
@file	bm_atomic.h

@brief	Zephyr atomic API — C17 <stdatomic.h> / C++ <atomic>

Maps Zephyr's atomic API to standard atomics.  Compiles to
LDREX/STREX on Cortex-M in both C and C++ translation units.

C path:   _Atomic int32_t  +  <stdatomic.h> functions (C11/C17)
C++ path: std::atomic<int32_t>  +  member functions  (C++11 and later)

Zephyr atomic_t is int-based (32-bit).

@author	IOsonata
@date	2025

@license MIT
----------------------------------------------------------------------------*/

#ifndef BM_ATOMIC_H__
#define BM_ATOMIC_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
/* ======================================================================
 * C++ path — std::atomic<int32_t>
 * ====================================================================== */
#include <atomic>

typedef std::atomic<int32_t>  atomic_t;
typedef int32_t               atomic_val_t;
typedef std::atomic<void *>   atomic_ptr_t;

#define ATOMIC_INIT(val)  {val}

#define ATOMIC_DEFINE(name, num_bits) \
	atomic_t name[((num_bits) + 31) / 32]

#define ATOMIC_BITMAP_SIZE(num_bits)  (((num_bits) + 31) / 32)

/* --- Load / Store --- */

static inline atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{
	return target->exchange(value, std::memory_order_seq_cst);
}

static inline atomic_val_t atomic_get(const atomic_t *target)
{
	return const_cast<atomic_t *>(target)->load(std::memory_order_seq_cst);
}

/* --- Arithmetic --- */

static inline atomic_val_t atomic_inc(atomic_t *target)
{
	return target->fetch_add(1, std::memory_order_seq_cst);
}

static inline atomic_val_t atomic_dec(atomic_t *target)
{
	return target->fetch_sub(1, std::memory_order_seq_cst);
}

static inline atomic_val_t atomic_add(atomic_t *target, atomic_val_t value)
{
	return target->fetch_add(value, std::memory_order_seq_cst);
}

static inline atomic_val_t atomic_sub(atomic_t *target, atomic_val_t value)
{
	return target->fetch_sub(value, std::memory_order_seq_cst);
}

/* --- Bitwise (whole-word) --- */

static inline atomic_val_t atomic_or(atomic_t *target, atomic_val_t value)
{
	return target->fetch_or(value, std::memory_order_seq_cst);
}

static inline atomic_val_t atomic_and(atomic_t *target, atomic_val_t value)
{
	return target->fetch_and(value, std::memory_order_seq_cst);
}

static inline atomic_val_t atomic_xor(atomic_t *target, atomic_val_t value)
{
	return target->fetch_xor(value, std::memory_order_seq_cst);
}

/* --- Compare-and-swap --- */

static inline bool atomic_cas(atomic_t *target,
			      atomic_val_t old_value,
			      atomic_val_t new_value)
{
	atomic_val_t expected = old_value;
	return target->compare_exchange_strong(expected, new_value,
		std::memory_order_seq_cst, std::memory_order_seq_cst);
}

/* --- Bit operations (indexed across atomic_t array) --- */

static inline void atomic_set_bit(atomic_t *target, int bit)
{
	target[bit / 32].fetch_or((int32_t)(1u << (bit % 32)),
				  std::memory_order_seq_cst);
}

static inline void atomic_clear_bit(atomic_t *target, int bit)
{
	target[bit / 32].fetch_and((int32_t)~(1u << (bit % 32)),
				   std::memory_order_seq_cst);
}

static inline bool atomic_test_bit(const atomic_t *target, int bit)
{
	int32_t val = const_cast<atomic_t *>(&target[bit / 32])->load(
		std::memory_order_seq_cst);
	return (val & (int32_t)(1u << (bit % 32))) != 0;
}

static inline bool atomic_test_and_set_bit(atomic_t *target, int bit)
{
	int32_t mask = (int32_t)(1u << (bit % 32));
	int32_t old = target[bit / 32].fetch_or(mask, std::memory_order_seq_cst);
	return (old & mask) != 0;
}

static inline bool atomic_test_and_clear_bit(atomic_t *target, int bit)
{
	int32_t mask = (int32_t)(1u << (bit % 32));
	int32_t old = target[bit / 32].fetch_and(~mask, std::memory_order_seq_cst);
	return (old & mask) != 0;
}

/* --- Pointer atomics --- */

static inline bool atomic_ptr_cas(atomic_ptr_t *target,
				  void *old_value,
				  void *new_value)
{
	void *expected = old_value;
	return target->compare_exchange_strong(expected, new_value,
		std::memory_order_seq_cst, std::memory_order_seq_cst);
}

static inline void *atomic_ptr_get(const atomic_ptr_t *target)
{
	return const_cast<atomic_ptr_t *>(target)->load(std::memory_order_seq_cst);
}

static inline void *atomic_ptr_set(atomic_ptr_t *target, void *value)
{
	return target->exchange(value, std::memory_order_seq_cst);
}

/* --- Clear --- */

static inline void atomic_clear(atomic_t *target)
{
	target->store(0, std::memory_order_seq_cst);
}

#else /* !__cplusplus */
/* ======================================================================
 * C path — _Atomic int32_t  +  <stdatomic.h>
 * ====================================================================== */
#include <stdatomic.h>

typedef _Atomic int32_t  atomic_t;
typedef int32_t          atomic_val_t;

#define ATOMIC_INIT(val)  (val)

#define ATOMIC_DEFINE(name, num_bits) \
	atomic_t name[((num_bits) + 31) / 32]

#define ATOMIC_BITMAP_SIZE(num_bits)  (((num_bits) + 31) / 32)


/* --- Load / Store --- */

static inline atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{
	return atomic_exchange_explicit(target, value, memory_order_seq_cst);
}

static inline atomic_val_t atomic_get(const atomic_t *target)
{
	return atomic_load_explicit((atomic_t *)target, memory_order_seq_cst);
}

/* --- Arithmetic --- */

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

/* --- Bitwise (whole-word) --- */

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

/* --- Compare-and-swap --- */

static inline bool atomic_cas(atomic_t *target,
			      atomic_val_t old_value,
			      atomic_val_t new_value)
{
	atomic_val_t expected = old_value;
	return atomic_compare_exchange_strong_explicit(
		target, &expected, new_value,
		memory_order_seq_cst, memory_order_seq_cst);
}

/* --- Bit operations (indexed across atomic_t array) --- */

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

static inline bool atomic_test_and_set_bit(atomic_t *target, int bit)
{
	int32_t mask = (int32_t)(1u << (bit % 32));
	int32_t old = atomic_fetch_or_explicit(&target[bit / 32], mask,
					       memory_order_seq_cst);
	return (old & mask) != 0;
}

static inline bool atomic_test_and_clear_bit(atomic_t *target, int bit)
{
	int32_t mask = (int32_t)(1u << (bit % 32));
	int32_t old = atomic_fetch_and_explicit(&target[bit / 32], ~mask,
						memory_order_seq_cst);
	return (old & mask) != 0;
}

/* --- Pointer atomics --- */

typedef _Atomic(void *) atomic_ptr_t;

static inline bool atomic_ptr_cas(atomic_ptr_t *target,
				  void *old_value,
				  void *new_value)
{
	void *expected = old_value;
	return atomic_compare_exchange_strong_explicit(
		target, &expected, new_value,
		memory_order_seq_cst, memory_order_seq_cst);
}

static inline void *atomic_ptr_get(const atomic_ptr_t *target)
{
	return atomic_load_explicit((atomic_ptr_t *)target, memory_order_seq_cst);
}

static inline void *atomic_ptr_set(atomic_ptr_t *target, void *value)
{
	return atomic_exchange_explicit(target, value, memory_order_seq_cst);
}

/* --- Clear --- */

static inline void atomic_clear(atomic_t *target)
{
	atomic_store_explicit(target, 0, memory_order_seq_cst);
}

#endif /* __cplusplus */

#endif /* BM_ATOMIC_H__ */

/**-------------------------------------------------------------------------
@file atomic_gcc.h

@brief Lightweight GCC atomic interface for embedded C++ builds.

The standard C++ atomic wrappers retain assertion and formatted-output code
in unoptimized debug builds. GCC's atomic builtins provide the same lock-free
operations directly without a C++ runtime dependency.

@author Hoang Nguyen Hoan
@date Sep. 11, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __ATOMIC_GCC_H__
#define __ATOMIC_GCC_H__

#include <stdint.h>

typedef bool atomic_flag;
typedef bool atomic_bool;
typedef int atomic_int;
typedef uint_fast8_t atomic_uint_fast8_t;
typedef uint_fast32_t atomic_uint_fast32_t;

static_assert(__atomic_always_lock_free(sizeof(atomic_flag), nullptr),
	"atomic_flag must be lock-free");
static_assert(__atomic_always_lock_free(sizeof(atomic_int), nullptr),
	"atomic_int must be lock-free");

#define ATOMIC_FLAG_INIT false

enum memory_order
{
	memory_order_relaxed = __ATOMIC_RELAXED,
	memory_order_consume = __ATOMIC_CONSUME,
	memory_order_acquire = __ATOMIC_ACQUIRE,
	memory_order_release = __ATOMIC_RELEASE,
	memory_order_acq_rel = __ATOMIC_ACQ_REL,
	memory_order_seq_cst = __ATOMIC_SEQ_CST,
};

template <typename T>
static inline __attribute__((always_inline)) T atomic_load(const T *pValue)
{
	return __atomic_load_n(pValue, __ATOMIC_SEQ_CST);
}

template <typename T>
static inline __attribute__((always_inline)) T atomic_load_explicit(
	const T *pValue, memory_order Order)
{
	return __atomic_load_n(pValue, (int)Order);
}

template <typename T, typename V>
static inline __attribute__((always_inline)) void atomic_store(T *pValue,
	V Value)
{
	__atomic_store_n(pValue, (T)Value, __ATOMIC_SEQ_CST);
}

template <typename T, typename V>
static inline __attribute__((always_inline)) void atomic_store_explicit(
	T *pValue, V Value, memory_order Order)
{
	__atomic_store_n(pValue, (T)Value, (int)Order);
}

template <typename T, typename V>
static inline __attribute__((always_inline)) T atomic_exchange(T *pValue,
	V Value)
{
	return __atomic_exchange_n(pValue, (T)Value, __ATOMIC_SEQ_CST);
}

template <typename T, typename V>
static inline __attribute__((always_inline)) T atomic_exchange_explicit(
	T *pValue, V Value, memory_order Order)
{
	return __atomic_exchange_n(pValue, (T)Value, (int)Order);
}

template <typename T, typename V>
static inline __attribute__((always_inline)) T atomic_fetch_add(T *pValue,
	V Value)
{
	return __atomic_fetch_add(pValue, (T)Value, __ATOMIC_SEQ_CST);
}

template <typename T, typename V>
static inline __attribute__((always_inline)) T atomic_fetch_sub(T *pValue,
	V Value)
{
	return __atomic_fetch_sub(pValue, (T)Value, __ATOMIC_SEQ_CST);
}

static inline __attribute__((always_inline)) bool atomic_flag_test_and_set(
	atomic_flag *pFlag)
{
	return __atomic_test_and_set(pFlag, __ATOMIC_SEQ_CST);
}

static inline __attribute__((always_inline)) void atomic_flag_clear(
	atomic_flag *pFlag)
{
	__atomic_clear(pFlag, __ATOMIC_SEQ_CST);
}

#endif

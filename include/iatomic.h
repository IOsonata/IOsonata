/**-------------------------------------------------------------------------
@file	iatomic.h

@brief	Atomic operations.

Because of it's platform dependent nature, this file requires conditional
compilation for each platform port.

Compile macro :
	WIN32             - Windows
	__TCS__           - Trimedia
	__ADSPBLACKFIN__  - ADSP Blackfin

@author	Hoang Nguyen Hoan
@date	Sep. 12, 1996

MIT License

Copyright (c) 1996-2022, I-SYST, all rights reserved

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

----------------------------------------------------------------------------
Modified by         Date           	Description
Hoan				17 nov. 2014	Adapt to GNU GCC
----------------------------------------------------------------------------*/
#ifndef __IATOMIC_H__
#define __IATOMIC_H__

#include <signal.h>

#if defined(_WIN32) || defined(WIN32)
//
// MS Windows
//
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <inttypes.h>
#ifndef __cplusplus
#include <stdbool.h>
#endif

typedef sig_atomic_t		atomic_int;
typedef uint32_t			atomic_uint_fast32_t;
typedef bool				atomic_flag;

#elif defined(__TCS__)
//
// Trimedia/Nexperia
//
//#include "tmlib/AppModel.h"

#elif defined(__ADSPBLACKFIN__)
//
// ADI Blackfin
//
#include <ccblkfn.h>
#elif defined(__GNUC__)
//GCC_VERSION) && GCC_VERSION >= 40700
#ifdef __arm__
#if defined ( __GNUC__ )
#ifndef __ASM
	#define __ASM            __asm                                      /*!< asm keyword for GNU Compiler */
#endif
#ifndef __INLINE
	#define __INLINE         inline                                     /*!< inline keyword for GNU Compiler */
#endif
#ifndef __STATIC_INLINE
	#define __STATIC_INLINE  static inline
#endif
#endif

#include "cmsis_gcc.h"
#endif

#else
#pragma message ("Platform undefined")
#error Platform not implemented

#endif   // Platform definitions

#include "istddef.h"

/**
 * Atomic increment
 *
 * @Param   pVar : Pointer to data value to be increased
 *
 * @Return  Newly incremented value
 */
#if defined(__TSOK__) || defined(__ADSPBLACKFIN__)

sig_atomic_t AtomicInc(sig_atomic_t *pVar);

#else

static inline sig_atomic_t AtomicInc(sig_atomic_t *pVar) {

#if defined(_WIN32) || defined(WIN32)
//
// MS Windows
//
   return InterlockedIncrement((LONG *)pVar);
#elif defined(__TCS__)  // Trimedia
//
// Trimedia
//
	#pragma TCS_atomic
    // AppModel_suspend_scheduling();
   	return ++(*pVar);
   	//   AppModel_resume_scheduling();
#elif defined(__GNUC__)
   	__atomic_store_n(pVar, *pVar + 1, __ATOMIC_SEQ_CST);
    return *pVar;
#else
#error Platform not implemented
#endif

}
#endif

/**
 * Atomic decrement
 *
 * @Param   pVar : Pointer to data value to be decreased
 *
 * @Return  Newly decremented value
 */
#if defined(__TSOK__) || defined(__ADSPBLACKFIN__)

sig_atomic_t AtomicDec(sig_atomic_t *pVar);

#else

static inline sig_atomic_t AtomicDec(sig_atomic_t *pVar) {

#if defined(_WIN32) || defined(WIN32)
//
// MS Windows
//
	return InterlockedDecrement((LONG *)pVar);

#elif defined(__TCS__)
//
// Trimedia
//
	#pragma TCS_atomic
	//   AppModel_suspend_scheduling();
	return --(*pVar);
	//   AppModel_resume_scheduling();
#elif defined(__GNUC__)
   	__atomic_store_n(pVar, *pVar - 1, __ATOMIC_SEQ_CST);
   	return *pVar;
//	return __atomic_fetch_sub (pVar, 1, __ATOMIC_SEQ_CST);
#endif
}
#endif  // __TSOK__

/**
 * Atomic assign value
 *
 * @Param   pVar   : Pointer to data value to be decreased
 * @param   NewVal : New value to be assigned to pVar
 */
#if defined(__TSOK__) || defined(__ADSPBLACKFIN__)

void AtomicAssign(sig_atomic_t *pVar, sig_atomic_t NewVal);

#else

static inline void AtomicAssign(sig_atomic_t *pVar, sig_atomic_t NewVal) {

#if defined(_WIN32) || defined(WIN32)
//
// MS Windows
//
   InterlockedExchange((LONG *)pVar, (LONG)NewVal);

#elif defined(__TCS__)
//
// Trimedia
//
   #pragma TCS_atomic
//   AppModel_suspend_scheduling();
   *pVar = NewVal;
//   AppModel_resume_scheduling();
#elif defined(__GNUC__)
   __atomic_store_n (pVar, NewVal, __ATOMIC_SEQ_CST);
#endif
}

#if defined(_WIN32) || defined(WIN32)
static inline void atomic_store(atomic_int* pVar, sig_atomic_t NewVal) {
	InterlockedExchange(pVar, NewVal);
}

#endif

#endif // __TSOK__

/**
 * Atomic exchange value
 *
 * @Param   pVar   : Pointer to data value to be decreased
 * @param   NewVal : New value to be assigned to pVar
 */
#if defined(__TSOK__) || defined(__ADSPBLACKFIN__)

sig_atomic_t AtomicExchange(sig_atomic_t *pVar, sig_atomic_t NewVal);

#else

static inline sig_atomic_t AtomicExchange(sig_atomic_t *pVar, sig_atomic_t NewVal) {

#if defined(_WIN32) || defined(WIN32)

#elif defined(__TCS__)

#elif defined(__GNUC__)
   return __atomic_exchange_n(pVar, NewVal, __ATOMIC_SEQ_CST);
#endif
}
#endif // __TSOK__

/**
 * Atomic test and set value
 *
 * @Param   pVar   : Pointer to data value to be decreased
 */
#if defined(__TSOK__) || defined(__ADSPBLACKFIN__)

sig_atomic_t AtomicExchange(sig_atomic_t *pVar, sig_atomic_t NewVal);

#else

static inline bool AtomicTestAndSet(void *pVar) {

#if defined(_WIN32) || defined(WIN32)
	return InterlockedCompareExchange((LONG *)pVar, (LONG)true, true) != 0;

#elif defined(__TCS__)

#elif defined(__GNUC__)
   return __atomic_test_and_set(pVar, __ATOMIC_SEQ_CST);
#endif
}

#define atomic_flag_test_and_set	AtomicTestAndSet
static inline void atomic_flag_clear(atomic_flag *pVar) {
	InterlockedExchange((LONG*)pVar, (LONG)0);
}
#endif // __TSOK__

/**
 * Atomic clear
 *
 * @Param   pVar   : Pointer to data value to be decreased
 */
#if defined(__TSOK__) || defined(__ADSPBLACKFIN__)

sig_atomic_t AtomicExchange(sig_atomic_t *pVar, sig_atomic_t NewVal);

#else

static inline void AtomicClear(void *pVar) {

#if defined(_WIN32) || defined(WIN32)
	InterlockedExchange((LONG*)pVar, (LONG)0);
#elif defined(__TCS__)

#elif defined(__GNUC__)
   return __atomic_clear(pVar, __ATOMIC_SEQ_CST);
#endif
}
#endif // __TSOK__

#if defined(_WIN32) || defined(WIN32)
#else
static inline uint32_t EnterCriticalSection(void) {
#ifdef __arm__
	uint32_t __state = __get_PRIMASK();
	__disable_irq();
	return __state;
#else
    return 0;
#endif
}

static inline void ExitCriticalSection(uint32_t State) {
#ifdef __arm__
	__set_PRIMASK(State);
#endif
}
#endif

#if 0
#ifdef __arm__
static inline uint32_t DisableInterrupt() {
	uint32_t __primmask = __get_PRIMASK();
	__disable_irq();
	return __primmask;
}

static inline void EnableInterrupt(uint32_t __primmask) {
	__set_PRIMASK(__primmask);
}
#endif
#endif

#endif // __IATOMIC_H__





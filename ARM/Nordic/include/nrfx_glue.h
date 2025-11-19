/**
 * Copyright (c) 2017 - 2024, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NRFX_GLUE_H__
#define NRFX_GLUE_H__

#ifdef __cplusplus
extern "C" {
#endif

// Fixed declaration bug in Nordic nrfx 4.0
#ifndef NRF_STATIC_INLINE
#define NRF_STATIC_INLINE __STATIC_INLINE
#endif

/**
 * @defgroup nrfx_glue nrfx_glue.h
 * @{
 * @ingroup nrfx
 *
 * @brief This file contains macros that should be implemented according to
 *        the needs of the host environment into which @em nrfx is integrated.
 */

// Uncomment this line to use the standard MDK way of binding IRQ handlers
// at linking time.
#include <soc/nrfx_irqs.h>

//------------------------------------------------------------------------------
#include <assert.h>
/**
 * @brief Macro for placing a runtime assertion.
 *
 * @param expression Expression to be evaluated.
 */
#define NRFX_ASSERT(expression)		//assert(expression)

/**
 * @brief Macro for placing a compile time assertion.
 *
 * @param expression Expression to be evaluated.
 */
#define NRFX_STATIC_ASSERT(expression)	_Static_assert(expression, "assert")

//------------------------------------------------------------------------------
#ifdef NRF51
#ifdef SOFTDEVICE_PRESENT
#define INTERRUPT_PRIORITY_IS_VALID(pri) (((pri) == 1) || ((pri) == 3))
#else
#define INTERRUPT_PRIORITY_IS_VALID(pri) ((pri) < 4)
#endif //SOFTDEVICE_PRESENT
#else
#ifdef SOFTDEVICE_PRESENT
#define INTERRUPT_PRIORITY_IS_VALID(pri) ((((pri) > 1) && ((pri) < 4)) || \
                                          (((pri) > 4) && ((pri) < 8)))
#else
#define INTERRUPT_PRIORITY_IS_VALID(pri) ((pri) < 8)
#endif //SOFTDEVICE_PRESENT
#endif //NRF52

/**
 * @brief Macro for setting the priority of a specific IRQ.
 *
 * @param irq_number IRQ number.
 * @param priority   Priority to be set.
 */
#define NRFX_IRQ_PRIORITY_SET(irq_number, priority)	\
	_NRFX_IRQ_PRIORITY_SET(irq_number, priority)
static inline void _NRFX_IRQ_PRIORITY_SET(IRQn_Type irq_number,
										  uint8_t   priority)
{
	NRFX_ASSERT(INTERRUPT_PRIORITY_IS_VALID(priority));
	NVIC_SetPriority(irq_number, priority);
}

/**
 * @brief Macro for enabling a specific IRQ.
 *
 * @param irq_number IRQ number.
 */
#define NRFX_IRQ_ENABLE(irq_number)	NVIC_EnableIRQ(irq_number)

/**
 * @brief Macro for checking if a specific IRQ is enabled.
 *
 * @param irq_number IRQ number.
 *
 * @retval true  If the IRQ is enabled.
 * @retval false Otherwise.
 */
//#define NRFX_IRQ_IS_ENABLED(irq_number)
static inline bool NRFX_IRQ_IS_ENABLED(IRQn_Type irq_number)
{
    return 0 != (NVIC->ISER[irq_number / 32] & (1UL << (irq_number % 32)));
}


/**
 * @brief Macro for disabling a specific IRQ.
 *
 * @param irq_number IRQ number.
 */
#define NRFX_IRQ_DISABLE(irq_number)	NVIC_DisableIRQ(irq_number)

/**
 * @brief Macro for setting a specific IRQ as pending.
 *
 * @param irq_number IRQ number.
 */
#define NRFX_IRQ_PENDING_SET(irq_number)	NVIC_SetPendingIRQ(irq_number)

/**
 * @brief Macro for clearing the pending status of a specific IRQ.
 *
 * @param irq_number IRQ number.
 */
#define NRFX_IRQ_PENDING_CLEAR(irq_number)	NVIC_ClearPendingIRQ(irq_number)

/**
 * @brief Macro for checking the pending status of a specific IRQ.
 *
 * @retval true  If the IRQ is pending.
 * @retval false Otherwise.
 */
//#define NRFX_IRQ_IS_PENDING(irq_number)
static inline bool NRFX_IRQ_IS_PENDING(IRQn_Type irq_number)
{
    return (NVIC_GetPendingIRQ(irq_number) == 1);
}

/** @brief Macro for entering into a critical section. */
//#define NRFX_CRITICAL_SECTION_ENTER()
#if defined(SOFTDEVICE_PRESENT)
#include "nrf_nvic.h"
#define NRFX_CRITICAL_SECTION_ENTER()	\
	{\
		uint8_t __CR_NESTED = 0;	\
		(void) sd_nvic_critical_region_enter(&__CR_NESTED);
#else
#include "coredev/interrupt.h"
#define NRFX_CRITICAL_SECTION_ENTER()	\
	{\
		uint32_t state = DisableInterrupt();
#endif

/** @brief Macro for exiting from a critical section. */
//#define NRFX_CRITICAL_SECTION_EXIT()
#if defined(SOFTDEVICE_PRESENT)
#define NRFX_CRITICAL_SECTION_EXIT()	\
	    (void) sd_nvic_critical_region_exit(__CR_NESTED);	\
	}
#else
#define NRFX_CRITICAL_SECTION_EXIT()	\
		EnableInterrupt(state);	\
	}
#endif


//------------------------------------------------------------------------------

/**
 * @brief When set to a non-zero value, this macro specifies that
 *        @ref nrfx_coredep_delay_us uses a precise DWT-based solution.
 *        A compilation error is generated if the DWT unit is not present
 *        in the SoC used.
 */
#define NRFX_DELAY_DWT_BASED    0

/**
 * @brief Macro for delaying the code execution for at least the specified time.
 *
 * @param us_time Number of microseconds to wait.
 */
#include <lib/nrfx_coredep.h>

#define NRFX_DELAY_US(us_time)	nrfx_coredep_delay_us(us_time)

//------------------------------------------------------------------------------
#include <lib/nrfx_atomic.h>

/** @brief Atomic 32-bit unsigned type. */
#define nrfx_atomic_t	nrfx_atomic_u32_t

/**
 * @brief Macro for storing a value to an atomic object and returning its previous value.
 *
 * @param[in] p_data Atomic memory pointer.
 * @param[in] value  Value to store.
 *
 * @return Previous value of the atomic object.
 */
#define NRFX_ATOMIC_FETCH_STORE(p_data, value)	nrfx_atomic_u32_fetch_store(p_data, value)

/**
 * @brief Macro for running a bitwise OR operation on an atomic object and returning its previous value.
 *
 * @param[in] p_data Atomic memory pointer.
 * @param[in] value  Value of the second operand in the OR operation.
 *
 * @return Previous value of the atomic object.
 */
#define NRFX_ATOMIC_FETCH_OR(p_data, value)	nrfx_atomic_u32_fetch_or(p_data, value)

/**
 * @brief Macro for running a bitwise AND operation on an atomic object
 *        and returning its previous value.
 *
 * @param[in] p_data Atomic memory pointer.
 * @param[in] value  Value of the second operand in the AND operation.
 *
 * @return Previous value of the atomic object.
 */
#define NRFX_ATOMIC_FETCH_AND(p_data, value)	nrfx_atomic_u32_fetch_and(p_data, value)

/**
 * @brief Macro for running a bitwise XOR operation on an atomic object
 *        and returning its previous value.
 *
 * @param[in] p_data Atomic memory pointer.
 * @param[in] value  Value of the second operand in the XOR operation.
 *
 * @return Previous value of the atomic object.
 */
#define NRFX_ATOMIC_FETCH_XOR(p_data, value)	nrfx_atomic_u32_fetch_xor(p_data, value)

/**
 * @brief Macro for running an addition operation on an atomic object
 *        and returning its previous value.
 *
 * @param[in] p_data Atomic memory pointer.
 * @param[in] value  Value of the second operand in the ADD operation.
 *
 * @return Previous value of the atomic object.
 */
#define NRFX_ATOMIC_FETCH_ADD(p_data, value)	nrfx_atomic_u32_fetch_add(p_data, value)

/**
 * @brief Macro for running a subtraction operation on an atomic object
 *        and returning its previous value.
 *
 * @param[in] p_data Atomic memory pointer.
 * @param[in] value  Value of the second operand in the SUB operation.
 *
 * @return Previous value of the atomic object.
 */
#define NRFX_ATOMIC_FETCH_SUB(p_data, value)	nrfx_atomic_u32_fetch_sub(p_data, value)

/**
 * @brief Macro for running compare and swap on an atomic object.
 *
 * Value is updated to the new value only if it previously equaled old value.
 *
 * @param[in,out] p_data    Atomic memory pointer.
 * @param[in]     old_value Expected old value.
 * @param[in]     new_value New value.
 *
 * @retval true  If value was updated.
 * @retval false If value was not updated because location was not equal to @p old_value.
 */
#define NRFX_ATOMIC_CAS(p_data, old_value, new_value) nrfx_atomic_u32_cmp_exch(p_data, old_value, new_value)

/**
 * @brief Macro for counting leading zeros.
 *
 * @param[in] value A word value.
 *
 * @return Number of leading 0-bits in @p value, starting at the most significant bit position.
 *         If x is 0, the result is undefined.
 */
//#define NRFX_CLZ(value)

/**
 * @brief Macro for counting trailing zeros.
 *
 * @param[in] value A word value.
 *
 * @return Number of trailing 0-bits in @p value, starting at the least significant bit position.
 *         If x is 0, the result is undefined.
 */
//#define NRFX_CTZ(value)

//------------------------------------------------------------------------------

//#ifndef NRFXLIB_SDC
//#ifdef NRF52_SERIES
#if defined(SOFTDEVICE_PRESENT)
/**
 * @brief When set to a non-zero value, this macro specifies that the
 *        @ref nrfx_error_codes and the @ref nrfx_err_t type itself are defined
 *        in a customized way and the default definitions from @c <nrfx_error.h>
 *        should not be used.
 */
#define NRFX_CUSTOM_ERROR_CODES 1
#endif
//------------------------------------------------------------------------------

/**
 * @brief When set to a non-zero value, this macro specifies that inside HALs
 *        the event registers are read back after clearing, on devices that
 *        otherwise could defer the actual register modification.
 */
#define NRFX_EVENT_READBACK_ENABLED 1

//------------------------------------------------------------------------------

/**
 * @brief Macro for writing back cache lines associated with the specified buffer.
 *
 * @note Macro should be empty if data cache is disabled or not present.
 *
 * @param[in] p_buffer Pointer to the buffer.
 * @param[in] size     Size of the buffer.
 */
//#define NRFY_CACHE_WB(p_buffer, size)

/**
 * @brief Macro for invalidating cache lines associated with the specified buffer.
 *
 * @note Macro should be empty if data cache is disabled or not present.
 *
 * @param[in] p_buffer Pointer to the buffer.
 * @param[in] size     Size of the buffer.
 */
//#define NRFY_CACHE_INV(p_buffer, size)

/**
 * @brief Macro for writing back and invalidating cache lines associated with
 *        the specified buffer.
 *
 * @note Macro should be empty if data cache is disabled or not present.
 *
 * @param[in] p_buffer Pointer to the buffer.
 * @param[in] size     Size of the buffer.
 */
//#define NRFY_CACHE_WBINV(p_buffer, size)

#if NRFX_CUSTOM_ERROR_CODES == 1

#include <sdk_errors.h>
/**
 * @brief When set to a non-zero value, this macro specifies that the
 *        @ref nrfx_error_codes and the @ref ret_code_t type itself are defined
 *        in a customized way and the default definitions from @c <nrfx_error.h>
 *        should not be used.
 */
#define NRFX_CUSTOM_ERROR_CODES 1

typedef ret_code_t nrfx_err_t;

#define NRFX_SUCCESS                    NRF_SUCCESS
#define NRFX_ERROR_INTERNAL             NRF_ERROR_INTERNAL
#define NRFX_ERROR_NO_MEM               NRF_ERROR_NO_MEM
#define NRFX_ERROR_NOT_SUPPORTED        NRF_ERROR_NOT_SUPPORTED
#define NRFX_ERROR_INVALID_PARAM        NRF_ERROR_INVALID_PARAM
#define NRFX_ERROR_INVALID_STATE        NRF_ERROR_INVALID_STATE
#define NRFX_ERROR_INVALID_LENGTH       NRF_ERROR_INVALID_LENGTH
#define NRFX_ERROR_TIMEOUT              NRF_ERROR_TIMEOUT
#define NRFX_ERROR_FORBIDDEN            NRF_ERROR_FORBIDDEN
#define NRFX_ERROR_NULL                 NRF_ERROR_NULL
#define NRFX_ERROR_INVALID_ADDR         NRF_ERROR_INVALID_ADDR
#define NRFX_ERROR_BUSY                 NRF_ERROR_BUSY
#define NRFX_ERROR_ALREADY_INITIALIZED  NRF_ERROR_MODULE_ALREADY_INITIALIZED
#define NRFX_ERROR_ALREADY				NRF_ERROR_MODULE_ALREADY_INITIALIZED

#define NRFX_ERROR_DRV_TWI_ERR_OVERRUN  NRF_ERROR_DRV_TWI_ERR_OVERRUN
#define NRFX_ERROR_DRV_TWI_ERR_ANACK    NRF_ERROR_DRV_TWI_ERR_ANACK
#define NRFX_ERROR_DRV_TWI_ERR_DNACK    NRF_ERROR_DRV_TWI_ERR_DNACK
#else
typedef uint32_t 	ret_code_t;
#endif // NRFX_CUSTOM_ERROR_CODES


//------------------------------------------------------------------------------

/** @brief Bitmask that defines DPPI channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI_CHANNELS_USED   NRF_DPPI_CHANNELS_USED

/** @brief Bitmask that defines DPPI groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI_GROUPS_USED     NRF_DPPI_GROUPS_USED

/** @brief Bitmask that defines PPI channels that are reserved for use outside of the nrfx library. */
#define NRFX_PPI_CHANNELS_USED    NRF_PPI_CHANNELS_USED

/** @brief Bitmask that defines PPI groups that are reserved for use outside of the nrfx library. */
#define NRFX_PPI_GROUPS_USED      NRF_PPI_GROUPS_USED

/** @brief Bitmask that defines GPIOTE channels that are reserved for use outside of the nrfx library. */
#define NRFX_GPIOTE_CHANNELS_USED 0

/**
 * @brief Bitmask defining SWI instances reserved to be used outside of nrfx.
 */
#define NRFX_SWI_USED           NRF_SWI_USED

/** @brief Bitmask that defines EGU instances that are reserved for use outside of the nrfx library. */
#define NRFX_EGUS_USED            0

/** @brief Bitmask that defines TIMER instances that are reserved for use outside of the nrfx library. */
#define NRFX_TIMERS_USED          NRF_TIMERS_USED


/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRFX_GLUE_H__

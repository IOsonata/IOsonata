/**-------------------------------------------------------------------------
@file	stm32wbaxx_hal.h

@brief	HAL names used by the STM32CubeWBA BLE platform files.

The STM32CubeWBA files built into the STM32WBA65 library, linklayer_plat.c and
ll_sys_if.c (through main.h), include stm32wbaxx_hal.h by name. IOsonata does
not use the ST HAL. This header gives those files the few HAL and LL items
they use, with CMSIS register access:

- HAL_NVIC_* and HAL_RCCEx_Enable/DisableRadioBBClock (bt_wba_platform.cpp)
- the AHB5 clock macros of the 2.4 GHz RADIO
- the PWR radio mode and voltage scaling flag
- the unique device number and ST company ID (flash engineering bytes)
- the DBGMCU revision ID
- UNUSED

The RCC radio sleep timer and HSE items are in stm32wbaxx_ll_rcc.h, the
assert_param macro is in stm32wbaxx_hal_conf.h.

This folder replaces the HAL headers of the same names. A project that builds
the ST HAL must not have it in its include path.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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
#ifndef __STM32WBAXX_HAL_H__
#define __STM32WBAXX_HAL_H__

#include <stdint.h>

#include "stm32wbaxx.h"
#include "stm32wbaxx_hal_conf.h"
#include "stm32wbaxx_ll_rcc.h"

#ifndef UNUSED
#define UNUSED(X)		(void)(X)
#endif

// AHB5 bus clock of the 2.4 GHz RADIO. The read back makes the next radio
// register access wait until the clock is running.
#define __HAL_RCC_RADIO_CLK_ENABLE()	do { \
		volatile uint32_t tmpreg; \
		SET_BIT(RCC->AHB5ENR, RCC_AHB5ENR_RADIOEN); \
		tmpreg = READ_BIT(RCC->AHB5ENR, RCC_AHB5ENR_RADIOEN); \
		(void)tmpreg; \
	} while (0)

// AHB5 RADIO clock in sleep mode
#define __HAL_RCC_RADIO_CLK_SLEEP_ENABLE()		SET_BIT(RCC->AHB5SMENR, RCC_AHB5SMENR_RADIOSMEN)
#define __HAL_RCC_RADIO_CLK_SLEEP_DISABLE()		CLEAR_BIT(RCC->AHB5SMENR, RCC_AHB5SMENR_RADIOSMEN)
#define __HAL_RCC_RADIO_IS_CLK_SLEEP_ENABLED()	(READ_BIT(RCC->AHB5SMENR, RCC_AHB5SMENR_RADIOSMEN) != 0U)

// 2.4 GHz RADIO operating modes, PWR_RADIOSCR MODE field
#define LL_PWR_RADIO_DEEP_SLEEP_MODE	0U
#define LL_PWR_RADIO_SLEEP_MODE			PWR_RADIOSCR_MODE_0
#define LL_PWR_RADIO_ACTIVE_MODE		PWR_RADIOSCR_MODE_1

#ifdef __cplusplus
extern "C" {
#endif

// Defined in bt_wba_platform.cpp with the CMSIS NVIC functions and RCC access
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_RCCEx_EnableRadioBBClock(void);
void HAL_RCCEx_DisableRadioBBClock(void);

// 2.4 GHz RADIO operating mode. Active when MODE bit 1 is set, otherwise
// sleep or deep sleep from MODE bit 0.
static inline uint32_t LL_PWR_GetRadioMode(void)
{
	if (READ_BIT(PWR->RADIOSCR, PWR_RADIOSCR_MODE_1) != 0UL)
	{
		return LL_PWR_RADIO_ACTIVE_MODE;
	}

	return READ_BIT(PWR->RADIOSCR, PWR_RADIOSCR_MODE_0);
}

// 1 when the regulator has reached the selected voltage scaling range
static inline uint32_t LL_PWR_IsActiveFlag_VOS(void)
{
	return (READ_BIT(PWR->VOSR, PWR_VOSR_VOSRDY) == PWR_VOSR_VOSRDY) ? 1UL : 0UL;
}

// Unique device number, first word of the 64-bit unique device ID
static inline uint32_t LL_FLASH_GetUDN(void)
{
	return READ_REG(*((uint32_t *)UID64_BASE));
}

// ST company ID, bits 31:8 of the second word of the 64-bit unique device ID
static inline uint32_t LL_FLASH_GetSTCompanyID(void)
{
	return (READ_REG(*((uint32_t *)UID64_BASE + 1U)) >> 8U) & 0x00FFFFFFU;
}

// Device revision ID, DBGMCU_IDCODE REV_ID
static inline uint32_t LL_DBGMCU_GetRevisionID(void)
{
	return READ_BIT(DBGMCU->IDCODE, DBGMCU_IDCODE_REV_ID) >> DBGMCU_IDCODE_REV_ID_Pos;
}

#ifdef __cplusplus
}
#endif

#endif // __STM32WBAXX_HAL_H__

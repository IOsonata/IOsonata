/**-------------------------------------------------------------------------
@file	stm32.h

@brief	Common STM32 CMSIS device-header selector.

		Include this file from STM32 port code that needs direct register access.
		It selects the correct ST CMSIS family header from the MCU macro supplied
		by the build.

		This is not a HAL header. It only selects the CMSIS register definitions.

		Supported IOsonata STM32 families:
			STM32F0xx
			STM32F4xx
			STM32L4xx / STM32L4+
			STM32WBxx
			STM32WBAxx

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __IOSONATA_STM32_H__
#define __IOSONATA_STM32_H__

#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F031x6) || defined(STM32F038xx) || \
	defined(STM32F042x6) || defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx) || defined(STM32F0)
#define IOSONATA_STM32_F0		1
#include "stm32f0xx.h"

#elif defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
	defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
	defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F410Tx) || defined(STM32F410Cx) || \
	defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx) || defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || defined(STM32F4)
#define IOSONATA_STM32_F4		1
#include "stm32f4xx.h"

#elif defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
	defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
	defined(STM32L452xx) || defined(STM32L462xx) || defined(STM32L471xx) || defined(STM32L475xx) || \
	defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx) || defined(STM32L496xx) || \
	defined(STM32L4A6xx) || defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4R5xx) || \
	defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || \
	defined(STM32L4S9xx) || defined(STM32L4)
#define IOSONATA_STM32_L4		1
#include "stm32l4xx.h"

#elif defined(STM32WB10xx) || defined(STM32WB15xx) || defined(STM32WB30xx) || defined(STM32WB35xx) || \
	defined(STM32WB50xx) || defined(STM32WB55xx) || defined(STM32WB5Mxx) || defined(STM32WB)
#define IOSONATA_STM32_WB		1
#include "stm32wbxx.h"

#elif defined(STM32WBA) || defined(STM32WBA50xx) || defined(STM32WBA52xx) || defined(STM32WBA54xx) || \
	defined(STM32WBA55xx) || defined(STM32WBA5Mxx) || defined(STM32WBA6xx)
#define IOSONATA_STM32_WBA		1
#include "stm32wbaxx.h"

#else
#error "stm32.h: unsupported STM32 family or missing STM32 device macro"
#endif

#if defined(RNG) && defined(RNG_CR_RNGEN) && defined(RNG_SR_DRDY)
#define IOSONATA_STM32_HAS_RNG		1
#else
#define IOSONATA_STM32_HAS_RNG		0
#endif

#endif // __IOSONATA_STM32_H__

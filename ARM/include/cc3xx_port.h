/**-------------------------------------------------------------------------
@file	cc3xx_port.h

@brief	Vendor integration interface for the generic Arm CC3xx driver.

		Declares the minimum target-specific hooks required before and after
		accessing a compatible Arm CC3xx crypto engine. The selected target
		library provides exactly one implementation.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CC3XX_PORT_H__
#define __CC3XX_PORT_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Enable target-specific access to the CC3xx engine.
 *
 * @return	true when the engine wrapper is enabled
 */
bool Cc3xxPortEnable(void);

/**
 * @brief	Disable target-specific access to the CC3xx engine.
 */
void Cc3xxPortDisable(void);

#ifdef __cplusplus
}
#endif

#endif // __CC3XX_PORT_H__

/**-------------------------------------------------------------------------
@file	cc3xx_nrfx.h

@brief	Nordic nrfx port for Arm CC3xx peripheral wrapper control.

		Provides the Nordic-specific power wrapper functions used by the
		portable CC3xx driver. Nordic device selection remains in the target
		library build through the selected device header. This file does not
		select a CC3xx revision or define the CC3xx register map.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CC3XX_NRFX_H__
#define __CC3XX_NRFX_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Enable the Nordic CC3xx peripheral wrapper.
 *
 * @return	true when the wrapper is enabled
 */
bool Cc3xxNrfxEnable(void);

/**
 * @brief	Disable the Nordic CC3xx peripheral wrapper.
 */
void Cc3xxNrfxDisable(void);

#ifdef __cplusplus
}
#endif

#endif // __CC3XX_NRFX_H__

/**-------------------------------------------------------------------------
 @file	cc3xx_nrfx.h

 @brief	Nordic nrfx port for the generic Arm CC3xx driver

 @author	Hoang Nguyen Hoan
 @date	Jul 2026

 @license MIT, (c) 2026 I-SYST
 ----------------------------------------------------------------------------*/
#ifndef __CC3XX_NRFX_H__
#define __CC3XX_NRFX_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Enable the CC3xx peripheral wrapper for the selected Nordic target.
 *
 * The target MCU library supplies the Nordic device header and selects the
 * appropriate CryptoCell/CC3xx instance at build time.
 */
bool Cc3xxNrfxEnable(void);

/** Disable the CC3xx peripheral wrapper. */
void Cc3xxNrfxDisable(void);

/** Return the CC3xx register block base address for the selected target. */
uintptr_t Cc3xxNrfxBaseAddress(void);

#ifdef __cplusplus
}
#endif

#endif // __CC3XX_NRFX_H__

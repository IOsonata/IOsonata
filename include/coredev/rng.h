/**-------------------------------------------------------------------------
@file	rng.h

@brief	Hardware random number generator interface.

Platform RNG driver API. The implementation is per-MCU (e.g. rng_nrfx.c on
Nordic, driving the RNG / CRACEN peripheral). Consumers include this header to
obtain the declaration - linkage comes from here, not from local forward
declarations.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __RNG_H__
#define __RNG_H__

#include <stdint.h>
#include <stddef.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialise the hardware RNG peripheral.
 *
 * @return	true on success.
 */
bool RngInit(void);

/**
 * @brief	Fill a buffer with hardware-generated random bytes.
 *
 * @param	pBuff	Destination buffer.
 * @param	Len		Number of bytes.
 *
 * @return	true on success.
 */
bool RngGet(uint8_t *pBuff, size_t Len);

#ifdef __cplusplus
}
#endif

#endif // __RNG_H__

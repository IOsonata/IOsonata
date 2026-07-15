/**-------------------------------------------------------------------------
@file	cracen.h

@brief	Bare-metal CRACEN ownership helpers for nRF54.

		CRACEN is shared by the random-number driver and the hardware crypto
		provider. These helpers provide a short non-blocking lock so only one
		IOsonata path programs CRACEN at a time.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __CRACEN_H__
#define __CRACEN_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool CracenTryAcquire(void);
void CracenRelease(void);

#ifdef __cplusplus
}
#endif

#endif // __CRACEN_H__

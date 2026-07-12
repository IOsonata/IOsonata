/**-------------------------------------------------------------------------
@file	cc3xx_nrfx.c

@brief	Nordic nrfx integration for Arm CC3xx peripheral wrapper control.

		Implements the generic CC3xx port enable and disable hooks with the
		CryptoCell wrapper selected by the Nordic MDK. Verified with nrfx
		commit d58575ae3c27748cabdfaa1ea2b8a386e6425d75 and NCS v3.3.0
		hal_nordic commit 1acb428a205bad58f3dfd4e38f2d1663bb784ba1.
		Re-check the wrapper symbols when either dependency is updated.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include "nrf.h"
#include "cc3xx_port.h"
#include "cc3xx_nrfx.h"

bool Cc3xxPortEnable(void)
{
	CC3XX_NRFX_WRAPPER->ENABLE = 1;

	return CC3XX_NRFX_WRAPPER->ENABLE != 0;
}

void Cc3xxPortDisable(void)
{
	CC3XX_NRFX_WRAPPER->ENABLE = 0;
}

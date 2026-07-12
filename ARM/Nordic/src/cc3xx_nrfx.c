/**-------------------------------------------------------------------------
@file	cc3xx_nrfx.c

@brief	Nordic nrfx port for Arm CC3xx peripheral wrapper control.

		Provides the Nordic-specific CC3xx wrapper enable and disable functions.
		The selected Nordic device header supplies the wrapper definition. The
		target library selects this source only for a compatible CryptoCell
		device.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include "nrf.h"
#include "cc3xx_nrfx.h"

#if defined(NRF_CRYPTOCELL)
#define CC3XX_NRFX_WRAPPER	NRF_CRYPTOCELL
#elif defined(NRF_CRYPTOCELL_S)
#define CC3XX_NRFX_WRAPPER	NRF_CRYPTOCELL_S
#else
#error CC3xx wrapper is not defined for the selected Nordic target
#endif

bool Cc3xxNrfxEnable(void)
{
	CC3XX_NRFX_WRAPPER->ENABLE = 1;

	return CC3XX_NRFX_WRAPPER->ENABLE != 0;
}

void Cc3xxNrfxDisable(void)
{
	CC3XX_NRFX_WRAPPER->ENABLE = 0;
}

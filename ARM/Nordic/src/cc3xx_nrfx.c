/**-------------------------------------------------------------------------
@file	cc3xx_nrfx.c

@brief	Nordic nrfx port for Arm CC3xx peripheral wrapper control.

		Provides the Nordic-specific CC3xx wrapper enable and disable functions.
		The selected Nordic device header supplies NRF_CRYPTOCELL for the target
		MCU. CC3xx core register addressing and revision selection remain in the
		portable driver configuration.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include "nrf.h"
#include "cc3xx_nrfx.h"

bool Cc3xxNrfxEnable(void)
{
	NRF_CRYPTOCELL->ENABLE = 1;

	return NRF_CRYPTOCELL->ENABLE != 0;
}

void Cc3xxNrfxDisable(void)
{
	NRF_CRYPTOCELL->ENABLE = 0;
}

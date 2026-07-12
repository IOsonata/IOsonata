/**-------------------------------------------------------------------------
@file	cc3xx_nrfx.h

@brief	Nordic nrfx integration definitions for the Arm CC3xx driver.

		Selects the CryptoCell wrapper symbol supplied by the Nordic MDK. The
		selected target library includes this header only for a compatible
		CryptoCell device. CC3xx core registers and feature selection remain
		outside this vendor integration header.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CC3XX_NRFX_H__
#define __CC3XX_NRFX_H__

#if defined(NRF_CRYPTOCELL)
#define CC3XX_NRFX_WRAPPER	NRF_CRYPTOCELL
#elif defined(NRF_CRYPTOCELL_S)
#define CC3XX_NRFX_WRAPPER	NRF_CRYPTOCELL_S
#else
#error CC3xx wrapper is not defined for the selected Nordic target
#endif

#endif // __CC3XX_NRFX_H__

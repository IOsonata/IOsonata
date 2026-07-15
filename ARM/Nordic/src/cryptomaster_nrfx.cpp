/**-------------------------------------------------------------------------
@file	cryptomaster_nrfx.cpp

@brief	nRF54 platform binding for the Silex CryptoMaster AES engine.

		Supplies the target pieces the generic engine (cryptomaster.cpp) reaches
		through cryptomaster.h: the block base address, the wrapper module power,
		and the shared engine lock. This is the only CryptoMaster file that names
		the vendor wrapper.

		The block is shared with the random number driver on this family, so the
		lock defined here is the one the RNG also takes. It is a non-recursive
		test and set under an interrupt mask, sized for main and interrupt
		context on a single core.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

#include "nrf.h"

#include "crypto/cryptomaster.h"

#if !defined(NRF54L15_XXAA) && !defined(NRF54H20_XXAA)
#error "cryptomaster_nrf.cpp is only for nRF54 targets that carry CRACEN"
#endif

volatile void *CryptoMasterBase(void)
{
	return (volatile void *)NRF_CRACENCORE;
}

void CryptoMasterModuleEnable(void)
{
	NRF_CRACEN->ENABLE |= CRACEN_ENABLE_CRYPTOMASTER_Msk;
}

void CryptoMasterModuleDisable(void)
{
	NRF_CRACEN->ENABLE &= ~CRACEN_ENABLE_CRYPTOMASTER_Msk;
}

// Shared engine lock. Non-recursive test and set under an interrupt mask. The
// random number driver takes the same lock, so a crypto operation and an RNG
// draw serialize against each other on this block.
static volatile bool s_CmBusy;

bool CryptoMasterTryAcquire(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	bool acquired = !s_CmBusy;
	if (acquired)
	{
		s_CmBusy = true;
		__DMB();
	}

	if (primask == 0)
	{
		__enable_irq();
	}

	return acquired;
}

void CryptoMasterRelease(void)
{
	__DMB();
	s_CmBusy = false;
}

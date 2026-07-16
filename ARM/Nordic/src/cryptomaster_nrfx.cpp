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

#include "cracen_intrf.h"
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
	// The CRACEN interface enabled the symmetric module when the transaction
	// was started (CryptoMasterTryAcquire -> StartTx). Nothing to do here.
}

void CryptoMasterModuleDisable(void)
{
	// The module is disabled when the transaction is released
	// (CryptoMasterRelease -> StopTx). Nothing to do here.
}

// Acquire and release go through the CRACEN interface. StartTx takes the
// interface busy flag (mutual exclusion against the other engines) and enables
// the symmetric module; StopTx disables it and clears the busy flag. There is
// no private lock: the interface owns the shared enable and the serialization.
bool CryptoMasterTryAcquire(void)
{
	return CracenIntrfInstance()->ModuleHold(CRACEN_MODULE_CRYPTOMASTER);
}

void CryptoMasterRelease(void)
{
	CracenIntrfInstance()->ModuleRelease();
}

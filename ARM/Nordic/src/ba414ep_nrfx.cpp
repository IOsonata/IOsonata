/**-------------------------------------------------------------------------
@file	ba414ep_nrfx.cpp

@brief	nRF54 platform binding for the Silex BA414EP P-256 engine.

		Supplies the target pieces the generic engine (ba414ep.cpp) reaches
		through ba414ep.h: the public-key block base, the operand RAM base, the
		wrapper module power, and the shared engine lock. This is the only
		Ba414ep file that names the vendor wrapper.

		The BA414EP on this family is fixed function for the standard curves, so
		no microcode is loaded here: there is no code RAM write and no firmware
		blob. The block is shared with the random number driver and the symmetric
		engine, so the lock defined here is the one they also take.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

#include "nrf.h"

#include "crypto/ba414ep.h"

#if !defined(NRF54L15_XXAA) && !defined(NRF54H20_XXAA)
#error "ba414ep_nrfx.cpp is only for nRF54 targets that carry CRACEN"
#endif

// Operand (crypto) RAM base. Per the Silex driver this is at a fixed offset
// from the CRACENCORE base, which is NOT the same as the public-key register
// sub-block base; do not derive it from the PK register address.
#define BA414EP_CRYPTORAM_OFFSET	0x8000U

volatile void *Ba414epBase(void)
{
	return (volatile void *)&NRF_CRACENCORE->PK;
}

volatile void *Ba414epOperandRam(void)
{
	return (volatile void *)((uintptr_t)NRF_CRACENCORE + BA414EP_CRYPTORAM_OFFSET);
}

void Ba414epModuleEnable(void)
{
	NRF_CRACEN->ENABLE |= CRACEN_ENABLE_PKEIKG_Msk;
}

void Ba414epModuleDisable(void)
{
	NRF_CRACEN->ENABLE &= ~CRACEN_ENABLE_PKEIKG_Msk;
}

// Public-key engine lock. The Nordic crypto core is one block shared by the RNG, this
// public-key accelerator and the symmetric engine, so all three must serialize
// against one another. These forward to the single shared crypto core lock
// (crypto_core_lock.h) rather than a private state variable, so a public-key
// operation actually excludes a concurrent RNG draw or AES operation.
bool Ba414epTryAcquire(void)
{
	return CryptoCoreTryLock();
}

void Ba414epRelease(void)
{
	CryptoCoreUnlock();
}

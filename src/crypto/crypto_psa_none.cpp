/**-------------------------------------------------------------------------
@file	crypto_psa_none.cpp

@brief	Crypto engine: no PSA Crypto implementation on this target.

Null implementation of the PSA engine entry points. A target whose lib links a
PSA Crypto implementation (TF-PSA-Crypto, or a platform driver such as
nrf_security over CRACEN) links crypto_psa.cpp. A target that does not links
this file, so CryptoPsaInit is always defined exactly once and CryptoInit keeps
its runtime dispatch with no conditional compilation.

Runtime selection still holds: CRYPTO_PROVIDER_PSA returns false here, and
CRYPTO_PROVIDER_AUTO falls through to the next engine.

Exactly one of crypto_psa.cpp or this file belongs in a lib project. The linker
will not report two: it extracts the first archive member defining the symbol
and never opens the second.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include "crypto/crypto.h"

bool CryptoPsaInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	(void)pDev;
	(void)pCfg;

	return false;	// no PSA Crypto implementation in this lib project
}

size_t CryptoPsaMemSize(void)
{
	return 0;
}

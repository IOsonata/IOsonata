/**-------------------------------------------------------------------------
@file	crypto_hw_none.cpp

@brief	Crypto engine: no hardware engine on this target.

Null implementation of the hardware engine entry point. A part with a hardware
crypto block links its own engine (crypto_cc3xx.cpp on nRF52840,
crypto_cracen_bm.cpp on nRF54). A part with none links this file instead, so
CryptoHwInit is always defined exactly once and CryptoInit needs no conditional
compilation to know which parts have hardware.

Exactly one hardware provider must be in a library project.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include "crypto/crypto.h"

bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	(void)pDev;
	(void)pCfg;

	return false;
}

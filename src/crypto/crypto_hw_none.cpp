/**-------------------------------------------------------------------------
@file	crypto_hw_none.cpp

@brief	Crypto engine: no hardware engine on this target.

Null implementation of the hardware engine entry point. A part with a hardware
crypto block links its own engine (crypto_cc310.cpp on nRF52840, crypto_psa.cpp
on nRF54). A part with none links this file instead, so CryptoHwInit is always
defined exactly once and CryptoInit needs no conditional compilation to know
which parts have hardware.

Exactly one of the three must be in a lib project. The linker will not report
two: it extracts the first archive member that defines CryptoHwInit and never
opens the second, so the choice would fall to archive member order. Keeping one
hardware engine per lib project is a project rule, the same as one UART driver.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include "crypto/crypto.h"

bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	(void)pDev;
	(void)pCfg;

	return false;	// no hardware crypto engine on this target
}

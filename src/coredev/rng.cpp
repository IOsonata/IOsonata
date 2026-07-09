/**-------------------------------------------------------------------------
@file	rng.cpp

@brief	Default software random number generator.

Weak fallback implementation of the RNG utility (coredev/rng.h). A target with
a hardware RNG peripheral provides a strong RngGet/RngInit (e.g. rng_nrfx.cpp on
Nordic) that overrides these at link time. A target with no RNG hardware links
this software default automatically.

This software generator is a simple PRNG and is NOT cryptographically strong.
Targets that need strong randomness for security (key generation, nonces) must
provide a hardware RNG. This default exists so that code depending on RngGet
links and runs on any target, and for non-security uses.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include "coredev/rng.h"

#ifndef __weak
#define __weak	__attribute__((weak))
#endif

// xorshift32 state. Seeded to a nonzero constant; a target wanting different
// behavior provides a strong RngGet.
static uint32_t s_RngState = 0x1234abcdUL;

__weak bool RngInit(void)
{
	return true;
}

__weak bool RngGet(uint8_t *pBuff, size_t Len)
{
	uint32_t x = s_RngState;
	for (size_t i = 0; i < Len; i++)
	{
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		pBuff[i] = (uint8_t)x;
	}
	s_RngState = x;
	return true;
}

#include <stddef.h>
#include "crypto_ecb_nrf53.h"
#include "crypto_rng_nrf.h"

extern "C" CryptoSoftAes *CryptoTestAesCreate(void *pMem, size_t MemSize)
{
	return CryptoEcbNrf53Create(pMem, MemSize);
}

extern "C" RngEngine *CryptoTestRngCreate(void *pMem, size_t MemSize)
{
	(void)pMem;
	(void)MemSize;
	return CryptoRngNrfInstance();
}

// Compatibility for existing direct users; MCU libraries use the common Nordic driver.
#ifndef __CRYPTO_ECB_NRF53_H__
#define __CRYPTO_ECB_NRF53_H__
#include "../../include/crypto_ecb_nrf.h"
using CryptoEcbNrf53 = CryptoEcbNrf;
#define CRYPTO_ECB_NRF53_MEMSIZE sizeof(CryptoEcbNrf)
inline CryptoEcbNrf53 *CryptoEcbNrf53Create(void *pMem, size_t MemSize)
{
	return CryptoEcbNrfCreate(pMem, MemSize);
}
#endif

#ifndef __CRYPTO_ECB_NRF53_H__
#define __CRYPTO_ECB_NRF53_H__

#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include "crypto/crypto_softaes.h"

// nRF5340 network-core ECB accelerator. CMAC and CCM use the inherited
// CryptoSoftAes algorithms over the hardware ECB block primitive.
class CryptoEcbNrf53 : public CryptoSoftAes {
public:
	CryptoEcbNrf53() {}
	bool Enable() override { return CryptoSoftAes::Enable(); }
	void Disable() override { CryptoSoftAes::Disable(); }
	void Reset() override {}
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
						 const CryptoKey &Key, const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pIn, size_t Len, uint8_t *pOut) override;
protected:
	bool AesOpBegin() override;
	void AesOpEnd() override;
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
					   uint8_t Out[16]) override;
private:
	struct alignas(4) EcbBlock {
		uint8_t Key[16];
		uint8_t Input[16];
		uint8_t Output[16];
	};
	// Shared DMA storage remains valid even if a caller destroys its engine.
	static EcbBlock s_Block;
	static std::atomic_flag s_Busy;
	static bool s_DmaUnresolved;
};

#define CRYPTO_ECB_NRF53_MEMSIZE sizeof(CryptoEcbNrf53)
CryptoEcbNrf53 *CryptoEcbNrf53Create(void *pMem, size_t MemSize);

#endif

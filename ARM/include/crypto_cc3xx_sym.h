/**-------------------------------------------------------------------------
@file	crypto_cc3xx_sym.h
@brief	CryptoCell AES and streaming SHA-256 overrides.

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/
#ifndef IOSONATA_CRYPTO_CC3XX_SYM_H
#define IOSONATA_CRYPTO_CC3XX_SYM_H
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "cc3xx_intrf.h"

class CryptoCc3xxSym : public CryptoAesModes, public HashEngine {
public:
	bool Init(Cc3xxIntrf *pIntrf);
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
		const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len, uint8_t *pOut) override;
	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
		const uint8_t *pMsg, size_t Len, uint8_t *pMac, size_t MacLen) override;
	CRYPTO_STATUS Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
		const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
		const uint8_t *pMsg, size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen) override;
	CRYPTO_STATUS Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
		const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
		const uint8_t *pMsg, size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen) override;

	CRYPTO_STATUS Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg, size_t Len, uint8_t *pDigest) override;
	size_t HashCtxSize() const override;
	size_t HashCtxAlign() const override;
	CRYPTO_STATUS HashInit(CRYPTO_HASH_ALG Alg, void *pCtx) override;
	CRYPTO_STATUS HashUpdate(void *pCtx, const uint8_t *pMsg, size_t Len) override;
	CRYPTO_STATUS HashFinal(void *pCtx, uint8_t *pDigest) override;
protected:
	bool AesOpBegin() override;
	void AesOpEnd() override;
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16]) override;
private:
	bool vEnabled = false;
	bool vFault = false;
	uint32_t RegRead(uint32_t Offset);
	void RegWrite(uint32_t Offset, uint32_t Value);
	bool Wait(uint32_t Offset, uint32_t Expected);
	CRYPTO_STATUS Begin();
	void End();
	bool Feed(const uint8_t *pIn, size_t Len);
	bool Block(const uint8_t *pKey, size_t KeyLen, bool Encrypt,
		const uint8_t In[16], uint8_t Out[16]);
	CRYPTO_STATUS HashBlock(void *pCtx);
};
#endif

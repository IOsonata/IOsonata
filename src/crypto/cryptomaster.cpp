/**-------------------------------------------------------------------------
@file	cryptomaster.cpp

@brief	Silex CryptoMaster hardware AES engine.

		The CryptoMaster module is held once for the complete one-shot
		cipher call. This prevents another core user from interleaving between
		blocks and avoids releasing and reacquiring the shared core for every block.

@author	Hoang Nguyen Hoan
@date	Jul. 15, 2026

@license

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
#include <stdint.h>
#include <string.h>

#include "idelay.h"
#include "crypto/cryptomaster.h"
#include "crypto/silex_internal.h"

#ifndef __DMB
#define __DMB()		((void)0)
#endif

#define AES_BLOCK		16U
#define CM_AES_BATCH		256U
#define CM_POLL_LIMIT	100000U

static bool s_bCmQuarantined;
// DMA must never retain addresses into a returned stack frame after a fault.
struct CmDmaStorage
{
	uint32_t Config;
	uint8_t Key[32];
	uint8_t Input[CM_AES_BATCH];
	uint8_t Output[CM_AES_BATCH];
	CryptoMasterDesc_t In[3];
	CryptoMasterDesc_t Out;
};
static CmDmaStorage s_Dma;

bool CryptoMaster::OpAcquire()
{
	return SilexOpAcquire();
}

void CryptoMaster::OpRelease()
{
	SilexOpRelease();
}

static void CmWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0U)
	{
		*p++ = 0U;
	}
}

static uint32_t CmRegRead(Device *pDev, uint32_t Offset)
{
	uint8_t off[4] = { (uint8_t)Offset, (uint8_t)(Offset >> 8),
					   (uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24) };
	pDev->DeviceAddress(CRYPTOMASTER_ADDR_REG);
	return pDev->Read32(off, 4);
}

static void CmRegWrite(Device *pDev, uint32_t Offset, uint32_t Value)
{
	uint8_t off[4] = { (uint8_t)Offset, (uint8_t)(Offset >> 8),
					   (uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24) };
	pDev->DeviceAddress(CRYPTOMASTER_ADDR_REG);
	pDev->Write32(off, 4, Value);
}

static bool CmReset(Device *pDev)
{
	CmRegWrite(pDev, CRYPTOMASTER_CONFIG, CRYPTOMASTER_SOFTRESET_ENABLE);
	usDelay(2);
	CmRegWrite(pDev, CRYPTOMASTER_CONFIG, 0U);

	for (uint32_t i = 0; i < CM_POLL_LIMIT; i++)
	{
		if ((CmRegRead(pDev, CRYPTOMASTER_STATUS) &
			 CRYPTOMASTER_STATUS_RESET_BUSY) == 0U)
		{
			CmRegWrite(pDev, CRYPTOMASTER_INT_STATCLR, 0xFFFFFFFFU);
			return true;
		}
	}
	CmRegWrite(pDev, CRYPTOMASTER_INT_STATCLR, 0xFFFFFFFFU);
	return false;
}

static bool CmWait(Device *pDev)
{
	for (uint32_t i = 0; i < CM_POLL_LIMIT; i++)
	{
		if ((CmRegRead(pDev, CRYPTOMASTER_INT_STATRAW) &
			 CRYPTOMASTER_INT_ERROR) != 0U)
		{
			return false;
		}
		if ((CmRegRead(pDev, CRYPTOMASTER_STATUS) &
			 CRYPTOMASTER_STATUS_BUSY) == 0U)
		{
			return (CmRegRead(pDev, CRYPTOMASTER_INT_STATRAW) &
				CRYPTOMASTER_INT_ERROR) == 0U;
		}
	}
	return false;
}

bool CryptoMaster::Recover()
{
	DeviceIntrf *pIntrf = Interface();

	if (pIntrf == nullptr)
	{
		return false;
	}

	// Reset through the interface with this engine selected. The
	// StartTx/StopTx pair serializes the reset against other engines
	// sharing the transport and latches the module an address-selective
	// interface reset acts on. A transport that cannot even open a
	// transfer is unusable; the engine stays down rather than resetting
	// an unselected module.
	if (!pIntrf->StartTx(CRYPTOMASTER_ADDR_REG))
	{
		return false;
	}
	pIntrf->Reset();
	pIntrf->StopTx();

	return CmReset(this) &&
		(CmRegRead(this, CRYPTOMASTER_HW_PRESENCE) &
		 CRYPTOMASTER_PRESENT_AES) != 0U;
}

static bool CmAesBlock(CryptoMaster *pDev, const uint8_t *Key,
						size_t KeyLen, bool Encrypt, const uint8_t *In, uint8_t *Out,
						size_t Len)
{
	uint32_t &config = s_Dma.Config;
	uint8_t *key = s_Dma.Key;
	uint8_t *input = s_Dma.Input;
	uint8_t *output = s_Dma.Output;
	CryptoMasterDesc_t *inDesc = s_Dma.In;
	CryptoMasterDesc_t &outDesc = s_Dma.Out;
	bool ok;
	bool resetOk;

	if (s_bCmQuarantined)
	{
		CmWipe(Out, Len);
		return false;
	}
	config = CRYPTOMASTER_BA411_MODE_ECB | (Encrypt ? 0U : 1U) |
		(KeyLen == 32U ? 4U : 0U);

	memcpy(key, Key, KeyLen);
	if (Len == 0U || Len > CM_AES_BATCH || (Len & (AES_BLOCK - 1U)) != 0U)
	{
		CmWipe(Out, Len);
		return false;
	}
	memcpy(input, In, Len);
	memset(output, 0, Len);
	memset(inDesc, 0, sizeof(s_Dma.In));
	memset(&outDesc, 0, sizeof(outDesc));

	inDesc[0].pAddr = (uint8_t *)&config;
	inDesc[0].pNext = &inDesc[1];
	inDesc[0].Size = sizeof(config) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[0].Tag = CRYPTOMASTER_TAG_ENGINE_AES |
		CRYPTOMASTER_TAG_CONFIG(CRYPTOMASTER_BA411_CFG_OFFSET);

	inDesc[1].pAddr = key;
	inDesc[1].pNext = &inDesc[2];
	inDesc[1].Size = KeyLen | CRYPTOMASTER_DMA_REALIGN;
	inDesc[1].Tag = CRYPTOMASTER_TAG_ENGINE_AES |
		CRYPTOMASTER_TAG_CONFIG(CRYPTOMASTER_BA411_KEY_OFFSET);

	inDesc[2].pAddr = input;
	inDesc[2].pNext = CRYPTOMASTER_DMA_STOP;
	inDesc[2].Size = Len | CRYPTOMASTER_DMA_REALIGN;
	inDesc[2].Tag = CRYPTOMASTER_TAG_ENGINE_AES | CRYPTOMASTER_TAG_LAST;

	outDesc.pAddr = output;
	outDesc.pNext = CRYPTOMASTER_DMA_STOP;
	outDesc.Size = Len | CRYPTOMASTER_DMA_REALIGN;
	outDesc.Tag = CRYPTOMASTER_TAG_LAST;

	// The previous transaction already reset the engine after DMA finished.
	// Avoid a second reset for every block of CBC, CMAC, CCM and GCM.
	// Recover a busy engine before submitting new descriptors.
	ok = (CmRegRead(pDev, CRYPTOMASTER_STATUS) & CRYPTOMASTER_STATUS_BUSY) == 0U;
	if (!ok)
	{
		ok = CmReset(pDev);
		if (!ok && !s_bCmQuarantined)
		{
			ok = pDev->Recover();
		}
	}
	if (ok)
	{
		CmRegWrite(pDev, CRYPTOMASTER_FETCH_ADDR, (uint32_t)(uintptr_t)inDesc);
		CmRegWrite(pDev, CRYPTOMASTER_PUSH_ADDR, (uint32_t)(uintptr_t)&outDesc);
		CmRegWrite(pDev, CRYPTOMASTER_CONFIG,
				   CRYPTOMASTER_CONFIG_SCATTERGATHER);
		__DMB();
		CmRegWrite(pDev, CRYPTOMASTER_START, CRYPTOMASTER_START_BOTH);
		ok = CmWait(pDev);
		__DMB();
	}

	resetOk = CmReset(pDev);
	if (!resetOk)
	{
		resetOk = pDev->Recover();
		if (!resetOk)
		{
			s_bCmQuarantined = true;
		}
	}
	ok = ok && resetOk;

	if (ok)
	{
		memcpy(Out, output, Len);
	}
	else
	{
		memset(Out, 0, Len);
	}

	// A stuck DMA retains stable descriptors until an explicit successful reset.
	if (resetOk)
	{
		CmWipe(&s_Dma, sizeof(s_Dma));
	}
	return ok;
}

bool CryptoMaster::Init(DeviceIntrf * const pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	Interface(pIntrf);
	return Enable();
}

bool CryptoMaster::Enable()
{
	if (Interface() == nullptr || s_bCmQuarantined)
	{
		vbValid = false;
		return false;
	}
	if (vbIntrfEnabled && vbValid)
	{
		return true;
	}

	bool acquiredRef = false;
	if (!vbIntrfEnabled)
	{
		Interface()->Enable();
		vbIntrfEnabled = true;
		acquiredRef = true;
	}
	if (!OpAcquire())
	{
		if (acquiredRef)
		{
			Interface()->Disable();
			vbIntrfEnabled = false;
		}
		vbValid = false;
		return false;
	}

	bool ok = (CmRegRead(this, CRYPTOMASTER_HW_PRESENCE) &
				 CRYPTOMASTER_PRESENT_AES) != 0U && CmReset(this);
	if (!ok)
	{
		ok = Recover();
	}
	OpRelease();
	if (!ok)
	{
		if (acquiredRef)
		{
			Interface()->Disable();
			vbIntrfEnabled = false;
		}
		vbValid = false;
		return false;
	}

	s_bCmQuarantined = false;
	vbValid = true;
	return true;
}

void CryptoMaster::Disable()
{
	if (!vbIntrfEnabled || Interface() == nullptr)
	{
		vbValid = false;
		return;
	}
	if (!OpAcquire())
	{
		return;
	}
	vbValid = false;
	Interface()->Disable();
	vbIntrfEnabled = false;
	OpRelease();
}

void CryptoMaster::Reset()
{
	if (!vbIntrfEnabled || Interface() == nullptr || !OpAcquire())
	{
		return;
	}
	vbValid = false;
	bool ok = Recover();
	s_bCmQuarantined = !ok;
	if (ok)
	{
		CmWipe(&s_Dma, sizeof(s_Dma));
	}
	vbValid = ok;
	OpRelease();
}

CRYPTO_STATUS CryptoMaster::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
								   const CryptoKey &Key,
								   const uint8_t *pIv, size_t IvLen,
								   const uint8_t *pIn, size_t Len,
								   uint8_t *pOut)
{
	size_t keyLen = Key.Type == CRYPTO_KEY_AES_256 ? 32U : 16U;
	uint32_t required = bEncrypt != 0 ? CRYPTO_KEY_USE_ENCRYPT : CRYPTO_KEY_USE_DECRYPT;
	if ((Key.Type != CRYPTO_KEY_AES_128 && Key.Type != CRYPTO_KEY_AES_256) ||
		Key.Loc != CRYPTO_KEY_LOC_PLAIN || Key.Plain.pData == nullptr ||
		Key.Plain.Len != keyLen || (Key.Usage & required) == 0U ||
		pIn == nullptr || pOut == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	if (!vbValid || !vbIntrfEnabled || Interface() == nullptr || s_bCmQuarantined)
	{
		if (Len > 0U) memset(pOut, 0, Len);
		return CRYPTO_STATUS_FAIL;
	}

	if (Alg == CRYPTO_CIPHER_ECB && (Len & (AES_BLOCK - 1U)) != 0U)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if ((Alg == CRYPTO_CIPHER_CTR || Alg == CRYPTO_CIPHER_CBC) &&
		(pIv == nullptr || IvLen != AES_BLOCK))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (Alg == CRYPTO_CIPHER_CBC && (Len & (AES_BLOCK - 1U)) != 0U)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (Alg != CRYPTO_CIPHER_ECB && Alg != CRYPTO_CIPHER_CTR &&
		Alg != CRYPTO_CIPHER_CBC)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!OpAcquire())
	{
		if (Len > 0U) memset(pOut, 0, Len);
		return CRYPTO_STATUS_BUSY;
	}

	const uint8_t *key = Key.Plain.pData;
	CRYPTO_STATUS status = CRYPTO_STATUS_OK;

	if (Alg == CRYPTO_CIPHER_ECB)
	{
		for (size_t off = 0; off < Len; )
		{
			size_t count = Len - off < CM_AES_BATCH ? Len - off : CM_AES_BATCH;
			if (!CmAesBlock(this, key, keyLen, bEncrypt != 0, pIn + off, pOut + off, count))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
			off += count;
		}
	}
	else if (Alg == CRYPTO_CIPHER_CTR)
	{
		uint8_t ctr[AES_BLOCK], stream[CM_AES_BATCH], counters[CM_AES_BATCH];
		memcpy(ctr, pIv, AES_BLOCK);
		for (size_t off = 0; off < Len; )
		{
			size_t count = Len - off < CM_AES_BATCH ? Len - off : CM_AES_BATCH;
			size_t padded = (count + AES_BLOCK - 1U) & ~(AES_BLOCK - 1U);
			for (size_t i = 0; i < padded; i += AES_BLOCK)
			{
				memcpy(counters + i, ctr, AES_BLOCK);
				for (int j = AES_BLOCK - 1; j >= 0; j--)
				{
					if (++ctr[j] != 0U)
						break;
				}
			}
			if (!CmAesBlock(this, key, keyLen, true, counters, stream, padded))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
			for (size_t i = 0; i < count; i++)
			{
				pOut[off + i] = pIn[off + i] ^ stream[i];
			}
			off += count;
		}
		CmWipe(ctr, sizeof(ctr));
		CmWipe(stream, sizeof(stream));
		CmWipe(counters, sizeof(counters));
	}
	else if (bEncrypt == 0)
	{
		uint8_t chain[AES_BLOCK];
		uint8_t ciphertext[CM_AES_BATCH];
		uint8_t plaintext[CM_AES_BATCH];

		memcpy(chain, pIv, AES_BLOCK);
		for (size_t off = 0; off < Len; )
		{
			size_t count = Len - off < CM_AES_BATCH ? Len - off : CM_AES_BATCH;
			memcpy(ciphertext, pIn + off, count);
			if (!CmAesBlock(this, key, keyLen, false, ciphertext, plaintext, count))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
			for (size_t i = 0; i < count; i += AES_BLOCK)
			{
				for (size_t j = 0; j < AES_BLOCK; j++)
				{
					pOut[off + i + j] = plaintext[i + j] ^ chain[j];
				}
				memcpy(chain, ciphertext + i, AES_BLOCK);
			}
			off += count;
		}
		CmWipe(chain, sizeof(chain));
		CmWipe(ciphertext, sizeof(ciphertext));
		CmWipe(plaintext, sizeof(plaintext));
	}
	else
	{
		uint8_t chain[AES_BLOCK];
		uint8_t block[AES_BLOCK];
		uint8_t saved[AES_BLOCK];
		memcpy(chain, pIv, AES_BLOCK);
		for (size_t off = 0; off < Len; off += AES_BLOCK)
		{
			memcpy(saved, pIn + off, AES_BLOCK);
			for (size_t i = 0; i < AES_BLOCK; i++)
			{
				block[i] = saved[i] ^ (bEncrypt ? chain[i] : 0U);
			}
			if (!CmAesBlock(this, key, keyLen, bEncrypt != 0, block, pOut + off, AES_BLOCK))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
			if (!bEncrypt)
			{
				for (size_t i = 0; i < AES_BLOCK; i++)
				{
					pOut[off + i] ^= chain[i];
				}
			}
			memcpy(chain, bEncrypt ? pOut + off : saved, AES_BLOCK);
		}
		CmWipe(chain, sizeof(chain));
		CmWipe(block, sizeof(block));
		CmWipe(saved, sizeof(saved));
	}

	if (s_bCmQuarantined)
	{
		vbValid = false;
		status = CRYPTO_STATUS_FAIL;
	}
	OpRelease();
	if (status != CRYPTO_STATUS_OK && Len > 0U)
	{
		memset(pOut, 0, Len);
	}
	return status;
}

bool CryptoMaster::AesOpBegin()
{
	return vbValid && vbIntrfEnabled && !s_bCmQuarantined &&
		Interface() != nullptr && OpAcquire();
}

void CryptoMaster::AesOpEnd()
{
	OpRelease();
}

bool CryptoMaster::AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
							   uint8_t Out[16])
{
	return CmAesBlock(this, Key, 16U, true, In, Out, AES_BLOCK);
}

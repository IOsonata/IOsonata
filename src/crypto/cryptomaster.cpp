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

#ifndef __DMB
#define __DMB()		((void)0)
#endif

#define AES_BLOCK		16U
#define CM_POLL_LIMIT	100000U

static bool s_bCmQuarantined;

extern "C" __attribute__((weak)) bool CryptoMasterPlatformReset(DeviceIntrf *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	pIntrf->Reset();
	return true;
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
			return true;
		}
	}
	return false;
}

static bool CmRecover(CryptoMaster *pDev)
{
	return pDev != nullptr &&
		CryptoMasterPlatformReset(pDev->Interface()) &&
		CmReset(pDev) &&
		(CmRegRead(pDev, CRYPTOMASTER_HW_PRESENCE) &
		 CRYPTOMASTER_PRESENT_AES) != 0U;
}

static bool CmAesBlock(CryptoMaster *pDev, const uint8_t Key[16],
						const uint8_t In[16], uint8_t Out[16])
{
	alignas(uint32_t) uint32_t config = CRYPTOMASTER_BA411_MODE_ECB;
	alignas(uint32_t) uint8_t key[AES_BLOCK];
	alignas(uint32_t) uint8_t input[AES_BLOCK];
	alignas(uint32_t) uint8_t output[AES_BLOCK];
	alignas(uint32_t) CryptoMasterDesc_t inDesc[3];
	alignas(uint32_t) CryptoMasterDesc_t outDesc;

	memcpy(key, Key, sizeof(key));
	memcpy(input, In, sizeof(input));
	memset(output, 0, sizeof(output));
	memset(inDesc, 0, sizeof(inDesc));
	memset(&outDesc, 0, sizeof(outDesc));

	inDesc[0].pAddr = (uint8_t *)&config;
	inDesc[0].pNext = &inDesc[1];
	inDesc[0].Size = sizeof(config) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[0].Tag = CRYPTOMASTER_TAG_ENGINE_AES |
		CRYPTOMASTER_TAG_CONFIG(CRYPTOMASTER_BA411_CFG_OFFSET);

	inDesc[1].pAddr = key;
	inDesc[1].pNext = &inDesc[2];
	inDesc[1].Size = sizeof(key) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[1].Tag = CRYPTOMASTER_TAG_ENGINE_AES |
		CRYPTOMASTER_TAG_CONFIG(CRYPTOMASTER_BA411_KEY_OFFSET);

	inDesc[2].pAddr = input;
	inDesc[2].pNext = CRYPTOMASTER_DMA_STOP;
	inDesc[2].Size = sizeof(input) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[2].Tag = CRYPTOMASTER_TAG_ENGINE_AES | CRYPTOMASTER_TAG_LAST;

	outDesc.pAddr = output;
	outDesc.pNext = CRYPTOMASTER_DMA_STOP;
	outDesc.Size = sizeof(output) | CRYPTOMASTER_DMA_REALIGN;
	outDesc.Tag = CRYPTOMASTER_TAG_LAST;

	bool ok = !s_bCmQuarantined && CmReset(pDev);
	if (!ok && !s_bCmQuarantined)
	{
		ok = CmRecover(pDev);
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

	bool resetOk = CmReset(pDev);
	if (!resetOk)
	{
		resetOk = CmRecover(pDev);
		if (!resetOk)
		{
			s_bCmQuarantined = true;
		}
	}
	ok = ok && resetOk;

	if (ok)
	{
		memcpy(Out, output, sizeof(output));
	}
	else
	{
		memset(Out, 0, AES_BLOCK);
	}

	CmWipe(key, sizeof(key));
	CmWipe(input, sizeof(input));
	CmWipe(output, sizeof(output));
	CmWipe(inDesc, sizeof(inDesc));
	CmWipe(&outDesc, sizeof(outDesc));
	config = 0U;
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
		ok = CmRecover(this);
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
	bool ok = CmRecover(this);
	s_bCmQuarantined = !ok;
	vbValid = ok;
	OpRelease();
}

CRYPTO_STATUS CryptoMaster::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
								   const CryptoKey &Key,
								   const uint8_t *pIv, size_t IvLen,
								   const uint8_t *pIn, size_t Len,
								   uint8_t *pOut)
{
	uint32_t required = bEncrypt != 0 ? CRYPTO_KEY_USE_ENCRYPT :
									 CRYPTO_KEY_USE_DECRYPT;
	if (Key.Type != CRYPTO_KEY_AES_128 || Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
		Key.Plain.pData == nullptr || Key.Plain.Len != AES_BLOCK ||
		(Key.Usage & required) == 0U || pIn == nullptr || pOut == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!vbValid || !vbIntrfEnabled || Interface() == nullptr || s_bCmQuarantined)
	{
		if (Len > 0U) memset(pOut, 0, Len);
		return CRYPTO_STATUS_FAIL;
	}

	if (bEncrypt == 0 && Alg != CRYPTO_CIPHER_CTR)
	{
		return CryptoSoftAes::Cipher(Alg, bEncrypt, Key, pIv, IvLen,
								 pIn, Len, pOut);
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
		for (size_t off = 0; off < Len; off += AES_BLOCK)
		{
			if (!CmAesBlock(this, key, pIn + off, pOut + off))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
		}
	}
	else if (Alg == CRYPTO_CIPHER_CTR)
	{
		uint8_t ctr[AES_BLOCK], stream[AES_BLOCK];
		memcpy(ctr, pIv, AES_BLOCK);
		for (size_t off = 0; off < Len; )
		{
			if (!CmAesBlock(this, key, ctr, stream))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
			size_t count = Len - off < AES_BLOCK ? Len - off : AES_BLOCK;
			for (size_t i = 0; i < count; i++)
			{
				pOut[off + i] = pIn[off + i] ^ stream[i];
			}
			for (int i = AES_BLOCK - 1; i >= 0; i--)
			{
				if (++ctr[i] != 0U) break;
			}
			off += count;
		}
		CmWipe(ctr, sizeof(ctr));
		CmWipe(stream, sizeof(stream));
	}
	else
	{
		uint8_t chain[AES_BLOCK];
		memcpy(chain, pIv, AES_BLOCK);
		for (size_t off = 0; off < Len; off += AES_BLOCK)
		{
			uint8_t block[AES_BLOCK];
			for (size_t i = 0; i < AES_BLOCK; i++)
			{
				block[i] = pIn[off + i] ^ chain[i];
			}
			if (!CmAesBlock(this, key, block, pOut + off))
			{
				status = CRYPTO_STATUS_FAIL;
				CmWipe(block, sizeof(block));
				break;
			}
			memcpy(chain, pOut + off, AES_BLOCK);
			CmWipe(block, sizeof(block));
		}
		CmWipe(chain, sizeof(chain));
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
	return CmAesBlock(this, Key, In, Out);
}

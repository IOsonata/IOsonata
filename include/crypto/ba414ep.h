/**-------------------------------------------------------------------------
@file	ba414ep.h

@brief	Silex BA414EP hardware P-256 engine on the OO engine tree.

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
#ifndef __BA414EP_H__
#define __BA414EP_H__

#include <stdint.h>
#include <stddef.h>

#include "device_intrf.h"
#include "crypto/icrypto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BA414EP_REG_CONFIG			0x00U
#define BA414EP_REG_COMMAND			0x04U
#define BA414EP_REG_CONTROL			0x08U
#define BA414EP_REG_STATUS			0x0CU
#define BA414EP_REG_HWCONFIG		0x18U

#define BA414EP_CONTROL_START		0x00000001U
#define BA414EP_CONTROL_CLEAR_IRQ	0x00000002U

#define BA414EP_STATUS_BUSY			(1U << 16)
#define BA414EP_STATUS_POINT_ERROR		(1U << 4)
#define BA414EP_STATUS_NOT_INVERTIBLE	(1U << 11)
#define BA414EP_STATUS_ERROR_MASK	0x0000FFF0U

#define BA414EP_CMD_ECC_PTMUL		0x22U
#define BA414EP_CMD_CHECK_XY		0x25U
#define BA414EP_CMD_OPSIZE(bytes)	(((uint32_t)((bytes) - 1U)) << 8)
#define BA414EP_CMD_SELCUR_P256		0x00100000U
#define BA414EP_CMD_RANDOM_SCALAR	(1U << 24)
#define BA414EP_CMD_RANDOM_PROJECTIVE	(1U << 25)
#define BA414EP_CMD_BIG_ENDIAN		(1U << 28)

#define BA414EP_CMD_P256_PTMUL \
	(BA414EP_CMD_ECC_PTMUL | BA414EP_CMD_OPSIZE(32U) | \
	 BA414EP_CMD_SELCUR_P256 | BA414EP_CMD_RANDOM_SCALAR | \
	 BA414EP_CMD_RANDOM_PROJECTIVE | BA414EP_CMD_BIG_ENDIAN)
#define BA414EP_CMD_P256_CHECK_XY \
	(BA414EP_CMD_CHECK_XY | BA414EP_CMD_OPSIZE(32U) | \
	 BA414EP_CMD_SELCUR_P256 | BA414EP_CMD_BIG_ENDIAN)

#define BA414EP_CONFIG_PTRS(a, b, c) \
	((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16))

#define BA414EP_SLOT_SIZE			0x200U
#define BA414EP_SLOT_SCALAR			8U
#define BA414EP_SLOT_RESULT_X		10U
#define BA414EP_SLOT_RESULT_Y		11U
#define BA414EP_SLOT_POINT_X		12U
#define BA414EP_SLOT_POINT_Y		13U
#define BA414EP_SLOT_BLIND			15U

#define BA414EP_CONFIG_PTMUL		BA414EP_CONFIG_PTRS(12U, 8U, 10U)

#define BA414EP_CRYPTORAM_OFFSET	0x8000U
#define BA414EP_ADDR_REG			0U
#define BA414EP_ADDR_MEM			1U

#ifdef __cplusplus
}

class CracenIntrf;

class Ba414ep : public KeyAgreeEngine {
public:
	struct KeyCtx {
		uint8_t PrivKey[32];
		bool bKeyValid;
	};

	Ba414ep() { vbValid = false; vpRng = nullptr; vpCracen = nullptr; }

	bool Init(CracenIntrf * const pIntrf, RngEngine *pRng);
	void SetRng(RngEngine *pRng) { vpRng = pRng; }

	bool Enable() override;
	void Disable() override {}
	void Reset() override {}

	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	void KeyReset(void *pKeyCtx) override;

	// Known-answer self-test: LESC debug key point multiplication on the
	// hardware, including the blinding countermeasure path.
	int SelfTest() override;
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPubKey) override;
	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
						const uint8_t *pPeerPubKey, uint8_t *pSharedX,
						bool bKeepKey = false) override;

private:
	RngEngine *vpRng;
	CracenIntrf *vpCracen;
};

static_assert(sizeof(Ba414ep::KeyCtx) <= CRYPTO_KEYCTX_MAX,
			  "BA414EP key context exceeds the common consumer storage");

#endif // __cplusplus

#endif // __BA414EP_H__

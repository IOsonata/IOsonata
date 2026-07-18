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

#define BA414EP_STATUS_BUSY			(1U << 16)	//!< Operation in progress
#define BA414EP_STATUS_IK_ACTIVE	(1U << 17)	//!< IK side of the module active

// Identity key bank of the Silex ba414e_with_ik configuration. Offsets are
// from the same register base as the public-key registers.
#define BA414EP_IK_REG_PK_STATUS	0x1024U
#define BA414EP_IK_PK_BUSY_MASK		0x00050000U
#define BA414EP_STATUS_POINT_ERROR		(1U << 4)
#define BA414EP_STATUS_NOT_INVERTIBLE	(1U << 11)
#define BA414EP_STATUS_ERROR_MASK	0x0000FFF0U

#define BA414EP_CMD_MOD_ADD			0x01U
#define BA414EP_CMD_ECC_PTMUL		0x22U
#define BA414EP_CMD_CHECK_XY		0x25U
#define BA414EP_CMD_ECC_PTONCURVE	0x26U
#define BA414EP_CMD_OPSIZE(bytes)	(((uint32_t)((bytes) - 1U)) << 8)
#define BA414EP_CMD_SELCUR_P256		0x00100000U
#define BA414EP_CMD_RANDOM_SCALAR	(1U << 24)
#define BA414EP_CMD_RANDOM_PROJECTIVE	(1U << 25)
#define BA414EP_CMD_BIG_ENDIAN		(1U << 28)
#define BA414EP_CMD_RESQUARE		(1U << 31)	//!< Recompute r square constant

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

// Generic (Silex reference) point-multiply: SELCUR stays zero, so the built-in
// curve constants are not used and the full P-256 domain is loaded into slots:
// p,n at 0,1; base point at 2,3; a,b at 4,5; scalar at 14; result point at 6,7.
// The PKE microcode is still required; the interface loads it on acquisition.
#define BA414EP_SLOT_P				0U
#define BA414EP_SLOT_N				1U
#define BA414EP_SLOT_GX				2U
#define BA414EP_SLOT_GY				3U
#define BA414EP_SLOT_A				4U
#define BA414EP_SLOT_B				5U
#define BA414EP_SLOT_RESULT_GX		6U
#define BA414EP_SLOT_RESULT_GY		7U
#define BA414EP_SLOT_SCALAR_GEN		14U

#define BA414EP_CMD_P256_PTMUL_GEN \
	(BA414EP_CMD_ECC_PTMUL | BA414EP_CMD_OPSIZE(32U) | \
	 BA414EP_CMD_RANDOM_SCALAR | BA414EP_CMD_RANDOM_PROJECTIVE | \
	 BA414EP_CMD_BIG_ENDIAN)

// On-curve check of the point in the operand A slots: x < p, y < p and the
// curve equation. Run before the multiply on a peer supplied point, the same
// order the Nordic sdk-nrf ECDH path uses (sx_ec_ptoncurve then sx_ecp_ptmult).
#define BA414EP_CMD_P256_PTONCURVE \
	(BA414EP_CMD_ECC_PTONCURVE | BA414EP_CMD_OPSIZE(32U) | \
	 BA414EP_CMD_BIG_ENDIAN)

#define BA414EP_CONFIG_PTMUL_GEN	BA414EP_CONFIG_PTRS(2U, 14U, 6U)

#define BA414EP_CRYPTORAM_OFFSET	0x8000U

// DevAddr sub-block selectors of this engine, used the way SPI uses a chip
// select. On an interface shared with other Silex engines the values must
// not collide: CryptoMaster uses 2, vendor additions start at 3.
#define BA414EP_ADDR_REG			0U	//!< Register base
#define BA414EP_ADDR_MEM			1U	//!< Operand memory base

#ifdef __cplusplus
}

class Ba414ep : public KeyAgreeEngine {
public:
	struct KeyCtx {
		uint8_t PrivKey[32];
		bool bKeyValid;
	};

	Ba414ep();

	bool Init(DeviceIntrf * const pIntrf, RngEngine *pRng);
	void SetRng(RngEngine *pRng) { vpRng = pRng; }

	// First Enable brings the engine to a proven-ready state: wait out the
	// IK side of the module, then run a self-checking modular add that also
	// absorbs the post-reset IK to PK handover of the ba414e_with_ik
	// configuration.
	bool Enable() override;
	void Disable() override;

	// Reset the selected PKE module, then prove the IK-to-PK handover and wipe
	// the operand slots before accepting another operation.
	void Reset() override;

	// Operation lock for the multi-transfer computation. bBusy in the
	// interface is per transfer; this is per operation, owned by the device.
	bool OpAcquire();
	void OpRelease();

	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	size_t KeyCtxAlign() const override { return alignof(KeyCtx); }
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
	bool WaitIkIdle();
	bool HandoverProbe();
	bool Recover();
	CRYPTO_STATUS PkPrepare();
	void PkCleanup();
	bool PkAbortAndReset();
	CRYPTO_STATUS PkPointMultiply(const uint8_t Point[64],
								  const uint8_t Scalar[32], uint8_t Result[64],
								  bool bValidatePoint);

	RngEngine *vpRng;
	atomic_flag vOpBusy;
	bool vbReady;
	bool vbIntrfEnabled;
};

static_assert(sizeof(Ba414ep::KeyCtx) <= CRYPTO_KEYCTX_MAX,
			  "BA414EP key context exceeds the common consumer storage");

#endif // __cplusplus

#endif // __BA414EP_H__

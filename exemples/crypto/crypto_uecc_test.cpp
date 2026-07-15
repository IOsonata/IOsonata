/**-------------------------------------------------------------------------
@file	crypto_uecc_test.cpp

@brief	Host validation harness for the OO CryptoUecc engine.

		Exercises CryptoUecc through the KeyAgreeEngine and SignEngine facet
		interfaces only. It includes just crypto_uecc.h: no micro-ecc header and
		no direct library call, so it tests the port, not the library. Runs on
		the build host; provides a test RngGet so no MCU driver is needed.

		The ECDSA test uses a fixed known key pair (the BLE spec private key A
		and its matching P-256 public key) so the harness needs no key
		derivation and stays on the facet interface.

		Tests:
		  1. BLE spec P-256 DH known-answer (SelfTest).
		  2. Facet-level ECDH round trip: two engines agree on the same secret.
		  3. Facet pointer polymorphism: the object is usable as a
		     KeyAgreeEngine* and a SignEngine* with one shared Device subobject.
		  4. Invalid-curve peer key is rejected (CVE-2018-5383).
		  5. Single-use private key: a second Agree fails after the first.
		  6. ECDSA sign then verify round trip; a tampered signature fails.

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "crypto/crypto_uecc.h"		// engine under test - only header needed

// Test entropy source standing in for the per-MCU RngGet. A fixed-seed xorshift
// keeps the run reproducible; this is a test stub, not a security generator.
static uint64_t s_rng = 0x123456789abcdef0ULL;
extern "C" bool RngGet(uint8_t *pBuff, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		s_rng ^= s_rng << 13;
		s_rng ^= s_rng >> 7;
		s_rng ^= s_rng << 17;
		pBuff[i] = (uint8_t)(s_rng & 0xFF);
	}
	return true;
}

static int s_pass = 0, s_fail = 0;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

// CryptoUecc's KeyCtx is private to the class; the facet reports its size via
// KeyCtxSize(). Allocate a generous aligned context buffer from that.
alignas(8) static uint8_t g_ctxA[64];
alignas(8) static uint8_t g_ctxB[64];

int main(void)
{
	printf("CryptoUecc OO engine validation\n");

	// Construct two engines in caller storage (no allocation).
	static uint8_t memA[128], memB[128];
	CryptoUecc *ea = CryptoUeccCreate(memA, sizeof(memA));
	CryptoUecc *eb = CryptoUeccCreate(memB, sizeof(memB));
	check("factory constructs engines", ea != nullptr && eb != nullptr);
	if (ea == nullptr || eb == nullptr) { printf("abort\n"); return 1; }

	// Hold as facet base pointers: the port must be usable polymorphically.
	KeyAgreeEngine *ka = ea;
	KeyAgreeEngine *kb = eb;
	SignEngine     *sa = ea;
	CryptoEngine   *cea = ea;			// single shared base subobject
	Device         *da = ea;
	check("facet + base pointers resolve (single subobject)",
		  ka != nullptr && sa != nullptr && cea != nullptr && da != nullptr);

	// Test 1: BLE spec known-answer self-test through the facet.
	check("BLE spec P-256 DH known-answer (SelfTest)", ea->SelfTest() == 0);

	// Test 2 + 3: full ECDH round trip through the facet interface.
	size_t ctxSz = ka->KeyCtxSize();
	check("KeyCtxSize reported and fits test buffer",
		  ctxSz > 0 && ctxSz <= sizeof(g_ctxA));

	uint8_t pubA[64], pubB[64], dhA[32], dhB[32];
	CRYPTO_STATUS r1 = ka->KeyGen(CRYPTO_CURVE_P256, g_ctxA, pubA);
	CRYPTO_STATUS r2 = kb->KeyGen(CRYPTO_CURVE_P256, g_ctxB, pubB);
	check("KeyGen A and B succeed", r1 == CRYPTO_STATUS_OK && r2 == CRYPTO_STATUS_OK);

	CRYPTO_STATUS r3 = ka->Agree(CRYPTO_CURVE_P256, g_ctxA, pubB, dhA);
	CRYPTO_STATUS r4 = kb->Agree(CRYPTO_CURVE_P256, g_ctxB, pubA, dhB);
	check("Agree A and B succeed", r3 == CRYPTO_STATUS_OK && r4 == CRYPTO_STATUS_OK);
	check("ECDH round trip: both sides derive the same secret",
		  memcmp(dhA, dhB, 32) == 0);

	// Test 4: invalid-curve peer key must be rejected (CVE-2018-5383).
	uint8_t badPeer[64];
	memset(badPeer, 0xAB, sizeof(badPeer));		// not a point on the curve
	CRYPTO_STATUS r5 = ka->KeyGen(CRYPTO_CURVE_P256, g_ctxA, pubA);
	CRYPTO_STATUS r6 = ka->Agree(CRYPTO_CURVE_P256, g_ctxA, badPeer, dhA);
	check("invalid-curve peer key rejected (CVE-2018-5383)",
		  r5 == CRYPTO_STATUS_OK && r6 == CRYPTO_STATUS_FAIL);

	// Test 5: single-use private key. After one successful Agree, a second
	// Agree with the same context must fail (key was wiped).
	CRYPTO_STATUS r7 = ka->KeyGen(CRYPTO_CURVE_P256, g_ctxA, pubA);
	CRYPTO_STATUS r8 = ka->Agree(CRYPTO_CURVE_P256, g_ctxA, pubB, dhA);
	CRYPTO_STATUS r9 = ka->Agree(CRYPTO_CURVE_P256, g_ctxA, pubB, dhA);
	check("single-use key: first Agree ok, second Agree fails",
		  r7 == CRYPTO_STATUS_OK && r8 == CRYPTO_STATUS_OK &&
		  r9 == CRYPTO_STATUS_FAIL);

	// Test 6: ECDSA sign (facet) then verify (facet), with a tamper check.
	// The signing key pair is a fixed known vector so the harness needs no
	// derivation and depends only on crypto_uecc.h: kSignPriv is the BLE spec
	// private key A, kSignPub its matching P-256 public key.
	static const uint8_t kSignPriv[32] = {
		0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,
		0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd };
	static const uint8_t kSignPub[64] = {
		0x20,0xb0,0x03,0xd2,0xf2,0x97,0xbe,0x2c,0x5e,0x2c,0x83,0xa7,0xe9,0xf9,0xa5,0xb9,
		0xef,0xf4,0x91,0x11,0xac,0xf4,0xfd,0xdb,0xcc,0x03,0x01,0x48,0x0e,0x35,0x9d,0xe6,
		0xdc,0x80,0x9c,0x49,0x65,0x2a,0xeb,0x6d,0x63,0x32,0x9a,0xbf,0x5a,0x52,0x15,0x5c,
		0x76,0x63,0x45,0xc2,0x8f,0xed,0x30,0x24,0x74,0x1c,0x8e,0xd0,0x15,0x89,0xd2,0x8b };

	uint8_t hash[32];
	for (int i = 0; i < 32; i++) hash[i] = (uint8_t)(i * 7 + 1);

	uint8_t sig[64];
	CryptoKey sk{ CRYPTO_KEY_ECC_P256, CRYPTO_KEY_LOC_PLAIN,
				  CRYPTO_KEY_USE_SIGN, {} };
	sk.Plain.pData = kSignPriv; sk.Plain.Len = 32;

	CRYPTO_STATUS rs = sa->Sign(CRYPTO_CURVE_P256, sk, hash, 32, sig);
	check("ECDSA Sign succeeds (facet)", rs == CRYPTO_STATUS_OK);

	CRYPTO_STATUS rv = sa->Verify(CRYPTO_CURVE_P256, kSignPub, hash, 32, sig);
	check("ECDSA Verify accepts a valid signature (facet)", rv == CRYPTO_STATUS_OK);

	uint8_t badSig[64];
	memcpy(badSig, sig, 64);
	badSig[0] ^= 0x01;					// tamper one bit
	CRYPTO_STATUS rvBad = sa->Verify(CRYPTO_CURVE_P256, kSignPub, hash, 32, badSig);
	check("ECDSA Verify rejects a tampered signature (facet)",
		  rvBad == CRYPTO_STATUS_FAIL);

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}

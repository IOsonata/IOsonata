/**-------------------------------------------------------------------------
@example	crypto_engine_test.cpp

@brief	Hardware crypto engine test through the generic engine interfaces.

Runs known-answer vectors on an AES engine, a P-256 engine and a random
source, then compares each of them with the software engines (CryptoSoftAes
and CryptoUecc) on random inputs, and times a P-256 scalar multiplication.
Every call goes through the CipherEngine, MacEngine, AeadEngine,
KeyAgreeEngine, SignEngine and RngEngine interfaces, so the same file tests
any chip engine.

Without any definition the engines under test are the software ones, which
checks the test itself on a host. A board selects its engines in board.h:

	CRYPTO_TEST_ENGINE_HEADER	header of the engines, quoted
	CRYPTO_TEST_AES_TYPE		class under test, a CryptoSoftAes derived
	CRYPTO_TEST_AES_MEMSIZE		its storage size
	CRYPTO_TEST_AES_CREATE(m,s)	factory: memory, size
	CRYPTO_TEST_AES256		enable AES-256 Cipher checks when supported
	CRYPTO_TEST_PKA_TYPE		class under test, KeyAgreeEngine and SignEngine
	CRYPTO_TEST_PKA_MEMSIZE
	CRYPTO_TEST_PKA_CREATE(m,s,rng)
	CRYPTO_TEST_RNG_TYPE		class under test, a RngEngine
	CRYPTO_TEST_RNG_MEMSIZE
	CRYPTO_TEST_RNG_CREATE(m,s)

Optional:

	CRYPTO_TEST_TICKS_INIT()	start a free running counter
	CRYPTO_TEST_TICKS()			read it, a uint32_t
	CRYPTO_TEST_TICKS_PER_US	counts per microsecond
	CRYPTO_TEST_SOAK_AES		random AES rounds, default 200
	CRYPTO_TEST_SOAK_PKA		random P-256 rounds, default 8
	CRYPTO_TEST_NO_MAIN			give CryptoEngineTest() and no main() or UART setup
	UART_RETARGET				send stdout to the board UART, as i2c_scan.cpp

The result is printed, and returned by main as 0 when every check passed.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdio.h>
#include <string.h>
#include <new>

#include "crypto/icrypto.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softrng.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

// The board names the engines under test, so its header comes first
#if defined(__has_include)
#if __has_include("board.h")
#include "board.h"
#endif
#else
#include "board.h"
#endif

#ifdef CRYPTO_TEST_ENGINE_HEADER
#include CRYPTO_TEST_ENGINE_HEADER
#endif

#if defined(UART_RETARGET) && !defined(CRYPTO_TEST_NO_MAIN)
#include <unistd.h>
#include "coredev/system_core_clock.h"
#include "coredev/uart.h"
#include "stddev.h"
#endif

// Secure flagged generator for the reference engine, and for the software
// only build. It is a deterministic generator, fit for a test and no more.
class EngineTestRng : public CryptoSoftRng {
public:
	bool IsSecure() const override { return true; }
};

// Applications select their target engine in board.h. Software engines are
// used when the board does not provide an implementation.
#ifndef CRYPTO_TEST_AES_TYPE
#define CRYPTO_TEST_AES_TYPE		CryptoSoftAes
#define CRYPTO_TEST_AES_MEMSIZE		CRYPTO_SOFTAES_MEMSIZE
#define CRYPTO_TEST_AES_CREATE(m, s)	CryptoSoftAesCreate((m), (s))
#endif

#ifndef CRYPTO_TEST_PKA_TYPE
#define CRYPTO_TEST_PKA_TYPE		CryptoUecc
#define CRYPTO_TEST_PKA_MEMSIZE		CRYPTO_UECC_MEMSIZE
#define CRYPTO_TEST_PKA_CREATE(m, s, r)	CryptoUeccCreate((m), (s), (r))
#endif

#ifndef CRYPTO_TEST_RNG_TYPE

static EngineTestRng *EngineTestRngCreate(void *pMem, size_t MemSize)
{
	if (MemSize < sizeof(EngineTestRng))
	{
		return nullptr;
	}

	EngineTestRng *p = new (pMem) EngineTestRng();

	p->Enable();

	return p;
}

#define CRYPTO_TEST_RNG_TYPE		EngineTestRng
#define CRYPTO_TEST_RNG_MEMSIZE		sizeof(EngineTestRng)
#define CRYPTO_TEST_RNG_CREATE(m, s)	EngineTestRngCreate((m), (s))
#endif

#ifndef CRYPTO_TEST_SOAK_AES
#define CRYPTO_TEST_SOAK_AES		200
#endif

#ifndef CRYPTO_TEST_SOAK_PKA
#define CRYPTO_TEST_SOAK_PKA		8
#endif

#if defined(UART_RETARGET) && !defined(CRYPTO_TEST_NO_MAIN)
#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCallback = NULL,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};

static UART s_Uart;
#endif

static int s_pass, s_fail;

alignas(16) static uint8_t s_HwAesMem[CRYPTO_TEST_AES_MEMSIZE];
alignas(CRYPTO_TEST_PKA_TYPE) static uint8_t s_HwPkaMem[CRYPTO_TEST_PKA_MEMSIZE];
alignas(16) static uint8_t s_HwRngMem[CRYPTO_TEST_RNG_MEMSIZE];
alignas(CryptoSoftAes) static uint8_t s_SwAesMem[CRYPTO_SOFTAES_MEMSIZE];
alignas(CryptoUecc) static uint8_t s_SwPkaMem[CRYPTO_UECC_MEMSIZE];
alignas(EngineTestRng) static uint8_t s_SwRngMem[sizeof(EngineTestRng)];

alignas(8) static uint8_t s_CtxA[CRYPTO_KEYCTX_MAX];
alignas(8) static uint8_t s_CtxB[CRYPTO_KEYCTX_MAX];

static void check(const char *name, bool ok)
{
	printf("  [%s] %s\r\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

// A direction the engine does not offer is reported, not failed
static void skip(const char *name)
{
	printf("  [SKIP] %s\r\n", name);
}

static int HexVal(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return c - 'A' + 10;
}

// Hex string to bytes, Len bytes
static void Hex(const char *pStr, uint8_t *pOut, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pOut[i] = (uint8_t)((HexVal(pStr[2 * i]) << 4) | HexVal(pStr[2 * i + 1]));
	}
}

static bool HexEq(const uint8_t *pData, const char *pStr, size_t Len)
{
	uint8_t tmp[64];

	if (Len > sizeof(tmp)) return false;
	Hex(pStr, tmp, Len);

	return memcmp(pData, tmp, Len) == 0;
}

// Test input generator, xorshift64. Not a key source.
static uint64_t s_Xs = 0x9E3779B97F4A7C15ULL;

static uint32_t XsNext(void)
{
	s_Xs ^= s_Xs << 13;
	s_Xs ^= s_Xs >> 7;
	s_Xs ^= s_Xs << 17;

	return (uint32_t)(s_Xs >> 16);
}

static void XsFill(uint8_t *pBuf, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pBuf[i] = (uint8_t)XsNext();
	}
}

static CryptoKey AesKey(const uint8_t *pKey, uint32_t Usage)
{
	CryptoKey k{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN, Usage, {}};

	k.Plain.pData = pKey;
	k.Plain.Len = 16;

	return k;
}

static CryptoKey EccKey(const uint8_t *pKey, uint32_t Usage)
{
	CryptoKey k{CRYPTO_KEY_ECC_P256, CRYPTO_KEY_LOC_PLAIN, Usage, {}};

	k.Plain.pData = pKey;
	k.Plain.Len = 32;

	return k;
}

static const uint8_t s_Spkey[16] = {
	0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
static const uint8_t s_Spmsg[64] = {
	0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
	0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
	0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
	0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10 };

static const char *s_EcbHex =
	"3ad77bb40d7a3660a89ecaf32466ef97f5d3d58503b9699de785895a96fdbaaf"
	"43b1cd7f598ece23881b00e3ed0306887b0c785e27e8ad3f8223207104725dd4";
static const char *s_CbcHex =
	"7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b2"
	"73bed6b8e3c1743b7116e69e222295163ff1caa1681fac09120eca307586e1a7";
static const char *s_CtrHex =
	"874d6191b620e3261bef6864990db6ce9806f66b7970fdff8617187bb9fffdff"
	"5ae4df3edbd5d35e5b4f09020db03eab1e031dda2fbe03d1792170a0f3009cee";

#ifdef CRYPTO_TEST_AES256
static void Aes256Vectors(CipherEngine *pAes)
{
	printf("AES-256 known answers, SP 800-38A\r\n");
	uint8_t key[33], iv[16], expected[64], out[66];
	Hex("603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4", key + 1, 32);
	CryptoKey k{CRYPTO_KEY_AES_256, CRYPTO_KEY_LOC_PLAIN,
		CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	k.Plain.pData = key + 1; k.Plain.Len = 32;
	const CRYPTO_CIPHER_ALG modes[] = {CRYPTO_CIPHER_ECB, CRYPTO_CIPHER_CBC, CRYPTO_CIPHER_CTR};
	const char *vectors[] = {
		"f3eed1bdb5d2a03c064b5a7e3db181f8591ccb10d410ed26dc5ba74a31362870"
		"b6ed21b99ca6f4f9f153e7b1beafed1d23304b7a39f9f3ff067d8d8f9e24ecc7",
		"f58c4c04d6e5f1ba779eabfb5f7bfbd69cfc4e967edb808d679f777bc6702c7d"
		"39f23369a9d9bacfa530e26304231461b2eb05e2c39be9fcda6c19078c6a9d1b",
		"601ec313775789a5b7a7f504bbf3d228f443e3ca4d62b59aca84e990cacaf5c5"
		"2b0930daa23de94ce87017ba2d84988ddfc9c58db67aada613c2dd08457941a6"};
	const char *enc[] = {"AES-256 ECB encrypt", "AES-256 CBC encrypt", "AES-256 CTR encrypt"};
	const char *dec[] = {"AES-256 ECB decrypt", "AES-256 CBC decrypt", "AES-256 CTR decrypt"};
	const char *inplace[] = {"AES-256 ECB in-place and bounds", "AES-256 CBC in-place and bounds", "AES-256 CTR in-place and bounds"};
	for (unsigned m = 0; m < 3; m++)
	{
		for (unsigned i = 0; i < 16; i++) iv[i] = m == 2 ? 0xf0 + i : i;
		Hex(vectors[m], expected, 64);
		memset(out, 0xa5, sizeof(out));
		check(enc[m], pAes->Cipher(modes[m], 1, k, iv, 16, s_Spmsg, 64, out + 1) == CRYPTO_STATUS_OK &&
			memcmp(out + 1, expected, 64) == 0 && out[0] == 0xa5 && out[65] == 0xa5);
		check(dec[m], pAes->Cipher(modes[m], 0, k, iv, 16, expected, 64, out + 1) == CRYPTO_STATUS_OK &&
			memcmp(out + 1, s_Spmsg, 64) == 0);
		memcpy(out + 1, s_Spmsg, 64);
		bool ok = pAes->Cipher(modes[m], 1, k, iv, 16, out + 1, 64, out + 1) == CRYPTO_STATUS_OK &&
			memcmp(out + 1, expected, 64) == 0;
		ok = pAes->Cipher(modes[m], 0, k, iv, 16, out + 1, 64, out + 1) == CRYPTO_STATUS_OK &&
			memcmp(out + 1, s_Spmsg, 64) == 0 && out[0] == 0xa5 && out[65] == 0xa5 && ok;
		check(inplace[m], ok);
	}
	memset(out, 0xa5, sizeof(out));
	check("AES-256 CTR partial tail and bounds",
		pAes->Cipher(CRYPTO_CIPHER_CTR, 1, k, iv, 16, s_Spmsg, 21, out + 1) == CRYPTO_STATUS_OK &&
		memcmp(out + 1, expected, 21) == 0 && out[0] == 0xa5 && out[22] == 0xa5);
	bool policy = true;
	for (unsigned len = 0; len <= 33; len++)
	{
		if (len == 32) continue;
		k.Plain.Len = len;
		policy = pAes->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, s_Spmsg, 16, out) != CRYPTO_STATUS_OK && policy;
	}
	k.Plain.Len = 32; k.Usage = CRYPTO_KEY_USE_DECRYPT;
	policy = pAes->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, s_Spmsg, 16, out) != CRYPTO_STATUS_OK && policy;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT;
	policy = pAes->Cipher(CRYPTO_CIPHER_ECB, 0, k, nullptr, 0, s_Spmsg, 16, out) != CRYPTO_STATUS_OK && policy;
	check("AES-256 key length and usage policy", policy);
	check("AES-256 block length and IV policy",
		pAes->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, s_Spmsg, 21, out) != CRYPTO_STATUS_OK &&
		pAes->Cipher(CRYPTO_CIPHER_CBC, 1, k, iv, 16, s_Spmsg, 21, out) != CRYPTO_STATUS_OK &&
		pAes->Cipher(CRYPTO_CIPHER_CBC, 1, k, nullptr, 16, s_Spmsg, 16, out) != CRYPTO_STATUS_OK &&
		pAes->Cipher(CRYPTO_CIPHER_CTR, 1, k, iv, 15, s_Spmsg, 16, out) != CRYPTO_STATUS_OK);
	CryptoSecureWipe(key, sizeof(key));
}
#endif

static void AesVectors(CipherEngine *pAes, MacEngine *pMac, AeadEngine *pAead)
{
	uint8_t iv[16], ctr[16], out[64], back[64];
	CryptoKey ek = AesKey(s_Spkey, CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT);

	printf("AES known answers\r\n");

	for (int i = 0; i < 16; i++)
	{
		iv[i] = (uint8_t)i;
	}
	Hex("f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff", ctr, 16);

	memset(out, 0, sizeof(out));
	check("ECB encrypt, SP 800-38A F.1.1",
		pAes->Cipher(CRYPTO_CIPHER_ECB, 1, ek, nullptr, 0, s_Spmsg, 64, out) == CRYPTO_STATUS_OK &&
		HexEq(out, s_EcbHex, 64));
	memset(back, 0, sizeof(back));
	CRYPTO_STATUS st = pAes->Cipher(CRYPTO_CIPHER_ECB, 0, ek, nullptr, 0, out, 64, back);
	if (st == CRYPTO_STATUS_UNSUPPORTED)
	{
		skip("ECB decrypt, not offered by this engine");
	}
	else
	{
		check("ECB decrypt, SP 800-38A F.1.2",
			st == CRYPTO_STATUS_OK && memcmp(back, s_Spmsg, 64) == 0);
	}

	memset(out, 0, sizeof(out));
	check("CBC encrypt, SP 800-38A F.2.1",
		pAes->Cipher(CRYPTO_CIPHER_CBC, 1, ek, iv, 16, s_Spmsg, 64, out) == CRYPTO_STATUS_OK &&
		HexEq(out, s_CbcHex, 64));
	memset(back, 0, sizeof(back));
	st = pAes->Cipher(CRYPTO_CIPHER_CBC, 0, ek, iv, 16, out, 64, back);
	if (st == CRYPTO_STATUS_UNSUPPORTED)
	{
		skip("CBC decrypt, not offered by this engine");
	}
	else
	{
		check("CBC decrypt, SP 800-38A F.2.2",
			st == CRYPTO_STATUS_OK && memcmp(back, s_Spmsg, 64) == 0);
	}

	memset(out, 0, sizeof(out));
	check("CTR encrypt, SP 800-38A F.5.1",
		pAes->Cipher(CRYPTO_CIPHER_CTR, 1, ek, ctr, 16, s_Spmsg, 64, out) == CRYPTO_STATUS_OK &&
		HexEq(out, s_CtrHex, 64));
	memset(back, 0, sizeof(back));
	check("CTR decrypt, SP 800-38A F.5.2",
		pAes->Cipher(CRYPTO_CIPHER_CTR, 0, ek, ctr, 16, out, 64, back) == CRYPTO_STATUS_OK &&
		memcmp(back, s_Spmsg, 64) == 0);

	// Counter carry: the low 32 bits wrap, then the low 64, then all 128.
	// The peripheral counter may be narrower than the block, the engine has
	// to carry into the upper bytes like the software one.
	static const char *wrIv[3] = {
		"000102030405060708090a0bfffffffe",
		"0000000000000000ffffffffffffffff",
		"ffffffffffffffffffffffffffffffff" };
	static const char *wrOut[3] = {
		"633e3fa130ca6717f4ae606a9cb408ee139a4ab85772d5de62df811d333d72a5"
		"de3082d2379dec3b57868c898ff4ad8f125a7a4792bc6972",
		"84468955ad84651e0fba9085149428447227b194980a6ef3f19d0c0fd95860c2"
		"f5238a521e7fbc621accb03c591f56935286125b26da7ab8",
		"e13338e36cb71962e00d020b4cedbd86d3dae15b04bb352fa0f59febfcb4da3e"
		"67da610697ed5aae4b0fa7a0dd783d2961a00ab697367915" };
	static const char *wrName[3] = {
		"CTR carry out of the low 32 bits",
		"CTR carry out of the low 64 bits",
		"CTR counter wraps to zero" };

	for (int i = 0; i < 3; i++)
	{
		Hex(wrIv[i], ctr, 16);
		memset(out, 0, sizeof(out));
		check(wrName[i],
			pAes->Cipher(CRYPTO_CIPHER_CTR, 1, ek, ctr, 16, s_Spmsg, 56, out) == CRYPTO_STATUS_OK &&
			HexEq(out, wrOut[i], 56));
	}

	// Lengths that are not a block multiple, CTR only
	memset(out, 0, sizeof(out));
	Hex("f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff", ctr, 16);
	check("CTR length 21 keeps the tail and stops at 21",
		pAes->Cipher(CRYPTO_CIPHER_CTR, 1, ek, ctr, 16, s_Spmsg, 21, out) == CRYPTO_STATUS_OK &&
		HexEq(out, "874d6191b620e3261bef6864990db6ce9806f66b79", 21) && out[21] == 0U);

	check("ECB refuses a length that is not a block multiple",
		pAes->Cipher(CRYPTO_CIPHER_ECB, 1, ek, nullptr, 0, s_Spmsg, 20, out) != CRYPTO_STATUS_OK);

	CryptoKey denied = AesKey(s_Spkey, CRYPTO_KEY_USE_DERIVE);
	check("cipher rejects a key without encrypt usage",
		pAes->Cipher(CRYPTO_CIPHER_ECB, 1, denied, nullptr, 0, s_Spmsg, 16, out) ==
			CRYPTO_STATUS_UNSUPPORTED);

	// CMAC, RFC 4493
	static const char *macHex[4] = {
		"bb1d6929e95937287fa37d129b756746", "070a16b46b4d4144f79bdd9dd04a287c",
		"dfa66747de9ae63030ca32611497c827", "51f0bebf7e3b9d92fc49741779363cfe" };
	static const size_t macLen[4] = { 0, 16, 40, 64 };
	CryptoKey sk = AesKey(s_Spkey, CRYPTO_KEY_USE_SIGN);
	uint8_t tag[16];
	bool cmac = true;

	for (int i = 0; i < 4; i++)
	{
		memset(tag, 0, sizeof(tag));
		cmac = cmac &&
			pMac->Mac(CRYPTO_MAC_CMAC, sk, s_Spmsg, macLen[i], tag, 16) == CRYPTO_STATUS_OK &&
			HexEq(tag, macHex[i], 16);
	}
	check("CMAC, RFC 4493 lengths 0, 16, 40, 64", cmac);

	// CCM, RFC 3610 packet vector 1
	static const uint8_t ccmKey[16] = {0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,
									   0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF};
	static const uint8_t ccmNonce[13] = {0,0,0,3,2,1,0,0xA0,0xA1,0xA2,0xA3,0xA4,0xA5};
	uint8_t aad[8], msg[23], ct[23], t[16], dec[23];
	CryptoKey ak = AesKey(ccmKey, CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT);

	for (int i = 0; i < 8; i++) aad[i] = (uint8_t)i;
	for (int i = 0; i < 23; i++) msg[i] = (uint8_t)(i + 8);

	memset(ct, 0, sizeof(ct));
	memset(t, 0, sizeof(t));
	check("CCM seal, RFC 3610 vector 1",
		pAead->Seal(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, aad, 8, msg, 23, ct, t, 8) ==
			CRYPTO_STATUS_OK &&
		HexEq(ct, "588c979a61c663d2f066d0c2c0f989806d5f6b61dac384", 23) &&
		HexEq(t, "17e8d12cfdf926e0", 8));
	check("CCM open recovers the message",
		pAead->Open(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, aad, 8, ct, 23, dec, t, 8) ==
			CRYPTO_STATUS_OK && memcmp(dec, msg, 23) == 0);
	t[0] ^= 1U;
	memset(dec, 0xA5, sizeof(dec));
	check("CCM open rejects a bad tag and clears the output",
		pAead->Open(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, aad, 8, ct, 23, dec, t, 8) ==
			CRYPTO_STATUS_FAIL && dec[0] == 0U && dec[22] == 0U);

	// GCM, McGrew and Viega test case 4
	uint8_t gk[16], giv[12], gaad[20], gmsg[60], gct[60], gt[16], gdec[60];

	Hex("feffe9928665731c6d6a8f9467308308", gk, 16);
	Hex("cafebabefacedbaddecaf888", giv, 12);
	Hex("feedfacedeadbeeffeedfacedeadbeefabaddad2", gaad, 20);
	Hex("d9313225f88406e5a55909c5aff5269a86a7a9531534f7da2e4c303d8a318a72"
		"1c3c0c95956809532fcf0e2449a6b525b16aedf5aa0de657ba637b39", gmsg, 60);
	CryptoKey gkd = AesKey(gk, CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT);

	memset(gct, 0, sizeof(gct));
	memset(gt, 0, sizeof(gt));
	check("GCM seal, test case 4",
		pAead->Seal(CRYPTO_AEAD_AES_GCM, gkd, giv, 12, gaad, 20, gmsg, 60, gct, gt, 16) ==
			CRYPTO_STATUS_OK &&
		HexEq(gct, "42831ec2217774244b7221b784d0d49ce3aa212f2c02a4e035c17e2329aca12e"
				   "21d514b25466931c7d8f6a5aac84aa051ba30b396a0aac973d58e091", 60) &&
		HexEq(gt, "5bc94fbc3221a5db94fae95ae7121a47", 16));
	check("GCM open recovers the message",
		pAead->Open(CRYPTO_AEAD_AES_GCM, gkd, giv, 12, gaad, 20, gct, 60, gdec, gt, 16) ==
			CRYPTO_STATUS_OK && memcmp(gdec, gmsg, 60) == 0);
	gt[15] ^= 1U;
	memset(gdec, 0xA5, sizeof(gdec));
	check("GCM open rejects a bad tag and clears the output",
		pAead->Open(CRYPTO_AEAD_AES_GCM, gkd, giv, 12, gaad, 20, gct, 60, gdec, gt, 16) ==
			CRYPTO_STATUS_FAIL && gdec[0] == 0U && gdec[59] == 0U);
}

// Software fallback must offer both directions even without an accelerator.
static void SoftwareAesDecryptChecks(CryptoSoftAes *pAes)
{
	printf("Software ECB/CBC decryption\r\n");
	uint8_t iv[16], cipher[64], buf[66], expected[66];
	for (int i = 0; i < 16; i++) iv[i] = (uint8_t)i;
	CryptoKey key = AesKey(s_Spkey, CRYPTO_KEY_USE_DECRYPT);
	for (int mode = 0; mode < 2; mode++)
	{
		CRYPTO_CIPHER_ALG alg = mode == 0 ? CRYPTO_CIPHER_ECB : CRYPTO_CIPHER_CBC;
		Hex(mode == 0 ? s_EcbHex : s_CbcHex, cipher, sizeof(cipher));
		memset(buf, 0xA5, sizeof(buf));
		check(mode == 0 ? "software ECB known-answer decrypt" : "software CBC known-answer decrypt",
			pAes->Cipher(alg, 0, key, iv, sizeof(iv), cipher, 64, buf + 1) == CRYPTO_STATUS_OK &&
			memcmp(buf + 1, s_Spmsg, 64) == 0 && buf[0] == 0xA5 && buf[65] == 0xA5);
		memcpy(buf + 1, cipher, 64);
		check(mode == 0 ? "software ECB in-place decrypt" : "software CBC in-place decrypt",
			pAes->Cipher(alg, 0, key, iv, sizeof(iv), buf + 1, 64, buf + 1) == CRYPTO_STATUS_OK &&
			memcmp(buf + 1, s_Spmsg, 64) == 0 && buf[0] == 0xA5 && buf[65] == 0xA5);
		memset(buf, 0xA5, sizeof(buf));
		memcpy(expected, buf, sizeof(buf));
		bool ok = pAes->Cipher(alg, 0, key, iv, sizeof(iv), cipher, 0, buf + 1) == CRYPTO_STATUS_OK;
		ok = ok && pAes->Cipher(alg, 0, key, iv, sizeof(iv), cipher, 63, buf + 1) == CRYPTO_STATUS_FAIL;
		CryptoKey denied = AesKey(s_Spkey, CRYPTO_KEY_USE_ENCRYPT);
		ok = ok && pAes->Cipher(alg, 0, denied, iv, sizeof(iv), cipher, 64, buf + 1) == CRYPTO_STATUS_UNSUPPORTED;
		if (mode != 0)
		{
			ok = ok && pAes->Cipher(alg, 0, key, nullptr, 16, cipher, 64, buf + 1) == CRYPTO_STATUS_FAIL;
			ok = ok && pAes->Cipher(alg, 0, key, iv, 15, cipher, 64, buf + 1) == CRYPTO_STATUS_FAIL;
		}
		check(mode == 0 ? "software ECB length and key policy" : "software CBC length, IV and key policy",
			ok && memcmp(buf, expected, sizeof(buf)) == 0);
	}
}

// Engine against the software one on random keys, data and lengths
static void AesSoak(CipherEngine *pHw, MacEngine *pHwMac, AeadEngine *pHwAead,
					CipherEngine *pSw, MacEngine *pSwMac, AeadEngine *pSwAead)
{
	int bad = 0;
	int n = CRYPTO_TEST_SOAK_AES;

	printf("AES against the software engine, %d random rounds\r\n", n);

	for (int r = 0; r < n; r++)
	{
		uint8_t key[16], iv[16], in[96], a[96], b[96], ta[16], tb[16], nonce[13], aad[24];
		size_t len = XsNext() % 97U;
		size_t blk = (len / 16U) * 16U;
		CryptoKey k = AesKey(key, CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT | CRYPTO_KEY_USE_SIGN);

		XsFill(key, 16);
		XsFill(iv, 16);
		XsFill(in, sizeof(in));
		XsFill(nonce, sizeof(nonce));
		XsFill(aad, sizeof(aad));
		if ((r & 3) == 0)
		{
			// Counter close to a carry
			memset(iv + 12, 0xFF, 3);
			iv[15] = (uint8_t)(0xF0U + (XsNext() & 0xFU));
		}
		if ((r & 7) == 1)
		{
			memset(iv + 8, 0xFF, 8);
		}

		// CTR, any length
		memset(a, 0, sizeof(a));
		memset(b, 0, sizeof(b));
		if (pHw->Cipher(CRYPTO_CIPHER_CTR, 1, k, iv, 16, in, len, a) != CRYPTO_STATUS_OK ||
			pSw->Cipher(CRYPTO_CIPHER_CTR, 1, k, iv, 16, in, len, b) != CRYPTO_STATUS_OK ||
			memcmp(a, b, len) != 0)
		{
			bad++;
			continue;
		}

		// CBC and ECB encrypt, whole blocks
		if (blk > 0U)
		{
			memset(a, 0, sizeof(a));
			memset(b, 0, sizeof(b));
			if (pHw->Cipher(CRYPTO_CIPHER_CBC, 1, k, iv, 16, in, blk, a) != CRYPTO_STATUS_OK ||
				pSw->Cipher(CRYPTO_CIPHER_CBC, 1, k, iv, 16, in, blk, b) != CRYPTO_STATUS_OK ||
				memcmp(a, b, blk) != 0)
			{
				bad++;
				continue;
			}
			// Decrypt back on the engine under test, when it offers it
			CRYPTO_STATUS ds = pHw->Cipher(CRYPTO_CIPHER_CBC, 0, k, iv, 16, a, blk, b);
			if (ds != CRYPTO_STATUS_UNSUPPORTED &&
				(ds != CRYPTO_STATUS_OK || memcmp(b, in, blk) != 0))
			{
				bad++;
				continue;
			}
			if (pHw->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, blk, a) != CRYPTO_STATUS_OK ||
				pSw->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, blk, b) != CRYPTO_STATUS_OK ||
				memcmp(a, b, blk) != 0)
			{
				bad++;
				continue;
			}
			ds = pHw->Cipher(CRYPTO_CIPHER_ECB, 0, k, nullptr, 0, a, blk, b);
			if (ds != CRYPTO_STATUS_UNSUPPORTED &&
				(ds != CRYPTO_STATUS_OK || memcmp(b, in, blk) != 0))
			{
				bad++;
				continue;
			}
		}

		// CMAC
		memset(ta, 0, sizeof(ta));
		memset(tb, 0, sizeof(tb));
		if (pHwMac->Mac(CRYPTO_MAC_CMAC, k, in, len, ta, 16) != CRYPTO_STATUS_OK ||
			pSwMac->Mac(CRYPTO_MAC_CMAC, k, in, len, tb, 16) != CRYPTO_STATUS_OK ||
			memcmp(ta, tb, 16) != 0)
		{
			bad++;
			continue;
		}

		// CCM and GCM
		size_t alen = XsNext() % 25U;
		memset(a, 0, sizeof(a));
		memset(b, 0, sizeof(b));
		memset(ta, 0, sizeof(ta));
		memset(tb, 0, sizeof(tb));
		if (pHwAead->Seal(CRYPTO_AEAD_AES_CCM, k, nonce, 13, aad, alen, in, len, a, ta, 8) != CRYPTO_STATUS_OK ||
			pSwAead->Seal(CRYPTO_AEAD_AES_CCM, k, nonce, 13, aad, alen, in, len, b, tb, 8) != CRYPTO_STATUS_OK ||
			memcmp(a, b, len) != 0 || memcmp(ta, tb, 8) != 0)
		{
			bad++;
			continue;
		}
		memset(a, 0, sizeof(a));
		memset(b, 0, sizeof(b));
		memset(ta, 0, sizeof(ta));
		memset(tb, 0, sizeof(tb));
		if (pHwAead->Seal(CRYPTO_AEAD_AES_GCM, k, nonce, 12, aad, alen, in, len, a, ta, 16) != CRYPTO_STATUS_OK ||
			pSwAead->Seal(CRYPTO_AEAD_AES_GCM, k, nonce, 12, aad, alen, in, len, b, tb, 16) != CRYPTO_STATUS_OK ||
			memcmp(a, b, len) != 0 || memcmp(ta, tb, 16) != 0)
		{
			bad++;
			continue;
		}
		// The engine under test must open what the reference sealed
		if (pHwAead->Open(CRYPTO_AEAD_AES_GCM, k, nonce, 12, aad, alen, b, len, a, tb, 16) != CRYPTO_STATUS_OK ||
			memcmp(a, in, len) != 0)
		{
			bad++;
		}
	}

	check("CTR, CBC, ECB, CMAC, CCM and GCM equal the software engine", bad == 0);
	if (bad != 0)
	{
		printf("    %d of %d rounds differ\r\n", bad, n);
	}
}

static const char s_PrivA[] = "3f49f6d4a3c55f3874c9b3e3d2103f504aff607beb40b7995899b8a6cd3c1abd";
static const char s_PubA[] =
	"20b003d2f297be2c5e2c83a7e9f9a5b9eff49111acf4fddbcc0301480e359de6"
	"dc809c49652aeb6d63329abf5a52155c766345c28fed3024741c8ed01589d28b";
static const char s_PrivB[] = "55188b3d32f6bb9a900afcfbeed4e72a59cb9ac2f19d7cfb6b4fdd49f47fc5fd";
static const char s_PubB[] =
	"1ea1f0f01faf1d9609592284f19e4c0047b58afd8615a69f559077b22faaa190"
	"4c55f33e429dad377356703a9ab85160472d1130e28e36765f89aff915b1214a";
static const char s_Dh[] = "ec0234a357c8ad05341010a60a397d9b99796b13b4f866f1868d34f373bfa698";

static void PkaVectors(KeyAgreeEngine *pKa, SignEngine *pSign, SignEngine *pRefSign)
{
	uint8_t privA[32], privB[32], pub[64], peer[64], dh[32], pubA2[64], pubB2[64];

	printf("P-256 known answers\r\n");

	Hex(s_PrivA, privA, 32);
	Hex(s_PrivB, privB, 32);

	memset(pub, 0, sizeof(pub));
	check("public key of Bluetooth sample key A (scalar multiplication)",
		pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, privA, pub) == CRYPTO_STATUS_OK &&
		HexEq(pub, s_PubA, 64));
	pKa->KeyReset(s_CtxA);
	memset(pub, 0, sizeof(pub));
	check("public key of Bluetooth sample key B",
		pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxB, privB, pub) == CRYPTO_STATUS_OK &&
		HexEq(pub, s_PubB, 64));
	pKa->KeyReset(s_CtxB);

	Hex(s_PubB, peer, 64);
	pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, privA, pubA2);
	memset(dh, 0, sizeof(dh));
	check("ECDH shared secret, sample key A with peer B",
		pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, peer, dh) == CRYPTO_STATUS_OK &&
		HexEq(dh, s_Dh, 32));
	check("a used key is consumed",
		pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, peer, dh) != CRYPTO_STATUS_OK);

	Hex(s_PubA, peer, 64);
	pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxB, privB, pubB2);
	memset(dh, 0, sizeof(dh));
	check("ECDH shared secret, sample key B with peer A",
		pKa->Agree(CRYPTO_CURVE_P256, s_CtxB, peer, dh) == CRYPTO_STATUS_OK &&
		HexEq(dh, s_Dh, 32));

	// Peer point checks
	Hex(s_PubB, peer, 64);
	pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, privA, pubA2);
	peer[63] ^= 1U;
	memset(dh, 0xA5, sizeof(dh));
	check("peer point off the curve is rejected, secret cleared",
		pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, peer, dh) == CRYPTO_STATUS_FAIL &&
		dh[0] == 0U && dh[31] == 0U);
	pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, privA, pubA2);
	memset(peer, 0, sizeof(peer));
	check("peer point at infinity (0, 0) is rejected",
		pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, peer, dh) == CRYPTO_STATUS_FAIL);
	pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, privA, pubA2);
	memset(peer, 0xFF, sizeof(peer));
	check("peer coordinates above the prime are rejected",
		pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, peer, dh) == CRYPTO_STATUS_FAIL);

	// Private scalar range
	uint8_t bad[32];

	memset(bad, 0, sizeof(bad));
	check("private scalar 0 is rejected",
		pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, bad, pub) != CRYPTO_STATUS_OK);
	Hex("ffffffff00000000ffffffffffffffffbce6faada7179e84f3b9cac2fc632551", bad, 32);
	check("private scalar n is rejected",
		pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, bad, pub) != CRYPTO_STATUS_OK);
	Hex("ffffffff00000000fffffffffffffffe000000000000000000000000000000ff", bad, 32);
	memset(pub, 0, sizeof(pub));
	check("large private scalar below n",
		pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, bad, pub) == CRYPTO_STATUS_OK &&
		HexEq(pub, "b28d398d419c76007e1b222337768b8cab4f62e4b2542eda6c55c95b515a953c"
				   "799fcb93fcdb2d95eca09a6c3222380f6bbc4a4b9fda1ca57c0f6fdae47a2b71", 64));
	pKa->KeyReset(s_CtxA);

	// Both ends of the valid private-scalar range must work, without skips.
	for (unsigned edge = 0; edge < 2U; edge++)
	{
		if (edge == 0U)
		{
			memset(bad, 0, sizeof(bad));
			bad[31] = 1U;
		}
		else
		{
			Hex("ffffffff00000000ffffffffffffffffbce6faada7179e84f3b9cac2fc632550", bad, 32);
		}
		memset(pub, 0, sizeof(pub));
		check(edge == 0U ? "private scalar 1 gives G" : "private scalar n - 1 gives -G",
			pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, bad, pub) == CRYPTO_STATUS_OK &&
			HexEq(pub, "6b17d1f2e12c4247f8bce6e563a440f277037d812deb33a0f4a13945d898c296", 32) &&
			HexEq(pub + 32, edge == 0U ?
				"4fe342e2fe1a7f9b8ee7eb4a7c0f9e162bce33576b315ececbb6406837bf51f5" :
				"b01cbd1c01e58065711814b583f061e9d431cca994cea1313449bf97c840ae0a", 32));
		Hex(s_PubB, peer, 64);
		check("boundary scalar ECDH keeps key",
			pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, peer, dh, true) == CRYPTO_STATUS_OK &&
			memcmp(dh, peer, 32) == 0);
		check("boundary scalar ECDH consumes key",
			pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, peer, dh) == CRYPTO_STATUS_OK &&
			memcmp(dh, peer, 32) == 0 &&
			pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, peer, dh) != CRYPTO_STATUS_OK);
		uint8_t edgeHash[32], edgeSig[64];
		XsFill(edgeHash, sizeof(edgeHash));
		CryptoKey edgeKey = EccKey(bad, CRYPTO_KEY_USE_SIGN);
		check("boundary scalar signature verifies",
			pSign->Sign(CRYPTO_CURVE_P256, edgeKey, edgeHash, sizeof(edgeHash), edgeSig) == CRYPTO_STATUS_OK &&
			pSign->Verify(CRYPTO_CURVE_P256, pub, edgeHash, sizeof(edgeHash), edgeSig) == CRYPTO_STATUS_OK &&
			pRefSign->Verify(CRYPTO_CURVE_P256, pub, edgeHash, sizeof(edgeHash), edgeSig) == CRYPTO_STATUS_OK);
		edgeHash[0] ^= 1U;
		check("boundary scalar signature rejects changed hash",
			pSign->Verify(CRYPTO_CURVE_P256, pub, edgeHash, sizeof(edgeHash), edgeSig) != CRYPTO_STATUS_OK);
		// Fixed OpenSSL signatures over a 256-bit all-ones digest exercise
		// hash reduction independently of this engine's signing routine.
		memset(edgeHash, 0xFF, sizeof(edgeHash));
		Hex(edge == 0U ?
			"d2709b43fd982ab3cf008b4c7578fc1b840c6b07a1217cf975697c39b8d3f02d3"
			"dddc66768835440e21b8e40dbae70b48cd7969e0604c514b5b43ef9bc762da4" :
			"ba071a88f3c32f76254ba18a933686c6abe1130867db5000e648a66e6acfb7efbd"
			"44bc482df1bde6d7d0b80d040a5debf2274b87cd0371fa723217d481490259", edgeSig, 64);
		check("boundary scalar OpenSSL signature",
			pSign->Verify(CRYPTO_CURVE_P256, pub, edgeHash, sizeof(edgeHash), edgeSig) == CRYPTO_STATUS_OK &&
			pRefSign->Verify(CRYPTO_CURVE_P256, pub, edgeHash, sizeof(edgeHash), edgeSig) == CRYPTO_STATUS_OK);
		check("boundary scalar import without public output",
			pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, bad, nullptr) == CRYPTO_STATUS_OK);
		peer[63] ^= 1U;
		memset(dh, 0xA5, sizeof(dh));
		check("boundary scalar rejects invalid peer and clears secret",
			pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, peer, dh) == CRYPTO_STATUS_FAIL &&
			P256IsZero(dh, sizeof(dh)));
		pKa->KeyReset(s_CtxA);
	}

	// ECDSA on the engine, checked by the reference verify, and the other way
	uint8_t hash[32], sig[64], sigRef[64], pubKey[64];

	XsFill(hash, sizeof(hash));
	Hex(s_PubA, pubKey, 64);
	CryptoKey sk = EccKey(privA, CRYPTO_KEY_USE_SIGN);

	memset(sig, 0, sizeof(sig));
	check("ECDSA sign",
		pSign->Sign(CRYPTO_CURVE_P256, sk, hash, 32, sig) == CRYPTO_STATUS_OK);
	check("ECDSA verify, own signature",
		pSign->Verify(CRYPTO_CURVE_P256, pubKey, hash, 32, sig) == CRYPTO_STATUS_OK);
	check("ECDSA signature verifies on the software engine",
		pRefSign->Verify(CRYPTO_CURVE_P256, pubKey, hash, 32, sig) == CRYPTO_STATUS_OK);
	check("ECDSA software signature verifies on the engine",
		pRefSign->Sign(CRYPTO_CURVE_P256, sk, hash, 32, sigRef) == CRYPTO_STATUS_OK &&
		pSign->Verify(CRYPTO_CURVE_P256, pubKey, hash, 32, sigRef) == CRYPTO_STATUS_OK);

	sig[10] ^= 1U;
	check("ECDSA verify rejects a changed r",
		pSign->Verify(CRYPTO_CURVE_P256, pubKey, hash, 32, sig) == CRYPTO_STATUS_FAIL);
	sig[10] ^= 1U;
	sig[40] ^= 1U;
	check("ECDSA verify rejects a changed s",
		pSign->Verify(CRYPTO_CURVE_P256, pubKey, hash, 32, sig) == CRYPTO_STATUS_FAIL);
	sig[40] ^= 1U;
	hash[0] ^= 1U;
	check("ECDSA verify rejects a changed hash",
		pSign->Verify(CRYPTO_CURVE_P256, pubKey, hash, 32, sig) == CRYPTO_STATUS_FAIL);
	hash[0] ^= 1U;
	memset(sig, 0, 32);
	check("ECDSA verify rejects r = 0",
		pSign->Verify(CRYPTO_CURVE_P256, pubKey, hash, 32, sig) == CRYPTO_STATUS_FAIL);
}

static uint32_t Ticks(void)
{
#ifdef CRYPTO_TEST_TICKS
	return (uint32_t)CRYPTO_TEST_TICKS();
#else
	return 0U;
#endif
}

static void PkaSoak(KeyAgreeEngine *pKa, SignEngine *pSign, KeyAgreeEngine *pRefKa,
					SignEngine *pRefSign)
{
	int n = CRYPTO_TEST_SOAK_PKA;
	int badPub = 0, badDh = 0, badSig = 0;
	uint32_t hwTicks = 0, swTicks = 0;
	int timed = 0;

	printf("P-256 against the software engine, %d random rounds\r\n", n);

	for (int r = 0; r < n; r++)
	{
		uint8_t priv[32], privB[32], pubHw[64], pubSw[64], pubPeer[64], dhHw[32], dhSw[32];
		uint8_t hash[32], sig[64];

		do
		{
			XsFill(priv, sizeof(priv));
			XsFill(privB, sizeof(privB));
			// Keep the scalars below n: top byte under 0xFF
			priv[0] &= 0x7F;
			privB[0] &= 0x7F;
		} while (priv[0] == 0U || privB[0] == 0U);

		uint32_t t0 = Ticks();
		CRYPTO_STATUS s1 = pKa->KeyImport(CRYPTO_CURVE_P256, s_CtxA, priv, pubHw);
		uint32_t t1 = Ticks();
		CRYPTO_STATUS s2 = pRefKa->KeyImport(CRYPTO_CURVE_P256, s_CtxB, priv, pubSw);
		uint32_t t2 = Ticks();

		if (s1 != CRYPTO_STATUS_OK || s2 != CRYPTO_STATUS_OK || memcmp(pubHw, pubSw, 64) != 0)
		{
			badPub++;
			pKa->KeyReset(s_CtxA);
			pRefKa->KeyReset(s_CtxB);
			continue;
		}
		pRefKa->KeyReset(s_CtxB);
		hwTicks += t1 - t0;
		swTicks += t2 - t1;
		timed++;

		// Shared secret with a second random key, both directions
		if (pRefKa->KeyImport(CRYPTO_CURVE_P256, s_CtxB, privB, pubPeer) != CRYPTO_STATUS_OK)
		{
			badDh++;
			pKa->KeyReset(s_CtxA);
			continue;
		}
		memset(dhHw, 0, sizeof(dhHw));
		memset(dhSw, 0, sizeof(dhSw));
		CRYPTO_STATUS d1 = pKa->Agree(CRYPTO_CURVE_P256, s_CtxA, pubPeer, dhHw);
		CRYPTO_STATUS d2 = pRefKa->Agree(CRYPTO_CURVE_P256, s_CtxB, pubHw, dhSw);
		if (d1 != CRYPTO_STATUS_OK || d2 != CRYPTO_STATUS_OK || memcmp(dhHw, dhSw, 32) != 0)
		{
			badDh++;
		}

		// Signatures cross-checked
		XsFill(hash, sizeof(hash));
		CryptoKey k = EccKey(priv, CRYPTO_KEY_USE_SIGN);
		if (pSign->Sign(CRYPTO_CURVE_P256, k, hash, 32, sig) != CRYPTO_STATUS_OK ||
			pRefSign->Verify(CRYPTO_CURVE_P256, pubHw, hash, 32, sig) != CRYPTO_STATUS_OK ||
			pSign->Verify(CRYPTO_CURVE_P256, pubHw, hash, 32, sig) != CRYPTO_STATUS_OK ||
			pRefSign->Sign(CRYPTO_CURVE_P256, k, hash, 32, sig) != CRYPTO_STATUS_OK ||
			pSign->Verify(CRYPTO_CURVE_P256, pubHw, hash, 32, sig) != CRYPTO_STATUS_OK)
		{
			badSig++;
		}
	}

	check("public keys equal the software engine", badPub == 0);
	check("shared secrets equal the software engine", badDh == 0);
	check("signatures verify across the two engines", badSig == 0);

#ifdef CRYPTO_TEST_TICKS
	if (timed > 0)
	{
		uint32_t perUs = (uint32_t)CRYPTO_TEST_TICKS_PER_US;

		if (perUs == 0U)
		{
			perUs = 1U;
		}
		uint32_t hwUs = hwTicks / (uint32_t)timed / perUs;
		uint32_t swUs = swTicks / (uint32_t)timed / perUs;

		printf("  P-256 scalar multiplication, mean of %d: engine %lu us, software %lu us\r\n",
			timed, (unsigned long)hwUs, (unsigned long)swUs);
	}
#else
	(void)timed;
	(void)hwTicks;
	(void)swTicks;
#endif
}

static void RngChecks(RngEngine *pRng)
{
	uint8_t a[32], b[32], big[1024];
	uint32_t seen[256];
	int missing = 0;

	printf("Random source\r\n");

	memset(a, 0, sizeof(a));
	memset(b, 0, sizeof(b));
	check("draws succeed",
		pRng->Random(a, 32) == CRYPTO_STATUS_OK && pRng->Random(b, 32) == CRYPTO_STATUS_OK);
	check("two draws differ", memcmp(a, b, 32) != 0);

	uint8_t odd[19];

	memset(odd, 0xCC, sizeof(odd));
	check("odd length request stays inside its length",
		pRng->Random(odd, 17) == CRYPTO_STATUS_OK && odd[17] == 0xCC && odd[18] == 0xCC);

	memset(seen, 0, sizeof(seen));
	uint32_t ones = 0;
	for (int rep = 0; rep < 4; rep++)
	{
		if (pRng->Random(big, sizeof(big)) != CRYPTO_STATUS_OK)
		{
			missing = 256;
			break;
		}
		for (size_t i = 0; i < sizeof(big); i++)
		{
			seen[big[i]]++;
			for (int bit = 0; bit < 8; bit++)
			{
				ones += (big[i] >> bit) & 1U;
			}
		}
	}
	for (int v = 0; v < 256; v++)
	{
		if (seen[v] == 0U) missing++;
	}
	// 32768 bits: the ones count is within 5 sigma (about 450) of 16384
	check("4 KB sample: ones within 5 sigma and byte values spread",
		ones > 16384U - 450U && ones < 16384U + 450U && missing < 8);

	check("source reports itself as security grade", pRng->IsSecure());
}

#ifdef CRYPTO_TEST_HASH_TYPE
// Optional hash facet, selected by the board like AES/PKA/RNG.
static void HashChecks()
{
	alignas(CRYPTO_TEST_HASH_TYPE) static uint8_t mem[CRYPTO_TEST_HASH_MEMSIZE];
	auto *hash = CRYPTO_TEST_HASH_CREATE(mem, sizeof(mem));
	check("SHA-256 engine constructs", hash != nullptr);
	if (hash == nullptr) return;
	check("SHA-256/HMAC self-test", hash->SelfTest() == 0);
	CryptoSoftSha256 reference;
	reference.Enable();
	uint8_t msg[258], expected[32], actual[34];
	XsFill(msg, sizeof(msg));
	bool match = true;
	// Every tail length, both padding boundaries, multi-block and unaligned IO.
	for (size_t len = 0; len <= 257U; len++)
	{
		memset(actual, 0xA5, sizeof(actual));
		match &= reference.Hash(CRYPTO_HASH_SHA256, msg + 1, len, expected) == CRYPTO_STATUS_OK;
		match &= hash->Hash(CRYPTO_HASH_SHA256, msg + 1, len, actual + 1) == CRYPTO_STATUS_OK &&
			memcmp(expected, actual + 1, 32) == 0 && actual[0] == 0xA5 && actual[33] == 0xA5;
	}
	check("SHA-256 lengths 0..257 match software, unaligned buffers bounded", match);
	uint8_t key[133];
	memset(key, 0xaa, sizeof(key));
	CryptoKey mk{CRYPTO_KEY_HMAC, CRYPTO_KEY_LOC_PLAIN, CRYPTO_KEY_USE_SIGN, {}};
	mk.Plain.pData = key + 1; mk.Plain.Len = 131;
	const char *longMsg = "Test Using Larger Than Block-Size Key - Hash Key First";
	check("HMAC-SHA-256 long key, RFC 4231 case 6",
		hash->Mac(CRYPTO_MAC_HMAC, mk, (const uint8_t *)longMsg, strlen(longMsg), actual + 1, 32) == CRYPTO_STATUS_OK &&
		HexEq(actual + 1, "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54", 32));
	memset(key, 0x0c, 20); mk.Plain.pData = key; mk.Plain.Len = 20;
	memset(actual, 0xa5, sizeof(actual));
	check("HMAC-SHA-256 truncation, RFC 4231 case 5",
		hash->Mac(CRYPTO_MAC_HMAC, mk, (const uint8_t *)"Test With Truncation", 20, actual + 1, 16) == CRYPTO_STATUS_OK &&
		HexEq(actual + 1, "a3b6167473100ee06e0c796c2955552b", 16) && actual[0] == 0xa5 && actual[17] == 0xa5);
	XsFill(key, sizeof(key));
	const size_t keyLengths[] = {0, 1, 63, 64, 65, 131};
	match = true;
	for (size_t kl : keyLengths)
	{
		mk.Plain.pData = kl == 0 ? nullptr : key + 1; mk.Plain.Len = kl;
		for (size_t len = 0; len <= 257; len++)
		{
			const uint8_t *input = len == 0 ? nullptr : msg + 1;
			size_t ml = 1 + len % 32;
			memset(actual, 0xa5, sizeof(actual));
			match &= reference.Mac(CRYPTO_MAC_HMAC, mk, input, len, expected, ml) == CRYPTO_STATUS_OK;
			match &= hash->Mac(CRYPTO_MAC_HMAC, mk, input, len, actual + 1, ml) == CRYPTO_STATUS_OK &&
				memcmp(expected, actual + 1, ml) == 0 && actual[0] == 0xa5 && actual[ml + 1] == 0xa5;
		}
	}
	check("HMAC key boundaries, message lengths 0..257 and output bounds", match);
	mk.Plain.pData = key; mk.Plain.Len = 32;
	match = reference.Mac(CRYPTO_MAC_HMAC, mk, msg + 1, 257, expected, 32) == CRYPTO_STATUS_OK;
	match &= hash->Mac(CRYPTO_MAC_HMAC, mk, msg + 1, 257, msg + 1, 32) == CRYPTO_STATUS_OK &&
		memcmp(expected, msg + 1, 32) == 0;
	check("HMAC output may overlap message", match);
	mk.Usage = CRYPTO_KEY_USE_ENCRYPT;
	match = hash->Mac(CRYPTO_MAC_HMAC, mk, msg, 16, actual, 32) != CRYPTO_STATUS_OK;
	mk.Usage = CRYPTO_KEY_USE_SIGN;
	match &= hash->Mac(CRYPTO_MAC_HMAC, mk, nullptr, 1, actual, 32) != CRYPTO_STATUS_OK;
	match &= hash->Mac(CRYPTO_MAC_HMAC, mk, msg, 16, actual, 0) != CRYPTO_STATUS_OK;
	match &= hash->Mac(CRYPTO_MAC_HMAC, mk, msg, 16, actual, 33) != CRYPTO_STATUS_OK;
	mk.Plain.pData = nullptr;
	match &= hash->Mac(CRYPTO_MAC_HMAC, mk, msg, 16, actual, 32) != CRYPTO_STATUS_OK;
	check("HMAC rejects invalid usage, key, message and output length", match);
	CryptoSecureWipe(key, sizeof(key));

}
#endif


#ifdef CRYPTO_TEST_TICKS
// Timed comparison includes acquisition, key expansion, and peripheral setup.
// Outputs are checked against the software reference before measurements.
static void CryptoBenchmark(CipherEngine *pTarget, MacEngine *pTargetMac,
		AeadEngine *pTargetAead, RngEngine *pTargetRng,
		CipherEngine *pReference, MacEngine *pReferenceMac,
		AeadEngine *pReferenceAead)
{
	static const size_t sizes[] = {16U, 64U, 256U, 1024U};
	static uint8_t input[1024], output[1024], reference[1024];
	static uint8_t key[16], iv[16], nonce[12], aad[16];
	uint8_t tag[16], referenceTag[16];
	const unsigned rounds = 64U;
	uint32_t targetCycles, referenceCycles, begin;
	CryptoKey cipherKey = AesKey(key, CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT);
	CryptoKey macKey = AesKey(key, CRYPTO_KEY_USE_SIGN);
	XsFill(key, sizeof(key));
	XsFill(iv, sizeof(iv));
	XsFill(nonce, sizeof(nonce));
	XsFill(aad, sizeof(aad));
	XsFill(input, sizeof(input));
	printf("\r\nCrypto performance (cycles per operation, including setup)\r\n");
	for (size_t j = 0; j < sizeof(sizes) / sizeof(sizes[0]); j++)
	{
		size_t len = sizes[j];
		for (int mode = 0; mode < 3; mode++)
		{
			CRYPTO_CIPHER_ALG alg = mode == 0 ? CRYPTO_CIPHER_ECB :
				(mode == 1 ? CRYPTO_CIPHER_CTR : CRYPTO_CIPHER_CBC);
			const uint8_t *pIv = mode == 0 ? nullptr : iv;
			size_t ivLen = mode == 0 ? 0U : sizeof(iv);
			if (pTarget->Cipher(alg, 1, cipherKey, pIv, ivLen, input, len, output) != CRYPTO_STATUS_OK ||
				pReference->Cipher(alg, 1, cipherKey, pIv, ivLen, input, len, reference) != CRYPTO_STATUS_OK ||
				memcmp(output, reference, len) != 0)
			{
				printf("  AES %d %lu: correctness FAIL\r\n", mode, (unsigned long)len);
				continue;
			}
			begin = (uint32_t)CRYPTO_TEST_TICKS();
			for (unsigned i = 0; i < rounds; i++)
			{
				if (pTarget->Cipher(alg, 1, cipherKey, pIv, ivLen, input, len, output) != CRYPTO_STATUS_OK) break;
			}
			targetCycles = (uint32_t)CRYPTO_TEST_TICKS() - begin;
			begin = (uint32_t)CRYPTO_TEST_TICKS();
			for (unsigned i = 0; i < rounds; i++)
			{
				if (pReference->Cipher(alg, 1, cipherKey, pIv, ivLen, input, len, reference) != CRYPTO_STATUS_OK) break;
			}
			referenceCycles = (uint32_t)CRYPTO_TEST_TICKS() - begin;
			printf("  AES %s %4lu: target %lu, software %lu cycles/op\r\n",
				mode == 0 ? "ECB" : (mode == 1 ? "CTR" : "CBC"),
				(unsigned long)len, (unsigned long)(targetCycles / rounds),
				(unsigned long)(referenceCycles / rounds));
		}
		if (pTargetMac->Mac(CRYPTO_MAC_CMAC, macKey, input, len, tag, 16) == CRYPTO_STATUS_OK &&
			pReferenceMac->Mac(CRYPTO_MAC_CMAC, macKey, input, len, referenceTag, 16) == CRYPTO_STATUS_OK &&
			memcmp(tag, referenceTag, 16) == 0)
		{
			begin = (uint32_t)CRYPTO_TEST_TICKS();
			for (unsigned i = 0; i < rounds; i++) pTargetMac->Mac(CRYPTO_MAC_CMAC, macKey, input, len, tag, 16);
			targetCycles = (uint32_t)CRYPTO_TEST_TICKS() - begin;
			begin = (uint32_t)CRYPTO_TEST_TICKS();
			for (unsigned i = 0; i < rounds; i++) pReferenceMac->Mac(CRYPTO_MAC_CMAC, macKey, input, len, referenceTag, 16);
			referenceCycles = (uint32_t)CRYPTO_TEST_TICKS() - begin;
			printf("  CMAC    %4lu: target %lu, software %lu cycles/op\r\n",
				(unsigned long)len, (unsigned long)(targetCycles / rounds),
				(unsigned long)(referenceCycles / rounds));
		}
		for (int mode = 0; mode < 2; mode++)
		{
			CRYPTO_AEAD_ALG alg = mode == 0 ? CRYPTO_AEAD_AES_CCM : CRYPTO_AEAD_AES_GCM;
			if (pTargetAead->Seal(alg, cipherKey, nonce, sizeof(nonce), aad, sizeof(aad), input, len, output, tag, 16) != CRYPTO_STATUS_OK ||
				pReferenceAead->Seal(alg, cipherKey, nonce, sizeof(nonce), aad, sizeof(aad), input, len, reference, referenceTag, 16) != CRYPTO_STATUS_OK ||
				memcmp(output, reference, len) != 0 || memcmp(tag, referenceTag, 16) != 0)
			{
				printf("  AEAD %d %lu: correctness FAIL\r\n", mode, (unsigned long)len);
				continue;
			}
			begin = (uint32_t)CRYPTO_TEST_TICKS();
			for (unsigned i = 0; i < rounds; i++) pTargetAead->Seal(alg, cipherKey, nonce, sizeof(nonce), aad, sizeof(aad), input, len, output, tag, 16);
			targetCycles = (uint32_t)CRYPTO_TEST_TICKS() - begin;
			begin = (uint32_t)CRYPTO_TEST_TICKS();
			for (unsigned i = 0; i < rounds; i++) pReferenceAead->Seal(alg, cipherKey, nonce, sizeof(nonce), aad, sizeof(aad), input, len, reference, referenceTag, 16);
			referenceCycles = (uint32_t)CRYPTO_TEST_TICKS() - begin;
			printf("  %s %4lu: target %lu, software %lu cycles/op\r\n",
				mode == 0 ? "CCM" : "GCM", (unsigned long)len,
				(unsigned long)(targetCycles / rounds),
				(unsigned long)(referenceCycles / rounds));
		}
	}
	static const size_t rngSizes[] = {1U, 16U, 256U, 1024U};
	for (size_t j = 0; j < sizeof(rngSizes) / sizeof(rngSizes[0]); j++)
	{
		size_t len = rngSizes[j];
		begin = (uint32_t)CRYPTO_TEST_TICKS();
		unsigned completed = 0;
		for (unsigned i = 0; i < rounds; i++)
		{
			if (pTargetRng->Random(output, len) != CRYPTO_STATUS_OK) break;
			completed++;
		}
		targetCycles = (uint32_t)CRYPTO_TEST_TICKS() - begin;
		printf("  RNG %4lu: %lu cycles/op (%u/%u draws)\r\n",
			(unsigned long)len, (unsigned long)(completed ? targetCycles / completed : 0U),
			completed, rounds);
	}
}
#endif

int CryptoEngineTest(void)
{
	printf("\r\nCrypto engine test\r\n");

	CRYPTO_TEST_RNG_TYPE *hwRng = CRYPTO_TEST_RNG_CREATE(s_HwRngMem, sizeof(s_HwRngMem));
	EngineTestRng *swRng = new (s_SwRngMem) EngineTestRng();
	CRYPTO_TEST_AES_TYPE *hwAes = CRYPTO_TEST_AES_CREATE(s_HwAesMem, sizeof(s_HwAesMem));
	CRYPTO_TEST_PKA_TYPE *hwPka = CRYPTO_TEST_PKA_CREATE(s_HwPkaMem, sizeof(s_HwPkaMem), hwRng);
	CryptoSoftAes *swAes = CryptoSoftAesCreate(s_SwAesMem, sizeof(s_SwAesMem));

	swRng->Enable();
	CryptoUecc *swPka = CryptoUeccCreate(s_SwPkaMem, sizeof(s_SwPkaMem), swRng);

	check("engines construct",
		hwRng != nullptr && hwAes != nullptr && hwPka != nullptr &&
		swAes != nullptr && swPka != nullptr);
	if (hwRng == nullptr || hwAes == nullptr || hwPka == nullptr ||
		swAes == nullptr || swPka == nullptr)
	{
		printf("\r\n%d passed, %d failed\r\n", s_pass, s_fail);

		return 1;
	}

#ifdef CRYPTO_TEST_TICKS_INIT
	CRYPTO_TEST_TICKS_INIT();
#endif

	check("AES engine self-test", hwAes->SelfTest() == 0);
	check("P-256 engine self-test", hwPka->SelfTest() == 0);

#ifdef CRYPTO_TEST_HASH_TYPE
	HashChecks();
#endif
	RngChecks(hwRng);
	AesVectors(hwAes, hwAes, hwAes);
#ifdef CRYPTO_TEST_AES256
	Aes256Vectors(hwAes);
#endif
	SoftwareAesDecryptChecks(swAes);
	AesSoak(hwAes, hwAes, hwAes, swAes, swAes, swAes);
	PkaVectors(hwPka, hwPka, swPka);
	PkaSoak(hwPka, hwPka, swPka, swPka);
#ifdef CRYPTO_TEST_TICKS
	CryptoBenchmark(hwAes, hwAes, hwAes, hwRng, swAes, swAes, swAes);
#endif

	printf("\r\n%d passed, %d failed\r\n", s_pass, s_fail);

	return s_fail == 0 ? 0 : 1;
}

#ifndef CRYPTO_TEST_NO_MAIN
int main(void)
{
#ifdef UART_RETARGET
	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1);
	}

#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
	setvbuf(stdout, NULL, _IONBF, 0);
#else
	// Batch semihosting output by line, using static storage without allocation.
	static char stdoutBuffer[256];
	setvbuf(stdout, stdoutBuffer, _IOLBF, sizeof(stdoutBuffer));
#endif
#endif

	return CryptoEngineTest();
}
#endif

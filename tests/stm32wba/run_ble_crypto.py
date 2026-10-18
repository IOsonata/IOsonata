#!/usr/bin/env python3
"""Host checks of the WBA AES adapter with the real IOsonata software engine.

No ST code is copied. Expected byte order follows CubeWBA v1.10.0
Projects/Common/WPAN/Modules/BasicAES/baes_ecb.c, baes_cmac.c, baes_ccm.c
and Projects/Common/WPAN/Interfaces/hw_aes.c. Vectors are FIPS-197,
RFC 4493 example 2 and RFC 3610 packet vector 1.
Device lifecycle and the hardware AES provider are stubbed. The provider uses
the real software engine or returns BUSY to exercise fallback. This is not
a firmware build; run_bt_crypto.py tests the actual hardware driver.
"""
from pathlib import Path
import os
import random
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PLATFORM = ROOT / "ARM/ST/STM32WBAxx/src/bt_wba_platform.cpp"

DEVICE = """
#pragma once
class Device
{
protected:
	bool vbValid = false;

public:
	virtual ~Device()
	{
	}

	virtual bool Enable()
	{
		return true;
	}

	virtual void Disable()
	{
	}

	virtual void Reset()
	{
	}
};
"""

PROVIDER = r"""
#include <new>
#include <cstring>
#include "crypto/crypto_softaes.h"
static bool s_TestBusy;
static int s_TestFailAfter = -1;
static unsigned s_TestAttempts;
static uint32_t BtWbaIrqSave() { return 0; }
static void BtWbaIrqRestore(uint32_t) {}
class CryptoWbaAes : public CryptoSoftAes {
public:
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int Encrypt, const CryptoKey &Key,
						 const uint8_t *pIv, size_t IvLen, const uint8_t *pIn,
						 size_t Len, uint8_t *pOut) override
	{
		s_TestAttempts++;
		if (s_TestBusy || s_TestFailAfter == 0)
		{
			memset(pOut, 0, Len);
			return s_TestBusy ? CRYPTO_STATUS_BUSY : CRYPTO_STATUS_FAIL;
		}
		if (s_TestFailAfter > 0)
		{
			s_TestFailAfter--;
		}
		return CryptoSoftAes::Cipher(Alg, Encrypt, Key, pIv, IvLen, pIn, Len, pOut);
	}
	CRYPTO_STATUS Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
					   const uint8_t *pNonce, size_t NonceLen,
					   const uint8_t *pAad, size_t AadLen, const uint8_t *pMsg,
					   size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen) override
	{
		return s_TestBusy ? CRYPTO_STATUS_BUSY :
			CryptoSoftAes::Seal(Alg, Key, pNonce, NonceLen, pAad, AadLen,
							   pMsg, Len, pOut, pTag, TagLen);
	}
	CRYPTO_STATUS Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
					   const uint8_t *pNonce, size_t NonceLen,
					   const uint8_t *pAad, size_t AadLen, const uint8_t *pMsg,
					   size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen) override
	{
		return s_TestBusy ? CRYPTO_STATUS_BUSY :
			CryptoSoftAes::Open(Alg, Key, pNonce, NonceLen, pAad, AadLen,
							   pMsg, Len, pOut, pTag, TagLen);
	}
};
#define CRYPTO_WBA_AES_MEMSIZE sizeof(CryptoWbaAes)
static CryptoWbaAes *CryptoWbaAesCreate(void *pMem, size_t)
{
	CryptoWbaAes *p = new (pMem) CryptoWbaAes;
	p->Enable();
	return p;
}
"""

TEST = r"""
#include <cstdlib>
#include <cstdio>
extern "C" void CryptoSecureWipe(void *p, size_t n)
{
	volatile uint8_t *b = (volatile uint8_t *)p;
	while (n--)
	{
		*b++ = 0;
	}
}

static int failures;
static int checks;

static void Check(bool ok, const char *name)
{
	checks++;
	if (!ok)
	{
		fprintf(stderr, "FAIL: %s\n", name);
		failures++;
	}
}

static void Reverse(uint8_t *d, const uint8_t *s)
{
	for (int i = 0; i < 16; i++)
	{
		d[i] = s[15 - i];
	}
}

static void KnownAnswers()
{
	const uint8_t key[16] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
	};
	const uint8_t plain[16] = {
		0, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
	};
	const uint8_t cipher[16] = {
		0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
		0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a
	};
	uint8_t leKey[16];
	uint8_t leIn[16];
	uint8_t expected[16];
	uint8_t out[32];

	Reverse(leKey, key);
	Reverse(leIn, plain);
	Reverse(expected, cipher);
	BAES_EcbCrypt(leKey, leIn, out, 1);
	Check(memcmp(out, expected, 16) == 0, "ST ECB little-endian FIPS vector");
	// BLEPLAT exposes ECB encryption only. CryptoSoftAes does not implement
	// raw ECB decryption; CCM decryption uses the supported encrypt primitive.
	memcpy(out, leIn, 16);
	BAES_EcbCrypt(leKey, out, out, 1);
	Check(memcmp(out, expected, 16) == 0, "ST ECB in-place");

	const uint8_t cmacKey[16] = {
		0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
		0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
	};
	const uint8_t msg[16] = {
		0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
		0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a
	};
	const uint8_t mac[16] = {
		0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
		0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7c
	};
	BAES_CmacSetKey(cmacKey);
	BAES_CmacCompute(msg, 16, out);
	Check(memcmp(out, mac, 16) == 0, "CMAC standard byte order unchanged");

	uint8_t ccmKey[16];
	uint8_t payload[23];
	uint8_t aad[8];
	uint8_t tag[8];
	uint8_t decoded[23];

	for (int i = 0; i < 16; i++)
	{
		ccmKey[i] = 0xc0 + i;
	}
	for (int i = 0; i < 23; i++)
	{
		payload[i] = 8 + i;
	}
	for (int i = 0; i < 8; i++)
	{
		aad[i] = i;
	}
	const uint8_t nonce[13] = {
		0, 0, 0, 3, 2, 1, 0, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5
	};
	const uint8_t encrypted[23] = {
		0x58, 0x8c, 0x97, 0x9a, 0x61, 0xc6, 0x63, 0xd2,
		0xf0, 0x66, 0xd0, 0xc2, 0xc0, 0xf9, 0x89, 0x80,
		0x6d, 0x5f, 0x6b, 0x61, 0xda, 0xc3, 0x84
	};
	const uint8_t expectedTag[8] = {
		0x17, 0xe8, 0xd1, 0x2c, 0xfd, 0xf9, 0x26, 0xe0
	};
	Reverse(leKey, ccmKey);
	Check(BAES_CcmCrypt(0, leKey, 13, nonce, 8, aad, 23, payload, 8, tag, out) == 0,
		"ST CCM encrypt status");
	Check(memcmp(out, encrypted, 23) == 0 && memcmp(tag, expectedTag, 8) == 0,
		"ST CCM reversed key RFC3610 vector");
	memcpy(tag, expectedTag, 8);
	Check(BAES_CcmCrypt(1, leKey, 13, nonce, 8, aad, 23,
					   encrypted, 8, tag, decoded) == 0 &&
		memcmp(decoded, payload, 23) == 0, "ST CCM decrypt");
	memcpy(out, encrypted, 23);
	Check(BAES_CcmCrypt(1, leKey, 13, nonce, 8, aad, 23, out, 8, tag, out) == 0 &&
		memcmp(out, payload, 23) == 0, "ST CCM in-place decrypt");
	tag[0] ^= 1;
	Check(BAES_CcmCrypt(1, leKey, 13, nonce, 8, aad, 23,
					   encrypted, 8, tag, decoded) != 0,
		"ST CCM rejects wrong tag");
	const uint8_t cleared[23] = {};
	Check(memcmp(decoded, cleared, 23) == 0,
		"CCM clears output on wrong tag");
}

static void IncrementalCmac()
{
	for (const auto &v : s_CmacVectors)
	{
		for (size_t split = 0; split < v.Len || split == 0; split += 16)
		{
			uint8_t tag[16];

			BAES_CmacSetKey(v.Key);
			for (size_t off = 0; off < split; off += 16)
			{
				BAES_CmacCompute(v.Msg + off, 16, nullptr);
			}
			BAES_CmacCompute(v.Msg + split, v.Len - split, tag);
			Check(memcmp(tag, v.Tag, 16) == 0, "CMAC split matches independent vector");

			// One call may append several whole blocks.
			BAES_CmacSetKey(v.Key);
			BAES_CmacCompute(v.Msg, split, nullptr);
			BAES_CmacCompute(v.Msg + split, v.Len - split, tag);
			Check(memcmp(tag, v.Tag, 16) == 0, "CMAC multi-block append");
		}
	}
}

static void FailureChecks()
{
	const auto &v = s_CmacVectors[5];
	uint8_t tag[16];
	const uint8_t zero[16] = {};

	// A failed append must not become a valid tag of only the final part.
	BAES_CmacSetKey(v.Key);
	s_TestFailAfter = 0;
	BAES_CmacCompute(v.Msg, 16, nullptr);
	s_TestFailAfter = -1;
	BAES_CmacCompute(v.Msg + 16, v.Len - 16, tag);
	Check(memcmp(tag, zero, 16) == 0, "CMAC retains append failure");

	for (int fail = 0; fail < 4; fail++)
	{
		BAES_CmacSetKey(v.Key);
		s_TestFailAfter = fail;
		BAES_CmacCompute(v.Msg, v.Len, tag);
		Check(memcmp(tag, zero, 16) == 0, "CMAC clears tag on cipher failure");
	}
	s_TestFailAfter = -1;
	BAES_CmacSetKey(v.Key);
	BAES_CmacCompute(v.Msg, 1, nullptr);
	BAES_CmacCompute(v.Msg, v.Len, tag);
	Check(memcmp(tag, zero, 16) == 0, "CMAC refuses partial nonfinal block");
	BAES_CmacSetKey(v.Key);
	BAES_CmacCompute(nullptr, 16, nullptr);
	BAES_CmacCompute(v.Msg, v.Len, tag);
	Check(memcmp(tag, zero, 16) == 0, "CMAC refuses missing input");
	BAES_CmacSetKey(v.Key);
	BAES_CmacCompute(v.Msg, v.Len, tag);
	Check(memcmp(tag, v.Tag, 16) == 0, "CMAC new key resets failed operation");
}

int main()
{
	for (int busy = 0; busy < 2; busy++)
	{
		s_TestBusy = busy;
		KnownAnswers();
		IncrementalCmac();
	}
	s_TestBusy = false;
	FailureChecks();
	Check(s_TestAttempts != 0, "hardware provider attempted");
	BAES_Reset();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
"""

def vectors():
    from cryptography.hazmat.primitives.cmac import CMAC
    from cryptography.hazmat.primitives.ciphers import algorithms
    rng = random.Random(4493)
    rows = []
    for length in (0, 1, 15, 16, 17, 40, 64, 129, 251):
        key = rng.randbytes(16)
        msg = rng.randbytes(length)
        mac = CMAC(algorithms.AES(key))
        mac.update(msg)
        fmt = lambda data: '{' + ','.join(str(b) for b in data) + '}'
        rows.append('{' + fmt(key) + ',' + fmt(msg or b'\0') + ',' +
                    str(length) + ',' + fmt(mac.finalize()) + '}')
    return ('static const struct { uint8_t Key[16]; uint8_t Msg[251]; '
            'size_t Len; uint8_t Tag[16]; } s_CmacVectors[] = {\n' +
            ',\n'.join(rows) + '\n};\n')


def main():
    source = PLATFORM.read_text()
    start = source.index('// AES for the stack:')
    end = source.index('// Link layer interrupts.', start)
    adapter = source[start:end]
    with tempfile.TemporaryDirectory(prefix="wba-crypto-") as d:
        tmp = Path(d)
        (tmp / "device.h").write_text(DEVICE)
        (tmp / "test.cpp").write_text(
            PROVIDER + adapter + vectors() + TEST)
        for opt in ("-O0", "-Os"):
            exe = tmp / "test"
            subprocess.run([os.environ.get("CXX", "g++"), "-std=c++23", "-Wall", "-Wextra", "-Werror", opt,
                            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                            "-I" + str(tmp), "-I" + str(ROOT / "include"),
                            str(tmp / "test.cpp"),
                            str(ROOT / "src/crypto/crypto_softaes.cpp"),
                            "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            print(opt + ": WBA ECB/CMAC/CCM adapter checks passed")

if __name__ == "__main__":
    main()

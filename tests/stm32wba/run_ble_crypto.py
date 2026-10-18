#!/usr/bin/env python3
"""Host checks of the WBA AES adapter with the real IOsonata software engine.

No ST code is copied. Expected byte order follows CubeWBA v1.10.0
Projects/Common/WPAN/Modules/BasicAES/baes_ecb.c, baes_cmac.c, baes_ccm.c
and Projects/Common/WPAN/Interfaces/hw_aes.c. Vectors are FIPS-197,
RFC 4493 example 2 and RFC 3610 packet vector 1.
Only Device lifecycle is stubbed; this is not a firmware build.
"""
from pathlib import Path
import os
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

static void Check(bool ok, const char *name)
{
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

int main()
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
	return failures ? 1 : 0;
}
"""

def main():
    source = PLATFORM.read_text()
    start = source.index("static uint64_t s_AesMem")
    end = source.index("// Link layer interrupts.", start)
    adapter = source[start:end]
    with tempfile.TemporaryDirectory(prefix="wba-crypto-") as d:
        tmp = Path(d)
        (tmp / "device.h").write_text(DEVICE)
        (tmp / "test.cpp").write_text(
            '#include <cstring>\n#include "crypto/crypto_softaes.h"\n'
            + adapter + TEST)
        for opt in ("-O0", "-Os"):
            exe = tmp / "test"
            subprocess.run([os.environ.get("CXX", "g++"), "-std=c++23", opt,
                            "-I" + str(tmp), "-I" + str(ROOT / "include"),
                            str(tmp / "test.cpp"),
                            str(ROOT / "src/crypto/crypto_softaes.cpp"),
                            "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            print(opt + ": WBA ECB/CMAC/CCM adapter checks passed")

if __name__ == "__main__":
    main()

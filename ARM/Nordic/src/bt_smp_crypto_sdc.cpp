/**-------------------------------------------------------------------------
@file	bt_smp_crypto_sdc.cpp

@brief	SMP crypto provider for the SDC / nrfxlib path.

The SoftDevice Controller offloads ONLY the AES-128 block (LE Encrypt) and
randomness (LE Rand). It does NOT implement the LE ECDH commands: there is
no sdc_hci_cmd_le_read_local_p256_public_key() or sdc_hci_cmd_le_generate_-
dhkey() in the SDC API (verified against sdk-nrfxlib). On the SDC
architecture the HOST is responsible for P-256 / ECDH, exactly as the Zephyr
host stack does.

So this provider is a hybrid:
  AES-128  -> sdc_hci_cmd_le_encrypt   (controller, synchronous)
  rand     -> sdc_hci_cmd_le_rand      (controller, synchronous)
  P-256    -> micro-ecc uECC_make_key  (software, synchronous)
  ECDH     -> micro-ecc uECC_shared_secret (software, synchronous)

Because the ECDH is software and synchronous, P256KeyGen and Ecdh return
BT_SMP_CRYPTO_OK (not PENDING) - the SMP state machine advances in-call, no
controller completion event is involved.

Byte order: micro-ecc is built here with uECC_VLI_NATIVE_LITTLE_ENDIAN=1
(see the project's -D flags), so it takes and returns keys/secret in
LITTLE-endian. The BLE SMP toolbox (f4/f5/f6) and the over-the-air Public
Key PDU use BIG-endian P-256 coordinates. This provider reverses each 32-byte
coordinate on the boundary so the rest of the stack stays big-endian.

Link this file into SDC targets only. nRF52 SoftDevice and nRF54 sdk-nrf-bm
do NOT link it; there the SoftDevice owns SMP.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include "bluetooth/bt_smp_crypto.h"
#include "bluetooth/bt_hci.h"

// This provider is for the SDC / nrfxlib path only. On a SoftDevice target
// sdc_hci_cmd_le.h does not exist and the SoftDevice owns SMP, so the whole
// file compiles to nothing there.
#if defined(BT_SMP_CRYPTO_SDC) || \
	(defined(__has_include) && __has_include("sdc_hci_cmd_le.h"))

extern "C" {
#include "sdc_hci_cmd_le.h"
}

#include "uECC.h"		// micro-ecc, in the IOsonata tree, secp256r1

//-----------------------------------------------------------------------------
// Local state
//-----------------------------------------------------------------------------

// Our P-256 private key, kept between keygen and the DH computation. uECC
// little-endian (NATIVE_LITTLE_ENDIAN=1). 32 bytes for secp256r1.
static uint8_t s_PrivKey[32];

static void ReverseCopy(uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[Len - 1 - i];
	}
}

// micro-ecc RNG adapter. uECC wants int(dest,size)->1 on success; back it
// with the controller LE Rand via BtSmpCryptoRand (forward declared, defined
// below as a provider hook).
extern "C" void BtSmpCryptoRand(uint8_t *pBuf, size_t Len);
static int SmpUeccRng(uint8_t *pDest, unsigned Size)
{
	BtSmpCryptoRand(pDest, Size);
	return 1;
}

//-----------------------------------------------------------------------------
// Provider hooks
//-----------------------------------------------------------------------------

extern "C" void BtSmpCryptoAes128(BtHciDevice_t * const pDev,
								  const uint8_t Key[16], const uint8_t In[16],
								  uint8_t Out[16])
{
	(void)pDev;	// LE Encrypt is a direct controller call

	// SMP toolbox is big-endian; LE Encrypt is little-endian. Reverse both.
	sdc_hci_cmd_le_encrypt_t        cmd;
	sdc_hci_cmd_le_encrypt_return_t rsp;

	ReverseCopy(cmd.key, Key, 16);
	ReverseCopy(cmd.plaintext_data, In, 16);

	if (sdc_hci_cmd_le_encrypt(&cmd, &rsp) != 0)
	{
		memset(Out, 0, 16);		// fail loud: zero block fails the confirm
		return;
	}
	ReverseCopy(Out, rsp.encrypted_data, 16);
}

extern "C" int BtSmpCryptoP256KeyGen(BtHciDevice_t * const pDev, uint8_t pPubKey[64])
{
	(void)pDev;

	// micro-ecc needs an RNG registered before make_key. Back it with the
	// controller LE Rand (via BtSmpCryptoRand). Register once.
	if (uECC_get_rng() == nullptr)
	{
		uECC_set_rng(SmpUeccRng);
	}

	// Software keygen via micro-ecc. The internal SMP representation is
	// big-endian (what f4/f5/f6 consume per Vol 3 Part H). On this build uECC
	// produces the public key in that same big-endian order, so copy it out
	// directly. The little-endian conversion for the air PDU happens in
	// bt_smp.cpp at the wire boundary, not here.
	if (uECC_make_key(pPubKey, s_PrivKey, uECC_secp256r1()) != 1)
	{
		return BT_SMP_CRYPTO_FAIL;
	}
	return BT_SMP_CRYPTO_OK;		// synchronous - no controller event
}

extern "C" int BtSmpCryptoEcdh(BtHciDevice_t * const pDev,
							   const uint8_t pPeerPubKey[64], uint8_t pDhKey[32])
{
	(void)pDev;

	// Peer public key is big-endian internally (matches uECC on this build).
	uint8_t secret[32];
	if (uECC_shared_secret(pPeerPubKey, s_PrivKey, secret, uECC_secp256r1()) != 1)
	{
		return BT_SMP_CRYPTO_FAIL;
	}
	// DHKey is the X coordinate of the shared point, big-endian for SMP f5.
	memcpy(pDhKey, secret, 32);
	return BT_SMP_CRYPTO_OK;		// synchronous - no controller event
}

extern "C" void BtSmpCryptoRand(uint8_t *pBuf, size_t Len)
{
	// Controller LE Rand returns 8 random bytes per call. Accumulate.
	size_t off = 0;
	while (off < Len)
	{
		sdc_hci_cmd_le_rand_return_t rr;
		if (sdc_hci_cmd_le_rand(&rr) != 0)
		{
			memset(pBuf + off, 0, Len - off);
			return;
		}
		size_t n = (Len - off) < sizeof(rr.random_number) ?
				   (Len - off) : sizeof(rr.random_number);
		memcpy(pBuf + off, &rr.random_number, n);
		off += n;
	}
}

// Self-test: run the BLE spec P-256 DH known vector (Vol 3 Part H 2.3.5.6.1)
// through uECC. Returns 0 on PASS. Verifies the software ECDH and byte order
// are correct on the target. All values big-endian as the spec writes them.
extern "C" int BtSmpCryptoSelfTest(void)
{
	static const uint8_t privA[32] = {
		0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,
		0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd };
	static const uint8_t pubB[64] = {
		0x1e,0xa1,0xf0,0xf0,0x1f,0xaf,0x1d,0x96,0x09,0x59,0x22,0x84,0xf1,0x9e,0x4c,0x00,
		0x47,0xb5,0x8a,0xfd,0x86,0x15,0xa6,0x9f,0x55,0x90,0x77,0xb2,0x2f,0xaa,0xa1,0x90,
		0x4c,0x55,0xf3,0x3e,0x42,0x9d,0xad,0x37,0x73,0x56,0x70,0x3a,0x9a,0xb8,0x51,0x60,
		0x47,0x2d,0x11,0x30,0xe2,0x8e,0x36,0x76,0x5f,0x89,0xaf,0xf9,0x15,0xb1,0x21,0x4a };
	static const uint8_t dhExpect[32] = {
		0xec,0x02,0x34,0xa3,0x57,0xc8,0xad,0x05,0x34,0x10,0x10,0xa6,0x0a,0x39,0x7d,0x9b,
		0x99,0x79,0x6b,0x13,0xb4,0xf8,0x66,0xf1,0x86,0x8d,0x34,0xf3,0x73,0xbf,0xa6,0x98 };

	uint8_t dh[32];
	if (uECC_shared_secret(pubB, privA, dh, uECC_secp256r1()) != 1)
	{
		return -1;		// computation failed (key not valid in this order)
	}
	return memcmp(dh, dhExpect, 32) == 0 ? 0 : -2;	// -2 = wrong DHKey
}

#endif // BT_SMP_CRYPTO_SDC || __has_include("sdc_hci_cmd_le.h")

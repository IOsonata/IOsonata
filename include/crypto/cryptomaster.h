/**-------------------------------------------------------------------------
@file	cryptomaster.h

@brief	Silex CryptoMaster hardware AES engine on the OO engine tree.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTOMASTER_H__
#define __CRYPTOMASTER_H__

#include <stdint.h>
#include <stddef.h>

#include "device_intrf.h"
#include "crypto/crypto_softaes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CRYPTOMASTER_FETCH_ADDR			0x00U
#define CRYPTOMASTER_PUSH_ADDR			0x10U
#define CRYPTOMASTER_INT_STATRAW		0x28U
#define CRYPTOMASTER_INT_STATCLR		0x30U
#define CRYPTOMASTER_CONFIG				0x34U
#define CRYPTOMASTER_START				0x38U
#define CRYPTOMASTER_STATUS				0x3CU
#define CRYPTOMASTER_HW_PRESENCE		0x400U

#define CRYPTOMASTER_CONFIG_SCATTERGATHER	0x3U
#define CRYPTOMASTER_START_BOTH			0x3U

#define CRYPTOMASTER_STATUS_FETCH_BUSY	(1U << 0)
#define CRYPTOMASTER_STATUS_PUSH_BUSY	(1U << 1)
#define CRYPTOMASTER_STATUS_PUSH_WAIT	(1U << 5)
#define CRYPTOMASTER_STATUS_BUSY \
	(CRYPTOMASTER_STATUS_FETCH_BUSY | CRYPTOMASTER_STATUS_PUSH_BUSY | \
	 CRYPTOMASTER_STATUS_PUSH_WAIT)

#define CRYPTOMASTER_INT_FETCH_ERROR	(1U << 2)
#define CRYPTOMASTER_INT_PUSH_ERROR		(1U << 5)
#define CRYPTOMASTER_INT_ERROR \
	(CRYPTOMASTER_INT_FETCH_ERROR | CRYPTOMASTER_INT_PUSH_ERROR)

#define CRYPTOMASTER_SOFTRESET_ENABLE	0x10U
#define CRYPTOMASTER_STATUS_RESET_BUSY	0x40U
#define CRYPTOMASTER_PRESENT_AES		(1U << 0)

typedef struct __CryptoMaster_Desc CryptoMasterDesc_t;
struct __CryptoMaster_Desc {
	uint8_t            *pAddr;
	CryptoMasterDesc_t *pNext;
	uint32_t            Size;
	uint32_t            Tag;
};

#define CRYPTOMASTER_DMA_STOP		((CryptoMasterDesc_t *)1)
#define CRYPTOMASTER_DMA_REALIGN	(1UL << 29)
#define CRYPTOMASTER_TAG_ENGINE_AES		1U
#define CRYPTOMASTER_TAG_CONFIG(RegOff)	((1U << 4) | ((uint32_t)(RegOff) << 8))
#define CRYPTOMASTER_TAG_LAST			(1U << 5)
#define CRYPTOMASTER_BA411_CFG_OFFSET	0x00U
#define CRYPTOMASTER_BA411_KEY_OFFSET	0x08U
#define CRYPTOMASTER_BA411_MODE_ECB		(1U << 8)

#ifdef __cplusplus
}

class CracenIntrf;

class CryptoMaster : public CryptoSoftAes {
public:
	CryptoMaster() { vbValid = false; vpCracen = nullptr; }

	bool Init(CracenIntrf * const pIntrf);

	bool Enable() override;
	void Disable() override {}
	void Reset() override {}

	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
						 const CryptoKey &Key,
						 const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pIn, size_t Len,
						 uint8_t *pOut) override;

private:
	CracenIntrf *vpCracen;
};

#endif // __cplusplus

#endif // __CRYPTOMASTER_H__

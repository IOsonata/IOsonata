/**-------------------------------------------------------------------------
@file	cryptomaster.h

@brief	Silex CryptoMaster hardware AES engine on the OO engine tree.

		CryptoMaster drives the Silex CryptoMaster block (BA411e AES behind a
		scatter/gather DMA front end). It inherits CryptoSoftAes and overrides
		only Cipher with the hardware AES, so the inherited software CMAC runs
		over the hardware AES through the virtual Cipher call, and the software
		AES remains as an unreferenced fallback the linker drops.

		The register offsets, masks and descriptor layout are hardware facts of
		the Silex core, restated here in this project own form. This unit is
		silicon independent: it reaches the block only through the injected base
		pointer and the vendor hooks below. The absolute base address, the
		wrapper module power, and the shared engine lock live in the target port
		(for the nRF54 family, cryptomaster_nrf.cpp), which is the only file
		that names the vendor wrapper.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTOMASTER_H__
#define __CRYPTOMASTER_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/crypto_softaes.h"

#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
// Silex CryptoMaster register interface (word offsets from the block base).
// The engine is a scatter/gather DMA master: a fetch descriptor chain feeds
// input, a push descriptor drains output. Offsets are relative; the port base.
//-----------------------------------------------------------------------------
#define CRYPTOMASTER_FETCH_ADDR			0x00U	//!< Fetch descriptor list address
#define CRYPTOMASTER_PUSH_ADDR			0x10U	//!< Push descriptor list address
#define CRYPTOMASTER_INT_STATRAW		0x28U	//!< Raw interrupt status
#define CRYPTOMASTER_INT_STATCLR		0x30U	//!< Interrupt status clear
#define CRYPTOMASTER_CONFIG				0x34U	//!< Mode configuration
#define CRYPTOMASTER_START				0x38U	//!< Start trigger
#define CRYPTOMASTER_STATUS				0x3CU	//!< Busy status
#define CRYPTOMASTER_HW_PRESENCE		0x400U	//!< Populated sub-engine bits

#define CRYPTOMASTER_CONFIG_SCATTERGATHER	0x3U	//!< Descriptor mode, both channels
#define CRYPTOMASTER_START_BOTH			0x3U	//!< Launch fetch and push

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

#define CRYPTOMASTER_SOFTRESET_ENABLE	0x10U	//!< In the CONFIG register
#define CRYPTOMASTER_STATUS_RESET_BUSY	0x40U

#define CRYPTOMASTER_PRESENT_AES		(1U << 0)	//!< BA411 AES populated

// Scatter/gather descriptor. The DMA walks a linked list: buffer pointer, next
// pointer, size with flags, tag. The terminator is address 1, not null. Size
// flags occupy the high bits; the low 24 carry the byte length.
typedef struct __CryptoMaster_Desc CryptoMasterDesc_t;
struct __CryptoMaster_Desc {
	uint8_t            *pAddr;	//!< Buffer pointer (word aligned)
	CryptoMasterDesc_t *pNext;	//!< Next descriptor, or CRYPTOMASTER_DMA_STOP
	uint32_t            Size;	//!< Byte length OR-ed with CRYPTOMASTER_DMA_* flags
	uint32_t            Tag;	//!< Engine/data tag
};

#define CRYPTOMASTER_DMA_STOP		((CryptoMasterDesc_t *)1)	//!< List terminator
#define CRYPTOMASTER_DMA_REALIGN	(1UL << 29)	//!< Realign on this descriptor

// BA411 AES tags. A config descriptor sets bit 4 and puts the target BA411
// register offset in bits 15:8; a data descriptor sets the engine bit, and the
// final data descriptor also sets the last bit.
#define CRYPTOMASTER_TAG_ENGINE_AES		1U
#define CRYPTOMASTER_TAG_CONFIG(RegOff)	((1U << 4) | ((uint32_t)(RegOff) << 8))
#define CRYPTOMASTER_TAG_LAST			(1U << 5)

#define CRYPTOMASTER_BA411_CFG_OFFSET	0x00U		//!< Mode/config word offset
#define CRYPTOMASTER_BA411_KEY_OFFSET	0x08U		//!< Key material offset
#define CRYPTOMASTER_BA411_MODE_ECB		(1U << 8)	//!< ECB mode select

//-----------------------------------------------------------------------------
// Vendor binding (implemented by the target port). The engine reaches the
// block only through these hooks; only the port names the vendor wrapper.
//-----------------------------------------------------------------------------

// Absolute base of the CryptoMaster register block.
volatile void *CryptoMasterBase(void);

// Enable/disable the CryptoMaster module in the wrapper for one operation.
void CryptoMasterModuleEnable(void);
void CryptoMasterModuleDisable(void);

// Non-blocking ownership lock for the shared block (shared with the RNG). Must
// not touch a register until acquire succeeds; release once after the op.
bool CryptoMasterTryAcquire(void);
void CryptoMasterRelease(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/** @addtogroup Crypto
  * @{
  */

/// @brief	Hardware AES-128 engine over the Silex CryptoMaster (BA411e).
///
/// Overrides Cipher with the hardware AES; ECB, CTR and CBC are built on the
/// hardware single-block. CMAC and the software AES fallback are inherited from
/// CryptoSoftAes, so inherited CMAC runs over the hardware AES automatically.
class CryptoMaster : public CryptoSoftAes {
public:
	CryptoMaster() { vbValid = false; }

	// Device lifecycle. Enable probes the AES presence bit and self-tests.
	bool Enable() override;
	void Disable() override {}
	void Reset() override {}

	// Hardware AES-128 ECB/CTR/CBC. Falls back to the software base only for a
	// mode the hardware path does not cover.
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
						 const CryptoKey &Key,
						 const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pIn, size_t Len, uint8_t *pOut) override;
};

/// Bytes of storage CryptoMasterCreate needs.
#define CRYPTO_CRYPTOMASTER_MEMSIZE		sizeof(CryptoMaster)

/// @brief	Construct a CryptoMaster in caller-provided storage (no allocation).
/// @return	Ready engine pointer, or nullptr on a too-small buffer or absent AES.
CryptoMaster *CryptoMasterCreate(void *pMem, size_t MemSize);

/** @} */

#endif // __cplusplus

#endif // __CRYPTOMASTER_H__

/**-------------------------------------------------------------------------
@file	cc3xx_config.h

@brief	Arm CC3xx low level driver configuration: nRF52840 CryptoCell CC310

		Configuration for building the open Arm CC3xx register level driver
		(Mbed-TLS/tf-psa-crypto-drivers, vendor/arm/cc3xx/low_level_driver)
		against the CC310 instance in the nRF52840, scoped to P-256 ECDH for
		LE Secure Connections: Weierstrass EC on the PKA and key generation.
		Randomness comes from IOsonata RngGet through ARM/src/cc3xx_rng.cpp;
		the CC3xx TRNG, entropy conditioner and DRBG are not built.

		Integration facts, each from an authoritative source:
		- Register file base 0x5002B000 and the wrapper enable register at
		  NRF_CRYPTOCELL (0x5002A000): Nordic MDK nrf52840.h, which defines
		  the CC register blocks (CC_PKA, CC_RNG, CC_CTL, ...) at this base.
		- CC310 hardware variant (4 KiB PKA SRAM, CC310 DMA interrupt
		  layout): CC3XX_CONFIG_HW_VERSION_CC310.

		The driver is polling (no CRYPTOCELL interrupt use), so it does not
		conflict with any interrupt vector and needs no NVIC setup.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CC3XX_CONFIG_H__
#define __CC3XX_CONFIG_H__

//----------------------------------------------------------------------------
// Hardware instance
//----------------------------------------------------------------------------

// CC310 register file on the nRF52840. The Nordic wrapper enable register
// (NRF_CRYPTOCELL->ENABLE at 0x5002A500) must be 1 while the block is in use.
// cc3xx_port.h enables it before ARM/src/crypto_cc3xx.cpp accesses the core.
#define CC3XX_CONFIG_BASE_ADDRESS               (0x5002B000UL)

// The nRF52840 integrates the CC310 variant: 4 KiB PKA SRAM, CC310 DMA
// completion interrupt layout, no always-on interface.
#ifndef CC3XX_CONFIG_HW_VERSION_CC310
#define CC3XX_CONFIG_HW_VERSION_CC310
#endif

//----------------------------------------------------------------------------
// Elliptic curve: P-256 Weierstrass on the PKA
//----------------------------------------------------------------------------

#define CC3XX_CONFIG_EC_CURVE_TYPE_WEIERSTRASS_ENABLE
#define CC3XX_CONFIG_EC_CURVE_SECP_256_R1_ENABLE

#define CC3XX_CONFIG_ECDH_ENABLE
#define CC3XX_CONFIG_ECDSA_KEYGEN_ENABLE

#define CC3XX_CONFIG_PKA_CALC_NP_ENABLE
#define CC3XX_CONFIG_PKA_INLINE_FOR_PERFORMANCE
#define CC3XX_CONFIG_PKA_ALIGN_FOR_PERFORMANCE
#ifndef CC3XX_CONFIG_PKA_MAX_VIRT_REG_AMOUNT
#define CC3XX_CONFIG_PKA_MAX_VIRT_REG_AMOUNT    64
#endif

//----------------------------------------------------------------------------
// Randomness: sourced from IOsonata RngGet, not the CC3xx DRBG
//----------------------------------------------------------------------------

// The CC3xx RNG entry points are provided by ARM/src/cc3xx_rng.cpp over RngGet,
// so the driver's own noise source, entropy conditioner and DRBG are not built.
// CC3XX_CONFIG_RNG_ENABLE stays on so the public RNG API is declared, but no
// DRBG or AES DRBG engine is selected: none of CC3XX_CONFIG_RNG_DRBG_CTR,
// CC3XX_CONFIG_DRBG_CTR_ENABLE, CC3XX_CONFIG_AES_*_ENABLE is defined. The TRNG
// ring oscillator and subsampling parameters are likewise unneeded; RngGet
// owns the hardware entropy path.
#define CC3XX_CONFIG_RNG_ENABLE

#ifndef CC3XX_CONFIG_RNG_MAX_ATTEMPTS
#define CC3XX_CONFIG_RNG_MAX_ATTEMPTS           100
#endif

#endif // __CC3XX_CONFIG_H__

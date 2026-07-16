/**-------------------------------------------------------------------------
@file	ba414ep.h

@brief	Silex BA414EP hardware P-256 engine on the OO engine tree.

		Ba414ep drives the Silex BA414EP public-key core to perform P-256 ECDH.
		It implements the KeyAgreeEngine facet: a hardware override of the same
		operations CryptoUecc provides in software. A build that links this
		engine gets hardware key agreement; the software CryptoUecc remains the
		portable fallback.

		The BA414EP is fixed function for the standard curves: the P-256 field
		prime, order, coefficients and generator are built into the hardware and
		selected by a curve bit in the command word, so no curve parameters and
		no microcode are loaded. Operands (scalar, point, result) live in numbered
		crypto-RAM slots. The register offsets, command opcodes and slot indices
		are hardware facts of the core, restated here in this project own form.

		This unit is silicon independent: it reaches the core only through the
		crypto interface it is given at Init, using Device Read / Write with the
		base selector and IP offset. The absolute base addresses and the module
		enable live in the interface (for the nRF54 family, CracenIntrf), which
		is the only file that names the vendor wrapper.

		Randomness for the private scalar and the scalar-blinding countermeasure
		comes from an injected RngEngine and must be security grade; the engine
		fails closed on a non-secure source, as CryptoUecc does.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
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

//-----------------------------------------------------------------------------
// Silex BA414EP public-key register interface (word offsets from the PK base).
//-----------------------------------------------------------------------------
#define BA414EP_REG_CONFIG			0x00U	//!< Operand slot pointer config
#define BA414EP_REG_COMMAND			0x04U	//!< Command / curve / mode select
#define BA414EP_REG_CONTROL			0x08U	//!< Start / clear-irq control
#define BA414EP_REG_STATUS			0x0CU	//!< Busy / error status
#define BA414EP_REG_HWCONFIG		0x18U	//!< Populated features / max size

#define BA414EP_CONTROL_START		0x00000001U	//!< Start the loaded command
#define BA414EP_CONTROL_CLEAR_IRQ	0x00000002U	//!< Clear the completion flag

// Status bits. Busy is bit 16; error conditions occupy bits 4 through 15. The
// core is idle and successful when status masked by the error window is zero.
#define BA414EP_STATUS_BUSY			(1U << 16)	//!< Operation in progress
#define BA414EP_STATUS_POINT_ERROR		(1U << 4)	//!< Point not on curve
#define BA414EP_STATUS_NOT_INVERTIBLE	(1U << 11)	//!< Retry with fresh blind
// All error bits (bits 4 through 15), excluding the busy bit at 16.
#define BA414EP_STATUS_ERROR_MASK	0x0000FFF0U

// Command word: operation opcode, operand-size field, curve-select bits, and
// the big-endian operand flag. The core reads the operand width from bits 8..15
// as (size in bytes - 1); without it the core misreads the operands and reports
// a valid point as off curve.
#define BA414EP_CMD_ECC_PTMUL		0x22U		//!< ECC scalar point multiply
#define BA414EP_CMD_CHECK_XY		0x25U		//!< Point-on-curve check
#define BA414EP_CMD_OPSIZE(bytes)	(((uint32_t)((bytes) - 1U)) << 8)	//!< Operand size field
#define BA414EP_CMD_SELCUR_P256		0x00100000U	//!< Select the built-in P-256
#define BA414EP_CMD_BIG_ENDIAN		(1U << 28)	//!< Big-endian operand encoding

#define BA414EP_CMD_P256_PTMUL \
	(BA414EP_CMD_ECC_PTMUL | BA414EP_CMD_OPSIZE(32U) | \
	 BA414EP_CMD_SELCUR_P256 | BA414EP_CMD_BIG_ENDIAN)
#define BA414EP_CMD_P256_CHECK_XY \
	(BA414EP_CMD_CHECK_XY | BA414EP_CMD_OPSIZE(32U) | \
	 BA414EP_CMD_SELCUR_P256 | BA414EP_CMD_BIG_ENDIAN)

// Config register packs the three operand slot pointers (A input, B input,
// C result) into one word.
#define BA414EP_CONFIG_PTRS(a, b, c) \
	((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16))

// Crypto-RAM slot layout. Each slot is BA414EP_SLOT_SIZE bytes; a point uses two
// consecutive slots (X then Y). These indices match the core point-multiply
// command: scalar in slot B, point in slots AA/AA+1, result in slots C/C+1.
#define BA414EP_SLOT_SIZE			0x200U		//!< Bytes per operand slot
#define BA414EP_SLOT_SCALAR			8U			//!< PTR_B: private scalar k
#define BA414EP_SLOT_RESULT_X		10U			//!< PTR_C: result X
#define BA414EP_SLOT_RESULT_Y		11U			//!< PTR_C + 1: result Y
#define BA414EP_SLOT_POINT_X		12U			//!< PTR_AA: input point X
#define BA414EP_SLOT_POINT_Y		13U			//!< PTR_AA + 1: input point Y

// Config pointer word for the point multiply: A = point (12), B = scalar (8),
// C = result (10).
#define BA414EP_CONFIG_PTMUL		BA414EP_CONFIG_PTRS(12U, 8U, 10U)

//-----------------------------------------------------------------------------
// Silex BA414EP IP layout. Register offsets are relative to the register
// sub-block; operand offsets start at the operand memory sub-block, which the
// crypto interface presents at BA414EP_CRYPTORAM_OFFSET so the engine addresses
// registers and operands in one offset space. These are IP-fixed constants, not
// MCU addresses; the interface holds the actual bases.
//-----------------------------------------------------------------------------

// Operand memory sub-block offset. Register offsets are below it; operand
// offsets are at or above it. The two ranges are disjoint, so the interface
// routes an access to the right sub-block by the offset alone.
#define BA414EP_CRYPTORAM_OFFSET	0x8000U

// Interface base selector (Device DeviceAddress / DevAddr), chosen by the engine
// per access the way an SPI device picks a chip-select. The crypto interface
// maps these two spaces to the register and operand memory sub-block bases.
#define BA414EP_ADDR_REG	0U		//!< Engine register space
#define BA414EP_ADDR_MEM	1U		//!< Operand memory space

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/** @addtogroup Crypto
  * @{
  */

/// @brief	Hardware P-256 engine over the Silex BA414EP (ECDH).
///
/// Implements KeyAgreeEngine with the hardware scalar point multiply. The
/// private key lives in caller key context (single use, wiped on Agree and on
/// failure). Randomness is drawn from an injected security-grade RngEngine; the
/// engine fails closed on a non-secure source.
class Ba414ep : public KeyAgreeEngine {
public:
	/// Per-instance key context the caller provides to KeyGen/Agree.
	struct KeyCtx {
		uint8_t PrivKey[32];	//!< P-256 private scalar, retained keygen to DH
		bool    bKeyValid;		//!< true only while PrivKey holds a usable key
	};

	Ba414ep() { vbValid = false; vpRng = nullptr; }

	/**
	 * @brief	Initialise the engine on a crypto interface.
	 *
	 * Sensor-style construction: the interface is created separately (like an
	 * SPI or I2C bus) and its pointer passed here. Register and operand access
	 * then go through the inherited Device Read / Write over that interface. The
	 * engine holds no base address and no MCU-specific knowledge.
	 *
	 * @param	pIntrf	The crypto core interface the engine sits on.
	 * @param	pRng	Security-grade random source for key generation and
	 *					blinding. Verify may run without it.
	 *
	 * @return	true on success.
	 */
	bool Init(DeviceIntrf * const pIntrf, RngEngine *pRng);

	/// Bind the security-grade random source for key generation and blinding.
	void SetRng(RngEngine *pRng) { vpRng = pRng; }

	// Device lifecycle. Enable probes the core and confirms P-256 support.
	bool Enable() override;
	void Disable() override {}
	void Reset() override {}

	// KeyAgreeEngine.
	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPubKey) override;
	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
						const uint8_t *pPeerPubKey, uint8_t *pSharedX,
						bool bKeepKey = false) override;

private:
	RngEngine *vpRng;			//!< Security-grade random source
};

// Construct a Ba414ep as a plain object and call Init(pIntrf, pRng), the same
// way a sensor is constructed and given its bus interface.

/** @} */

#endif // __cplusplus

#endif // __BA414EP_H__

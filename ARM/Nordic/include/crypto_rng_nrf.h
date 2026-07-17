/**-------------------------------------------------------------------------
@file	crypto_rng_nrf.h

@brief	Nordic nRF hardware random generator on the OO engine tree.

		Declares CryptoRngNrf, the Nordic implementation of the RngEngine facet.
		It is security grade (IsSecure() returns true). The engine is a thin
		policy object with no hardware knowledge; entropy is read through the
		DeviceIntrf it is constructed on. On nRF54 parts that is CracenIntrf
		(CTR-DRBG, or the SoftDevice entropy pool when an S115/S145 stack is
		enabled at run time). On nRF52/nRF53 it is RngPeriphIntrf over the RNG
		peripheral registers, or the SoftDevice entropy pool when a stack is
		enabled. The SoftDevice check is made inside the interface on every
		draw.

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
#ifndef __CRYPTO_RNG_NRF_H__
#define __CRYPTO_RNG_NRF_H__

#include <stdint.h>
#include <stddef.h>

#include "device_intrf.h"
#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

/// @brief	Nordic hardware random generator implementing RngEngine.
///
/// Security grade: IsSecure() is true. Random draws hardware entropy through
/// the per-part path selected at build time. Stateless beyond the Device
/// lifecycle; the underlying peripheral holds the state.
#if !defined(NRF54H20_XXAA) && !defined(NRF54L15_XXAA)
/// @brief	Entropy interface over the RNG peripheral (nRF52 / nRF53).
///
/// An Rx transfer fills the buffer with hardware entropy; there is no address
/// phase and no Tx direction. While a SoftDevice is enabled it owns the RNG
/// peripheral, so the draw goes through the SoftDevice entropy pool instead;
/// the check is made at run time on every draw.
class RngPeriphIntrf : public DeviceIntrf {
public:
	bool Init(void);

	operator DevIntrf_t * const () override { return &vDevIntrf; }
	uint32_t Rate(uint32_t DataRate) override { (void)DataRate; return 0; }
	uint32_t Rate(void) override { return 0; }
	bool StartRx(uint32_t DevAddr) override {
		return DeviceIntrfStartRx(&vDevIntrf, DevAddr);
	}
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vDevIntrf, pBuff, BuffLen);
	}
	void StopRx(void) override { DeviceIntrfStopRx(&vDevIntrf); }
	bool StartTx(uint32_t DevAddr) override {
		return DeviceIntrfStartTx(&vDevIntrf, DevAddr);
	}
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vDevIntrf, pData, DataLen);
	}
	void StopTx(void) override { DeviceIntrfStopTx(&vDevIntrf); }

private:
	DevIntrf_t vDevIntrf;
};

/// Interface singleton for the standard construction path.
RngPeriphIntrf *RngPeriphIntrfInstance(void);
#endif

class CryptoRngNrf : public RngEngine {
public:
	CryptoRngNrf() { vbValid = false; }

	/**
	 * @brief	Initialise the engine on a crypto interface.
	 *
	 * Sensor-style construction, like the other crypto engines: the interface is
	 * created separately and its pointer passed here, held in the inherited
	 * Device interface pointer. On the CRACEN parts the RNG module is enabled and
	 * released through it; on parts with a standalone RNG peripheral the engine
	 * drives that directly and the interface is unused.
	 *
	 * @param	pIntrf	The crypto core interface, or nullptr on parts with a
	 *					standalone RNG peripheral.
	 *
	 * @return	true on success.
	 */
	bool Init(DeviceIntrf * const pIntrf);

	// Device lifecycle. Enable brings up the hardware RNG (CTR-DRBG init on the
	// CRACEN parts); it is idempotent.
	bool Enable() override;
	void Disable() override {}
	void Reset() override {}

	// RngEngine: fill pOut with Len hardware random bytes.
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;

	// Hardware entropy: security grade.
	bool IsSecure() const override { return true; }
};

/// @brief	Singleton accessor for the Nordic hardware RNG engine. Constructs on
///			first use in internal static storage (no allocation) and returns the
///			same instance thereafter. Returns the engine even before Enable; the
///			caller or the first Random draw enables it.
CryptoRngNrf *CryptoRngNrfInstance(void);

//-----------------------------------------------------------------------------
// The RNG is reached through the RngEngine facet: hold a CryptoRngNrf (or the
// singleton CryptoRngNrfInstance) and call Random. Consumers that need entropy
// take an injected RngEngine pointer, so a software generator can stand in for
// the hardware one. There is no free-function entropy shim.
//-----------------------------------------------------------------------------

/** @} */

#endif // __CRYPTO_RNG_NRF_H__

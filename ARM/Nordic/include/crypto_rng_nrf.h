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

#if !defined(NRF54H20_XXAA) && !defined(NRF54L15_XXAA)
/// @brief	Entropy interface over the RNG peripheral (nRF52 / nRF53).
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

RngPeriphIntrf *RngPeriphIntrfInstance(void);
#endif

class CryptoRngNrf : public RngEngine {
public:
	CryptoRngNrf() : vbIntrfEnabled(false) { vbValid = false; }

	bool Init(DeviceIntrf * const pIntrf);

	bool Enable() override;
	void Disable() override;
	void Reset() override;

	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;
	bool IsSecure() const override { return true; }

private:
	bool vbIntrfEnabled;
};

CryptoRngNrf *CryptoRngNrfInstance(void);

/** @} */

#endif // __CRYPTO_RNG_NRF_H__

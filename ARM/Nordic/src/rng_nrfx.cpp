/**-------------------------------------------------------------------------
@file	rng_nrfx.cpp

@brief	Random number generator implementation on Nordic nRF series.

		Provides the CryptoRngNrf engine on the RngEngine facet. There is no
		software default: a part without an RNG peripheral does not link.

		The engine is a thin policy object with no hardware knowledge. Entropy
		is read through the DeviceIntrf the engine is constructed on:

		- nRF54 (CRACEN parts): CracenIntrf, DevAddr CRACEN_ADDR_RNG. The
		  interface draws from the NIST SP800-90A CTR-DRBG, or from the
		  SoftDevice entropy pool when an S115/S145 stack is enabled.
		- nRF52 / nRF53: RngPeriphIntrf (defined here) over the RNG peripheral
		  registers, or the SoftDevice entropy pool when a stack is enabled.

		The SoftDevice check is made at run time on every draw, inside the
		interface that owns the entropy hardware: while a SoftDevice is enabled
		it owns that hardware and direct access asserts the stack.

@author	Hoang Nguyen Hoan
@date	Aug. 9, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf.h"

#include "istddef.h"
#include "crypto/icrypto.h"
#include "crypto_rng_nrf.h"

#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
#include "cracen_intrf.h"
#define RNG_USE_CRACEN		1
#else
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif
#endif

#if !defined(RNG_USE_CRACEN)

//-----------------------------------------------------------------------------
// RngPeriphIntrf : entropy interface over the RNG peripheral (nRF52 / nRF53).
//
// An Rx transfer fills the buffer with hardware entropy; there is no address
// phase and no Tx direction. While a SoftDevice is enabled it owns the RNG
// peripheral, so the draw goes through the SoftDevice entropy pool instead;
// the check is made at run time on every draw because the stack can be
// enabled and disabled during execution.
//-----------------------------------------------------------------------------

static inline NRF_RNG_Type *RngPeriphReg(void)
{
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
 #ifdef NRF5340_XXAA_NETWORK
	return NRF_RNG_NS;
 #else
	return NRF_RNG_S;
 #endif
#else
	return NRF_RNG;
#endif
}

// True when a SoftDevice is present in the build and enabled at run time.
static bool RngPeriphSdEnabled(void)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	uint8_t en = 0;
	(void)sd_softdevice_is_enabled(&en);
	return en != 0;
#else
	return false;
#endif
}

// Fill the buffer from the SoftDevice entropy pool. The pool refills at a
// finite rate, so take what is available until the request is filled.
static int RngPeriphSdRandFill(uint8_t *pBuff, int BuffLen)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	int idx = 0;
	while (idx < BuffLen)
	{
		uint8_t avail = 0;
		(void)sd_rand_application_bytes_available_get(&avail);
		if (avail == 0)
		{
			continue;
		}
		int n = BuffLen - idx;
		if (n > (int)avail)
		{
			n = avail;
		}
		if (sd_rand_application_vector_get(&pBuff[idx], (uint8_t)n) == NRF_SUCCESS)
		{
			idx += n;
		}
	}
	return BuffLen;
#else
	(void)pBuff; (void)BuffLen;
	return 0;
#endif
}

static bool s_bPeriphSdXfer;			// current draw goes through the SoftDevice pool

static void RngPeriphDisable(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; }
static void RngPeriphEnable(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; }
static uint32_t RngPeriphGetRate(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; return 0; }
static uint32_t RngPeriphSetRate(DevIntrf_t * const pDevIntrf, uint32_t Rate) {
	(void)pDevIntrf; (void)Rate; return 0;
}

static bool RngPeriphStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf; (void)DevAddr;
	s_bPeriphSdXfer = RngPeriphSdEnabled();
	return true;
}

static int RngPeriphRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	(void)pIntrf;
	if (pBuff == nullptr || BuffLen <= 0)
	{
		return 0;
	}
	if (s_bPeriphSdXfer)
	{
		return RngPeriphSdRandFill(pBuff, BuffLen);
	}

	NRF_RNG_Type *reg = RngPeriphReg();

	// Enable bias correction so the byte stream is unbiased.
	reg->CONFIG = RNG_CONFIG_DERCEN_Enabled;
	reg->TASKS_START = 1;

	for (int i = 0; i < BuffLen; i++)
	{
		reg->EVENTS_VALRDY = 0;
		while (reg->EVENTS_VALRDY == 0);
		pBuff[i] = (uint8_t)reg->VALUE;
	}

	reg->TASKS_STOP = 1;
	reg->CONFIG = RNG_CONFIG_DERCEN_Disabled;

	return BuffLen;
}

static void RngPeriphStopRx(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; }
static bool RngPeriphStartTx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr) {
	(void)pDevIntrf; (void)DevAddr; return false;
}
static int RngPeriphTxData(DevIntrf_t * const pDevIntrf, const uint8_t *pData, int DataLen) {
	(void)pDevIntrf; (void)pData; (void)DataLen; return 0;
}
static int RngPeriphTxSrData(DevIntrf_t * const pDevIntrf, const uint8_t *pData, int DataLen) {
	(void)pDevIntrf; (void)pData; (void)DataLen; return 0;
}
static void RngPeriphStopTx(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; }
static void RngPeriphReset(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; }
static void RngPeriphPowerOff(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; }
static void *RngPeriphGetHandle(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; return nullptr; }

bool RngPeriphIntrf::Init(void)
{
	vDevIntrf.pDevData = this;
	vDevIntrf.EvtCB = nullptr;
	atomic_flag_clear(&vDevIntrf.bBusy);
	vDevIntrf.MaxRetry = 5;
	atomic_store(&vDevIntrf.EnCnt, 1);
	vDevIntrf.Type = DEVINTRF_TYPE_CRYPTO;
	vDevIntrf.bDma = false;
	vDevIntrf.bIntEn = false;
	atomic_store(&vDevIntrf.bTxReady, true);
	atomic_store(&vDevIntrf.bNoStop, false);
	vDevIntrf.MaxTrxLen = 0;
	vDevIntrf.Disable = RngPeriphDisable;
	vDevIntrf.Enable = RngPeriphEnable;
	vDevIntrf.GetRate = RngPeriphGetRate;
	vDevIntrf.SetRate = RngPeriphSetRate;
	vDevIntrf.StartRx = RngPeriphStartRx;
	vDevIntrf.RxData = RngPeriphRxData;
	vDevIntrf.StopRx = RngPeriphStopRx;
	vDevIntrf.StartTx = RngPeriphStartTx;
	vDevIntrf.TxData = RngPeriphTxData;
	vDevIntrf.TxSrData = RngPeriphTxSrData;
	vDevIntrf.StopTx = RngPeriphStopTx;
	vDevIntrf.Reset = RngPeriphReset;
	vDevIntrf.PowerOff = RngPeriphPowerOff;
	vDevIntrf.GetHandle = RngPeriphGetHandle;

	return true;
}

// Interface singleton for the standard construction path.
RngPeriphIntrf *RngPeriphIntrfInstance(void)
{
	static RngPeriphIntrf s_RngPeriphIntrf;
	static bool s_bInit = false;

	if (!s_bInit)
	{
		s_bInit = s_RngPeriphIntrf.Init();
	}
	return s_bInit ? &s_RngPeriphIntrf : nullptr;
}

#endif	// !RNG_USE_CRACEN

//-----------------------------------------------------------------------------
// CryptoRngNrf : the RngEngine policy object. No hardware knowledge; entropy
// is read through the interface the engine is constructed on. The framework
// transfer path (StartRx / RxData / StopRx with MaxRetry) does the rest.
//-----------------------------------------------------------------------------

bool CryptoRngNrf::Init(DeviceIntrf * const pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	Interface(pIntrf);
#if defined(RNG_USE_CRACEN)
	DeviceAddress(CRACEN_ADDR_RNG);
#else
	DeviceAddress(0);
#endif
	return Enable();
}

bool CryptoRngNrf::Enable(void)
{
	vbValid = Interface() != nullptr;
	return vbValid;
}

CRYPTO_STATUS CryptoRngNrf::Random(uint8_t *pOut, size_t Len)
{
	if (!vbValid || pOut == nullptr || Len == 0)
	{
		return CRYPTO_STATUS_FAIL;
	}
	int n = Interface()->Rx(DeviceAddress(), pOut, (int)Len);

	return n == (int)Len ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

// Singleton accessor. Constructs the engine on the entropy interface for the
// part: CracenIntrf on CRACEN parts, the RNG peripheral interface otherwise.
CryptoRngNrf *CryptoRngNrfInstance(void)
{
	static CryptoRngNrf s_CryptoRngNrf;
	static bool s_bInit = false;

	if (!s_bInit)
	{
#if defined(RNG_USE_CRACEN)
		s_bInit = s_CryptoRngNrf.Init(CracenIntrfInstance());
#else
		s_bInit = s_CryptoRngNrf.Init(RngPeriphIntrfInstance());
#endif
	}
	return s_bInit ? &s_CryptoRngNrf : nullptr;
}

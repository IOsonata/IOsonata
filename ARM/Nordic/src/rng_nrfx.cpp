/**-------------------------------------------------------------------------
@file	rng_nrfx.cpp

@brief	Nordic hardware random generator on the OO RngEngine facet.

@author	Hoang Nguyen Hoan
@date	Aug. 9, 2024

@license MIT, (c) 2024 I-SYST inc.
----------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>

#include "nrf.h"
#include "istddef.h"
#include "crypto/icrypto.h"
#include "crypto_rng_nrf.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the a configured (UART, USB, RTT, BLE, or any other
// DeviceIntrf); the trace does not assume a transport. A release build
// defines NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

#if defined(NRF54H20_XXAA)
// The nRF54H20 application and radio cores have no RNG peripheral and no
// CRACEN; the secure domain owns them. The nRF54H20 libraries do not link this
// file.
#error "rng_nrfx.cpp: no RNG peripheral or CRACEN on the nRF54H20 application and radio cores"
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
#include "cracen_intrf.h"

#define RNG_USE_CRACEN	1
#elif defined(NRF91_SERIES) || defined(NRF5340_XXAA_APPLICATION)
// nRF91 and the nRF5340 application core have no RNG peripheral: entropy comes from the true random
// generator of the CryptoCell, through its interface
#include "cc3xx_intrf.h"
#include "crypto_cc3xx_sym.h"

#define RNG_USE_CC3XX	1
#else
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
extern "C" void ResetEntry(void);
#endif
#endif

#ifndef RNG_NRF_POLL_LIMIT
#define RNG_NRF_POLL_LIMIT	1000000U
#endif

#if !defined(RNG_USE_CRACEN) && !defined(RNG_USE_CC3XX)

static inline NRF_RNG_Type *RngPeriphReg(void)
{
#if defined(NRF53_SERIES)
 #ifdef NRF5340_XXAA_NETWORK
	return NRF_RNG_NS;
 #else
	return NRF_RNG_S;
 #endif
#else
	return NRF_RNG;
#endif
}

static bool RngPeriphSdEnabled(void)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	// A standalone image owns the vector table at address zero. Do not issue
	// a SoftDevice SVC when no SoftDevice image is installed: SVC_Handler
	// is the application's default handler in that configuration.
	if (*(volatile const uint32_t *)(uintptr_t)4U ==
		(uint32_t)(uintptr_t)&ResetEntry)
	{
		return false;
	}
	uint8_t enabled = 0U;
	(void)sd_softdevice_is_enabled(&enabled);
	return enabled != 0U;
#else
	return false;
#endif
}

static int RngPeriphSdRandFill(uint8_t *pBuff, int BuffLen)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	int index = 0;
	uint32_t polls = RNG_NRF_POLL_LIMIT;
	while (index < BuffLen && polls-- > 0U)
	{
		uint8_t available = 0U;
		if (sd_rand_application_bytes_available_get(&available) != NRF_SUCCESS ||
			available == 0U)
		{
			continue;
		}
		int count = BuffLen - index;
		if (count > (int)available) count = available;
		if (sd_rand_application_vector_get(&pBuff[index], (uint8_t)count) ==
			NRF_SUCCESS)
		{
			index += count;
		}
	}
	return index == BuffLen ? BuffLen : 0;
#else
	(void)pBuff;
	(void)BuffLen;
	return 0;
#endif
}

static bool s_bPeriphSdXfer;

static void RngPeriphDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}
static void RngPeriphEnable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}
static uint32_t RngPeriphGetRate(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	return 0U;
}
static uint32_t RngPeriphSetRate(DevIntrf_t * const pDevIntrf, uint32_t Rate)
{
	(void)pDevIntrf;
	(void)Rate;
	return 0U;
}

static bool RngPeriphStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf;
	(void)DevAddr;
	s_bPeriphSdXfer = RngPeriphSdEnabled();
	return true;
}

static int RngPeriphRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff,
							   int BuffLen)
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
	reg->CONFIG = RNG_CONFIG_DERCEN_Enabled;
	reg->TASKS_START = 1U;
	int count = 0;
	for (; count < BuffLen; count++)
	{
		reg->EVENTS_VALRDY = 0U;
		uint32_t polls = RNG_NRF_POLL_LIMIT;
		while (reg->EVENTS_VALRDY == 0U && polls-- > 0U) { }
		if (reg->EVENTS_VALRDY == 0U)
		{
			break;
		}
		pBuff[count] = (uint8_t)reg->VALUE;
	}
	reg->TASKS_STOP = 1U;
	reg->CONFIG = RNG_CONFIG_DERCEN_Disabled;
	return count == BuffLen ? BuffLen : 0;
}

static void RngPeriphStopRx(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	s_bPeriphSdXfer = false;
}
static bool RngPeriphStartTx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr)
{
	(void)pDevIntrf; (void)DevAddr; return false;
}
static int RngPeriphTxData(DevIntrf_t * const pDevIntrf,
						   const uint8_t *pData, int DataLen)
{
	(void)pDevIntrf; (void)pData; (void)DataLen; return 0;
}
static int RngPeriphTxSrData(DevIntrf_t * const pDevIntrf,
							 const uint8_t *pData, int DataLen)
{
	(void)pDevIntrf; (void)pData; (void)DataLen; return 0;
}
static void RngPeriphStopTx(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}
static void RngPeriphReset(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	s_bPeriphSdXfer = false;
}
static void RngPeriphPowerOff(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	NRF_RNG_Type *reg = RngPeriphReg();
	reg->TASKS_STOP = 1U;
	reg->CONFIG = RNG_CONFIG_DERCEN_Disabled;
}
static void *RngPeriphGetHandle(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	return RngPeriphReg();
}

bool RngPeriphIntrf::Init(void)
{
	memset(&vDevIntrf, 0, sizeof(vDevIntrf));
	vDevIntrf.pDevData = this;
	vDevIntrf.EvtCB = nullptr;
	atomic_flag_clear(&vDevIntrf.bBusy);
	vDevIntrf.MaxRetry = 5;
	atomic_store(&vDevIntrf.EnCnt, 0);
	vDevIntrf.Type = DEVINTRF_TYPE_CRYPTO;
	vDevIntrf.bDma = false;
	vDevIntrf.bIntEn = false;
	atomic_store(&vDevIntrf.bTxReady, true);
	atomic_store(&vDevIntrf.bNoStop, false);
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

RngPeriphIntrf *RngPeriphIntrfInstance(void)
{
	static RngPeriphIntrf instance;
	static bool initialized = false;
	if (!initialized)
	{
		initialized = instance.Init();
	}
	return initialized ? &instance : nullptr;
}

#endif

bool CryptoRngNrf::Init(DeviceIntrf * const pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	if (vbIntrfEnabled)
	{
		return Interface() == pIntrf && Enable();
	}
#if defined(RNG_USE_CC3XX)
	DrbgClear();
#endif
	Interface(pIntrf);
#if defined(RNG_USE_CRACEN)
	DeviceAddress(CRACEN_ADDR_RNG);
#elif defined(RNG_USE_CC3XX)
	DeviceAddress(CC3XX_ADDR_RNG);
#else
	DeviceAddress(0U);
#endif
	return Enable();
}

bool CryptoRngNrf::Enable(void)
{
	if (Interface() == nullptr)
	{
		vbValid = false;
		return false;
	}
	if (vbIntrfEnabled && vbValid)
	{
		return true;
	}
	if (atomic_flag_test_and_set(&vOpBusy))
	{
		return false;
	}
	if (!vbIntrfEnabled)
	{
		Interface()->Enable();
		vbIntrfEnabled = true;
	}
	vbValid = true;
	atomic_flag_clear(&vOpBusy);
	return true;
}

void CryptoRngNrf::Disable(void)
{
	if (atomic_flag_test_and_set(&vOpBusy))
	{
		return;
	}
	vbValid = false;
#if defined(RNG_USE_CC3XX)
	DrbgClear();
#endif
	if (vbIntrfEnabled && Interface() != nullptr)
	{
		Interface()->Disable();
		vbIntrfEnabled = false;
	}
	atomic_flag_clear(&vOpBusy);
}

void CryptoRngNrf::Reset(void)
{
	if (atomic_flag_test_and_set(&vOpBusy))
	{
		return;
	}
	vbValid = false;
#if defined(RNG_USE_CC3XX)
	DrbgClear();
#endif
	if (vbIntrfEnabled && Interface() != nullptr)
	{
#if !defined(RNG_USE_CRACEN)
		Interface()->Reset();
#endif
		vbValid = true;
	}
	atomic_flag_clear(&vOpBusy);
}

CRYPTO_STATUS CryptoRngNrf::Random(uint8_t *pOut, size_t Len)
{
#if defined(RNG_USE_CC3XX)
	CRYPTO_STATUS status;
#endif
	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	if (!vbValid || !vbIntrfEnabled || Interface() == nullptr || pOut == nullptr ||
		Len > (size_t)INT_MAX)
	{
		DEBUG_PRINTF("RngNrf Random: valid=%d enabled=%d out=%p len=%u -> FAIL\r\n",
					 (int)vbValid, (int)vbIntrfEnabled, (void *)pOut,
					 (unsigned)Len);
		return CRYPTO_STATUS_FAIL;
	}
	if (atomic_flag_test_and_set(&vOpBusy))
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_BUSY;
	}
#if defined(RNG_USE_CC3XX)
	status = DrbgGenerate(pOut, Len);
	atomic_flag_clear(&vOpBusy);
	return status;
#else
	if (!Interface()->StartRx(DeviceAddress()))
	{
		atomic_flag_clear(&vOpBusy);
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_BUSY;
	}

	int count = Interface()->RxData(pOut, (int)Len);
	Interface()->StopRx();
	atomic_flag_clear(&vOpBusy);
	if (count != (int)Len)
	{
		DEBUG_PRINTF("RngNrf Random: RxData returned %d of %u -> FAIL\r\n",
					 count, (unsigned)Len);
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
#endif
}

#if defined(RNG_USE_CC3XX)
// SP 800-90A rev. 1, 10.1.2: HMAC_DRBG with SHA-256, no additional input.
// The first 48 entropy bytes supply a 32-byte seed and 16-byte nonce.
// Reseed after 1024 requests or 64 KiB. Large calls are split at that bound.
// The entropy interface retains its hardware health tests and raw-read API.
void CryptoRngNrf::DrbgClear()
{
	CryptoSecureWipe(vDrbgKey, sizeof(vDrbgKey));
	CryptoSecureWipe(vDrbgValue, sizeof(vDrbgValue));
	vDrbgCalls = 0U;
	vDrbgBytes = 0U;
}

static CRYPTO_STATUS DrbgMac(HashEngine *pHash, const uint8_t Key[32],
	const uint8_t *pMsg, size_t Len, uint8_t Out[32])
{
	CryptoKey key = {};
	key.Type = CRYPTO_KEY_HMAC;
	key.Loc = CRYPTO_KEY_LOC_PLAIN;
	key.Usage = CRYPTO_KEY_USE_SIGN;
	key.Plain.pData = Key;
	key.Plain.Len = 32U;
	return CryptoHmacSha256(pHash, CRYPTO_MAC_HMAC, key, pMsg, Len, Out, 32U);
}

CRYPTO_STATUS CryptoRngNrf::DrbgUpdate(HashEngine *pHash,
	const uint8_t *pSeed, size_t Len)
{
	uint8_t input[81];
	unsigned round;
	CRYPTO_STATUS status = CRYPTO_STATUS_OK;

	for (round = 0U; round < (Len == 0U ? 1U : 2U); round++)
	{
		memcpy(input, vDrbgValue, 32U);
		input[32] = (uint8_t)round;
		if (Len != 0U)
		{
			memcpy(input + 33U, pSeed, Len);
		}
		status = DrbgMac(pHash, vDrbgKey, input, 33U + Len, vDrbgKey);
		if (status != CRYPTO_STATUS_OK)
		{
			break;
		}
		status = DrbgMac(pHash, vDrbgKey, vDrbgValue, 32U, vDrbgValue);
		if (status != CRYPTO_STATUS_OK)
		{
			break;
		}
	}
	CryptoSecureWipe(input, sizeof(input));
	return status;
}

CRYPTO_STATUS CryptoRngNrf::DrbgGenerate(uint8_t *pOut, size_t Len)
{
	CryptoCc3xxSym hash;
	Cc3xxIntrf *intrf = Cc3xxIntrfInstance();
	uint8_t seed[48];
	size_t offset = 0U;
	size_t chunk;
	size_t remaining;
	size_t count;
	int received;
	CRYPTO_STATUS status = CRYPTO_STATUS_OK;

	// Report an occupied engine before HMAC starts changing the DRBG state.
	// Each SHA block still takes its own operation hold, allowing other
	// crypto operations between blocks without retaining hardware secrets.
	if (intrf != nullptr && !intrf->OpHold(this))
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_BUSY;
	}
	if (intrf != nullptr)
	{
		intrf->OpRelease(this);
	}
	if (!hash.Init(intrf))
	{
		status = CRYPTO_STATUS_FAIL;
	}
	while (offset < Len && status == CRYPTO_STATUS_OK)
	{
		if (vDrbgCalls == 0U || vDrbgCalls > 1024U || vDrbgBytes == 65536U)
		{
			if (!Interface()->StartRx(DeviceAddress()))
			{
				status = CRYPTO_STATUS_BUSY;
				break;
			}
			received = Interface()->RxData(seed, sizeof(seed));
			Interface()->StopRx();
			if (received != (int)sizeof(seed))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
			if (vDrbgCalls == 0U)
			{
				memset(vDrbgKey, 0, sizeof(vDrbgKey));
				memset(vDrbgValue, 1, sizeof(vDrbgValue));
			}
			status = DrbgUpdate(&hash, seed, sizeof(seed));
			CryptoSecureWipe(seed, sizeof(seed));
			if (status != CRYPTO_STATUS_OK)
			{
				break;
			}
			vDrbgCalls = 1U;
			vDrbgBytes = 0U;
		}
		chunk = Len - offset;
		if (chunk > 65536U - vDrbgBytes)
		{
			chunk = 65536U - vDrbgBytes;
		}
		remaining = chunk;
		while (remaining != 0U)
		{
			status = DrbgMac(&hash, vDrbgKey, vDrbgValue, 32U, vDrbgValue);
			if (status != CRYPTO_STATUS_OK)
			{
				break;
			}
			count = remaining < 32U ? remaining : 32U;
			memcpy(pOut + offset, vDrbgValue, count);
			offset += count;
			remaining -= count;
		}
		if (status == CRYPTO_STATUS_OK)
		{
			status = DrbgUpdate(&hash, nullptr, 0U);
			vDrbgBytes += chunk;
			vDrbgCalls++;
		}
	}
	hash.Disable();
	CryptoSecureWipe(seed, sizeof(seed));
	if (status != CRYPTO_STATUS_OK)
	{
		DrbgClear();
		CryptoSecureWipe(pOut, Len);
	}
	return status;
}
#endif

CryptoRngNrf *CryptoRngNrfInstance(void)
{
	static CryptoRngNrf instance;
	static bool initialized = false;
	if (!initialized)
	{
#if defined(RNG_USE_CRACEN)
		initialized = instance.Init(CracenIntrfInstance());
#elif defined(RNG_USE_CC3XX)
		initialized = instance.Init(Cc3xxIntrfInstance());
#else
		initialized = instance.Init(RngPeriphIntrfInstance());
#endif
	}
	return initialized ? &instance : nullptr;
}

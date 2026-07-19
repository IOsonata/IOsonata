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
#include "syslog.h"

#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
#include "cracen_intrf.h"

#define RNG_USE_CRACEN	1
#else
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif
#endif

#define RNGNRF_TRACE_ENABLE
#if defined(RNGNRF_TRACE_ENABLE)
#define RNGNRF_TRACE(...)	SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define RNGNRF_TRACE(...)
#endif

#ifndef RNG_NRF_POLL_LIMIT
#define RNG_NRF_POLL_LIMIT	1000000U
#endif

#if !defined(RNG_USE_CRACEN)

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

static bool RngPeriphSdEnabled(void)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
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

#endif // !RNG_USE_CRACEN

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
	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	if (!vbValid || !vbIntrfEnabled || Interface() == nullptr || pOut == nullptr ||
		Len > (size_t)INT_MAX)
	{
		RNGNRF_TRACE("RngNrf Random: valid=%d enabled=%d out=%p len=%u -> FAIL\r\n",
					 (int)vbValid, (int)vbIntrfEnabled, (void *)pOut,
					 (unsigned)Len);
		return CRYPTO_STATUS_FAIL;
	}
	if (atomic_flag_test_and_set(&vOpBusy))
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_BUSY;
	}
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
		RNGNRF_TRACE("RngNrf Random: RxData returned %d of %u -> FAIL\r\n",
					 count, (unsigned)Len);
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

CryptoRngNrf *CryptoRngNrfInstance(void)
{
	static CryptoRngNrf instance;
	static bool initialized = false;
	if (!initialized)
	{
#if defined(RNG_USE_CRACEN)
		initialized = instance.Init(CracenIntrfInstance());
#else
		initialized = instance.Init(RngPeriphIntrfInstance());
#endif
	}
	return initialized ? &instance : nullptr;
}

/**-------------------------------------------------------------------------
@file	cracen_intrf.cpp

@brief	Device interface to the Nordic on-die crypto core (CRACEN).

		Leaf DeviceIntrf that owns the CRACENCORE register and operand memory
		access. CRACEN is not a crypto engine; it is the access path the Silex
		engines sit on. An engine reaches it through the inherited Device Read /
		Write, exactly as a sensor reaches an SPI or I2C bus.

		The transfer follows the same pattern as SPI and I2C: StartTx or StartRx
		receives the DevAddr and saves it. TxSrData latches the byte offset from
		the address phase; TxData writes and RxData reads at the saved base plus
		the latched offset. Registers are accessed as 32-bit words. PKE
		operand memory uses target-appropriate accesses.

		On CRACEN Lite, PKE operand memory accesses use aligned 32-bit
		transfers. This is
		required by CRACEN Lite (nRF54LM20); the source/destination buffers
		themselves may remain byte aligned.

		Every transfer is self-addressing through the DevAddr module
		selector, so no cross-operation state lives here. Module power is
		owned by the transport Enable/Disable through the enable reference
		count. The interface Reset is address-selected: it recovers only the
		module of the DevAddr latched by the surrounding StartTx, the same
		way a transfer selects its module. The whole core resets only when
		no module was ever selected.

@author	Hoang Nguyen Hoan
@date	Jul 2026

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
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "nrf.h"

#include "cracen_intrf.h"
#include "cracen_ba414e_ucode.h"
#include "crypto/ba414ep.h"
#include "crypto/cryptomaster.h"

#include "hal/nrf_cracen_rng.h"
#include "crypto/silex_internal.h"
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#ifndef CRACEN_SD_RNG_POLL_LIMIT
#define CRACEN_SD_RNG_POLL_LIMIT	1000000U
#endif

#define CRACEN_ENABLE_ALL_Msk	(CRACEN_ENABLE_CRYPTOMASTER_Msk | \
								 CRACEN_ENABLE_PKEIKG_Msk | \
								 CRACEN_ENABLE_RNG_Msk)

// PKE code RAM offset from the CRACENCORE base.
#define BA414EP_CODE_OFFSET		0xC000U

// No module selected yet; an interface Reset in this state resets the core.
#define CRACEN_ADDR_NONE		0xFFFFFFFFU

//---------------------------------------------------------------------------
// Per die state. There is one CRACEN core, so these describe the hardware
// rather than a transaction: where its engines sit.
//---------------------------------------------------------------------------
static volatile uint8_t *s_pPkeRegBase;
static volatile uint8_t *s_pCmRegBase;
static volatile uint8_t *s_pMemBase;
#ifndef CRACEN_RNG_POLL_LIMIT
#define CRACEN_RNG_POLL_LIMIT 1000000U
#endif

// Which engine a transfer addresses and where in it are per transaction, so
// they live in CracenIntrfXfer_t behind pDevData.
static inline CracenIntrfXfer_t *CracenXfer(DevIntrf_t * const pIntrf)
{
	return (CracenIntrfXfer_t *)pIntrf->pDevData;
}

static void CracenSelectBase(CracenIntrfXfer_t *pXfer, uint32_t DevAddr)
{
	pXfer->SelDevAddr = DevAddr;
	pXfer->bXferMem = (DevAddr == BA414EP_ADDR_MEM);
	pXfer->pXferBase = pXfer->bXferMem ? s_pMemBase :
					   (DevAddr == CRYPTOMASTER_ADDR_REG) ? s_pCmRegBase
														: s_pPkeRegBase;
}

static bool CracenStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	CracenIntrfXfer_t *xfer = CracenXfer(pIntrf);

	CracenSelectBase(xfer, DevAddr);
	xfer->bAddrLatched = false;

	return true;
}

static bool CracenSdEnabled(void)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	uint8_t en = 0;
	(void)sd_softdevice_is_enabled(&en);
	return en != 0;
#else
	return false;
#endif
}

static bool CracenStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	CracenIntrfXfer_t *xfer = CracenXfer(pIntrf);

	if (DevAddr == CRACEN_ADDR_RNG)
	{
		if (atomic_load(&pIntrf->EnCnt) == 0 || !SilexOpAcquire())
		{
			return false;
		}
		xfer->SelDevAddr = DevAddr;
		xfer->bRngXfer = true;

		return true;
	}

	xfer->bRngXfer = false;
	CracenSelectBase(xfer, DevAddr);

	return true;
}

static int CracenTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	CracenIntrfXfer_t *xfer = CracenXfer(pIntrf);

	if (pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	int consumed = 0;
	if (!xfer->bAddrLatched)
	{
		int n = (DataLen < 4) ? DataLen : 4;
		uint32_t off = 0U;
		for (int i = 0; i < n; i++)
		{
			off |= (uint32_t)pData[i] << (8 * i);
		}
		xfer->Offset = off;
		xfer->bAddrLatched = true;
		consumed = n;
	}

	int len = DataLen - consumed;
	const uint8_t *p = &pData[consumed];
	if (xfer->bXferMem)
	{
#if defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		// CRACEN Lite requires word-aligned, word-sized PKE RAM accesses.
		// All BA414EP operands currently start on a word boundary. Preserve
		// bytes outside a partial final word rather than issuing byte stores.
		if ((xfer->Offset & 3U) != 0U)
		{
			return consumed;
		}

		volatile uint32_t *w =
			(volatile uint32_t *)(xfer->pXferBase + xfer->Offset);
		int i = 0;
		for (; i + 4 <= len; i += 4)
		{
			w[i / 4] = (uint32_t)p[i] |
					   ((uint32_t)p[i + 1] << 8) |
					   ((uint32_t)p[i + 2] << 16) |
					   ((uint32_t)p[i + 3] << 24);
		}
		if (i < len)
		{
			uint32_t v = w[i / 4];
			for (int j = 0; i + j < len; j++)
			{
				uint32_t sh = (uint32_t)j * 8U;
				v = (v & ~(0xFFU << sh)) |
					((uint32_t)p[i + j] << sh);
			}
			w[i / 4] = v;
		}
#else
		volatile uint8_t *d = xfer->pXferBase + xfer->Offset;
		for (int i = 0; i < len; i++)
		{
			d[i] = p[i];
		}
#endif
	}
	else
	{
		volatile uint32_t *w =
				(volatile uint32_t *)(xfer->pXferBase + xfer->Offset);
		for (int i = 0; i + 4 <= len; i += 4)
		{
			w[i / 4] = (uint32_t)p[i] | ((uint32_t)p[i + 1] << 8) |
					   ((uint32_t)p[i + 2] << 16) | ((uint32_t)p[i + 3] << 24);
		}
	}
	return DataLen;
}

static int CracenTxSrData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	return CracenTxData(pIntrf, pData, DataLen);
}

static int CracenSdRandFill(uint8_t *pBuff, int BuffLen)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	int idx = 0;
	uint32_t polls = CRACEN_SD_RNG_POLL_LIMIT;
	while (idx < BuffLen && polls-- > 0U)
	{
		uint8_t avail = 0;
		if (sd_rand_application_bytes_available_get(&avail) != NRF_SUCCESS ||
			avail == 0U)
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
	return idx == BuffLen ? BuffLen : 0;
#else
	(void)pBuff;
	(void)BuffLen;
	return 0;
#endif
}

// TRNG configuration follows nrfx_cracen.c v4.5.0, with bounded waits and
// failure propagation. Its license applies to the adapted configuration below.
/*
 * Copyright (c) 2025 - 2026, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// Use the RNG's own conditioner. No CryptoMaster DMA or CTR-DRBG runs here.
static void CracenTrngStart()
{
	nrf_cracen_rng_control_t control = {};
	control.soft_reset = true;
	nrf_cracen_rng_control_set(NRF_CRACENCORE, &control);
#if NRF_CRACEN_RNG_HAS_IDLE_TIMER
	nrf_cracen_rng_off_timer_set(NRF_CRACENCORE, 0U);
#endif
#if NRF_CRACEN_HAS_RECOMMENDED_CONF
	nrf_cracen_rng_clk_div_set(NRF_CRACENCORE,
		CRACENCORE_RNGCONTROL_SAMPLINGPERIOD_SAMPLINGPERIOD_Recommended);
	nrf_cracen_rng_init_wait_val_set(NRF_CRACENCORE,
		CRACENCORE_RNGCONTROL_WARMUPPERIOD_WARMUPPERIOD_Recommended);
	control.number_128_blocks = CRACENCORE_RNGCONTROL_CONTROL_NB128BITBLOCKS_Recommended;
#if NRF_CRACEN_RNG_HAS_FIFOWRITESTARTUP
	control.fifo_write_startup = CRACENCORE_RNGCONTROL_CONTROL_FIFOWRITESTARTUP_Recommended;
#endif
#if NRF_CRACEN_RNG_HAS_REPEATTHRESHOLD
	nrf_cracen_rng_repeatthreshold_set(NRF_CRACENCORE,
		CRACENCORE_RNGCONTROL_REPEATTHRESHOLD_REPEATTHRESHOLD_Recommended);
#endif
#if NRF_CRACEN_RNG_HAS_PROPTESTCUTOFF
	nrf_cracen_rng_proptestcutoff_set(NRF_CRACENCORE,
		CRACENCORE_RNGCONTROL_PROPTESTCUTOFF_PROPTESTCUTOFF_Recommended);
#endif
#if NRF_CRACEN_RNG_HAS_DISABLEOSC
	nrf_cracen_rng_disableosc_set(NRF_CRACENCORE, 0,
		CRACENCORE_RNGCONTROL_DISABLEOSC_DISABLEOSC_Recommended);
#endif
#if NRF_CRACEN_RNG_HAS_COOLDOWNPERIOD
	nrf_cracen_rng_cooldown_period_set(NRF_CRACENCORE,
		CRACENCORE_RNGCONTROL_COOLDOWNPERIOD_COOLDOWNPERIOD_Recommended);
#endif
#else
	nrf_cracen_rng_clk_div_set(NRF_CRACENCORE, TRNG_CLK_DIV);
	nrf_cracen_rng_init_wait_val_set(NRF_CRACENCORE, 512U);
	control.number_128_blocks = 4U;
#endif
#if NRF_CRACEN_RNG_HAS_BLENDING
	control.blending_method = NRF_CRACEN_RNG_BLENDING_CONCATENATION;
#endif
	control.soft_reset = false;
	control.enable = true;
	nrf_cracen_rng_control_set(NRF_CRACENCORE, &control);
}

static bool CracenTrngWord(uint32_t *pWord)
{
	uint32_t polls;
	nrf_cracen_rng_fsm_state_t state;
	for (polls = 0; polls < CRACEN_RNG_POLL_LIMIT; polls++)
	{
		state = nrf_cracen_rng_fsm_state_get(NRF_CRACENCORE);
		if (state == NRF_CRACEN_RNG_FSM_STATE_ERROR)
		{
			return false;
		}
		if (state != NRF_CRACEN_RNG_FSM_STATE_RESET &&
			state != NRF_CRACEN_RNG_FSM_STATE_STARTUP &&
			nrf_cracen_rng_fifo_level_get(NRF_CRACENCORE) != 0U)
		{
			*pWord = nrf_cracen_rng_fifo_get(NRF_CRACENCORE);
			return nrf_cracen_rng_fsm_state_get(NRF_CRACENCORE) != NRF_CRACEN_RNG_FSM_STATE_ERROR;
		}
	}
	return false;
}

static int CracenTrngRead(uint8_t *pBuff, int BuffLen)
{
	nrf_cracen_rng_control_t control = {};
	uint32_t word = 0U;
	int count = 0;
	unsigned i;
	bool ok = true;
	CracenTrngStart();
	// The first four words seed the hardware conditioning function.
	for (i = 0U; i < 4U && ok; i++)
	{
		ok = CracenTrngWord(&word);
		if (ok)
		{
			nrf_cracen_rng_key_set(NRF_CRACENCORE, i, word);
		}
	}
	while (count < BuffLen && ok)
	{
		ok = CracenTrngWord(&word);
		if (ok)
		{
			for (i = 0U; i < 4U && count < BuffLen; i++, count++)
			{
				pBuff[count] = (uint8_t)(word >> (8U * i));
			}
		}
	}
	control.soft_reset = true;
	nrf_cracen_rng_control_set(NRF_CRACENCORE, &control);
	for (i = 0U; i < 4U; i++)
	{
		nrf_cracen_rng_key_set(NRF_CRACENCORE, i, 0U);
	}
	CryptoSecureWipe(&word, sizeof(word));
	if (!ok)
	{
		CryptoSecureWipe(pBuff, (size_t)BuffLen);
		return 0;
	}
	return count;
}

static int CracenRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	CracenIntrfXfer_t *xfer = CracenXfer(pIntrf);

	if (pBuff == nullptr || BuffLen <= 0)
	{
		return 0;
	}

	if (xfer->bRngXfer)
	{
		if (CracenSdEnabled())
		{
			return CracenSdRandFill(pBuff, BuffLen);
		}

		return CracenTrngRead(pBuff, BuffLen);
	}

	if (!xfer->bAddrLatched)
	{
		return 0;
	}

	if (xfer->bXferMem)
	{
#if defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		if ((xfer->Offset & 3U) != 0U)
		{
			return 0;
		}

		const volatile uint32_t *w =
			(const volatile uint32_t *)(xfer->pXferBase + xfer->Offset);
		for (int i = 0; i < BuffLen; )
		{
			uint32_t v = w[i / 4];
			int n = BuffLen - i;
			if (n > 4)
			{
				n = 4;
			}
			for (int j = 0; j < n; j++)
			{
				pBuff[i + j] = (uint8_t)(v >> ((uint32_t)j * 8U));
			}
			i += n;
		}
#else
		const volatile uint8_t *d = xfer->pXferBase + xfer->Offset;
		for (int i = 0; i < BuffLen; i++)
		{
			pBuff[i] = d[i];
		}
#endif
	}
	else
	{
		const volatile uint32_t *w =
			(const volatile uint32_t *)(xfer->pXferBase + xfer->Offset);
		for (int i = 0; i + 4 <= BuffLen; i += 4)
		{
			uint32_t v = w[i / 4];
			pBuff[i]     = (uint8_t)v;
			pBuff[i + 1] = (uint8_t)(v >> 8);
			pBuff[i + 2] = (uint8_t)(v >> 16);
			pBuff[i + 3] = (uint8_t)(v >> 24);
		}
	}
	return BuffLen;
}

static void CracenStopTx(DevIntrf_t * const pIntrf)
{
	CracenXfer(pIntrf)->bAddrLatched = false;
}

static void CracenStopRx(DevIntrf_t * const pIntrf)
{
	CracenIntrfXfer_t *xfer = CracenXfer(pIntrf);

	if (xfer->bRngXfer)
	{
		xfer->bRngXfer = false;
		SilexOpRelease();

		return;
	}

	xfer->bAddrLatched = false;
}

// CRACEN Base devices that use writable PKE code RAM require the BA414EP
// microcode at NRF_CRACENCORE + BA414EP_CODE_OFFSET. Initialization verifies
// the complete retained image before taking the fast path. CRACEN Lite
// (nRF54LM20) does not use this upload path.
static bool CracenLoadPkeMicrocode(void)
{
#if defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	// nRF54LM20 is CRACEN Lite. Nordic's driver only enables microcode
	// loading for CRACEN Base; the Lite PKE does not use this upload path.
	return true;
#else
	volatile uint32_t *pCode = (volatile uint32_t *)
		((uintptr_t)NRF_CRACENCORE + BA414EP_CODE_OFFSET);
	const size_t words = CRACEN_BA414E_UCODE_WORDS;

	bool match = true;
	for (size_t i = 0; i < words; i++)
	{
		if (pCode[i] != s_CracenBa414eUcode[i])
		{
			match = false;
			break;
		}
	}
	if (match)
	{
		return true;
	}

	for (size_t i = 0; i < words; i++)
	{
		pCode[i] = s_CracenBa414eUcode[i];
	}
	__DSB();
	for (size_t i = 0; i < words; i++)
	{
		if (pCode[i] != s_CracenBa414eUcode[i])
		{
			return false;
		}
	}
	return true;
#endif
}

// Transport power, driven by the EnCnt reference count: the first engine
// Enable powers the whole core (all three modules, the way the sdk-nrf
// cracen_acquire does), the last Disable powers it off. Retained state
// (microcode RAM and handover) survives these cycles. NRF_CRACEN is the
// Nordic wrapper and lives only here; the Silex engines never touch it.
static void CracenCoreDisable(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	NRF_CRACEN->ENABLE &= ~CRACEN_ENABLE_ALL_Msk;
	__DMB();
}

static void CracenCoreEnable(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	NRF_CRACEN->ENABLE |= CRACEN_ENABLE_ALL_Msk;
	__DMB();
}
static uint32_t CracenGetRate(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	return 0U;
}
static uint32_t CracenSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	(void)pIntrf;
	(void)Rate;
	return 0U;
}

// Interface reset, selected by the DevAddr the transport last latched. An
// engine resets its own module by opening a transfer on its DevAddr, calling
// Reset, then closing the transfer: the StartTx/StopTx pair serializes the
// reset against the other engines and makes the module selection explicit.
// Only the selected module power-cycles; the other engines are untouched.
// A failed restore surfaces through the readiness probes the engine runs
// after every reset, so this stays a void interface function.
static void CracenReset(DevIntrf_t * const pIntrf)
{
	CracenIntrfXfer_t *xfer = CracenXfer(pIntrf);
	uint32_t mask;

	switch (xfer->SelDevAddr)
	{
		case BA414EP_ADDR_REG:
		case BA414EP_ADDR_MEM:
			mask = CRACEN_ENABLE_PKEIKG_Msk;
			break;
		case CRYPTOMASTER_ADDR_REG:
			mask = CRACEN_ENABLE_CRYPTOMASTER_Msk;
			break;
		case CRACEN_ADDR_RNG:
			mask = CRACEN_ENABLE_RNG_Msk;
			break;
		default:
			mask = CRACEN_ENABLE_ALL_Msk;
			break;
	}

	NRF_CRACEN->ENABLE &= ~mask;
	__DMB();
	if (atomic_load(&pIntrf->EnCnt) > 0)
	{
		NRF_CRACEN->ENABLE |= mask;
		__DMB();
		if ((mask & CRACEN_ENABLE_PKEIKG_Msk) != 0U)
		{
			(void)CracenLoadPkeMicrocode();
		}
	}

	xfer->bAddrLatched = false;
}
static void CracenPowerOff(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	// Module power is owned by the transport Enable/Disable (whole core on).
}
static void *CracenGetHandle(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	return (void *)NRF_CRACENCORE;
}

bool CracenIntrf::Init(void)
{
	s_pPkeRegBase = (volatile uint8_t *)&NRF_CRACENCORE->PK;
	s_pCmRegBase = (volatile uint8_t *)NRF_CRACENCORE;
	s_pMemBase = (volatile uint8_t *)((uintptr_t)NRF_CRACENCORE +
									 BA414EP_CRYPTORAM_OFFSET);

	// Prepare the PKE implementation while its module is powered. CRACEN Base
	// loads/verifies the BA414EP microcode; CRACEN Lite has no upload step.
	// The IK handover and readiness are the generic engine's concern, proven
	// by its first Enable.
	const uint32_t prev = NRF_CRACEN->ENABLE & CRACEN_ENABLE_PKEIKG_Msk;
	NRF_CRACEN->ENABLE |= CRACEN_ENABLE_PKEIKG_Msk;
	__DMB();
	const bool bUcodeOk = CracenLoadPkeMicrocode();
	if (prev == 0U)
	{
		NRF_CRACEN->ENABLE &= ~CRACEN_ENABLE_PKEIKG_Msk;
		__DMB();
	}
	if (!bUcodeOk)
	{
		return false;
	}

	memset(&vXfer, 0, sizeof(vXfer));
	vXfer.pXferBase = s_pPkeRegBase;
	vXfer.SelDevAddr = CRACEN_ADDR_NONE;

	memset(&vDevIntrf, 0, sizeof(vDevIntrf));

	// pDevData is the transaction state, so a transfer callback reaches it
	// without knowing anything about this class.
	vDevIntrf.pDevData = &vXfer;
	vDevIntrf.IntPrio = 0;
	vDevIntrf.EvtCB = nullptr;
	vDevIntrf.MaxRetry = 5;
	vDevIntrf.Type = DEVINTRF_TYPE_CRYPTO;
	vDevIntrf.bDma = false;
	vDevIntrf.bIntEn = false;
	atomic_flag_clear(&vDevIntrf.bBusy);
	atomic_store(&vDevIntrf.EnCnt, 0);
	atomic_store(&vDevIntrf.bTxReady, true);
	atomic_store(&vDevIntrf.bNoStop, false);

	vDevIntrf.Disable = CracenCoreDisable;
	vDevIntrf.Enable = CracenCoreEnable;
	vDevIntrf.GetRate = CracenGetRate;
	vDevIntrf.SetRate = CracenSetRate;
	vDevIntrf.StartRx = CracenStartRx;
	vDevIntrf.RxData = CracenRxData;
	vDevIntrf.StopRx = CracenStopRx;
	vDevIntrf.StartTx = CracenStartTx;
	vDevIntrf.TxData = CracenTxData;
	vDevIntrf.TxSrData = CracenTxSrData;
	vDevIntrf.StopTx = CracenStopTx;
	vDevIntrf.Reset = CracenReset;
	vDevIntrf.PowerOff = CracenPowerOff;
	vDevIntrf.GetHandle = CracenGetHandle;
	return true;
}

CracenIntrf *CracenIntrfInstance(void)
{
	static CracenIntrf s_Instance;
	static bool s_bInit = false;
	if (!s_bInit)
	{
		s_bInit = s_Instance.Init();
	}
	return s_bInit ? &s_Instance : nullptr;
}

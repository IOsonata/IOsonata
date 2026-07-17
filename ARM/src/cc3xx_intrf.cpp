/**-------------------------------------------------------------------------
@file	cc3xx_intrf.cpp

@brief	DeviceIntrf over the Arm CryptoCell CC3xx register file.

		Implementation notes. StartTx / StartRx save the DevAddr selector and
		open a fresh address phase; the first bytes of the Tx data (or the
		TxSrData of a read) latch a little-endian register offset; the data
		phase then moves 32-bit words at base plus offset. All CC3xx access
		is word access. OpHold / OpRelease is the operation-level exclusion,
		distinct from the per transfer busy the framework manages. Enable and
		Disable hooks power the CryptoCell wrapper by the interface reference
		count.

@author	Hoang Nguyen Hoan
@date	Jul. 17, 2026

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
#include <stdatomic.h>
#include <stdint.h>

#include "istddef.h"
#include "cc3xx_intrf.h"

static atomic_flag s_HoldFlag = ATOMIC_FLAG_INIT;
static const void *s_pHoldOwner;
static uint32_t s_Offset;				// byte offset latched from address phase
static bool s_bAddrLatched;

static uintptr_t s_Base;				// register file base, cached at Init

static inline volatile uint32_t *Cc3xxWord(uint32_t Offset)
{
	return (volatile uint32_t *)(s_Base + Offset);
}

static void Cc3xxIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	Cc3xxDisable();
}

static void Cc3xxIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	(void)Cc3xxEnable();
}

static uint32_t Cc3xxGetRate(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; return 0; }
static uint32_t Cc3xxSetRate(DevIntrf_t * const pDevIntrf, uint32_t Rate) {
	(void)pDevIntrf; (void)Rate; return 0;
}

static bool Cc3xxStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf; (void)DevAddr;
	// Restart condition inside a read: keep the offset latched by the
	// preceding address phase.
	return true;
}

static int Cc3xxRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	(void)pIntrf;
	if (pBuff == nullptr || BuffLen <= 0 || !s_bAddrLatched)
	{
		return 0;
	}
	volatile uint32_t *w = Cc3xxWord(s_Offset);
	for (int i = 0; i + 4 <= BuffLen; i += 4)
	{
		uint32_t v = w[i / 4];
		pBuff[i] = (uint8_t)v;
		pBuff[i + 1] = (uint8_t)(v >> 8);
		pBuff[i + 2] = (uint8_t)(v >> 16);
		pBuff[i + 3] = (uint8_t)(v >> 24);
	}
	return BuffLen;
}

static void Cc3xxStopRx(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	s_bAddrLatched = false;
}

static bool Cc3xxStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf; (void)DevAddr;
	// Single register file: the selector is saved for protocol symmetry and
	// a fresh address phase opens.
	s_bAddrLatched = false;
	return true;
}

static int Cc3xxTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	(void)pIntrf;
	if (pData == nullptr || DataLen <= 0)
	{
		return 0;
	}
	int consumed = 0;
	if (!s_bAddrLatched)
	{
		int n = (DataLen < 4) ? DataLen : 4;
		uint32_t off = 0U;
		for (int i = 0; i < n; i++)
		{
			off |= (uint32_t)pData[i] << (8 * i);
		}
		s_Offset = off;
		s_bAddrLatched = true;
		consumed = n;
	}
	int len = DataLen - consumed;
	const uint8_t *p = &pData[consumed];
	volatile uint32_t *w = Cc3xxWord(s_Offset);
	for (int i = 0; i + 4 <= len; i += 4)
	{
		w[i / 4] = (uint32_t)p[i] | ((uint32_t)p[i + 1] << 8) |
				   ((uint32_t)p[i + 2] << 16) | ((uint32_t)p[i + 3] << 24);
	}
	return DataLen;
}

static int Cc3xxTxSrData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	return Cc3xxTxData(pIntrf, pData, DataLen);
}

static void Cc3xxStopTx(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	s_bAddrLatched = false;
}

static void Cc3xxReset(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; }
static void Cc3xxPowerOff(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	Cc3xxDisable();
}
static void *Cc3xxGetHandle(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; return nullptr; }

bool Cc3xxIntrf::Init(void)
{
	vDevIntrf.pDevData = this;
	vDevIntrf.EvtCB = nullptr;
	atomic_flag_clear(&vDevIntrf.bBusy);
	vDevIntrf.MaxRetry = 5;
	// The user count starts at zero: engines are the users. The first engine
	// Enable powers the wrapper (0 to 1) and the last engine Disable powers
	// it off, through the DeviceIntrf reference count.
	atomic_store(&vDevIntrf.EnCnt, 0);
	vDevIntrf.Type = DEVINTRF_TYPE_CRYPTO;
	vDevIntrf.bDma = false;
	vDevIntrf.bIntEn = false;
	atomic_store(&vDevIntrf.bTxReady, true);
	atomic_store(&vDevIntrf.bNoStop, false);
	vDevIntrf.MaxTrxLen = 0;
	vDevIntrf.Disable = Cc3xxIntrfDisable;
	vDevIntrf.Enable = Cc3xxIntrfEnable;
	vDevIntrf.GetRate = Cc3xxGetRate;
	vDevIntrf.SetRate = Cc3xxSetRate;
	vDevIntrf.StartRx = Cc3xxStartRx;
	vDevIntrf.RxData = Cc3xxRxData;
	vDevIntrf.StopRx = Cc3xxStopRx;
	vDevIntrf.StartTx = Cc3xxStartTx;
	vDevIntrf.TxData = Cc3xxTxData;
	vDevIntrf.TxSrData = Cc3xxTxSrData;
	vDevIntrf.StopTx = Cc3xxStopTx;
	vDevIntrf.Reset = Cc3xxReset;
	vDevIntrf.PowerOff = Cc3xxPowerOff;
	vDevIntrf.GetHandle = Cc3xxGetHandle;

	// Wiring only: cache the target base and leave the wrapper unpowered.
	// Power follows the user count.
	s_Base = Cc3xxBase();
	return true;
}

bool Cc3xxIntrf::OpHold(const void *pOwner)
{
	if (pOwner == nullptr || atomic_flag_test_and_set(&s_HoldFlag))
	{
		return false;
	}
	s_pHoldOwner = pOwner;
	return true;
}

bool Cc3xxIntrf::OpRelease(const void *pOwner)
{
	if (pOwner == nullptr || s_pHoldOwner != pOwner)
	{
		return false;
	}
	s_pHoldOwner = nullptr;
	atomic_flag_clear(&s_HoldFlag);
	return true;
}

Cc3xxIntrf *Cc3xxIntrfInstance(void)
{
	static Cc3xxIntrf s_Cc3xxIntrf;
	static bool s_bInit = false;

	if (!s_bInit)
	{
		s_bInit = s_Cc3xxIntrf.Init();
	}
	return s_bInit ? &s_Cc3xxIntrf : nullptr;
}

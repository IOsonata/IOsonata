/**-------------------------------------------------------------------------
@file	semihost_intrf.cpp

@brief	Semihosting console device interface

Generic part of the semihosting console interface. The semihosting call itself
is in the architecture port, ARM/src/semihost_arm.c or
RISCV/src/semihost_riscv.c.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

#include "semihost_intrf.h"

// Host console name for SYS_OPEN
static const char s_SemihostConsole[] = ":tt";

static void SemihostIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void SemihostIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

// The console has no data rate, 0 is returned for both get and set.
static uint32_t SemihostIntrfGetRate(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;

	return 0;
}

static uint32_t SemihostIntrfSetRate(DevIntrf_t * const pDevIntrf, uint32_t Rate)
{
	(void)pDevIntrf;
	(void)Rate;

	return 0;
}

// Transmit only, receive is not supported.
static bool SemihostIntrfStartRx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr)
{
	(void)pDevIntrf;
	(void)DevAddr;

	return false;
}

static int SemihostIntrfRxData(DevIntrf_t * const pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	(void)pDevIntrf;
	(void)pBuff;
	(void)BuffLen;

	return 0;
}

static void SemihostIntrfStopRx(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

// Opens the host console on first use, so a debugger connected after
// SemihostInit still gets the output. Fails when the port detects that no
// debugger is connected, nothing is sent in that case.
static bool SemihostIntrfStartTx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr)
{
	SemihostDev_t *dev = (SemihostDev_t *)pDevIntrf->pDevData;

	(void)DevAddr;

	if (SemihostDebuggerAttached() == false)
	{
		return false;
	}

	if (dev->Handle < 0)
	{
		uintptr_t arg[3] = {
			(uintptr_t)s_SemihostConsole,
			(uintptr_t)SEMIHOST_OPEN_MODE_W,
			(uintptr_t)(sizeof(s_SemihostConsole) - 1)
		};

		dev->Handle = SemihostCall(SEMIHOST_SYS_OPEN, arg);
	}

	return dev->Handle >= 0;
}

// SYS_WRITE returns the number of bytes not written, 0 when all were.
static int SemihostIntrfTxData(DevIntrf_t * const pDevIntrf, const uint8_t *pData, int DataLen)
{
	SemihostDev_t *dev = (SemihostDev_t *)pDevIntrf->pDevData;

	if (pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	uintptr_t arg[3] = {
		(uintptr_t)dev->Handle,
		(uintptr_t)pData,
		(uintptr_t)DataLen
	};

	int left = SemihostCall(SEMIHOST_SYS_WRITE, arg);

	if (left < 0 || left >= DataLen)
	{
		// Nothing written. Open the console again on the next transmit, a
		// new debugger session may have replaced the previous one.
		dev->Handle = -1;

		return 0;
	}

	return DataLen - left;
}

static int SemihostIntrfTxSrData(DevIntrf_t * const pDevIntrf, const uint8_t *pData, int DataLen)
{
	return SemihostIntrfTxData(pDevIntrf, pData, DataLen);
}

static void SemihostIntrfStopTx(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void SemihostIntrfReset(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void SemihostIntrfPowerOff(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void *SemihostIntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return pDevIntrf->pDevData;
}

bool SemihostInit(SemihostDev_t * const pDev)
{
	if (pDev == NULL)
	{
		return false;
	}

	pDev->Handle = -1;

	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.IntPrio = 0;
	pDev->DevIntrf.EvtCB = NULL;
	pDev->DevIntrf.MaxRetry = 0;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_UNKOWN;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = false;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;

	pDev->DevIntrf.Disable = SemihostIntrfDisable;
	pDev->DevIntrf.Enable = SemihostIntrfEnable;
	pDev->DevIntrf.GetRate = SemihostIntrfGetRate;
	pDev->DevIntrf.SetRate = SemihostIntrfSetRate;
	pDev->DevIntrf.StartRx = SemihostIntrfStartRx;
	pDev->DevIntrf.RxData = SemihostIntrfRxData;
	pDev->DevIntrf.StopRx = SemihostIntrfStopRx;
	pDev->DevIntrf.StartTx = SemihostIntrfStartTx;
	pDev->DevIntrf.TxData = SemihostIntrfTxData;
	pDev->DevIntrf.TxSrData = SemihostIntrfTxSrData;
	pDev->DevIntrf.StopTx = SemihostIntrfStopTx;
	pDev->DevIntrf.Reset = SemihostIntrfReset;
	pDev->DevIntrf.PowerOff = SemihostIntrfPowerOff;
	pDev->DevIntrf.GetHandle = SemihostIntrfGetHandle;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

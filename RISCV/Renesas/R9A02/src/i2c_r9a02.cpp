/**-------------------------------------------------------------------------
@file	i2c_r9a02.cpp

@brief	R9A02G021 I2C master on the I2C bus interface (IICA)

R9A02G021 User's Manual R01UH1036EJ0110, chapter 24.

	DevNo 0 : IICA0, SCLA0 and SDAA0 (P302, P303 or P102, P103)
	DevNo 1 : IICA1, SCLA1 and SDAA1 (P011, P010, 48-pin package)

Master, polling, 7-bit addresses, standard mode up to 100 kHz, fast mode up
to 400 kHz, fast mode plus up to 1 MHz. Single master: communication
reservation is off and a start may be issued without a prior stop (IICF
IICRSV = 1, STCEN = 1). No slave mode, 10-bit address, SMBus, interrupt or
DMA transfer, these fail I2CInit.

The IICA has no end of byte status flag. Its event (IICn_ENDI) is routed to
an IELSR without a CPU interrupt and the IR flag is polled (24.4, 12.2.6).
WTIM = 1: the event and the clock stretch come at the falling edge of the 9th
clock, after the ACK, for the address and for every data byte (table 24.2).

Transfer clock = fMCK / (IICWL + IICWH + fMCK x (tR + tF)), fMCK = PCLKB
(PCLKB up to 20 MHz) or PCLKB / 2 (above 20 MHz) (24.3.2). PCLKB must be 32
MHz or lower with the IICA (notes of table 8.2), I2CInit fails above it: lower
the core clock or PCLKB (SystemPeriphClockSet) first, it must also be 1 MHz
or more. IICWL and IICWH are computed from the manual formulas with tR + tF
= 0, the bus clock then does not exceed the rate set whatever the rise time,
except below the lowest rate the 8-bit widths can make at that fMCK, which
is then used. SetRate returns this nominal rate.

The IICA is enabled only with SCL and SDA high (24.2.3 IICE): a bus held
low is first cleared by clocking SCL as a GPIO (figure 24.16 note 2), the
IICA stays off and the transfers fail while it does not clear. In master
reception the last byte is not acknowledged before a stop or a restart
(24.2.3 SPT and STT): when none was read, one byte is read and dropped.

Pins: PinOp R9A02_PSEL_IICA (IOPINOP_FUNC6), type IOPINTYPE_OPENDRAIN, external
pull-ups. The pins are GPIO inputs whenever IICE = 0, the IICA drives them low
in peripheral mode while it is off (24.2.9).

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

#include "r9a02g021.h"
#include "coredev/i2c.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"

#define R9A02_I2C_FM_MAX			400000UL
#define R9A02_I2C_FMP_MAX			1000000UL
#define R9A02_I2C_FM_PCLKB_MIN		3500000UL	//!< 24.2, table 37.38
#define R9A02_I2C_FMP_PCLKB_MIN		10000000UL	//!< 24.2, table 37.39
#define R9A02_I2C_PCLKB_MIN			1000000UL	//!< IICCTLn1.PRS, 24.2
#define R9A02_I2C_WIDTH_MAX			255U		//!< IICWL and IICWH are 8-bit
#define R9A02_I2C_BUS_RECOVER_CLK	9			//!< SCL pulses to free SDA

#pragma pack(push, 4)
typedef struct __R9a02_I2c_Dev {
	R9A02_IICA_Type *pReg;
	uint32_t MstpMask;				// MSTPCRB bit
	uint8_t EvtEnd;					// IICn_ENDI event
	I2CDev_t *pI2cDev;				// Generic I2C device data
	int IntNo;						// Polled IELSR route, -1 when none
	uint32_t Loops;					// Poll loops allowed for one byte
	uint8_t Ctl1;					// IICCTL1: PRS, DFC, SMC
	uint8_t Wl;						// IICWL
	uint8_t Wh;						// IICWH
	bool bEnabled;
	bool bOpen;						// Transaction started
	bool bActive;					// Master on the bus, SCL stretched after a byte
	bool bRx;						// Receive direction
	bool bNacked;					// Last byte received and not acknowledged
	bool bError;
	bool bLost;						// No event in time, the IICA is reset at the stop
} R9a02I2cDev_t;
#pragma pack(pop)

static R9a02I2cDev_t s_R9a02I2cDev[] = {
	{ R9A02_IICA0, R9A02_MSTPCRB_IICA0, R9A02_EVT_IIC0_ENDI, NULL, -1, 0, 0, 0, 0,
	  false, false, false, false, false, false, false },
	{ R9A02_IICA1, R9A02_MSTPCRB_IICA1, R9A02_EVT_IIC1_ENDI, NULL, -1, 0, 0, 0, 0,
	  false, false, false, false, false, false, false },
};

static const int s_NbR9a02I2cDev = sizeof(s_R9a02I2cDev) / sizeof(s_R9a02I2cDev[0]);

static R9a02I2cDev_t *R9a02I2cHandle(DevIntrf_t * const pIntrf)
{
	R9a02I2cDev_t *dev = pIntrf ? (R9a02I2cDev_t *)pIntrf->pDevData : NULL;

	return dev && dev->pI2cDev && &dev->pI2cDev->DevIntrf == pIntrf ? dev : NULL;
}

// Mode, fMCK and widths of a transfer clock. Above the fast mode plus limit,
// or above the limit of the mode PCLKB allows, the highest rate is used.
static bool R9a02I2cRate(uint32_t Rate, uint8_t *pCtl1, uint8_t *pWl, uint8_t *pWh, uint32_t *pActual)
{
	uint32_t pclk = SystemPeriphClockGet(0);

	if (Rate == 0U || pclk < R9A02_I2C_PCLKB_MIN || pclk > R9A02_IICA_PCLKB_MAX)
	{
		return false;
	}

	uint32_t prs = pclk > R9A02_IICA_FMCK_MAX ? 1U : 0U;
	uint64_t fmck = pclk >> prs;
	uint32_t lowpm;					// Low width per mille of the period (24.3.2)
	uint32_t highpm;
	uint32_t lowns;					// Minimum widths (table 37.37 to 37.39)
	uint32_t highns;
	uint8_t ctl1 = prs ? R9A02_IICCTL1_PRS : 0U;

	if (Rate > R9A02_I2C_FMP_MAX)
	{
		Rate = R9A02_I2C_FMP_MAX;
	}
	if (Rate > R9A02_I2C_FM_MAX && pclk < R9A02_I2C_FMP_PCLKB_MIN)
	{
		Rate = R9A02_I2C_FM_MAX;
	}
	if (Rate > I2C_SCL_STD_MODE_MAX_SPEED * 1000U && pclk < R9A02_I2C_FM_PCLKB_MIN)
	{
		Rate = I2C_SCL_STD_MODE_MAX_SPEED * 1000U;
	}

	if (Rate <= I2C_SCL_STD_MODE_MAX_SPEED * 1000U)
	{
		lowpm = 470;
		highpm = 530;
		lowns = I2C_SCL_TLOW_STD_MODE_MIN;
		highns = I2C_SCL_THIGH_STD_MODE_MIN;
	}
	else if (Rate <= R9A02_I2C_FM_MAX)
	{
		lowpm = 520;
		highpm = 480;
		lowns = I2C_SCL_TLOW_FAST_MODE_MIN;
		highns = I2C_SCL_THIGH_FAST_MODE_MIN;
		ctl1 |= R9A02_IICCTL1_SMC | R9A02_IICCTL1_DFC;
	}
	else
	{
		lowpm = 500;
		highpm = 500;
		lowns = I2C_SCL_TLOW_FAST_MODE_PLUS_MIN;
		highns = I2C_SCL_THIGH_FAST_MODE_PLUS_MIN;
		ctl1 |= R9A02_IICCTL1_SMC | R9A02_IICCTL1_DFC;
	}

	// Rounded up (24.3.2), at least the minimum widths
	uint64_t wl = (fmck * lowpm + 1000ULL * Rate - 1U) / (1000ULL * Rate);
	uint64_t wh = (fmck * highpm + 1000ULL * Rate - 1U) / (1000ULL * Rate);
	uint64_t wlmin = (fmck * lowns + 999999999ULL) / 1000000000ULL;
	uint64_t whmin = (fmck * highns + 999999999ULL) / 1000000000ULL;

	if (wl < wlmin)
	{
		wl = wlmin;
	}
	if (wh < whmin)
	{
		wh = whmin;
	}
	if (wl > R9A02_I2C_WIDTH_MAX)
	{
		wl = R9A02_I2C_WIDTH_MAX;
	}
	if (wh > R9A02_I2C_WIDTH_MAX)
	{
		wh = R9A02_I2C_WIDTH_MAX;
	}
	if (wl == 0U || wh == 0U || wlmin > R9A02_I2C_WIDTH_MAX || whmin > R9A02_I2C_WIDTH_MAX)
	{
		return false;
	}

	*pCtl1 = ctl1;
	*pWl = (uint8_t)wl;
	*pWh = (uint8_t)wh;
	*pActual = (uint32_t)(fmck / (wl + wh));

	return *pActual != 0U;
}

// SDA and SCL as GPIO inputs, released to the pull-ups
static void R9a02I2cPinsGpio(R9a02I2cDev_t * const pDev)
{
	const IOPinCfg_t *pins = pDev->pI2cDev->Cfg.pIOPinMap;

	for (int i = I2C_SDA_IOPIN_IDX; i <= I2C_SCL_IOPIN_IDX; i++)
	{
		IOPinConfig(pins[i].PortNo, pins[i].PinNo, IOPINOP_GPIO, IOPINDIR_INPUT, pins[i].Res, IOPINTYPE_NORMAL);
	}
}

// SCL as open drain output, waiting for a slave that stretches it
static bool R9a02I2cSclHigh(R9a02I2cDev_t * const pDev, const IOPinCfg_t *pScl)
{
	uint32_t n = pDev->Loops;

	while (!IOPinRead(pScl->PortNo, pScl->PinNo))
	{
		if (--n == 0U)
		{
			return false;
		}
	}

	return true;
}

static bool R9a02I2cBusIdle(R9a02I2cDev_t * const pDev)
{
	const IOPinCfg_t *pins = pDev->pI2cDev->Cfg.pIOPinMap;

	return IOPinRead(pins[I2C_SDA_IOPIN_IDX].PortNo, pins[I2C_SDA_IOPIN_IDX].PinNo) &&
		   IOPinRead(pins[I2C_SCL_IOPIN_IDX].PortNo, pins[I2C_SCL_IOPIN_IDX].PinNo);
}

// Bus recovery (figure 24.16 note 2), the IICA off: SCL is clocked as a GPIO
// until the slave releases SDA, then a stop condition is made. The pins are
// left as GPIO inputs. Returns true when both lines are high.
static bool R9a02I2cBusRecover(R9a02I2cDev_t * const pDev)
{
	const IOPinCfg_t *sda = &pDev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX];
	const IOPinCfg_t *scl = &pDev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX];
	uint32_t half = 5U;			// us, 100 kHz bus clock

	R9a02I2cPinsGpio(pDev);
	pDev->pReg->IICCTL0 = 0U;

	IOPinSet(sda->PortNo, sda->PinNo);
	IOPinSet(scl->PortNo, scl->PinNo);
	IOPinConfig(sda->PortNo, sda->PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT, sda->Res, IOPINTYPE_OPENDRAIN);
	IOPinConfig(scl->PortNo, scl->PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT, scl->Res, IOPINTYPE_OPENDRAIN);

	bool high = R9a02I2cSclHigh(pDev, scl);

	for (int i = 0; high && i < R9A02_I2C_BUS_RECOVER_CLK && !IOPinRead(sda->PortNo, sda->PinNo); i++)
	{
		IOPinClear(scl->PortNo, scl->PinNo);
		R9a02DelayUs(half);
		IOPinSet(scl->PortNo, scl->PinNo);
		high = R9a02I2cSclHigh(pDev, scl);
		R9a02DelayUs(half);
	}

	if (high)
	{
		IOPinClear(scl->PortNo, scl->PinNo);
		IOPinClear(sda->PortNo, sda->PinNo);
		R9a02DelayUs(half);
		IOPinSet(scl->PortNo, scl->PinNo);
		if (R9a02I2cSclHigh(pDev, scl))
		{
			R9a02DelayUs(half);
			IOPinSet(sda->PortNo, sda->PinNo);
			R9a02DelayUs(half);
		}
	}

	IOPinSet(sda->PortNo, sda->PinNo);
	IOPinSet(scl->PortNo, scl->PinNo);
	R9a02I2cPinsGpio(pDev);

	return R9a02I2cBusIdle(pDev);
}

// IICA stopped, set up, then started with the pins given back when enabled
// (figure 24.16). IICE is set only with SCL and SDA high (24.2.3), a bus
// held low is cleared first, with the interrupts on. Returns false when the
// IICA stays off.
static bool R9a02I2cConfigure(R9a02I2cDev_t * const pDev)
{
	R9A02_IICA_Type *reg = pDev->pReg;
	const IOPinCfg_t *pins = pDev->pI2cDev->Cfg.pIOPinMap;
	bool ok = true;

	R9a02I2cPinsGpio(pDev);
	reg->IICCTL0 = 0U;
	reg->IICWL = pDev->Wl;
	reg->IICWH = pDev->Wh;
	reg->SVA = 0U;
	reg->IICF = R9A02_IICF_IICRSV | R9A02_IICF_STCEN;
	reg->IICCTL1 = pDev->Ctl1;
	reg->IICCTL0 = R9A02_IICCTL0_WTIM;

	if (pDev->bEnabled)
	{
		ok = R9a02I2cBusIdle(pDev) || R9a02I2cBusRecover(pDev);
	}

	uintptr_t state = DisableInterrupt();

	if (pDev->bEnabled && ok)
	{
		reg->IICCTL0 = R9A02_IICCTL0_IICE | R9A02_IICCTL0_WTIM;
		IOPinCfg(pins, 2);
	}

	if (pDev->IntNo >= 0)
	{
		R9a02EventFlagClear(pDev->IntNo);
	}
	pDev->bActive = false;
	pDev->bNacked = false;
	pDev->bLost = false;

	EnableInterrupt(state);

	return ok;
}

static bool R9a02I2cWaitEvent(R9a02I2cDev_t * const pDev)
{
	uint32_t n = pDev->Loops;

	while (!R9a02EventFlag(pDev->IntNo))
	{
		if (--n == 0U)
		{
			pDev->bError = true;
			pDev->bLost = true;

			return false;
		}
	}

	return true;
}

// Status after an event. Arbitration lost: the IICA left the bus, it is
// told to leave the communication.
static uint8_t R9a02I2cStatus(R9a02I2cDev_t * const pDev)
{
	R9A02_IICA_Type *reg = pDev->pReg;
	uint8_t st = reg->IICS;

	if ((st & R9A02_IICS_MSTS) == 0U)
	{
		reg->IICCTL0 = (uint8_t)(reg->IICCTL0 | R9A02_IICCTL0_LREL);
		pDev->bActive = false;
		pDev->bError = true;
	}

	return st;
}

// Master reception: the slave sends until a byte is not acknowledged, a stop
// or a restart is made only after it (24.2.3). When none was read, one byte
// is read without ACK and dropped.
static void R9a02I2cRxEnd(R9a02I2cDev_t * const pDev)
{
	R9A02_IICA_Type *reg = pDev->pReg;

	if (!pDev->bActive || !pDev->bRx || pDev->bNacked || pDev->bLost)
	{
		return;
	}

	reg->IICCTL0 = R9A02_IICCTL0_IICE | R9A02_IICCTL0_WTIM;
	R9a02EventFlagClear(pDev->IntNo);
	reg->IICCTL0 = R9A02_IICCTL0_IICE | R9A02_IICCTL0_WTIM | R9A02_IICCTL0_WREL;

	if (R9a02I2cWaitEvent(pDev))
	{
		(void)R9a02I2cStatus(pDev);
		(void)reg->IICA;
		pDev->bNacked = true;
	}
}

// Stop condition from the clock stretch after the 9th clock, then wait for
// the bus to be free. A transfer that did not end is cleared by a new set up.
static void R9a02I2cFinish(R9a02I2cDev_t * const pDev)
{
	R9A02_IICA_Type *reg = pDev->pReg;

	R9a02I2cRxEnd(pDev);

	if (pDev->bActive && !pDev->bLost)
	{
		uint32_t n = pDev->Loops;

		reg->IICCTL0 = (uint8_t)((reg->IICCTL0 & ~R9A02_IICCTL0_ACKE) | R9A02_IICCTL0_SPT);
		while ((reg->IICF & R9A02_IICF_IICBSY) && --n > 0U)
		{
		}
		if (n == 0U)
		{
			pDev->bLost = true;
		}
	}

	if (pDev->bLost)
	{
		R9a02I2cConfigure(pDev);
	}

	pDev->bActive = false;
	pDev->bNacked = false;
	if (pDev->IntNo >= 0)
	{
		R9a02EventFlagClear(pDev->IntNo);
	}
}

static void R9a02I2cStop(DevIntrf_t * const pIntrf)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	if (dev == NULL || !dev->bOpen)
	{
		return;
	}

	R9a02I2cFinish(dev);
	dev->bOpen = false;
	dev->bRx = false;
	dev->bError = false;
	pIntrf->bTxReady = true;
}

// Start, or repeated start in a transaction, and the address byte. Its event
// comes at the 9th clock whatever WTIM (24.4).
static bool R9a02I2cStart(DevIntrf_t * const pIntrf, uint32_t Addr, bool bRx)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	if (dev == NULL || !dev->bEnabled || Addr > 0x7FU)
	{
		if (dev && dev->bOpen)
		{
			dev->bError = true;
		}

		return false;
	}

	R9A02_IICA_Type *reg = dev->pReg;
	bool wasopen = dev->bOpen;

	if (wasopen)
	{
		// A restart after a reception follows the NACK of its last byte
		R9a02I2cRxEnd(dev);
		if (dev->bError || !dev->bActive)
		{
			return false;
		}
	}

	// IICA left off by a bus that did not clear, tried again
	if ((reg->IICCTL0 & R9A02_IICCTL0_IICE) == 0U && !R9a02I2cConfigure(dev))
	{
		return false;
	}

	if (!wasopen)
	{
		uint32_t n = dev->Loops;

		while ((reg->IICF & R9A02_IICF_IICBSY) && --n > 0U)
		{
		}
		if (n == 0U)
		{
			return false;
		}
		dev->bError = false;
	}

	dev->bOpen = true;
	dev->bRx = bRx;
	dev->bNacked = false;

	R9a02EventFlagClear(dev->IntNo);
	reg->IICCTL0 = (uint8_t)((reg->IICCTL0 & ~R9A02_IICCTL0_ACKE) | R9A02_IICCTL0_STT);

	// The master may write the shift register once after STT (24.2)
	reg->IICA = (uint8_t)((Addr << 1) | (bRx ? 1U : 0U));

	bool ok = R9a02I2cWaitEvent(dev);

	if (ok)
	{
		dev->bActive = true;

		uint8_t st = R9a02I2cStatus(dev);

		ok = !dev->bError && (st & R9A02_IICS_ACKD);
	}

	if (!ok)
	{
		dev->bError = true;
		if (!wasopen)
		{
			// Not seen by StopRx or StopTx when the start fails, the bus is
			// released here
			R9a02I2cFinish(dev);
			dev->bOpen = false;
			dev->bRx = false;
			dev->bError = false;
		}
	}

	return ok;
}

static bool R9a02I2cStartTx(DevIntrf_t * const pIntrf, uint32_t Addr)
{
	return R9a02I2cStart(pIntrf, Addr, false);
}

static bool R9a02I2cStartRx(DevIntrf_t * const pIntrf, uint32_t Addr)
{
	return R9a02I2cStart(pIntrf, Addr, true);
}

// Writing the shift register releases the stretch and sends the byte. A byte
// not acknowledged ends the transfer, it is not counted.
static int R9a02I2cTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	if (dev == NULL || !dev->bOpen || !dev->bActive || dev->bRx || dev->bError || pData == NULL || Len <= 0)
	{
		return 0;
	}

	R9A02_IICA_Type *reg = dev->pReg;
	int cnt = 0;

	pIntrf->bTxReady = false;

	while (cnt < Len)
	{
		R9a02EventFlagClear(dev->IntNo);
		reg->IICA = pData[cnt];

		if (!R9a02I2cWaitEvent(dev))
		{
			break;
		}

		uint8_t st = R9a02I2cStatus(dev);

		if (dev->bError || (st & R9A02_IICS_ACKD) == 0U)
		{
			dev->bError = true;
			break;
		}
		cnt++;
	}

	pIntrf->bTxReady = true;

	return cnt;
}

// ACKE is set for every byte but the last one, which is not acknowledged.
// WREL releases the stretch, the event at the 9th clock follows the ACK or
// NACK and the byte is read in the stretch (figure 24.16).
static int R9a02I2cRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int Len)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	if (dev == NULL || !dev->bOpen || !dev->bActive || !dev->bRx || dev->bError || dev->bNacked ||
		pBuff == NULL || Len <= 0)
	{
		return 0;
	}

	R9A02_IICA_Type *reg = dev->pReg;
	int cnt = 0;

	while (cnt < Len)
	{
		uint8_t ctl0 = R9A02_IICCTL0_IICE | R9A02_IICCTL0_WTIM;

		if (cnt + 1 < Len)
		{
			ctl0 |= R9A02_IICCTL0_ACKE;
		}

		reg->IICCTL0 = ctl0;
		R9a02EventFlagClear(dev->IntNo);
		reg->IICCTL0 = (uint8_t)(ctl0 | R9A02_IICCTL0_WREL);

		if (!R9a02I2cWaitEvent(dev))
		{
			break;
		}

		(void)R9a02I2cStatus(dev);
		if (dev->bError)
		{
			break;
		}

		pBuff[cnt++] = reg->IICA;
	}

	dev->bNacked = cnt == Len;

	return cnt;
}

static uint32_t R9a02I2cGetRate(DevIntrf_t * const pIntrf)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	return dev ? dev->pI2cDev->Cfg.Rate : 0;
}

static void R9a02I2cSetLoops(R9a02I2cDev_t * const pDev)
{
	uint32_t rate = pDev->pI2cDev->Cfg.Rate ? pDev->pI2cDev->Cfg.Rate : 1U;

	// A byte with its ACK, slave clock stretching allowed up to about 50 ms
	pDev->Loops = SystemCoreClock / 20U + (SystemCoreClock / rate + 1U) * 20U;
}

static uint32_t R9a02I2cSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);
	uint8_t ctl1;
	uint8_t wl;
	uint8_t wh;
	uint32_t actual;

	if (dev == NULL || dev->bOpen || !R9a02I2cRate(Rate, &ctl1, &wl, &wh, &actual))
	{
		return 0;
	}

	dev->Ctl1 = ctl1;
	dev->Wl = wl;
	dev->Wh = wh;
	dev->pI2cDev->Cfg.Rate = actual;
	R9a02I2cSetLoops(dev);
	R9a02I2cConfigure(dev);

	return actual;
}

static void R9a02I2cDisable(DevIntrf_t * const pIntrf)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	R9a02I2cStop(pIntrf);

	uintptr_t state = DisableInterrupt();

	R9a02I2cPinsGpio(dev);
	dev->pReg->IICCTL0 = 0U;
	dev->bEnabled = false;

	EnableInterrupt(state);
}

static void R9a02I2cEnable(DevIntrf_t * const pIntrf)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	dev->bEnabled = true;
	R9a02I2cConfigure(dev);
}

// Bus recovery (figure 24.16 note 2), then a new set up
static void R9a02I2cReset(DevIntrf_t * const pIntrf)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	R9a02I2cStop(pIntrf);
	R9a02I2cBusRecover(dev);
	R9a02I2cConfigure(dev);
}

// IICA off, route removed, pins disconnected, module stopped. A new I2CInit
// is needed to use it again.
static void R9a02I2cPowerOff(DevIntrf_t * const pIntrf)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	R9a02I2cDisable(pIntrf);

	uintptr_t state = DisableInterrupt();

	R9a02UnregisterIntHandler(dev->IntNo);
	dev->IntNo = -1;
	IOPinDis(dev->pI2cDev->Cfg.pIOPinMap, 2);
	R9a02ModuleStop(R9A02_MSTPCR_B, dev->MstpMask);
	dev->pI2cDev = NULL;
	pIntrf->pDevData = NULL;
	pIntrf->EnCnt = 0;

	EnableInterrupt(state);
}

static void *R9a02I2cGetHandle(DevIntrf_t * const pIntrf)
{
	R9a02I2cDev_t *dev = R9a02I2cHandle(pIntrf);

	return dev ? dev->pI2cDev : NULL;
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo < 0 || pCfg->DevNo >= s_NbR9a02I2cDev ||
		pCfg->Type != I2CTYPE_STANDARD || pCfg->Mode != I2CMODE_MASTER ||
		pCfg->AddrType != I2CADDR_TYPE_NORMAL || pCfg->bIntEn || pCfg->bDmaEn ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins != 2 || pCfg->MaxRetry < 0)
	{
		return false;
	}

	for (int i = 0; i < 2; i++)
	{
		const IOPinCfg_t *pin = &pCfg->pIOPinMap[i];

		if (!R9a02PinValid(pin->PortNo, pin->PinNo) || pin->PinOp != R9A02_PSEL_IICA ||
			pin->Type != IOPINTYPE_OPENDRAIN || pin->Res == IOPINRES_PULLDOWN)
		{
			return false;
		}
	}

	if (pCfg->pIOPinMap[0].PortNo == pCfg->pIOPinMap[1].PortNo &&
		pCfg->pIOPinMap[0].PinNo == pCfg->pIOPinMap[1].PinNo)
	{
		return false;
	}

	uint8_t ctl1;
	uint8_t wl;
	uint8_t wh;
	uint32_t actual;

	if (!R9a02I2cRate(pCfg->Rate, &ctl1, &wl, &wh, &actual))
	{
		return false;
	}

	R9a02I2cDev_t *dev = &s_R9a02I2cDev[pCfg->DevNo];
	uintptr_t state = DisableInterrupt();

	// An instance in use by another I2C object is not taken over, an I2C
	// object is not used for two instances
	if ((dev->pI2cDev != NULL && dev->pI2cDev != pDev) || dev->bOpen ||
		s_R9a02I2cDev[1 - pCfg->DevNo].pI2cDev == pDev)
	{
		EnableInterrupt(state);

		return false;
	}

	if (dev->IntNo < 0)
	{
		dev->IntNo = R9a02RouteEvent(dev->EvtEnd);
	}
	if (dev->IntNo < 0)
	{
		EnableInterrupt(state);

		return false;
	}

	// Module clock first, register writes before it are ignored
	R9a02ModuleStart(R9A02_MSTPCR_B, dev->MstpMask);

	pDev->Cfg = *pCfg;
	pDev->Cfg.Rate = actual;
	dev->pI2cDev = pDev;
	dev->Ctl1 = ctl1;
	dev->Wl = wl;
	dev->Wh = wh;
	dev->bEnabled = true;
	dev->bOpen = false;
	dev->bRx = false;
	dev->bError = false;
	R9a02I2cSetLoops(dev);

	DevIntrf_t *intrf = &pDev->DevIntrf;

	intrf->pDevData = dev;
	intrf->Type = DEVINTRF_TYPE_I2C;
	intrf->bDma = false;
	intrf->bIntEn = false;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = R9a02I2cEnable;
	intrf->Disable = R9a02I2cDisable;
	intrf->PowerOff = R9a02I2cPowerOff;
	intrf->Reset = R9a02I2cReset;
	intrf->GetHandle = R9a02I2cGetHandle;
	intrf->GetRate = R9a02I2cGetRate;
	intrf->SetRate = R9a02I2cSetRate;
	intrf->StartRx = R9a02I2cStartRx;
	intrf->StartTx = R9a02I2cStartTx;
	intrf->StopRx = R9a02I2cStop;
	intrf->StopTx = R9a02I2cStop;
	intrf->RxData = R9a02I2cRxData;
	intrf->TxData = R9a02I2cTxData;
	intrf->TxSrData = R9a02I2cTxData;
	intrf->EnCnt = 1;
	atomic_flag_clear(&intrf->bBusy);

	R9a02I2cConfigure(dev);

	EnableInterrupt(state);

	return true;
}

/**-------------------------------------------------------------------------
@file	i2c_ra4m1.cpp

@brief	RA4M1 IIC0/IIC1 polling master implementation.

		DevNo 0 selects IIC0 and DevNo 1 selects IIC1. This first increment
		supports standard I2C, 7-bit master mode and polling transfers up to
		400 kHz. Interrupt, DMA/DTC, SMBus, 10-bit addressing and slave mode
		are rejected explicitly.

		The IIC clock is PCLKB. StartTx/StartRx retain the bus between an
		address-prefix write and the repeated-start read used by DeviceIntrfRead.
		Receive handling follows the RA IIC dummy-read, WAIT and ACKBT sequence.
		Reset performs an open-drain nine-clock bus recovery before restoring
		the configured peripheral pins.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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

#include "ra4m1xxx.h"

#include "coredev/i2c.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"

#define RA4M1_I2C_COUNT				2
#define RA4M1_I2C_MAX_RATE			400000U
#define RA4M1_I2C_ICBR_RESERVED		0xE0U
#define RA4M1_I2C_ICMR1_BASE			0x08U
#define RA4M1_I2C_ICMR2_BASE			0x04U
#define RA4M1_I2C_ICFER_MASTER		0x77U
#define RA4M1_I2C_ICCR1_RESET			0x40U
#define RA4M1_I2C_ICCR1_RESET_ENABLE	0xC0U
#define RA4M1_I2C_ICCR1_RUN			0x9FU

typedef enum
{
	RA4M1_I2C_WAIT_TDRE,
	RA4M1_I2C_WAIT_TEND,
	RA4M1_I2C_WAIT_RDRF,
	RA4M1_I2C_WAIT_STOP
} Ra4m1I2cWait_t;

typedef struct
{
	int DevNo;
	R_IIC0_Type *pReg;
	uint32_t MstpMask;
	I2CDev_t *pI2cDev;
	uint8_t Cks;
	uint8_t Brl;
	uint8_t Brh;
	bool bEnabled;
	bool bOpen;
	bool bActive;
	bool bRx;
	bool bError;
} Ra4m1I2cDev_t;

static Ra4m1I2cDev_t s_Ra4m1I2cDev[RA4M1_I2C_COUNT] = {
	{
		.DevNo = 0,
		.pReg = R_IIC0,
		.MstpMask = (1UL << 9),
	},
	{
		.DevNo = 1,
		.pReg = R_IIC1,
		.MstpMask = (1UL << 8),
	},
};

static Ra4m1I2cDev_t *Ra4m1I2cHandle(DevIntrf_t * const pIntrf)
{
	if (pIntrf == NULL)
	{
		return NULL;
	}

	Ra4m1I2cDev_t *dev = (Ra4m1I2cDev_t *)pIntrf->pDevData;

	if (dev == NULL || dev->pI2cDev == NULL)
	{
		return NULL;
	}

	if (&dev->pI2cDev->DevIntrf != pIntrf)
	{
		return NULL;
	}

	return dev;
}

static void Ra4m1I2cModuleStart(Ra4m1I2cDev_t * const pDev, bool bStart)
{
	uint16_t prcr = R_SYSTEM->PRCR & 0x0FU;

	R_SYSTEM->PRCR = 0xA500U | prcr | R_SYSTEM_PRCR_PRC1_Msk;

	if (bStart)
	{
		R_MSTP->MSTPCRB &= ~pDev->MstpMask;
	}
	else
	{
		R_MSTP->MSTPCRB |= pDev->MstpMask;
	}

	(void)R_MSTP->MSTPCRB;
	R_SYSTEM->PRCR = 0xA500U | prcr;
}

static uint32_t Ra4m1I2cTimeout(const Ra4m1I2cDev_t * const pDev)
{
	uint32_t core = SystemCoreClockGet();
	uint32_t rate = pDev->pI2cDev->Cfg.Rate;

	if (core == 0U || rate == 0U)
	{
		return 1000000U;
	}

	return core / 20U + (core / rate) * 16U + 256U;
}

static bool Ra4m1I2cError(const R_IIC0_Type * const pReg)
{
	return pReg->ICSR2_b.NACKF != 0U ||
		   pReg->ICSR2_b.AL != 0U ||
		   pReg->ICSR2_b.TMOF != 0U;
}

static bool Ra4m1I2cCondition(const R_IIC0_Type * const pReg, Ra4m1I2cWait_t Cond)
{
	switch (Cond)
	{
		case RA4M1_I2C_WAIT_TDRE:
			return pReg->ICSR2_b.TDRE != 0U;

		case RA4M1_I2C_WAIT_TEND:
			return pReg->ICSR2_b.TEND != 0U;

		case RA4M1_I2C_WAIT_RDRF:
			return pReg->ICSR2_b.RDRF != 0U;

		case RA4M1_I2C_WAIT_STOP:
			return pReg->ICSR2_b.STOP != 0U;

		default:
			return false;
	}
}

static bool Ra4m1I2cWait(Ra4m1I2cDev_t * const pDev, Ra4m1I2cWait_t Cond)
{
	uint32_t count = Ra4m1I2cTimeout(pDev);

	while (count > 0U)
	{
		if (Ra4m1I2cError(pDev->pReg))
		{
			break;
		}

		if (Ra4m1I2cCondition(pDev->pReg, Cond))
		{
			return true;
		}

		count--;
	}

	pDev->bError = true;
	return false;
}

static bool Ra4m1I2cRate(uint32_t Rate, uint8_t *pCks, uint8_t *pBrl,
	uint8_t *pBrh, uint32_t *pActual)
{
	if (pCks == NULL || pBrl == NULL || pBrh == NULL || pActual == NULL)
	{
		return false;
	}

	uint32_t clk = SystemPeriphClockGet(1);

	if (Rate == 0U || Rate > RA4M1_I2C_MAX_RATE || clk == 0U)
	{
		return false;
	}

	uint32_t lowNs = Rate <= I2C_SCL_STD_MODE_MAX_SPEED * 1000U ?
		I2C_SCL_TLOW_STD_MODE_MIN : I2C_SCL_TLOW_FAST_MODE_MIN;
	uint32_t highNs = Rate <= I2C_SCL_STD_MODE_MAX_SPEED * 1000U ?
		I2C_SCL_THIGH_STD_MODE_MIN : I2C_SCL_THIGH_FAST_MODE_MIN;
	uint32_t bestTicks = UINT32_MAX;
	uint8_t bestCks = 0U;
	uint8_t bestBrl = 0U;
	uint8_t bestBrh = 0U;

	/* SCLE is enabled and NFE is disabled below. With CKS=0 the IIC adds
	 * three IIC-phi clocks to each programmed high/low count; with a divided
	 * IIC-phi it adds two. Rise/fall time is intentionally treated as zero
	 * here. The programmed low/high minima and requested maximum rate are
	 * therefore conservative for a real open-drain bus, whose edge times
	 * only lengthen the SCL period.
	 */
	for (uint8_t cks = 0U; cks < 8U; cks++)
	{
		uint32_t offset = cks == 0U ? 3U : 2U;
		uint32_t divider = 1UL << cks;

		for (uint8_t brl = 0U; brl < 32U; brl++)
		{
			uint32_t lowTicks = ((uint32_t)brl + offset) * divider;

			if ((uint64_t)lowTicks * 1000000000ULL < (uint64_t)lowNs * clk)
			{
				continue;
			}

			for (uint8_t brh = 0U; brh < 32U; brh++)
			{
				uint32_t highTicks = ((uint32_t)brh + offset) * divider;
				uint32_t ticks = lowTicks + highTicks;

				if ((uint64_t)highTicks * 1000000000ULL < (uint64_t)highNs * clk)
				{
					continue;
				}

				if ((uint64_t)ticks * Rate < clk)
				{
					continue;
				}

				if (ticks < bestTicks)
				{
					bestTicks = ticks;
					bestCks = cks;
					bestBrl = brl;
					bestBrh = brh;
				}
			}
		}
	}

	if (bestTicks == UINT32_MAX)
	{
		return false;
	}

	*pCks = bestCks;
	*pBrl = bestBrl;
	*pBrh = bestBrh;
	*pActual = clk / bestTicks;

	return *pActual != 0U;
}

static void Ra4m1I2cAck(Ra4m1I2cDev_t * const pDev, bool bNack)
{
	pDev->pReg->ICMR3_b.ACKWP = 1U;
	pDev->pReg->ICMR3_b.ACKBT = bNack ? 1U : 0U;
	pDev->pReg->ICMR3_b.ACKWP = 0U;
}

static void Ra4m1I2cClearStatus(R_IIC0_Type * const pReg)
{
	pReg->ICSR2_b.TMOF = 0U;
	pReg->ICSR2_b.AL = 0U;
	pReg->ICSR2_b.NACKF = 0U;
	pReg->ICSR2_b.STOP = 0U;
	pReg->ICSR2_b.START = 0U;
}

static void Ra4m1I2cConfigure(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;

	/* Renesas IIC initialization sequence: external outputs inactive, assert
	 * IICRST, enable the IIC while reset is active, program the bus, then
	 * release reset with SCL/SDA software-output bits high.
	 */
	reg->ICCR1 = 0U;
	reg->ICCR1 = RA4M1_I2C_ICCR1_RESET;
	reg->ICCR1 = RA4M1_I2C_ICCR1_RESET_ENABLE;

	reg->SARL0 = 0U;
	reg->SARU0 = 0U;
	reg->SARL1 = 0U;
	reg->SARU1 = 0U;
	reg->SARL2 = 0U;
	reg->SARU2 = 0U;
	reg->ICSER = 0U;

	reg->ICBRL = (uint8_t)(RA4M1_I2C_ICBR_RESERVED | pDev->Brl);
	reg->ICBRH = (uint8_t)(RA4M1_I2C_ICBR_RESERVED | pDev->Brh);
	reg->ICMR1 = RA4M1_I2C_ICMR1_BASE;
	reg->ICMR1_b.CKS = pDev->Cks;
	reg->ICMR2 = RA4M1_I2C_ICMR2_BASE;
	reg->ICMR3 = 0U;

	/* Use the standard FSP master-function baseline, but disable the digital
	 * noise filter so the timing model above is exact and conservative.
	 */
	reg->ICFER = RA4M1_I2C_ICFER_MASTER;
	reg->ICFER_b.NFE = 0U;
	reg->ICIER = 0U;
	Ra4m1I2cClearStatus(reg);
	reg->ICCR1 = RA4M1_I2C_ICCR1_RUN;

	if (!pDev->bEnabled)
	{
		reg->ICCR1_b.ICE = 0U;
	}

	pDev->bActive = false;
}

static bool Ra4m1I2cFinish(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;
	bool stopped = !pDev->bActive;

	if (pDev->bActive && reg->ICSR2_b.AL == 0U &&
		reg->ICCR2_b.MST != 0U && reg->ICCR2_b.BBSY != 0U)
	{
		Ra4m1I2cAck(pDev, true);
		reg->ICCR2_b.SP = 1U;

		if (reg->ICSR2_b.RDRF != 0U)
		{
			(void)reg->ICDRR;
		}

		reg->ICMR3_b.WAIT = 0U;
		uint32_t count = Ra4m1I2cTimeout(pDev);

		while (count > 0U)
		{
			if (reg->ICSR2_b.AL != 0U)
			{
				break;
			}

			if (reg->ICSR2_b.STOP != 0U)
			{
				stopped = true;
				break;
			}

			count--;
		}
	}

	if (!stopped || pDev->bError)
	{
		Ra4m1I2cConfigure(pDev);
	}
	else
	{
		reg->ICSR2_b.STOP = 0U;
		reg->ICSR2_b.NACKF = 0U;
		reg->ICSR2_b.START = 0U;
		reg->ICMR3_b.WAIT = 0U;
		Ra4m1I2cAck(pDev, false);
		pDev->bActive = false;
	}

	return stopped;
}

static void Ra4m1I2cStop(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || !dev->bOpen)
	{
		return;
	}

	(void)Ra4m1I2cFinish(dev);
	dev->bOpen = false;
	dev->bRx = false;
	dev->bError = false;
	pIntrf->bTxReady = true;
}

static bool Ra4m1I2cStart(DevIntrf_t * const pIntrf, uint32_t Addr, bool bRx)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || !dev->bEnabled || Addr > 0x7FU)
	{
		if (dev != NULL && dev->bOpen)
		{
			dev->bError = true;
		}

		return false;
	}

	bool wasOpen = dev->bOpen;

	if (wasOpen && (dev->bError || dev->bRx || !dev->bActive))
	{
		return false;
	}

	R_IIC0_Type *reg = dev->pReg;

	if (!wasOpen)
	{
		uint32_t count = Ra4m1I2cTimeout(dev);

		while (reg->ICCR2_b.BBSY != 0U && count > 0U)
		{
			count--;
		}

		if (reg->ICCR2_b.BBSY != 0U)
		{
			return false;
		}

		dev->bError = false;
		Ra4m1I2cClearStatus(reg);
	}

	dev->bOpen = true;
	dev->bActive = true;
	dev->bRx = bRx;

	if (wasOpen)
	{
		reg->ICSR2_b.START = 0U;
		reg->ICCR2_b.RS = 1U;
	}
	else
	{
		reg->ICCR2_b.ST = 1U;
	}

	if (Ra4m1I2cWait(dev, RA4M1_I2C_WAIT_TDRE))
	{
		reg->ICDRT = (uint8_t)((Addr << 1U) | (bRx ? 1U : 0U));

		Ra4m1I2cWait_t cond = bRx ? RA4M1_I2C_WAIT_RDRF : RA4M1_I2C_WAIT_TEND;

		if (Ra4m1I2cWait(dev, cond))
		{
			return true;
		}
	}

	(void)Ra4m1I2cFinish(dev);

	if (!wasOpen)
	{
		dev->bOpen = false;
		dev->bRx = false;
		dev->bError = false;
	}

	return false;
}

static bool Ra4m1I2cStartTx(DevIntrf_t * const pIntrf, uint32_t Addr)
{
	return Ra4m1I2cStart(pIntrf, Addr, false);
}

static bool Ra4m1I2cStartRx(DevIntrf_t * const pIntrf, uint32_t Addr)
{
	return Ra4m1I2cStart(pIntrf, Addr, true);
}

static int Ra4m1I2cTx(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || !dev->bOpen || !dev->bActive || dev->bRx ||
		dev->bError || pData == NULL || Len <= 0)
	{
		return 0;
	}

	int count = 0;
	pIntrf->bTxReady = false;

	while (count < Len)
	{
		if (!Ra4m1I2cWait(dev, RA4M1_I2C_WAIT_TDRE))
		{
			break;
		}

		dev->pReg->ICDRT = pData[count];

		if (!Ra4m1I2cWait(dev, RA4M1_I2C_WAIT_TEND))
		{
			break;
		}

		count++;
	}

	if (dev->bError)
	{
		(void)Ra4m1I2cFinish(dev);
	}

	pIntrf->bTxReady = true;
	return count;
}

static int Ra4m1I2cRx(DevIntrf_t * const pIntrf, uint8_t *pBuff, int Len)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || !dev->bOpen || !dev->bActive || !dev->bRx ||
		dev->bError || pBuff == NULL || Len <= 0)
	{
		return 0;
	}

	R_IIC0_Type *reg = dev->pReg;

	if (Len <= 2)
	{
		reg->ICMR3_b.WAIT = 1U;
	}

	if (Len == 1)
	{
		Ra4m1I2cAck(dev, true);
	}

	/* Address-phase dummy read releases the first receive clock. */
	(void)reg->ICDRR;

	int count = 0;

	while (count < Len)
	{
		if (!Ra4m1I2cWait(dev, RA4M1_I2C_WAIT_RDRF))
		{
			break;
		}

		int remaining = Len - count;

		if (remaining == 3)
		{
			reg->ICMR3_b.WAIT = 1U;
		}

		if (remaining == 2)
		{
			Ra4m1I2cAck(dev, true);
		}

		if (remaining == 1)
		{
			reg->ICCR2_b.SP = 1U;
		}

		pBuff[count] = reg->ICDRR;
		count++;
	}

	if (count == Len)
	{
		reg->ICMR3_b.WAIT = 0U;

		if (Ra4m1I2cWait(dev, RA4M1_I2C_WAIT_STOP))
		{
			reg->ICSR2_b.STOP = 0U;
			reg->ICSR2_b.NACKF = 0U;
			reg->ICSR2_b.START = 0U;
			Ra4m1I2cAck(dev, false);
			dev->bActive = false;
		}
	}

	if (dev->bError)
	{
		(void)Ra4m1I2cFinish(dev);
	}

	return count;
}

static void Ra4m1I2cDisable(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	Ra4m1I2cStop(pIntrf);
	dev->pReg->ICCR1_b.ICE = 0U;
	dev->bEnabled = false;
}

static void Ra4m1I2cEnable(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	IOPinCfg(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);
	dev->bEnabled = true;
	Ra4m1I2cConfigure(dev);
}

static bool Ra4m1I2cSclHigh(Ra4m1I2cDev_t * const pDev,
	const IOPinCfg_t * const pScl)
{
	uint32_t count = Ra4m1I2cTimeout(pDev);

	while (count > 0U)
	{
		if (IOPinRead(pScl->PortNo, pScl->PinNo) != 0)
		{
			return true;
		}

		count--;
	}

	return false;
}

static void Ra4m1I2cBusDelay(void)
{
	uint32_t count = (SystemCoreClockGet() + 199999U) / 200000U;

	if (count == 0U)
	{
		count = 1U;
	}

	while (count > 0U)
	{
		__NOP();
		count--;
	}
}

static void Ra4m1I2cReset(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	Ra4m1I2cStop(pIntrf);
	bool enabled = dev->bEnabled;
	dev->pReg->ICCR1_b.ICE = 0U;

	const IOPinCfg_t *sda = &dev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX];
	const IOPinCfg_t *scl = &dev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX];

	IOPinSet(sda->PortNo, sda->PinNo);
	IOPinSet(scl->PortNo, scl->PinNo);

	IOPinConfig(sda->PortNo, sda->PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT,
		sda->Res, IOPINTYPE_OPENDRAIN);
	IOPinConfig(scl->PortNo, scl->PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT,
		scl->Res, IOPINTYPE_OPENDRAIN);

	bool high = Ra4m1I2cSclHigh(dev, scl);

	for (int i = 0; high && i < 9 && IOPinRead(sda->PortNo, sda->PinNo) == 0; i++)
	{
		IOPinClear(scl->PortNo, scl->PinNo);
		Ra4m1I2cBusDelay();
		IOPinSet(scl->PortNo, scl->PinNo);
		high = Ra4m1I2cSclHigh(dev, scl);
		Ra4m1I2cBusDelay();
	}

	if (high)
	{
		IOPinClear(scl->PortNo, scl->PinNo);
		IOPinClear(sda->PortNo, sda->PinNo);
		Ra4m1I2cBusDelay();
		IOPinSet(scl->PortNo, scl->PinNo);

		if (Ra4m1I2cSclHigh(dev, scl))
		{
			Ra4m1I2cBusDelay();
			IOPinSet(sda->PortNo, sda->PinNo);
			Ra4m1I2cBusDelay();
		}
	}

	IOPinSet(sda->PortNo, sda->PinNo);
	IOPinSet(scl->PortNo, scl->PinNo);
	IOPinCfg(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);

	dev->bEnabled = enabled;
	Ra4m1I2cConfigure(dev);
}

static void Ra4m1I2cPowerOff(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	Ra4m1I2cDisable(pIntrf);
	IOPinDis(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);

	uint32_t state = DisableInterrupt();
	Ra4m1I2cModuleStart(dev, false);
	dev->pI2cDev = NULL;
	pIntrf->pDevData = NULL;
	pIntrf->EnCnt = 0;
	EnableInterrupt(state);
}

static void *Ra4m1I2cGetHandle(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	return dev != NULL ? dev->pI2cDev : NULL;
}

static uint32_t Ra4m1I2cGetRate(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	return dev != NULL ? dev->pI2cDev->Cfg.Rate : 0U;
}

static uint32_t Ra4m1I2cSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || dev->bOpen)
	{
		return 0U;
	}

	uint8_t cks = 0U;
	uint8_t brl = 0U;
	uint8_t brh = 0U;
	uint32_t actual = 0U;

	if (!Ra4m1I2cRate(Rate, &cks, &brl, &brh, &actual))
	{
		return 0U;
	}

	dev->Cks = cks;
	dev->Brl = brl;
	dev->Brh = brh;
	dev->pI2cDev->Cfg.Rate = actual;
	Ra4m1I2cConfigure(dev);

	return actual;
}

static bool Ra4m1I2cPin(Ra4m1I2cDev_t * const pDev, const IOPinCfg_t * const pPin,
	bool bScl)
{
	if (!Ra4m1PinValid(pPin->PortNo, pPin->PinNo))
	{
		return false;
	}

	if (pPin->PinOp != IOPINOP_FUNC6 || pPin->Type != IOPINTYPE_OPENDRAIN)
	{
		return false;
	}

	if (pPin->Res != IOPINRES_NONE && pPin->Res != IOPINRES_PULLUP)
	{
		return false;
	}

	if (pDev->DevNo == 0)
	{
		if (bScl)
		{
			return (pPin->PortNo == 2 && pPin->PinNo == 4) ||
				   (pPin->PortNo == 4 && pPin->PinNo == 0) ||
				   (pPin->PortNo == 4 && pPin->PinNo == 8);
		}

		return (pPin->PortNo == 4 && pPin->PinNo == 1) ||
			   (pPin->PortNo == 4 && pPin->PinNo == 7);
	}

	if (bScl)
	{
		return (pPin->PortNo == 1 && pPin->PinNo == 0) ||
			   (pPin->PortNo == 2 && pPin->PinNo == 5);
	}

	return (pPin->PortNo == 1 && pPin->PinNo == 1) ||
		   (pPin->PortNo == 2 && pPin->PinNo == 6);
}

static bool Ra4m1I2cPinMap(Ra4m1I2cDev_t * const pDev, const I2CCfg_t * const pCfg)
{
	const IOPinCfg_t *sda = &pCfg->pIOPinMap[I2C_SDA_IOPIN_IDX];
	const IOPinCfg_t *scl = &pCfg->pIOPinMap[I2C_SCL_IOPIN_IDX];

	if (sda->PortNo == scl->PortNo && sda->PinNo == scl->PinNo)
	{
		return false;
	}

	return Ra4m1I2cPin(pDev, sda, false) &&
		   Ra4m1I2cPin(pDev, scl, true);
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= RA4M1_I2C_COUNT ||
		pCfg->Type != I2CTYPE_STANDARD ||
		pCfg->Mode != I2CMODE_MASTER ||
		pCfg->AddrType != I2CADDR_TYPE_NORMAL ||
		pCfg->bIntEn ||
		pCfg->bDmaEn ||
		pCfg->pIOPinMap == NULL ||
		pCfg->NbIOPins != 2 ||
		pCfg->MaxRetry < 0)
	{
		return false;
	}

	Ra4m1I2cDev_t *dev = &s_Ra4m1I2cDev[pCfg->DevNo];

	if (!Ra4m1I2cPinMap(dev, pCfg))
	{
		return false;
	}

	uint8_t cks = 0U;
	uint8_t brl = 0U;
	uint8_t brh = 0U;
	uint32_t actual = 0U;

	if (!Ra4m1I2cRate(pCfg->Rate, &cks, &brl, &brh, &actual))
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	if ((dev->pI2cDev != NULL && dev->pI2cDev != pDev) ||
		dev->bOpen ||
		s_Ra4m1I2cDev[1 - pCfg->DevNo].pI2cDev == pDev)
	{
		EnableInterrupt(state);
		return false;
	}

	dev->pI2cDev = pDev;
	dev->Cks = cks;
	dev->Brl = brl;
	dev->Brh = brh;
	dev->bEnabled = true;
	dev->bOpen = false;
	dev->bActive = false;
	dev->bRx = false;
	dev->bError = false;
	Ra4m1I2cModuleStart(dev, true);
	EnableInterrupt(state);

	pDev->Cfg = *pCfg;
	pDev->Cfg.Rate = actual;

	for (int i = 0; i < I2C_SLAVEMODE_MAX_ADDR; i++)
	{
		pDev->pRRData[i] = NULL;
		pDev->RRDataLen[i] = 0;
		pDev->pTRBuff[i] = NULL;
		pDev->TRBuffLen[i] = 0;
	}

	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);

	DevIntrf_t *intrf = &pDev->DevIntrf;
	intrf->pDevData = dev;
	intrf->Type = DEVINTRF_TYPE_I2C;
	intrf->EnCnt = 1;
	intrf->bDma = false;
	intrf->bIntEn = false;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = Ra4m1I2cEnable;
	intrf->Disable = Ra4m1I2cDisable;
	intrf->PowerOff = Ra4m1I2cPowerOff;
	intrf->Reset = Ra4m1I2cReset;
	intrf->GetHandle = Ra4m1I2cGetHandle;
	intrf->GetRate = Ra4m1I2cGetRate;
	intrf->SetRate = Ra4m1I2cSetRate;
	intrf->StartRx = Ra4m1I2cStartRx;
	intrf->StartTx = Ra4m1I2cStartTx;
	intrf->StopRx = Ra4m1I2cStop;
	intrf->StopTx = Ra4m1I2cStop;
	intrf->RxData = Ra4m1I2cRx;
	intrf->TxData = Ra4m1I2cTx;
	intrf->TxSrData = Ra4m1I2cTx;
	atomic_flag_clear(&intrf->bBusy);

	Ra4m1I2cConfigure(dev);

	return true;
}

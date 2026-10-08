/**-------------------------------------------------------------------------
@file	i2c_re01.cpp
@brief	RE01 1500KB RIIC0/1 polling, 7-bit I2C master implementation.

Register sequencing follows Renesas re-driver-package r_i2c_cmsis_api.c,
including the receive dummy read, protected ACKBT writes and final-byte STOP.
Pin functions and external pull-ups are supplied by the application's board.h.

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
#include "re01xxx.h"
#include "coredev/i2c.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"

typedef struct {
	IIC0_Type *pReg;
	I2CDev_t *pI2cDev;
	uint32_t StopMask;
	uint8_t Cks;
	uint8_t Brl;
	uint8_t Brh;
	bool bEnabled;
	bool bOpen;
	bool bActive;
	bool bRx;
	bool bError;
} Re01I2cDev_t;

static Re01I2cDev_t s_Re01I2cDev[] = {
	{IIC0, NULL, MSTP_MSTPCRB_MSTPB9_Msk, 0, 0, 0, false, false, false, false, false},
	{(IIC0_Type*)IIC1, NULL, MSTP_MSTPCRB_MSTPB8_Msk, 0, 0, 0, false, false, false, false, false},
};

#define RE01_I2C_ERRORS	(IIC0_ICSR2_NACKF_Msk | IIC0_ICSR2_AL_Msk | IIC0_ICSR2_TMOF_Msk)

static Re01I2cDev_t *Re01I2cHandle(DevIntrf_t *pIntrf)
{
	Re01I2cDev_t *dev = pIntrf ? (Re01I2cDev_t*)pIntrf->pDevData : NULL;
	return dev && dev->pI2cDev && &dev->pI2cDev->DevIntrf == pIntrf ? dev : NULL;
}

static uint32_t Re01I2cTimeout(Re01I2cDev_t *dev)
{
	// Finite allowance for clock stretching, plus a byte at the slowest rate.
	return SystemCoreClock / 20U +
		   (SystemCoreClock / dev->pI2cDev->Cfg.Rate) * 16U + 256U;
}

static bool Re01I2cWait(Re01I2cDev_t *dev, uint8_t Mask, uint8_t Value)
{
	uint32_t n = Re01I2cTimeout(dev);
	do {
		uint8_t status = dev->pReg->ICSR2;
		if (status & RE01_I2C_ERRORS)
			break;
		if ((status & Mask) == Value)
			return true;
	} while (n--);
	dev->bError = true;
	return false;
}

static bool Re01I2cRate(uint32_t Rate, uint8_t *pCks, uint8_t *pBrl,
					   uint8_t *pBrh, uint32_t *pActual)
{
	uint32_t clk = SystemPeriphClockGet(1); // PCLKB, independent of ICLK/PCLKA.
	if (Rate == 0 || Rate > 400000 || clk == 0 || clk > 32000000)
		return false;
	uint32_t lowNs = Rate <= 100000 ? I2C_SCL_TLOW_STD_MODE_MIN : I2C_SCL_TLOW_FAST_MODE_MIN;
	uint32_t highNs = Rate <= 100000 ? I2C_SCL_THIGH_STD_MODE_MIN : I2C_SCL_THIGH_FAST_MODE_MIN;
	uint32_t best = UINT32_MAX;
	// Two noise-filter stages: offsets are 5 (CKS=0) and 4 (CKS=1..7).
	// Do not subtract an assumed board rise/fall time. The nominal clock is
	// an upper bound; real edge times and stretching can only reduce it.
	for (unsigned cks = 0; cks < 8; cks++)
	{
		unsigned offset = cks == 0 ? 5 : 4;
		for (unsigned low = 0; low < 32; low++)
		{
			uint32_t lowTicks = (low + offset) << cks;
			if ((uint64_t)lowTicks * 1000000000ULL < (uint64_t)lowNs * clk)
				continue;
			for (unsigned high = 0; high < 32; high++)
			{
				uint32_t highTicks = (high + offset) << cks;
				uint32_t ticks = lowTicks + highTicks;
				if ((uint64_t)highTicks * 1000000000ULL < (uint64_t)highNs * clk ||
					(uint64_t)ticks * Rate < clk || ticks >= best)
					continue;
				best = ticks;
				*pCks = (uint8_t)(cks << IIC0_ICMR1_CKS_Pos);
				*pBrl = (uint8_t)(0xE0 | low);
				*pBrh = (uint8_t)(0xE0 | high);
			}
		}
	}
	if (best == UINT32_MAX)
		return false;
	*pActual = clk / best;
	return *pActual != 0;
}

static void Re01I2cConfigure(Re01I2cDev_t *dev)
{
	IIC0_Type *reg = dev->pReg;
	reg->ICCR1 &= ~IIC0_ICCR1_ICE_Msk;
	reg->ICCR1 |= IIC0_ICCR1_IICRST_Msk;
	reg->ICCR1 |= IIC0_ICCR1_ICE_Msk;
	reg->SARL0 = 0;
	reg->SARU0 = 0;
	reg->SARL1 = 0;
	reg->SARU1 = 0;
	reg->SARL2 = 0;
	reg->SARU2 = 0;
	reg->ICSER = 0;
	reg->ICMR1 = dev->Cks;
	reg->ICBRL = dev->Brl;
	reg->ICBRH = dev->Brh;
	reg->ICMR2 = 0x06;
	reg->ICMR3 = 1; // I2C, RDRFS=0, ACK, two noise-filter stages.
	reg->ICFER = IIC0_ICFER_SCLE_Msk | IIC0_ICFER_NFE_Msk | IIC0_ICFER_NACKE_Msk |
				 IIC0_ICFER_SALE_Msk | IIC0_ICFER_MALE_Msk;
	reg->ICIER = 0;
	reg->ICCR1 &= ~IIC0_ICCR1_IICRST_Msk;
	if (!dev->bEnabled)
		reg->ICCR1 &= ~IIC0_ICCR1_ICE_Msk;
	dev->bActive = false;
}

static void Re01I2cAck(Re01I2cDev_t *dev, bool bNack)
{
	// ACKBT must be unlocked in a separate write before changing it.
	dev->pReg->ICMR3 |= IIC0_ICMR3_ACKWP_Msk;
	uint8_t mode = dev->pReg->ICMR3 & ~(IIC0_ICMR3_ACKBT_Msk | IIC0_ICMR3_ACKWP_Msk);
	dev->pReg->ICMR3 = mode | (bNack ? IIC0_ICMR3_ACKBT_Msk : 0);
}

static bool Re01I2cFinish(Re01I2cDev_t *dev)
{
	IIC0_Type *reg = dev->pReg;
	bool stopped = !dev->bActive;
	if (dev->bActive)
	{
		// Arbitration loss belongs to the other master; never request STOP.
		if (!(reg->ICSR2 & IIC0_ICSR2_AL_Msk) &&
			(reg->ICCR2 & (IIC0_ICCR2_MST_Msk | IIC0_ICCR2_BBSY_Msk)) ==
			(IIC0_ICCR2_MST_Msk | IIC0_ICCR2_BBSY_Msk))
		{
			Re01I2cAck(dev, true);
			reg->ICCR2 |= IIC0_ICCR2_SP_Msk;
			if (reg->ICSR2 & IIC0_ICSR2_RDRF_Msk)
				(void)reg->ICDRR;
			reg->ICMR3 &= ~IIC0_ICMR3_WAIT_Msk;
			uint32_t n = Re01I2cTimeout(dev);
			do {
				uint8_t status = reg->ICSR2;
				if (status & IIC0_ICSR2_AL_Msk)
					break;
				// A NACK caused the abort; it must not prevent STOP cleanup.
				if (status & IIC0_ICSR2_STOP_Msk)
				{
					stopped = true;
					break;
				}
			} while (n--);
		}
	}
	if (!stopped || dev->bError)
		Re01I2cConfigure(dev);
	else
	{
		reg->ICSR2 &= ~(IIC0_ICSR2_STOP_Msk | IIC0_ICSR2_NACKF_Msk | IIC0_ICSR2_START_Msk);
		reg->ICMR3 &= ~IIC0_ICMR3_WAIT_Msk;
		Re01I2cAck(dev, false);
		dev->bActive = false;
	}
	return stopped;
}

static void Re01I2cStop(DevIntrf_t *pIntrf)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	if (!dev || !dev->bOpen)
		return;
	(void)Re01I2cFinish(dev);
	dev->bOpen = dev->bRx = dev->bError = false;
	pIntrf->bTxReady = true;
}

static bool Re01I2cStart(DevIntrf_t *pIntrf, uint32_t Addr, bool bRx)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	if (!dev || !dev->bEnabled || Addr > 0x7F)
	{
		if (dev && dev->bOpen)
			dev->bError = true;
		return false;
	}
	bool wasOpen = dev->bOpen;
	if (wasOpen && (dev->bError || dev->bRx || !dev->bActive))
		return false;
	IIC0_Type *reg = dev->pReg;
	if (!wasOpen)
	{
		// Do not reset or drive pins while another master holds the bus.
		uint32_t n = Re01I2cTimeout(dev);
		while (reg->ICCR2 & IIC0_ICCR2_BBSY_Msk)
			if (n-- == 0)
				return false;
		dev->bError = false;
		reg->ICSR2 &= ~(IIC0_ICSR2_START_Msk | IIC0_ICSR2_STOP_Msk | RE01_I2C_ERRORS);
	}
	dev->bOpen = dev->bActive = true;
	dev->bRx = bRx;
	reg->ICCR2 |= wasOpen ? IIC0_ICCR2_RS_Msk : IIC0_ICCR2_ST_Msk;
	if (Re01I2cWait(dev, IIC0_ICSR2_TDRE_Msk, IIC0_ICSR2_TDRE_Msk))
	{
		reg->ICDRT = (uint8_t)((Addr << 1) | (bRx ? 1 : 0));
		// A read address completes at RDRF. Defer the dummy read until
		// RxData provides its length, so a one-byte read can NACK correctly.
		if (Re01I2cWait(dev, bRx ? IIC0_ICSR2_RDRF_Msk : IIC0_ICSR2_TEND_Msk,
						bRx ? IIC0_ICSR2_RDRF_Msk : IIC0_ICSR2_TEND_Msk))
			return true;
	}
	(void)Re01I2cFinish(dev);
	// Keep a prefix failure sticky until Stop: DeviceIntrfRead calls the
	// raw StartRx hook without checking its result, then calls RxData.
	if (!wasOpen)
		dev->bOpen = dev->bRx = dev->bError = false;
	return false;
}

static bool Re01I2cStartTx(DevIntrf_t *pIntrf, uint32_t Addr)
{
	return Re01I2cStart(pIntrf, Addr, false);
}

static bool Re01I2cStartRx(DevIntrf_t *pIntrf, uint32_t Addr)
{
	return Re01I2cStart(pIntrf, Addr, true);
}

static int Re01I2cTx(DevIntrf_t *pIntrf, const uint8_t *pData, int Len)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	if (!dev || !dev->bOpen || !dev->bActive || dev->bRx || dev->bError ||
		!pData || Len <= 0)
		return 0;
	int count = 0;
	pIntrf->bTxReady = false;
	while (count < Len)
	{
		if (!Re01I2cWait(dev, IIC0_ICSR2_TDRE_Msk, IIC0_ICSR2_TDRE_Msk))
			break;
		dev->pReg->ICDRT = pData[count];
		if (!Re01I2cWait(dev, IIC0_ICSR2_TEND_Msk, IIC0_ICSR2_TEND_Msk))
			break;
		count++;
	}
	if (dev->bError)
		(void)Re01I2cFinish(dev);
	pIntrf->bTxReady = true;
	return count;
}

static int Re01I2cRx(DevIntrf_t *pIntrf, uint8_t *pBuff, int Len)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	if (!dev || !dev->bOpen || !dev->bActive || !dev->bRx || dev->bError ||
		!pBuff || Len <= 0)
		return 0;
	IIC0_Type *reg = dev->pReg;
	if (Len <= 2)
		reg->ICMR3 |= IIC0_ICMR3_WAIT_Msk;
	if (Len == 1)
		Re01I2cAck(dev, true);
	(void)reg->ICDRR; // Address-phase dummy read starts reception.
	int count = 0;
	while (count < Len)
	{
		if (!Re01I2cWait(dev, IIC0_ICSR2_RDRF_Msk, IIC0_ICSR2_RDRF_Msk))
			break;
		int remaining = Len - count;
		if (remaining == 3)
			reg->ICMR3 |= IIC0_ICMR3_WAIT_Msk;
		if (remaining == 2)
			Re01I2cAck(dev, true);
		if (remaining == 1)
			reg->ICCR2 |= IIC0_ICCR2_SP_Msk; // Before reading the final byte.
		pBuff[count++] = reg->ICDRR;
	}
	if (count == Len)
	{
		// STOP has already been requested, so do not request it again.
		reg->ICMR3 &= ~IIC0_ICMR3_WAIT_Msk;
		if (Re01I2cWait(dev, IIC0_ICSR2_STOP_Msk, IIC0_ICSR2_STOP_Msk))
		{
			reg->ICSR2 &= ~(IIC0_ICSR2_STOP_Msk | IIC0_ICSR2_NACKF_Msk | IIC0_ICSR2_START_Msk);
			Re01I2cAck(dev, false);
			dev->bActive = false;
		}
	}
	if (dev->bError)
		(void)Re01I2cFinish(dev);
	return count;
}

static void Re01I2cDisable(DevIntrf_t *pIntrf)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	if (!dev)
		return;
	Re01I2cStop(pIntrf);
	dev->pReg->ICCR1 &= ~IIC0_ICCR1_ICE_Msk;
	dev->bEnabled = false;
}

static void Re01I2cEnable(DevIntrf_t *pIntrf)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	if (!dev)
		return;
	IOPinCfg(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);
	dev->bEnabled = true;
	Re01I2cConfigure(dev);
}

static bool Re01I2cSclHigh(Re01I2cDev_t *dev, const IOPinCfg_t *pScl)
{
	uint32_t n = Re01I2cTimeout(dev);
	do {
		if (IOPinRead(pScl->PortNo, pScl->PinNo))
			return true;
	} while (n--);
	return false;
}

static void Re01I2cBusDelay(void)
{
	// At least 5 us, even when the shared usDelay factor rounds to zero
	// at a low ICLK. Loop overhead makes this deliberately conservative.
	uint32_t n = (SystemCoreClock + 199999U) / 200000U;
	do {
		__NOP();
	} while (n--);
}

static void Re01I2cReset(DevIntrf_t *pIntrf)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	if (!dev)
		return;
	Re01I2cStop(pIntrf);
	bool enabled = dev->bEnabled;
	dev->pReg->ICCR1 &= ~IIC0_ICCR1_ICE_Msk;
	const IOPinCfg_t *sda = &dev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX];
	const IOPinCfg_t *scl = &dev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX];
	IOPinSet(sda->PortNo, sda->PinNo);
	IOPinSet(scl->PortNo, scl->PinNo);
	IOPinConfig(sda->PortNo, sda->PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT,
				sda->Res, IOPINTYPE_OPENDRAIN);
	IOPinConfig(scl->PortNo, scl->PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT,
				scl->Res, IOPINTYPE_OPENDRAIN);
	bool high = Re01I2cSclHigh(dev, scl);
	for (int i = 0; high && i < 9 && !IOPinRead(sda->PortNo, sda->PinNo); i++)
	{
		IOPinClear(scl->PortNo, scl->PinNo);
		Re01I2cBusDelay();
		IOPinSet(scl->PortNo, scl->PinNo);
		high = Re01I2cSclHigh(dev, scl);
		Re01I2cBusDelay();
	}
	if (high)
	{
		// Make STOP with open-drain outputs and honor a stretched SCL.
		IOPinClear(scl->PortNo, scl->PinNo);
		IOPinClear(sda->PortNo, sda->PinNo);
		Re01I2cBusDelay();
		IOPinSet(scl->PortNo, scl->PinNo);
		if (Re01I2cSclHigh(dev, scl))
		{
			Re01I2cBusDelay();
			IOPinSet(sda->PortNo, sda->PinNo);
			Re01I2cBusDelay();
		}
	}
	IOPinSet(sda->PortNo, sda->PinNo);
	IOPinSet(scl->PortNo, scl->PinNo);
	IOPinCfg(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);
	dev->bEnabled = enabled;
	Re01I2cConfigure(dev);
}

static void Re01I2cPowerOff(DevIntrf_t *pIntrf)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	if (!dev)
		return;
	Re01I2cDisable(pIntrf);
	for (int i = 0; i < dev->pI2cDev->Cfg.NbIOPins; i++)
	{
		const IOPinCfg_t *pin = &dev->pI2cDev->Cfg.pIOPinMap[i];
		IOPinDisable(pin->PortNo, pin->PinNo);
	}
	uint32_t state = DisableInterrupt();
	MSTP->MSTPCRB |= dev->StopMask;
	dev->pI2cDev = NULL;
	pIntrf->pDevData = NULL;
	pIntrf->EnCnt = 0;
	EnableInterrupt(state);
}

static void *Re01I2cGetHandle(DevIntrf_t *pIntrf)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	return dev ? dev->pI2cDev : NULL;
}

static uint32_t Re01I2cGetRate(DevIntrf_t *pIntrf)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	return dev ? dev->pI2cDev->Cfg.Rate : 0;
}

static uint32_t Re01I2cSetRate(DevIntrf_t *pIntrf, uint32_t Rate)
{
	Re01I2cDev_t *dev = Re01I2cHandle(pIntrf);
	uint8_t cks, brl, brh;
	uint32_t actual;
	if (!dev || dev->bOpen || !Re01I2cRate(Rate, &cks, &brl, &brh, &actual))
		return 0;
	dev->Cks = cks;
	dev->Brl = brl;
	dev->Brh = brh;
	dev->pI2cDev->Cfg.Rate = actual;
	Re01I2cConfigure(dev);
	return actual;
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfg)
{
	if (!pDev || !pCfg || pCfg->DevNo < 0 || pCfg->DevNo >= 2 ||
		pCfg->Type != I2CTYPE_STANDARD || pCfg->Mode != I2CMODE_MASTER ||
		pCfg->AddrType != I2CADDR_TYPE_NORMAL || pCfg->bIntEn || pCfg->bDmaEn ||
		!pCfg->pIOPinMap || pCfg->NbIOPins != 2 || pCfg->MaxRetry < 0)
		return false;
	for (int i = 0; i < 2; i++)
	{
		const IOPinCfg_t *pin = &pCfg->pIOPinMap[i];
		if (pin->PortNo < 0 || pin->PortNo >= 9 || pin->PinNo < 0 || pin->PinNo >= 16 ||
			pin->PinOp <= 0 || pin->PinOp >= IOPINOP_FUNC31 ||
			pin->Type != IOPINTYPE_OPENDRAIN ||
			(pin->Res != IOPINRES_NONE && pin->Res != IOPINRES_PULLUP && pin->Res != IOPINRES_FOLLOW))
			return false;
	}
	if (pCfg->pIOPinMap[0].PortNo == pCfg->pIOPinMap[1].PortNo &&
		pCfg->pIOPinMap[0].PinNo == pCfg->pIOPinMap[1].PinNo)
		return false;
	uint8_t cks, brl, brh;
	uint32_t actual;
	if (!Re01I2cRate(pCfg->Rate, &cks, &brl, &brh, &actual))
		return false;
	uint32_t state = DisableInterrupt();
	Re01I2cDev_t *dev = &s_Re01I2cDev[pCfg->DevNo];
	if ((dev->pI2cDev && dev->pI2cDev != pDev) || dev->bOpen ||
		s_Re01I2cDev[1 - pCfg->DevNo].pI2cDev == pDev)
	{
		EnableInterrupt(state);
		return false;
	}
	dev->pI2cDev = pDev;
	dev->Cks = cks;
	dev->Brl = brl;
	dev->Brh = brh;
	dev->bEnabled = true;
	dev->bOpen = dev->bActive = dev->bRx = dev->bError = false;
	MSTP->MSTPCRB &= ~dev->StopMask;
	(void)MSTP->MSTPCRB;
	EnableInterrupt(state);
	pDev->Cfg = *pCfg;
	pDev->Cfg.Rate = actual;
	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	DevIntrf_t *intrf = &pDev->DevIntrf;
	intrf->pDevData = dev;
	intrf->Type = DEVINTRF_TYPE_I2C;
	intrf->EnCnt = 1;
	intrf->bDma = intrf->bIntEn = false;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = Re01I2cEnable;
	intrf->Disable = Re01I2cDisable;
	intrf->PowerOff = Re01I2cPowerOff;
	intrf->Reset = Re01I2cReset;
	intrf->GetHandle = Re01I2cGetHandle;
	intrf->GetRate = Re01I2cGetRate;
	intrf->SetRate = Re01I2cSetRate;
	intrf->StartRx = Re01I2cStartRx;
	intrf->StartTx = Re01I2cStartTx;
	intrf->StopRx = intrf->StopTx = Re01I2cStop;
	intrf->RxData = Re01I2cRx;
	intrf->TxData = intrf->TxSrData = Re01I2cTx;
	atomic_flag_clear(&intrf->bBusy);
	Re01I2cConfigure(dev);
	return true;
}

/**-------------------------------------------------------------------------
@file	i2c_stm32wba.cpp

@brief	STM32WBA I2C master implementation

		DevNo 0 to 3 are I2C1, I2C2, I2C3 and I2C4. I2C1, I2C2 and I2C4 are
		clocked from PCLK1, I2C3 from PCLK7. The kernel clock selection is
		PCLK. The rate is set from the kernel clock with the PRESC, SCLDEL,
		SDADEL, SCLL and SCLH fields. SetRate and I2CInit return and store the
		rate actually set, the closest the dividers give: a request is not
		rejected for being out of range. Rate 0 selects 100 kHz. Above
		400 kHz the fast mode plus drive of the pins is enabled.

		Master mode, polling. Slave mode, interrupt, DMA and 10 bit addresses
		are not implemented: I2CInit returns false for them.

		The hardware takes the number of bytes with the START, which StartTx
		and StartRx do not know. StartTx sends the address alone with a STOP
		(NBYTES 0, AUTOEND, as HAL_I2C_IsDeviceReady does) and returns false
		when the device does not acknowledge it. StartRx only selects the
		address. TxData and RxData start the transfer with their data length,
		in chunks of up to 255 bytes with RELOAD, as HAL_I2C_Master_Transmit
		and HAL_I2C_Master_Receive do. The transfer
		ends with a STOP, or when the interface bNoStop is set (a command
		phase before a read) it is held with SCL low after the last byte, and
		the following StartRx and RxData generate a repeated START. A second
		TxData after one StartTx begins a new transfer. The address probe of
		StartTx adds one address byte to each write.

		A refused address or byte, an arbitration loss or a bus error stops
		the bus, clears the flags and returns the number of bytes done. Waits
		are bounded.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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
#include <string.h>
#include <stdint.h>

#include "stm32wbaxx.h"

#include "istddef.h"
#include "coredev/i2c.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"

#define WBA_I2C_MAXDEV				4

#define WBA_I2C_NBYTES_MAX			255

// Kernel clock of the I2C is the APB clock of the instance (CCIPR field 0)
#define WBA_I2C_CLKSEL_PCLK			0

#define WBA_I2C_RATE_DEFAULT		100000
#define WBA_I2C_RATE_MAX			1000000

// Minimum SCL times in picoseconds. Standard, fast and fast plus modes
#define WBA_I2C_TLOW_STD_PS			(I2C_SCL_TLOW_STD_MODE_MIN * 1000ULL)
#define WBA_I2C_THIGH_STD_PS		(I2C_SCL_THIGH_STD_MODE_MIN * 1000ULL)
#define WBA_I2C_TLOW_FAST_PS		(I2C_SCL_TLOW_FAST_MODE_MIN * 1000ULL)
#define WBA_I2C_THIGH_FAST_PS		(I2C_SCL_THIGH_FAST_MODE_MIN * 1000ULL)
#define WBA_I2C_TLOW_FMP_PS			(I2C_SCL_TLOW_FAST_MODE_PLUS_MIN * 1000ULL)
#define WBA_I2C_THIGH_FMP_PS		(I2C_SCL_THIGH_FAST_MODE_PLUS_MIN * 1000ULL)

// Analog filter delay, lower bound of the datasheet range, in picoseconds
#define WBA_I2C_TAF_MIN_PS			50000ULL

// Flags that end a transfer in error
#define WBA_I2C_ERR_FLAGS			(I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_TIMEOUT)
#define WBA_I2C_ICR_ALL				(I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | \
									 I2C_ICR_OVRCF | I2C_ICR_TIMOUTCF)

#pragma pack(push, 4)

typedef struct {
	int DevNo;
	I2CDev_t *pI2cDev;
	I2C_TypeDef *pReg;
	uint32_t Addr;			//!< 7 bit address of the open transfer
	bool bActive;			//!< StartTx or StartRx done, StopTx or StopRx not yet
	bool bRx;				//!< Direction of the open transfer
	bool bHold;				//!< Write ended without STOP, SCL held, next is a repeated start
	bool bError;			//!< Last transfer failed
} WbaI2cDev_t;

#pragma pack(pop)

static WbaI2cDev_t s_WbaI2cDev[WBA_I2C_MAXDEV] = {
	{ 0, NULL, I2C1, 0, false, false, false, false },
	{ 1, NULL, I2C2, 0, false, false, false, false },
	{ 2, NULL, I2C3, 0, false, false, false, false },
	{ 3, NULL, I2C4, 0, false, false, false, false },
};

static WbaI2cDev_t *WbaI2cHandle(DevIntrf_t * const pIntrf)
{
	WbaI2cDev_t *dev = pIntrf ? (WbaI2cDev_t*)pIntrf->pDevData : NULL;

	return dev && dev->pI2cDev && &dev->pI2cDev->DevIntrf == pIntrf ? dev : NULL;
}

// RCC kernel clock selection, clock enable and reset of one instance.
// I2C1, I2C2 and I2C4 are on APB1 and I2C3 is on APB7.
static void WbaI2cClkEnable(int DevNo, bool bEn)
{
	switch (DevNo)
	{
		case 0:
			if (bEn)
			{
				RCC->CCIPR1 &= ~RCC_CCIPR1_I2C1SEL_Msk;
				RCC->CCIPR1 |= WBA_I2C_CLKSEL_PCLK << RCC_CCIPR1_I2C1SEL_Pos;
				RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
			}
			else
			{
				RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C1EN;
			}
			break;
		case 1:
			if (bEn)
			{
				RCC->CCIPR1 &= ~RCC_CCIPR1_I2C2SEL_Msk;
				RCC->CCIPR1 |= WBA_I2C_CLKSEL_PCLK << RCC_CCIPR1_I2C2SEL_Pos;
				RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
			}
			else
			{
				RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;
			}
			break;
		case 2:
			if (bEn)
			{
				RCC->CCIPR3 &= ~RCC_CCIPR3_I2C3SEL_Msk;
				RCC->CCIPR3 |= WBA_I2C_CLKSEL_PCLK << RCC_CCIPR3_I2C3SEL_Pos;
				RCC->APB7ENR |= RCC_APB7ENR_I2C3EN;
			}
			else
			{
				RCC->APB7ENR &= ~RCC_APB7ENR_I2C3EN;
			}
			break;
		case 3:
			if (bEn)
			{
				RCC->CCIPR1 &= ~RCC_CCIPR1_I2C4SEL_Msk;
				RCC->CCIPR1 |= WBA_I2C_CLKSEL_PCLK << RCC_CCIPR1_I2C4SEL_Pos;
				RCC->APB1ENR2 |= RCC_APB1ENR2_I2C4EN;
			}
			else
			{
				RCC->APB1ENR2 &= ~RCC_APB1ENR2_I2C4EN;
			}
			break;
	}
}

static void WbaI2cRccReset(int DevNo)
{
	switch (DevNo)
	{
		case 0:
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;
			break;
		case 1:
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST;
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C2RST;
			break;
		case 2:
			RCC->APB7RSTR |= RCC_APB7RSTR_I2C3RST;
			RCC->APB7RSTR &= ~RCC_APB7RSTR_I2C3RST;
			break;
		case 3:
			RCC->APB1RSTR2 |= RCC_APB1RSTR2_I2C4RST;
			RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_I2C4RST;
			break;
	}
}

// Kernel clock in Hz: PCLK1 for I2C1, I2C2 and I2C4, PCLK7 for I2C3
static uint32_t WbaI2cClock(int DevNo)
{
	return SystemPeriphClockGet(DevNo == 2 ? 2 : 0);
}

// Wait bound in loop counts: a clock stretch allowance of 50 ms at the CPU
// clock, plus the time of 16 bit periods at the programmed rate.
static uint32_t WbaI2cTimeout(WbaI2cDev_t *pDev)
{
	uint32_t rate = pDev->pI2cDev->Cfg.Rate;

	if (rate == 0)
	{
		rate = 1;
	}

	return SystemCoreClock / 20U + (SystemCoreClock / rate) * 16U + 256U;
}

// Wait until (ISR & Mask) != 0. Return 1 on success, 0 on timeout,
// -1 when an error flag came first.
static int WbaI2cWait(WbaI2cDev_t *pDev, uint32_t Mask)
{
	uint32_t n = WbaI2cTimeout(pDev);

	do {
		uint32_t isr = pDev->pReg->ISR;

		if (isr & Mask)
		{
			return 1;
		}
		if (isr & WBA_I2C_ERR_FLAGS)
		{
			return -1;
		}
	} while (n--);

	return 0;
}

// Bring the transfer to an end after an error, as the ST HAL does
// (I2C_IsErrorOccurred): a NACK is followed by an automatic STOP, so wait for
// it. When the bus stays busy without a STOP, generate one. Then flush TXDR,
// clear the flags and the transfer setup in CR2.
static void WbaI2cAbort(WbaI2cDev_t *pDev)
{
	I2C_TypeDef *reg = pDev->pReg;
	uint32_t n = WbaI2cTimeout(pDev);

	while (!(reg->ISR & I2C_ISR_STOPF) && (reg->ISR & I2C_ISR_BUSY) && n--)
	{
	}
	if ((reg->ISR & I2C_ISR_BUSY) && !(reg->ISR & I2C_ISR_STOPF))
	{
		reg->CR2 |= I2C_CR2_STOP;
		n = WbaI2cTimeout(pDev);
		while (!(reg->ISR & I2C_ISR_STOPF) && n--)
		{
		}
	}
	if (reg->ISR & I2C_ISR_TXIS)
	{
		reg->TXDR = 0;
	}
	if (!(reg->ISR & I2C_ISR_TXE))
	{
		reg->ISR |= I2C_ISR_TXE;
	}
	reg->ICR = WBA_I2C_ICR_ALL;
	reg->CR2 = 0;
	pDev->bHold = false;
	pDev->bError = true;
}

// Compute TIMINGR for a kernel clock and a requested rate. The rate is
// rounded to what the divider chain gives, never rejected: the actual rate
// is returned. Returns false only when Clk is 0.
//
// SCL period = tsync1 + tsync2 + ((SCLL + 1) + (SCLH + 1)) * (PRESC + 1) * tclk
// tsync1 and tsync2 are 2 kernel clocks each plus the analog filter delay.
// The smallest PRESC that satisfies the SCL low and high minimums, the SCLDEL
// and SDADEL ranges and the 8 bit SCLL and SCLH is used.
static bool WbaI2cTiming(uint32_t Clk, uint32_t Rate, uint32_t *pTiming, uint32_t *pActual)
{
	uint64_t lowps, highps, trps, tsups, tfps;
	uint64_t tclkps;
	uint64_t period;

	if (Clk == 0)
	{
		return false;
	}
	if (Rate == 0)
	{
		Rate = WBA_I2C_RATE_DEFAULT;
	}
	if (Rate > WBA_I2C_RATE_MAX)
	{
		Rate = WBA_I2C_RATE_MAX;
	}

	if (Rate <= 100000)
	{
		lowps = WBA_I2C_TLOW_STD_PS;
		highps = WBA_I2C_THIGH_STD_PS;
		trps = I2C_TR_STDMODE_MAX * 1000ULL;
		tsups = I2C_TSUDAT_STDMODE_MIN * 1000ULL;
		tfps = I2C_TF_STDMODE_MAX * 1000ULL;
	}
	else if (Rate <= 400000)
	{
		lowps = WBA_I2C_TLOW_FAST_PS;
		highps = WBA_I2C_THIGH_FAST_PS;
		trps = I2C_TR_FASTMODE_MAX * 1000ULL;
		tsups = I2C_TSUDAT_FASTMODE_MIN * 1000ULL;
		tfps = I2C_TF_FASTMODE_MAX * 1000ULL;
	}
	else
	{
		lowps = WBA_I2C_TLOW_FMP_PS;
		highps = WBA_I2C_THIGH_FMP_PS;
		trps = I2C_TR_FASTMODEPLUS_MAX * 1000ULL;
		tsups = I2C_TSUDAT_FASTMODEPLUS_MIN * 1000ULL;
		tfps = I2C_TF_FASTMODEPLUS_MAX * 1000ULL;
	}

	tclkps = 1000000000000ULL / Clk;
	period = 1000000000000ULL / Rate;

	// Two synchronization delays, each 2 kernel clocks and the analog filter
	uint64_t tsync = 2 * (WBA_I2C_TAF_MIN_PS + 2 * tclkps);

	for (uint32_t presc = 0; presc < 16; presc++)
	{
		uint64_t tp = tclkps * (presc + 1);
		uint64_t lowmin = (lowps + tp - 1) / tp;
		uint64_t highmin = (highps + tp - 1) / tp;
		uint64_t total = period > tsync ? (period - tsync + tp - 1) / tp : 0;
		uint64_t low, high;
		uint64_t scldel = (trps + tsups + tp - 1) / tp;
		uint64_t sdadel = 0;
		uint64_t sdamin = WBA_I2C_TAF_MIN_PS + 3 * tclkps;

		// SCLDEL = (tr + tsu) / tp - 1, 4 bits
		if (scldel == 0 || scldel > 16)
		{
			continue;
		}
		scldel--;

		// SDADEL: tf - tAF(min) - 3 kernel clocks over tp, 4 bits
		if (tfps > sdamin)
		{
			sdadel = (tfps - sdamin + tp - 1) / tp;
		}
		if (sdadel > 15)
		{
			continue;
		}

		if (total < lowmin + highmin)
		{
			total = lowmin + highmin;
		}
		low = (total * lowmin + (lowmin + highmin) / 2) / (lowmin + highmin);
		if (low < lowmin)
		{
			low = lowmin;
		}
		high = total - low;
		if (high < highmin)
		{
			high = highmin;
			total = low + high;
		}
		if (low > 256 || high > 256)
		{
			continue;
		}

		*pTiming = (presc << I2C_TIMINGR_PRESC_Pos) | ((uint32_t)scldel << I2C_TIMINGR_SCLDEL_Pos) |
				   ((uint32_t)sdadel << I2C_TIMINGR_SDADEL_Pos) |
				   ((uint32_t)(high - 1) << I2C_TIMINGR_SCLH_Pos) | (uint32_t)(low - 1);
		*pActual = (uint32_t)(1000000000000ULL / (tsync + total * tp));

		return true;
	}

	// The requested rate is lower than the divider chain gives with the
	// slowest PRESC: use the slowest setting.
	{
		uint64_t tp = tclkps * 16;
		uint64_t scldel = (trps + tsups + tp - 1) / tp;

		scldel = scldel == 0 ? 0 : (scldel > 16 ? 15 : scldel - 1);
		*pTiming = (15U << I2C_TIMINGR_PRESC_Pos) | ((uint32_t)scldel << I2C_TIMINGR_SCLDEL_Pos) |
				   (255U << I2C_TIMINGR_SCLH_Pos) | 255U;
		*pActual = (uint32_t)(1000000000000ULL / (tsync + 512 * tp));
	}

	return true;
}

static uint32_t WbaI2cGetRate(DevIntrf_t * const pIntrf)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	return dev ? dev->pI2cDev->Cfg.Rate : 0;
}

// Set the rate in Hz. Returns the rate actually set, the closest the divider
// chain gives to the request. Rate 0 selects 100 kHz. The peripheral is
// disabled while TIMINGR is written.
static uint32_t WbaI2cSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	if (dev == NULL)
	{
		return 0;
	}

	uint32_t timing;
	uint32_t actual;

	if (WbaI2cTiming(WbaI2cClock(dev->DevNo), Rate, &timing, &actual) == false)
	{
		return dev->pI2cDev->Cfg.Rate;
	}

	I2C_TypeDef *reg = dev->pReg;
	bool pe = (reg->CR1 & I2C_CR1_PE) != 0;

	reg->CR1 &= ~I2C_CR1_PE;
	reg->TIMINGR = timing;
	// Fast mode plus drive above 400 kHz
	if (actual > 400000)
	{
		reg->CR1 |= I2C_CR1_FMP;
	}
	else
	{
		reg->CR1 &= ~I2C_CR1_FMP;
	}
	if (pe)
	{
		reg->CR1 |= I2C_CR1_PE;
	}
	dev->pI2cDev->Cfg.Rate = actual;

	return actual;
}

static void WbaI2cReset(DevIntrf_t * const pIntrf)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	// PE low resets the state machine and the flags, TIMINGR is kept
	dev->pReg->CR1 &= ~I2C_CR1_PE;
	for (uint32_t n = 100; n > 0 && (dev->pReg->CR1 & I2C_CR1_PE); n--)
	{
	}
	dev->pReg->CR1 |= I2C_CR1_PE;
	dev->bActive = false;
	dev->bHold = false;
	dev->bError = false;
}

static void WbaI2cEnable(DevIntrf_t * const pIntrf)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	if (dev)
	{
		WbaI2cClkEnable(dev->DevNo, true);
		dev->pReg->CR1 |= I2C_CR1_PE;
	}
}

static void WbaI2cDisable(DevIntrf_t * const pIntrf)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	if (dev)
	{
		dev->pReg->CR1 &= ~I2C_CR1_PE;
		WbaI2cClkEnable(dev->DevNo, false);
	}
}

static void WbaI2cPowerOff(DevIntrf_t * const pIntrf)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	if (dev)
	{
		dev->pReg->CR1 &= ~I2C_CR1_PE;
		WbaI2cClkEnable(dev->DevNo, false);
		IOPinDis(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);
	}
}

// Open a transfer. Common checks: the interface is enabled and the bus is
// free, or held by a write that a read follows with a repeated START.
static bool WbaI2cStart(DevIntrf_t * const pIntrf, uint32_t DevAddr, bool bRx)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	if (dev == NULL || !(dev->pReg->CR1 & I2C_CR1_PE))
	{
		return false;
	}

	if (dev->bHold && !bRx)
	{
		// A new write cannot follow a held write: end the held one first
		dev->pReg->CR2 |= I2C_CR2_STOP;
		(void)WbaI2cWait(dev, I2C_ISR_STOPF);
		dev->pReg->ICR = WBA_I2C_ICR_ALL;
		dev->pReg->CR2 = 0;
		dev->bHold = false;
	}
	if (!dev->bHold)
	{
		uint32_t n = WbaI2cTimeout(dev);

		while ((dev->pReg->ISR & I2C_ISR_BUSY) && n--)
		{
		}
		if (dev->pReg->ISR & I2C_ISR_BUSY)
		{
			return false;
		}
	}

	dev->pReg->ICR = WBA_I2C_ICR_ALL;
	dev->Addr = DevAddr & 0x7F;
	dev->bRx = bRx;
	dev->bError = false;
	dev->bActive = true;

	return true;
}

// The hardware takes the number of bytes with the START, which is not known
// here. StartTx sends the address alone (NBYTES 0 with AUTOEND, the ST HAL
// device ready probe) and returns false when the device does not acknowledge
// it. TxData then starts the transfer of its data.
static bool WbaI2cStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	if (WbaI2cStart(pIntrf, DevAddr, false) == false)
	{
		return false;
	}

	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);
	I2C_TypeDef *reg = dev->pReg;

	reg->CR2 = ((dev->Addr << 1) & I2C_CR2_SADD_Msk) | I2C_CR2_AUTOEND | I2C_CR2_START;

	int res = WbaI2cWait(dev, I2C_ISR_STOPF);

	if (res != 1)
	{
		WbaI2cAbort(dev);
		dev->bActive = false;

		return false;
	}

	bool ack = (reg->ISR & I2C_ISR_NACKF) == 0;

	reg->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
	reg->CR2 = 0;
	if (!ack)
	{
		dev->bActive = false;

		return false;
	}

	return true;
}

static bool WbaI2cStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	return WbaI2cStart(pIntrf, DevAddr, true);
}

// Send Data in chunks of up to 255 bytes. The hardware needs NBYTES with
// the START. A longer transfer uses RELOAD between chunks. The last chunk
// ends with STOP (AUTOEND), or, when the interface bNoStop is set, with
// the bus held after the last byte (TC), so that a following StartRx
// generates a repeated START. All of Data goes out in one call: a second
// TxData after StartTx begins a new START.
static int WbaI2cTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	if (dev == NULL || !dev->bActive || dev->bRx || dev->bError || pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	I2C_TypeDef *reg = dev->pReg;
	int cnt = 0;
	bool first = true;
	bool bNoStop = pIntrf->bNoStop;

	while (cnt < DataLen)
	{
		int len = DataLen - cnt;
		bool last = len <= WBA_I2C_NBYTES_MAX;
		uint32_t cr2;

		if (!last)
		{
			len = WBA_I2C_NBYTES_MAX;
		}

		cr2 = ((dev->Addr << 1) & I2C_CR2_SADD_Msk) | ((uint32_t)len << I2C_CR2_NBYTES_Pos);
		if (!last)
		{
			cr2 |= I2C_CR2_RELOAD;
		}
		else if (!bNoStop)
		{
			cr2 |= I2C_CR2_AUTOEND;
		}
		if (first)
		{
			cr2 |= I2C_CR2_START;
			first = false;
		}
		reg->CR2 = cr2;

		for (int i = 0; i < len; i++)
		{
			if (WbaI2cWait(dev, I2C_ISR_TXIS) != 1)
			{
				// A refused byte is the last one written to TXDR
				if ((reg->ISR & I2C_ISR_NACKF) && cnt > 0)
				{
					cnt--;
				}
				WbaI2cAbort(dev);
				return cnt;
			}
			reg->TXDR = pData[cnt];
			cnt++;
		}

		if (!last)
		{
			if (WbaI2cWait(dev, I2C_ISR_TCR) != 1)
			{
				WbaI2cAbort(dev);
				return cnt;
			}
		}
		else if (bNoStop)
		{
			if (WbaI2cWait(dev, I2C_ISR_TC) != 1)
			{
				WbaI2cAbort(dev);
				return cnt;
			}
			dev->bHold = true;
		}
		else
		{
			if (WbaI2cWait(dev, I2C_ISR_STOPF) != 1)
			{
				WbaI2cAbort(dev);
				return cnt;
			}
			// The last byte was refused: the STOP came with a NACK
			if (reg->ISR & I2C_ISR_NACKF)
			{
				WbaI2cAbort(dev);
				return cnt - 1;
			}
			reg->ICR = I2C_ICR_STOPCF;
			reg->CR2 = 0;
		}
	}

	return cnt;
}

// Receive BuffLen bytes. Starts the transfer with START, or a repeated START
// when a write was held. Chunks of up to 255 bytes with RELOAD between them,
// the last one ends with NACK and STOP (AUTOEND).
static int WbaI2cRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	if (dev == NULL || !dev->bActive || !dev->bRx || dev->bError || pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	I2C_TypeDef *reg = dev->pReg;
	int cnt = 0;
	bool first = true;

	dev->bHold = false;

	while (cnt < BuffLen)
	{
		int len = BuffLen - cnt;
		bool last = len <= WBA_I2C_NBYTES_MAX;
		uint32_t cr2;

		if (!last)
		{
			len = WBA_I2C_NBYTES_MAX;
		}

		cr2 = ((dev->Addr << 1) & I2C_CR2_SADD_Msk) | I2C_CR2_RD_WRN | ((uint32_t)len << I2C_CR2_NBYTES_Pos);
		cr2 |= last ? I2C_CR2_AUTOEND : I2C_CR2_RELOAD;
		if (first)
		{
			cr2 |= I2C_CR2_START;
			first = false;
		}
		reg->CR2 = cr2;

		for (int i = 0; i < len; i++)
		{
			if (WbaI2cWait(dev, I2C_ISR_RXNE) != 1)
			{
				WbaI2cAbort(dev);
				return cnt;
			}
			pBuff[cnt] = (uint8_t)reg->RXDR;
			cnt++;
		}

		if (!last)
		{
			if (WbaI2cWait(dev, I2C_ISR_TCR) != 1)
			{
				WbaI2cAbort(dev);
				return cnt;
			}
		}
		else
		{
			if (WbaI2cWait(dev, I2C_ISR_STOPF) != 1)
			{
				WbaI2cAbort(dev);
				return cnt;
			}
			reg->ICR = I2C_ICR_STOPCF;
			reg->CR2 = 0;
		}
	}

	return cnt;
}

// Close the transfer. A write held without STOP, with no read after it, is
// ended here.
static void WbaI2cStop(DevIntrf_t * const pIntrf)
{
	WbaI2cDev_t *dev = WbaI2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	if (dev->bHold)
	{
		dev->pReg->CR2 |= I2C_CR2_STOP;
		(void)WbaI2cWait(dev, I2C_ISR_STOPF);
		dev->bHold = false;
	}
	dev->pReg->ICR = WBA_I2C_ICR_ALL;
	dev->pReg->CR2 = 0;
	dev->bActive = false;
}

// Master mode, polling. Slave mode, interrupt and DMA are not implemented
// and make I2CInit fail. DevNo 0 to 3 are I2C1 to I2C4.
bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->DevNo < 0 || pCfgData->DevNo >= WBA_I2C_MAXDEV)
	{
		return false;
	}

	if (pCfgData->Mode != I2CMODE_MASTER || pCfgData->bIntEn || pCfgData->bDmaEn ||
		pCfgData->AddrType != I2CADDR_TYPE_NORMAL)
	{
		return false;
	}

	WbaI2cDev_t *dev = &s_WbaI2cDev[pCfgData->DevNo];
	I2C_TypeDef *reg = dev->pReg;

	memcpy(&pDev->Cfg, pCfgData, sizeof(I2CCfg_t));

	dev->pI2cDev = pDev;
	dev->bActive = false;
	dev->bRx = false;
	dev->bHold = false;
	dev->bError = false;

	pDev->DevIntrf.pDevData = (void*)dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2C;
	pDev->DevIntrf.Reset = WbaI2cReset;
	pDev->DevIntrf.Disable = WbaI2cDisable;
	pDev->DevIntrf.Enable = WbaI2cEnable;
	pDev->DevIntrf.PowerOff = WbaI2cPowerOff;
	pDev->DevIntrf.GetRate = WbaI2cGetRate;
	pDev->DevIntrf.SetRate = WbaI2cSetRate;
	pDev->DevIntrf.StartRx = WbaI2cStartRx;
	pDev->DevIntrf.RxData = WbaI2cRxData;
	pDev->DevIntrf.StopRx = WbaI2cStop;
	pDev->DevIntrf.StartTx = WbaI2cStartTx;
	pDev->DevIntrf.TxData = WbaI2cTxData;
	pDev->DevIntrf.TxSrData = WbaI2cTxData;
	pDev->DevIntrf.StopTx = WbaI2cStop;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = false;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	WbaI2cClkEnable(pCfgData->DevNo, true);
	WbaI2cRccReset(pCfgData->DevNo);

	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);
	for (int i = 0; i < pDev->Cfg.NbIOPins; i++)
	{
		IOPinSetSpeed(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	// Master only: no own address, analog filter on, digital filter off
	reg->CR1 = 0;
	reg->OAR1 = 0;
	reg->OAR2 = 0;
	reg->TIMEOUTR = 0;
	reg->AUTOCR = 0;

	WbaI2cSetRate(&pDev->DevIntrf, pCfgData->Rate);

	reg->ICR = WBA_I2C_ICR_ALL;
	reg->CR1 |= I2C_CR1_PE;

	return true;
}

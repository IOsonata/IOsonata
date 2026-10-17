/**-------------------------------------------------------------------------
@file	i2c_stm32.cpp

@brief	Generic STM32 I2C implementation, master and slave

		For the STM32 families with the I2C version 2 peripheral (TIMINGR,
		ISR with TXIS and RXNE, NBYTES in CR2): WBA, L4 and G4. The engine is
		the same for all, the instance table holds what differs: the RCC
		enable, reset and kernel clock selection, the interrupts and the DMA
		requests. A family is added with its table.

		DevNo 0 to 3 are I2C1 to I2C4 of the part. The kernel clock selection
		is the APB clock. The rate is set from the kernel clock with the PRESC,
		SCLDEL, SDADEL, SCLL and SCLH fields. SetRate and I2CInit return and
		store the rate actually set, the closest the dividers give: a request
		is not rejected for being out of range. Rate 0 selects 100 kHz. Above
		400 kHz the fast mode plus drive of the pins is enabled where the
		peripheral has the control bit (WBA).

		Master. The hardware takes the number of bytes with the START, which
		StartTx and StartRx do not know. StartTx sends the address alone with a
		STOP (NBYTES 0, AUTOEND, as HAL_I2C_IsDeviceReady does) and returns
		false when the device does not acknowledge it. StartRx only selects the
		address. TxData and RxData start the transfer with their data length,
		in chunks of up to 255 bytes with RELOAD. The transfer ends with a
		STOP, or when the interface bNoStop is set (a command phase before a
		read) it is held with SCL low after the last byte, and the following
		StartRx and RxData generate a repeated START. A second TxData after one
		StartTx begins a new transfer. The address probe of StartTx adds one
		address byte to each write.

		Polling, interrupt and DMA run the same engine. Without bIntEn the
		call waits for the end of the transfer. With bIntEn a receive, and a
		transmit of up to I2C_STM32_TXBUF_SIZE bytes without bNoStop, return -1
		at once and the end is reported with DEVINTRF_EVT_COMPLETED and the
		count; the transmit data is copied first. With bDmaEn the data bytes
		are moved by a GPDMA channel. DMA is available on the families that
		have a channel table (WBA); I2CInit returns false for bDmaEn on the
		others. A DMA channel is taken at I2CInit and kept until PowerOff.

		A refused address or byte, an arbitration loss, a bus error or a bus
		timeout stops the bus, clears the flags and returns the number of
		bytes done. Waits are bounded in polling mode, and the SCL low timeout
		of the peripheral ends a stuck transfer in interrupt mode.

		Slave. Interrupt driven. One or two 7 bit addresses (OAR1 and OAR2).
		The slave stretches SCL until the application has answered the event.
		The sequence follows the other IOsonata I2C slaves: a master write
		raises DEVINTRF_EVT_WRITE_RQST, the application sets the receive
		buffer with SetWriteRqstBuffer, the bytes are stored and the end is
		reported with DEVINTRF_EVT_COMPLETED and the byte count. A master read
		raises DEVINTRF_EVT_READ_RQST with the byte count of a write that came
		before it with a repeated START, the application sets the data with
		SetReadRqstData; bytes past the data are sent as 0xFF and not counted.
		Bytes received past the buffer are acknowledged and dropped, not
		counted. DMA is not used in slave mode: the length of a transfer is
		not known in advance, bDma reports false. 10 bit addresses and the
		general call are not implemented: I2CInit returns false for a 10 bit
		address type.

@author	Hoang Nguyen Hoan
@date	Oct. 17, 2026

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

#include "stm32.h"

#if defined(IOSONATA_STM32_WBA)
#include "gpdma_stm32wba.h"
#define STM32_I2C_DMA
#endif

#include "istddef.h"
#include "coredev/i2c.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"

#define STM32_I2C_MAXDEV			4

#define STM32_I2C_NBYTES_MAX		255

#define I2C_STM32_TXBUF_SIZE		64

#define STM32_I2C_RATE_DEFAULT		100000
#define STM32_I2C_RATE_MAX			1000000

// SCL low timeout of the master in interrupt mode, 50 ms
#define STM32_I2C_TIMEOUT_MS		50U

// Minimum SCL times in picoseconds. Standard, fast and fast plus modes
#define STM32_I2C_TLOW_STD_PS		(I2C_SCL_TLOW_STD_MODE_MIN * 1000ULL)
#define STM32_I2C_THIGH_STD_PS		(I2C_SCL_THIGH_STD_MODE_MIN * 1000ULL)
#define STM32_I2C_TLOW_FAST_PS		(I2C_SCL_TLOW_FAST_MODE_MIN * 1000ULL)
#define STM32_I2C_THIGH_FAST_PS		(I2C_SCL_THIGH_FAST_MODE_MIN * 1000ULL)
#define STM32_I2C_TLOW_FMP_PS		(I2C_SCL_TLOW_FAST_MODE_PLUS_MIN * 1000ULL)
#define STM32_I2C_THIGH_FMP_PS		(I2C_SCL_THIGH_FAST_MODE_PLUS_MIN * 1000ULL)

// Analog filter delay, lower bound of the datasheet range, in picoseconds
#define STM32_I2C_TAF_MIN_PS		50000ULL

// Flags that end a transfer in error. A NACK is handled apart: the STOP
// follows it.
#define STM32_I2C_ERR_FLAGS			(I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_TIMEOUT)
#define STM32_I2C_ICR_ALL			(I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | \
									 I2C_ICR_OVRCF | I2C_ICR_TIMOUTCF | I2C_ICR_ADDRCF)

// Interrupt enables of a master transfer, without the data one
#define STM32_I2C_CR1_MASTER_INT	(I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE | I2C_CR1_TCIE)
#define STM32_I2C_CR1_DATA_INT		(I2C_CR1_TXIE | I2C_CR1_RXIE)
#define STM32_I2C_CR1_DMA			(I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN)
#define STM32_I2C_CR1_SLAVE_INT		(I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_STOPIE | \
									 I2C_CR1_NACKIE | I2C_CR1_ERRIE)

#define STM32_I2C_SLAVE_MAX_ADDR	2

#pragma pack(push, 4)

/// What differs between the instances: registers of the RCC, interrupts, DMA
typedef struct {
	I2C_TypeDef *pReg;
	volatile uint32_t *pEnReg;		//!< RCC clock enable register
	uint32_t EnMask;
	volatile uint32_t *pRstReg;		//!< RCC reset register
	uint32_t RstMask;
	volatile uint32_t *pSelReg;		//!< RCC kernel clock selection register, NULL none
	uint32_t SelMask;				//!< Selection field, written 0 for the APB clock
	int ClkIdx;						//!< SystemPeriphClockGet index of the APB clock
	IRQn_Type EvIrq;
	IRQn_Type ErIrq;
	uint8_t TxReq;					//!< GPDMA request of the transmit, 0 none
	uint8_t RxReq;
} Stm32I2cInfo_t;

typedef enum {
	STM32_I2C_S_IDLE,
	STM32_I2C_S_WRITE,				//!< Master writes, the slave receives
	STM32_I2C_S_READ				//!< Master reads, the slave transmits
} Stm32I2cSState_t;

typedef struct {
	int DevNo;
	const Stm32I2cInfo_t *pInfo;
	I2CDev_t *pI2cDev;
	I2C_TypeDef *pReg;
	// Master
	uint32_t Addr;			//!< 7 bit address of the open transfer
	bool bActive;			//!< StartTx or StartRx done, StopTx or StopRx not yet
	bool bRx;				//!< Direction of the open transfer
	bool bHold;				//!< Write ended without STOP, SCL held, next is a repeated start
	bool bError;			//!< Last transfer failed
	bool bRun;				//!< Transfer engine running
	bool bAsync;			//!< Running transfer ends in the interrupt and reports with the event
	bool bNack;				//!< Address or byte refused
	bool bFail;				//!< Bus error, arbitration loss, timeout
	bool bStopEnd;			//!< The last chunk ends with a STOP
	bool bDma;				//!< Data bytes moved by DMA
	int DmaTx;				//!< GPDMA channel of the transmit, -1 none
	int DmaRx;
	const uint8_t *pTx;
	uint8_t *pRx;
	int Len;				//!< Bytes of the transfer
	int Done;				//!< Bytes of the chunks completed
	int Chunk;				//!< Bytes of the running chunk
	int Moved;				//!< Bytes moved by the CPU
	int Result;				//!< Bytes done of the last transfer
	// Slave
	Stm32I2cSState_t SState;
	int SIdx;
	int SCount;
	bool bTxData;		// the byte loaded last was data, not filler
	uint8_t TxBuf[I2C_STM32_TXBUF_SIZE];
} Stm32I2cDev_t;

#pragma pack(pop)

// Instance tables
#if defined(IOSONATA_STM32_WBA)
// I2C1, I2C2 and I2C4 are on APB1 and clocked from PCLK1, I2C3 is on APB7
// and clocked from PCLK7.
static const Stm32I2cInfo_t s_Stm32I2cInfo[STM32_I2C_MAXDEV] = {
	{ I2C1, &RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN, &RCC->APB1RSTR1, RCC_APB1RSTR1_I2C1RST,
	  &RCC->CCIPR1, RCC_CCIPR1_I2C1SEL_Msk, 0, I2C1_EV_IRQn, I2C1_ER_IRQn,
	  WBA_GPDMA_REQ_I2C1_TX, WBA_GPDMA_REQ_I2C1_RX },
	{ I2C2, &RCC->APB1ENR1, RCC_APB1ENR1_I2C2EN, &RCC->APB1RSTR1, RCC_APB1RSTR1_I2C2RST,
	  &RCC->CCIPR1, RCC_CCIPR1_I2C2SEL_Msk, 0, I2C2_EV_IRQn, I2C2_ER_IRQn,
	  WBA_GPDMA_REQ_I2C2_TX, WBA_GPDMA_REQ_I2C2_RX },
	{ I2C3, &RCC->APB7ENR, RCC_APB7ENR_I2C3EN, &RCC->APB7RSTR, RCC_APB7RSTR_I2C3RST,
	  &RCC->CCIPR3, RCC_CCIPR3_I2C3SEL_Msk, 2, I2C3_EV_IRQn, I2C3_ER_IRQn,
	  WBA_GPDMA_REQ_I2C3_TX, WBA_GPDMA_REQ_I2C3_RX },
	{ I2C4, &RCC->APB1ENR2, RCC_APB1ENR2_I2C4EN, &RCC->APB1RSTR2, RCC_APB1RSTR2_I2C4RST,
	  &RCC->CCIPR1, RCC_CCIPR1_I2C4SEL_Msk, 0, I2C4_EV_IRQn, I2C4_ER_IRQn,
	  WBA_GPDMA_REQ_I2C4_TX, WBA_GPDMA_REQ_I2C4_RX },
};
#elif defined(IOSONATA_STM32_L4) || defined(IOSONATA_STM32_G4)
// I2C1 to I2C3 are in APB1ENR1, I2C4 in APB1ENR2, all clocked from PCLK1
static const Stm32I2cInfo_t s_Stm32I2cInfo[STM32_I2C_MAXDEV] = {
	{ I2C1, &RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN, &RCC->APB1RSTR1, RCC_APB1RSTR1_I2C1RST,
	  &RCC->CCIPR, RCC_CCIPR_I2C1SEL_Msk, 0, I2C1_EV_IRQn, I2C1_ER_IRQn, 0, 0 },
#ifdef I2C2
	{ I2C2, &RCC->APB1ENR1, RCC_APB1ENR1_I2C2EN, &RCC->APB1RSTR1, RCC_APB1RSTR1_I2C2RST,
	  &RCC->CCIPR, RCC_CCIPR_I2C2SEL_Msk, 0, I2C2_EV_IRQn, I2C2_ER_IRQn, 0, 0 },
#else
	{ NULL },
#endif
#ifdef I2C3
	{ I2C3, &RCC->APB1ENR1, RCC_APB1ENR1_I2C3EN, &RCC->APB1RSTR1, RCC_APB1RSTR1_I2C3RST,
	  &RCC->CCIPR, RCC_CCIPR_I2C3SEL_Msk, 0, I2C3_EV_IRQn, I2C3_ER_IRQn, 0, 0 },
#else
	{ NULL },
#endif
#ifdef I2C4
	{ I2C4, &RCC->APB1ENR2, RCC_APB1ENR2_I2C4EN, &RCC->APB1RSTR2, RCC_APB1RSTR2_I2C4RST,
	  &RCC->CCIPR2, RCC_CCIPR2_I2C4SEL_Msk, 0, I2C4_EV_IRQn, I2C4_ER_IRQn, 0, 0 },
#else
	{ NULL },
#endif
};
#else
#error "i2c_stm32.cpp: no I2C instance table for this STM32 family"
#endif

static Stm32I2cDev_t s_Stm32I2cDev[STM32_I2C_MAXDEV];

static Stm32I2cDev_t *Stm32I2cHandle(DevIntrf_t * const pIntrf)
{
	Stm32I2cDev_t *dev = pIntrf ? (Stm32I2cDev_t*)pIntrf->pDevData : NULL;

	return dev && dev->pI2cDev && &dev->pI2cDev->DevIntrf == pIntrf ? dev : NULL;
}

static void Stm32I2cClkEnable(Stm32I2cDev_t * const pDev, bool bEn)
{
	const Stm32I2cInfo_t *info = pDev->pInfo;

	if (bEn)
	{
		if (info->pSelReg)
		{
			*info->pSelReg &= ~info->SelMask;
		}
		*info->pEnReg |= info->EnMask;
		(void)*info->pEnReg;
	}
	else
	{
		*info->pEnReg &= ~info->EnMask;
	}
}

static void Stm32I2cRccReset(Stm32I2cDev_t * const pDev)
{
	const Stm32I2cInfo_t *info = pDev->pInfo;

	*info->pRstReg |= info->RstMask;
	*info->pRstReg &= ~info->RstMask;
}

// Kernel clock in Hz
static uint32_t Stm32I2cClock(Stm32I2cDev_t * const pDev)
{
	return SystemPeriphClockGet(pDev->pInfo->ClkIdx);
}

// Wait bound in loop counts: a clock stretch allowance of 50 ms at the CPU
// clock, plus the time of 16 bit periods at the programmed rate.
static uint32_t Stm32I2cTimeout(Stm32I2cDev_t * const pDev)
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
static int Stm32I2cWait(Stm32I2cDev_t * const pDev, uint32_t Mask)
{
	uint32_t n = Stm32I2cTimeout(pDev);

	do {
		uint32_t isr = pDev->pReg->ISR;

		if (isr & Mask)
		{
			return 1;
		}
		if (isr & (STM32_I2C_ERR_FLAGS | I2C_ISR_NACKF))
		{
			return -1;
		}
	} while (n--);

	return 0;
}

// TXDR emptied as the ST HAL does (I2C_Flush_TXDR)
static void Stm32I2cFlushTx(I2C_TypeDef * const pReg)
{
	if (pReg->ISR & I2C_ISR_TXIS)
	{
		pReg->TXDR = 0;
	}
	if (!(pReg->ISR & I2C_ISR_TXE))
	{
		pReg->ISR |= I2C_ISR_TXE;
	}
}

// PE low resets the state machine and the flags, the other registers are kept
static void Stm32I2cPeReset(I2C_TypeDef * const pReg)
{
	pReg->CR1 &= ~I2C_CR1_PE;
	for (uint32_t n = 100; n > 0 && (pReg->CR1 & I2C_CR1_PE); n--)
	{
	}
	for (uint32_t n = 16; n > 0; n--)
	{
		__NOP();
	}
	pReg->CR1 |= I2C_CR1_PE;
}

// Compute TIMINGR for a kernel clock and a requested rate. The rate is
// rounded to what the divider chain gives, never rejected: the actual rate
// is returned. Returns false only when Clk is 0.
//
// SCL period = tsync1 + tsync2 + ((SCLL + 1) + (SCLH + 1)) * (PRESC + 1) * tclk
// tsync1 and tsync2 are 2 kernel clocks each plus the analog filter delay.
// The smallest PRESC that satisfies the SCL low and high minimums, the SCLDEL
// and SDADEL ranges and the 8 bit SCLL and SCLH is used.
static bool Stm32I2cTiming(uint32_t Clk, uint32_t Rate, uint32_t *pTiming, uint32_t *pActual)
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
		Rate = STM32_I2C_RATE_DEFAULT;
	}
	if (Rate > STM32_I2C_RATE_MAX)
	{
		Rate = STM32_I2C_RATE_MAX;
	}

	if (Rate <= 100000)
	{
		lowps = STM32_I2C_TLOW_STD_PS;
		highps = STM32_I2C_THIGH_STD_PS;
		trps = I2C_TR_STDMODE_MAX * 1000ULL;
		tsups = I2C_TSUDAT_STDMODE_MIN * 1000ULL;
		tfps = I2C_TF_STDMODE_MAX * 1000ULL;
	}
	else if (Rate <= 400000)
	{
		lowps = STM32_I2C_TLOW_FAST_PS;
		highps = STM32_I2C_THIGH_FAST_PS;
		trps = I2C_TR_FASTMODE_MAX * 1000ULL;
		tsups = I2C_TSUDAT_FASTMODE_MIN * 1000ULL;
		tfps = I2C_TF_FASTMODE_MAX * 1000ULL;
	}
	else
	{
		lowps = STM32_I2C_TLOW_FMP_PS;
		highps = STM32_I2C_THIGH_FMP_PS;
		trps = I2C_TR_FASTMODEPLUS_MAX * 1000ULL;
		tsups = I2C_TSUDAT_FASTMODEPLUS_MIN * 1000ULL;
		tfps = I2C_TF_FASTMODEPLUS_MAX * 1000ULL;
	}

	tclkps = 1000000000000ULL / Clk;
	period = 1000000000000ULL / Rate;

	// Two synchronization delays, each 2 kernel clocks and the analog filter
	uint64_t tsync = 2 * (STM32_I2C_TAF_MIN_PS + 2 * tclkps);

	for (uint32_t presc = 0; presc < 16; presc++)
	{
		uint64_t tp = tclkps * (presc + 1);
		uint64_t lowmin = (lowps + tp - 1) / tp;
		uint64_t highmin = (highps + tp - 1) / tp;
		uint64_t total = period > tsync ? (period - tsync + tp - 1) / tp : 0;
		uint64_t low, high;
		uint64_t scldel = (trps + tsups + tp - 1) / tp;
		uint64_t sdadel = 0;
		uint64_t sdamin = STM32_I2C_TAF_MIN_PS + 3 * tclkps;

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

static uint32_t Stm32I2cGetRate(DevIntrf_t * const pIntrf)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	return dev ? dev->pI2cDev->Cfg.Rate : 0;
}

// Set the rate in Hz. Returns the rate actually set, the closest the divider
// chain gives to the request. Rate 0 selects 100 kHz. The peripheral is
// disabled while TIMINGR is written.
static uint32_t Stm32I2cSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return 0;
	}

	uint32_t timing;
	uint32_t actual;

	if (Stm32I2cTiming(Stm32I2cClock(dev), Rate, &timing, &actual) == false)
	{
		return dev->pI2cDev->Cfg.Rate;
	}

	I2C_TypeDef *reg = dev->pReg;
	bool pe = (reg->CR1 & I2C_CR1_PE) != 0;

	reg->CR1 &= ~I2C_CR1_PE;
	reg->TIMINGR = timing;
#ifdef I2C_CR1_FMP
	// Fast mode plus drive above 400 kHz
	if (actual > 400000)
	{
		reg->CR1 |= I2C_CR1_FMP;
	}
	else
	{
		reg->CR1 &= ~I2C_CR1_FMP;
	}
#endif
	if (pe)
	{
		reg->CR1 |= I2C_CR1_PE;
	}
	dev->pI2cDev->Cfg.Rate = actual;

	return actual;
}

// SCL low timeout of the master, TIMEOUTA for 50 ms. Written with the
// peripheral disabled.
static void Stm32I2cMasterTimeout(Stm32I2cDev_t * const pDev)
{
	uint64_t t = ((uint64_t)Stm32I2cClock(pDev) * STM32_I2C_TIMEOUT_MS) / (2048ULL * 1000ULL);

	if (t > 0)
	{
		t--;
	}
	if (t > 0xFFFULL)
	{
		t = 0xFFFULL;
	}
	pDev->pReg->TIMEOUTR = ((uint32_t)t << I2C_TIMEOUTR_TIMEOUTA_Pos) | I2C_TIMEOUTR_TIMOUTEN;
}

// ---------------------------------------------------------------------------
// DMA
// ---------------------------------------------------------------------------

#ifdef STM32_I2C_DMA
static void Stm32I2cDmaIrq(int Ch, void *pCtx);
static void Stm32I2cService(Stm32I2cDev_t * const pDev);

// Take the two channels of the master. Both or none.
static bool Stm32I2cDmaTake(Stm32I2cDev_t * const pDev, bool bIrq, int IntPrio)
{
	const Stm32I2cInfo_t *info = pDev->pInfo;

	if (pDev->DmaTx >= 0 && pDev->DmaRx >= 0)
	{
		return true;
	}

	pDev->DmaTx = WbaGpdmaAlloc(Stm32I2cDmaIrq, pDev, IntPrio, bIrq);
	if (pDev->DmaTx < 0)
	{
		return false;
	}
	pDev->DmaRx = WbaGpdmaAlloc(Stm32I2cDmaIrq, pDev, IntPrio, bIrq);
	if (pDev->DmaRx < 0)
	{
		WbaGpdmaFree(pDev->DmaTx);
		pDev->DmaTx = -1;

		return false;
	}

	WbaGpdmaSetup(pDev->DmaTx, info->TxReq, true, &pDev->pReg->TXDR, 0, bIrq);
	WbaGpdmaSetup(pDev->DmaRx, info->RxReq, false, &pDev->pReg->RXDR, 0, bIrq);

	return true;
}

static void Stm32I2cDmaGive(Stm32I2cDev_t * const pDev)
{
	if (pDev->DmaTx >= 0)
	{
		WbaGpdmaFree(pDev->DmaTx);
		pDev->DmaTx = -1;
	}
	if (pDev->DmaRx >= 0)
	{
		WbaGpdmaFree(pDev->DmaRx);
		pDev->DmaRx = -1;
	}
}

static void Stm32I2cDmaIrq(int Ch, void *pCtx)
{
	Stm32I2cDev_t *dev = (Stm32I2cDev_t *)pCtx;

	(void)Ch;
	Stm32I2cService(dev);
}

static inline int Stm32I2cDmaCh(Stm32I2cDev_t * const pDev)
{
	return pDev->bRx ? pDev->DmaRx : pDev->DmaTx;
}
#endif

// ---------------------------------------------------------------------------
// Master engine
// ---------------------------------------------------------------------------

// Bytes done of a transfer that failed. A write counts the bytes the
// peripheral completed, from the NBYTES left in CR2. A read counts the bytes
// taken from RXDR, by the CPU or by the channel.
static int Stm32I2cMasterCount(Stm32I2cDev_t * const pDev)
{
	int n;

	if (pDev->bRx == false)
	{
		int left = (int)((pDev->pReg->CR2 & I2C_CR2_NBYTES_Msk) >> I2C_CR2_NBYTES_Pos);

		n = pDev->Done + pDev->Chunk - left;
	}
#ifdef STM32_I2C_DMA
	else if (pDev->bDma)
	{
		n = pDev->Done + pDev->Chunk - (int)WbaGpdmaRemain(pDev->DmaRx);
	}
#endif
	else
	{
		n = pDev->Moved;
	}

	return n < 0 ? 0 : (n > pDev->Len ? pDev->Len : n);
}

// End of the transfer. Interrupts and DMA off, flags cleared, the result
// stored. A write ended without STOP is held.
static void Stm32I2cMasterEnd(Stm32I2cDev_t * const pDev, bool bHold)
{
	I2C_TypeDef *reg = pDev->pReg;
	int count;
	bool bad = pDev->bNack || pDev->bFail;

	if (bad == false)
	{
		count = pDev->Len;
	}
	else
	{
		count = Stm32I2cMasterCount(pDev);
	}

	reg->CR1 &= ~(STM32_I2C_CR1_MASTER_INT | STM32_I2C_CR1_DATA_INT | STM32_I2C_CR1_DMA);
#ifdef STM32_I2C_DMA
	if (pDev->bDma)
	{
		int ch = Stm32I2cDmaCh(pDev);

		if (ch >= 0)
		{
			WbaGpdmaStop(ch);
		}
	}
#endif

	if (bHold && bad == false)
	{
		pDev->bHold = true;
	}
	else
	{
		Stm32I2cFlushTx(reg);
		reg->ICR = STM32_I2C_ICR_ALL;
		reg->CR2 = 0;
		pDev->bHold = false;
	}

	pDev->bError = bad;
	pDev->Result = count;
	pDev->bRun = false;
}

// Bus error, arbitration loss, timeout or DMA error: the transfer ends and
// the peripheral is reset.
static void Stm32I2cMasterFail(Stm32I2cDev_t * const pDev)
{
	pDev->bFail = true;
	Stm32I2cMasterEnd(pDev, false);
	Stm32I2cPeReset(pDev->pReg);
}

// Program the next chunk: the whole transfer up to 255 bytes, RELOAD when
// more follow. The first chunk starts with the START.
static void Stm32I2cMasterChunk(Stm32I2cDev_t * const pDev, bool bFirst)
{
	I2C_TypeDef *reg = pDev->pReg;
	int remain = pDev->Len - pDev->Done;
	bool last = remain <= STM32_I2C_NBYTES_MAX;
	int len = last ? remain : STM32_I2C_NBYTES_MAX;
	uint32_t cr2 = ((pDev->Addr << 1) & I2C_CR2_SADD_Msk) | ((uint32_t)len << I2C_CR2_NBYTES_Pos);

	pDev->Chunk = len;
	if (pDev->bRx)
	{
		cr2 |= I2C_CR2_RD_WRN;
	}
	if (last == false)
	{
		cr2 |= I2C_CR2_RELOAD;
	}
	else if (pDev->bStopEnd)
	{
		cr2 |= I2C_CR2_AUTOEND;
	}
	if (bFirst)
	{
		cr2 |= I2C_CR2_START;
	}

#ifdef STM32_I2C_DMA
	if (pDev->bDma)
	{
		if (pDev->bRx)
		{
			WbaGpdmaStart(pDev->DmaRx, pDev->pRx + pDev->Done, (uint32_t)len);
		}
		else
		{
			WbaGpdmaStart(pDev->DmaTx, pDev->pTx + pDev->Done, (uint32_t)len);
		}
	}
#endif
	reg->CR2 = cr2;
}

// A refused address or byte. With AUTOEND the STOP follows, a transfer in
// chunks or ended by software needs it. Returns the flags read again.
static uint32_t Stm32I2cMasterNack(Stm32I2cDev_t * const pDev, uint32_t Isr)
{
	I2C_TypeDef *reg = pDev->pReg;

	if ((Isr & I2C_ISR_NACKF) && pDev->bNack == false)
	{
		pDev->bNack = true;
		reg->ICR = I2C_ICR_NACKCF;
		if ((reg->CR2 & I2C_CR2_AUTOEND) == 0U)
		{
			reg->CR2 |= I2C_CR2_STOP;
		}
		Isr = reg->ISR;
	}

	return Isr;
}

// One step of the transfer from the flags. Called by the polling loop and by
// the interrupt.
static void Stm32I2cMasterService(Stm32I2cDev_t * const pDev)
{
	I2C_TypeDef *reg = pDev->pReg;
	uint32_t isr = reg->ISR;

#ifdef STM32_I2C_DMA
	if (pDev->bDma && (WbaGpdmaFlags(Stm32I2cDmaCh(pDev)) & WBA_GPDMA_FLAG_ERR))
	{
		Stm32I2cMasterFail(pDev);

		return;
	}
#endif

	if (isr & STM32_I2C_ERR_FLAGS)
	{
		Stm32I2cMasterFail(pDev);

		return;
	}

	isr = Stm32I2cMasterNack(pDev, isr);

	if (pDev->bNack == false && pDev->bDma == false)
	{
		if (pDev->bRx)
		{
			if ((isr & I2C_ISR_RXNE) && pDev->Moved < pDev->Len)
			{
				pDev->pRx[pDev->Moved] = (uint8_t)reg->RXDR;
				pDev->Moved++;
				isr = reg->ISR;
			}
		}
		else if ((isr & I2C_ISR_TXIS) && pDev->Moved < pDev->Len)
		{
			reg->TXDR = pDev->pTx[pDev->Moved];
			pDev->Moved++;
			isr = Stm32I2cMasterNack(pDev, reg->ISR);
		}
	}

	if (isr & I2C_ISR_STOPF)
	{
#ifdef STM32_I2C_DMA
		// The channel finishes its last transfer after the STOP
		if (pDev->bDma && pDev->bNack == false &&
			(WbaGpdmaFlags(Stm32I2cDmaCh(pDev)) & WBA_GPDMA_FLAG_TC) == 0U)
		{
			return;
		}
#endif
		Stm32I2cMasterEnd(pDev, false);

		return;
	}

	if (pDev->bNack)
	{
		return;
	}

	if (isr & I2C_ISR_TCR)
	{
#ifdef STM32_I2C_DMA
		if (pDev->bDma && (WbaGpdmaFlags(Stm32I2cDmaCh(pDev)) & WBA_GPDMA_FLAG_TC) == 0U)
		{
			return;
		}
#endif
		pDev->Done += pDev->Chunk;
		Stm32I2cMasterChunk(pDev, false);
	}
	else if (isr & I2C_ISR_TC)
	{
#ifdef STM32_I2C_DMA
		if (pDev->bDma && (WbaGpdmaFlags(Stm32I2cDmaCh(pDev)) & WBA_GPDMA_FLAG_TC) == 0U)
		{
			return;
		}
#endif
		// Last chunk of a write held without STOP
		Stm32I2cMasterEnd(pDev, true);
	}
}

// Open the transfer: START, or the repeated START when a write is held.
static bool Stm32I2cMasterBegin(Stm32I2cDev_t * const pDev, bool bRx, bool bStop, bool bAsync)
{
	I2C_TypeDef *reg = pDev->pReg;
	uint32_t cr1;

	pDev->bRx = bRx;
	pDev->bStopEnd = bStop;
	pDev->bAsync = bAsync;
	pDev->bNack = false;
	pDev->bFail = false;
	pDev->bError = false;
	pDev->Done = 0;
	pDev->Chunk = 0;
	pDev->Moved = 0;
	pDev->Result = 0;

	// The data moves by DMA when the channels are held
#ifdef STM32_I2C_DMA
	pDev->bDma = pDev->pI2cDev->DevIntrf.bDma && pDev->DmaTx >= 0 && pDev->DmaRx >= 0;
#else
	pDev->bDma = false;
#endif

	reg->ICR = STM32_I2C_ICR_ALL;
	cr1 = reg->CR1 & ~(STM32_I2C_CR1_MASTER_INT | STM32_I2C_CR1_DATA_INT | STM32_I2C_CR1_DMA);
	if (pDev->bDma)
	{
		cr1 |= bRx ? I2C_CR1_RXDMAEN : I2C_CR1_TXDMAEN;
	}
	if (bAsync)
	{
		cr1 |= STM32_I2C_CR1_MASTER_INT;
		if (pDev->bDma == false)
		{
			cr1 |= bRx ? I2C_CR1_RXIE : I2C_CR1_TXIE;
		}
	}
	reg->CR1 = cr1;

	pDev->bRun = true;
	pDev->bHold = false;
	Stm32I2cMasterChunk(pDev, true);

	return true;
}

// Polling loop of the engine. The wait restarts whenever the transfer
// progresses.
static void Stm32I2cMasterRun(Stm32I2cDev_t * const pDev)
{
	const uint32_t budget = Stm32I2cTimeout(pDev);
	uint32_t left = budget;
	int last = -1;

	while (pDev->bRun)
	{
		Stm32I2cMasterService(pDev);

		int sig = pDev->Moved + pDev->Done * 4 + (pDev->bNack ? 1 << 20 : 0);

		if (sig != last)
		{
			last = sig;
			left = budget;
		}
		else if (--left == 0)
		{
			Stm32I2cMasterFail(pDev);
		}
	}
}

// Take the finished transfer result
static int Stm32I2cMasterResult(Stm32I2cDev_t * const pDev)
{
	return pDev->Result;
}

// An interrupt driven transfer ended
static void Stm32I2cAsyncDone(Stm32I2cDev_t * const pDev)
{
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;
	const bool rx = pDev->bRx;
	const int count = pDev->Result;

	pDev->bAsync = false;
	if (rx)
	{
		DeviceIntrfStopRx(intrf);
	}
	else
	{
		DeviceIntrfTxComplete(intrf);
	}
	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, NULL, count);
	}
}

// ---------------------------------------------------------------------------
// Slave engine
// ---------------------------------------------------------------------------

static void Stm32I2cSlaveEvent(Stm32I2cDev_t * const pDev, DEVINTRF_EVT Evt, int Len)
{
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, Evt, NULL, Len);
	}
}

// The slave is asked for the next byte as soon as the previous one is in the
// shift register, so a read that the master ends with a NACK always has one
// byte loaded that was never sent. It is counted out so that the count is the
// number of bytes on the bus. A load of filler is not in the count.
static int Stm32I2cSlaveSent(Stm32I2cDev_t * const pDev)
{
	int count = pDev->SCount;

	if (pDev->SState == STM32_I2C_S_READ && pDev->bTxData && count > 0)
	{
		count--;
	}

	return count;
}

// Address matched: the direction and the address index. A repeated START
// comes here without a STOP before it.
static void Stm32I2cSlaveAddress(Stm32I2cDev_t * const pDev, uint32_t Isr)
{
	I2C_TypeDef *reg = pDev->pReg;
	I2CDev_t *i2c = pDev->pI2cDev;
	const uint32_t code = (Isr & I2C_ISR_ADDCODE_Msk) >> I2C_ISR_ADDCODE_Pos;
	const bool read = (Isr & I2C_ISR_DIR) != 0U;
	int idx = 0;
	int prior = 0;

	for (int i = 0; i < i2c->Cfg.NbSlaveAddr && i < STM32_I2C_SLAVE_MAX_ADDR; i++)
	{
		if (i2c->Cfg.SlaveAddr[i] == code)
		{
			idx = i;
			break;
		}
	}

	if (pDev->SState == STM32_I2C_S_WRITE)
	{
		prior = pDev->SCount;
	}
	else if (pDev->SState == STM32_I2C_S_READ)
	{
		Stm32I2cSlaveEvent(pDev, DEVINTRF_EVT_COMPLETED, Stm32I2cSlaveSent(pDev));
	}

	pDev->SIdx = idx;
	pDev->SCount = 0;
	pDev->SState = read ? STM32_I2C_S_READ : STM32_I2C_S_WRITE;

	// The application installs the buffer again for every request
	if (read)
	{
		i2c->pRRData[idx] = NULL;
		i2c->RRDataLen[idx] = 0;
		Stm32I2cSlaveEvent(pDev, DEVINTRF_EVT_READ_RQST, prior);
	}
	else
	{
		i2c->pTRBuff[idx] = NULL;
		i2c->TRBuffLen[idx] = 0;
		if (prior > 0)
		{
			Stm32I2cSlaveEvent(pDev, DEVINTRF_EVT_COMPLETED, prior);
		}
		Stm32I2cSlaveEvent(pDev, DEVINTRF_EVT_WRITE_RQST, 0);
	}

	// SCL is released
	reg->ICR = I2C_ICR_ADDRCF;
}

static void Stm32I2cSlaveStop(Stm32I2cDev_t * const pDev)
{
	I2C_TypeDef *reg = pDev->pReg;
	const int count = Stm32I2cSlaveSent(pDev);
	const bool active = pDev->SState != STM32_I2C_S_IDLE;

	reg->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
	Stm32I2cFlushTx(reg);
	pDev->SState = STM32_I2C_S_IDLE;
	pDev->SCount = 0;
	if (active)
	{
		Stm32I2cSlaveEvent(pDev, DEVINTRF_EVT_COMPLETED, count);
	}
}

static void Stm32I2cSlaveService(Stm32I2cDev_t * const pDev)
{
	I2C_TypeDef *reg = pDev->pReg;
	I2CDev_t *i2c = pDev->pI2cDev;
	uint32_t isr = reg->ISR;

	if (isr & (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_TIMEOUT))
	{
		reg->ICR = I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF | I2C_ICR_TIMOUTCF;
		isr = reg->ISR;
	}

	// The last byte of a write is read before a repeated START is handled
	if ((isr & I2C_ISR_RXNE) != 0U)
	{
		uint8_t value = (uint8_t)reg->RXDR;

		if (pDev->SState == STM32_I2C_S_WRITE)
		{
			uint8_t *buf = i2c->pTRBuff[pDev->SIdx];

			if (buf != NULL && pDev->SCount < i2c->TRBuffLen[pDev->SIdx])
			{
				buf[pDev->SCount++] = value;
			}
		}
		isr = reg->ISR;
	}

	if (isr & I2C_ISR_ADDR)
	{
		Stm32I2cSlaveAddress(pDev, isr);
		isr = reg->ISR;
	}

	if ((isr & I2C_ISR_TXIS) != 0U && pDev->SState == STM32_I2C_S_READ)
	{
		const uint8_t *data = i2c->pRRData[pDev->SIdx];

		if (data != NULL && pDev->SCount < i2c->RRDataLen[pDev->SIdx])
		{
			reg->TXDR = data[pDev->SCount++];
			pDev->bTxData = true;
		}
		else
		{
			reg->TXDR = 0xFF;
			pDev->bTxData = false;
		}
		isr = reg->ISR;
	}

	if (isr & I2C_ISR_NACKF)
	{
		// The master ends a read with a NACK, the STOP follows
		reg->ICR = I2C_ICR_NACKCF;
		isr = reg->ISR;
	}

	if (isr & I2C_ISR_STOPF)
	{
		Stm32I2cSlaveStop(pDev);
	}
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

static void Stm32I2cService(Stm32I2cDev_t * const pDev)
{
	if (pDev->pI2cDev == NULL)
	{
		return;
	}

	if (pDev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		Stm32I2cSlaveService(pDev);

		return;
	}

	if (pDev->bRun == false)
	{
		return;
	}

	Stm32I2cMasterService(pDev);
	if (pDev->bRun == false && pDev->bAsync)
	{
		Stm32I2cAsyncDone(pDev);
	}
}

// ---------------------------------------------------------------------------
// Device interface
// ---------------------------------------------------------------------------

static void Stm32I2cReset(DevIntrf_t * const pIntrf)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	dev->pReg->CR1 &= ~(STM32_I2C_CR1_MASTER_INT | STM32_I2C_CR1_DATA_INT | STM32_I2C_CR1_DMA);
	Stm32I2cPeReset(dev->pReg);
#ifdef STM32_I2C_DMA
	if (dev->DmaTx >= 0)
	{
		WbaGpdmaStop(dev->DmaTx);
	}
	if (dev->DmaRx >= 0)
	{
		WbaGpdmaStop(dev->DmaRx);
	}
#endif
	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		dev->pReg->CR1 |= STM32_I2C_CR1_SLAVE_INT;
	}
	dev->bActive = false;
	dev->bHold = false;
	dev->bError = false;
	dev->bRun = false;
	dev->bAsync = false;
	dev->SState = STM32_I2C_S_IDLE;
	dev->SCount = 0;
	atomic_store(&pIntrf->bTxReady, true);
}

static void Stm32I2cEnable(DevIntrf_t * const pIntrf)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	if (dev)
	{
		Stm32I2cClkEnable(dev, true);
		dev->pReg->CR1 |= I2C_CR1_PE;
	}
}

static void Stm32I2cDisable(DevIntrf_t * const pIntrf)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	if (dev)
	{
		dev->pReg->CR1 &= ~I2C_CR1_PE;
		Stm32I2cClkEnable(dev, false);
	}
}

static void Stm32I2cPowerOff(DevIntrf_t * const pIntrf)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	if (dev)
	{
		dev->pReg->CR1 &= ~I2C_CR1_PE;
		NVIC_DisableIRQ(dev->pInfo->EvIrq);
		NVIC_DisableIRQ(dev->pInfo->ErIrq);
#ifdef STM32_I2C_DMA
		Stm32I2cDmaGive(dev);
#endif
		Stm32I2cClkEnable(dev, false);
		IOPinDis(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);
	}
}

static void *Stm32I2cGetHandle(DevIntrf_t * const pIntrf)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	return dev != NULL ? dev->pI2cDev : NULL;
}

// Open a transfer. Common checks: the interface is enabled and the bus is
// free, or held by a write that a read follows with a repeated START.
static bool Stm32I2cStart(DevIntrf_t * const pIntrf, uint32_t DevAddr, bool bRx)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	if (dev == NULL || dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER || DevAddr > 0x7FU ||
		!(dev->pReg->CR1 & I2C_CR1_PE) || dev->bRun)
	{
		return false;
	}

	if (dev->bHold && !bRx)
	{
		// A new write cannot follow a held write: end the held one first
		dev->pReg->CR2 |= I2C_CR2_STOP;
		(void)Stm32I2cWait(dev, I2C_ISR_STOPF);
		dev->pReg->ICR = STM32_I2C_ICR_ALL;
		dev->pReg->CR2 = 0;
		dev->bHold = false;
	}
	if (!dev->bHold)
	{
		uint32_t n = Stm32I2cTimeout(dev);

		while ((dev->pReg->ISR & I2C_ISR_BUSY) && n--)
		{
		}
		if (dev->pReg->ISR & I2C_ISR_BUSY)
		{
			return false;
		}
	}

	dev->pReg->ICR = STM32_I2C_ICR_ALL;
	dev->Addr = DevAddr & 0x7F;
	dev->bRx = bRx;
	dev->bError = false;
	dev->bActive = true;

	return true;
}

// The hardware needs the number of bytes with the START, which is not known
// here. StartTx sends the address alone (NBYTES 0 with AUTOEND, the ST HAL
// device ready probe) and returns false when the device does not acknowledge
// it. TxData then starts the transfer of its data.
static bool Stm32I2cStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	if (Stm32I2cStart(pIntrf, DevAddr, false) == false)
	{
		return false;
	}

	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);
	I2C_TypeDef *reg = dev->pReg;

	reg->CR2 = ((dev->Addr << 1) & I2C_CR2_SADD_Msk) | I2C_CR2_AUTOEND | I2C_CR2_START;

	int res = Stm32I2cWait(dev, I2C_ISR_STOPF | I2C_ISR_NACKF);

	if (res == 1 && (reg->ISR & I2C_ISR_NACKF) && (reg->ISR & I2C_ISR_STOPF) == 0U)
	{
		// Refused. The STOP follows the NACK
		res = Stm32I2cWait(dev, I2C_ISR_STOPF);
		if (res == -1)
		{
			res = 1;
		}
	}
	if (res == -1 && (reg->ISR & I2C_ISR_NACKF))
	{
		res = Stm32I2cWait(dev, I2C_ISR_STOPF);
		res = res == -1 ? 1 : res;
	}
	if (res != 1)
	{
		// Bus error, arbitration loss, timeout or no end
		reg->CR2 |= I2C_CR2_STOP;
		Stm32I2cFlushTx(reg);
		reg->ICR = STM32_I2C_ICR_ALL;
		reg->CR2 = 0;
		Stm32I2cPeReset(reg);
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

static bool Stm32I2cStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	return Stm32I2cStart(pIntrf, DevAddr, true);
}

// Send Data. The whole data goes out in one transfer, in chunks of up to 255
// bytes. The last chunk ends with STOP, or, when the interface bNoStop is
// set, the bus is held after the last byte so that a following StartRx
// generates a repeated START. With bIntEn a transfer without bNoStop of up
// to I2C_STM32_TXBUF_SIZE bytes is copied, started and reported by the event.
static int Stm32I2cTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	if (dev == NULL || !dev->bActive || dev->bRx || dev->bError || dev->bRun ||
		pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	bool bNoStop = pIntrf->bNoStop;
	bool async = pIntrf->bIntEn && !bNoStop && DataLen <= I2C_STM32_TXBUF_SIZE;

	if (async)
	{
		memcpy(dev->TxBuf, pData, DataLen);
		dev->pTx = dev->TxBuf;
	}
	else
	{
		dev->pTx = pData;
	}
	dev->Len = DataLen;
	atomic_store(&pIntrf->bTxReady, false);

	Stm32I2cMasterBegin(dev, false, !bNoStop, async);
	if (async)
	{
		return -1;
	}

	Stm32I2cMasterRun(dev);
	atomic_store(&pIntrf->bTxReady, true);

	return Stm32I2cMasterResult(dev);
}

// Receive BuffLen bytes. Starts the transfer with START, or a repeated START
// when a write was held. The last chunk ends with NACK and STOP.
static int Stm32I2cRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	if (dev == NULL || !dev->bActive || !dev->bRx || dev->bError || dev->bRun ||
		pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	bool async = pIntrf->bIntEn;

	dev->pRx = pBuff;
	dev->Len = BuffLen;

	Stm32I2cMasterBegin(dev, true, true, async);
	if (async)
	{
		return -1;
	}

	Stm32I2cMasterRun(dev);

	return Stm32I2cMasterResult(dev);
}

// Close the transfer. A write held without STOP, with no read after it, is
// ended here.
static void Stm32I2cStop(DevIntrf_t * const pIntrf)
{
	Stm32I2cDev_t *dev = Stm32I2cHandle(pIntrf);

	if (dev == NULL || dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER || dev->bRun)
	{
		return;
	}

	if (dev->bHold)
	{
		dev->pReg->CR2 |= I2C_CR2_STOP;
		(void)Stm32I2cWait(dev, I2C_ISR_STOPF);
		dev->bHold = false;
	}
	dev->pReg->ICR = STM32_I2C_ICR_ALL;
	dev->pReg->CR2 = 0;
	dev->bActive = false;
}

// Slave data calls. The buffers are installed by the application from the
// request events.
void I2CSetReadRqstData(I2CDev_t * const pDev, int SlaveIdx, uint8_t * const pData, int DataLen)
{
	if (pDev == NULL || SlaveIdx < 0 || SlaveIdx >= I2C_SLAVEMODE_MAX_ADDR || DataLen < 0)
	{
		return;
	}

	pDev->pRRData[SlaveIdx] = pData;
	pDev->RRDataLen[SlaveIdx] = DataLen;
}

void I2CSetWriteRqstBuffer(I2CDev_t * const pDev, int SlaveIdx, uint8_t * const pBuff, int BuffLen)
{
	if (pDev == NULL || SlaveIdx < 0 || SlaveIdx >= I2C_SLAVEMODE_MAX_ADDR || BuffLen < 0)
	{
		return;
	}

	pDev->pTRBuff[SlaveIdx] = pBuff;
	pDev->TRBuffLen[SlaveIdx] = BuffLen;
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->DevNo < 0 || pCfgData->DevNo >= STM32_I2C_MAXDEV ||
		s_Stm32I2cInfo[pCfgData->DevNo].pReg == NULL)
	{
		return false;
	}

	if (pCfgData->AddrType != I2CADDR_TYPE_NORMAL || pCfgData->pIOPinMap == NULL ||
		pCfgData->NbIOPins < 2)
	{
		return false;
	}

	const bool slave = pCfgData->Mode == I2CMODE_SLAVE;

	if (pCfgData->Mode != I2CMODE_MASTER && slave == false)
	{
		return false;
	}
	if (slave)
	{
		if (pCfgData->NbSlaveAddr < 1 || pCfgData->NbSlaveAddr > STM32_I2C_SLAVE_MAX_ADDR)
		{
			return false;
		}
		for (int i = 0; i < pCfgData->NbSlaveAddr; i++)
		{
			if (pCfgData->SlaveAddr[i] > 0x7F)
			{
				return false;
			}
		}
	}

	Stm32I2cDev_t *dev = &s_Stm32I2cDev[pCfgData->DevNo];

	// An instance running for another I2C object is not taken over
	if (dev->pI2cDev != NULL && dev->pI2cDev != pDev && (dev->pReg->CR1 & I2C_CR1_PE))
	{
		return false;
	}

	dev->DevNo = pCfgData->DevNo;
	dev->pInfo = &s_Stm32I2cInfo[pCfgData->DevNo];
	dev->pReg = dev->pInfo->pReg;
	if (dev->pI2cDev == NULL)
	{
		dev->DmaTx = -1;
		dev->DmaRx = -1;
	}
#ifdef STM32_I2C_DMA
	else if (pCfgData->bDmaEn == false || pCfgData->Mode != I2CMODE_MASTER)
	{
		// Initialized again without DMA: the channels are given back
		Stm32I2cDmaGive(dev);
	}
#endif

	I2C_TypeDef *reg = dev->pReg;

	// DMA is for the master data, on the families with a channel table
	bool useDma = false;

	if (pCfgData->bDmaEn && slave == false)
	{
#ifdef STM32_I2C_DMA
		if (Stm32I2cDmaTake(dev, pCfgData->bIntEn, pCfgData->IntPrio) == false)
		{
			return false;
		}
		useDma = true;
#else
		return false;
#endif
	}

	memcpy(&pDev->Cfg, pCfgData, sizeof(I2CCfg_t));
	if (slave)
	{
		// The slave is driven by its interrupts
		pDev->Cfg.bIntEn = true;
		pDev->Cfg.bDmaEn = false;
	}
	memset(pDev->pRRData, 0, sizeof(pDev->pRRData));
	memset(pDev->RRDataLen, 0, sizeof(pDev->RRDataLen));
	memset(pDev->pTRBuff, 0, sizeof(pDev->pTRBuff));
	memset(pDev->TRBuffLen, 0, sizeof(pDev->TRBuffLen));

	dev->pI2cDev = pDev;
	dev->bActive = false;
	dev->bRx = false;
	dev->bHold = false;
	dev->bError = false;
	dev->bRun = false;
	dev->bAsync = false;
	dev->bNack = false;
	dev->bFail = false;
	dev->bDma = false;
	dev->SState = STM32_I2C_S_IDLE;
	dev->SCount = 0;
	dev->SIdx = 0;

	pDev->DevIntrf.pDevData = (void*)dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2C;
	pDev->DevIntrf.Reset = Stm32I2cReset;
	pDev->DevIntrf.Disable = Stm32I2cDisable;
	pDev->DevIntrf.Enable = Stm32I2cEnable;
	pDev->DevIntrf.PowerOff = Stm32I2cPowerOff;
	pDev->DevIntrf.GetHandle = Stm32I2cGetHandle;
	pDev->DevIntrf.GetRate = Stm32I2cGetRate;
	pDev->DevIntrf.SetRate = Stm32I2cSetRate;
	pDev->DevIntrf.StartRx = Stm32I2cStartRx;
	pDev->DevIntrf.RxData = Stm32I2cRxData;
	pDev->DevIntrf.StopRx = Stm32I2cStop;
	pDev->DevIntrf.StartTx = Stm32I2cStartTx;
	pDev->DevIntrf.TxData = Stm32I2cTxData;
	pDev->DevIntrf.TxSrData = Stm32I2cTxData;
	pDev->DevIntrf.StopTx = Stm32I2cStop;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = useDma;
	pDev->DevIntrf.bIntEn = pDev->Cfg.bIntEn;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	atomic_store(&pDev->DevIntrf.bTxReady, true);

	Stm32I2cClkEnable(dev, true);
	Stm32I2cRccReset(dev);

	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);
	for (int i = 0; i < pDev->Cfg.NbIOPins; i++)
	{
		IOPinSetSpeed(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	// Analog filter on, digital filter off, clock stretching on
	reg->CR1 = 0;
	reg->OAR1 = 0;
	reg->OAR2 = 0;
	reg->TIMEOUTR = 0;
#ifdef I2C_AUTOCR_TCDMAEN
	reg->AUTOCR = 0;
#endif

	Stm32I2cSetRate(&pDev->DevIntrf, pCfgData->Rate);

	if (slave)
	{
		reg->OAR1 = I2C_OAR1_OA1EN | ((uint32_t)pCfgData->SlaveAddr[0] << 1);
		if (pCfgData->NbSlaveAddr > 1)
		{
			reg->OAR2 = I2C_OAR2_OA2EN | ((uint32_t)pCfgData->SlaveAddr[1] << 1);
		}
	}
	else
	{
		Stm32I2cMasterTimeout(dev);
	}

	reg->ICR = STM32_I2C_ICR_ALL;

	if (pDev->Cfg.bIntEn)
	{
		NVIC_ClearPendingIRQ(dev->pInfo->EvIrq);
		NVIC_ClearPendingIRQ(dev->pInfo->ErIrq);
		NVIC_SetPriority(dev->pInfo->EvIrq, pCfgData->IntPrio);
		NVIC_SetPriority(dev->pInfo->ErIrq, pCfgData->IntPrio);
		NVIC_EnableIRQ(dev->pInfo->EvIrq);
		NVIC_EnableIRQ(dev->pInfo->ErIrq);
	}

	reg->CR1 |= I2C_CR1_PE;
	if (slave)
	{
		reg->CR1 |= STM32_I2C_CR1_SLAVE_INT;
	}

	return true;
}

// Interrupt vectors of the instances
extern "C" void I2C1_EV_IRQHandler(void)
{
	Stm32I2cService(&s_Stm32I2cDev[0]);
}

extern "C" void I2C1_ER_IRQHandler(void)
{
	Stm32I2cService(&s_Stm32I2cDev[0]);
}

#ifdef I2C2
extern "C" void I2C2_EV_IRQHandler(void)
{
	Stm32I2cService(&s_Stm32I2cDev[1]);
}

extern "C" void I2C2_ER_IRQHandler(void)
{
	Stm32I2cService(&s_Stm32I2cDev[1]);
}
#endif

#ifdef I2C3
extern "C" void I2C3_EV_IRQHandler(void)
{
	Stm32I2cService(&s_Stm32I2cDev[2]);
}

extern "C" void I2C3_ER_IRQHandler(void)
{
	Stm32I2cService(&s_Stm32I2cDev[2]);
}
#endif

#ifdef I2C4
extern "C" void I2C4_EV_IRQHandler(void)
{
	Stm32I2cService(&s_Stm32I2cDev[3]);
}

extern "C" void I2C4_ER_IRQHandler(void)
{
	Stm32I2cService(&s_Stm32I2cDev[3]);
}
#endif

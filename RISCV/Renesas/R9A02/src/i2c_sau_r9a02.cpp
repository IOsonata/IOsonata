/**-------------------------------------------------------------------------
@file	i2c_sau_r9a02.cpp

@brief	R9A02G021 I2C master on the serial array units (simplified I2C)

R9A02G021 User's Manual R01UH1036EJ0110, 23.8. One SAU channel per bus,
I2CInit DevNo 2 to 7:

	DevNo 2 : IIC00, SAU0 channel 0, SCL00, SDA00
	DevNo 3 : IIC01, SAU0 channel 1, SCL01, SDA01
	DevNo 4 : IIC10, SAU0 channel 2, SCL10, SDA10
	DevNo 5 : IIC11, SAU0 channel 3, SCL11, SDA11
	DevNo 6 : IIC20, SAU1 channel 0, SCL20, SDA20
	DevNo 7 : IIC21, SAU1 channel 1, SCL21, SDA21

A UART, SPI and I2C cannot use the same channel, the channels are claimed
through sau_r9a02.cpp. IIC00 on the 48-pin package uses P300 and P301, the
debug pins.

Single master, 7-bit addresses (23.1.3). The SAU makes the 8 data clocks and
the ACK clock of a byte, the software makes the start, restart and stop
conditions with the output bits SOm and SOEm (tables 23.109, 23.126). There
is no clock stretching during a byte, no arbitration and no bus busy
detection. SCL is checked high at the start, restart and stop, a slave may
stretch there. The manual gives no restart procedure (23.8.6): it is made as
a stop without the final SDA rise, SDA released and SCL high, then a start.

Transfer clock fSCL = fMCK / (2 x (STCLK + 1)), STCLK 1 to 127, duty 50 %
(23.8.5). SetRate takes the closest rate from 1 kHz to 1 MHz and at most
fMCK / 4 (table 37.35), with fMCK at least PCLKB / 2^11 (table 23.71 note 2).
At 400 kHz and 1 MHz the 50 % duty gives a low phase shorter than the bus
specification asks, the electrical limits of table 37.35 hold: 475 ns low at
1 MHz with VCC 2.7 V or more, 1150 ns at 400 kHz.

Transfer modes, as the IICA (i2c_r9a02.cpp): polling, or with bIntEn the
data in the SAUm_ENDIn interrupt. bDmaEn fails I2CInit. The stop condition
is then made in the interrupt, three half periods of SCL (up to 1.5 ms at 1
kHz), plus the time a slave holds SCL low. A transfer stopped by Reset or
Disable can leave the bus with a byte not ended: the next stop or start
first clocks SCL until SDA is free (figure 24.16 note 2 of the IICA).

Pins: PinOp R9A02_PSEL_SAU. SDA IOPINTYPE_OPENDRAIN (NCODR = 1, 16.5.1).
SCL IOPINTYPE_NORMAL, CMOS as the note of table 37.35, or IOPINTYPE_OPENDRAIN as
23.8. External pull-ups. The noise filter of the channel pair is turned off
(23.3.27).

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
#include <stddef.h>

#include "r9a02g021.h"
#include "coredev/i2c.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "sau_r9a02.h"
#include "i2c_r9a02.h"

#define R9A02_I2C_SAU_RATE_MIN		1000UL		// fMCK with PRS 11 or less
#define R9A02_I2C_SAU_RATE_MAX		1000000UL	// Table 37.35
#define R9A02_I2C_SAU_PRS_MAX		11U			// Table 23.71 note 2
#define R9A02_I2C_SAU_DIV_MIN		2U			// STCLK 0 prohibited (23.3.8)
#define R9A02_I2C_SAU_RECOVER_CLK	9			// SCL pulses to free SDA

// Mode and format, tables 23.102 and 23.103: 8 bits, ACK slot as the stop
// bit, MSB first, TRXE changed between transmission and reception
#define R9A02_I2C_SAU_SMR			(R9A02_SAU_SMR_FIXED | R9A02_SAU_SMR_MD_I2C)
#define R9A02_I2C_SAU_SCR_TX		(R9A02_SAU_SCR_TRXE_TX | R9A02_SAU_SCR_SLC_1BIT | R9A02_SAU_SCR_DLS_8BIT)
#define R9A02_I2C_SAU_SCR_RX		(R9A02_SAU_SCR_TRXE_RX | R9A02_SAU_SCR_SLC_1BIT | R9A02_SAU_SCR_DLS_8BIT)

#pragma pack(push, 4)
typedef struct __R9a02_I2c_Sau_Dev {
	uint8_t Unit;					// SAU unit
	uint8_t Ch;						// SAU channel
	uint8_t EvtEnd;					// SAUm_ENDIn
	uint8_t Snfen;					// SNFEN bit of the channel pair
	I2CDev_t *pI2cDev;				// Generic I2C device data
	int IntNo;						// IELSR route, -1 when none
	uint32_t Loops;					// Poll loops allowed for one byte
	uint32_t HalfUs;				// Half SCL period in us, at least 1
	bool bEnabled;
	bool bOpen;						// Transaction started
	bool bActive;					// Channel running, SCL low after a byte
	bool bRx;						// Receive direction
	bool bNacked;					// Last byte received and not acknowledged
	bool bError;
	bool bLost;						// Byte not ended, bus recovery before the next use
	volatile bool bAsync;			// Data in the background
	uint8_t *pRxBuf;				// Background reception
	int XferLen;					// Background transfer length
	volatile int XferCnt;
	uint8_t TxBuf[R9A02_I2C_TX_COPY_SIZE];	// Background transmission
} R9a02I2cSauDev_t;
#pragma pack(pop)

static R9a02I2cSauDev_t s_R9a02I2cSauDev[] = {
	{ 0, 0, R9A02_EVT_SAU0_ENDI0, R9A02_SNFEN_SNFEN00, NULL, -1, 0, 1, false, false, false, false, false, false,
	  false, false, NULL, 0, 0, {} },
	{ 0, 1, R9A02_EVT_SAU0_ENDI1, R9A02_SNFEN_SNFEN00, NULL, -1, 0, 1, false, false, false, false, false, false,
	  false, false, NULL, 0, 0, {} },
	{ 0, 2, R9A02_EVT_SAU0_ENDI2, R9A02_SNFEN_SNFEN10, NULL, -1, 0, 1, false, false, false, false, false, false,
	  false, false, NULL, 0, 0, {} },
	{ 0, 3, R9A02_EVT_SAU0_ENDI3, R9A02_SNFEN_SNFEN10, NULL, -1, 0, 1, false, false, false, false, false, false,
	  false, false, NULL, 0, 0, {} },
	{ 1, 0, R9A02_EVT_SAU1_ENDI0, R9A02_SNFEN_SNFEN20, NULL, -1, 0, 1, false, false, false, false, false, false,
	  false, false, NULL, 0, 0, {} },
	{ 1, 1, R9A02_EVT_SAU1_ENDI1, R9A02_SNFEN_SNFEN20, NULL, -1, 0, 1, false, false, false, false, false, false,
	  false, false, NULL, 0, 0, {} },
};

static const int s_NbR9a02I2cSauDev = sizeof(s_R9a02I2cSauDev) / sizeof(s_R9a02I2cSauDev[0]);

static R9a02I2cSauDev_t *R9a02I2cSauHandle(DevIntrf_t * const pIntrf)
{
	R9a02I2cSauDev_t *dev = pIntrf ? (R9a02I2cSauDev_t *)pIntrf->pDevData : NULL;

	return dev && dev->pI2cDev && &dev->pI2cDev->DevIntrf == pIntrf ? dev : NULL;
}

// SDA level when the serial output is off (SOE = 0). SOm is common to the
// unit, the bits of the other channels are kept.
static void R9a02I2cSauSda(R9a02I2cSauDev_t * const pDev, bool bHigh)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);
	uint16_t bit = (uint16_t)(1U << pDev->Ch);
	uintptr_t state = DisableInterrupt();

	reg->SO = bHigh ? (uint16_t)(reg->SO | bit) : (uint16_t)(reg->SO & ~bit);

	EnableInterrupt(state);
}

// SCL level when the channel is stopped (SE = 0)
static void R9a02I2cSauScl(R9a02I2cSauDev_t * const pDev, bool bHigh)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);
	uint16_t cko = (uint16_t)R9A02_SAU_SO_CKO(pDev->Ch);
	uintptr_t state = DisableInterrupt();

	reg->SO = bHigh ? (uint16_t)(reg->SO | cko) : (uint16_t)(reg->SO & ~cko);

	EnableInterrupt(state);
}

// Serial output on: SDA follows the shift register and the ACK
static void R9a02I2cSauOutput(R9a02I2cSauDev_t * const pDev, bool bOn)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);
	uint16_t bit = (uint16_t)(1U << pDev->Ch);
	uintptr_t state = DisableInterrupt();

	reg->SOE = bOn ? (uint16_t)(reg->SOE | bit) : (uint16_t)(reg->SOE & ~bit);

	EnableInterrupt(state);
}

// SCL high on the pin, a slave may hold it low
static bool R9a02I2cSauSclHigh(R9a02I2cSauDev_t * const pDev)
{
	const IOPinCfg_t *scl = &pDev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX];
	uint32_t n = pDev->Loops;

	while (!IOPinRead(scl->PortNo, scl->PinNo))
	{
		if (--n == 0U)
		{
			return false;
		}
	}

	return true;
}

static bool R9a02I2cSauBusIdle(R9a02I2cSauDev_t * const pDev)
{
	const IOPinCfg_t *pins = pDev->pI2cDev->Cfg.pIOPinMap;

	return IOPinRead(pins[I2C_SDA_IOPIN_IDX].PortNo, pins[I2C_SDA_IOPIN_IDX].PinNo) &&
		   IOPinRead(pins[I2C_SCL_IOPIN_IDX].PortNo, pins[I2C_SCL_IOPIN_IDX].PinNo);
}

static bool R9a02I2cSauWaitEvent(R9a02I2cSauDev_t * const pDev)
{
	uint32_t n = pDev->Loops;

	while (!R9a02EventFlag(pDev->IntNo))
	{
		if (--n == 0U)
		{
			pDev->bError = true;

			return false;
		}
	}

	return true;
}

// Channel stopped, serial output off, SDA and SCL released, the error flags
// cleared (table 23.108)
static void R9a02I2cSauIdle(R9a02I2cSauDev_t * const pDev)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);
	uint16_t bit = (uint16_t)(1U << pDev->Ch);
	uintptr_t state = DisableInterrupt();

	reg->ST = bit;
	reg->SOE = (uint16_t)(reg->SOE & ~bit);
	reg->SO = (uint16_t)(reg->SO | bit | R9A02_SAU_SO_CKO(pDev->Ch));
	reg->SIR[pDev->Ch] = R9A02_SAU_SIR_PECT | R9A02_SAU_SIR_OVCT;
	if (pDev->IntNo >= 0)
	{
		R9a02EventFlagClear(pDev->IntNo);
	}
	pDev->bActive = false;
	pDev->bNacked = false;

	EnableInterrupt(state);
}

// Nine SCL pulses until the slave releases SDA, then a stop condition (the
// IICA procedure of figure 24.16 note 2, with the SO and CKO bits). Returns
// true when both lines are high.
static bool R9a02I2cSauBusRecover(R9a02I2cSauDev_t * const pDev)
{
	const IOPinCfg_t *sda = &pDev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX];

	R9a02I2cSauIdle(pDev);

	bool high = R9a02I2cSauSclHigh(pDev);

	for (int i = 0; high && i < R9A02_I2C_SAU_RECOVER_CLK && !IOPinRead(sda->PortNo, sda->PinNo); i++)
	{
		R9a02I2cSauScl(pDev, false);
		R9a02DelayUs(pDev->HalfUs);
		R9a02I2cSauScl(pDev, true);
		high = R9a02I2cSauSclHigh(pDev);
		R9a02DelayUs(pDev->HalfUs);
	}

	if (high)
	{
		R9a02I2cSauScl(pDev, false);
		R9a02I2cSauSda(pDev, false);
		R9a02DelayUs(pDev->HalfUs);
		R9a02I2cSauScl(pDev, true);
		if (R9a02I2cSauSclHigh(pDev))
		{
			R9a02DelayUs(pDev->HalfUs);
		}
	}

	R9a02I2cSauSda(pDev, true);
	R9a02I2cSauScl(pDev, true);
	R9a02DelayUs(pDev->HalfUs);

	return R9a02I2cSauBusIdle(pDev);
}

// Master reception: the last byte before a stop or a restart is not
// acknowledged. When none was read, one byte is read with the serial output
// off, no ACK, and dropped.
static void R9a02I2cSauRxEnd(R9a02I2cSauDev_t * const pDev)
{
	if (!pDev->bActive || !pDev->bRx || pDev->bNacked)
	{
		return;
	}

	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);

	R9a02I2cSauOutput(pDev, false);
	R9a02EventFlagClear(pDev->IntNo);
	reg->SDR[pDev->Ch] = 0xFFU;

	if (R9a02I2cSauWaitEvent(pDev))
	{
		(void)reg->SDR[pDev->Ch];
		reg->SIR[pDev->Ch] = R9A02_SAU_SIR_OVCT;
	}
	pDev->bNacked = true;
}

// Stop condition (table 23.126): channel stopped with SCL low, SDA low, SCL
// high, then SDA high. A byte that did not end is cleared by the bus
// recovery, which makes the stop.
static void R9a02I2cSauFinish(R9a02I2cSauDev_t * const pDev)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);

	if (pDev->bLost)
	{
		R9a02I2cSauBusRecover(pDev);
		pDev->bLost = false;

		return;
	}

	R9a02I2cSauRxEnd(pDev);

	uintptr_t state = DisableInterrupt();

	reg->ST = (uint16_t)(1U << pDev->Ch);
	R9a02I2cSauOutput(pDev, false);
	R9a02I2cSauSda(pDev, false);

	EnableInterrupt(state);

	R9a02DelayUs(pDev->HalfUs);
	R9a02I2cSauScl(pDev, true);
	if (!R9a02I2cSauSclHigh(pDev))
	{
		pDev->bError = true;
	}
	R9a02DelayUs(pDev->HalfUs);
	R9a02I2cSauSda(pDev, true);

	// Bus free time before a new start
	R9a02DelayUs(pDev->HalfUs);

	R9a02I2cSauIdle(pDev);
}

// Background data stopped without event, the route kept. Returns true when
// a transfer was running.
static bool R9a02I2cSauCancel(R9a02I2cSauDev_t * const pDev)
{
	uintptr_t state = DisableInterrupt();
	bool running = pDev->bAsync;

	if (running)
	{
		R9a02IntEnable(pDev->IntNo, false);
		R9a02ClearPendingInt(pDev->IntNo);
		pDev->bAsync = false;
		pDev->bError = true;
		pDev->bLost = true;
	}

	EnableInterrupt(state);

	return running;
}

static void R9a02I2cSauStop(DevIntrf_t * const pIntrf)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL || !dev->bOpen)
	{
		return;
	}

	R9a02I2cSauCancel(dev);
	R9a02I2cSauFinish(dev);
	dev->bOpen = false;
	dev->bRx = false;
	dev->bError = false;
	pIntrf->bTxReady = true;
}

// Start, or restart in a transaction, and the address byte (table 23.109)
static bool R9a02I2cSauStart(DevIntrf_t * const pIntrf, uint32_t Addr, bool bRx)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL || !dev->bEnabled || Addr > 0x7FU || (dev->bOpen && (dev->bError || dev->bAsync)))
	{
		if (dev && dev->bOpen)
		{
			dev->bError = true;
		}

		return false;
	}

	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);
	uint16_t bit = (uint16_t)(1U << dev->Ch);
	bool wasopen = dev->bOpen;

	if (wasopen)
	{
		// Restart: the last received byte not acknowledged, channel stopped
		// with SCL low, SDA released, then SCL high
		R9a02I2cSauRxEnd(dev);

		uintptr_t state = DisableInterrupt();

		reg->ST = bit;
		R9a02I2cSauOutput(dev, false);
		R9a02I2cSauSda(dev, true);

		EnableInterrupt(state);

		R9a02DelayUs(dev->HalfUs);
		R9a02I2cSauScl(dev, true);
		if (!R9a02I2cSauSclHigh(dev))
		{
			dev->bError = true;

			return false;
		}
		R9a02DelayUs(dev->HalfUs);
	}
	else
	{
		// A bus held low is cleared first
		if (!R9a02I2cSauBusIdle(dev) && !R9a02I2cSauBusRecover(dev))
		{
			return false;
		}
		dev->bError = false;
	}

	dev->bOpen = true;
	dev->bRx = bRx;
	dev->bNacked = false;

	// SDA falls with SCL high, then SCL low after the hold time
	R9a02I2cSauSda(dev, false);
	R9a02DelayUs(dev->HalfUs);

	uintptr_t state = DisableInterrupt();

	R9a02I2cSauScl(dev, false);
	reg->SCR[dev->Ch] = R9A02_I2C_SAU_SCR_TX;
	reg->SIR[dev->Ch] = R9A02_SAU_SIR_PECT | R9A02_SAU_SIR_OVCT;
	R9a02I2cSauOutput(dev, true);
	reg->SS = bit;
	R9a02EventFlagClear(dev->IntNo);
	reg->SDR[dev->Ch] = (uint16_t)((Addr << 1) | (bRx ? 1U : 0U));

	EnableInterrupt(state);

	bool ok = R9a02I2cSauWaitEvent(dev);

	if (ok)
	{
		dev->bActive = true;

		// PEF is the NACK of the address
		if (reg->SSR[dev->Ch] & R9A02_SAU_SSR_PEF)
		{
			reg->SIR[dev->Ch] = R9A02_SAU_SIR_PECT;
			ok = false;
		}
	}

	if (ok && bRx)
	{
		// TRXE changed with the channel stopped (table 23.125)
		state = DisableInterrupt();

		reg->ST = bit;
		reg->SCR[dev->Ch] = R9A02_I2C_SAU_SCR_RX;
		reg->SS = bit;

		EnableInterrupt(state);
	}

	if (!ok)
	{
		dev->bError = true;
		if (!wasopen)
		{
			// Not seen by StopRx or StopTx when the start fails, the bus is
			// released here
			R9a02I2cSauFinish(dev);
			dev->bOpen = false;
			dev->bRx = false;
			dev->bError = false;
		}
	}

	return ok;
}

static bool R9a02I2cSauStartTx(DevIntrf_t * const pIntrf, uint32_t Addr)
{
	return R9a02I2cSauStart(pIntrf, Addr, false);
}

static bool R9a02I2cSauStartRx(DevIntrf_t * const pIntrf, uint32_t Addr)
{
	return R9a02I2cSauStart(pIntrf, Addr, true);
}

// Data transmission (table 23.117). A byte not acknowledged (PEF) ends the
// transfer, it is not counted.
static int R9a02I2cSauTxPoll(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL || !dev->bOpen || !dev->bActive || dev->bRx || dev->bError || dev->bAsync ||
		pData == NULL || Len <= 0)
	{
		return 0;
	}

	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);
	int cnt = 0;

	pIntrf->bTxReady = false;

	while (cnt < Len)
	{
		R9a02EventFlagClear(dev->IntNo);
		reg->SDR[dev->Ch] = pData[cnt];

		if (!R9a02I2cSauWaitEvent(dev))
		{
			break;
		}

		if (reg->SSR[dev->Ch] & R9A02_SAU_SSR_PEF)
		{
			reg->SIR[dev->Ch] = R9A02_SAU_SIR_PECT;
			dev->bError = true;
			break;
		}
		cnt++;
	}

	pIntrf->bTxReady = true;

	return cnt;
}

// Data reception (table 23.125): a dummy 0xFF starts each byte, the serial
// output is turned off before the last one so that it is not acknowledged
static int R9a02I2cSauRxPoll(DevIntrf_t * const pIntrf, uint8_t *pBuff, int Len)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL || !dev->bOpen || !dev->bActive || !dev->bRx || dev->bError || dev->bNacked ||
		dev->bAsync || pBuff == NULL || Len <= 0)
	{
		return 0;
	}

	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);
	int cnt = 0;

	while (cnt < Len)
	{
		if (cnt + 1 == Len)
		{
			R9a02I2cSauOutput(dev, false);
		}

		R9a02EventFlagClear(dev->IntNo);
		reg->SDR[dev->Ch] = 0xFFU;

		if (!R9a02I2cSauWaitEvent(dev))
		{
			break;
		}

		uint16_t ssr = reg->SSR[dev->Ch];
		uint8_t d = (uint8_t)reg->SDR[dev->Ch];

		if (ssr & R9A02_SAU_SSR_OVF)
		{
			reg->SIR[dev->Ch] = R9A02_SAU_SIR_OVCT;
			dev->bError = true;
			break;
		}
		pBuff[cnt++] = d;
	}

	dev->bNacked = cnt == Len;

	return cnt;
}

// End of background data: stop condition and release, then the event
static void R9a02I2cSauAsyncEnd(R9a02I2cSauDev_t * const pDev)
{
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;
	bool rx = pDev->bRx;
	int cnt = pDev->XferCnt;

	// Polled from here: a stop after a reception without NACK reads a byte
	R9a02IntEnable(pDev->IntNo, false);
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
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, rx ? pDev->pRxBuf : NULL, cnt);
	}
}

// SAUm_ENDIn of a byte in the background, after its ACK clock
static void R9a02I2cSauIrqHandler(int IntNo, void *pCtx)
{
	R9a02I2cSauDev_t *dev = (R9a02I2cSauDev_t *)pCtx;

	if (!dev->bAsync || dev->pI2cDev == NULL)
	{
		return;
	}

	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);
	uint32_t ch = dev->Ch;
	uint16_t ssr = reg->SSR[ch];

	if (dev->bRx)
	{
		uint8_t d = (uint8_t)reg->SDR[ch];

		if (ssr & R9A02_SAU_SSR_OVF)
		{
			reg->SIR[ch] = R9A02_SAU_SIR_OVCT;
			dev->bError = true;
		}
		else
		{
			dev->pRxBuf[dev->XferCnt] = d;
			dev->XferCnt = dev->XferCnt + 1;

			if (dev->XferCnt < dev->XferLen)
			{
				if (dev->XferCnt + 1 == dev->XferLen)
				{
					R9a02I2cSauOutput(dev, false);
				}
				reg->SDR[ch] = 0xFFU;

				return;
			}
			dev->bNacked = true;
		}
	}
	else
	{
		if (ssr & R9A02_SAU_SSR_PEF)
		{
			reg->SIR[ch] = R9A02_SAU_SIR_PECT;
			dev->bError = true;
		}
		else
		{
			dev->XferCnt = dev->XferCnt + 1;

			if (dev->XferCnt < dev->XferLen)
			{
				reg->SDR[ch] = dev->TxBuf[dev->XferCnt];

				return;
			}
		}
	}

	R9a02I2cSauAsyncEnd(dev);
}

static int R9a02I2cSauTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev != NULL && pIntrf->bIntEn && !pIntrf->bNoStop && pData != NULL && Len > 0 &&
		Len <= R9A02_I2C_TX_COPY_SIZE && dev->bOpen && dev->bActive && !dev->bRx && !dev->bError &&
		!dev->bAsync)
	{
		for (int i = 0; i < Len; i++)
		{
			dev->TxBuf[i] = pData[i];
		}
		dev->XferLen = Len;
		dev->XferCnt = 0;
		pIntrf->bTxReady = false;

		uintptr_t state = DisableInterrupt();

		dev->bAsync = true;
		R9a02EventFlagClear(dev->IntNo);
		R9a02IntEnable(dev->IntNo, true);
		R9a02SauReg(dev->Unit)->SDR[dev->Ch] = dev->TxBuf[0];

		EnableInterrupt(state);

		return -1;
	}

	// Polled: longer than the copy, the command of a read, or no interrupt
	if (dev == NULL || !pIntrf->bIntEn || pIntrf->bNoStop || pData == NULL || Len <= 0 || !dev->bOpen ||
		!dev->bActive || dev->bRx || dev->bError || dev->bAsync)
	{
		return R9a02I2cSauTxPoll(pIntrf, pData, Len);
	}

	// With bIntEn it ends as a background transfer: stop and release, the
	// event with the bytes acknowledged, -1
	int cnt = R9a02I2cSauTxPoll(pIntrf, pData, Len);

	DeviceIntrfTxComplete(pIntrf);

	if (pIntrf->EvtCB != NULL)
	{
		pIntrf->EvtCB(pIntrf, DEVINTRF_EVT_COMPLETED, NULL, cnt);
	}

	return -1;
}

static int R9a02I2cSauRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int Len)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL || !pIntrf->bIntEn)
	{
		return R9a02I2cSauRxPoll(pIntrf, pBuff, Len);
	}

	if (!dev->bOpen || !dev->bActive || !dev->bRx || dev->bError || dev->bNacked || dev->bAsync ||
		pBuff == NULL || Len <= 0)
	{
		return 0;
	}

	dev->pRxBuf = pBuff;
	dev->XferLen = Len;
	dev->XferCnt = 0;

	uintptr_t state = DisableInterrupt();

	dev->bAsync = true;
	if (Len == 1)
	{
		R9a02I2cSauOutput(dev, false);
	}
	R9a02EventFlagClear(dev->IntNo);
	R9a02IntEnable(dev->IntNo, true);
	R9a02SauReg(dev->Unit)->SDR[dev->Ch] = 0xFFU;

	EnableInterrupt(state);

	return -1;
}

static uint32_t R9a02I2cSauGetRate(DevIntrf_t * const pIntrf)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	return dev ? dev->pI2cDev->Cfg.Rate : 0;
}

// Operation clock and STCLK, with the channel stopped
static uint32_t R9a02I2cSauClock(R9a02I2cSauDev_t * const pDev, uint32_t Rate)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);
	uint16_t bit = (uint16_t)(1U << pDev->Ch);
	uint32_t pclk = SystemPeriphClockGet(0);
	R9a02SauClk_t clk;

	if (Rate < R9A02_I2C_SAU_RATE_MIN)
	{
		Rate = R9A02_I2C_SAU_RATE_MIN;
	}
	if (Rate > R9A02_I2C_SAU_RATE_MAX)
	{
		Rate = R9A02_I2C_SAU_RATE_MAX;
	}

	reg->ST = bit;

	uint32_t best = R9a02SauSetRate(pDev->Unit, bit, Rate, R9A02_I2C_SAU_DIV_MIN, R9A02_I2C_SAU_RATE_MAX, &clk);

	if (best == 0U || clk.FmckHz < (pclk >> R9A02_I2C_SAU_PRS_MAX))
	{
		return 0;
	}

	reg->SMR[pDev->Ch] = (uint16_t)(R9A02_I2C_SAU_SMR | clk.Cks);
	reg->SDR[pDev->Ch] = clk.Sdr;

	pDev->pI2cDev->Cfg.Rate = best;
	pDev->HalfUs = (500000UL + best - 1U) / best;

	// A byte with its ACK clock, SCL may also be held low by a slave at a
	// start or a stop for about 50 ms
	pDev->Loops = SystemCoreClock / 20U + (SystemCoreClock / best + 1U) * 20U;

	return best;
}

static uint32_t R9a02I2cSauSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL || dev->bOpen || Rate == 0U)
	{
		return 0;
	}

	uintptr_t state = DisableInterrupt();
	uint32_t best = R9a02I2cSauClock(dev, Rate);

	R9a02I2cSauIdle(dev);

	EnableInterrupt(state);

	return best;
}

static void R9a02I2cSauDisable(DevIntrf_t * const pIntrf)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	// A background transfer stopped here releases the interface
	bool running = R9a02I2cSauCancel(dev);

	R9a02I2cSauStop(pIntrf);
	if (running)
	{
		atomic_flag_clear(&pIntrf->bBusy);
	}

	R9a02I2cSauIdle(dev);
	dev->bLost = false;
	dev->bEnabled = false;
}

static void R9a02I2cSauEnable(DevIntrf_t * const pIntrf)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	R9a02I2cSauIdle(dev);
	dev->bEnabled = true;
}

// Bus recovery, then idle. A background transfer is dropped without event
// and the interface released.
static void R9a02I2cSauReset(DevIntrf_t * const pIntrf)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	bool running = R9a02I2cSauCancel(dev);

	R9a02I2cSauStop(pIntrf);
	R9a02I2cSauBusRecover(dev);
	dev->bLost = false;
	if (running)
	{
		atomic_flag_clear(&pIntrf->bBusy);
	}
}

// Channel stopped and released, route removed, pins disconnected. A new
// I2CInit is needed to use it again.
static void R9a02I2cSauPowerOff(DevIntrf_t * const pIntrf)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	R9a02I2cSauDisable(pIntrf);

	uintptr_t state = DisableInterrupt();

	R9a02UnregisterIntHandler(dev->IntNo);
	dev->IntNo = -1;
	IOPinDis(dev->pI2cDev->Cfg.pIOPinMap, 2);
	R9a02SauRelease(dev->Unit, (uint16_t)(1U << dev->Ch), dev);
	dev->pI2cDev = NULL;
	pIntrf->pDevData = NULL;
	pIntrf->EnCnt = 0;

	EnableInterrupt(state);
}

static void *R9a02I2cSauGetHandle(DevIntrf_t * const pIntrf)
{
	R9a02I2cSauDev_t *dev = R9a02I2cSauHandle(pIntrf);

	return dev ? dev->pI2cDev : NULL;
}

bool R9a02I2cSauUses(const I2CDev_t *pDev, int DevNo)
{
	for (int i = 0; i < s_NbR9a02I2cSauDev; i++)
	{
		if (i + R9A02_I2C_DEVNO_IIC00 != DevNo && s_R9a02I2cSauDev[i].pI2cDev == pDev)
		{
			return true;
		}
	}

	return false;
}

bool R9a02I2cSauInit(I2CDev_t * const pDev, const I2CCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo < R9A02_I2C_DEVNO_IIC00 ||
		pCfg->DevNo >= R9A02_I2C_DEVNO_IIC00 + s_NbR9a02I2cSauDev ||
		pCfg->Type != I2CTYPE_STANDARD || pCfg->Mode != I2CMODE_MASTER ||
		pCfg->AddrType != I2CADDR_TYPE_NORMAL || pCfg->bDmaEn || pCfg->Rate == 0U ||
		(pCfg->bIntEn && (pCfg->IntPrio < 0 || pCfg->IntPrio > (int)R9A02_CLIC_PRIO_MAX)) ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins != 2 || pCfg->MaxRetry < 0)
	{
		return false;
	}

	const IOPinCfg_t *sda = &pCfg->pIOPinMap[I2C_SDA_IOPIN_IDX];
	const IOPinCfg_t *scl = &pCfg->pIOPinMap[I2C_SCL_IOPIN_IDX];

	if (!R9a02PinValid(sda->PortNo, sda->PinNo) || !R9a02PinValid(scl->PortNo, scl->PinNo) ||
		(sda->PortNo == scl->PortNo && sda->PinNo == scl->PinNo) ||
		sda->PinOp != R9A02_PSEL_SAU || scl->PinOp != R9A02_PSEL_SAU ||
		sda->Type != IOPINTYPE_OPENDRAIN ||
		(scl->Type != IOPINTYPE_OPENDRAIN && scl->Type != IOPINTYPE_NORMAL) ||
		sda->Res == IOPINRES_PULLDOWN || scl->Res == IOPINRES_PULLDOWN)
	{
		return false;
	}

	R9a02I2cSauDev_t *dev = &s_R9a02I2cSauDev[pCfg->DevNo - R9A02_I2C_DEVNO_IIC00];
	uint16_t bit = (uint16_t)(1U << dev->Ch);
	uintptr_t state = DisableInterrupt();

	// An instance in use by another I2C object is not taken over, an I2C
	// object is not used for two instances. The claim fails when a UART or
	// an SPI has the channel, it also starts the SAU module clock.
	if ((dev->pI2cDev != NULL && dev->pI2cDev != pDev) || dev->bOpen ||
		R9a02I2cSauUses(pDev, pCfg->DevNo) || R9a02I2cIicaUses(pDev, -1) ||
		!R9a02SauClaim(dev->Unit, bit, dev))
	{
		EnableInterrupt(state);

		return false;
	}

	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);

	reg->ST = bit;

	// Route for the mode: a handler with bIntEn, its CLIC input on only for
	// the data in the background, the IR flag polled otherwise
	if (dev->IntNo >= 0)
	{
		R9a02UnregisterIntHandler(dev->IntNo);
		dev->IntNo = -1;
	}
	if (pCfg->bIntEn)
	{
		dev->IntNo = R9a02RegisterIntHandler(dev->EvtEnd, pCfg->IntPrio, R9a02I2cSauIrqHandler, dev);
		R9a02IntEnable(dev->IntNo, false);
	}
	else
	{
		dev->IntNo = R9a02RouteEvent(dev->EvtEnd);
	}
	if (dev->IntNo < 0)
	{
		dev->pI2cDev = NULL;
		pDev->DevIntrf.pDevData = NULL;
		pDev->DevIntrf.EnCnt = 0;
		R9a02SauRelease(dev->Unit, bit, dev);
		EnableInterrupt(state);

		return false;
	}

	pDev->Cfg = *pCfg;
	dev->pI2cDev = pDev;
	dev->bEnabled = true;
	dev->bOpen = false;
	dev->bRx = false;
	dev->bError = false;
	dev->bLost = false;
	dev->bAsync = false;

	// Noise filter off for the simplified I2C (23.3.27), SPI clock input
	// switch at its reset value for channels 0 and 1 (23.3.26). The UART of
	// the channel pair cannot run with this channel claimed.
	R9A02_PORGA_SNFEN = (uint8_t)(R9A02_PORGA_SNFEN & ~dev->Snfen);
	if (dev->Unit == 0U && dev->Ch < 2U)
	{
		R9A02_PORGA_ISC = (uint8_t)(R9A02_PORGA_ISC & ~(dev->Ch ? R9A02_ISC_ISC76_Msk : R9A02_ISC_ISC43_Msk));
	}

	// Initial setting, table 23.108: mode, format, clock, output level
	// register 0 for this channel, SDA and SCL high
	reg->SOL = (uint16_t)(reg->SOL & ~bit);
	reg->SCR[dev->Ch] = R9A02_I2C_SAU_SCR_TX;

	if (R9a02I2cSauClock(dev, pCfg->Rate) == 0U)
	{
		R9a02UnregisterIntHandler(dev->IntNo);
		dev->IntNo = -1;
		dev->pI2cDev = NULL;
		pDev->DevIntrf.pDevData = NULL;
		pDev->DevIntrf.EnCnt = 0;
		R9a02SauRelease(dev->Unit, bit, dev);
		EnableInterrupt(state);

		return false;
	}

	R9a02I2cSauIdle(dev);

	DevIntrf_t *intrf = &pDev->DevIntrf;

	intrf->pDevData = dev;
	intrf->Type = DEVINTRF_TYPE_I2C;
	intrf->bDma = false;
	intrf->bIntEn = pCfg->bIntEn;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = R9a02I2cSauEnable;
	intrf->Disable = R9a02I2cSauDisable;
	intrf->PowerOff = R9a02I2cSauPowerOff;
	intrf->Reset = R9a02I2cSauReset;
	intrf->GetHandle = R9a02I2cSauGetHandle;
	intrf->GetRate = R9a02I2cSauGetRate;
	intrf->SetRate = R9a02I2cSauSetRate;
	intrf->StartRx = R9a02I2cSauStartRx;
	intrf->StartTx = R9a02I2cSauStartTx;
	intrf->StopRx = R9a02I2cSauStop;
	intrf->StopTx = R9a02I2cSauStop;
	intrf->RxData = R9a02I2cSauRxData;
	intrf->TxData = R9a02I2cSauTxData;
	intrf->TxSrData = R9a02I2cSauTxPoll;
	intrf->EnCnt = 1;
	atomic_flag_clear(&intrf->bBusy);

	// SO and CKO at 1 drive the lines high before the pins are connected.
	// SCL is a SAU output with PDR = 1, SDA has its output buffer on with
	// NCODR = 1 (16.5.1).
	IOPinConfig(scl->PortNo, scl->PinNo, R9A02_PSEL_SAU, IOPINDIR_OUTPUT, scl->Res, scl->Type);
	IOPinConfig(sda->PortNo, sda->PinNo, R9A02_PSEL_SAU, IOPINDIR_INPUT, sda->Res, IOPINTYPE_OPENDRAIN);

	EnableInterrupt(state);

	return true;
}

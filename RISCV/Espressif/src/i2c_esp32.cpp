/**-------------------------------------------------------------------------
@file	i2c_esp32.cpp

@brief	HP I2C0 polling master and interrupt slave for ESP32-C2/C3/C5/C6/C61.

Uses the generic DeviceIntrf start/data/stop sequence, as the R9A02 master
port. Seven-bit addressing, repeated START and FIFO-sized chunks. C2 has
16-byte FIFOs; the other targets have 32-byte FIFOs. All waits are bounded.
Rate selects the nearest integer-divider timing from 1 kHz to 400 kHz.
At least 32 module-clock cycles per half period keep the glitch filter
shorter than the data sampling intervals. Timing follows ESP-IDF v5.5.1
i2c_ll.h (XTAL clock, no fractional divider).

SDA then SCL in the pin map, PortNo 0. Board maps select the I2C PinOp
aliases, bidirectional direction and open-drain type; IOPinCfg applies them. External pull-ups are required. LP I2C,
SMBus, 10-bit addresses, DMA and master interrupt transfers are not
implemented. C3/C5/C6/C61 support one 7-bit slave address with interrupts
and clock stretching; C2 has no slave hardware. RxData NACKs its final byte; use a new START for another read.
Reset releases the controller; it does not clock a stuck external device.


@author	Hoang Nguyen Hoan
@date	June 21, 2025

@license

MIT

Copyright (c) 2025, I-SYST inc., all rights reserved

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
#include "coredev/i2c.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "esp32xx_i2c.h"
#include "esp32xx_irq.h"

#define I2C_BASE ESP32_I2C0_BASE
#define I2C_CPUINT 13U
#define I2C_ERRORS (ESP32_I2C_INT_NACK | ESP32_I2C_INT_ARBITRATION_LOST | \
	ESP32_I2C_INT_TIME_OUT | ESP32_I2C_INT_SCL_ST_TO | ESP32_I2C_INT_SCL_MAIN_ST_TO | \
	ESP32_I2C_INT_RXFIFO_OVF | ESP32_I2C_INT_MST_TXFIFO_UDF)

typedef struct {
	I2CDev_t *pOwner;
	uint32_t Divider;
	uint32_t HalfCycle;
	bool bEnabled;
	bool bOpen;
	bool bRx;
	bool bReadEnded;
	bool bError;
	bool bLost;
	bool bSlave;
	int SlaveCount;
	int SlaveQueued;
} Esp32I2cDev_t;

static Esp32I2cDev_t s_I2c;

static Esp32I2cDev_t *Esp32I2cHandle(DevIntrf_t *pIntrf)
{
	if (pIntrf == NULL || s_I2c.pOwner == NULL ||
		pIntrf != &s_I2c.pOwner->DevIntrf || pIntrf->pDevData != &s_I2c)
	{
		return NULL;
	}
	return &s_I2c;
}

static void Esp32I2cClock(bool Enable)
{
	uintptr_t state;

	state = DisableInterrupt();
#if defined(ESP32C2) || defined(ESP32C3)
	if (Enable)
	{
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x10U) |= 1UL << 7;
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x18U) |= 1UL << 7;
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x18U) &= ~(1UL << 7);
	}
	else
	{
		ESP32_I2C_CLK_CONF_REG(I2C_BASE) = 0;
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x10U) &= ~(1UL << 7);
	}
#else
	if (Enable)
	{
		ESP32_I2C_PCR_CONF_REG |= ESP32_I2C_PCR_CLK_EN | ESP32_I2C_PCR_RST_EN;
		ESP32_I2C_PCR_CONF_REG &= ~ESP32_I2C_PCR_RST_EN;
	}
	else
	{
		ESP32_I2C_PCR_SCLK_CONF_REG = 0;
		ESP32_I2C_PCR_CONF_REG &= ~ESP32_I2C_PCR_CLK_EN;
	}
#endif
	EnableInterrupt(state);
}

static void Esp32I2cFifoReset(void)
{
	ESP32_I2C_FIFO_CONF_REG(I2C_BASE) = ESP32_I2C_FIFO_CONF_FIFO_PRT_EN_Msk |
		ESP32_I2C_FIFO_CONF_RX_FIFO_RST_Msk | ESP32_I2C_FIFO_CONF_TX_FIFO_RST_Msk;
	ESP32_I2C_FIFO_CONF_REG(I2C_BASE) = ESP32_I2C_FIFO_CONF_FIFO_PRT_EN_Msk |
		(s_I2c.bSlave ? (16U | (8U << 5)) : 0);
}

static void Esp32I2cTiming(Esp32I2cDev_t *pDev)
{
	uint32_t half;
	uint32_t wait;

	half = pDev->HalfCycle;
	wait = pDev->pOwner->Cfg.Rate >= 80000U ? half / 2U - 2U : half / 4U;
#if defined(ESP32C2) || defined(ESP32C3)
	ESP32_I2C_CLK_CONF_REG(I2C_BASE) = (pDev->Divider - 1U) |
		ESP32_I2C_CLK_CONF_SCLK_ACTIVE_Msk;
#else
	ESP32_I2C_PCR_SCLK_CONF_REG = ((pDev->Divider - 1U) << ESP32_I2C_PCR_SCLK_DIV_NUM_Pos) |
		ESP32_I2C_PCR_SCLK_EN;
#endif
	ESP32_I2C_SCL_LOW_PERIOD_REG(I2C_BASE) = half - 1U;
	ESP32_I2C_SCL_HIGH_PERIOD_REG(I2C_BASE) = (wait << 9) | (half - wait);
	ESP32_I2C_SDA_HOLD_REG(I2C_BASE) = half / 4U - 1U;
	ESP32_I2C_SDA_SAMPLE_REG(I2C_BASE) = half / 2U - 1U;
	ESP32_I2C_SCL_START_HOLD_REG(I2C_BASE) = half - 1U;
	ESP32_I2C_SCL_RSTART_SETUP_REG(I2C_BASE) = half - 1U;
	ESP32_I2C_SCL_STOP_HOLD_REG(I2C_BASE) = half - 1U;
	ESP32_I2C_SCL_STOP_SETUP_REG(I2C_BASE) = half - 1U;
	// Timeout is measured in powers of two module clock cycles, per SCL stretch.
	ESP32_I2C_TO_REG(I2C_BASE) = pDev->bSlave ? 0 :
		(32U - __builtin_clz(5U * half) + 2U) | (1U << 5);
	ESP32_I2C_CTR_REG(I2C_BASE) |= ESP32_I2C_CTR_CONF_UPGATE_Msk;
}

static uint32_t Esp32I2cSetRate(DevIntrf_t *pIntrf, uint32_t Rate)
{
	Esp32I2cDev_t *dev;
	uint32_t source;
	uint32_t pre;
	uint32_t half;
	uint32_t candidate;
	uint32_t step;
	uint32_t actual;
	uint32_t error;
	uint32_t bestError;
	uint32_t bestRate;
	uint32_t bestPre;
	uint32_t bestHalf;
	uint32_t wait;

	dev = Esp32I2cHandle(pIntrf);
	if (dev == NULL || dev->bOpen)
	{
		return 0;
	}
	source = GetHighFreqOscFreq();
	if (source == 0)
	{
		return 0;
	}
	if (Rate < 1000U)
	{
		Rate = 1000U;
	}
	if (Rate > 400000U)
	{
		Rate = 400000U;
	}
	bestError = UINT32_MAX;
	bestRate = 0;
	bestPre = 1;
	bestHalf = 32;
	for (pre = 1; pre <= 256U; pre++)
	{
		half = source / (2U * pre * Rate);
		if (half < 32U)
		{
			half = 32U;
		}
		if (half > 511U)
		{
			half = 511U;
		}
		for (step = 0; step < 2U; step++)
		{
			candidate = half + step;
			actual = source / (2U * pre * candidate);
			wait = actual >= 80000U ? candidate / 2U - 2U : candidate / 4U;
			if (candidate > 511U || wait > 127U || actual < 1000U || actual > 400000U)
			{
				continue;
			}
			error = actual > Rate ? actual - Rate : Rate - actual;
			if (error < bestError)
			{
				bestError = error;
				bestRate = actual;
				bestPre = pre;
				bestHalf = candidate;
			}
		}
	}
	if (bestRate != 0)
	{
		dev->Divider = bestPre;
		dev->HalfCycle = bestHalf;
		dev->pOwner->Cfg.Rate = bestRate;
		if (dev->bEnabled)
		{
			Esp32I2cTiming(dev);
		}
	}
	return bestRate;
}

static uint32_t Esp32I2cGetRate(DevIntrf_t *pIntrf)
{
	Esp32I2cDev_t *dev;

	dev = Esp32I2cHandle(pIntrf);
	return dev ? dev->pOwner->Cfg.Rate : 0;
}

static void Esp32I2cCommand(uint32_t Slot, uint32_t Opcode, uint32_t Count, uint32_t Ack)
{
	ESP32_I2C_COMD_REG(I2C_BASE, Slot) = (Opcode << ESP32_I2C_COMD_OPCODE_Pos) | Count | Ack;
}

static bool Esp32I2cRun(Esp32I2cDev_t *pDev, uint32_t Done, uint32_t Bytes)
{
	uint32_t status;
	uint32_t loops;
	uint64_t budget;

	// At least the wire time plus 100 ms in CPU cycles. Each poll costs more
	// than one cycle; this is a conservative bound, not a calibrated timer.
	budget = (uint64_t)SystemCoreClockGet() * (Bytes * 9U + 20U) / pDev->pOwner->Cfg.Rate;
	budget += SystemCoreClockGet() / 10U + 1U;
	loops = budget > UINT32_MAX ? UINT32_MAX : (uint32_t)budget;
	ESP32_I2C_INT_CLR_REG(I2C_BASE) = UINT32_MAX;
	ESP32_I2C_CTR_REG(I2C_BASE) |= ESP32_I2C_CTR_CONF_UPGATE_Msk;
	ESP32_I2C_CTR_REG(I2C_BASE) |= ESP32_I2C_CTR_TRANS_START_Msk;
	while (loops-- > 0)
	{
		status = ESP32_I2C_INT_RAW_REG(I2C_BASE);
		if (status & I2C_ERRORS)
		{
			pDev->bError = true;
			pDev->bLost = (status & ESP32_I2C_INT_ARBITRATION_LOST) != 0;
			return false;
		}
		if (status & Done)
		{
			return true;
		}
	}
	pDev->bError = true;
	return false;
}

static void Esp32I2cFinish(Esp32I2cDev_t *pDev)
{
	if (!pDev->bSlave && pDev->bOpen && !pDev->bLost)
	{
		Esp32I2cCommand(0, ESP32_I2C_OPCODE_STOP, 0, 0);
		Esp32I2cRun(pDev, ESP32_I2C_INT_TRANS_COMPLETE, 0);
	}
	ESP32_I2C_CTR_REG(I2C_BASE) |= ESP32_I2C_CTR_FSM_RST_Msk;
	Esp32I2cFifoReset();
	ESP32_I2C_INT_CLR_REG(I2C_BASE) = UINT32_MAX;
	pDev->bOpen = false;
	pDev->bRx = false;
	pDev->bReadEnded = false;
	pDev->bLost = false;
}

static void Esp32I2cEnable(DevIntrf_t *pIntrf);
static void Esp32I2cDisable(DevIntrf_t *pIntrf);

static void Esp32I2cStop(DevIntrf_t *pIntrf)
{
	Esp32I2cDev_t *dev;

	dev = Esp32I2cHandle(pIntrf);
	if (dev && dev->bEnabled)
	{
		if (dev->bSlave)
		{
			Esp32I2cDisable(pIntrf);
			Esp32I2cEnable(pIntrf);
		}
		else
		{
			Esp32I2cFinish(dev);
		}
		dev->bError = false;
	}
}

static bool Esp32I2cStart(DevIntrf_t *pIntrf, uint32_t Address, bool Rx)
{
	Esp32I2cDev_t *dev;
	bool wasOpen;
	bool success;

	dev = Esp32I2cHandle(pIntrf);
	if (dev == NULL || !dev->bEnabled || dev->bSlave)
	{
		return false;
	}
	wasOpen = dev->bOpen;
	if (Address > 127U || (wasOpen && dev->bError))
	{
		dev->bError = true;
		return false;
	}
	if (!wasOpen)
	{
		dev->bError = false;
		if (ESP32_I2C_SR_REG(I2C_BASE) & ESP32_I2C_SR_BUS_BUSY_Msk)
		{
			return false;
		}
	}
	Esp32I2cFifoReset();
	ESP32_I2C_DATA_REG(I2C_BASE) = (Address << 1) | (Rx ? 1U : 0U);
	Esp32I2cCommand(0, ESP32_I2C_OPCODE_RSTART, 0, 0);
	Esp32I2cCommand(1, ESP32_I2C_OPCODE_WRITE, 1, ESP32_I2C_COMD_ACK_EN);
	Esp32I2cCommand(2, ESP32_I2C_OPCODE_END, 0, 0);
	dev->bOpen = true;
	dev->bRx = Rx;
	dev->bReadEnded = false;
	success = Esp32I2cRun(dev, ESP32_I2C_INT_END_DETECT, 1);
	if (!success && !wasOpen)
	{
		// A failed initial START is not followed by the generic Stop hook.
		Esp32I2cFinish(dev);
	}
	return success;
}

static bool Esp32I2cStartRx(DevIntrf_t *pIntrf, uint32_t Address)
{
	return Esp32I2cStart(pIntrf, Address, true);
}

static bool Esp32I2cStartTx(DevIntrf_t *pIntrf, uint32_t Address)
{
	return Esp32I2cStart(pIntrf, Address, false);
}

static int Esp32I2cTxData(DevIntrf_t *pIntrf, const uint8_t *pData, int Length)
{
	Esp32I2cDev_t *dev;
	int count;
	int chunk;
	int i;

	dev = Esp32I2cHandle(pIntrf);
	if (dev == NULL || !dev->bEnabled || !dev->bOpen || dev->bRx || dev->bError ||
		pData == NULL || Length <= 0)
	{
		return 0;
	}
	count = 0;
	while (count < Length)
	{
		chunk = Length - count;
		if (chunk > (int)ESP32_I2C_FIFO_DEPTH)
		{
			chunk = ESP32_I2C_FIFO_DEPTH;
		}
		Esp32I2cFifoReset();
		for (i = 0; i < chunk; i++)
		{
			ESP32_I2C_DATA_REG(I2C_BASE) = pData[count + i];
		}
		Esp32I2cCommand(0, ESP32_I2C_OPCODE_WRITE, chunk, ESP32_I2C_COMD_ACK_EN);
		Esp32I2cCommand(1, ESP32_I2C_OPCODE_END, 0, 0);
		if (!Esp32I2cRun(dev, ESP32_I2C_INT_END_DETECT, chunk))
		{
			break;
		}
		count += chunk;
	}
	return count;
}

static int Esp32I2cRxData(DevIntrf_t *pIntrf, uint8_t *pData, int Length)
{
	Esp32I2cDev_t *dev;
	int count;
	int chunk;
	int ack;
	int slot;
	int i;

	dev = Esp32I2cHandle(pIntrf);
	if (dev == NULL || !dev->bEnabled || !dev->bOpen || !dev->bRx || dev->bError ||
		dev->bReadEnded || pData == NULL || Length <= 0)
	{
		return 0;
	}
	count = 0;
	while (count < Length)
	{
		chunk = Length - count;
		if (chunk > (int)ESP32_I2C_FIFO_DEPTH)
		{
			chunk = ESP32_I2C_FIFO_DEPTH;
		}
		ack = chunk;
		if (count + chunk == Length)
		{
			ack--;
		}
		Esp32I2cFifoReset();
		slot = 0;
		if (ack > 0)
		{
			Esp32I2cCommand(slot, ESP32_I2C_OPCODE_READ, ack, 0);
			slot++;
		}
		if (ack != chunk)
		{
			Esp32I2cCommand(slot, ESP32_I2C_OPCODE_READ, 1, ESP32_I2C_COMD_ACK_VAL);
			slot++;
		}
		Esp32I2cCommand(slot, ESP32_I2C_OPCODE_END, 0, 0);
		if (!Esp32I2cRun(dev, ESP32_I2C_INT_END_DETECT, chunk))
		{
			break;
		}
		for (i = 0; i < chunk; i++)
		{
			pData[count + i] = ESP32_I2C_DATA_REG(I2C_BASE);
		}
		count += chunk;
	}
	dev->bReadEnded = count == Length;
	return count;
}

#if !defined(ESP32C2)
static void Esp32I2cSlaveDrain(Esp32I2cDev_t *pDev)
{
	I2CDev_t *i2c;
	uint32_t available;
	uint8_t value;

	i2c = pDev->pOwner;
	available = (ESP32_I2C_SR_REG(I2C_BASE) & ESP32_I2C_SR_RXFIFO_CNT_Msk) >> ESP32_I2C_SR_RXFIFO_CNT_Pos;
	while (available--)
	{
		value = ESP32_I2C_DATA_REG(I2C_BASE);
		if (i2c->pTRBuff[0] && pDev->SlaveCount < i2c->TRBuffLen[0])
		{
			i2c->pTRBuff[0][pDev->SlaveCount] = value;
		}
		else
		{
			pDev->bError = true;
		}
		if (pDev->SlaveCount < INT32_MAX)
		{
			pDev->SlaveCount++;
		}
	}
}

static void Esp32I2cSlaveFill(Esp32I2cDev_t *pDev)
{
	I2CDev_t *i2c;
	uint32_t available;
	uint8_t value;

	i2c = pDev->pOwner;
	available = ESP32_I2C_FIFO_DEPTH - ((ESP32_I2C_SR_REG(I2C_BASE) &
		ESP32_I2C_SR_TXFIFO_CNT_Msk) >> ESP32_I2C_SR_TXFIFO_CNT_Pos);
	while (available--)
	{
		value = i2c->pRRData[0] && pDev->SlaveQueued < i2c->RRDataLen[0] ?
			i2c->pRRData[0][pDev->SlaveQueued] : 0xff;
		ESP32_I2C_DATA_REG(I2C_BASE) = value;
		if (pDev->SlaveQueued < INT32_MAX)
		{
			pDev->SlaveQueued++;
		}
	}
}

static void Esp32I2cSlaveComplete(Esp32I2cDev_t *pDev)
{
	I2CDev_t *i2c;
	int remaining;

	if (!pDev->bOpen)
	{
		return;
	}
	i2c = pDev->pOwner;
	if (pDev->bRx)
	{
		Esp32I2cSlaveDrain(pDev);
	}
	else
	{
		remaining = (ESP32_I2C_SR_REG(I2C_BASE) & ESP32_I2C_SR_TXFIFO_CNT_Msk) >> ESP32_I2C_SR_TXFIFO_CNT_Pos;
		pDev->SlaveCount = pDev->SlaveQueued - remaining;
	}
	pDev->bOpen = false;
	if (i2c->Cfg.EvtCB)
	{
		i2c->Cfg.EvtCB(&i2c->DevIntrf, DEVINTRF_EVT_COMPLETED,
			pDev->bRx ? i2c->pTRBuff[0] : i2c->pRRData[0], pDev->bError ? -1 : pDev->SlaveCount);
	}
}

static void Esp32I2cSlaveIrq(void)
{
	Esp32I2cDev_t *dev;
	I2CDev_t *i2c;
	uint32_t status;
	uint32_t state;
	uint32_t cause;
	int previous;

	dev = &s_I2c;
	if (!dev->pOwner || !dev->bEnabled || !dev->bSlave)
	{
		return;
	}
	i2c = dev->pOwner;
	status = ESP32_I2C_INT_STATUS_REG(I2C_BASE);
	state = ESP32_I2C_SR_REG(I2C_BASE);
	ESP32_I2C_INT_CLR_REG(I2C_BASE) = status;
	if (status & ESP32_I2C_INT_RXFIFO_OVF)
	{
		dev->bError = true;
	}
	if (status & ESP32_I2C_INT_TRANS_COMPLETE)
	{
		Esp32I2cSlaveComplete(dev);
		ESP32_I2C_INT_ENA_REG(I2C_BASE) &= ~ESP32_I2C_INT_TXFIFO_WM;
	}
	if (status & ESP32_I2C_INT_SLAVE_STRETCH)
	{
		cause = (state >> ESP32_I2C_SR_STRETCH_CAUSE_Pos) & 3U;
		if (cause == 0)
		{
			previous = 0;
			if (dev->bOpen && dev->bRx)
			{
				Esp32I2cSlaveDrain(dev);
				previous = dev->SlaveCount;
			}
			dev->bOpen = true;
			dev->bRx = !(state & ESP32_I2C_SR_SLAVE_RW_Msk);
			dev->bError = false;
			dev->SlaveCount = 0;
			dev->SlaveQueued = 0;
			Esp32I2cFifoReset();
			if (i2c->Cfg.EvtCB)
			{
				i2c->Cfg.EvtCB(&i2c->DevIntrf,
					dev->bRx ? DEVINTRF_EVT_WRITE_RQST : DEVINTRF_EVT_READ_RQST,
					i2c->pTRBuff[0], previous);
			}
			if (!dev->bRx)
			{
				Esp32I2cSlaveFill(dev);
				ESP32_I2C_INT_ENA_REG(I2C_BASE) |= ESP32_I2C_INT_TXFIFO_WM;
			}
			else
			{
				ESP32_I2C_INT_ENA_REG(I2C_BASE) &= ~ESP32_I2C_INT_TXFIFO_WM;
			}
		}
		else if (dev->bOpen)
		{
			if (dev->bRx)
			{
				Esp32I2cSlaveDrain(dev);
			}
			else
			{
				Esp32I2cSlaveFill(dev);
			}
		}
		ESP32_I2C_CTR_REG(I2C_BASE) |= ESP32_I2C_CTR_CONF_UPGATE_Msk;
		ESP32_I2C_SCL_STRETCH_CONF_REG(I2C_BASE) |= ESP32_I2C_SLAVE_STRETCH_CLR_Msk;
	}
	else if (dev->bOpen)
	{
		if (dev->bRx && (status & ESP32_I2C_INT_RXFIFO_WM))
		{
			Esp32I2cSlaveDrain(dev);
		}
		if (!dev->bRx && (status & ESP32_I2C_INT_TXFIFO_WM))
		{
			Esp32I2cSlaveFill(dev);
		}
	}
}
#endif

static void Esp32I2cEnable(DevIntrf_t *pIntrf)
{
	Esp32I2cDev_t *dev;

	dev = Esp32I2cHandle(pIntrf);
	if (dev == NULL || dev->bEnabled)
	{
		return;
	}
	Esp32I2cClock(true);
	ESP32_I2C_CTR_REG(I2C_BASE) = (dev->bSlave ? 0 : ESP32_I2C_CTR_MS_MODE_Msk) | ESP32_I2C_CTR_CLK_EN_Msk |
		ESP32_I2C_CTR_ARBITRATION_EN_Msk | ESP32_I2C_CTR_SDA_FORCE_OUT_Msk | ESP32_I2C_CTR_SCL_FORCE_OUT_Msk;
	ESP32_I2C_INT_ENA_REG(I2C_BASE) = 0;
	ESP32_I2C_INT_CLR_REG(I2C_BASE) = UINT32_MAX;
	// Seven source-clock cycles of glitch filtering on SDA and SCL.
	ESP32_I2C_FILTER_CFG_REG(I2C_BASE) = 7U | (7U << 4) | (3U << 8);
	Esp32I2cTiming(dev);
	Esp32I2cFifoReset();
	IOPinCfg(dev->pOwner->Cfg.pIOPinMap, dev->pOwner->Cfg.NbIOPins);
	dev->bError = false;
	dev->bLost = false;
	dev->bEnabled = true;
#if !defined(ESP32C2)
	if (dev->bSlave)
	{
		ESP32_I2C_SLAVE_ADDR_REG(I2C_BASE) = dev->pOwner->Cfg.SlaveAddr[0];
		ESP32_I2C_CTR_REG(I2C_BASE) |= ESP32_I2C_CTR_SLV_TX_AUTO_START_Msk;
		ESP32_I2C_SCL_STRETCH_CONF_REG(I2C_BASE) = 0x3ffU | ESP32_I2C_SLAVE_STRETCH_EN_Msk;
		ESP32_I2C_TO_REG(I2C_BASE) = 0;
		ESP32_I2C_CTR_REG(I2C_BASE) |= ESP32_I2C_CTR_CONF_UPGATE_Msk;
		if (!Esp32EnableSourceIrq(ESP32_I2C_INTR_SOURCE_I2C0, I2C_CPUINT,
			dev->pOwner->Cfg.IntPrio, Esp32I2cSlaveIrq))
		{
			Esp32I2cDisable(pIntrf);
			return;
		}
		ESP32_I2C_INT_ENA_REG(I2C_BASE) = ESP32_I2C_INT_SLAVE_STRETCH |
			ESP32_I2C_INT_TRANS_COMPLETE | ESP32_I2C_INT_RXFIFO_WM | ESP32_I2C_INT_RXFIFO_OVF;
	}
#endif
}

static void Esp32I2cDisable(DevIntrf_t *pIntrf)
{
	Esp32I2cDev_t *dev;
	const IOPinCfg_t *pins;
	int i;

	dev = Esp32I2cHandle(pIntrf);
	if (dev == NULL || !dev->bEnabled)
	{
		return;
	}
	ESP32_I2C_INT_ENA_REG(I2C_BASE) = 0;
	if (dev->bSlave)
	{
		Esp32DisableSourceIrq(ESP32_I2C_INTR_SOURCE_I2C0, I2C_CPUINT);
	}
	Esp32I2cFinish(dev);
	pins = dev->pOwner->Cfg.pIOPinMap;
	for (i = 0; i < 2; i++)
	{
		IOPinConfig(0, pins[i].PinNo, IOPINOP_GPIO, IOPINDIR_INPUT, pins[i].Res, IOPINTYPE_NORMAL);
	}
	Esp32I2cClock(false);
	dev->bEnabled = false;
}

static void Esp32I2cReset(DevIntrf_t *pIntrf)
{
	Esp32I2cDev_t *dev;

	dev = Esp32I2cHandle(pIntrf);
	if (dev && dev->bEnabled)
	{
		if (dev->bSlave)
		{
			Esp32I2cDisable(pIntrf);
			Esp32I2cEnable(pIntrf);
		}
		else
		{
			Esp32I2cFinish(dev);
		}
		dev->bError = false;
	}
}

static void Esp32I2cPowerOff(DevIntrf_t *pIntrf)
{
	if (Esp32I2cHandle(pIntrf))
	{
		Esp32I2cDisable(pIntrf);
		s_I2c.pOwner = NULL;
		pIntrf->pDevData = NULL;
		pIntrf->EnCnt = 0;
	}
}

static void *Esp32I2cGetHandle(DevIntrf_t *pIntrf)
{
	Esp32I2cDev_t *dev;

	dev = Esp32I2cHandle(pIntrf);
	return dev ? dev->pOwner : NULL;
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfg)
{
	DevIntrf_t *intrf;
	const IOPinCfg_t *pins;
	uintptr_t state;
	int i;

	if (pDev == NULL || pCfg == NULL || pCfg->DevNo != 0 || pCfg->Type != I2CTYPE_STANDARD ||
		(pCfg->Mode != I2CMODE_MASTER && pCfg->Mode != I2CMODE_SLAVE) || pCfg->AddrType != I2CADDR_TYPE_NORMAL ||
		pCfg->bDmaEn || (pCfg->Mode == I2CMODE_MASTER && pCfg->bIntEn) ||
		(pCfg->Mode == I2CMODE_SLAVE && (!pCfg->bIntEn || pCfg->NbSlaveAddr != 1 ||
		 pCfg->SlaveAddr[0] == 0 || pCfg->SlaveAddr[0] > 127 || pCfg->IntPrio < 1 || pCfg->IntPrio > 7)) ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins != 2 || pCfg->MaxRetry < 0 ||
		GetHighFreqOscFreq() == 0)
	{
		return false;
	}
#if defined(ESP32C2)
	// C2 has a master-only I2C controller.
	if (pCfg->Mode == I2CMODE_SLAVE)
	{
		return false;
	}
#endif
	pins = pCfg->pIOPinMap;
	for (i = 0; i < 2; i++)
	{
		if (pins[i].PortNo != 0 || pins[i].PinNo < 0 || pins[i].PinNo > ESP32_GPIO_PIN_MAX ||
			pins[i].Res == IOPINRES_PULLDOWN || pins[i].PinDir != IOPINDIR_BI ||
			pins[i].Type != IOPINTYPE_OPENDRAIN ||
			pins[i].PinOp != (i == I2C_SDA_IOPIN_IDX ? ESP32_PINOP_I2C0_SDA : ESP32_PINOP_I2C0_SCL))
		{
			return false;
		}
	}
	if (pins[0].PinNo == pins[1].PinNo)
	{
		return false;
	}
	state = DisableInterrupt();
	if (s_I2c.pOwner)
	{
		EnableInterrupt(state);
		return false;
	}
	s_I2c.pOwner = pDev;
	s_I2c.bSlave = pCfg->Mode == I2CMODE_SLAVE;
	EnableInterrupt(state);
	pDev->Cfg = *pCfg;
	intrf = &pDev->DevIntrf;
	intrf->pDevData = &s_I2c;
	intrf->Type = DEVINTRF_TYPE_I2C;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->bDma = false;
	intrf->bIntEn = s_I2c.bSlave;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	atomic_flag_clear(&intrf->bBusy);
	intrf->EnCnt = 1;
	intrf->Enable = Esp32I2cEnable;
	intrf->Disable = Esp32I2cDisable;
	intrf->Reset = Esp32I2cReset;
	intrf->PowerOff = Esp32I2cPowerOff;
	intrf->GetRate = Esp32I2cGetRate;
	intrf->SetRate = Esp32I2cSetRate;
	intrf->StartRx = Esp32I2cStartRx;
	intrf->StartTx = Esp32I2cStartTx;
	intrf->StopRx = Esp32I2cStop;
	intrf->StopTx = Esp32I2cStop;
	intrf->RxData = Esp32I2cRxData;
	intrf->TxData = Esp32I2cTxData;
	intrf->TxSrData = Esp32I2cTxData;
	intrf->GetHandle = Esp32I2cGetHandle;
	if (Esp32I2cSetRate(intrf, pCfg->Rate) == 0)
	{
		Esp32I2cPowerOff(intrf);
		return false;
	}
	Esp32I2cEnable(intrf);
	if (!s_I2c.bEnabled)
	{
		Esp32I2cPowerOff(intrf);
		return false;
	}
	return true;
}

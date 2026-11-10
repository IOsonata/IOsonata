/**-------------------------------------------------------------------------
@file	i2c_esp32.cpp

@brief	HP I2C for ESP32-C2/C3/C5/C6/C61 and C6 LP I2C master.

Uses the generic DeviceIntrf start/data/stop sequence, as the R9A02 master
port. Seven-bit addressing, repeated START and FIFO-sized chunks. C2 has
16-byte FIFOs; the other targets have 32-byte FIFOs. All waits are bounded.
Rate selects the nearest integer-divider timing from 1 kHz to 400 kHz.
For HP I2C, at least 32 module-clock cycles per half period keep the glitch filter
shorter than the data sampling intervals. Timing follows ESP-IDF v5.5.1
i2c_ll.h (XTAL clock, no fractional divider).

SDA then SCL in the pin map, PortNo 0. Board maps select the I2C PinOp
aliases, bidirectional direction and open-drain type; IOPinCfg applies them.
The bus needs pull-ups; pin maps may enable internal ones with IOPINRES_PULLUP.
Use external pull-ups where needed to meet bus rise-time requirements.
C6 device 1 is the LP master, fixed to GPIO6/7,
with 16-byte FIFOs and XTAL/2 timing. Its minimum rate is about 19.6 kHz
with a 40 MHz crystal; Rate returns the closest supported rate up to 400 kHz.
Device 0 remains HP I2C. LP clock changes do not change the HP divider.
SMBus, 10-bit addresses, DMA and master interrupt transfers are not implemented.
C3/C5/C6/C61 support one 7-bit slave address with interrupts
and clock stretching; C2 has no slave hardware. RxData NACKs its final byte;
use a new START for another read.
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

#define I2C_CPUINT 13U
#define I2C_ERRORS (ESP32_I2C_INT_NACK | ESP32_I2C_INT_ARBITRATION_LOST | \
	ESP32_I2C_INT_TIME_OUT | ESP32_I2C_INT_SCL_ST_TO | ESP32_I2C_INT_SCL_MAIN_ST_TO | \
	ESP32_I2C_INT_RXFIFO_OVF | ESP32_I2C_INT_MST_TXFIFO_UDF)

typedef struct {
	uintptr_t Base;
	unsigned FifoDepth;
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

static Esp32I2cDev_t s_I2c[] = {
	{ESP32_I2C0_BASE, ESP32_I2C_FIFO_DEPTH},
#if defined(ESP32C6)
	{ESP32_LP_I2C_BASE, ESP32_LP_I2C_FIFO_DEPTH},
#endif
};

static Esp32I2cDev_t *Esp32I2cHandle(DevIntrf_t *pIntrf)
{
	unsigned i;

	if (pIntrf == NULL)
	{
		return NULL;
	}
	for (i = 0; i < sizeof(s_I2c) / sizeof(s_I2c[0]); i++)
	{
		if (s_I2c[i].pOwner && pIntrf == &s_I2c[i].pOwner->DevIntrf && pIntrf->pDevData == &s_I2c[i])
		{
			return &s_I2c[i];
		}
	}
	return NULL;
}

static void Esp32I2cClock(Esp32I2cDev_t *pDev, bool Enable)
{
	uintptr_t state;

	state = DisableInterrupt();
#if defined(ESP32C6)
	if (pDev->Base == ESP32_LP_I2C_BASE)
	{
		if (Enable)
		{
			ESP32_LP_PERI_CLK_EN_REG |= ESP32_LP_I2C_CLK_EN_Msk;
			ESP32_LP_PERI_RESET_EN_REG |= ESP32_LP_I2C_RESET_EN_Msk;
			ESP32_LP_PERI_RESET_EN_REG &= ~ESP32_LP_I2C_RESET_EN_Msk;
			ESP32_LP_CLKRST_PERI_REG |= ESP32_LP_I2C_CLK_SEL_Msk;
		}
		else
		{
			ESP32_LP_PERI_CLK_EN_REG &= ~ESP32_LP_I2C_CLK_EN_Msk;
		}
		EnableInterrupt(state);
		return;
	}
#endif

#if defined(ESP32C2) || defined(ESP32C3)
	if (Enable)
	{
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x10U) |= 1UL << 7;
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x18U) |= 1UL << 7;
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x18U) &= ~(1UL << 7);
	}
	else
	{
		ESP32_I2C_CLK_CONF_REG(pDev->Base) = 0;
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

static void Esp32I2cFifoReset(Esp32I2cDev_t *pDev)
{
	ESP32_I2C_FIFO_CONF_REG(pDev->Base) = ESP32_I2C_FIFO_CONF_FIFO_PRT_EN_Msk |
		ESP32_I2C_FIFO_CONF_RX_FIFO_RST_Msk | ESP32_I2C_FIFO_CONF_TX_FIFO_RST_Msk;
	ESP32_I2C_FIFO_CONF_REG(pDev->Base) = ESP32_I2C_FIFO_CONF_FIFO_PRT_EN_Msk |
		(pDev->bSlave ? (16U | (8U << 5)) : 0);
}

static void Esp32I2cTiming(Esp32I2cDev_t *pDev)
{
	uint32_t half;
	uint32_t wait;

	half = pDev->HalfCycle;
	wait = pDev->pOwner->Cfg.Rate >= 80000U ? half / 2U - 2U : half / 4U;
#if defined(ESP32C2) || defined(ESP32C3)
	ESP32_I2C_CLK_CONF_REG(pDev->Base) = (pDev->Divider - 1U) |
		ESP32_I2C_CLK_CONF_SCLK_ACTIVE_Msk;
#else
#if defined(ESP32C6)
	if (pDev->Base == ESP32_I2C0_BASE)
#endif
	{
		ESP32_I2C_PCR_SCLK_CONF_REG = ((pDev->Divider - 1U) << ESP32_I2C_PCR_SCLK_DIV_NUM_Pos) |
			ESP32_I2C_PCR_SCLK_EN;
	}
#endif
	ESP32_I2C_SCL_LOW_PERIOD_REG(pDev->Base) = half - 1U;
	ESP32_I2C_SCL_HIGH_PERIOD_REG(pDev->Base) = (wait << 9) | (half - wait);
	ESP32_I2C_SDA_HOLD_REG(pDev->Base) = half / 4U - 1U;
	ESP32_I2C_SDA_SAMPLE_REG(pDev->Base) = half / 2U - 1U;
	ESP32_I2C_SCL_START_HOLD_REG(pDev->Base) = half - 1U;
	ESP32_I2C_SCL_RSTART_SETUP_REG(pDev->Base) = half - 1U;
	ESP32_I2C_SCL_STOP_HOLD_REG(pDev->Base) = half - 1U;
	ESP32_I2C_SCL_STOP_SETUP_REG(pDev->Base) = half - 1U;
	// Timeout is measured in powers of two module clock cycles, per SCL stretch.
	ESP32_I2C_TO_REG(pDev->Base) = pDev->bSlave ? 0 :
		(32U - __builtin_clz(5U * half) + 2U) | (1U << 5);
	ESP32_I2C_CTR_REG(pDev->Base) |= ESP32_I2C_CTR_CONF_UPGATE_Msk;
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
	uint32_t maxPre;
	uint32_t minHalf;

	dev = Esp32I2cHandle(pIntrf);
	if (dev == NULL || dev->bOpen)
	{
		return 0;
	}
	source = GetHighFreqOscFreq();
	maxPre = 256U;
	minHalf = 32U;
#if defined(ESP32C6)
	if (dev->Base == ESP32_LP_I2C_BASE)
	{
		// LP I2C runs directly from XTAL / 2, without the HP PCR divider.
		source /= 2U;
		maxPre = 1U;
		minHalf = 20U;
	}
#endif
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
	for (pre = 1; pre <= maxPre; pre++)
	{
		half = source / (2U * pre * Rate);
		if (half < minHalf)
		{
			half = minHalf;
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

static void Esp32I2cCommand(Esp32I2cDev_t *pDev, uint32_t Slot, uint32_t Opcode, uint32_t Count, uint32_t Ack)
{
	ESP32_I2C_COMD_REG(pDev->Base, Slot) = (Opcode << ESP32_I2C_COMD_OPCODE_Pos) | Count | Ack;
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
	ESP32_I2C_INT_CLR_REG(pDev->Base) = UINT32_MAX;
	ESP32_I2C_CTR_REG(pDev->Base) |= ESP32_I2C_CTR_CONF_UPGATE_Msk;
	ESP32_I2C_CTR_REG(pDev->Base) |= ESP32_I2C_CTR_TRANS_START_Msk;
	while (loops-- > 0)
	{
		status = ESP32_I2C_INT_RAW_REG(pDev->Base);
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
		Esp32I2cCommand(pDev, 0, ESP32_I2C_OPCODE_STOP, 0, 0);
		Esp32I2cRun(pDev, ESP32_I2C_INT_TRANS_COMPLETE, 0);
	}
	ESP32_I2C_CTR_REG(pDev->Base) |= ESP32_I2C_CTR_FSM_RST_Msk;
	Esp32I2cFifoReset(pDev);
	ESP32_I2C_INT_CLR_REG(pDev->Base) = UINT32_MAX;
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
		if (ESP32_I2C_SR_REG(dev->Base) & ESP32_I2C_SR_BUS_BUSY_Msk)
		{
			return false;
		}
	}
	Esp32I2cFifoReset(dev);
	ESP32_I2C_DATA_REG(dev->Base) = (Address << 1) | (Rx ? 1U : 0U);
	Esp32I2cCommand(dev, 0, ESP32_I2C_OPCODE_RSTART, 0, 0);
	Esp32I2cCommand(dev, 1, ESP32_I2C_OPCODE_WRITE, 1, ESP32_I2C_COMD_ACK_EN);
	Esp32I2cCommand(dev, 2, ESP32_I2C_OPCODE_END, 0, 0);
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
		if (chunk > (int)dev->FifoDepth)
		{
			chunk = dev->FifoDepth;
		}
		Esp32I2cFifoReset(dev);
		for (i = 0; i < chunk; i++)
		{
			ESP32_I2C_DATA_REG(dev->Base) = pData[count + i];
		}
		Esp32I2cCommand(dev, 0, ESP32_I2C_OPCODE_WRITE, chunk, ESP32_I2C_COMD_ACK_EN);
		Esp32I2cCommand(dev, 1, ESP32_I2C_OPCODE_END, 0, 0);
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
		if (chunk > (int)dev->FifoDepth)
		{
			chunk = dev->FifoDepth;
		}
		ack = chunk;
		if (count + chunk == Length)
		{
			ack--;
		}
		Esp32I2cFifoReset(dev);
		slot = 0;
		if (ack > 0)
		{
			Esp32I2cCommand(dev, slot, ESP32_I2C_OPCODE_READ, ack, 0);
			slot++;
		}
		if (ack != chunk)
		{
			Esp32I2cCommand(dev, slot, ESP32_I2C_OPCODE_READ, 1, ESP32_I2C_COMD_ACK_VAL);
			slot++;
		}
		Esp32I2cCommand(dev, slot, ESP32_I2C_OPCODE_END, 0, 0);
		if (!Esp32I2cRun(dev, ESP32_I2C_INT_END_DETECT, chunk))
		{
			break;
		}
		for (i = 0; i < chunk; i++)
		{
			pData[count + i] = ESP32_I2C_DATA_REG(dev->Base);
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
	available = (ESP32_I2C_SR_REG(pDev->Base) & ESP32_I2C_SR_RXFIFO_CNT_Msk) >> ESP32_I2C_SR_RXFIFO_CNT_Pos;
	while (available--)
	{
		value = ESP32_I2C_DATA_REG(pDev->Base);
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
	available = ESP32_I2C_FIFO_DEPTH - ((ESP32_I2C_SR_REG(pDev->Base) &
		ESP32_I2C_SR_TXFIFO_CNT_Msk) >> ESP32_I2C_SR_TXFIFO_CNT_Pos);
	while (available--)
	{
		value = i2c->pRRData[0] && pDev->SlaveQueued < i2c->RRDataLen[0] ?
			i2c->pRRData[0][pDev->SlaveQueued] : 0xff;
		ESP32_I2C_DATA_REG(pDev->Base) = value;
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
		remaining = (ESP32_I2C_SR_REG(pDev->Base) & ESP32_I2C_SR_TXFIFO_CNT_Msk) >> ESP32_I2C_SR_TXFIFO_CNT_Pos;
		pDev->SlaveCount = pDev->SlaveQueued - remaining;
		// The output register holds one prefetched byte outside the FIFO.
		// At STOP that byte has not been clocked out by the master.
		if (pDev->SlaveCount > 0)
		{
			pDev->SlaveCount--;
		}
		// Discard the rest of this response before another read address can
		// prefetch it. Resetting at READ_RQST is too late for that first byte.
		// Leave RX intact: a following command may already be arriving.
		ESP32_I2C_INT_ENA_REG(pDev->Base) &= ~ESP32_I2C_INT_TXFIFO_WM;
		ESP32_I2C_FIFO_CONF_REG(pDev->Base) |= ESP32_I2C_FIFO_CONF_TX_FIFO_RST_Msk;
		ESP32_I2C_FIFO_CONF_REG(pDev->Base) &= ~ESP32_I2C_FIFO_CONF_TX_FIFO_RST_Msk;
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
	bool readRequest;
	int previous;

	dev = &s_I2c[0];
	if (!dev->pOwner || !dev->bEnabled || !dev->bSlave)
	{
		return;
	}
	i2c = dev->pOwner;
	status = ESP32_I2C_INT_STATUS_REG(dev->Base);
	state = ESP32_I2C_SR_REG(dev->Base);
	ESP32_I2C_INT_CLR_REG(dev->Base) = status;
	cause = (state >> ESP32_I2C_SR_STRETCH_CAUSE_Pos) & 3U;
	readRequest = (status & ESP32_I2C_INT_SLAVE_STRETCH) && cause == 0 &&
		(state & ESP32_I2C_SR_SLAVE_RW_Msk);
	// Writes may reach the RX watermark or STOP without an address stretch.
	// Open the receive buffer before draining, including a command pending
	// when a repeated START addresses the slave for reading. Do not reset
	// the FIFO here: it already contains the received bytes.
	if (!dev->bOpen && (state & ESP32_I2C_SR_RXFIFO_CNT_Msk))
	{
		dev->bOpen = true;
		dev->bRx = true;
		dev->bError = false;
		dev->SlaveCount = 0;
		dev->SlaveQueued = 0;
		if (i2c->Cfg.EvtCB)
		{
			i2c->Cfg.EvtCB(&i2c->DevIntrf, DEVINTRF_EVT_WRITE_RQST,
				i2c->pTRBuff[0], 0);
		}
	}
	if (status & ESP32_I2C_INT_RXFIFO_OVF)
	{
		dev->bError = true;
	}
	// A repeated START hands the write count to READ_RQST below. Completing
	// that write first would discard the command context in the callback.
	if ((status & ESP32_I2C_INT_TRANS_COMPLETE) && !readRequest)
	{
		Esp32I2cSlaveComplete(dev);
		ESP32_I2C_INT_ENA_REG(dev->Base) &= ~ESP32_I2C_INT_TXFIFO_WM;
	}
	if (status & ESP32_I2C_INT_SLAVE_STRETCH)
	{
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
			Esp32I2cFifoReset(dev);
			if (i2c->Cfg.EvtCB)
			{
				i2c->Cfg.EvtCB(&i2c->DevIntrf,
					dev->bRx ? DEVINTRF_EVT_WRITE_RQST : DEVINTRF_EVT_READ_RQST,
					i2c->pTRBuff[0], previous);
			}
			if (!dev->bRx)
			{
				Esp32I2cSlaveFill(dev);
				ESP32_I2C_INT_ENA_REG(dev->Base) |= ESP32_I2C_INT_TXFIFO_WM;
			}
			else
			{
				ESP32_I2C_INT_ENA_REG(dev->Base) &= ~ESP32_I2C_INT_TXFIFO_WM;
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
		ESP32_I2C_CTR_REG(dev->Base) |= ESP32_I2C_CTR_CONF_UPGATE_Msk;
		ESP32_I2C_SCL_STRETCH_CONF_REG(dev->Base) |= ESP32_I2C_SLAVE_STRETCH_CLR_Msk;
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
	Esp32I2cClock(dev, true);
	ESP32_I2C_CTR_REG(dev->Base) = (dev->bSlave ? 0 : ESP32_I2C_CTR_MS_MODE_Msk) | ESP32_I2C_CTR_CLK_EN_Msk |
		ESP32_I2C_CTR_ARBITRATION_EN_Msk | ESP32_I2C_CTR_SDA_FORCE_OUT_Msk | ESP32_I2C_CTR_SCL_FORCE_OUT_Msk;
#if defined(ESP32C6)
	if (dev->Base == ESP32_LP_I2C_BASE)
	{
		ESP32_I2C_CTR_REG(dev->Base) &= ~(ESP32_I2C_CTR_MS_MODE_Msk |
			ESP32_I2C_CTR_SDA_FORCE_OUT_Msk | ESP32_I2C_CTR_SCL_FORCE_OUT_Msk);
	}
#endif
	ESP32_I2C_INT_ENA_REG(dev->Base) = 0;
	ESP32_I2C_INT_CLR_REG(dev->Base) = UINT32_MAX;
	// Seven source-clock cycles of glitch filtering on SDA and SCL.
	ESP32_I2C_FILTER_CFG_REG(dev->Base) = 7U | (7U << 4) | (3U << 8);
	Esp32I2cTiming(dev);
	Esp32I2cFifoReset(dev);
	#if defined(ESP32C6)
	if (dev->Base == ESP32_LP_I2C_BASE)
	{
		// Connect SCL before SDA to avoid a spurious START while changing muxes.
		IOPinCfg(dev->pOwner->Cfg.pIOPinMap + I2C_SCL_IOPIN_IDX, 1);
		IOPinCfg(dev->pOwner->Cfg.pIOPinMap + I2C_SDA_IOPIN_IDX, 1);
	}
	else
#endif
	{
		IOPinCfg(dev->pOwner->Cfg.pIOPinMap, dev->pOwner->Cfg.NbIOPins);
	}
	dev->bError = false;
	dev->bLost = false;
	dev->bEnabled = true;
#if !defined(ESP32C2)
	if (dev->bSlave)
	{
		ESP32_I2C_SLAVE_ADDR_REG(dev->Base) = dev->pOwner->Cfg.SlaveAddr[0];
		ESP32_I2C_CTR_REG(dev->Base) |= ESP32_I2C_CTR_SLV_TX_AUTO_START_Msk;
		ESP32_I2C_SCL_STRETCH_CONF_REG(dev->Base) = 0x3ffU | ESP32_I2C_SLAVE_STRETCH_EN_Msk;
		ESP32_I2C_TO_REG(dev->Base) = 0;
		ESP32_I2C_CTR_REG(dev->Base) |= ESP32_I2C_CTR_CONF_UPGATE_Msk;
		if (!Esp32EnableSourceIrq(ESP32_I2C_INTR_SOURCE_I2C0, I2C_CPUINT,
			dev->pOwner->Cfg.IntPrio, Esp32I2cSlaveIrq))
		{
			Esp32I2cDisable(pIntrf);
			return;
		}
		ESP32_I2C_INT_ENA_REG(dev->Base) = ESP32_I2C_INT_SLAVE_STRETCH |
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
	ESP32_I2C_INT_ENA_REG(dev->Base) = 0;
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
	Esp32I2cClock(dev, false);
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
	Esp32I2cDev_t *dev;

	dev = Esp32I2cHandle(pIntrf);
	if (dev)
	{
		Esp32I2cDisable(pIntrf);
		dev->pOwner = NULL;
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
	Esp32I2cDev_t *dev;
	int op;
	const IOPinCfg_t *pins;
	uintptr_t state;
	int i;

	if (pDev == NULL || pCfg == NULL || pCfg->DevNo < 0 ||
		pCfg->DevNo >= (int)(sizeof(s_I2c) / sizeof(s_I2c[0])) || pCfg->Type != I2CTYPE_STANDARD ||
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
	dev = &s_I2c[pCfg->DevNo];
#if defined(ESP32C6)
	if (pCfg->DevNo == 1 && pCfg->Mode != I2CMODE_MASTER)
	{
		return false;
	}
#endif
	pins = pCfg->pIOPinMap;
	for (i = 0; i < 2; i++)
	{
		op = i == I2C_SDA_IOPIN_IDX ? ESP32_PINOP_I2C0_SDA : ESP32_PINOP_I2C0_SCL;
#if defined(ESP32C6)
		if (pCfg->DevNo == 1)
		{
			if (pins[i].PinNo != 6 + i)
			{
				return false;
			}
			op = i == I2C_SDA_IOPIN_IDX ? ESP32_PINOP_LP_I2C_SDA : ESP32_PINOP_LP_I2C_SCL;
		}
#endif
		if (pins[i].PortNo != 0 || pins[i].PinNo < 0 || pins[i].PinNo > ESP32_GPIO_PIN_MAX ||
			pins[i].Res == IOPINRES_PULLDOWN || pins[i].PinDir != IOPINDIR_BI ||
			pins[i].Type != IOPINTYPE_OPENDRAIN ||
			pins[i].PinOp != op)
		{
			return false;
		}
	}
	if (pins[0].PinNo == pins[1].PinNo)
	{
		return false;
	}
	state = DisableInterrupt();
	if (dev->pOwner)
	{
		EnableInterrupt(state);
		return false;
	}
	for (i = 0; i < (int)(sizeof(s_I2c) / sizeof(s_I2c[0])); i++)
	{
		if (s_I2c[i].pOwner == pDev)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	dev->pOwner = pDev;
	dev->bSlave = pCfg->Mode == I2CMODE_SLAVE;
	EnableInterrupt(state);
	pDev->Cfg = *pCfg;
	intrf = &pDev->DevIntrf;
	intrf->pDevData = dev;
	intrf->Type = DEVINTRF_TYPE_I2C;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->bDma = false;
	intrf->bIntEn = dev->bSlave;
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
	if (!dev->bEnabled)
	{
		Esp32I2cPowerOff(intrf);
		return false;
	}
	return true;
}

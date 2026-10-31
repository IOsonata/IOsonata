/**-------------------------------------------------------------------------
@file	spi_samd.cpp

@brief	SAM D11/D21 SERCOM SPI master and slave

		DevNo is the SERCOM number. The pads come from the pins: data out
		and SCK on pads 0 and 1, 2 and 3, 3 and 1 or 0 and 3, data in on
		another pad. 4-wire SPI, 8-bit characters.

		Master: polling or interrupt, chip selects on GPIO pins from index
		SPI_CS_IOPIN_IDX of the pin map, driven by the driver with
		SPICSEL_AUTO. SCK is the main clock divided by 2 to 512, the closest
		rate to the request, 1 MHz for 0. In interrupt mode TxData and RxData
		return -1 and report the transfer with DEVINTRF_EVT_COMPLETED. The
		command phase of a read (bNoStop) is sent by polling. Data to send is
		copied to a buffer shared by the instances; data longer than the
		buffer, or sent while another instance uses it, is sent by polling and
		reported the same way.

		Master with bDmaEn: two DMAC channels, on the SERCOM RX (RXC) and TX
		(DRE) triggers, move the bytes in place of the CPU, taken at SPIInit
		(SPIInit fails without two free channels, and with SAMD_DMAC_NONE in
		the library project), given back at PowerOff: transfers return 0
		until SPIInit is called again.
		The receive channel is armed first, a direction without data of the
		application runs from the DummyByte or into a sink byte. The transfer
		ends when the receive channel has its last byte. Same modes as without
		DMA: with bIntEn the DMAC interrupt reports the end, the cases sent by
		polling above are sent by DMA with the CPU waiting, as all transfers
		without bIntEn. A buffer overflow or a channel transfer error ends the
		transfer with the count of bytes received. Blocks of at most 65535
		bytes, a longer transfer is done with the CPU waiting. FirstRdData is
		not updated by DMA transfers.

		Slave: interrupt, the hardware SS pad framing the transfer, the CS pin
		of index SPI_CS_IOPIN_IDX. DEVINTRF_EVT_STATECHG before each frame,
		while SS is high: the handler sets the buffers with
		SPISetSlaveRxBuffer and SPISetSlaveTxData. The first byte is loaded
		in the shift register then. DEVINTRF_EVT_COMPLETED when SS goes high
		with the count of bytes received; the SERCOM is reset there to drop
		the bytes loaded ahead, which takes a few microseconds before the
		next frame. Bytes past the receive buffer are dropped, the dummy byte
		goes out past the data. No DMA: SPIInit fails with bDmaEn.

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "samdxxx.h"

#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/spi.h"
#include "coredev/shared_intrf.h"
#include "iopinctrl.h"
#include "clock_samd.h"
#include "sercom_samd.h"
#include "dmac_samd.h"

// Bound of the register and transfer waits
#define SAMD_SPI_WAIT_LIMIT			100000UL

#define SAMD_SPI_RATE_DEFAULT		1000000U

// Interrupt transmit buffer, shared by the instances
#if SAMD_SERCOM_CNT > 3
#define SAMD_SPI_TXBUF_SIZE			256
#else
#define SAMD_SPI_TXBUF_SIZE			64
#endif

#define SAMD_SPIM_INTS				(SERCOM_SPIM_INTENSET_RXC_Msk | SERCOM_SPIM_INTENSET_ERROR_Msk)
#define SAMD_SPIS_INTS				(SERCOM_SPIS_INTENSET_DRE_Msk | SERCOM_SPIS_INTENSET_TXC_Msk | \
									 SERCOM_SPIS_INTENSET_RXC_Msk | SERCOM_SPIS_INTENSET_ERROR_Msk)
#define SAMD_SPI_ALL_INTS			(SERCOM_SPIM_INTENCLR_DRE_Msk | SERCOM_SPIM_INTENCLR_TXC_Msk | \
									 SERCOM_SPIM_INTENCLR_RXC_Msk | SERCOM_SPIM_INTENCLR_SSL_Msk | \
									 SERCOM_SPIM_INTENCLR_ERROR_Msk)

typedef enum {
	SAMD_SPI_OP_NONE,
	SAMD_SPI_OP_TX,
	SAMD_SPI_OP_RX,
} SAMD_SPI_OP;

#pragma pack(push, 4)

typedef struct {
	int DevNo;						//!< SERCOM number
	sercom_spim_registers_t *pReg;
	SPIDev_t *pSpiDev;				//!< Generic SPI device data
	uint32_t CtrlA;					//!< CTRLA without ENABLE
	uint8_t Baud;					//!< Master BAUD
	int NbCs;						//!< Master chip select pins
	volatile SAMD_SPI_OP Op;		//!< Master interrupt transfer
	const uint8_t *pTxData;
	uint8_t *pRxData;
	int Length;
	int Count;
	uint8_t *pSlaveRx;				//!< Slave buffers of the frame
	int SlaveRxLen;
	const uint8_t *pSlaveTx;
	int SlaveTxLen;
	int SlaveTxLoaded;				//!< Slave bytes written to DATA in the frame
	int SlaveCount;					//!< Slave bytes received in the frame
	bool bSlaveSel;					//!< Slave SS went low in the frame
	bool bDma;						//!< DmaRx and DmaTx taken
	volatile bool bDmaRun;			//!< Master DMA transfer ended by the DMAC interrupt
	uint8_t Sink;					//!< Received bytes nobody wants
	int8_t DmaRx;					//!< DMAC channels of the master
	int8_t DmaTx;
} SamdSpiDev_t;

#pragma pack(pop)

// Master DMA, false with SAMD_DMAC_NONE: the DMA code is left out
static inline __attribute__((always_inline)) bool SamdSpiDmaOn(const SamdSpiDev_t *pDev)
{
	return SAMD_DMAC_ON && pDev->bDma;
}

static SamdSpiDev_t s_SamdSpiDev[SAMD_SERCOM_CNT];

static uint8_t s_SamdSpiTxBuf[SAMD_SPI_TXBUF_SIZE];
static SamdSpiDev_t *s_pSamdSpiTxOwner;

static bool SamdSpiSync(sercom_spim_registers_t *pReg, uint32_t Mask)
{
	for (uint32_t i = 0; i < SAMD_SPI_WAIT_LIMIT; i++)
	{
		if ((pReg->SERCOM_SYNCBUSY & Mask) == 0U)
		{
			return true;
		}
	}

	return false;
}

static bool SamdSpiWaitFlag(sercom_spim_registers_t *pReg, uint8_t Flag)
{
	for (uint32_t i = 0; i < SAMD_SPI_WAIT_LIMIT; i++)
	{
		if ((pReg->SERCOM_INTFLAG & Flag) != 0U)
		{
			return true;
		}
	}

	return false;
}

// DOPO of the data out and SCK pads, with the slave select pad
static int SamdSpiDopo(int DoPad, int SckPad, int *pSsPad)
{
	static const int8_t s_Pads[4][3] = {
		// DO, SCK, SS
		{ 0, 1, 2 },
		{ 2, 3, 1 },
		{ 3, 1, 2 },
		{ 0, 3, 1 },
	};

	for (int i = 0; i < 4; i++)
	{
		if (s_Pads[i][0] == DoPad && s_Pads[i][1] == SckPad)
		{
			*pSsPad = s_Pads[i][2];

			return i;
		}
	}

	return -1;
}

// Pad routing of CTRLA from the pin map, 0 when the pins do not fit
static uint32_t SamdSpiPads(const SPICfg_t *pCfg)
{
	const IOPinCfg_t *pins = pCfg->pIOPinMap;
	bool slave = pCfg->Mode == SPIMODE_SLAVE;
	int sck = SamdSercomPad(pCfg->DevNo, &pins[SPI_SCK_IOPIN_IDX]);
	int miso = SamdSercomPad(pCfg->DevNo, &pins[SPI_MISO_IOPIN_IDX]);
	int mosi = SamdSercomPad(pCfg->DevNo, &pins[SPI_MOSI_IOPIN_IDX]);
	int dout = slave ? miso : mosi;
	int din = slave ? mosi : miso;
	int ss;

	if (sck < 0 || miso < 0 || mosi < 0)
	{
		return 0;
	}

	int dopo = SamdSpiDopo(dout, sck, &ss);

	if (dopo < 0 || din == dout || din == sck || (slave && din == ss))
	{
		return 0;
	}

	if (slave && SamdSercomPad(pCfg->DevNo, &pins[SPI_CS_IOPIN_IDX]) != ss)
	{
		return 0;
	}

	return SERCOM_SPIM_CTRLA_DOPO((uint32_t)dopo) | SERCOM_SPIM_CTRLA_DIPO((uint32_t)din);
}

// Closest SCK, the lower one when two are as close
static uint32_t SamdSpiBaud(uint32_t Rate, uint8_t *pBaud)
{
	uint32_t fclk = SystemPeriphClockGet(SAMD_GCLK_MAIN);
	uint32_t best = 0;
	uint32_t bestdiff = UINT32_MAX;

	if (fclk == 0U)
	{
		return 0;
	}
	if (Rate == 0U)
	{
		Rate = SAMD_SPI_RATE_DEFAULT;
	}

	// fSCK = fclk / (2 * (BAUD + 1)), the two candidates around the request
	uint32_t div = fclk / (2U * Rate);

	for (uint32_t b = div > 0U ? div - 1U : 0U; b <= div && b <= 255U; b++)
	{
		uint32_t f = fclk / (2U * (b + 1U));
		uint32_t diff = f > Rate ? f - Rate : Rate - f;

		if (diff < bestdiff || (diff == bestdiff && f < best))
		{
			bestdiff = diff;
			best = f;
			*pBaud = (uint8_t)b;
		}
	}

	if (best == 0U)
	{
		*pBaud = 255U;
		best = fclk / 512U;
	}

	return best;
}

static inline const IOPinCfg_t *SamdSpiCsPin(SamdSpiDev_t *pDev, int DevCs)
{
	return &pDev->pSpiDev->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + DevCs];
}

static void SamdSpiCsSet(SamdSpiDev_t *pDev, bool bActive)
{
	SPIDev_t *spi = pDev->pSpiDev;

	if (spi->Cfg.ChipSel != SPICSEL_AUTO || spi->CurDevCs < 0 || spi->CurDevCs >= pDev->NbCs)
	{
		return;
	}

	const IOPinCfg_t *cs = SamdSpiCsPin(pDev, spi->CurDevCs);

	if (bActive)
	{
		IOPinClear(cs->PortNo, cs->PinNo);
	}
	else
	{
		IOPinSet(cs->PortNo, cs->PinNo);
	}
}

// Configuration, from the SERCOM reset. The SERCOM is enabled.
static bool SamdSpiHwInit(SamdSpiDev_t *pDev)
{
	sercom_spim_registers_t *reg = pDev->pReg;
	bool slave = pDev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE;

	reg->SERCOM_CTRLA = SERCOM_SPIM_CTRLA_SWRST_Msk;
	if (!SamdSpiSync(reg, SERCOM_SPIM_SYNCBUSY_SWRST_Msk))
	{
		return false;
	}

	reg->SERCOM_CTRLA = pDev->CtrlA;
	// 8-bit characters. The slave loads the first byte while SS is high and
	// flags SS going low (SSL).
	reg->SERCOM_CTRLB = SERCOM_SPIM_CTRLB_RXEN_Msk |
						(slave ? SERCOM_SPIS_CTRLB_PLOADEN_Msk | SERCOM_SPIS_CTRLB_SSDE_Msk : 0U);
	if (!slave)
	{
		reg->SERCOM_BAUD = pDev->Baud;
	}
	reg->SERCOM_CTRLA = pDev->CtrlA | SERCOM_SPIM_CTRLA_ENABLE_Msk;

	return SamdSpiSync(reg, SERCOM_SPIM_SYNCBUSY_ENABLE_Msk | SERCOM_SPIM_SYNCBUSY_CTRLB_Msk);
}

// Slave buffers of the next frame, its first byte loaded while SS is high
static void SamdSpisArm(SamdSpiDev_t *pDev)
{
	SPIDev_t *spi = pDev->pSpiDev;
	DevIntrf_t *intrf = &spi->DevIntrf;
	sercom_spim_registers_t *reg = pDev->pReg;

	if (intrf->EvtCB)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_STATECHG, NULL, 0);
	}

	pDev->pSlaveRx = spi->pRxBuff[0];
	pDev->SlaveRxLen = pDev->pSlaveRx != NULL && spi->RxBuffLen[0] > 0 ? spi->RxBuffLen[0] : 0;
	pDev->pSlaveTx = spi->pTxData[0];
	pDev->SlaveTxLen = pDev->pSlaveTx != NULL && spi->TxDataLen[0] > 0 ? spi->TxDataLen[0] : 0;
	pDev->SlaveCount = 0;
	pDev->SlaveTxLoaded = 1;
	pDev->bSlaveSel = false;

	reg->SERCOM_DATA = pDev->SlaveTxLen > 0 ? pDev->pSlaveTx[0] : spi->Cfg.DummyByte;
	reg->SERCOM_INTFLAG = SERCOM_SPIS_INTFLAG_TXC_Msk | SERCOM_SPIS_INTFLAG_ERROR_Msk |
						  SERCOM_SPIS_INTFLAG_SSL_Msk;
	reg->SERCOM_STATUS = SERCOM_SPIS_STATUS_BUFOVF_Msk;
	reg->SERCOM_INTENSET = SAMD_SPIS_INTS;
}

// The receive buffer holds two bytes
static void SamdSpisDrain(SamdSpiDev_t *pDev)
{
	sercom_spim_registers_t *reg = pDev->pReg;

	while ((reg->SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_RXC_Msk) != 0U)
	{
		uint8_t data = (uint8_t)reg->SERCOM_DATA;

		if (pDev->SlaveCount < pDev->SlaveRxLen)
		{
			pDev->pSlaveRx[pDev->SlaveCount] = data;
		}
		pDev->SlaveCount++;
	}
}

// End of a frame, SS high. The last byte can be received after TXC is set.
// The reset drops the bytes loaded for this frame and not sent.
static void SamdSpisEnd(SamdSpiDev_t *pDev)
{
	sercom_spim_registers_t *reg = pDev->pReg;
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;

	SamdSpisDrain(pDev);

	int count = pDev->SlaveCount;

	reg->SERCOM_INTENCLR = SAMD_SPI_ALL_INTS;
	(void)SamdSpiHwInit(pDev);

	if (intrf->EvtCB)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, pDev->pSlaveRx, count);
	}

	if (intrf->EnCnt > 0)
	{
		SamdSpisArm(pDev);
	}
}

static void SamdSpisIrq(SamdSpiDev_t *pDev)
{
	sercom_spim_registers_t *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;

	SamdSpisDrain(pDev);

	if ((reg->SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_ERROR_Msk) != 0U)
	{
		reg->SERCOM_STATUS = SERCOM_SPIS_STATUS_BUFOVF_Msk;
		reg->SERCOM_INTFLAG = SERCOM_SPIS_INTFLAG_ERROR_Msk;
	}

	// SS high: end of the frame
	if ((reg->SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_TXC_Msk) != 0U)
	{
		SamdSpisEnd(pDev);

		return;
	}

	// One byte ahead of the shift register
	if ((reg->SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_DRE_Msk) != 0U)
	{
		reg->SERCOM_DATA = pDev->SlaveTxLoaded < pDev->SlaveTxLen ? pDev->pSlaveTx[pDev->SlaveTxLoaded] :
						   spi->Cfg.DummyByte;
		pDev->SlaveTxLoaded++;

		// The DATA write clears TXC: SS high once it went low in the frame
		// is the end of the frame. SSL is read after the write, for a frame
		// that started and ended before it. Before the frame, SS is high
		// while the second byte is loaded.
		if ((reg->SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_SSL_Msk) != 0U)
		{
			reg->SERCOM_INTFLAG = SERCOM_SPIS_INTFLAG_SSL_Msk;
			pDev->bSlaveSel = true;
		}

		const IOPinCfg_t *ss = SamdSpiCsPin(pDev, 0);

		if ((pDev->bSlaveSel || pDev->SlaveCount > 0) && IOPinRead(ss->PortNo, ss->PinNo) != 0)
		{
			SamdSpisEnd(pDev);
		}
	}
}

static int SamdSpimPolling(SamdSpiDev_t *pDev, const uint8_t *pTx, uint8_t *pRx, int Length)
{
	sercom_spim_registers_t *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	int count = 0;

	while (count < Length)
	{
		if (!SamdSpiWaitFlag(reg, SERCOM_SPIM_INTFLAG_DRE_Msk))
		{
			break;
		}
		reg->SERCOM_DATA = pTx != NULL ? pTx[count] : spi->Cfg.DummyByte;

		if (!SamdSpiWaitFlag(reg, SERCOM_SPIM_INTFLAG_RXC_Msk))
		{
			break;
		}

		uint8_t data = (uint8_t)reg->SERCOM_DATA;

		if (pRx != NULL)
		{
			pRx[count] = data;
		}
		else if (count == 0)
		{
			spi->FirstRdData = data;
		}
		count++;
	}

	return count;
}

// Channels of a master DMA transfer stopped. Returns the bytes the receive
// channel did not move.
static int SamdSpimDmaEnd(SamdSpiDev_t *pDev)
{
	int left = (int)SamdDmacStop(pDev->DmaRx);

	(void)SamdDmacStop(pDev->DmaTx);

	return left;
}

// Master DMA block: stale received bytes and overflow cleared, the receive
// channel armed, then the transmit one. A direction without data runs from
// the DummyByte or into Sink. With bErrInt the SERCOM interrupts on an
// overflow.
static void SamdSpimDmaStart(SamdSpiDev_t *pDev, const uint8_t *pTx, uint8_t *pRx, int Length, bool bErrInt)
{
	sercom_spim_registers_t *reg = pDev->pReg;

	while ((reg->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC_Msk) != 0U)
	{
		(void)reg->SERCOM_DATA;
	}
	reg->SERCOM_STATUS = SERCOM_SPIM_STATUS_BUFOVF_Msk;
	reg->SERCOM_INTFLAG = SERCOM_SPIM_INTFLAG_ERROR_Msk;
	if (bErrInt)
	{
		reg->SERCOM_INTENSET = SERCOM_SPIM_INTENSET_ERROR_Msk;
	}

	SamdDmacStart(pDev->DmaRx, &reg->SERCOM_DATA, false, pRx != NULL ? pRx : &pDev->Sink, pRx != NULL, (uint32_t)Length);
	SamdDmacStart(pDev->DmaTx, pTx != NULL ? pTx : &pDev->pSpiDev->Cfg.DummyByte, pTx != NULL,
				  &reg->SERCOM_DATA, false, (uint32_t)Length);
}

// Master DMA transfer, the CPU waiting, in blocks of at most
// SAMD_DMAC_COUNT_MAX bytes. The wait of a block is bounded from the byte
// time, 16 x (BAUD + 1) clocks, more than one loop pass each. Returns the
// bytes received.
static int SamdSpimDmaWait(SamdSpiDev_t *pDev, const uint8_t *pTx, uint8_t *pRx, int Length)
{
	sercom_spim_registers_t *reg = pDev->pReg;
	int count = 0;

	while (count < Length)
	{
		int len = Length - count < SAMD_DMAC_COUNT_MAX ? Length - count : SAMD_DMAC_COUNT_MAX;
		uint32_t limit = (uint32_t)len * 16U * ((uint32_t)pDev->Baud + 1U) + SAMD_SPI_WAIT_LIMIT;

		SamdSpimDmaStart(pDev, pTx != NULL ? pTx + count : NULL, pRx != NULL ? pRx + count : NULL, len, false);

		for (uint32_t i = 0; i < limit && SamdDmacBusy(pDev->DmaRx); i++)
		{
			if ((reg->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_ERROR_Msk) != 0U)
			{
				break;
			}
		}

		int left = SamdSpimDmaEnd(pDev);

		count += len - left;
		if (left != 0)
		{
			break;
		}
	}

	return count;
}

// Transfer with the CPU waiting: DMA with bDmaEn, polling otherwise
static int SamdSpimSync(SamdSpiDev_t *pDev, const uint8_t *pTx, uint8_t *pRx, int Length)
{
	if (SamdSpiDmaOn(pDev))
	{
		return SamdSpimDmaWait(pDev, pTx, pRx, Length);
	}

	return SamdSpimPolling(pDev, pTx, pRx, Length);
}

// End of the master transfer taken by the first of the DMAC interrupt, the
// SERCOM interrupt and the caller, interrupts disabled: SERCOM interrupts
// off, a DMA transfer stopped with the count of bytes its receive channel
// moved. Returns the operation, none when another path took it.
static SAMD_SPI_OP SamdSpimTake(SamdSpiDev_t *pDev)
{
	SAMD_SPI_OP op = pDev->Op;

	if (op != SAMD_SPI_OP_NONE)
	{
		pDev->pReg->SERCOM_INTENCLR = SAMD_SPI_ALL_INTS;
		if (SAMD_DMAC_ON && pDev->bDmaRun)
		{
			pDev->bDmaRun = false;
			pDev->Count = pDev->Length - SamdSpimDmaEnd(pDev);
		}
		pDev->Op = SAMD_SPI_OP_NONE;
		if (s_pSamdSpiTxOwner == pDev)
		{
			s_pSamdSpiTxOwner = NULL;
		}
	}

	return op;
}

// Transfer reported, after SamdSpimTake
static void SamdSpimFinish(SamdSpiDev_t *pDev, SAMD_SPI_OP Op, int Count)
{
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;

	if (Op == SAMD_SPI_OP_TX)
	{
		DeviceIntrfTxComplete(intrf);
	}
	else
	{
		DeviceIntrfStopRx(intrf);
	}

	if (intrf->EvtCB)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, NULL, Count);
	}
}

static void SamdSpimComplete(SamdSpiDev_t *pDev)
{
	uint32_t state = DisableInterrupt();
	SAMD_SPI_OP op = SamdSpimTake(pDev);
	int count = pDev->Count;

	EnableInterrupt(state);

	if (op != SAMD_SPI_OP_NONE)
	{
		SamdSpimFinish(pDev, op, count);
	}
}

// Byte by byte on the receive interrupt
static int SamdSpimInt(SamdSpiDev_t *pDev, const uint8_t *pTx, uint8_t *pRx, int Length)
{
	sercom_spim_registers_t *reg = pDev->pReg;
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;

	if (pTx != NULL)
	{
		uint32_t state = DisableInterrupt();
		bool buff = s_pSamdSpiTxOwner == NULL && Length <= SAMD_SPI_TXBUF_SIZE;

		if (buff)
		{
			s_pSamdSpiTxOwner = pDev;
		}
		EnableInterrupt(state);

		intrf->bTxReady = false;

		if (!buff)
		{
			pDev->Count = SamdSpimSync(pDev, pTx, NULL, Length);
			pDev->Op = SAMD_SPI_OP_TX;
			SamdSpimComplete(pDev);

			return -1;
		}

		// The caller data does not outlive the call
		memcpy(s_SamdSpiTxBuf, pTx, (size_t)Length);
		pTx = s_SamdSpiTxBuf;
	}
	else if (SamdSpiDmaOn(pDev) && Length > SAMD_DMAC_COUNT_MAX)
	{
		// Longer than a block
		pDev->Count = SamdSpimSync(pDev, NULL, pRx, Length);
		pDev->Op = SAMD_SPI_OP_RX;
		SamdSpimComplete(pDev);

		return -1;
	}

	pDev->pTxData = pTx;
	pDev->pRxData = pRx;
	pDev->Length = Length;
	pDev->Count = 0;
	pDev->Op = pTx != NULL ? SAMD_SPI_OP_TX : SAMD_SPI_OP_RX;

	if (SamdSpiDmaOn(pDev))
	{
		// The DMAC interrupt of the receive channel ends it
		pDev->bDmaRun = true;
		SamdSpimDmaStart(pDev, pTx, pRx, Length, true);

		return -1;
	}

	// Stale received bytes
	while ((reg->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC_Msk) != 0U)
	{
		(void)reg->SERCOM_DATA;
	}

	reg->SERCOM_DATA = pTx != NULL ? pTx[0] : pDev->pSpiDev->Cfg.DummyByte;
	reg->SERCOM_INTENSET = SAMD_SPIM_INTS;

	return -1;
}

static void SamdSpimIrq(SamdSpiDev_t *pDev)
{
	sercom_spim_registers_t *reg = pDev->pReg;

	if (pDev->Op == SAMD_SPI_OP_NONE)
	{
		reg->SERCOM_INTENCLR = SAMD_SPI_ALL_INTS;

		return;
	}

	if ((reg->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_ERROR_Msk) != 0U)
	{
		// The flag read again and the transfer taken with interrupts
		// disabled: a DMAC interrupt in between could end this transfer and
		// start the next one
		uint32_t state = DisableInterrupt();
		SAMD_SPI_OP op = SAMD_SPI_OP_NONE;
		int count = 0;

		if ((reg->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_ERROR_Msk) != 0U)
		{
			reg->SERCOM_STATUS = SERCOM_SPIM_STATUS_BUFOVF_Msk;
			reg->SERCOM_INTFLAG = SERCOM_SPIM_INTFLAG_ERROR_Msk;
			op = SamdSpimTake(pDev);
			count = pDev->Count;
		}
		EnableInterrupt(state);

		if (op != SAMD_SPI_OP_NONE)
		{
			SamdSpimFinish(pDev, op, count);
		}

		return;
	}

	if (pDev->bDmaRun || (reg->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC_Msk) == 0U)
	{
		return;
	}

	uint8_t data = (uint8_t)reg->SERCOM_DATA;

	if (pDev->pRxData != NULL)
	{
		pDev->pRxData[pDev->Count] = data;
	}
	else if (pDev->Count == 0)
	{
		pDev->pSpiDev->FirstRdData = data;
	}
	pDev->Count++;

	if (pDev->Count < pDev->Length)
	{
		reg->SERCOM_DATA = pDev->pTxData != NULL ? pDev->pTxData[pDev->Count] : pDev->pSpiDev->Cfg.DummyByte;
	}
	else
	{
		SamdSpimComplete(pDev);
	}
}

// DMAC interrupt of a master channel: the end of the receive channel, or a
// transfer error of either, ends a transfer started with the interrupt
static void SamdSpimDmaHandler(int Chan, uint32_t Flags, void *pCtx)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pCtx;

	if (!dev->bDmaRun || (Chan == dev->DmaTx && (Flags & DMAC_CHINTFLAG_TERR_Msk) == 0U))
	{
		return;
	}

	SamdSpimComplete(dev);
}

// Master DMAC channels, both or none. The interrupt with bIntEn.
static bool SamdSpiDmaTake(SamdSpiDev_t *pDev)
{
	SPIDev_t *spi = pDev->pSpiDev;
	SamdDmacHandler_t handler = spi->Cfg.bIntEn ? SamdSpimDmaHandler : NULL;

	pDev->DmaRx = (int8_t)SamdDmacOpen(SAMD_DMAC_TRIG_SERCOM_RX(pDev->DevNo), handler, pDev, spi->Cfg.IntPrio);
	pDev->DmaTx = -1;
	if (pDev->DmaRx >= 0)
	{
		pDev->DmaTx = (int8_t)SamdDmacOpen(SAMD_DMAC_TRIG_SERCOM_TX(pDev->DevNo), handler, pDev, spi->Cfg.IntPrio);
	}
	if (pDev->DmaTx < 0)
	{
		SamdDmacClose(pDev->DmaRx);
		pDev->DmaRx = -1;

		return false;
	}
	pDev->bDma = true;

	return true;
}

static void SamdSpiDmaGive(SamdSpiDev_t *pDev)
{
	if (SamdSpiDmaOn(pDev))
	{
		pDev->bDma = false;
		pDev->bDmaRun = false;
		SamdDmacClose(pDev->DmaRx);
		SamdDmacClose(pDev->DmaTx);
		pDev->DmaRx = -1;
		pDev->DmaTx = -1;
	}
}

static void SamdSpiIrqHandler(int DevNo, DevIntrf_t * const pDev)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;

	(void)DevNo;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		SamdSpisIrq(dev);
	}
	else
	{
		SamdSpimIrq(dev);
	}
}

static uint32_t SamdSpiGetRate(DevIntrf_t * const pDev)
{
	return ((SamdSpiDev_t *)pDev->pDevData)->pSpiDev->Cfg.Rate;
}

// Master: the SERCOM is stopped for the change. Slave: the master sets SCK.
static uint32_t SamdSpiSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;
	sercom_spim_registers_t *reg = dev->pReg;
	uint8_t baud;

	if (spi->Cfg.Mode == SPIMODE_SLAVE)
	{
		return spi->Cfg.Rate;
	}

	uint32_t rate = SamdSpiBaud(Rate, &baud);

	if (rate == 0U || dev->Op != SAMD_SPI_OP_NONE)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	bool enabled = (reg->SERCOM_CTRLA & SERCOM_SPIM_CTRLA_ENABLE_Msk) != 0U;

	dev->Baud = baud;
	spi->Cfg.Rate = rate;

	// BAUD is written with the SPI disabled
	reg->SERCOM_CTRLA = dev->CtrlA;
	(void)SamdSpiSync(reg, SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);
	reg->SERCOM_BAUD = baud;
	if (enabled)
	{
		reg->SERCOM_CTRLA = dev->CtrlA | SERCOM_SPIM_CTRLA_ENABLE_Msk;
		(void)SamdSpiSync(reg, SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);
	}

	EnableInterrupt(state);

	return rate;
}

static bool SamdSpiStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;

	if (spi->Cfg.Mode != SPIMODE_MASTER || dev->Op != SAMD_SPI_OP_NONE ||
		(spi->Cfg.ChipSel == SPICSEL_AUTO && (int)DevAddr >= dev->NbCs))
	{
		return false;
	}

	spi->CurDevCs = (int)DevAddr;
	SamdSpiCsSet(dev, true);

	return true;
}

// Also the receive phase of DeviceIntrfRead, the chip select stays active
static bool SamdSpiStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return SamdSpiStartTx(pDev, DevAddr);
}

static int SamdSpiTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;

	// bDmaEn without the channels: given back by PowerOff
	if (pData == NULL || DataLen <= 0 || dev->pSpiDev->Cfg.Mode != SPIMODE_MASTER ||
		dev->pSpiDev->Cfg.bDmaEn != dev->bDma)
	{
		return 0;
	}

	if (pDev->bIntEn && !pDev->bNoStop)
	{
		return SamdSpimInt(dev, pData, NULL, DataLen);
	}

	return SamdSpimSync(dev, pData, NULL, DataLen);
}

static int SamdSpiRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;

	if (pBuff == NULL || BuffLen <= 0 || dev->pSpiDev->Cfg.Mode != SPIMODE_MASTER ||
		dev->pSpiDev->Cfg.bDmaEn != dev->bDma)
	{
		return 0;
	}

	if (pDev->bIntEn)
	{
		return SamdSpimInt(dev, NULL, pBuff, BuffLen);
	}

	return SamdSpimSync(dev, NULL, pBuff, BuffLen);
}

static void SamdSpiStopTx(DevIntrf_t * const pDev)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_MASTER && !pDev->bNoStop)
	{
		SamdSpiCsSet(dev, false);
	}
}

static void SamdSpiStopRx(DevIntrf_t * const pDev)
{
	SamdSpiStopTx(pDev);
}

static void SamdSpiDisable(DevIntrf_t * const pDev)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;
	sercom_spim_registers_t *reg = dev->pReg;

	uint32_t state = DisableInterrupt();

	reg->SERCOM_INTENCLR = SAMD_SPI_ALL_INTS;
	if (SamdSpiDmaOn(dev))
	{
		// Channels stopped, a transfer started with the interrupt is not
		// reported: Reset ends it
		dev->bDmaRun = false;
		(void)SamdSpimDmaEnd(dev);
	}
	EnableInterrupt(state);

	reg->SERCOM_CTRLA = dev->CtrlA;
	(void)SamdSpiSync(reg, SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);
	SamdSercomClockDisable(dev->DevNo);
}

static void SamdSpiEnable(DevIntrf_t * const pDev)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;

	if (!SamdSercomClockEnable(dev->DevNo))
	{
		return;
	}
	IOPinCfg(spi->Cfg.pIOPinMap, spi->Cfg.NbIOPins);

	uint32_t state = DisableInterrupt();

	if (SamdSpiHwInit(dev) && spi->Cfg.Mode == SPIMODE_SLAVE)
	{
		SamdSpisArm(dev);
	}

	EnableInterrupt(state);
}

// Configured again, a slave waits for the next frame with its buffers set
// again
static void SamdSpiReset(DevIntrf_t * const pDev)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pReg->SERCOM_INTENCLR = SAMD_SPI_ALL_INTS;
	if (SamdSpiDmaOn(dev))
	{
		dev->bDmaRun = false;
		(void)SamdSpimDmaEnd(dev);
	}
	dev->Op = SAMD_SPI_OP_NONE;
	if (s_pSamdSpiTxOwner == dev)
	{
		s_pSamdSpiTxOwner = NULL;
	}

	if (SamdSpiHwInit(dev) && dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		SamdSpisArm(dev);
	}

	EnableInterrupt(state);
}

static void SamdSpiPowerOff(DevIntrf_t * const pDev)
{
	SamdSpiDev_t *dev = (SamdSpiDev_t *)pDev->pDevData;

	SamdSpiDisable(pDev);

	uint32_t state = DisableInterrupt();

	SamdSpiDmaGive(dev);
	EnableInterrupt(state);

	IOPinDis(dev->pSpiDev->Cfg.pIOPinMap, dev->pSpiDev->Cfg.NbIOPins);
}

static void *SamdSpiGetHandle(DevIntrf_t * const pDev)
{
	return ((SamdSpiDev_t *)pDev->pDevData)->pSpiDev;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL || pCfgData->pIOPinMap == NULL ||
		pCfgData->DevNo < 0 || pCfgData->DevNo >= SAMD_SERCOM_CNT || pCfgData->Phy != SPIPHY_NORMAL ||
		(pCfgData->Mode != SPIMODE_MASTER && pCfgData->Mode != SPIMODE_SLAVE) ||
		pCfgData->DataSize != 8U || (pCfgData->bDmaEn && pCfgData->Mode == SPIMODE_SLAVE))
	{
		return false;
	}

	bool slave = pCfgData->Mode == SPIMODE_SLAVE;

	// The slave needs its SS pin and runs on interrupts
	if (pCfgData->NbIOPins < (slave ? SPI_CS_IOPIN_IDX + 1 : SPI_CS_IOPIN_IDX) ||
		(slave && pCfgData->ChipSel != SPICSEL_AUTO))
	{
		return false;
	}

	bool intEn = slave || pCfgData->bIntEn;

	if (intEn && pCfgData->IntPrio < 0)
	{
		return false;
	}

	uint32_t pads = SamdSpiPads(pCfgData);

	if (pads == 0U)
	{
		return false;
	}

	SamdSpiDev_t *dev = &s_SamdSpiDev[pCfgData->DevNo];
	uint32_t state = DisableInterrupt();

	if (SamdSercomOwned(pCfgData->DevNo, &pDev->DevIntrf) || !SamdSercomClockEnable(pCfgData->DevNo))
	{
		EnableInterrupt(state);

		return false;
	}

	if (s_pSamdSpiTxOwner == dev)
	{
		s_pSamdSpiTxOwner = NULL;
	}

	IRQn_Type irq = SamdSercomIrq(pCfgData->DevNo);

	NVIC_DisableIRQ(irq);

	// Channels of an earlier SPIInit given back
	SamdSpiDmaGive(dev);
	memset(dev, 0, sizeof(SamdSpiDev_t));
	dev->DevNo = pCfgData->DevNo;
	dev->pReg = &SamdSercomGetReg(pCfgData->DevNo)->SPIM;
	dev->pSpiDev = pDev;
	dev->NbCs = slave ? 0 : pCfgData->NbIOPins - SPI_CS_IOPIN_IDX;
	dev->DmaRx = -1;
	dev->DmaTx = -1;

	memcpy(&pDev->Cfg, pCfgData, sizeof(SPICfg_t));
	pDev->Cfg.bIntEn = intEn;

	// No channel with SAMD_DMAC_NONE
	if (pCfgData->bDmaEn && (!SAMD_DMAC_ON || !SamdSpiDmaTake(dev)))
	{
		// The SERCOM released as by a failed configuration below
		SharedIntrfSetIrqHandler(pCfgData->DevNo, NULL, NULL);
		SamdSercomClockDisable(pCfgData->DevNo);
		EnableInterrupt(state);

		return false;
	}
	pDev->FirstRdData = 0;
	pDev->CurDevCs = 0;
	memset(pDev->pRxBuff, 0, sizeof(pDev->pRxBuff));
	memset(pDev->RxBuffLen, 0, sizeof(pDev->RxBuffLen));
	memset(pDev->pTxData, 0, sizeof(pDev->pTxData));
	memset(pDev->TxDataLen, 0, sizeof(pDev->TxDataLen));

	dev->CtrlA = (slave ? SERCOM_SPIS_CTRLA_MODE_SPI_SLAVE : SERCOM_SPIM_CTRLA_MODE_SPI_MASTER) | pads |
				 (pCfgData->ClkPol == SPICLKPOL_LOW ? SERCOM_SPIM_CTRLA_CPOL_Msk : 0U) |
				 (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK ? SERCOM_SPIM_CTRLA_CPHA_Msk : 0U) |
				 (pCfgData->BitOrder == SPIDATABIT_LSB ? SERCOM_SPIM_CTRLA_DORD_Msk : 0U);

	if (!slave)
	{
		pDev->Cfg.Rate = SamdSpiBaud(pCfgData->Rate, &dev->Baud);
	}

	pDev->DevIntrf.pDevData = dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.bIntEn = intEn;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.Disable = SamdSpiDisable;
	pDev->DevIntrf.Enable = SamdSpiEnable;
	pDev->DevIntrf.GetRate = SamdSpiGetRate;
	pDev->DevIntrf.SetRate = SamdSpiSetRate;
	pDev->DevIntrf.StartRx = SamdSpiStartRx;
	pDev->DevIntrf.RxData = SamdSpiRxData;
	pDev->DevIntrf.StopRx = SamdSpiStopRx;
	pDev->DevIntrf.StartTx = SamdSpiStartTx;
	pDev->DevIntrf.TxData = SamdSpiTxData;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.StopTx = SamdSpiStopTx;
	pDev->DevIntrf.Reset = SamdSpiReset;
	pDev->DevIntrf.PowerOff = SamdSpiPowerOff;
	pDev->DevIntrf.GetHandle = SamdSpiGetHandle;
	pDev->DevIntrf.IntPrio = (int)SamdIrqPrio(pCfgData->IntPrio);
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	SharedIntrfSetIrqHandler(pCfgData->DevNo, &pDev->DevIntrf, SamdSpiIrqHandler);

	// Chip selects inactive before they drive
	for (int i = 0; i < dev->NbCs; i++)
	{
		const IOPinCfg_t *cs = SamdSpiCsPin(dev, i);

		IOPinSet(cs->PortNo, cs->PinNo);
	}
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	bool res = pDev->Cfg.Rate != 0U && SamdSpiHwInit(dev);

	if (res)
	{
		if (slave)
		{
			SamdSpisArm(dev);
		}

		if (intEn)
		{
			NVIC_ClearPendingIRQ(irq);
			NVIC_SetPriority(irq, SamdIrqPrio(pCfgData->IntPrio));
			NVIC_EnableIRQ(irq);
		}
	}
	else
	{
		SamdSpiDmaGive(dev);
		SharedIntrfSetIrqHandler(pCfgData->DevNo, NULL, NULL);
		IOPinDis(pCfgData->pIOPinMap, pCfgData->NbIOPins);
		SamdSercomClockDisable(pCfgData->DevNo);
	}

	EnableInterrupt(state);

	return res;
}

/**-------------------------------------------------------------------------
@file	sdmmc_lpc546xx.cpp

@brief	LPC546xx SDIF, the SD/MMC host as an SD bus interface (coredev/sdmmc.h)

One host, slot 0. Host register use as the NXP SDK fsl_sdif.c: transfers
polled through the FIFO, the clock update command, the hold register. Each
transfer runs the command of its frame: the response in the RESP registers,
the data through the FIFO, with the auto stop of the host for the stop flag.
The card busy (STATUS DATA_BUSY) is waited before a command other than
CMD13, after an R1b response and after the data written. A data error, or an
error bit in the R1 of a data command, ends with a FIFO reset and a stop
command; an error bit in the response of the auto stop (RESP1) fails the
transfer.

Init switches the card supply (SD_POW_EN) off then on when the board wires
it (bPowerCtrl). With the card detect wired (bCardDetect), StartTx and
StartRx fail without a card. Polling only: bDmaEn and bIntEn are refused.

The SDIF clock is the main clock divided by SDIOCLKDIV to 52 MHz or less, the
card clock that clock divided by 2 x CLKDIV or not at all.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "LPC546xx.h"

#include "idelay.h"
#include "coredev/iopincfg.h"
#include "coredev/sdmmc.h"
#include "iopinctrl.h"

#ifndef LPC546XX_SDIF_WAIT_MAX
// Register reads without progress before a command or a transfer is given
// up, about one second. The host times out the card first (TMOUT).
#define LPC546XX_SDIF_WAIT_MAX		50000000U
#endif

#ifndef LPC546XX_SDIF_BUSY_MAX
// Busy waits of 10 us for the card programming, one second
#define LPC546XX_SDIF_BUSY_MAX		100000U
#endif

#define LPC546XX_SDIF_FIFO_DEPTH	64U			// Words (FSL_FEATURE_SDIF_FIFO_DEPTH_64_32BITS)
#define LPC546XX_SDIF_DIV_MAX		256U		// SDIOCLKDIV DIV + 1
#define LPC546XX_SDIF_CLK_MAX		52000000U	// SDIF clock (FSL_FEATURE_SDIF_MAX_SOURCE_CLOCK)
#define LPC546XX_SDIF_CLKDIV_MAX	255U		// CLKDIV: card clock = SDIF clock / (2 x CLKDIV)
#define LPC546XX_SDIF_CLKSEL_MAIN	0U
#define LPC546XX_SDIF_CLKSEL_NONE	7U
#define LPC546XX_SDIF_DEFAULT_RATE	25000000U	// Default speed
#define LPC546XX_SDIF_STOP_CMD		12U			// Stop transmission
#define LPC546XX_SDIF_STATUS_CMD	13U			// Send status, taken while the card is busy

// Card clock sample and drive delays of the NXP SDK fsl_sdif.c at every card
// clock, 250 ps steps: 3 ns and 7.75 ns
#define LPC546XX_SDIF_SAMPLE_DELAY	12U
#define LPC546XX_SDIF_DRV_DELAY		31U

// Response kinds of the commands
#define SDIF_RESP_SHORT				(SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK)
#define SDIF_RESP_R3				SDIF_CMD_RESPONSE_EXPECT_MASK
#define SDIF_RESP_LONG				(SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_RESPONSE_LENGTH_MASK | \
									 SDIF_CMD_CHECK_RESPONSE_CRC_MASK)

#define SDIF_CMD_ERRORS				(SDIF_RINTSTS_RE_MASK | SDIF_RINTSTS_RCRC_MASK | \
									 SDIF_RINTSTS_RTO_BAR_MASK | SDIF_RINTSTS_HLE_MASK)
#define SDIF_DATA_ERRORS			(SDIF_RINTSTS_DCRC_MASK | SDIF_RINTSTS_DRTO_BDS_MASK | \
									 SDIF_RINTSTS_HTO_MASK | SDIF_RINTSTS_FRUN_MASK | \
									 SDIF_RINTSTS_SBE_MASK | SDIF_RINTSTS_EBE_MASK)

static uint32_t s_Lpc546xxSdmmcRate = 0;	// Card clock, Hz
static bool s_bLpc546xxSdmmcOn = false;		// Set up, not powered off

// Wait for the bits of Mask clear in CTRL, the reset bits
static bool Lpc546xxSdifCtrlWait(uint32_t Mask)
{
	for (uint32_t i = 0; i < LPC546XX_SDIF_WAIT_MAX; i++)
	{
		if ((SDIF->CTRL & Mask) == 0)
		{
			return true;
		}
	}

	return false;
}

// Wait until the host took the command register
static bool Lpc546xxSdifStartWait(void)
{
	for (uint32_t i = 0; i < LPC546XX_SDIF_WAIT_MAX; i++)
	{
		if ((SDIF->CMD & SDIF_CMD_START_CMD_MASK) == 0)
		{
			return true;
		}
	}

	return false;
}

// The clock registers to the card interface. The host resets when it does
// not take the command (NXP SDK).
static bool Lpc546xxSdifUpdateClock(void)
{
	for (int retry = 0; retry < 3; retry++)
	{
		SDIF->CMD = SDIF_CMD_START_CMD_MASK | SDIF_CMD_UPDATE_CLOCK_REGISTERS_ONLY_MASK |
					SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK;
		if (Lpc546xxSdifStartWait())
		{
			return true;
		}
		SDIF->CTRL |= SDIF_CTRL_CONTROLLER_RESET_MASK;
		(void)Lpc546xxSdifCtrlWait(SDIF_CTRL_CONTROLLER_RESET_MASK);
	}

	return false;
}

// The card clock: the highest not above Rate, the slowest when none is.
// Returns the clock, 0 when the host does not take it.
static uint32_t Lpc546xxSdifSetClock(uint32_t Rate)
{
	uint32_t clk = SystemCoreClock * ((SYSCON->AHBCLKDIV & SYSCON_AHBCLKDIV_DIV_MASK) + 1U);
	uint32_t best = 0, bd = LPC546XX_SDIF_DIV_MAX, bn = LPC546XX_SDIF_CLKDIV_MAX;

	for (uint32_t d = (clk + LPC546XX_SDIF_CLK_MAX - 1U) / LPC546XX_SDIF_CLK_MAX;
		 d <= LPC546XX_SDIF_DIV_MAX && Rate > 0; d++)
	{
		uint32_t s = clk / d;
		uint32_t n = 0;
		uint32_t f = s;

		if (s > Rate)
		{
			n = (s + 2U * Rate - 1U) / (2U * Rate);
			if (n > LPC546XX_SDIF_CLKDIV_MAX)
			{
				continue;
			}
			f = s / (2U * n);
		}
		if (f > best)
		{
			best = f;
			bd = d;
			bn = n;
		}
	}

	if (best == 0)
	{
		best = clk / LPC546XX_SDIF_DIV_MAX / (2U * LPC546XX_SDIF_CLKDIV_MAX);
	}

	// Clock off, the dividers, clock on, each through the card interface
	SDIF->CLKENA = 0;
	if (Lpc546xxSdifUpdateClock() == false)
	{
		return 0;
	}
	SYSCON->SDIOCLKDIV = SYSCON_SDIOCLKDIV_DIV(bd - 1U);
	SDIF->CLKDIV = SDIF_CLKDIV_CLK_DIVIDER0(bn);
	if (Lpc546xxSdifUpdateClock() == false)
	{
		return 0;
	}
	SDIF->CLKENA = SDIF_CLKENA_CCLK_ENABLE_MASK;
	if (Lpc546xxSdifUpdateClock() == false)
	{
		return 0;
	}

	return best;
}

// A command, its response in pResp (1 word, 4 for a long response). Returns
// false without response or on a response error.
static bool Lpc546xxSdifCmd(uint32_t Idx, uint32_t Arg, uint32_t Flags, uint32_t *pResp)
{
	uint32_t st = 0;

	// A stop command does not wait the data in progress
	if ((Flags & SDIF_CMD_STOP_ABORT_CMD_MASK) == 0)
	{
		Flags |= SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK;
	}

	SDIF->RINTSTS = SDIF_CMD_ERRORS | SDIF_RINTSTS_CDONE_MASK;
	SDIF->CMDARG = Arg;
	SDIF->CMD = SDIF_CMD_START_CMD_MASK | SDIF_CMD_USE_HOLD_REG_MASK | Flags | SDIF_CMD_CMD_INDEX(Idx);

	if (Lpc546xxSdifStartWait() == false)
	{
		// Command not taken: the reset of the host ends it
		SDIF->CTRL |= SDIF_CTRL_CONTROLLER_RESET_MASK;
		(void)Lpc546xxSdifCtrlWait(SDIF_CTRL_CONTROLLER_RESET_MASK);

		return false;
	}

	uint32_t i;

	for (i = 0; i < LPC546XX_SDIF_WAIT_MAX; i++)
	{
		st = SDIF->RINTSTS;
		if (st & (SDIF_RINTSTS_CDONE_MASK | SDIF_CMD_ERRORS))
		{
			break;
		}
	}

	if (i >= LPC546XX_SDIF_WAIT_MAX || (st & SDIF_CMD_ERRORS))
	{
		return false;
	}

	pResp[0] = SDIF->RESP[0];
	if (Flags & SDIF_CMD_RESPONSE_LENGTH_MASK)
	{
		pResp[1] = SDIF->RESP[1];
		pResp[2] = SDIF->RESP[2];
		pResp[3] = SDIF->RESP[3];
	}

	return true;
}

// Data line 0 released by the card at the end of a programming or a select
static bool Lpc546xxSdifWaitBusy(void)
{
	for (uint32_t i = 0; i < LPC546XX_SDIF_BUSY_MAX; i++)
	{
		if ((SDIF->STATUS & SDIF_STATUS_DATA_BUSY_MASK) == 0)
		{
			return true;
		}
		usDelay(10);
	}

	return false;
}

// End a data command that failed. The FIFO is emptied first: the host stops
// the card clock while its FIFO is full, the stop command needs the clock.
// The stop command ends the data path of the host and the data state of the
// card. A card already back in its transfer state does not answer it.
static void Lpc546xxSdifAbort(void)
{
	uint32_t r[1];

	SDIF->CTRL |= SDIF_CTRL_FIFO_RESET_MASK;
	(void)Lpc546xxSdifCtrlWait(SDIF_CTRL_FIFO_RESET_MASK);
	(void)Lpc546xxSdifCmd(LPC546XX_SDIF_STOP_CMD, 0, SDIF_RESP_SHORT | SDIF_CMD_STOP_ABORT_CMD_MASK, r);
	(void)Lpc546xxSdifWaitBusy();
}

// The data of a command through the FIFO, little endian words, then the end
// of the data and of the auto stop
static bool Lpc546xxSdifData(uint8_t *pRx, const uint8_t *pTx, uint32_t Len, bool bStop)
{
	uint32_t words = (Len + 3U) / 4U;
	uint32_t done = 0;

	for (uint32_t idle = 0; done < words;)
	{
		uint32_t moved = done;

		if (SDIF->RINTSTS & SDIF_DATA_ERRORS)
		{
			return false;
		}

		uint32_t cnt = (SDIF->STATUS & SDIF_STATUS_FIFO_COUNT_MASK) >> SDIF_STATUS_FIFO_COUNT_SHIFT;

		if (pRx != NULL)
		{
			for (; cnt > 0 && done < words; cnt--, done++)
			{
				uint32_t w = SDIF->FIFO[0];
				uint32_t n = Len - done * 4U < 4U ? Len - done * 4U : 4U;

				memcpy(pRx + done * 4U, &w, n);
			}
		}
		else
		{
			for (cnt = cnt < LPC546XX_SDIF_FIFO_DEPTH ? LPC546XX_SDIF_FIFO_DEPTH - cnt : 0;
				 cnt > 0 && done < words; cnt--, done++)
			{
				uint32_t w = 0;
				uint32_t n = Len - done * 4U < 4U ? Len - done * 4U : 4U;

				memcpy(&w, pTx + done * 4U, n);
				SDIF->FIFO[0] = w;
			}
		}

		idle = done == moved ? idle + 1U : 0U;
		if (idle >= LPC546XX_SDIF_WAIT_MAX)
		{
			return false;
		}
	}

	uint32_t end = SDIF_RINTSTS_DTO_MASK | (bStop ? SDIF_RINTSTS_ACD_MASK : 0U);
	uint32_t st = 0;

	for (uint32_t i = 0; i < LPC546XX_SDIF_WAIT_MAX; i++)
	{
		st |= SDIF->RINTSTS;
		if ((st & end) == end)
		{
			break;
		}
	}

	return (st & end) == end && (st & SDIF_DATA_ERRORS) == 0;
}

// The command of the frame, with its data: pRx or pTx, Len bytes
static bool Lpc546xxSdmmcRun(SdmmcDev_t * const pDev, uint8_t *pRx, const uint8_t *pTx, int Len)
{
	const uint8_t *f = pDev->Frame;
	uint32_t idx = f[SDMMC_FRAME_CMD_IDX] & 0x3FU;
	uint32_t arg = ((uint32_t)f[1] << 24) | ((uint32_t)f[2] << 16) | ((uint32_t)f[3] << 8) | f[4];
	uint32_t fl = f[SDMMC_FRAME_FLAGS_IDX];
	uint32_t data = fl & SDMMC_FRAME_DATA_MASK;
	uint32_t flags;
	uint32_t r[4] = { 0, 0, 0, 0 };

	pDev->bSent = true;
	pDev->RespLen = 0;

	switch (fl & SDMMC_FRAME_RESP_MASK)
	{
		case SDMMC_FRAME_RESP_NONE:
			flags = 0;
			break;
		case SDMMC_FRAME_RESP_R1:
		case SDMMC_FRAME_RESP_R1B:
			flags = SDIF_RESP_SHORT;
			break;
		case SDMMC_FRAME_RESP_R2:
			flags = SDIF_RESP_LONG;
			break;
		case SDMMC_FRAME_RESP_R3:
			flags = SDIF_RESP_R3;
			break;
		default:
			return false;
	}
	if ((f[SDMMC_FRAME_CMD_IDX] & 0xC0U) != 0x40U)
	{
		return false;
	}
	// 4 data lines asked with fewer wired
	if ((fl & SDMMC_FRAME_BUS_WIDTH) && (arg & 3U) == 2U && pDev->Cfg.BusWidth < 4)
	{
		return false;
	}
	if (data != SDMMC_FRAME_DATA_NONE)
	{
		// Blocks of 512 bytes, or one shorter block
		if (Len > SDMMC_BLOCK_SIZE && (Len % SDMMC_BLOCK_SIZE) != 0)
		{
			return false;
		}
		flags |= SDIF_CMD_DATA_EXPECTED_MASK | (data == SDMMC_FRAME_DATA_WRITE ? SDIF_CMD_READ_WRITE_MASK : 0U) |
				 ((fl & SDMMC_FRAME_STOP) ? SDIF_CMD_SEND_AUTO_STOP_MASK : 0U);
	}

	// The card done with a programming, the status command aside
	if (idx != LPC546XX_SDIF_STATUS_CMD && Lpc546xxSdifWaitBusy() == false)
	{
		return false;
	}

	if (fl & SDMMC_FRAME_INIT)
	{
		// 80 clocks before the command, the card back to 1 data line
		flags |= SDIF_CMD_SEND_INITIALIZATION_MASK;
		SDIF->CTYPE = 0;
		pDev->BusWidth = 1;
	}

	if (data != SDMMC_FRAME_DATA_NONE)
	{
		SDIF->CTRL |= SDIF_CTRL_FIFO_RESET_MASK;
		if (Lpc546xxSdifCtrlWait(SDIF_CTRL_FIFO_RESET_MASK) == false)
		{
			return false;
		}
		SDIF->BLKSIZ = Len < SDMMC_BLOCK_SIZE ? (uint32_t)Len : SDMMC_BLOCK_SIZE;
		SDIF->BYTCNT = (uint32_t)Len;
		// Status of the transfers before
		SDIF->RINTSTS = 0xFFFFFFFFU;
	}

	// A data command refused by the card: the host waits for data the card
	// does not send
	if (Lpc546xxSdifCmd(idx, arg, flags, r) == false ||
		(data != SDMMC_FRAME_DATA_NONE && (r[0] & SDMMC_R1_ERRORS)))
	{
		if (data != SDMMC_FRAME_DATA_NONE)
		{
			Lpc546xxSdifAbort();
		}

		return false;
	}

	// The response, most significant byte first
	if (flags & SDIF_CMD_RESPONSE_EXPECT_MASK)
	{
		int nw = (flags & SDIF_CMD_RESPONSE_LENGTH_MASK) ? 4 : 1;

		for (int i = 0; i < nw * 4; i++)
		{
			pDev->Resp[i] = (uint8_t)(r[nw - 1 - i / 4] >> (24 - 8 * (i % 4)));
		}
		pDev->RespLen = nw * 4;
	}

	if ((fl & SDMMC_FRAME_RESP_MASK) == SDMMC_FRAME_RESP_R1B && Lpc546xxSdifWaitBusy() == false)
	{
		pDev->RespLen = 0;

		return false;
	}

	if (fl & SDMMC_FRAME_BUS_WIDTH)
	{
		pDev->BusWidth = (arg & 3U) == 2U ? 4 : 1;
		SDIF->CTYPE = pDev->BusWidth == 4 ? SDIF_CTYPE_CARD_WIDTH0_MASK : 0U;
	}

	if (data == SDMMC_FRAME_DATA_NONE)
	{
		return true;
	}

	if (Lpc546xxSdifData(pRx, pTx, (uint32_t)Len, (fl & SDMMC_FRAME_STOP) != 0) == false)
	{
		Lpc546xxSdifAbort();

		return false;
	}

	// Errors of a multiple block transfer in the response of the auto stop
	if ((fl & SDMMC_FRAME_STOP) && (SDIF->RESP[1] & SDMMC_R1_ERRORS))
	{
		(void)Lpc546xxSdifWaitBusy();

		return false;
	}

	// The card programs the data written
	return data == SDMMC_FRAME_DATA_READ || Lpc546xxSdifWaitBusy();
}

// Powered off, or no card in the slot with the card detect wired
static bool Lpc546xxSdmmcNoCard(SdmmcDev_t * const pDev)
{
	return s_bLpc546xxSdmmcOn == false ||
		   (pDev->Cfg.bCardDetect && (SDIF->CDETECT & SDIF_CDETECT_CARD_DETECT_MASK));
}

static void Lpc546xxSdmmcDisable(DevIntrf_t * const pDev)
{
	(void)pDev;

	// Card clock off
	if (s_bLpc546xxSdmmcOn)
	{
		SDIF->CLKENA = 0;
		(void)Lpc546xxSdifUpdateClock();
	}
}

static void Lpc546xxSdmmcEnable(DevIntrf_t * const pDev)
{
	(void)pDev;

	if (s_bLpc546xxSdmmcOn)
	{
		SDIF->CLKENA = SDIF_CLKENA_CCLK_ENABLE_MASK;
		(void)Lpc546xxSdifUpdateClock();
	}
}

static uint32_t Lpc546xxSdmmcGetRate(DevIntrf_t * const pDev)
{
	(void)pDev;

	return s_Lpc546xxSdmmcRate;
}

static uint32_t Lpc546xxSdmmcSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	(void)pDev;

	if (s_bLpc546xxSdmmcOn == false)
	{
		return 0;
	}
	s_Lpc546xxSdmmcRate = Lpc546xxSdifSetClock(Rate);

	return s_Lpc546xxSdmmcRate;
}

// The frame of a transfer is kept from StartTx to the stop
static bool Lpc546xxSdmmcStartTx(DevIntrf_t * const pDev, uint32_t Slot)
{
	SdmmcDev_t *dev = (SdmmcDev_t *)pDev->pDevData;

	if (Slot != 0 || Lpc546xxSdmmcNoCard(dev))
	{
		return false;
	}
	dev->CurSlot = 0;
	dev->FrameLen = 0;
	dev->bSent = false;
	dev->RespLen = 0;

	return true;
}

static bool Lpc546xxSdmmcStartRx(DevIntrf_t * const pDev, uint32_t Slot)
{
	SdmmcDev_t *dev = (SdmmcDev_t *)pDev->pDevData;

	return Slot == 0 && Lpc546xxSdmmcNoCard(dev) == false;
}

// The frame, then the data of a write. A command without data runs when its
// frame is complete. A frame with the two data directions is never run.
static int Lpc546xxSdmmcTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	SdmmcDev_t *dev = (SdmmcDev_t *)pDev->pDevData;
	int n = 0;

	if (pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	if (dev->FrameLen < SDMMC_FRAME_SIZE)
	{
		while (dev->FrameLen < SDMMC_FRAME_SIZE && n < DataLen)
		{
			dev->Frame[dev->FrameLen++] = pData[n++];
		}
		if (dev->FrameLen < SDMMC_FRAME_SIZE)
		{
			return n;
		}

		uint32_t data = dev->Frame[SDMMC_FRAME_FLAGS_IDX] & SDMMC_FRAME_DATA_MASK;

		if (data == SDMMC_FRAME_DATA_NONE)
		{
			return Lpc546xxSdmmcRun(dev, NULL, NULL, 0) && n == DataLen ? n : 0;
		}
		if (data != SDMMC_FRAME_DATA_WRITE || n == DataLen)
		{
			return n;
		}
	}

	if ((dev->Frame[SDMMC_FRAME_FLAGS_IDX] & SDMMC_FRAME_DATA_MASK) != SDMMC_FRAME_DATA_WRITE || dev->bSent)
	{
		return 0;
	}

	return Lpc546xxSdmmcRun(dev, NULL, pData + n, DataLen - n) ? DataLen : 0;
}

// The data of a read, or the response of the command
static int Lpc546xxSdmmcRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	SdmmcDev_t *dev = (SdmmcDev_t *)pDev->pDevData;

	if (pBuff == NULL || BuffLen <= 0 || dev->FrameLen < SDMMC_FRAME_SIZE)
	{
		return 0;
	}

	if ((dev->Frame[SDMMC_FRAME_FLAGS_IDX] & SDMMC_FRAME_DATA_MASK) == SDMMC_FRAME_DATA_READ)
	{
		return dev->bSent == false && Lpc546xxSdmmcRun(dev, pBuff, NULL, BuffLen) ? BuffLen : 0;
	}
	if (dev->bSent == false)
	{
		return 0;
	}

	// 0 when the command failed
	int n = BuffLen < dev->RespLen ? BuffLen : dev->RespLen;

	memcpy(pBuff, dev->Resp, n);

	return n;
}

static void Lpc546xxSdmmcStop(DevIntrf_t * const pDev)
{
	SdmmcDev_t *dev = (SdmmcDev_t *)pDev->pDevData;

	dev->FrameLen = 0;
	dev->bSent = false;
}

// Host and FIFO reset, the card clock taken again
static void Lpc546xxSdmmcReset(DevIntrf_t * const pDev)
{
	SdmmcDev_t *dev = (SdmmcDev_t *)pDev->pDevData;

	if (s_bLpc546xxSdmmcOn == false)
	{
		return;
	}
	SDIF->CTRL |= SDIF_CTRL_CONTROLLER_RESET_MASK | SDIF_CTRL_FIFO_RESET_MASK | SDIF_CTRL_DMA_RESET_MASK;
	(void)Lpc546xxSdifCtrlWait(SDIF_CTRL_CONTROLLER_RESET_MASK | SDIF_CTRL_FIFO_RESET_MASK |
							   SDIF_CTRL_DMA_RESET_MASK);
	(void)Lpc546xxSdifUpdateClock();
	dev->FrameLen = 0;
	dev->bSent = false;
}

// The reset of the host turns the card supply (PWREN) and the card clock off
static void Lpc546xxSdmmcPowerOff(DevIntrf_t * const pDev)
{
	SdmmcDev_t *dev = (SdmmcDev_t *)pDev->pDevData;

	SYSCON->PRESETCTRLSET[2] = SYSCON_PRESETCTRL_SDIO_RST_MASK;
	SYSCON->PRESETCTRLCLR[2] = SYSCON_PRESETCTRL_SDIO_RST_MASK;
	SYSCON->AHBCLKCTRLCLR[2] = SYSCON_AHBCLKCTRL_SDIO_MASK;
	SYSCON->SDIOCLKSEL = SYSCON_SDIOCLKSEL_SEL(LPC546XX_SDIF_CLKSEL_NONE);
	s_Lpc546xxSdmmcRate = 0;
	s_bLpc546xxSdmmcOn = false;
	dev->BusWidth = 1;

	if (dev->Cfg.pIOPinMap != NULL)
	{
		IOPinDis(dev->Cfg.pIOPinMap, dev->Cfg.NbIOPins);
	}
}

static void *Lpc546xxSdmmcGetHandle(DevIntrf_t * const pDev)
{
	return pDev->pDevData;
}

bool SdmmcInit(SdmmcDev_t * const pDev, const SdmmcCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo != 0 || pCfg->pIOPinMap == NULL ||
		(pCfg->BusWidth != 1 && pCfg->BusWidth != 4) ||
		pCfg->NbIOPins < SDMMC_D0_IOPIN_IDX + pCfg->BusWidth || pCfg->bDmaEn || pCfg->bIntEn)
	{
		return false;
	}

	pDev->Cfg = *pCfg;
	pDev->BusWidth = 1;
	pDev->CurSlot = 0;
	pDev->FrameLen = 0;
	pDev->bSent = false;
	pDev->RespLen = 0;

	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		IOPinSetSpeed(pCfg->pIOPinMap[i].PortNo, pCfg->pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	// Bus clock on and reset of the host: card supply and clock off, FIFO
	// empty, 1 bit bus. The SDIF clock from the main clock.
	SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_SDIO_MASK;
	SYSCON->PRESETCTRLSET[2] = SYSCON_PRESETCTRL_SDIO_RST_MASK;
	SYSCON->PRESETCTRLCLR[2] = SYSCON_PRESETCTRL_SDIO_RST_MASK;
	SYSCON->SDIOCLKSEL = SYSCON_SDIOCLKSEL_SEL(LPC546XX_SDIF_CLKSEL_MAIN);
	SYSCON->SDIOCLKCTRL = SYSCON_SDIOCLKCTRL_CCLK_SAMPLE_DELAY_ACTIVE_MASK |
						  SYSCON_SDIOCLKCTRL_CCLK_SAMPLE_DELAY(LPC546XX_SDIF_SAMPLE_DELAY) |
						  SYSCON_SDIOCLKCTRL_CCLK_DRV_DELAY_ACTIVE_MASK |
						  SYSCON_SDIOCLKCTRL_CCLK_DRV_DELAY(LPC546XX_SDIF_DRV_DELAY);

	// Response timeout 64 clocks, data timeout the largest, card detect
	// debounce of the NXP SDK. Interrupts stay masked, the transfers poll the
	// FIFO count: the FIFO thresholds are not used.
	SDIF->TMOUT = SDIF_TMOUT_RESPONSE_TIMEOUT(0x40U) | SDIF_TMOUT_DATA_TIMEOUT(0xFFFFFFU);
	SDIF->DEBNCE = SDIF_DEBNCE_DEBOUNCE_COUNT(0xFFFFFFU);
	SDIF->CTYPE = 0;

	// Supply off then on: a card starts from its idle state
	if (pCfg->bPowerCtrl)
	{
		SDIF->PWREN = 0;
		usDelay(10000);
	}
	SDIF->PWREN = SDIF_PWREN_POWER_ENABLE_MASK;
	usDelay(10000);

	s_bLpc546xxSdmmcOn = true;
	s_Lpc546xxSdmmcRate = Lpc546xxSdifSetClock(pCfg->Rate > 0 ? pCfg->Rate : LPC546XX_SDIF_DEFAULT_RATE);
	if (s_Lpc546xxSdmmcRate == 0)
	{
		s_bLpc546xxSdmmcOn = false;

		return false;
	}

	DevIntrf_t *intrf = &pDev->DevIntrf;

	intrf->pDevData = pDev;
	intrf->Type = DEVINTRF_TYPE_SDMMC;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->MaxRetry = 0;
	intrf->bDma = false;
	intrf->bIntEn = false;
	atomic_flag_clear(&intrf->bBusy);
	atomic_store(&intrf->EnCnt, 1);
	atomic_store(&intrf->bTxReady, true);
	atomic_store(&intrf->bNoStop, false);
	intrf->Disable = Lpc546xxSdmmcDisable;
	intrf->Enable = Lpc546xxSdmmcEnable;
	intrf->GetRate = Lpc546xxSdmmcGetRate;
	intrf->SetRate = Lpc546xxSdmmcSetRate;
	intrf->StartRx = Lpc546xxSdmmcStartRx;
	intrf->RxData = Lpc546xxSdmmcRxData;
	intrf->StopRx = Lpc546xxSdmmcStop;
	intrf->StartTx = Lpc546xxSdmmcStartTx;
	intrf->TxData = Lpc546xxSdmmcTxData;
	intrf->TxSrData = Lpc546xxSdmmcTxData;
	intrf->StopTx = Lpc546xxSdmmcStop;
	intrf->Reset = Lpc546xxSdmmcReset;
	intrf->PowerOff = Lpc546xxSdmmcPowerOff;
	intrf->GetHandle = Lpc546xxSdmmcGetHandle;

	return true;
}

/**-------------------------------------------------------------------------
@file	spifi_lpc546xx.cpp

@brief	LPC546xx SPIFI, the quad SPI flash interface, as a QSPI device

SPI DevNo LPC546XX_SPIFI_DEVNO, see spifi_lpc546xx.h. Master in polling
mode, command mode only, as the NXP SDK fsl_spifi.c and its LPCXpresso54628
polling example.

A flash command is one SPIFI command: ADDR and IDATA, then CMD starts it with
its opcode, address bytes (FRAMEFORM), intermediate bytes (INTLEN) and data
length (DATALEN). FIELDFORM selects the fields sent on the dual or quad
lines, CTRL DUAL selects dual or quad. The lines of a command come from its
opcode:
	1-1-2	0x3B, 0x3C			1-2-2	0xBB, 0xBC
	1-1-4	0x6B, 0x6C, 0x32, 0x34
	1-4-4	0xEB, 0xEC, 0x38, 0x3E
	1-1-1	all the others
The dummy cycles are sent as intermediate bytes on the lines of the address,
8 bits per byte: a dummy cycle count that does not make whole bytes, or more
than 7 bytes, is refused.

QuadSPISendCmd runs a command without data at once. A command with data is
kept and run by RxData or TxData, which give the direction. The data
register is read or written one byte per access, the SPIFI holds the bus
until the byte is there. A read longer than DATALEN (16383 bytes) is split
in several commands at the following addresses, a command without address
and a write move 16383 bytes at most.

A command that does not end in LPC546XX_SPIFI_WAIT_MAX status reads is
aborted with STAT RESET. The SPIFI is left in command mode: memory mode is
not used, a memory mode found active (MCINIT) is ended before a command.

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

#include "LPC546xx.h"

#include "istddef.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/spi.h"
#include "spifi_lpc546xx.h"

#ifndef LPC546XX_SPIFI_WAIT_MAX
// STAT reads before a command is aborted
#define LPC546XX_SPIFI_WAIT_MAX		1000000U
#endif

#ifndef LPC546XX_SPIFI_DATA8
// One byte of the data field per access
#define LPC546XX_SPIFI_DATA8		(*(volatile uint8_t *)&SPIFI0->DATA)
#endif

#define LPC546XX_SPIFI_DATALEN_MAX	16383U		// CMD DATALEN
#define LPC546XX_SPIFI_INTLEN_MAX	7U			// CMD INTLEN
#define LPC546XX_SPIFI_DIV_MAX		256U		// SPIFICLKDIV DIV + 1
#define LPC546XX_SPIFI_CLKSEL_MAIN	0U			// SPIFICLKSEL main clock
#define LPC546XX_SPIFI_CLKSEL_NONE	7U

// CMD FIELDFORM
#define LPC546XX_SPIFI_FIELD_SERIAL	0U			// All fields serial
#define LPC546XX_SPIFI_FIELD_DATA	1U			// Data on the dual or quad lines
#define LPC546XX_SPIFI_FIELD_ADDR	2U			// Opcode serial, the others not

// CMD FRAMEFORM of a command without address
#define LPC546XX_SPIFI_FRAME_OPCODE	1U

#pragma pack(push, 4)

typedef struct {
	SPIDev_t *pSpiDev;
	uint32_t Ctrl;			// CTRL, DUAL is added per command
	uint32_t Clk;			// SPIFI clock in Hz
	uint8_t AddrLen;		// Address bytes of a command that gives none
	bool bOn;				// SPIFI clock on
	// Command with a data phase, run by RxData or TxData
	bool bPend;
	bool bPendAddr;
	uint32_t PendCmd;		// CMD without DATALEN and DOUT
	uint32_t PendCtrl;
	uint32_t PendAddr;
	uint32_t PendLen;
} Lpc546xxSpifiDev_t;

#pragma pack(pop)

static Lpc546xxSpifiDev_t s_Lpc546xxSpifiDev = {
	NULL, 0, 0, 3, false, false, false, 0, 0, 0, 0
};

static bool Lpc546xxSpifiIsDev(SPIDev_t * const pDev)
{
	return pDev != NULL && s_Lpc546xxSpifiDev.pSpiDev == pDev;
}

// Wait for the STAT bits of Mask to be 0
static bool Lpc546xxSpifiWait(uint32_t Mask)
{
	for (uint32_t i = 0; i < LPC546XX_SPIFI_WAIT_MAX; i++)
	{
		if ((SPIFI0->STAT & Mask) == 0U)
		{
			return true;
		}
	}

	return false;
}

// End the command or the memory mode in progress
static void Lpc546xxSpifiAbort(void)
{
	SPIFI0->STAT = SPIFI_STAT_RESET_MASK;
	(void)Lpc546xxSpifiWait(SPIFI_STAT_RESET_MASK);
}

// Command mode and no command running
static bool Lpc546xxSpifiIdle(void)
{
	if ((SPIFI0->STAT & SPIFI_STAT_MCINIT_MASK) || Lpc546xxSpifiWait(SPIFI_STAT_CMD_MASK) == false)
	{
		Lpc546xxSpifiAbort();
	}

	return (SPIFI0->STAT & (SPIFI_STAT_MCINIT_MASK | SPIFI_STAT_CMD_MASK | SPIFI_STAT_RESET_MASK)) == 0U;
}

// Main clock, the source of the SPIFI clock
static uint32_t Lpc546xxSpifiMainClk(void)
{
	return SystemCoreClock * ((SYSCON->AHBCLKDIV & SYSCON_AHBCLKDIV_DIV_MASK) + 1U);
}

// Lines of the address and of the data of a flash command: 1, 2 or 4
static void Lpc546xxSpifiLines(uint8_t Cmd, uint32_t &AddrLines, uint32_t &DataLines)
{
	switch (Cmd)
	{
		case 0x3B:		// Dual output read, 1-1-2
		case 0x3C:		// Same, 4 byte address
			AddrLines = 1;
			DataLines = 2;
			break;
		case 0xBB:		// Dual I/O read, 1-2-2
		case 0xBC:
			AddrLines = 2;
			DataLines = 2;
			break;
		case 0x6B:		// Quad output read, 1-1-4
		case 0x6C:
		case 0x32:		// Quad input page program, 1-1-4
		case 0x34:
			AddrLines = 1;
			DataLines = 4;
			break;
		case 0xEB:		// Quad I/O read, 1-4-4
		case 0xEC:
		case 0x38:		// Quad I/O page program, 1-4-4
		case 0x3E:
			AddrLines = 4;
			DataLines = 4;
			break;
		default:
			AddrLines = 1;
			DataLines = 1;
			break;
	}
}

// Run one command with Len bytes of data into pRx or from pTx. Returns Len,
// or -1 when the command could not start or did not end.
static int Lpc546xxSpifiRun(uint32_t Cmd, uint32_t Ctrl, uint32_t Addr, bool bAddr,
							uint32_t Len, uint8_t *pRx, const uint8_t *pTx)
{
	if (Lpc546xxSpifiIdle() == false)
	{
		return -1;
	}

	SPIFI0->CTRL = Ctrl;
	if (bAddr)
	{
		SPIFI0->ADDR = Addr;
	}
	SPIFI0->IDATA = 0;
	SPIFI0->CMD = Cmd | SPIFI_CMD_DATALEN(Len) | (pTx != NULL ? SPIFI_CMD_DOUT_MASK : 0U);

	for (uint32_t i = 0; i < Len; i++)
	{
		if (pRx != NULL)
		{
			pRx[i] = LPC546XX_SPIFI_DATA8;
		}
		else
		{
			LPC546XX_SPIFI_DATA8 = pTx[i];
		}
	}

	if (Lpc546xxSpifiWait(SPIFI_STAT_CMD_MASK) == false)
	{
		Lpc546xxSpifiAbort();

		return -1;
	}

	return (int)Len;
}

static bool Lpc546xxSpifiSendCmd(Lpc546xxSpifiDev_t *pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen,
								 uint32_t DataLen, uint8_t DummyCycle)
{
	bool baddr = Addr != (uint32_t)-1;
	SPIPHY phy = pDev->pSpiDev->Cfg.Phy;
	uint32_t maxlines = phy == SPIPHY_QUAD_SDR ? 4U : (phy == SPIPHY_DUAL ? 2U : 1U);
	uint32_t alines, dlines;

	pDev->bPend = false;

	if (pDev->bOn == false)
	{
		return false;
	}

	Lpc546xxSpifiLines(Cmd, alines, dlines);

	uint32_t alen = AddrLen != 0 ? AddrLen : pDev->AddrLen;
	uint32_t bits = (uint32_t)DummyCycle * alines;

	if (dlines > maxlines || (baddr && alen > 4U) ||
		(bits % 8U) != 0 || bits / 8U > LPC546XX_SPIFI_INTLEN_MAX)
	{
		return false;
	}

	uint32_t form = dlines == 1U ? LPC546XX_SPIFI_FIELD_SERIAL :
					(alines == 1U ? LPC546XX_SPIFI_FIELD_DATA : LPC546XX_SPIFI_FIELD_ADDR);
	uint32_t cmd = SPIFI_CMD_INTLEN(bits / 8U) | SPIFI_CMD_FIELDFORM(form) |
				   SPIFI_CMD_FRAMEFORM(baddr ? alen + 1U : LPC546XX_SPIFI_FRAME_OPCODE) |
				   SPIFI_CMD_OPCODE(Cmd);
	uint32_t ctrl = pDev->Ctrl | (dlines == 2U ? SPIFI_CTRL_DUAL_MASK : 0U);

	if (DataLen > 0)
	{
		pDev->PendCmd = cmd;
		pDev->PendCtrl = ctrl;
		pDev->PendAddr = Addr;
		pDev->bPendAddr = baddr;
		pDev->PendLen = DataLen;
		pDev->bPend = true;

		return true;
	}

	return Lpc546xxSpifiRun(cmd, ctrl, Addr, baddr, 0, NULL, NULL) == 0;
}

// The data of the kept command. A read without address and a write are one
// command, a read with address continues at the next address.
static int Lpc546xxSpifiData(Lpc546xxSpifiDev_t *pDev, uint8_t *pRx, const uint8_t *pTx, int Len)
{
	if (pDev->bPend == false || Len <= 0)
	{
		return 0;
	}

	uint32_t len = (uint32_t)Len < pDev->PendLen ? (uint32_t)Len : pDev->PendLen;
	uint32_t cnt = 0;

	pDev->bPend = false;

	if ((pRx == NULL || pDev->bPendAddr == false) && len > LPC546XX_SPIFI_DATALEN_MAX)
	{
		len = LPC546XX_SPIFI_DATALEN_MAX;
	}

	while (cnt < len)
	{
		uint32_t l = len - cnt;

		if (l > LPC546XX_SPIFI_DATALEN_MAX)
		{
			l = LPC546XX_SPIFI_DATALEN_MAX;
		}

		if (Lpc546xxSpifiRun(pDev->PendCmd, pDev->PendCtrl, pDev->PendAddr + cnt, pDev->bPendAddr, l,
							 pRx != NULL ? pRx + cnt : NULL, pTx != NULL ? pTx + cnt : NULL) < 0)
		{
			break;
		}
		cnt += l;
	}

	return (int)cnt;
}

static int Lpc546xxSpifiRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	if (pBuff == NULL)
	{
		return 0;
	}

	return Lpc546xxSpifiData((Lpc546xxSpifiDev_t *)pDev->pDevData, pBuff, NULL, BuffLen);
}

static int Lpc546xxSpifiTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	if (pData == NULL)
	{
		return 0;
	}

	return Lpc546xxSpifiData((Lpc546xxSpifiDev_t *)pDev->pDevData, NULL, pData, DataLen);
}

// The SPIFI drives its chip select for each command
static bool Lpc546xxSpifiStart(DevIntrf_t * const pDev, uint32_t DevCs)
{
	Lpc546xxSpifiDev_t *dev = (Lpc546xxSpifiDev_t *)pDev->pDevData;

	dev->bPend = false;

	return DevCs == 0 && dev->bOn;
}

static void Lpc546xxSpifiStop(DevIntrf_t * const pDev)
{
	Lpc546xxSpifiDev_t *dev = (Lpc546xxSpifiDev_t *)pDev->pDevData;

	dev->bPend = false;
}

static void *Lpc546xxSpifiGetHandle(DevIntrf_t * const pDev)
{
	Lpc546xxSpifiDev_t *dev = (Lpc546xxSpifiDev_t *)pDev->pDevData;

	return dev != NULL ? dev->pSpiDev : NULL;
}

static uint32_t Lpc546xxSpifiGetRate(DevIntrf_t * const pDev)
{
	Lpc546xxSpifiDev_t *dev = (Lpc546xxSpifiDev_t *)pDev->pDevData;

	return dev->Clk;
}

// The main clock divided by 1 to 256, the closest to Rate, the lower one of
// two as close
static uint32_t Lpc546xxSpifiSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Lpc546xxSpifiDev_t *dev = (Lpc546xxSpifiDev_t *)pDev->pDevData;
	uint32_t clk = Lpc546xxSpifiMainClk();
	uint32_t div = Rate > 0U ? clk / Rate : LPC546XX_SPIFI_DIV_MAX;

	if (div < 1U)
	{
		div = 1U;
	}
	else if (div < LPC546XX_SPIFI_DIV_MAX && Rate - clk / (div + 1U) <= clk / div - Rate)
	{
		div++;
	}
	if (div > LPC546XX_SPIFI_DIV_MAX)
	{
		div = LPC546XX_SPIFI_DIV_MAX;
	}

	if (dev->bOn)
	{
		(void)Lpc546xxSpifiIdle();
	}

	SYSCON->SPIFICLKDIV = SYSCON_SPIFICLKDIV_DIV(div - 1U);
	dev->Clk = clk / div;
	dev->pSpiDev->Cfg.Rate = dev->Clk;

	return dev->Clk;
}

static void Lpc546xxSpifiReset(DevIntrf_t * const pDev)
{
	Lpc546xxSpifiDev_t *dev = (Lpc546xxSpifiDev_t *)pDev->pDevData;

	dev->bPend = false;
	if (dev->bOn)
	{
		Lpc546xxSpifiAbort();
	}
}

static void Lpc546xxSpifiDisable(DevIntrf_t * const pDev)
{
	Lpc546xxSpifiDev_t *dev = (Lpc546xxSpifiDev_t *)pDev->pDevData;

	dev->bPend = false;
	if (dev->bOn)
	{
		(void)Lpc546xxSpifiIdle();
		SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_SPIFI_MASK;
		dev->bOn = false;
	}
}

static void Lpc546xxSpifiEnable(DevIntrf_t * const pDev)
{
	Lpc546xxSpifiDev_t *dev = (Lpc546xxSpifiDev_t *)pDev->pDevData;

	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_SPIFI_MASK;
	dev->bOn = true;
}

static void Lpc546xxSpifiPowerOff(DevIntrf_t * const pDev)
{
	Lpc546xxSpifiDev_t *dev = (Lpc546xxSpifiDev_t *)pDev->pDevData;

	// The reset ends a command in progress
	dev->bPend = false;
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_SPIFI_MASK;
	SYSCON->PRESETCTRLSET[0] = SYSCON_PRESETCTRL_SPIFI_RST_MASK;
	SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_SPIFI_RST_MASK;
	SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_SPIFI_MASK;
	SYSCON->SPIFICLKSEL = SYSCON_SPIFICLKSEL_SEL(LPC546XX_SPIFI_CLKSEL_NONE);
	dev->bOn = false;

	IOPinDis(dev->pSpiDev->Cfg.pIOPinMap, dev->pSpiDev->Cfg.NbIOPins);
}

bool Lpc546xxSpifiInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL || pCfgData->DevNo != LPC546XX_SPIFI_DEVNO ||
		pCfgData->Mode != SPIMODE_MASTER || pCfgData->pIOPinMap == NULL ||
		pCfgData->NbIOPins < QSPI_CS_IOPIN_IDX + 1 || pCfgData->ChipSel != SPICSEL_AUTO ||
		pCfgData->DataSize != 8 || pCfgData->BitOrder != SPIDATABIT_MSB ||
		pCfgData->bIntEn || pCfgData->bDmaEn)
	{
		return false;
	}

	if (pCfgData->Phy != SPIPHY_NORMAL && pCfgData->Phy != SPIPHY_DUAL &&
		pCfgData->Phy != SPIPHY_QUAD_SDR)
	{
		return false;
	}

	bool mode0 = pCfgData->ClkPol == SPICLKPOL_HIGH && pCfgData->DataPhase == SPIDATAPHASE_FIRST_CLK;
	bool mode3 = pCfgData->ClkPol == SPICLKPOL_LOW && pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK;

	if (mode0 == false && mode3 == false)
	{
		return false;
	}

	Lpc546xxSpifiDev_t *dev = &s_Lpc546xxSpifiDev;

	pDev->Cfg = *pCfgData;
	pDev->CurDevCs = 0;
	dev->pSpiDev = pDev;
	dev->bPend = false;
	dev->AddrLen = 3;
	pDev->DevIntrf.pDevData = (void *)dev;

	// Bus clock on and SPIFI reset, its clock from the main clock
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_SPIFI_MASK;
	SYSCON->PRESETCTRLSET[0] = SYSCON_PRESETCTRL_SPIFI_RST_MASK;
	SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_SPIFI_RST_MASK;
	SYSCON->SPIFICLKSEL = SYSCON_SPIFICLKSEL_SEL(LPC546XX_SPIFI_CLKSEL_MAIN);
	dev->bOn = true;

	Lpc546xxSpifiSetRate(&pDev->DevIntrf, pCfgData->Rate);

	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	for (int i = 0; i < pCfgData->NbIOPins; i++)
	{
		IOPinSetSpeed(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	// Mode 0 samples the input on the falling edge with the feedback clock,
	// as the NXP SDK. MODE3, RFCLK and FBCLK are never all set.
	dev->Ctrl = SPIFI_CTRL_TIMEOUT(0xFFFFU) | SPIFI_CTRL_CSHIGH(0xFU) |
				(mode3 ? SPIFI_CTRL_MODE3_MASK | SPIFI_CTRL_FBCLK_MASK :
						 SPIFI_CTRL_RFCLK_MASK | SPIFI_CTRL_FBCLK_MASK);
	SPIFI0->CTRL = dev->Ctrl;

	pDev->DevIntrf.Type = DEVINTRF_TYPE_QSPI;
	pDev->DevIntrf.Disable = Lpc546xxSpifiDisable;
	pDev->DevIntrf.Enable = Lpc546xxSpifiEnable;
	pDev->DevIntrf.GetHandle = Lpc546xxSpifiGetHandle;
	pDev->DevIntrf.GetRate = Lpc546xxSpifiGetRate;
	pDev->DevIntrf.SetRate = Lpc546xxSpifiSetRate;
	pDev->DevIntrf.StartRx = Lpc546xxSpifiStart;
	pDev->DevIntrf.RxData = Lpc546xxSpifiRxData;
	pDev->DevIntrf.StopRx = Lpc546xxSpifiStop;
	pDev->DevIntrf.StartTx = Lpc546xxSpifiStart;
	pDev->DevIntrf.TxData = Lpc546xxSpifiTxData;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.StopTx = Lpc546xxSpifiStop;
	pDev->DevIntrf.Reset = Lpc546xxSpifiReset;
	pDev->DevIntrf.PowerOff = Lpc546xxSpifiPowerOff;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = false;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EnCnt = 1;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

/**
 * @brief	Set Quad SPI Flash size
 *
 * Size in KBytes: above 16 MBytes the commands that give no address size
 * use 4 address bytes, otherwise 3.
 */
void QuadSPISetMemSize(SPIDev_t * const pDev, uint32_t Size)
{
	if (Lpc546xxSpifiIsDev(pDev))
	{
		s_Lpc546xxSpifiDev.AddrLen = Size > 16U * 1024U ? 4 : 3;
	}
}

bool QuadSPISendCmd(SPIDev_t * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	if (Lpc546xxSpifiIsDev(pDev) == false)
	{
		return false;
	}

	return Lpc546xxSpifiSendCmd(&s_Lpc546xxSpifiDev, Cmd, Addr, AddrLen, DataLen, DummyCycle);
}

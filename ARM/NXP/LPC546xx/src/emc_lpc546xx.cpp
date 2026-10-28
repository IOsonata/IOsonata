/**-------------------------------------------------------------------------
@file	emc_lpc546xx.cpp

@brief	LPC546xx EMC, SDRAM on a dynamic chip select (coredev/sdram.h)

SdramInit sets up the EMC and one SDRAM on EMC_DYCS0 to EMC_DYCS3 (DevNo 0
to 3), memory mapped at EMC_DYCSn_BASE, as the NXP SDK fsl_emc.c and the
LPCXpresso54628 board init: pins, EMC clock, address mapping, RAS and CAS
latencies, timings, then the SDRAM power up sequence (NOP, precharge all, 8
or more refreshes, mode register, normal mode) and the EMC buffer. The EMC
is reset first: one SDRAM, the static memory chip selects not used.

Data bus of 16 or 32 bits. The parts of the address mapping table of the
user manual, row-bank-column mapping:

	16 Mb	2 banks, 11 rows	x8 9 columns, x16 8
	64 Mb	4 banks, 12 rows	x8 9 columns, x16 8; x32 11 rows 8 columns
	128 Mb	4 banks, 12 rows	x8 10 columns, x16 9, x32 8
	256 Mb	4 banks, 13 rows	x8 10 columns, x16 9, x32 8
	512 Mb	4 banks, 13 rows	x8 11 columns, x16 10

x32 parts on a 32 bit bus only, the memory in the 128 MB of the chip select.
CAS latency 1 to 3, tMRD 1 to 16 clocks.

The EMC clock is the CPU clock divided by 1 to 256 (EMCCLKDIV), the closest
to Rate. The timings are converted to clocks rounded up, a value above the
register field takes its largest value, tRCD 1 to 3 clocks. tDAL is tWR +
tRP. Reads are command delayed (DYNAMICREADCONFIG RD 1), EMCDLYCTRL keeps
its value at reset: the NXP SDK runs the EMC at half the CPU clock and asks
for a delay calibration above 100 MHz. The mode register: the CAS latency, a
sequential burst of 8 on a 16 bit bus, 4 on a 32 bit bus.

Run it before any access to the SDRAM, from code and data in the internal
memories.

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

#include "idelay.h"
#include "coredev/iopincfg.h"
#include "coredev/sdram.h"
#include "iopinctrl.h"

#ifndef LPC546XX_EMC_READ32
#define LPC546XX_EMC_READ32(Addr)	(*(volatile uint32_t *)(uintptr_t)(Addr))
#endif

#define LPC546XX_EMC_DIV_MAX		256U		// EMCCLKDIV DIV + 1
#define LPC546XX_EMC_NB_CS			4			// Dynamic chip selects
#define LPC546XX_EMC_CS_SIZE		0x08000000U	// Address space of a chip select
#define LPC546XX_EMC_READ_CMD_DLY	1U			// DYNAMICREADCONFIG RD: command delayed
#define LPC546XX_EMC_RAS_MAX		3U
#define LPC546XX_EMC_CAS_MAX		3U
#define LPC546XX_EMC_MRD_MAX		16U

// DYNAMICCONTROL I: SDRAM command
#define LPC546XX_EMC_CMD_NORMAL		0x000U
#define LPC546XX_EMC_CMD_MODE		0x080U
#define LPC546XX_EMC_CMD_PALL		0x100U
#define LPC546XX_EMC_CMD_NOP		0x180U

// Clock enable high and clock running during the power up sequence
#define LPC546XX_EMC_CLK_ON			(EMC_DYNAMICCONTROL_CE_MASK | EMC_DYNAMICCONTROL_CS_MASK)

// DYNAMICREFRESH counts by 16 clocks. 2: a refresh each 32 clocks during the
// power up sequence, which needs 8 of them.
#define LPC546XX_EMC_REFRESH_UNIT	16U
#define LPC546XX_EMC_REFRESH_INIT	2U
#define LPC546XX_EMC_REFRESH_COUNT	8U

// Waits of the power up sequence, NXP SDK fsl_emc.h
#define LPC546XX_EMC_NOP_DELAY_US		100U
#define LPC546XX_EMC_PALL_DELAY_US		100U
#define LPC546XX_EMC_REFRESH_DELAY_US	50U

// Mode register: burst length code of a sequential burst of 8 and of 4
#define LPC546XX_EMC_MODE_BURST8	3U
#define LPC546XX_EMC_MODE_BURST4	2U
#define LPC546XX_EMC_MODE_CAS_POS	4U

// DYNAMICCONFIG address mapping: AM0 bits 1:0 the part width (0 x8, 1 x16,
// 2 x32), bits 4:2 its size (0 16 Mb to 4 512 Mb), bit 5 clear for
// row-bank-column; AM1 set for a 32 bit bus
typedef struct {
	uint8_t Width;
	uint8_t Banks;
	uint8_t Rows;
	uint8_t Cols;
	uint8_t Map;
} Lpc546xxEmcPart_t;

static const Lpc546xxEmcPart_t s_Lpc546xxEmcParts[] = {
	{ 8, 2, 11, 9, 0x00 }, { 16, 2, 11, 8, 0x01 },
	{ 8, 4, 12, 9, 0x04 }, { 16, 4, 12, 8, 0x05 }, { 32, 4, 11, 8, 0x06 },
	{ 8, 4, 12, 10, 0x08 }, { 16, 4, 12, 9, 0x09 }, { 32, 4, 12, 8, 0x0A },
	{ 8, 4, 13, 10, 0x0C }, { 16, 4, 13, 9, 0x0D }, { 32, 4, 13, 8, 0x0E },
	{ 8, 4, 13, 11, 0x10 }, { 16, 4, 13, 10, 0x11 },
};

static const uint32_t s_Lpc546xxEmcBase[LPC546XX_EMC_NB_CS] = EMC_DYCS_ADDRESS;

// Clocks of Ns at Clk rounded up, less Minus, at most Max
static uint32_t Lpc546xxEmcClocks(uint32_t Clk, uint32_t Ns, uint32_t Minus, uint32_t Max)
{
	uint64_t c = ((uint64_t)Clk * Ns + 999999999U) / 1000000000U;

	c = c > Minus ? c - Minus : 0;

	return c > Max ? Max : (uint32_t)c;
}

// The part in the address mapping table, NULL when not there
static const Lpc546xxEmcPart_t *Lpc546xxEmcPart(const SdramCfg_t *pCfg)
{
	for (const Lpc546xxEmcPart_t &p : s_Lpc546xxEmcParts)
	{
		if (p.Width == pCfg->DevWidth && p.Banks == pCfg->NbBanks && p.Rows == pCfg->RowBits &&
			p.Cols == pCfg->ColBits)
		{
			return &p;
		}
	}

	return NULL;
}

bool SdramInit(SdramDev_t * const pDev, const SdramCfg_t * const pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->pIOPinMap == NULL || pCfg->NbIOPins <= 0 ||
		pCfg->DevNo < 0 || pCfg->DevNo >= LPC546XX_EMC_NB_CS)
	{
		return false;
	}

	const Lpc546xxEmcPart_t *part = Lpc546xxEmcPart(pCfg);
	bool bus32 = pCfg->BusWidth == 32U;

	if (part == NULL || (pCfg->BusWidth != 16U && bus32 == false) || pCfg->DevWidth > pCfg->BusWidth)
	{
		return false;
	}

	uint32_t size = ((uint32_t)part->Banks << (part->Rows + part->Cols)) * (pCfg->BusWidth / 8U);

	if (size > LPC546XX_EMC_CS_SIZE ||
		pCfg->CasLatency < 1U || pCfg->CasLatency > LPC546XX_EMC_CAS_MAX ||
		pCfg->TmrdClk < 1U || pCfg->TmrdClk > LPC546XX_EMC_MRD_MAX)
	{
		return false;
	}

	// The CPU clock divided, the closest to Rate, the lower of two as close
	uint32_t cpu = SystemCoreClock;
	uint32_t req = pCfg->Rate;
	uint32_t div = req > 0U ? cpu / req : LPC546XX_EMC_DIV_MAX;

	if (div < 1U)
	{
		div = 1U;
	}
	else if (div < LPC546XX_EMC_DIV_MAX && req - cpu / (div + 1U) <= cpu / div - req)
	{
		div++;
	}
	if (div > LPC546XX_EMC_DIV_MAX)
	{
		div = LPC546XX_EMC_DIV_MAX;
	}

	uint32_t clk = cpu / div;
	uint32_t ras = Lpc546xxEmcClocks(clk, pCfg->TrcdNs, 0, LPC546XX_EMC_RAS_MAX);
	uint32_t mode = ((uint32_t)pCfg->CasLatency << LPC546XX_EMC_MODE_CAS_POS) |
					(bus32 ? LPC546XX_EMC_MODE_BURST4 : LPC546XX_EMC_MODE_BURST8);

	// The mode value goes on the row address lines (NXP SDK EMC_ModeOffset):
	// above the byte lanes, the columns, then the bank bits
	uint32_t rowShift = (bus32 ? 2U : 1U) + part->Cols + (part->Banks == 2U ? 1U : 2U);
	uint32_t base = s_Lpc546xxEmcBase[pCfg->DevNo];

	if (ras < 1U)
	{
		ras = 1U;
	}

	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		IOPinSetSpeed(pCfg->pIOPinMap[i].PortNo, pCfg->pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	// EMC bus clock on and reset, the EMC clock, the feedback clock from the
	// EMC_CLK loop back
	SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_EMC_MASK;
	SYSCON->PRESETCTRLSET[2] = SYSCON_PRESETCTRL_EMC_RESET_MASK;
	SYSCON->PRESETCTRLCLR[2] = SYSCON_PRESETCTRL_EMC_RESET_MASK;
	SYSCON->EMCCLKDIV = SYSCON_EMCCLKDIV_DIV(div - 1U);
	SYSCON->EMCSYSCTRL = 0;

	// Enabled, little endian (CONFIG 0 after the reset)
	EMC->CONTROL = EMC_CONTROL_E_MASK;

	EMC->DYNAMIC[pCfg->DevNo].DYNAMICCONFIG = EMC_DYNAMIC_DYNAMICCONFIG_AM0(part->Map) |
											  EMC_DYNAMIC_DYNAMICCONFIG_AM1(bus32 ? 1U : 0U);
	EMC->DYNAMIC[pCfg->DevNo].DYNAMICRASCAS = EMC_DYNAMIC_DYNAMICRASCAS_RAS(ras) |
											  EMC_DYNAMIC_DYNAMICRASCAS_CAS(pCfg->CasLatency);
	EMC->DYNAMICREADCONFIG = EMC_DYNAMICREADCONFIG_RD(LPC546XX_EMC_READ_CMD_DLY);

	// The registers hold the clocks less one, DAL the clocks
	uint32_t wr = Lpc546xxEmcClocks(clk, pCfg->TwrNs, 0, 0xFFFFU) + pCfg->TwrClk;
	uint32_t rp = Lpc546xxEmcClocks(clk, pCfg->TrpNs, 0, 0xFFFFU);

	EMC->DYNAMICRP = Lpc546xxEmcClocks(clk, pCfg->TrpNs, 1, EMC_DYNAMICRP_TRP_MASK);
	EMC->DYNAMICRAS = Lpc546xxEmcClocks(clk, pCfg->TrasNs, 1, EMC_DYNAMICRAS_TRAS_MASK);
	EMC->DYNAMICSREX = Lpc546xxEmcClocks(clk, pCfg->TsrexNs, 1, EMC_DYNAMICSREX_TSREX_MASK);
	EMC->DYNAMICAPR = Lpc546xxEmcClocks(clk, pCfg->TaprNs, 1, EMC_DYNAMICAPR_TAPR_MASK);
	EMC->DYNAMICDAL = wr + rp > EMC_DYNAMICDAL_TDAL_MASK ? EMC_DYNAMICDAL_TDAL_MASK : wr + rp;
	EMC->DYNAMICWR = wr < 1U ? 0 : (wr - 1U > EMC_DYNAMICWR_TWR_MASK ? EMC_DYNAMICWR_TWR_MASK : wr - 1U);
	EMC->DYNAMICRC = Lpc546xxEmcClocks(clk, pCfg->TrcNs, 1, EMC_DYNAMICRC_TRC_MASK);
	EMC->DYNAMICRFC = Lpc546xxEmcClocks(clk, pCfg->TrfcNs, 1, EMC_DYNAMICRFC_TRFC_MASK);
	EMC->DYNAMICXSR = Lpc546xxEmcClocks(clk, pCfg->TxsrNs, 1, EMC_DYNAMICXSR_TXSR_MASK);
	EMC->DYNAMICRRD = Lpc546xxEmcClocks(clk, pCfg->TrrdNs, 1, EMC_DYNAMICRRD_TRRD_MASK);
	EMC->DYNAMICMRD = EMC_DYNAMICMRD_TMRD(pCfg->TmrdClk - 1U);

	// Power up: NOP, precharge all, refreshes, mode register, normal
	usDelay(LPC546XX_EMC_NOP_DELAY_US);
	EMC->DYNAMICCONTROL = LPC546XX_EMC_CLK_ON | LPC546XX_EMC_CMD_NOP;
	usDelay(LPC546XX_EMC_PALL_DELAY_US);
	EMC->DYNAMICCONTROL = LPC546XX_EMC_CLK_ON | LPC546XX_EMC_CMD_PALL;
	EMC->DYNAMICREFRESH = LPC546XX_EMC_REFRESH_INIT;

	// At least 8 refreshes, at a slow clock more than the SDK wait
	uint32_t us = (uint32_t)(((uint64_t)LPC546XX_EMC_REFRESH_COUNT * LPC546XX_EMC_REFRESH_INIT *
							  LPC546XX_EMC_REFRESH_UNIT * 1000000U + clk - 1U) / clk) + 1U;

	usDelay(us > LPC546XX_EMC_REFRESH_DELAY_US ? us : LPC546XX_EMC_REFRESH_DELAY_US);

	uint32_t refresh = Lpc546xxEmcClocks(clk, pCfg->RefreshNs, 0, 0xFFFFFFFFU) / LPC546XX_EMC_REFRESH_UNIT;

	if (refresh < 1U)
	{
		refresh = 1U;
	}
	if (refresh > EMC_DYNAMICREFRESH_REFRESH_MASK)
	{
		refresh = EMC_DYNAMICREFRESH_REFRESH_MASK;
	}
	EMC->DYNAMICREFRESH = refresh;

	// The mode value is the row address of a read, bank 0
	EMC->DYNAMICCONTROL = LPC546XX_EMC_CLK_ON | LPC546XX_EMC_CMD_MODE;
	(void)LPC546XX_EMC_READ32(base + (mode << rowShift));

	EMC->DYNAMICCONTROL = LPC546XX_EMC_CMD_NORMAL;

	// The buffer is enabled after the set up, NXP SDK
	EMC->DYNAMIC[pCfg->DevNo].DYNAMICCONFIG |= EMC_DYNAMIC_DYNAMICCONFIG_B_MASK;

	pDev->Cfg = *pCfg;
	pDev->Rate = clk;
	pDev->pBase = (void *)(uintptr_t)base;
	pDev->Size = size;

	return true;
}

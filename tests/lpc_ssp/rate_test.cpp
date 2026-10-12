/*--------------------------------------------------------------------------
File   : rate_test.cpp

Author : Hoang Nguyen Hoan          Jan. 16, 2012

Desc   : Host regression tests for the legacy LPC SSP clock

Copyright (c) 2012, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstddef>
#include <initializer_list>
#include <sys/mman.h>
#include "spi_lpcxx.h"

static const uint32_t s_Clocks[] = {12000000U, 48000000U, 120000000U, UINT32_MAX};
static const uint32_t s_Rates[] = {0, 1, 738, 1000, 100000, 1234567,
	10000000, 24000000, 48000000, UINT32_MAX};
static const IOPinCfg_t s_Pins[3] = {};
uint32_t SystemCoreClock = 12000000;
uint32_t SystemMainClkFreq = 48000000;

void NVIC_DisableIRQ(IRQn_Type)
{
}

void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}

#include "../../ARM/NXP/src/spi_lpcxx.c"
#ifndef TEST_LPC17
#include "../../ARM/NXP/LPC11xx/src/spi_lpc11uxx.c"
#endif

// Simulate status in mapped peripheral memory, retaining the vendor's
// read-only SR declaration in the driver.
static void SetStatus(LpcSSPReg_t *Reg, uint32_t Status)
{
	memcpy((uint8_t *)Reg + offsetof(LpcSSPReg_t, SR), &Status, sizeof(Status));
}

static void CheckRate(SPIDev_t *Spi, SSPDEV *Dev, uint32_t Request)
{
	LpcSSPReg_t *reg = Dev->pSspReg;
	uint32_t cr0 = reg->CR0 & ~LPCSSP_CR0_SCR_MASK;
	uint32_t cr1 = reg->CR1;
	uint32_t actual = LpcSSPSetRate(&Spi->DevIntrf, Request);
	uint32_t prescale = reg->CPSR;
	uint32_t serial = ((reg->CR0 & LPCSSP_CR0_SCR_MASK) >> 8) + 1U;

	assert(prescale >= 2 && prescale <= 254 && (prescale & 1U) == 0);
	assert(serial >= 1 && serial <= 256);
	assert((reg->CR0 & ~LPCSSP_CR0_SCR_MASK) == cr0);
	assert(reg->CR1 == cr1);
	uint32_t divisor = prescale * serial;
	assert(actual == ((uint64_t)Dev->PClkFreq + divisor / 2U) / divisor);
	assert(Spi->Cfg.Rate == actual);

	// Independent oracle: evaluate every legal hardware pair, without the
	// driver's floor/ceiling candidate reduction or integer comparison.
	long double chosen = fabsl((long double)Dev->PClkFreq / divisor - Request);
	for (unsigned p = 2; p <= 254; p += 2)
	{
		for (unsigned s = 1; s <= 256; s++)
		{
			long double error = fabsl((long double)Dev->PClkFreq / (p * s) - Request);
			assert(chosen <= error + 0.000001L);
		}
	}
}

int main()
{
	void *memory = mmap((void *)0x40000000, 0x200000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	assert(memory == (void *)0x40000000);
	SPIDev_t spi = {};
	SSPDEV dev = {0, 48000000, LPC_SSP0, &spi};
	spi.DevIntrf.pDevData = &dev;
	LPC_SSP0->CR0 = 0xC7U;
	LPC_SSP0->CR1 = LPCSSP_CR1_SSP_EN | LPCSSP_CR1_LBM_EN;
	SetStatus(LPC_SSP0, LPCSSP_SR_TFE | LPCSSP_SR_TNF);
	for (uint32_t clock : s_Clocks)
	{
		dev.PClkFreq = clock;
		for (uint32_t rate : s_Rates)
		{
			CheckRate(&spi, &dev, rate);
		}
	}
	dev.PClkFreq = 48000000;
	assert(LpcSSPSetRate(&spi.DevIntrf, 1000000) == 1000000);
	assert(LpcSSPSetRate(&spi.DevIntrf, UINT32_MAX) == 24000000);
	assert(LpcSSPSetRate(&spi.DevIntrf, 0) == 738);
	assert(LPC_SSP0->CPSR == 254 && (LPC_SSP0->CR0 >> 8) == 255);

	uint32_t cr0 = LPC_SSP0->CR0;
	uint32_t cr1 = LPC_SSP0->CR1;
	uint32_t cpsr = LPC_SSP0->CPSR;
	for (uint32_t status : {0U, (uint32_t)(LPCSSP_SR_BSY | LPCSSP_SR_TFE)})
	{
		SetStatus(LPC_SSP0, status);
		assert(LpcSSPSetRate(&spi.DevIntrf, 1000000) == 738);
		assert(LPC_SSP0->CR0 == cr0 && LPC_SSP0->CR1 == cr1 && LPC_SSP0->CPSR == cpsr);
	}
	SetStatus(LPC_SSP0, LPCSSP_SR_TFE);
	LPC_SSP0->CR1 = 0;
	CheckRate(&spi, &dev, 7000000);
	assert(LPC_SSP0->CR1 == 0);
	dev.PClkFreq = 0;
	assert(LpcSSPSetRate(&spi.DevIntrf, 1000000) == 0);
	assert(LpcSSPSetRate(nullptr, 1000000) == 0);
	DevIntrf_t empty = {};
	assert(LpcSSPSetRate(&empty, 1000000) == 0);

#ifndef TEST_LPC17
	SPICfg_t cfg = {};
	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 3;
	cfg.ChipSel = SPICSEL_MAN;
	cfg.Rate = 1000000;
	cfg.DataSize = 8;
	// The unused SSP divider remains zero, as after reset. AHB is slower
	// than main clock: the SSP source must still use 48 MHz.
	LPC_SYSCON->SYSAHBCLKDIV = 4;
	LPC_SYSCON->SSP1CLKDIV = 0;
	assert(SPIInit(&spi, &cfg));
	assert(g_SspDev[0].PClkFreq == 48000000);
	assert(spi.Cfg.Rate == 1000000);
	assert(LPC_SYSCON->SSP1CLKDIV == 0);
	SPIDev_t spi1 = {};
	cfg.DevNo = 1;
	SetStatus(LPC_SSP1, LPCSSP_SR_TFE);
	LPC_SYSCON->SSP0CLKDIV = 7;
	assert(SPIInit(&spi1, &cfg));
	assert(g_SspDev[1].PClkFreq == 48000000);
	assert(LPC_SYSCON->SSP0CLKDIV == 7);
	cfg.DevNo = -1;
	assert(!SPIInit(&spi, &cfg));
	cfg.DevNo = 2;
	assert(!SPIInit(&spi, &cfg));
	assert(!SPIInit(nullptr, &cfg));
	assert(!SPIInit(&spi, nullptr));
#endif
	puts("Legacy LPC SSP rate: PASS");
	munmap(memory, 0x200000);
	return 0;
}

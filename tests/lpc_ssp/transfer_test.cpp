/*--------------------------------------------------------------------------
File   : transfer_test.cpp

Author : Hoang Nguyen Hoan          Jan. 16, 2012

Desc   : Host regression tests for the legacy LPC SSP transfers

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
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>
#include <cstddef>
#include <sys/mman.h>
#include "spi_lpcxx.h"
#include "iopinctrl.h"

struct SspModel {
	std::deque<uint32_t> Rx;
	std::vector<uint32_t> Tx;
	std::vector<int> Cs;
	bool TxStall = false;
	bool BusyStall = false;
	bool BusyOnRead = false;
	int RxFailAt = -1;
	int BusyDelay = 0;
	int Resets = 0;
};
static SspModel s_Model;
static SPIDev_t s_Spi;
static const IOPinCfg_t s_Pins[] = {
	{0, 6, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 8, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 9, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 2, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 3, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}
};
uint32_t SystemCoreClock = 48000000;
uint32_t SystemMainClkFreq = 48000000;

void NVIC_DisableIRQ(IRQn_Type)
{
}
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}
static void ModelPinSet(int, int Pin)
{
	assert(s_Model.Rx.empty());
	assert(!s_Model.BusyStall);
	assert(s_Model.BusyDelay == 0);
	s_Model.Cs.push_back(Pin);
}
static void ModelPinClear(int, int Pin)
{
	s_Model.Cs.push_back(-Pin);
}
static uint32_t ModelRead(LpcSSPReg_t *, const char *Field)
{
	if (!strcmp(Field, "SR"))
	{
		uint32_t status = s_Model.TxStall ? 0 : LPCSSP_SR_TNF;
		if (!s_Model.Rx.empty() && (s_Model.RxFailAt < 0 ||
			(int)s_Model.Tx.size() < s_Model.RxFailAt))
		{
			status |= LPCSSP_SR_RNE;
		}
		if (s_Model.BusyStall || s_Model.BusyDelay > 0)
		{
			status |= LPCSSP_SR_BSY;
			if (s_Model.BusyDelay > 0)
			{
				s_Model.BusyDelay--;
			}
		}
		else
		{
			status |= LPCSSP_SR_TFE;
		}
		return status;
	}
	assert(!strcmp(Field, "DR") && !s_Model.Rx.empty());
	uint32_t data = s_Model.Rx.front();
	s_Model.Rx.pop_front();
	if (s_Model.BusyOnRead)
	{
		s_Model.BusyStall = true;
	}
	return data;
}
static void ModelWrite(LpcSSPReg_t *, const char *Field, uint32_t Value)
{
	if (!strcmp(Field, "ICR"))
	{
		return;
	}
	assert(!strcmp(Field, "DR"));
	assert(s_Model.Rx.empty());
	s_Model.Tx.push_back(Value);
	s_Model.Rx.push_back(Value);
	s_Model.BusyDelay = 3;
}

#undef LPCSSP_READ
#undef LPCSSP_WRITE
#define LPCSSP_READ(Reg, Field) ModelRead(Reg, #Field)
#define LPCSSP_WRITE(Reg, Field, Value) ModelWrite(Reg, #Field, Value)
#define IOPinSet ModelPinSet
#define IOPinClear ModelPinClear
#include "../../ARM/NXP/src/spi_lpcxx.c"
#include "../../ARM/NXP/LPC11xx/src/spi_lpc11uxx.c"
#include "../../src/device_intrf.cpp"

static void ModelReset(DevIntrf_t *Dev)
{
	s_Model.Rx.clear();
	s_Model.BusyDelay = 0;
	s_Model.BusyStall = false;
	s_Model.Resets++;
	Lpc11SSPReset(Dev);
}
static SPICfg_t Config()
{
	SPICfg_t cfg = {};
	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 5;
	cfg.DataSize = 8;
	cfg.Rate = 1000000;
	cfg.DummyByte = 0xA5;
	return cfg;
}
static void Init(SPICfg_t Cfg)
{
	s_Model = {};
	assert(SPIInit(&s_Spi, &Cfg));
	s_Spi.DevIntrf.Reset = ModelReset;
	s_Model.Cs.clear();
}

int main()
{
	void *memory = mmap((void *)0x40000000, 0x200000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	assert(memory == (void *)0x40000000);
	uint32_t ready = LPCSSP_SR_TFE | LPCSSP_SR_TNF;
	memcpy((uint8_t *)LPC_SSP0 + offsetof(LpcSSPReg_t, SR), &ready, sizeof(ready));
	SPICfg_t cfg = Config();
	uint8_t tx[129];
	uint8_t rx[129];
	for (int i = 0; i < 129; i++)
	{
		tx[i] = (uint8_t)i;
	}
	for (int mode = 0; mode < 4; mode++)
	{
		cfg.ClkPol = mode & 2 ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
		cfg.DataPhase = mode & 1 ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
		Init(cfg);
		assert((LPC_SSP0->CR0 & 0xC0U) == (uint32_t)(((mode & 2) << 5) | ((mode & 1) << 7)));
		assert(SPITx(&s_Spi, 1, tx, sizeof(tx)) == 129);
		assert(s_Model.Cs == std::vector<int>({-3, 3}));
		assert(s_Model.Tx.size() == 129 && s_Model.Rx.empty());
		for (int i = 0; i < 129; i++)
		{
			assert(s_Model.Tx[i] == tx[i]);
		}
		s_Model.Cs.clear();
		assert(SPIRx(&s_Spi, 0, rx, sizeof(rx)) == 129);
		assert(s_Model.Cs == std::vector<int>({-2, 2}));
		for (uint8_t byte : rx)
		{
			assert(byte == cfg.DummyByte);
		}
		s_Model.Cs.clear();
		assert(DeviceIntrfRead(&s_Spi.DevIntrf, 1, tx, 3, rx, 16) == 16);
		assert(s_Model.Cs == std::vector<int>({-3, 3}));
		assert(s_Spi.FirstRdData == tx[0]);
	}
	Init(cfg);
	assert(!SPIStartTx(&s_Spi, 2));
	assert(SPIStartTx(&s_Spi, 0));
	assert(!SPIStartTx(&s_Spi, 0));
	assert(!SPIInit(&s_Spi, &cfg));
	assert(SPISetRate(&s_Spi, 2000000) == 1000000);
	SPIStopTx(&s_Spi);
	assert(SPIStartTx(&s_Spi, 0));
	SPIStopTx(&s_Spi);
	assert(SPIGetHandle(&s_Spi.DevIntrf) == &s_Spi);

	cfg.DataSize = 16;
	Init(cfg);
	assert(SPITx(&s_Spi, 0, tx + 1, 16) == 16);
	assert(s_Model.Tx[0] == 0x0201);
	assert(SPIRx(&s_Spi, 0, rx + 1, 16) == 16);
	assert(rx[1] == 0xA5 && rx[2] == 0xA5);
	size_t frames = s_Model.Tx.size();
	assert(SPITx(&s_Spi, 0, tx + 1, 15) == 0);
	assert(s_Model.Tx.size() == frames);
	assert(s_Model.Resets == 1);
	cfg.DataSize = 4;
	Init(cfg);
	assert(SPIRx(&s_Spi, 0, rx, 1) == 1 && rx[0] == 5);

	cfg.DataSize = 8;
	Init(cfg);
	s_Model.RxFailAt = 2;
	assert(DeviceIntrfRead(&s_Spi.DevIntrf, 0, tx, 3, rx, 16) == 0);
	assert(s_Model.Tx.size() == 2);
	assert(s_Model.Resets == 1 && s_Model.Rx.empty());
	assert(s_Model.Cs == std::vector<int>({-2, 2}));
	s_Model.RxFailAt = -1;
	assert(SPIRx(&s_Spi, 0, rx, 16) == 16);
	Init(cfg);
	s_Model.TxStall = true;
	assert(SPITx(&s_Spi, 0, tx, 1) == 0);
	assert(s_Model.Resets == 1 && s_Model.Tx.empty());
	Init(cfg);
	s_Model.BusyStall = true;
	assert(SPITx(&s_Spi, 0, tx, 1) == 0);
	assert(s_Model.Resets == 1 && s_Model.Cs.empty());

	Init(cfg);
	s_Model.BusyOnRead = true;
	assert(SPITx(&s_Spi, 0, tx, 1) == 0);
	assert(s_Model.Resets == 1);
	assert(s_Model.Cs == std::vector<int>({-2, 2}));
	Init(cfg);
	for (int i = 0; i < 8; i++)
	{
		s_Model.Rx.push_back(0xEE);
	}
	assert(SPITx(&s_Spi, 0, tx, 1) == 1);
	assert(s_Spi.FirstRdData == tx[0]);
	uint32_t format = LPC_SSP0->CR0;
	uint32_t prescale = LPC_SSP0->CPSR;
	DeviceIntrfReset(&s_Spi.DevIntrf);
	assert(LPC_SSP0->CR0 == format && LPC_SSP0->CPSR == prescale);
	assert(LPC_SSP0->CR1 == LPCSSP_CR1_SSP_EN);
	SPIPowerOff(&s_Spi);
	assert((LPC_SYSCON->SYSAHBCLKCTRL & LPC11U_SYSAHBCLKCTRL_SSP0_EN) == 0);
	assert(!g_SspDev[0].bEnabled);
	Init(cfg);
	SPIEnable(&s_Spi);
	SPIDisable(&s_Spi);
	assert(g_SspDev[0].bEnabled);
	SPIDisable(&s_Spi);
	assert(!g_SspDev[0].bEnabled);
	assert(SPITx(&s_Spi, 0, tx, 1) == 0);
	SPIEnable(&s_Spi);
	assert(SPITx(&s_Spi, 0, tx, 1) == 1);
	cfg.ChipSel = SPICSEL_MAN;
	Init(cfg);
	assert(SPITx(&s_Spi, 77, tx, 1) == 1);
	assert(s_Model.Cs.empty());
	cfg.bIntEn = true;
	assert(!SPIInit(&s_Spi, &cfg));
	cfg.bIntEn = false;
	cfg.bDmaEn = true;
	assert(!SPIInit(&s_Spi, &cfg));
	cfg.bDmaEn = false;
	cfg.Mode = SPIMODE_SLAVE;
	assert(!SPIInit(&s_Spi, &cfg));
	cfg.Mode = SPIMODE_MASTER;
	cfg.BitOrder = SPIDATABIT_LSB;
	assert(!SPIInit(&s_Spi, &cfg));
	// Updated GPIO mapping accepts debug pins as GPIO CS and bounds P0 at 23.
	IOPinCfg_t pins[5];
	memcpy(pins, s_Pins, sizeof(pins));
	cfg.pIOPinMap = pins;
	cfg.BitOrder = SPIDATABIT_MSB;
	cfg.ChipSel = SPICSEL_AUTO;
	pins[3].PinNo = 14;
	Init(cfg);
	assert(SPITx(&s_Spi, 0, tx, 1) == 1);
	assert(s_Model.Cs == std::vector<int>({-14, 14}));
	pins[3].PinNo = 24;
	assert(!SPIInit(&s_Spi, &cfg));
	pins[3].PinNo = 2;
	pins[3].PinOp = IOPINOP_FUNC1;
	assert(!SPIInit(&s_Spi, &cfg));
	pins[3].PinOp = IOPINOP_GPIO;
	pins[0].PinOp = IOPINOP_FUNC8;
	assert(!SPIInit(&s_Spi, &cfg));
	puts("Legacy LPC SSP transfer: PASS");
	munmap(memory, 0x200000);
	return 0;
}

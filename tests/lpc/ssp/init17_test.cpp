/*--------------------------------------------------------------------------
File   : init17_test.cpp

Desc   : LPC17xx SSP initialization and polling transfer regressions

Copyright (c) 2026 I-SYST inc. MIT License.
----------------------------------------------------------------------------*/
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <deque>
#include <sys/mman.h>
#include "spi_lpcxx.h"
#include "iopinctrl.h"

static std::deque<uint32_t> s_Rx;
static bool s_Stall;
static unsigned s_Disabled;
static unsigned s_Cleared;
static SPIDev_t s_Spi[2];
static const IOPinCfg_t s_Pins[2][4] = {
	{{0, 15, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 17, IOPINOP_FUNC2, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 18, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 16, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}},
	{{0, 7, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 8, IOPINOP_FUNC2, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 9, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 6, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}}
};
uint32_t SystemCoreClock = 120000000;

// Polling SSP initialization must not enable IRQs or set GPIO priorities.
void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	(void)Irq;
	(void)Priority;
	assert(false);
}

void NVIC_EnableIRQ(IRQn_Type Irq)
{
	(void)Irq;
	assert(false);
}

void NVIC_DisableIRQ(IRQn_Type Irq)
{
	s_Disabled |= 1U << (Irq == SSP0_IRQn ? 0 : 1);
}
void NVIC_ClearPendingIRQ(IRQn_Type Irq)
{
	s_Cleared |= 1U << (Irq == SSP0_IRQn ? 0 : 1);
}
static uint32_t Read(LpcSSPReg_t *Reg, const char *Field)
{
	if (!strcmp(Field, "SR"))
	{
		return s_Stall ? LPCSSP_SR_BSY : Reg->SR | (s_Rx.empty() ? 0 : LPCSSP_SR_RNE);
	}
	assert(!s_Rx.empty());
	uint32_t data = s_Rx.front();

	s_Rx.pop_front();
	return data;
}
static void Write(LpcSSPReg_t *Reg, const char *Field, uint32_t Value)
{
	if (!strcmp(Field, "DR"))
	{
		s_Rx.push_back(Value);
	}
	else
	{
		Reg->ICR = Value;
	}
}
#undef LPCSSP_READ
#undef LPCSSP_WRITE
#define LPCSSP_READ(Reg, Field) Read(Reg, #Field)
#define LPCSSP_WRITE(Reg, Field, Value) Write(Reg, #Field, Value)
#include "../../../ARM/NXP/src/spi_lpcxx.c"
#include "../../../ARM/NXP/LPC17xx/src/spi_lpc17xx.c"
#include "../../../ARM/NXP/LPC17xx/src/iopincfg.c"
#include "../../../src/device_intrf.cpp"

static SPICfg_t Config(int Dev)
{
	SPICfg_t cfg = {};

	cfg.DevNo = Dev;
	cfg.pIOPinMap = s_Pins[Dev];
	cfg.NbIOPins = 4;
	cfg.DataSize = 8;
	cfg.Rate = 1000000;
	cfg.DummyByte = 0xA5;
	return cfg;
}
static void Reject(SPIDev_t *Dev, const SPICfg_t &Cfg)
{
	unsigned char regs[0x100000];
	unsigned char object[sizeof(*Dev)];

	memcpy(regs, (void *)0x40000000, sizeof(regs));
	memcpy(object, Dev, sizeof(object));
	assert(!SPIInit(Dev, &Cfg));
	assert(!memcmp(regs, (void *)0x40000000, sizeof(regs)));
	assert(!memcmp(object, Dev, sizeof(object)));
}
int main()
{
	assert(mmap((void *)0x40000000, 0x100000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x40000000);
	assert(mmap((void *)LPC_GPIO_BASE, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)LPC_GPIO_BASE);
	uint32_t ready = LPCSSP_SR_TFE | LPCSSP_SR_TNF;

	memcpy((char *)LPC_SSP0 + offsetof(LpcSSPReg_t, SR), &ready, sizeof(ready));
	memcpy((char *)LPC_SSP1 + offsetof(LpcSSPReg_t, SR), &ready, sizeof(ready));
	LPC_SC->PCLKSEL0 = UINT32_MAX;
	LPC_SC->PCLKSEL1 = UINT32_MAX;
	LPC_SC->PCONP = 1U << 3;
	for (int dev = 0; dev < 2; dev++)
	{
		SPICfg_t cfg = Config(dev);
		LpcSSPReg_t *reg = dev == 0 ? LPC_SSP0 : LPC_SSP1;
		IOPinCfg_t pins[4];
		SPIDev_t other = {};
		uint8_t bytes[130];

		for (int mode = 0; mode < 4; mode++)
		{
			cfg.ClkPol = mode & 2 ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
			cfg.DataPhase = mode & 1 ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
			reg->DMACR = 3;
			s_Rx.push_back(0xEE);
			assert(SPIInit(&s_Spi[dev], &cfg));
			assert(LPC_SC->PCONP & s_PowerMask[dev]);
			assert(g_SspDev[dev].PClkFreq == SystemCoreClock);
			assert(SPIGetRate(&s_Spi[dev]) == 1000000);
			assert(LPC_GPIO0->FIOSET == (1U << s_Pins[dev][3].PinNo));
			if (dev == 0)
			{
				assert((LPC_PINCON->PINSEL0 >> 30) == 2);
				assert((LPC_PINCON->PINSEL1 & 0x3FU) == 0x28U);
			}
			else
			{
				assert(((LPC_PINCON->PINSEL0 >> 12) & 0xFFU) == 0xA8U);
			}
			assert(s_Rx.empty());
			assert(reg->CR0 % 256 == (uint32_t)(7 | ((mode & 2) << 5) | ((mode & 1) << 7)));
			assert(reg->CR1 == 2 && reg->DMACR == 0 && reg->IMSC == 0);
			assert(SPIRx(&s_Spi[dev], 0, bytes, 129) == 129);
			for (int i = 0; i < 129; i++)
			{
				assert(bytes[i] == 0xA5);
			}
			assert(DeviceIntrfRead(&s_Spi[dev].DevIntrf, 0, bytes, 3, bytes + 3, 16) == 16);
		}
		assert(SPISetRate(&s_Spi[dev], UINT32_MAX) == 60000000);
		assert(SPISetRate(&s_Spi[dev], 0) == 1845);
		Reject(&other, cfg);
		assert(SPIStartTx(&s_Spi[dev], 0));
		Reject(&s_Spi[dev], cfg);
		SPIStopTx(&s_Spi[dev]);
		cfg.DataSize = 16;
		assert(SPIInit(&s_Spi[dev], &cfg));
		assert(SPIRx(&s_Spi[dev], 0, bytes + 1, 16) == 16);
		assert(SPIRx(&s_Spi[dev], 0, bytes, 15) == 0);
		assert(!g_SspDev[dev].bError);
		memcpy(pins, s_Pins[dev], sizeof(pins));
		cfg.pIOPinMap = pins;
		for (int route = 0; route < (dev == 0 ? 8 : 2); route++)
		{
			for (int signal = 0; signal < 3; signal++)
			{
				pins[signal] = s_Pins[dev][signal];
				if (dev == 0 && (route & (1 << signal)))
				{
					const int alternate[] = {20, 23, 24};
					pins[signal].PortNo = 1;
					pins[signal].PinNo = alternate[signal];
					pins[signal].PinOp = IOPINOP_FUNC3;
				}
			}
			if (dev == 1 && route)
			{
				pins[0].PortNo = 1;
				pins[0].PinNo = 31;
			}
			assert(SPIInit(&s_Spi[dev], &cfg));
		}
		pins[0].PinOp = IOPINOP_GPIO;
		Reject(&s_Spi[dev], cfg);
		memcpy(pins, s_Pins[dev], sizeof(pins));
		pins[3] = pins[0];
		Reject(&s_Spi[dev], cfg);
		pins[3] = s_Pins[dev][3];
		pins[3].PinNo = 12;
		Reject(&s_Spi[dev], cfg);
		cfg = Config(dev);
		cfg.bDmaEn = true;
		Reject(&s_Spi[dev], cfg);
		cfg.bDmaEn = false;
		cfg.bIntEn = true;
		Reject(&s_Spi[dev], cfg);
		cfg.bIntEn = false;
		cfg.Mode = SPIMODE_SLAVE;
		Reject(&s_Spi[dev], cfg);
		cfg = Config(dev);
		assert(SPIInit(&s_Spi[dev], &cfg));
		s_Stall = true;
		DeviceIntrfReset(&s_Spi[dev].DevIntrf);
		assert(g_SspDev[dev].bError && !g_SspDev[dev].bEnabled && reg->CR1 == 0);
		s_Spi[dev].DevIntrf.Enable(&s_Spi[dev].DevIntrf);
		assert(reg->CR1 == 0);
		s_Stall = false;
		assert(SPIInit(&s_Spi[dev], &cfg));
		SPIPowerOff(&s_Spi[dev]);
		assert(!(LPC_SC->PCONP & s_PowerMask[dev]));
		assert(LPC_SC->PCONP & (1U << 3));
	}
	assert(LPC_SC->PCLKSEL0 == ((UINT32_MAX & ~(3U << 20)) | (1U << 20)));
	assert(LPC_SC->PCLKSEL1 == ((UINT32_MAX & ~(3U << 10)) | (1U << 10)));
	assert(s_Disabled == 3 && s_Cleared == 3);
	puts("LPC17xx SSP initialization and polling: PASS");
}

/**-------------------------------------------------------------------------
@file	pdm_fw.cpp

@brief	STM32WBA PDM test firmware.

Production PDM port with a modeled DMA transport and the real CIC/CFifo.
Run by run_pdm.py.

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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
#include "stm32wbaxx.h"
#include "coredev/pdm.h"
#include "gpdma_stm32wba.h"

static volatile uint32_t *const s_Io = (volatile uint32_t *)0x500FF000;
static unsigned s_Checks;
static unsigned s_Configured;
static unsigned s_Events[DEVINTRF_EVT_COMPLETED + 1];
static bool s_Allocate = true;
static bool s_DmaUsed;
static bool s_DmaRunning;
static bool s_DmaIrq;
static uint8_t *s_pDma;
static uint32_t s_DmaFlags;
static uint32_t s_DmaLatched;
static WbaGpdmaHandler_t s_DmaHandler;
static void *s_pDmaCtx;
alignas(4) static uint8_t s_FifoMem[CFIFO_TOTAL_MEMSIZE(4, 64)];
static const IOPinCfg_t s_Pins[] = {
	{ IOPORTA, 1, IOPINOP_FUNC3, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ IOPORTA, 2, IOPINOP_FUNC3, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }
};

static void Check(bool Ok)
{
	s_Io[1] = ++s_Checks | (Ok ? 0 : 0x80000000U);
}

static void Event(PdmDev_t *, DEVINTRF_EVT Evt)
{
	s_Events[Evt]++;
}

extern "C" void *memset(void *pDest, int Val, size_t Count)
{
	volatile uint8_t *dest = (volatile uint8_t *)pDest;
	while (Count--)
	{
		*dest++ = Val;
	}
	return pDest;
}

extern "C" void *memcpy(void *pDest, const void *pSrc, size_t Count)
{
	volatile uint8_t *dest = (volatile uint8_t *)pDest;
	const volatile uint8_t *src = (const volatile uint8_t *)pSrc;
	while (Count--)
	{
		*dest++ = *src++;
	}
	return pDest;
}

extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	s_Configured++;
}

extern "C" int WbaGpdmaAlloc(WbaGpdmaHandler_t Handler, void *pCtx, int Prio, bool Irq)
{
	Check(Prio == 6 && !s_DmaUsed);
	if (!s_Allocate)
	{
		return -1;
	}
	s_DmaHandler = Handler;
	s_pDmaCtx = pCtx;
	s_DmaIrq = Irq;
	s_DmaUsed = true;
	return 3;
}

extern "C" void WbaGpdmaFree(int Ch)
{
	Check(Ch == 3 && s_DmaUsed);
	s_DmaUsed = false;
	s_DmaRunning = false;
	s_DmaFlags = 0;
	s_DmaLatched = 0;
}

extern "C" void WbaGpdmaStop(int Ch)
{
	Check(Ch == 3);
	s_DmaRunning = false;
	s_DmaFlags = 0;
	s_DmaLatched = 0;
}

extern "C" void WbaGpdmaSetup(int Ch, uint32_t Req, bool ToPeriph, volatile void *pPeriph, int Width, bool Irq)
{
	Check(Ch == 3 && Req == 17 && !ToPeriph && pPeriph == &SAI1_Block_A->DR && Width == 1);
	Check(Irq == s_DmaIrq && !(SAI1_Block_A->CR1 & SAI_xCR1_SAIEN));
	Check((SAI1_Block_A->CR1 & (SAI_xCR1_MODE | SAI_xCR1_DS | SAI_xCR1_NODIV)) ==
		  (SAI_xCR1_MODE_0 | SAI_xCR1_DS_2 | SAI_xCR1_NODIV));
	Check(SAI1_Block_A->FRCR == (15U | SAI_xFRCR_FSPOL));
	Check(SAI1_Block_A->SLOTR == (1U << SAI_xSLOTR_SLOTEN_Pos));
}

extern "C" void WbaGpdmaStart(int Ch, const void *pMem, uint32_t Count)
{
	Check(Ch == 3 && s_DmaUsed && Count == 512 && !(uintptr_t(pMem) & 3U));
	s_pDma = (uint8_t *)pMem;
	s_DmaRunning = true;
	s_DmaFlags = 0;
	s_DmaLatched = 0;
}

extern "C" uint32_t WbaGpdmaFlags(int Ch)
{
	Check(Ch == 3);
	return s_DmaFlags | s_DmaLatched;
}

extern "C" void WbaGpdmaAck(int Ch)
{
	Check(Ch == 3);
	s_DmaLatched |= s_DmaFlags;
	s_DmaFlags = 0;
}

static void Capture(uint16_t Word, bool InvalidPrefix = false)
{
	uint16_t *dest = (uint16_t *)s_pDma;

	Check(s_DmaRunning && (SAI1_Block_A->CR1 & SAI_xCR1_SAIEN));
	for (unsigned i = 0; i < 256; i++)
	{
		dest[i] = InvalidPrefix && i < 2 ? uint16_t(~Word) : Word;
	}
	s_DmaRunning = false;
	s_DmaFlags = WBA_GPDMA_FLAG_TC;
	if (s_DmaIrq)
	{
		s_DmaHandler(3, s_pDmaCtx);
	}
}

static int16_t *ReadBlock(PdmDev_t *pDev, uint16_t Word, bool Prefix = false)
{
	int16_t *result = nullptr;

	for (unsigned i = 0; i < 3 && !result; i++)
	{
		Capture(Word, Prefix && i == 0);
		result = (int16_t *)PdmGetSamples(pDev);
	}
	Check(result != nullptr);
	return result;
}

extern "C" void SAI1_IRQHandler();

static void PdmChecks()
{
	PdmDev_t dev = {};
	PdmDev_t other = {};
	PdmCfg_t cfg = {
		.pPins = s_Pins,
		.NbPins = 2,
		.Freq = 1032000,
		.SmplMode = PDM_SMPLMODE_FALLING,
		.OpMode = PDM_OPMODE_MONO_LEFT,
		.GainLeft = 0,
		.GainRight = 0,
		.bIntEn = true,
		.IntPrio = 6,
		.EvtHandler = Event,
		.pFifoMem = s_FifoMem,
		.FifoMemSize = sizeof(s_FifoMem),
		.FifoBlkSize = 64
	};
	PdmCfg_t bad;
	int16_t *pcm;
	unsigned eventCount;
	unsigned sampleCount;
	IOPinCfg_t pins[2];

	Check(!PdmInit(nullptr, &cfg) && !PdmInit(&dev, nullptr));
	bad = cfg;
	bad.FifoBlkSize = 3;
	Check(!PdmInit(&dev, &bad));
	bad = cfg;
	bad.FifoMemSize = sizeof(CFifo_t);
	Check(!PdmInit(&dev, &bad));
	bad = cfg;
	bad.pFifoMem++;
	Check(!PdmInit(&dev, &bad));
	bad = cfg;
	bad.OpMode = PDM_OPMODE_STEREO;
	bad.FifoBlkSize = 2;
	Check(!PdmInit(&dev, &bad));
	bad = cfg;
	bad.GainLeft = 1;
	Check(!PdmInit(&dev, &bad));
	memcpy(pins, s_Pins, sizeof(pins));
	bad = cfg;
	bad.pPins = pins;
	pins[0].PinOp = IOPINOP_FUNC13;
	Check(!PdmInit(&dev, &bad));
	pins[0] = s_Pins[0];
	pins[1].PinNo = 3;
	Check(!PdmInit(&dev, &bad));
	RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
	Check(!PdmInit(&dev, &cfg) && !s_DmaUsed);
	RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
	s_Allocate = false;
	Check(!PdmInit(&dev, &cfg) && !s_DmaUsed);
	s_Allocate = true;
	s_Io[0] = 1; // HSI ready timeout
	Check(!PdmInit(&dev, &cfg) && !s_DmaUsed);
	s_Io[0] = 0;
	RCC->CCIPR2 = RCC_CCIPR2_SAI1SEL_0;
	Check(PdmInit(&dev, &cfg) && s_DmaUsed && s_Configured == 2);
	Check(dev.CfgData.Freq == 1000000 && dev.NbSamples == 32);
	Check((RCC->CCIPR2 & RCC_CCIPR2_SAI1SEL) == RCC_CCIPR2_SAI1SEL_2);
	Check(!PdmInit(&other, &cfg) && !PdmInit(&dev, &cfg));
	Check(!PdmEnable(&other) && !PdmStart(&other) && !PdmGetSamples(&other));
	Check(!PdmGetSamples(&dev));
	Check(PdmStart(&dev) && PdmStart(&dev));
	Check(SAI1->PDMCR == (SAI_PDMCR_CKEN1 | SAI_PDMCR_PDMEN));
	Capture(0xFFFF, true);
	Check(CFifoUsed(dev.hFifo) == 0);	   // DMA IRQ did not filter or publish PCM.
	Check(PdmGetSamples(&dev) == nullptr); // 31 frames after invalid prefix.
	Capture(0xFFFF);
	pcm = (int16_t *)PdmGetSamples(&dev);
	Check(pcm && pcm[0] == 5720 && pcm[31] == INT16_MAX);
	Check(CFifoUsed(dev.hFifo) == 0);
	PdmStop(&dev);
	Check(!s_DmaRunning && !(SAI1_Block_A->CR1 & SAI_xCR1_SAIEN));
	Check(!PdmGetSamples(&dev) && CFifoUsed(dev.hFifo) == 0);
	// Late interrupts while stopped must clear their source without
	// publishing another raw block or repeatedly pending the vector.
	s_DmaFlags = WBA_GPDMA_FLAG_TC;
	eventCount = s_Events[DEVINTRF_EVT_RX_DATA];
	s_DmaHandler(3, &dev);
	Check(!s_DmaFlags && s_Events[DEVINTRF_EVT_RX_DATA] == eventCount);
	s_Io[0] = 2;
	SAI1_IRQHandler();
	Check(!(SAI1_Block_A->SR & SAI_xSR_OVRUDR) && SAI1_Block_A->IMR == 0);
	s_Io[0] = 0;
	PdmSetMode(&dev, PDM_OPMODE_MONO_RIGHT);
	Check(dev.CfgData.OpMode == PDM_OPMODE_MONO_RIGHT && PdmStart(&dev));
	pcm = ReadBlock(&dev, 0xFF00, true);
	Check(pcm[31] == INT16_MIN);
	PdmSetMode(&dev, PDM_OPMODE_STEREO);
	Check(PdmStart(&dev));
	pcm = ReadBlock(&dev, 0xFF00, true);
	Check(pcm[30] == INT16_MAX && pcm[31] == INT16_MIN);
	PdmDisable(&dev);
	Check(!PdmStart(&dev) && PdmEnable(&dev) && PdmStart(&dev));
	// Stop on a DMA error, even with transfer-complete also set.
	s_DmaFlags = WBA_GPDMA_FLAG_TC | WBA_GPDMA_FLAG_ERR;
	eventCount = s_Events[DEVINTRF_EVT_STATECHG];
	s_DmaHandler(3, &dev);
	Check(s_Events[DEVINTRF_EVT_STATECHG] == eventCount + 1 && !s_DmaRunning);
	Check(PdmGetSamples(&dev) == nullptr && PdmStart(&dev));
	// A slow consumer exhausts the raw ring; no active buffer is overwritten.
	eventCount = s_Events[DEVINTRF_EVT_RX_FIFO_FULL];
	for (unsigned i = 0; i < 4; i++)
	{
		Capture(0xFFFF);
	}
	Check(!s_DmaRunning && s_Events[DEVINTRF_EVT_RX_FIFO_FULL] == eventCount + 1);
	Check(PdmGetSamples(&dev) == nullptr && PdmStart(&dev));
	// SAI overrun must discard the partial PCM and require a fresh start.
	s_Io[0] = 2;
	eventCount = s_Events[DEVINTRF_EVT_STATECHG];
	SAI1_IRQHandler();
	Check(!s_DmaRunning && s_Events[DEVINTRF_EVT_STATECHG] == eventCount + 1);
	Check(PdmGetSamples(&dev) == nullptr);
	s_Io[0] = 0;
	Check(PdmStart(&dev));
	// A stuck SAI disable is bounded and reset releases the peripheral.
	s_Io[0] = 3;
	PdmStop(&dev);
	Check(!(SAI1_Block_A->CR1 & SAI_xCR1_SAIEN));
	s_Io[0] = 0;
	Check(PdmStart(&dev));
	PdmPowerOff(&dev);
	Check(!s_DmaUsed && !PdmStart(&dev) && !PdmEnable(&dev));
	Check(!(RCC->APB2ENR & RCC_APB2ENR_SAI1EN));
	Check((RCC->CCIPR2 & RCC_CCIPR2_SAI1SEL) == RCC_CCIPR2_SAI1SEL_0);
	// Polling uses the same DMA path but sends no RX callback.
	cfg.bIntEn = false;
	cfg.SmplMode = PDM_SMPLMODE_RISING;
	cfg.OpMode = PDM_OPMODE_MONO_LEFT;
	cfg.Freq = 0;
	Check(PdmInit(&dev, &cfg) && dev.CfgData.Freq == 1000000 && PdmStart(&dev));
	eventCount = s_Events[DEVINTRF_EVT_RX_DATA];
	pcm = ReadBlock(&dev, 0xFF00, true);
	Check(pcm[31] == INT16_MIN && s_Events[DEVINTRF_EVT_RX_DATA] == eventCount);
	PdmPowerOff(&dev);
	// A tiny PCM FIFO must not lose samples just because a raw DMA block
	// contains more PCM samples than the caller's complete FIFO capacity.
	cfg.FifoBlkSize = 2;
	cfg.FifoMemSize = CFIFO_TOTAL_MEMSIZE(1, 2);
	cfg.SmplMode = PDM_SMPLMODE_FALLING;
	Check(PdmInit(&dev, &cfg) && PdmStart(&dev));
	PdmSetMode(&dev, PDM_OPMODE_STEREO);
	Check(dev.CfgData.OpMode == PDM_OPMODE_MONO_LEFT && s_DmaRunning);
	Capture(0xFFFF, true);
	sampleCount = 0;
	while ((pcm = (int16_t *)PdmGetSamples(&dev)) != nullptr)
	{
		sampleCount++;
	}
	Check(sampleCount == 31 && dev.hFifo->DropCnt == 0);
	Capture(0xFFFF);
	sampleCount = 0;
	while ((pcm = (int16_t *)PdmGetSamples(&dev)) != nullptr)
	{
		Check(pcm[0] == INT16_MAX);
		sampleCount++;
	}
	Check(sampleCount == 32 && dev.hFifo->DropCnt == 0);
	PdmPowerOff(&dev);
	cfg.FifoBlkSize = 64;
	cfg.FifoMemSize = sizeof(s_FifoMem);
	// Out-of-range frequency requests choose the nearest divider.
	cfg.Freq = UINT32_MAX;
	Check(PdmInit(&dev, &cfg) && dev.CfgData.Freq == 8000000);
	PdmPowerOff(&dev);
	cfg.Freq = 1;
	Check(PdmInit(&dev, &cfg) && dev.CfgData.Freq == 16000000U / 126);
	PdmPowerOff(&dev);
	// CK2 is also a valid microphone clock; D1 remains the captured pair.
	memcpy(pins, s_Pins, sizeof(pins));
	pins[0].PinNo = 6;
	cfg.pPins = pins;
	Check(PdmInit(&dev, &cfg) && PdmStart(&dev));
	Check(SAI1->PDMCR == (SAI_PDMCR_CKEN2 | SAI_PDMCR_PDMEN));
	PdmPowerOff(&dev);
	s_Io[2] = 1;
	while (true)
	{
	}
}

extern "C" void ResetHandler()
{
	extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
	uint32_t *source = &_sidata;

	for (uint32_t *dest = &_sdata; dest < &_edata;)
	{
		*dest++ = *source++;
	}
	for (uint32_t *dest = &_sbss; dest < &_ebss;)
	{
		*dest++ = 0;
	}
	PdmChecks();
}

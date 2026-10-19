/**-------------------------------------------------------------------------
@file	pdm_stm32wba.cpp

@brief	STM32WBA SAI1 PDM capture


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

#include <limits.h>
#include <string.h>
#include "stm32wbaxx.h"
#include "coredev/pdm.h"
#include "audio/pdm_filter.h"
#include "gpdma_stm32wba.h"
#include "iopinctrl.h"

#define PDM_RAW_BLOCK_SIZE 512
#define PDM_RAW_BLOCK_COUNT 4
#define PDM_STOP_POLLS 10000
#define PDM_SAI_ERRORS (SAI_xSR_OVRUDR | SAI_xSR_WCKCFG)

typedef struct __Pdm_Clock_Pin
{
	int Port;
	int Pin;
	uint32_t Clock;
} PdmClockPin_t;

static const PdmClockPin_t s_ClockPins[] = {
	{ IOPORTA, 1, SAI_PDMCR_CKEN1 },
	{ IOPORTA, 9, SAI_PDMCR_CKEN1 },
	{ IOPORTE, 2, SAI_PDMCR_CKEN1 },
	{ IOPORTG, 7, SAI_PDMCR_CKEN1 },
	{ IOPORTA, 6, SAI_PDMCR_CKEN2 },
	{ IOPORTE, 5, SAI_PDMCR_CKEN2 }
};
static const int s_DataPins[][2] = {
	{ IOPORTA, 2 },
	{ IOPORTA, 10 },
	{ IOPORTC, 3 },
	{ IOPORTD, 5 },
	{ IOPORTE, 6 }
};

// One SAI1 owner; the PDM interface uses block A and its shared clock.
// Lifecycle calls and GetSamples have one application-context caller.
// The callback signals raw input availability; it should defer work.
alignas(4) static uint8_t s_RawMem[CFIFO_TOTAL_MEMSIZE(PDM_RAW_BLOCK_COUNT, PDM_RAW_BLOCK_SIZE)];
static hCFifo_t s_RawFifo;
static PdmDev_t *s_pDev;
static int s_Dma = -1;
static bool s_Enabled;
static volatile bool s_Running;
static volatile bool s_Failed;
static volatile bool s_Reading;
static uint32_t s_Divider;
static uint32_t s_ClockEnable;
static uint32_t s_OldClock;
static uint16_t *s_pPcm;
static unsigned s_PcmIndex;
static unsigned s_RawIndex;
static unsigned s_Discard;
static PdmFilter_t s_Filter[2];

static void PdmDmaHandler(int Ch, void *pCtx);

static bool PdmOwned(PdmDev_t *pDev)
{
	return pDev && s_pDev == pDev;
}

static bool PdmModeValid(PDM_OPMODE Mode, uint32_t BlockSize)
{
	return (Mode == PDM_OPMODE_MONO_LEFT || Mode == PDM_OPMODE_MONO_RIGHT ||
			Mode == PDM_OPMODE_STEREO) &&
		   (Mode != PDM_OPMODE_STEREO || BlockSize % 4 == 0);
}

static void PdmNotify(DEVINTRF_EVT Evt)
{
	if (s_pDev->CfgData.EvtHandler)
	{
		s_pDev->CfgData.EvtHandler(s_pDev, Evt);
	}
}

static void PdmHalt()
{
	s_Running = false;
	SAI1_Block_A->IMR = 0;
	SAI1_Block_A->CR1 &= ~SAI_xCR1_DMAEN;
	SAI1_Block_A->CR1 &= ~SAI_xCR1_SAIEN;
	for (unsigned count = 0; count < PDM_STOP_POLLS; count++)
	{
		if (!(SAI1_Block_A->CR1 & SAI_xCR1_SAIEN))
		{
			break;
		}
	}
	WbaGpdmaStop(s_Dma);
	// The peripheral can be reset safely because this port owns SAI1.
	if (SAI1_Block_A->CR1 & SAI_xCR1_SAIEN)
	{
		RCC->APB2RSTR |= RCC_APB2RSTR_SAI1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_SAI1RST;
	}
	SAI1->PDMCR = 0;
	SAI1_Block_A->CR2 |= SAI_xCR2_FFLUSH;
	SAI1_Block_A->CLRFR = SAI_xCLRFR_COVRUDR | SAI_xCLRFR_CWCKCFG;
	NVIC_ClearPendingIRQ(SAI1_IRQn);
}

static void PdmFail(DEVINTRF_EVT Evt)
{
	if (!s_Running)
	{
		return;
	}
	s_Failed = true;
	PdmHalt();
	PdmNotify(Evt);
}

static void PdmService()
{
	uint32_t flags;
	uint8_t *next;

	if (!s_Running)
	{
		return;
	}
	flags = WbaGpdmaFlags(s_Dma);
	if ((flags & WBA_GPDMA_FLAG_ERR) || (SAI1_Block_A->SR & PDM_SAI_ERRORS))
	{
		PdmFail(DEVINTRF_EVT_STATECHG);
		return;
	}
	if (!(flags & WBA_GPDMA_FLAG_TC))
	{
		return;
	}
	WbaGpdmaAck(s_Dma);
	// Publish only a completed DMA block. Never overwrite a raw block
	// while GetSamples is filtering it in application context.
	CFifoPut(s_RawFifo);
	next = CFifoResv(s_RawFifo);
	if (!next)
	{
		PdmFail(DEVINTRF_EVT_RX_FIFO_FULL);
		return;
	}
	WbaGpdmaStart(s_Dma, next, PDM_RAW_BLOCK_SIZE);
	if (s_pDev->CfgData.bIntEn)
	{
		PdmNotify(DEVINTRF_EVT_RX_DATA);
	}
}

static void PdmDmaHandler(int Ch, void *pCtx)
{
	if (s_pDev == pCtx && Ch == s_Dma)
	{
		if (s_Running)
		{
			PdmService();
		}
		else
		{
			// A completion can interrupt Stop after it clears s_Running,
			// before the channel is stopped. Retire it without publishing.
			WbaGpdmaAck(Ch);
		}
	}
}

bool PdmEnable(PdmDev_t *pDev)
{
	if (!PdmOwned(pDev))
	{
		return false;
	}
	s_Enabled = true;
	return true;
}

void PdmStop(PdmDev_t *pDev)
{
	if (!PdmOwned(pDev))
	{
		return;
	}
	PdmHalt();
	CFifoFlush(s_RawFifo);
	CFifoFlush(pDev->hFifo);
	s_pPcm = nullptr;
	s_PcmIndex = 0;
	s_RawIndex = 0;
}

void PdmDisable(PdmDev_t *pDev)
{
	if (PdmOwned(pDev))
	{
		PdmStop(pDev);
		s_Enabled = false;
	}
}

bool PdmStart(PdmDev_t *pDev)
{
	uint8_t *next;

	if (!PdmOwned(pDev) || !s_Enabled)
	{
		return false;
	}
	if (s_Running)
	{
		return true;
	}
	PdmStop(pDev);
	PdmFilterReset(&s_Filter[0]);
	PdmFilterReset(&s_Filter[1]);
	s_Discard = 2; // RM0515: the first two PDM frames are invalid.
	s_Failed = false;
	next = CFifoResv(s_RawFifo);
	if (!next)
	{
		return false;
	}
	// RM0515 tables 422/423: one microphone pair, one 16-bit slot,
	// MSB first, falling-edge internal capture, active-high frame pulse.
	SAI1->GCR = 0;
	SAI1_Block_A->CR1 = SAI_xCR1_MODE_0 | SAI_xCR1_DS_2 |
						SAI_xCR1_NODIV | (s_Divider << SAI_xCR1_MCKDIV_Pos);
	SAI1_Block_A->CR2 = SAI_xCR2_FTH_1;
	SAI1_Block_A->FRCR = 15U | SAI_xFRCR_FSPOL;
	SAI1_Block_A->SLOTR = 1U << SAI_xSLOTR_SLOTEN_Pos;
	SAI1->PDMDLY = 0;
	WbaGpdmaSetup(s_Dma, WBA_GPDMA_REQ_SAI1_A, false, &SAI1_Block_A->DR, 1, pDev->CfgData.bIntEn);
	WbaGpdmaStart(s_Dma, next, PDM_RAW_BLOCK_SIZE);
	SAI1_Block_A->IMR = pDev->CfgData.bIntEn ? SAI_xIMR_OVRUDRIE | SAI_xIMR_WCKCFGIE : 0;
	s_Running = true;
	SAI1->PDMCR = s_ClockEnable;
	SAI1->PDMCR |= SAI_PDMCR_PDMEN;
	SAI1_Block_A->CR1 |= SAI_xCR1_DMAEN;
	SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
	return true;
}

void PdmSetMode(PdmDev_t *pDev, PDM_OPMODE Mode)
{
	if (!PdmOwned(pDev) || !PdmModeValid(Mode, pDev->CfgData.FifoBlkSize))
	{
		return;
	}
	PdmStop(pDev);
	pDev->CfgData.OpMode = Mode;
}

void PdmPowerOff(PdmDev_t *pDev)
{
	if (!PdmOwned(pDev))
	{
		return;
	}
	PdmDisable(pDev);
	NVIC_DisableIRQ(SAI1_IRQn);
	WbaGpdmaFree(s_Dma);
	s_Dma = -1;
	RCC->APB2RSTR |= RCC_APB2RSTR_SAI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SAI1RST;
	RCC->CCIPR2 = (RCC->CCIPR2 & ~RCC_CCIPR2_SAI1SEL) | s_OldClock;
	RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
	s_pDev = nullptr;
}

bool PdmInit(PdmDev_t *const pDev, const PdmCfg_t *const pCfg)
{
	uint32_t requested;
	uint32_t clockEnable = 0;
	bool dataPin = false;
	uint32_t best = 1;
	uint64_t difference = UINT64_MAX;
	uint32_t irq;

	if (!pDev || !pCfg || !pCfg->pPins || pCfg->NbPins != 2 ||
		!pCfg->pFifoMem || (uintptr_t(pCfg->pFifoMem) & 3U) ||
		pCfg->FifoBlkSize < 2 || pCfg->FifoBlkSize > INT_MAX || pCfg->FifoBlkSize % 2 ||
		pCfg->FifoMemSize < sizeof(CFifo_t) ||
		pCfg->FifoMemSize - sizeof(CFifo_t) < pCfg->FifoBlkSize ||
		!PdmModeValid(pCfg->OpMode, pCfg->FifoBlkSize) ||
		(pCfg->SmplMode != PDM_SMPLMODE_FALLING && pCfg->SmplMode != PDM_SMPLMODE_RISING) ||
		pCfg->GainLeft != 0 || pCfg->GainRight != 0 ||
		pCfg->IntPrio < 0 || pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS))
	{
		return false;
	}
	// DS14736 table 24: PDM has AF3 routes distinct from SAI audio AF13.
	for (const auto &pin : s_ClockPins)
	{
		if (pCfg->pPins[0].PortNo == pin.Port && pCfg->pPins[0].PinNo == pin.Pin)
		{
			clockEnable = pin.Clock;
		}
	}
	for (const auto &pin : s_DataPins)
	{
		if (pCfg->pPins[1].PortNo == pin[0] && pCfg->pPins[1].PinNo == pin[1])
		{
			dataPin = true;
		}
	}
	if (!clockEnable || !dataPin ||
		pCfg->pPins[0].PinOp != IOPINOP_FUNC3 || pCfg->pPins[0].PinDir != IOPINDIR_OUTPUT ||
		pCfg->pPins[1].PinOp != IOPINOP_FUNC3 || pCfg->pPins[1].PinDir != IOPINDIR_INPUT)
	{
		return false;
	}
	irq = __get_PRIMASK();
	__disable_irq();
	if (s_pDev || (RCC->APB2ENR & RCC_APB2ENR_SAI1EN))
	{
		__set_PRIMASK(irq);
		return false;
	}
	s_pDev = pDev;
	__set_PRIMASK(irq);
	// Select the closest hardware clock; zero requests the 1 MHz default.
	requested = pCfg->Freq ? pCfg->Freq : 1000000;
	for (uint32_t divider = 1; divider <= 63; divider++)
	{
		uint32_t freq = 16000000U / (divider * 2U);
		uint64_t diff = freq > requested ? uint64_t(freq) - requested : uint64_t(requested) - freq;

		if (diff < difference)
		{
			difference = diff;
			best = divider;
		}
	}
	s_Dma = WbaGpdmaAlloc(PdmDmaHandler, pDev, pCfg->IntPrio, pCfg->bIntEn);
	if (s_Dma < 0)
	{
		s_pDev = nullptr;
		return false;
	}
	RCC->CR |= RCC_CR_HSION;
	for (unsigned count = 0; count < PDM_STOP_POLLS && !(RCC->CR & RCC_CR_HSIRDY); count++)
	{
	}
	if (!(RCC->CR & RCC_CR_HSIRDY))
	{
		WbaGpdmaFree(s_Dma);
		s_Dma = -1;
		s_pDev = nullptr;
		return false;
	}
	pDev->CfgData = *pCfg;
	pDev->CfgData.Freq = 16000000U / (best * 2U);
	pDev->NbSamples = pCfg->FifoBlkSize / 2;
	pDev->hFifo = CFifoInit(pCfg->pFifoMem, pCfg->FifoMemSize, pCfg->FifoBlkSize, false);
	s_RawFifo = CFifoInit(s_RawMem, sizeof(s_RawMem), PDM_RAW_BLOCK_SIZE, true);
	s_Divider = best;
	s_ClockEnable = clockEnable;
	s_OldClock = RCC->CCIPR2 & RCC_CCIPR2_SAI1SEL;
	RCC->CCIPR2 = (RCC->CCIPR2 & ~RCC_CCIPR2_SAI1SEL) | RCC_CCIPR2_SAI1SEL_2;
	RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
	(void)RCC->APB2ENR;
	RCC->APB2RSTR |= RCC_APB2RSTR_SAI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SAI1RST;
	IOPinCfg(pCfg->pPins, pCfg->NbPins);
	NVIC_ClearPendingIRQ(SAI1_IRQn);
	NVIC_SetPriority(SAI1_IRQn, pCfg->IntPrio);
	if (pCfg->bIntEn)
	{
		NVIC_EnableIRQ(SAI1_IRQn);
	}
	s_Enabled = true;
	s_Running = false;
	s_Failed = false;
	s_pPcm = nullptr;
	s_PcmIndex = 0;
	s_RawIndex = 0;
	return true;
}

uint16_t *PdmGetSamples(PdmDev_t *pDev)
{
	uint16_t *raw;
	uint16_t *result = nullptr;
	uint32_t irq;
	unsigned first;

	if (!PdmOwned(pDev))
	{
		return nullptr;
	}
	irq = __get_PRIMASK();
	__disable_irq();
	if (s_Reading)
	{
		__set_PRIMASK(irq);
		return nullptr;
	}
	s_Reading = true;
	__set_PRIMASK(irq);
	if (!pDev->CfgData.bIntEn)
	{
		PdmService();
	}
	if (s_Failed)
	{
		s_Reading = false;
		return nullptr;
	}
	raw = reinterpret_cast<uint16_t *>(CFifoPeek(s_RawFifo));
	first = pDev->CfgData.SmplMode == PDM_SMPLMODE_FALLING ? 8 : 0;
	if (raw)
	{
		while (s_RawIndex < PDM_RAW_BLOCK_SIZE / 2 && !s_Failed)
		{
			int16_t left;
			int16_t right;
			bool ready;
			uint16_t word = raw[s_RawIndex++];

			if (s_Discard)
			{
				s_Discard--;
				continue;
			}
			ready = PdmFilterByte(&s_Filter[0], word >> first, &left);
			PdmFilterByte(&s_Filter[1], word >> (8 - first), &right);
			if (!ready)
			{
				continue;
			}
			if (!s_pPcm)
			{
				s_pPcm = reinterpret_cast<uint16_t *>(CFifoResv(pDev->hFifo));
			}
			if (pDev->CfgData.OpMode != PDM_OPMODE_MONO_RIGHT)
			{
				s_pPcm[s_PcmIndex++] = uint16_t(left);
			}
			if (pDev->CfgData.OpMode != PDM_OPMODE_MONO_LEFT)
			{
				s_pPcm[s_PcmIndex++] = uint16_t(right);
			}
			if (s_PcmIndex == unsigned(pDev->NbSamples))
			{
				CFifoPut(pDev->hFifo);
				s_pPcm = nullptr;
				s_PcmIndex = 0;
				break;
			}
		}
		if (s_RawIndex == PDM_RAW_BLOCK_SIZE / 2)
		{
			CFifoGet(s_RawFifo);
			s_RawIndex = 0;
		}
	}
	if (!s_Failed)
	{
		result = reinterpret_cast<uint16_t *>(CFifoGet(pDev->hFifo));
	}
	s_Reading = false;
	return result;
}

extern "C" void SAI1_IRQHandler()
{
	if (s_pDev && (SAI1_Block_A->SR & PDM_SAI_ERRORS))
	{
		if (s_Running)
		{
			PdmFail(DEVINTRF_EVT_STATECHG);
		}
		else
		{
			SAI1_Block_A->IMR = 0;
			SAI1_Block_A->CLRFR = SAI_xCLRFR_COVRUDR | SAI_xCLRFR_CWCKCFG;
		}
	}
}

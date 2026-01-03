/**-------------------------------------------------------------------------
@file	timer_sam4l_ast.cpp

@brief	SAM4Lxx series AST timer implementation

@author	Hoang Nguyen Hoan
@date	Aug. 24, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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
#ifdef __ICCARM__
#include "intrinsics.h"
#endif

#include "timer_sam4l.h"

extern Sam4l_TimerData_t g_Sam4lTimerData[SAM4L_TIMER_MAXCNT];

/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 */
static bool Sam4lASTEnable(TimerDev_t * const pTimer)
{
	Sam4l_TimerData_t *dev = &g_Sam4lTimerData[pTimer->DevNo];

	if (dev->pTimer->DevNo < 0 || dev->pTimer->DevNo >= SAM4L_AST_TIMER_MAXCNT)
	{
		return false;
	}

	dev->pAstReg->AST_CLOCK |= AST_CLOCK_CEN;
	dev->pAstReg->AST_CR |= AST_CR_EN;

	return true;
}

/**
 * @brief   Turn off timer.
 *
 * This is used to disable timer for power saving. Call Enable() to
 * re-enable timer instead of full initialization sequence
 */
static void Sam4lASTDisable(TimerDev_t * const pTimer)
{
	Sam4l_TimerData_t *dev = &g_Sam4lTimerData[pTimer->DevNo];

	if (dev->pTimer->DevNo < SAM4L_AST_TIMER_MAXCNT)
	{
		dev->pAstReg->AST_CR &= ~AST_CR_EN;
		dev->pAstReg->AST_CLOCK &= ~AST_CLOCK_CEN;
	}
}

/**
 * @brief   Reset timer.
 */
static void Sam4lASTReset(TimerDev_t * const pTimer)
{
}

/**
 * @brief	Set timer main frequency.
 *
 * This function allows dynamically changing the timer frequency.  Timer
 * will be reset and restarted with new frequency
 *
 * @param 	Freq : Frequency in Hz
 *
 * @return  Real frequency
 */

// fPRSC Fcnt = Fprsc / (1 << (PSEL + 1))
// (1 << (PSEL + 1)) = Fprsc / Fcnt
static uint32_t Sam4lASTSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	Sam4l_TimerData_t *dev = &g_Sam4lTimerData[pTimer->DevNo];
	uint32_t div = Freq > 0 ? (dev->BaseFreq + (Freq >> 1)) / Freq : 1;
	uint32_t tmp = 0;

	if (div < 2)
	{
		tmp = 0;
	}
	else if (div < 4)
	{
		tmp = 1;
	}
	else if (div < 8)
	{
		tmp = 2;
	}
	else if (div < 16)
	{
		tmp = 3;
	}
	else if (div < 32)
	{
		tmp = 4;
	}
	else if (div < 64)
	{
		tmp = 5;
	}
	else if (div < 128)
	{
		tmp = 6;
	}
	else
	{
		tmp = 7;
	}

	pTimer->Freq = dev->BaseFreq >> tmp;

	// Pre-calculate periods for faster timer counter to time conversion use later
    pTimer->nsPeriod = (1000000000ULL + ((uint64_t)pTimer->Freq >> 1))/ (uint64_t)pTimer->Freq;     // Period in nsec

	// Disable AST
	while ((SAM4L_AST->AST_SR & AST_SR_BUSY) != 0);
	SAM4L_AST->AST_CR &= ~AST_CR_EN;
	while ((SAM4L_AST->AST_SR & AST_SR_CLKBUSY) != 0);
	SAM4L_AST->AST_CLOCK &= ~AST_CLOCK_CEN;
	while ((SAM4L_AST->AST_SR & AST_SR_CLKBUSY) != 0);

    if (GetLowFreqOscType() != OSC_TYPE_RC)
    {
    	// External Xtal or Xco
    	SAM4L_AST->AST_CLOCK = AST_CLOCK_CSSEL_32KHZCLK;// | AST_CLOCK_CEN;
    	while ((SAM4L_AST->AST_SR & AST_SR_CLKBUSY) != 0);
    }
    else
    {
    	SAM4L_AST->AST_CLOCK = AST_CLOCK_CSSEL_32KHZCLK;// | AST_CLOCK_CEN;
    	while ((SAM4L_AST->AST_SR & AST_SR_CLKBUSY) != 0);
    }

	SAM4L_AST->AST_CLOCK |= AST_CLOCK_CEN;
	while ((SAM4L_AST->AST_SR & AST_SR_CLKBUSY) != 0);

	return pTimer->Freq;
}

static uint64_t Sam4lASTGetTickCount(TimerDev_t * const pTimer)
{
	Sam4l_TimerData_t *dev = &g_Sam4lTimerData[pTimer->DevNo];
    uint32_t cnt = dev->pAstReg->AST_CV;

    return (uint64_t)cnt + pTimer->Rollover;
}

/**
 * @brief	Get maximum available timer trigger event for the timer.
 *
 * @return	count
 */
int Sam4lASTGetMaxTrigger(TimerDev_t * const pTimer)
{
	return SAM4L_AST_TIMER_TRIG_MAXCNT;
}

/**
 * @brief	Enable a specific nanosecond timer trigger event.
 *
 * @param   TrigNo : Trigger number to enable. Index value starting at 0
 * @param   nsPeriod : Trigger period in nsec.
 * @param   Type     : Trigger type single shot or continuous
 * @param	Handler	 : Optional Timer trigger user callback
 * @param   pContext : Optional pointer to user private data to be passed
 *                     to the callback. This could be a class or structure pointer.
 *
 * @return  real period in nsec based on clock calculation
 */
uint64_t Sam4lASTEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
								 TimerTrigEvtHandler_t const Handler, void * const pContext)
{
    if (TrigNo < 0 || TrigNo >= SAM4L_AST_TIMER_TRIG_MAXCNT)
        return 0;

	Sam4l_TimerData_t *dev = &g_Sam4lTimerData[pTimer->DevNo];
    uint64_t cc = (nsPeriod + (pTimer->nsPeriod >> 1ULL)) / pTimer->nsPeriod;

    if (cc <= 2ULL || cc >= 0x10000ULL)
    {
        return 0;
    }

    uint64_t retval = 0;

    dev->Trigger[TrigNo].Type = Type;
    dev->CC[TrigNo] = cc & 0xFFFF;

    dev->Trigger[TrigNo].nsPeriod = pTimer->nsPeriod * (uint64_t)cc;
    dev->Trigger[TrigNo].Handler = Handler;
    dev->Trigger[TrigNo].pContext = pContext;

    uint32_t count = dev->pAstReg->AST_CV;

    if (Type == TIMER_TRIG_TYPE_CONTINUOUS)
    {
    	dev->pAstReg->AST_AR0 = cc;
    }
    else
    {
    	dev->pAstReg->AST_AR0 = cc + count;
    }
#if 0
    // Fpa = Fcs / (2 ^ (insel + 1))
    // Fpa = 1 / nsPeriod = Fcs / (2 ^ (insel + 1))
    // 2 ^ (insel + 1) = Fcs * nsPeriod

    uint64_t d = nsPeriod * (uint64_t)pTimer->Freq / 1000000000ULL;
    uint32_t insel = 32 - __CLZ(d) - 1;

    dev->pAstReg->AST_PIR0 = insel;

    dev->pAstReg->AST_IER = AST_IER_PER0;

  //  pTimer->nsPeriod = (uint64_t)(1 << (insel + 1)) * 1000000000ULL / pTimer->Freq;
    cc = 1 << (insel + 1);
#endif

    dev->pAstReg->AST_IER = AST_IER_ALARM0;

    NVIC_ClearPendingIRQ(AST_ALARM_IRQn);
	//NVIC_SetPriority(AST_ALARM_IRQn, dev->pTimer->IntPrio);
	NVIC_EnableIRQ(AST_ALARM_IRQn);

    return pTimer->nsPeriod * cc; // Return real period in nsec
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void Sam4lASTDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void Sam4lASTDisableExtTrigger(TimerDev_t * const pTimer)
{
	Ast *reg = g_Sam4lTimerData[pTimer->DevNo].pAstReg;

}

/**
 * @brief	Enable external timer trigger event.
 *
 * @param   TrigDevNo : External trigger device number to enable. Index value starting at 0
 * @param   pContext : Optional pointer to user private data to be passed
 *                     to the callback. This could be a class or structure pointer.
 *
 * @return  true - Success
 */
bool Sam4lASTEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	Sam4l_TimerData_t *dev = &g_Sam4lTimerData[pTimer->DevNo];
#if 0
	LPTIM_TypeDef *reg = g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg;
	uint32_t tmp = reg->CFGR & ~(LPTIM_CFGR_TRIGEN_Msk | LPTIM_CFGR_TRIGSEL_Msk);

	switch (Sense)
	{
		case TIMER_EXTTRIG_SENSE_LOW_TRANSITION:
			tmp |= LPTIM_CFGR_TRIGEN_1;
			break;
		case TIMER_EXTTRIG_SENSE_HIGH_TRANSITION:
			tmp |= LPTIM_CFGR_TRIGEN_0;
			break;
		case TIMER_EXTTRIG_SENSE_TOGGLE:
			tmp |= LPTIM_CFGR_TRIGEN_Msk;
			break;
		case TIMER_EXTTRIG_SENSE_DISABLE:
		default:
			break;
	}
	reg->CFGR = tmp | LPTIM_CFGR_TIMOUT | ((TrigDevNo & 7) << LPTIM_CFGR_TRIGSEL_Pos);// | (LPTIM_CFGR_TRIGEN_Msk);
	reg->OR = 0;
	reg->IER |= LPTIM_IER_EXTTRIGIE;
#endif
	return true;
}

int Sam4lASTFindAvailTrigger(TimerDev_t * const pTimer)
{
	return 0;
}


bool Sam4lASTTimerInit(Sam4l_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg)
{
	if (pTimerData == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= SAM4L_AST_TIMER_MAXCNT)
	{
		return false;
	}

	// Enable AST power
	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu) |
						  PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_PPCR - (uint32_t)SAM4L_PM);
	SAM4L_PM->PM_PPCR |= PM_PPCR_ASTRCMASK;

	// Connect clock oscillator
	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
		| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_PBDMASK - (uint32_t)SAM4L_PM);
	SAM4L_PM->PM_PBDMASK |= PM_PBDMASK_AST;

	pTimerData->pTimer->DevNo = pCfg->DevNo;
	pTimerData->pTimer->EvtHandler = pCfg->EvtHandler;
    pTimerData->pTimer->Disable = Sam4lASTDisable;
    pTimerData->pTimer->Enable = Sam4lASTEnable;
    pTimerData->pTimer->Reset = Sam4lASTReset;
    pTimerData->pTimer->GetTickCount = Sam4lASTGetTickCount;
    pTimerData->pTimer->SetFrequency = Sam4lASTSetFrequency;
    pTimerData->pTimer->GetMaxTrigger = Sam4lASTGetMaxTrigger;
    pTimerData->pTimer->FindAvailTrigger = Sam4lASTFindAvailTrigger;
    pTimerData->pTimer->DisableTrigger = Sam4lASTDisableTrigger;
    pTimerData->pTimer->EnableTrigger = Sam4lASTEnableTrigger;
    pTimerData->pTimer->DisableExtTrigger = Sam4lASTDisableExtTrigger;
    pTimerData->pTimer->EnableExtTrigger = Sam4lASTEnableExtTrigger;

    Sam4lASTSetFrequency(pTimerData->pTimer, pCfg->Freq);

    SAM4L_AST->AST_IER = AST_IER_OVF;

	NVIC_ClearPendingIRQ(AST_OVF_IRQn);
	NVIC_SetPriority(AST_OVF_IRQn, pCfg->IntPrio);
	NVIC_EnableIRQ(AST_OVF_IRQn);

	return true;
}

extern "C" {
void AST_ALARM_Handler(void)
{
	NVIC_ClearPendingIRQ(AST_ALARM_IRQn);
}

void AST_PER_Handler(void)
{
	NVIC_ClearPendingIRQ(AST_PER_IRQn);
}

void AST_OVF_Handler(void)
{
	NVIC_ClearPendingIRQ(AST_OVF_IRQn);
}

void AST_READY_Handler(void)
{
	NVIC_ClearPendingIRQ(AST_READY_IRQn);
}

void AST_CLKREADY_Handler(void)
{
	NVIC_ClearPendingIRQ(AST_CLKREADY_IRQn);
}

} // __cplusplus

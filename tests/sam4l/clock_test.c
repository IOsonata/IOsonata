/**-------------------------------------------------------------------------
@file clock_test.c
@brief SAM4L runtime clock checks using production register definitions.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "sam4lxxx.h"

// Supplied by ARM startup in firmware builds.
uint32_t SystemMicroSecLoopCnt = 1000000UL;
uint32_t g_TestMask;
uint32_t g_TestIpsr;
static jmp_buf s_Halt;
static uint8_t s_ClockRegisters[0x100000];

void ClockTestHalt(void)
{
	longjmp(s_Halt, 1);
}

#include "../../ARM/Microchip/SAM4L/src/system_sam4l.c"

int main(void)
{
	void *regs = mmap((void *)0x40000000, 0x100000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	assert(regs == (void *)0x40000000);
	*(volatile uint32_t *)&SAM4L_FREQM->FREQM_ISR = FREQM_ISR_DONE;
	*(volatile uint32_t *)&SAM4L_FREQM->FREQM_VALUE = 312500U;
	SAM4L_PM->PM_PBDMASK = PM_PBDMASK_RESETVALUE;
	SAM4L_PM->PM_PBCMASK = PM_PBCMASK_RESETVALUE;
	// The DFLL is measured against the running 32 kHz clock
	SAM4L_BPM->BPM_PMCON = BPM_PMCON_CK32S;
	*(volatile uint32_t *)&SAM4L_BSCIF->BSCIF_PCLKSR = BSCIF_PCLKSR_RC32KRDY;
	uint32_t measured = 0;
	uint32_t freqmMode = FREQM_MODE_REFNUM(64U) | FREQM_MODE_CLKSEL(2U);
	uint32_t pbcMask = SAM4L_PM->PM_PBCMASK;
	SAM4L_FREQM->FREQM_MODE = freqmMode;
	assert(Sam4lMeasureDfll(&measured));
	assert(measured == 80000000U);
	assert(SAM4L_FREQM->FREQM_MODE == freqmMode);
	assert(SAM4L_PM->PM_PBCMASK == pbcMask);
	*(volatile uint32_t *)&SAM4L_FREQM->FREQM_ISR = 0;
	assert(!Sam4lMeasureDfll(&measured));
	assert(s_StartupError == SAM4L_STARTUP_FREQM);
	assert(SAM4L_FREQM->FREQM_MODE == freqmMode);
	assert(SAM4L_PM->PM_PBCMASK == pbcMask);
	*(volatile uint32_t *)&SAM4L_FREQM->FREQM_ISR = FREQM_ISR_DONE;
	*(volatile uint32_t *)&SAM4L_FREQM->FREQM_VALUE = 335938U;
	// The bus dividers are not changed on request at run time, and there is
	// no run time clock source selection.
	for (int bus = 0; bus < 4; bus++)
	{
		uint32_t held = SystemPeriphClockGet(bus);
		assert(SystemPeriphClockSet(bus, held / 2U) == held);
		assert(SystemPeriphClockGet(bus) == held);
	}
	assert(SystemPeriphClockSet(-1, 1000000U) == 0U);
	assert(SystemPeriphClockSet(4, 1000000U) == 0U);

	// The shared clock counts refuse nothing: a power mode change is made
	// with a user of the high frequency clock.
	*(volatile uint32_t *)&SAM4L_PM->PM_SR = PM_SR_CKRDY;
	uint32_t performance = SystemCoreClockGet();
	assert(SystemHighFreqClockEnable());
	assert(SystemPowerModeSet(SYSTEM_POWER_MODE_LOW_POWER));
	assert(SystemCoreClockGet() == performance / 2U);
	assert(SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE));
	assert(SystemCoreClockGet() == performance);
	SystemHighFreqClockDisable();

	*(volatile uint32_t *)&SAM4L_BSCIF->BSCIF_PCLKSR = BSCIF_PCLKSR_RC32KRDY;
	SAM4L_PM->PM_PBDMASK = PM_PBDMASK_RESETVALUE;
	SAM4L_PM->PM_PBCMASK = PM_PBCMASK_RESETVALUE;

	// Model a firmware reset with a retained, unowned AST alarm.
	// The core and software Timer ownership have reset but AST WER/CR
	// and its alarm state still refer to the previous image.
	SAM4L_PM->PM_PBDMASK = PM_PBDMASK_RESETVALUE & ~PM_PBDMASK_AST;
	SAM4L_AST->AST_CLOCK = AST_CLOCK_CSSEL_32KHZCLK | AST_CLOCK_CEN;
	SAM4L_AST->AST_CR = AST_CR_EN;
	SAM4L_AST->AST_WER = AST_WER_ALARM0 | AST_WER_OVF;
	*(volatile uint32_t *)&SAM4L_AST->AST_IMR = AST_SR_ALARM0;
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = AST_SR_ALARM0;
	assert(Sam4lResetAstWake());
	assert(SAM4L_PM->PM_PBDMASK == (PM_PBDMASK_RESETVALUE & ~PM_PBDMASK_AST));
	assert(SAM4L_AST->AST_WER == 0U);
	assert(SAM4L_AST->AST_CR == 0U);
	assert((SAM4L_AST->AST_CLOCK & AST_CLOCK_CEN) == 0U);
	assert((SAM4L_AST->AST_CLOCK & AST_CLOCK_CSSEL_Msk) ==
		AST_CLOCK_CSSEL_32KHZCLK);
	assert(SAM4L_AST->AST_IDR == AST_IDR_MASK);
	assert(SAM4L_AST->AST_SCR == AST_SCR_MASK);

	// Without inherited ownership there is no change to AST state.
	SAM4L_AST->AST_WER = 0U;
	SAM4L_AST->AST_CR = 0U;
	*(volatile uint32_t *)&SAM4L_AST->AST_IMR = 0U;
	SAM4L_AST->AST_CLOCK = AST_CLOCK_CSSEL_32KHZCLK;
	assert(Sam4lResetAstWake());
	assert(SAM4L_AST->AST_CLOCK == AST_CLOCK_CSSEL_32KHZCLK);

	// A clock still synchronizing must be reported, not silently reset.
	SAM4L_AST->AST_WER = AST_WER_ALARM0;
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = AST_SR_CLKBUSY;
	assert(!Sam4lResetAstWake());
	assert(SAM4L_AST->AST_WER == AST_WER_ALARM0);
	SAM4L_AST->AST_WER = 0U;
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = 0U;
	SAM4L_PM->PM_PBDMASK = PM_PBDMASK_RESETVALUE;

	// PICOUART can retain wake selection across a firmware reset.
	// Reset only its owned selection and retain unrelated PM/BPM wakes.
	SAM4L_PM->PM_PBDMASK = PM_PBDMASK_RESETVALUE & ~PM_PBDMASK_PICOUART;
	SAM4L_PM->PM_AWEN = (1UL << PM_AWEN_PICOUART) |
		(1UL << PM_AWEN_TWIS0);
	SAM4L_BPM->BPM_BKUPWEN = (1UL << 5) | (1UL << 0);
	SAM4L_PICOUART->PICOUART_CFG = PICOUART_CFG_SOURCE(2);
	assert(Sam4lResetPicoWake());
	assert(SAM4L_PICOUART->PICOUART_CR == PICOUART_CR_DIS);
	assert(SAM4L_PICOUART->PICOUART_CFG == 0U);
	assert(SAM4L_PM->PM_AWEN == (1UL << PM_AWEN_TWIS0));
	assert(SAM4L_BPM->BPM_BKUPWEN == (1UL << 0));
	assert(SAM4L_PM->PM_PBDMASK ==
		(PM_PBDMASK_RESETVALUE & ~PM_PBDMASK_PICOUART));

	// Refuse to reconfigure a PICOUART that has not yet disabled.
	*(volatile uint32_t *)&SAM4L_PICOUART->PICOUART_SR = PICOUART_SR_EN;
	SAM4L_PICOUART->PICOUART_CFG = PICOUART_CFG_SOURCE(2);
	assert(!Sam4lResetPicoWake());
	assert(SAM4L_PICOUART->PICOUART_CFG == PICOUART_CFG_SOURCE(2));
	assert(SAM4L_PM->PM_PBDMASK ==
		(PM_PBDMASK_RESETVALUE & ~PM_PBDMASK_PICOUART));
	*(volatile uint32_t *)&SAM4L_PICOUART->PICOUART_SR = 0U;

	SAM4L_PM->PM_AWEN = 0U;
	SAM4L_BPM->BPM_BKUPWEN = 0U;
	SAM4L_PM->PM_PBDMASK = PM_PBDMASK_RESETVALUE;

	// The optional oscillator hook must be linkable and leave live clocks alone.
	memcpy(s_ClockRegisters, (void *)0x40000000, sizeof(s_ClockRegisters));
	SystemOscInit();
	assert(memcmp(s_ClockRegisters, (void *)0x40000000, sizeof(s_ClockRegisters)) == 0);

	// Startup still halts on clock failure and disables the default watchdog.
	*(volatile uint32_t *)&SAM4L_PM->PM_SR = 0;
	if (setjmp(s_Halt) == 0)
	{
		SystemInit();
		assert(0);
	}
	assert((SAM4L_WDT->WDT_CTRL & WDT_CTRL_EN) == 0);
	assert(munmap(regs, 0x100000) == 0);
	puts("SAM4L clock queries, power mode, retained wake reset and startup failure handling PASS");
	return 0;
}

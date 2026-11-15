// SAM4L runtime clock failure tests using production register definitions.
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "sam4lxxx.h"

uint32_t g_TestMask;
uint32_t g_TestIpsr;
static jmp_buf s_Halt;
static uint8_t s_ClockRegisters[0x100000];

void ClockTestHalt(void)
{
	longjmp(s_Halt, 1);
}

#include "../../ARM/Microchip/SAM4L/src/system_sam4l.c"

static void CheckClockBusy(bool Low)
{
	McuOsc_t previous = g_McuOsc;
	uint32_t frequency = SystemCoreClock;

	memcpy(s_ClockRegisters, (void *)0x40000000, sizeof(s_ClockRegisters));
	for (unsigned mask = 0; mask < 2; mask++)
	{
		g_TestMask = mask;
		assert(!(Low ? SystemLowFreqClockSelect(OSC_TYPE_XTAL, 32768) :
			SystemCoreClockSelect(OSC_TYPE_XTAL, 12000000)));
		assert(g_TestMask == mask);
		assert(memcmp(s_ClockRegisters, (void *)0x40000000, sizeof(s_ClockRegisters)) == 0);
		assert(memcmp(&previous, &g_McuOsc, sizeof(previous)) == 0);
		assert(SystemCoreClock == frequency);
	}
}

int main(void)
{
	void *regs = mmap((void *)0x40000000, 0x100000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	assert(regs == (void *)0x40000000);
	SAM4L_PM->PM_PBDMASK = PM_PBDMASK_RESETVALUE;
	SAM4L_PM->PM_PBCMASK = PM_PBCMASK_RESETVALUE;
	assert(setjmp(s_Halt) == 0); // Runtime LF failures must return, not halt.

	for (uint32_t mask = 0; mask < 2; mask++)
	{
		g_TestMask = mask;
		g_McuOsc.LowPwrOsc.Type = OSC_TYPE_RC;
		g_McuOsc.LowPwrOsc.Freq = 32768;
		SAM4L_BPM->BPM_PMCON = BPM_PMCON_CK32S | BPM_PMCON_PS(2);
		*(volatile uint32_t *)&SAM4L_BSCIF->BSCIF_PCLKSR = 0;
		assert(!SystemLowFreqClockSelect(OSC_TYPE_XTAL, 32768));
		assert(g_TestMask == mask);
		assert(g_McuOsc.LowPwrOsc.Type == OSC_TYPE_RC);
		assert(SAM4L_BPM->BPM_PMCON == (BPM_PMCON_CK32S | BPM_PMCON_PS(2)));

		*(volatile uint32_t *)&SAM4L_BSCIF->BSCIF_PCLKSR = BSCIF_PCLKSR_OSC32RDY;
		assert(SystemLowFreqClockSelect(OSC_TYPE_XTAL, 32768));
		assert(g_TestMask == mask);
		assert(g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL);
		assert(SAM4L_BPM->BPM_PMCON == BPM_PMCON_PS(2));
		// A matching descriptor must not conceal a missing clock.
		*(volatile uint32_t *)&SAM4L_BSCIF->BSCIF_PCLKSR = 0;
		assert(!SystemLowFreqClockSelect(OSC_TYPE_XTAL, 32768));
		assert(!SystemLowFreqClockSelect(OSC_TYPE_RC, 32768));
		assert(g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL);
		assert(g_TestMask == mask);
		*(volatile uint32_t *)&SAM4L_BSCIF->BSCIF_PCLKSR = BSCIF_PCLKSR_RC32KRDY;
		assert(SystemLowFreqClockSelect(OSC_TYPE_RC, 32768));
		assert(g_McuOsc.LowPwrOsc.Type == OSC_TYPE_RC);
		assert(g_TestMask == mask);
	}
	g_TestIpsr = 1;
	assert(!SystemLowFreqClockSelect(OSC_TYPE_XTAL, 32768));
	g_TestIpsr = 0;
	assert(!SystemLowFreqClockSelect(OSC_TYPE_XTAL, 12345));

	// Low-clock refusal must preserve all registers, flags and descriptors.
	SAM4L_AST->AST_CR = AST_CR_EN;
	CheckClockBusy(true);
	SAM4L_AST->AST_CR = 0;
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = AST_SR_BUSY;
	CheckClockBusy(true);
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = AST_SR_CLKBUSY;
	CheckClockBusy(true);
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = AST_SR_ALARM0;
	assert(!Sam4lLowClockBusy());
	*(volatile uint32_t *)&SAM4L_PICOUART->PICOUART_SR = PICOUART_SR_EN;
	CheckClockBusy(true);
	*(volatile uint32_t *)&SAM4L_PICOUART->PICOUART_SR = 0;
	*(volatile uint32_t *)&SAM4L_FREQM->FREQM_STATUS = FREQM_STATUS_BUSY;
	CheckClockBusy(true);
	*(volatile uint32_t *)&SAM4L_FREQM->FREQM_STATUS = FREQM_STATUS_RCLKBUSY;
	CheckClockBusy(true);
	*(volatile uint32_t *)&SAM4L_FREQM->FREQM_STATUS = 0;
	SAM4L_PM->PM_PBAMASK = PM_PBAMASK_LCDCA;
	CheckClockBusy(true);
	SAM4L_PM->PM_PBAMASK = PM_PBAMASK_CATB;
	CheckClockBusy(true);
	SAM4L_PM->PM_PBAMASK = 0;
	SAM4L_WDT->WDT_CTRL = WDT_CTRL_EN | WDT_CTRL_CSSEL;
	CheckClockBusy(true);
	SAM4L_WDT->WDT_CTRL = WDT_CTRL_CEN | WDT_CTRL_CSSEL1;
	CheckClockBusy(true);
	SAM4L_WDT->WDT_CTRL = WDT_CTRL_EN | WDT_CTRL_CEN;
	assert(!Sam4lLowClockBusy()); // RCSYS watchdog is unaffected.
	SAM4L_BSCIF->BSCIF_BOD33CTRL = BSCIF_BOD33CTRL_EN | BSCIF_BOD33CTRL_MODE;
	CheckClockBusy(true);
	SAM4L_BSCIF->BSCIF_BOD33CTRL = BSCIF_BOD33CTRL_EN;
	SAM4L_BSCIF->BSCIF_BOD18CTRL = BSCIF_BOD18CTRL_EN | BSCIF_BOD18CTRL_MODE;
	CheckClockBusy(true);
	SAM4L_BSCIF->BSCIF_BOD18CTRL = BSCIF_BOD18CTRL_EN;
	assert(!Sam4lLowClockBusy()); // Continuous brownout detection is unaffected.
	for (unsigned i = 0; i < 12; i++)
	{
		uint32_t sources[] = {OSC32K_GEN_CLK_SRC, RC32K_GEN_CLK_SRC, OSC1K_GEN_CLK_SRC};
		for (unsigned n = 0; n < 3; n++)
		{
			SAM4L_SCIF->SCIF_GCCTRL[i].SCIF_GCCTRL = SCIF_GCCTRL_CEN | SCIF_GCCTRL_OSCSEL(sources[n]);
			CheckClockBusy(true);
		}
		SAM4L_SCIF->SCIF_GCCTRL[i].SCIF_GCCTRL = SCIF_GCCTRL_CEN | SCIF_GCCTRL_OSCSEL(OSC0_GEN_CLK_SRC);
		assert(!Sam4lLowClockBusy());
		SAM4L_SCIF->SCIF_GCCTRL[i].SCIF_GCCTRL = 0;
	}
	for (unsigned i = 0; i < 6; i++)
	{
		if (i == 4) continue; // EIC is not needed for these reads.
		SAM4L_PM->PM_PBDMASK = PM_PBDMASK_RESETVALUE & ~(1U << i);
		CheckClockBusy(true);
	}
	SAM4L_PM->PM_PBDMASK = PM_PBDMASK_RESETVALUE;
	SAM4L_PM->PM_PBCMASK &= ~PM_PBCMASK_FREQM;
	CheckClockBusy(true);
	SAM4L_PM->PM_PBCMASK = PM_PBCMASK_RESETVALUE & ~PM_PBCMASK_SCIF;
	CheckClockBusy(true);
	SAM4L_PM->PM_PBCMASK = PM_PBCMASK_RESETVALUE;
	*(volatile uint32_t *)&SAM4L_BSCIF->BSCIF_PCLKSR = BSCIF_PCLKSR_OSC32RDY | BSCIF_PCLKSR_RC32KRDY;
	assert(SystemLowFreqClockSelect(OSC_TYPE_XTAL, 32768));
	assert(SAM4L_WDT->WDT_CTRL == (WDT_CTRL_EN | WDT_CTRL_CEN));
	assert(SAM4L_BSCIF->BSCIF_BOD33CTRL == BSCIF_BOD33CTRL_EN);
	assert(SAM4L_BSCIF->BSCIF_BOD18CTRL == BSCIF_BOD18CTRL_EN);
	assert(SAM4L_AST->AST_SR == AST_SR_ALARM0);
	assert(SystemLowFreqClockSelect(OSC_TYPE_RC, 32768));
	*(volatile uint32_t *)&SAM4L_BSCIF->BSCIF_PCLKSR = BSCIF_PCLKSR_RC32KRDY;
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = 0;
	// Runtime failures return with the original interrupt mask and WDT state.
	for (uint32_t mask = 0; mask < 2; mask++)
	{
		for (unsigned enabled = 0; enabled < 2; enabled++)
		{
			g_TestMask = mask;
			SAM4L_WDT->WDT_CTRL = enabled ? WDT_CTRL_EN : 0;
			*(volatile uint32_t *)&SAM4L_PM->PM_SR = 0;
			g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
			g_McuOsc.CoreOsc.Freq = 12000000;
			assert(!SystemCoreClockSelect(OSC_TYPE_RC, 8000000));
			assert(g_TestMask == mask);
			assert(SAM4L_WDT->WDT_CTRL == (enabled ? WDT_CTRL_EN : 0));
			assert(s_StartupError == SAM4L_STARTUP_CLOCK_SWITCH);
			assert(g_McuOsc.CoreOsc.Freq == 12000000);
			assert(SystemCoreClockGet() == RCSYS_FREQ);
		}
	}

	*(volatile uint32_t *)&SAM4L_PM->PM_SR = PM_SR_CKRDY;
	SAM4L_BPM->BPM_PMCON = BPM_PMCON_PS(2) | BPM_PMCON_CK32S;
	*(volatile uint32_t *)&SAM4L_BPM->BPM_SR = BPM_SR_PSOK;
	*(volatile uint32_t *)&SAM4L_HFLASHC->FLASHCALW_FSR =
		FLASHCALW_FSR_FRDY | FLASHCALW_FSR_HSMODE;
	for (uint32_t mask = 0; mask < 2; mask++)
	{
		g_TestMask = mask;
		// Missing external oscillator, then missing PLL lock.
		*(volatile uint32_t *)&SAM4L_SCIF->SCIF_PCLKSR = 0;
		assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 12000000));
		assert(s_StartupError == SAM4L_STARTUP_OSC0);
		assert(g_TestMask == mask);
		assert(SystemCoreClockGet() == RCSYS_FREQ);
		*(volatile uint32_t *)&SAM4L_SCIF->SCIF_PCLKSR = SCIF_PCLKSR_OSC0RDY;
		assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 12000000));
		assert(s_StartupError == SAM4L_STARTUP_PLL);
		assert(g_TestMask == mask);
		assert(SystemCoreClockGet() == RCSYS_FREQ);
		// Retry succeeds; a repeated request must not hide a later lock failure.
		*(volatile uint32_t *)&SAM4L_SCIF->SCIF_PCLKSR =
			SCIF_PCLKSR_OSC0RDY | SCIF_PCLKSR_PLL0LOCK;
		assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 12000000));
		assert(SystemCoreClockGet() == 48000000);
		assert(g_McuOsc.CoreOsc.Type == OSC_TYPE_XTAL);
		*(volatile uint32_t *)&SAM4L_SCIF->SCIF_PCLKSR = SCIF_PCLKSR_OSC0RDY;
		assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 12000000));
		assert(SystemCoreClockGet() == RCSYS_FREQ);
		// Missing DFLL synchronization and lock must also return.
		assert(!SystemCoreClockSelect(OSC_TYPE_RC, 8000000));
		assert(s_StartupError == SAM4L_STARTUP_DFLL);
		*(volatile uint32_t *)&SAM4L_SCIF->SCIF_PCLKSR = SCIF_PCLKSR_DFLL0RDY;
		assert(!SystemCoreClockSelect(OSC_TYPE_RC, 8000000));
		assert(s_StartupError == SAM4L_STARTUP_DFLL);
		*(volatile uint32_t *)&SAM4L_SCIF->SCIF_PCLKSR = SCIF_PCLKSR_DFLL0RDY |
			SCIF_PCLKSR_DFLL0LOCKC | SCIF_PCLKSR_DFLL0LOCKF;
		assert(SystemCoreClockSelect(OSC_TYPE_RC, 8000000));
		assert(SystemCoreClockGet() == 40000000);
		assert(g_TestMask == mask);
		assert(SAM4L_WDT->WDT_CTRL == WDT_CTRL_EN);
		assert(SAM4L_BPM->BPM_PMCON == (BPM_PMCON_PS(2) | BPM_PMCON_CK32S));
		// Runtime does not initialize the low clock or cache.
		assert(SAM4L_HCACHE->HCACHE_CTRL == 0);
		assert(SAM4L_BSCIF->BSCIF_PCLKSR == BSCIF_PCLKSR_RC32KRDY);
		*(volatile uint32_t *)&SAM4L_HFLASHC->FLASHCALW_FSR = 0;
		assert(!SystemCoreClockSelect(OSC_TYPE_RC, 80000000));
		assert(s_StartupError == SAM4L_STARTUP_FLASH);
		assert(SystemCoreClockGet() == RCSYS_FREQ);
		assert(g_TestMask == mask);
		*(volatile uint32_t *)&SAM4L_HFLASHC->FLASHCALW_FSR =
			FLASHCALW_FSR_FRDY | FLASHCALW_FSR_HSMODE;
		assert(SystemCoreClockSelect(OSC_TYPE_RC, 80000000));
		assert(SystemCoreClockGet() == 20000000);
	}

	// Reject enabled transports before any clock or descriptor write.
	uint32_t transports[] = {
		PM_PBAMASK_SPI, PM_PBAMASK_TWIM0, PM_PBAMASK_TWIM1,
		PM_PBAMASK_TWIM2, PM_PBAMASK_TWIM3, PM_PBAMASK_TWIS0,
		PM_PBAMASK_TWIS1, PM_PBAMASK_USART0, PM_PBAMASK_USART1,
		PM_PBAMASK_USART2, PM_PBAMASK_USART3, PM_PBAMASK_ADCIFE,
		PM_PBAMASK_DACC, PM_PBAMASK_ACIFC, PM_PBAMASK_IISC,
		PM_PBAMASK_ABDACB, PM_PBAMASK_GLOC, PM_PBAMASK_TRNG,
		PM_PBAMASK_PARC, PM_PBAMASK_CATB, PM_PBAMASK_LCDCA,
		PM_PBAMASK_TC0, PM_PBAMASK_TC1
	};
	for (unsigned i = 0; i < sizeof(transports) / sizeof(transports[0]); i++)
	{
		SAM4L_PM->PM_PBAMASK = transports[i];
		CheckClockBusy(false);
	}
	SAM4L_PM->PM_PBAMASK = 0;
	SAM4L_PM->PM_PBDMASK = PM_PBDMASK_AST;
	SAM4L_AST->AST_CR = AST_CR_EN;
	CheckClockBusy(false);
	SAM4L_AST->AST_CR = 0;
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = AST_SR_BUSY;
	CheckClockBusy(false);
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = AST_SR_CLKBUSY;
	CheckClockBusy(false);
	*(volatile uint32_t *)&SAM4L_AST->AST_SR = 0;
	assert(!Sam4lCoreClockBusy());
	SAM4L_PM->PM_PBBMASK = PM_PBBMASK_USBC;
	SAM4L_USBC->USBC_USBCON = USBC_USBCON_USBE;
	CheckClockBusy(false);
	SAM4L_USBC->USBC_USBCON |= USBC_USBCON_FRZCLK;
	CheckClockBusy(false);
	SAM4L_USBC->USBC_USBCON = USBC_USBCON_FRZCLK;
	assert(!Sam4lCoreClockBusy());
	SAM4L_PM->PM_PBBMASK = PM_PBBMASK_PDCA;
	for (unsigned i = 0; i < 16U; i++)
	{
		// An enabled channel may be waiting for a request with zero count.
		*(volatile uint32_t *)&SAM4L_PDCA->PDCA_CHANNEL[i].PDCA_SR = PDCA_SR_TEN;
		CheckClockBusy(false);
		*(volatile uint32_t *)&SAM4L_PDCA->PDCA_CHANNEL[i].PDCA_SR = 0;
	}
	assert(!Sam4lCoreClockBusy());
	SAM4L_PM->PM_PBBMASK = 0;
	SAM4L_PM->PM_HSBMASK = PM_HSBMASK_PDCA;
	CheckClockBusy(false);
	SAM4L_PM->PM_HSBMASK = PM_HSBMASK_USBC;
	CheckClockBusy(false);
	SAM4L_PM->PM_HSBMASK = 0;
	for (unsigned i = 1; i < 12U; i++)
	{
		SAM4L_SCIF->SCIF_GCCTRL[i].SCIF_GCCTRL = SCIF_GCCTRL_CEN;
		CheckClockBusy(false);
		SAM4L_SCIF->SCIF_GCCTRL[i].SCIF_GCCTRL = 0;
	}
	g_McuOsc.bUSBClk = true;
	SAM4L_SCIF->SCIF_GCCTRL[7].SCIF_GCCTRL = SCIF_GCCTRL_CEN;
	assert(!Sam4lCoreClockBusy());
	SAM4L_PM->PM_PBBMASK = PM_PBBMASK_USBC;
	SAM4L_USBC->USBC_USBCON = USBC_USBCON_USBE;
	CheckClockBusy(false);
	SAM4L_USBC->USBC_USBCON = 0;
	SAM4L_PM->PM_PBBMASK = 0;
	SAM4L_SCIF->SCIF_GCCTRL[7].SCIF_GCCTRL = 0;
	g_McuOsc.bUSBClk = false;
	// Releasing every checked user permits the next clock change.
	assert(SystemCoreClockSelect(OSC_TYPE_RC, 80000000));

	// Startup still halts on clock failure and disables the default watchdog.
	*(volatile uint32_t *)&SAM4L_PM->PM_SR = 0;
	if (setjmp(s_Halt) == 0)
	{
		SystemInit();
		assert(0);
	}
	assert((SAM4L_WDT->WDT_CTRL & WDT_CTRL_EN) == 0);
	assert(munmap(regs, 0x100000) == 0);
	puts("SAM4L runtime clock success, busy rejection, failure recovery, PRIMASK and watchdog preservation PASS");
	return 0;
}

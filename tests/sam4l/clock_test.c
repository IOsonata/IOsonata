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

	// Force a later clock-ready failure to inspect WDT handling before it.
	for (volatile unsigned enabled = 0; enabled < 2; enabled++)
	{
		SAM4L_WDT->WDT_CTRL = enabled ? WDT_CTRL_EN : 0;
		*(volatile uint32_t *)&SAM4L_PM->PM_SR = 0;
		g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
		g_McuOsc.CoreOsc.Freq = 12000000;
		if (setjmp(s_Halt) == 0)
		{
			(void)SystemCoreClockSelect(OSC_TYPE_RC, 8000000);
			assert(0);
		}
		assert(SAM4L_WDT->WDT_CTRL == (enabled ? WDT_CTRL_EN : 0));
		assert(s_StartupError == SAM4L_STARTUP_CLOCK_SWITCH);
		if (setjmp(s_Halt) == 0)
		{
			SystemInit();
			assert(0);
		}
		assert((SAM4L_WDT->WDT_CTRL & WDT_CTRL_EN) == 0);
	}
	assert(munmap(regs, 0x100000) == 0);
	puts("SAM4L LF success/timeouts, PRIMASK and runtime watchdog preservation PASS");
	return 0;
}

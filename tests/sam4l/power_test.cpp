/**-------------------------------------------------------------------------
@file power_test.cpp
@brief SAM4L power register tests using the production device header.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <cassert>
#include <csetjmp>
#include <cstdio>
#include "sam4lxxx.h"
static Pm s_Pm;
static Bpm s_Bpm;
static Bscif s_Bscif;
static Eic s_Eic;
static std::jmp_buf s_Off;
PowerTestScb g_PowerTestScb;
uint32_t g_PowerTestMask;
uint32_t g_PowerTestIrqEnabled;
uint32_t g_PowerTestIrqPending;
static bool s_TestSleep;
static unsigned s_SleepCount;
static uint32_t s_ExpectedEicWakeClear;
static uint32_t s_SavedCpuDivider;
static uint32_t s_SavedPbaDivider;
static uint32_t s_SavedPbbDivider;
static uint32_t s_SavedPbcDivider;
static uint32_t s_SavedPbdDivider;

void PowerTestWfi(void)
{
	if (!s_TestSleep)
	{
		std::longjmp(s_Off, 1);
	}
	assert(g_PowerTestMask == 1);
	assert(s_Pm.PM_CPUSEL == s_SavedCpuDivider);
	assert(s_Pm.PM_PBASEL == s_SavedPbaDivider);
	assert(s_Pm.PM_PBBSEL == s_SavedPbbDivider);
	assert(s_Pm.PM_PBCSEL == s_SavedPbcDivider);
	assert(s_Pm.PM_PBDSEL == s_SavedPbdDivider);
	assert(!(g_PowerTestScb.SCR & SCB_SCR_SLEEPONEXIT_Msk));
	assert(g_PowerTestScb.SCR & SCB_SCR_SLEEPDEEP_Msk);
	assert(!(s_Bpm.BPM_PMCON & (BPM_PMCON_BKUP | BPM_PMCON_SLEEP_Msk)));
	assert(s_Bpm.BPM_PMCON & BPM_PMCON_RET);
	if (s_ExpectedEicWakeClear != 0U)
	{
		// A stale level-triggered EIC event is cleared before WFI.
		assert(s_Eic.EIC_ICR == s_ExpectedEicWakeClear);
	}
	assert(g_PowerTestIrqEnabled == 1);
	assert(s_Pm.PM_IER == PM_IER_WAKE);
	g_PowerTestIrqPending = 1;
	s_SleepCount++;
}
#undef SAM4L_PM
#undef SAM4L_BPM
#undef SAM4L_BSCIF
#undef SAM4L_EIC
#define SAM4L_PM (&s_Pm)
#define SAM4L_BPM (&s_Bpm)
#define SAM4L_BSCIF (&s_Bscif)
#define SAM4L_EIC (&s_Eic)
#include "../../ARM/Microchip/SAM4L/src/system_power_sam4l.cpp"

// The host register model retains the normal CPU rate across sleep.
uint32_t SystemCoreClockGet(void) { return 12000000U; }

int main(void)
{
	unsigned i;
	uint32_t bit;
	uint32_t cause[] = {PM_RCAUSE_POR, PM_RCAUSE_POR33, PM_RCAUSE_BOD,
			PM_RCAUSE_BOD33, PM_RCAUSE_EXT, PM_RCAUSE_WDT, PM_RCAUSE_OCDRST};
	uint32_t expected[] = {SYSTEM_RESET_POWERON, SYSTEM_RESET_POWERON,
			SYSTEM_RESET_POWERON, SYSTEM_RESET_POWERON, SYSTEM_RESET_PIN,
			SYSTEM_RESET_WDT, SYSTEM_RESET_SOFT};

	for (i = 0; i < sizeof(cause) / sizeof(cause[0]); i++)
	{
		s_Pm.PM_RCAUSE = cause[i];
		s_Bpm.BPM_BKUPWCAUSE = 0x1f;
		assert(SystemResetReason() == expected[i]);
		assert(SystemResetReason() == expected[i]);
	}
	s_Pm.PM_RCAUSE = PM_RCAUSE_BKUP;
	assert(SystemResetReason() == (SYSTEM_RESET_POWERON | SYSTEM_RESET_WAKE_PIN |
			SYSTEM_RESET_WAKE_TIMER | SYSTEM_RESET_WDT));
	s_Bpm.BPM_BKUPWCAUSE = 1 << 5; // No generic PICOUART reset flag.
	assert(SystemResetReason() == 0);
	assert(SystemRetRegCount() == 4);
	for (i = 0; i < 4; i++)
	{
		assert(SystemRetRegWrite(i, 0x12345678 + i));
		assert(SystemRetRegRead(i) == 0x12345678 + i);
		assert(s_Bscif.BSCIF_UNLOCK == (BSCIF_UNLOCK_KEY(0xaa) |
				BSCIF_UNLOCK_ADDR(0x78 + 4 * i)));
	}
	assert(!SystemRetRegWrite(-1, 1));
	assert(!SystemRetRegWrite(4, 1));
	assert(SystemRetRegRead(-1) == 0);
	assert(SystemRetRegRead(4) == 0);
	assert(!SystemRamRetain(HRAMC0_ADDR, HRAMC0_SIZE, true));
	assert(!SystemRamRetain(HRAMC1_ADDR, HRAMC1_SIZE, false));
	assert(!SystemWakePin(0, 16, IOPINSENSE_LOW_TRANSITION));
	assert(!SystemWakePin(-1, 6, IOPINSENSE_LOW_TRANSITION));
	assert(!SystemWakePin(0, 6, IOPINSENSE_TOGGLE));
	s_Eic.EIC_CTRL = 1 << 1;
	assert(!SystemWakePin(0, 6, IOPINSENSE_LOW_TRANSITION));
	assert(s_Eic.EIC_DIS == 0);
	// Start with retained hardware and freshly initialized driver RAM, as
	// after Backup wake. An enabled EIC line alone was refused above.
	s_Bpm.BPM_BKUPPMUX = 1U << 1;
	s_Bpm.BPM_BKUPWEN = SAM4L_BACKUP_WAKE_EIC;
	s_Eic.EIC_MODE = 1U << 1;
	s_Eic.EIC_ASYNC = 1U << 1;
	for (i = 0; i < 3U; i++)
	{
		assert(SystemWakePin(IOPORTA, 6, IOPINSENSE_LOW_TRANSITION));
		assert(s_Eic.EIC_EN == (1U << 1));
		assert(s_Bpm.BPM_BKUPPMUX & (1U << 1));
	}
	assert(s_Eic.EIC_IER == (1U << 1));
	assert(SystemWakePin(IOPORTA, 6, IOPINSENSE_DISABLE));
	assert(s_Eic.EIC_IDR == (1U << 1));
	assert(!(s_Bpm.BPM_BKUPPMUX & (1U << 1)));
	s_Eic.EIC_CTRL = 0;
	s_Bpm.BPM_BKUPWEN = SAM4L_BACKUP_WAKE_AST;
	s_Eic.EIC_MODE = 1UL << 31;
	s_Eic.EIC_LEVEL = 1UL << 31;
	for (i = 0; i < sizeof(s_Sam4lBackupPins) / sizeof(s_Sam4lBackupPins[0]); i++)
	{
		bit = s_Sam4lBackupPins[i].Mask;
		assert(SystemWakePin(s_Sam4lBackupPins[i].Port, s_Sam4lBackupPins[i].Pin,
				IOPINSENSE_HIGH_TRANSITION));
		assert(s_Eic.EIC_EN == bit);
		assert(s_Eic.EIC_IER == bit);
		assert(s_Eic.EIC_IDR == bit);
		assert(s_Eic.EIC_MODE & bit);
		assert(s_Eic.EIC_ASYNC & bit);
		assert(s_Eic.EIC_LEVEL & bit);
		assert(s_Bpm.BPM_BKUPPMUX & bit);
		assert(SystemWakePin(s_Sam4lBackupPins[i].Port, s_Sam4lBackupPins[i].Pin,
				IOPINSENSE_LOW_TRANSITION));
		assert(!(s_Eic.EIC_LEVEL & bit));
		assert(SystemWakePin(s_Sam4lBackupPins[i].Port, s_Sam4lBackupPins[i].Pin,
				IOPINSENSE_DISABLE));
		assert(s_Eic.EIC_DIS == bit);
		assert(!(s_Bpm.BPM_BKUPPMUX & bit));
		assert(s_Eic.EIC_LEVEL & (1UL << 31));
		assert(s_Eic.EIC_MODE & (1UL << 31));
		assert(s_Bpm.BPM_BKUPWEN & SAM4L_BACKUP_WAKE_AST);
		assert(g_PowerTestMask == 0);
	}
#if defined(SAM4LSXA) || defined(SAM4LCXA)
	assert(!SystemWakePin(1, 1, IOPINSENSE_LOW_TRANSITION));
#endif
#if !defined(SAM4LSXC) && !defined(SAM4LCXC)
	assert(!SystemWakePin(2, 3, IOPINSENSE_LOW_TRANSITION));
#endif
	s_SavedCpuDivider = PM_CPUSEL_CPUDIV | PM_CPUSEL_CPUSEL(0);
	s_SavedPbaDivider = PM_PBASEL_PBDIV | PM_PBASEL_PBSEL(0);
	s_SavedPbbDivider = 0U;
	s_SavedPbcDivider = 0U;
	s_SavedPbdDivider = 0U;
	s_Pm.PM_CPUSEL = s_SavedCpuDivider;
	s_Pm.PM_PBASEL = s_SavedPbaDivider;
	s_Pm.PM_PBBSEL = s_SavedPbbDivider;
	s_Pm.PM_PBCSEL = s_SavedPbcDivider;
	s_Pm.PM_PBDSEL = s_SavedPbdDivider;
	s_TestSleep = true;
	s_Pm.PM_PBDMASK = 0;
	s_Bpm.BPM_PMCON = BPM_PMCON_BKUP | BPM_PMCON_SLEEP(3) | BPM_PMCON_CK32S;
	uint32_t saved = s_Bpm.BPM_PMCON;
	g_PowerTestScb.SCR = SCB_SCR_SLEEPONEXIT_Msk | 0x10;
	s_Bscif.BSCIF_PCLKSR = BSCIF_PCLKSR_VREGOK;

	// RETENTION whatever the devices are doing: no device state is read
	for (unsigned mask = 0; mask < 2; mask++)
	{
		g_PowerTestMask = mask;
		s_Pm.PM_AWEN = mask ? (1U << PM_AWEN_TWIS0) : 0U;
		s_Pm.PM_PBAMASK = mask ? 0xFFFFFFFFU : 0U;
		SystemSleep();
		assert(g_PowerTestMask == mask);
		assert(s_Pm.PM_PBDMASK == 0);
		assert(s_Bpm.BPM_PMCON == saved);
		assert(g_PowerTestScb.SCR == (SCB_SCR_SLEEPONEXIT_Msk | 0x10));
		assert(g_PowerTestIrqEnabled == 0);
		assert(g_PowerTestIrqPending == 0);
		assert(s_Pm.PM_IDR == PM_IDR_WAKE);
		assert(s_Pm.PM_ICR == PM_ICR_WAKE);
	}
	assert(s_SleepCount == 2);
	s_Pm.PM_AWEN = 0U;
	s_Pm.PM_PBAMASK = 0U;

	// EIC8 simulates the latched wake button interrupt after a RETENTION
	// wake. EIC9 is not a SystemWakePin line and must not be acknowledged.
	s_Eic.EIC_CTRL = (1U << 8) | (1U << 9);
	s_Eic.EIC_IMR = (1U << 8) | (1U << 9);
	s_Eic.EIC_ASYNC = (1U << 8) | (1U << 9);
	s_Eic.EIC_MODE = 1U << 8;
	s_Bpm.BPM_BKUPPMUX = 1U << 8;
	s_Eic.EIC_ISR = (1U << 8) | (1U << 9);
	s_Eic.EIC_ICR = 0U;
	s_ExpectedEicWakeClear = 1U << 8;
	SystemSleep();
	assert(s_SleepCount == 3);
	assert(s_Eic.EIC_ICR == (1U << 8));
	s_ExpectedEicWakeClear = 0U;
	s_Eic.EIC_ISR = 1U << 9;
	s_Eic.EIC_ICR = 0U;
	SystemSleep();
	assert(s_SleepCount == 4);
	assert(s_Eic.EIC_ICR == 0U);
	s_Eic.EIC_CTRL = 0U;
	s_Bpm.BPM_BKUPPMUX = 0U;

	// The power mode is not changed while the regulator is not ready
	s_Bscif.BSCIF_PCLKSR = 0;
	SystemSleep();
	assert(s_SleepCount == 4);
	assert(s_Pm.PM_PBDMASK == 0);
	assert(s_Bpm.BPM_PMCON == saved);

	s_TestSleep = false;
	s_Bpm.BPM_PMCON = BPM_PMCON_RET | BPM_PMCON_CK32S | BPM_PMCON_PS(2);
	if (setjmp(s_Off) == 0)
	{
		SystemPowerOff();
		assert(false);
	}
	assert(s_Bpm.BPM_PMCON == (BPM_PMCON_BKUP | BPM_PMCON_CK32S | BPM_PMCON_PS(2)));
	assert(s_Bpm.BPM_UNLOCK == (BPM_UNLOCK_KEY(0xaa) | BPM_UNLOCK_ADDR(0x1c)));
	assert(g_PowerTestScb.SCR & SCB_SCR_SLEEPDEEP_Msk);
	assert(g_PowerTestMask == 1);
	std::puts("SAM4L reset, backup registers, EIC wake acknowledgement, PDCA/timer sleep refusal, SLEEP0, WAIT, RETENTION and Backup entry PASS");
}

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
static Ast s_Ast;
static Pdca s_Pdca;
static std::jmp_buf s_Off;
PowerTestScb g_PowerTestScb;
uint32_t g_PowerTestMask;
uint32_t g_PowerTestIrqEnabled;
uint32_t g_PowerTestIrqPending;
static bool s_DeepSleep;
static bool s_WaitSleep;
static bool s_TestSleep;
static unsigned s_SleepCount;
static uint32_t s_TimerBusyMask;

// Optional driver functions must be provided by the host test. Linking
// undefined weak C++ references is not portable to macOS clang.
struct SleepHookTest {
	unsigned RejectStage;
	unsigned UartPrepare;
	unsigned UsbCheck;
	unsigned I2CPrepare;
	unsigned SpiPrepare;
	unsigned UartSuspend;
	unsigned I2CResume;
	unsigned SpiResume;
	unsigned UartResume;
};
static SleepHookTest s_Hooks;

void PowerTestWfi(void)
{
	if (!s_TestSleep)
	{
		std::longjmp(s_Off, 1);
	}
	assert(g_PowerTestMask == 1);
	assert(!(g_PowerTestScb.SCR & SCB_SCR_SLEEPONEXIT_Msk));
	assert(bool(g_PowerTestScb.SCR & SCB_SCR_SLEEPDEEP_Msk) == s_DeepSleep);
	assert(!(s_Bpm.BPM_PMCON & (BPM_PMCON_BKUP | BPM_PMCON_SLEEP_Msk)));
	assert(bool(s_Bpm.BPM_PMCON & BPM_PMCON_RET) == (s_DeepSleep && !s_WaitSleep));
	if (s_DeepSleep)
	{
		assert(g_PowerTestIrqEnabled == 1);
		assert(s_Pm.PM_IER == PM_IER_WAKE);
		g_PowerTestIrqPending = 1;
	}
	s_SleepCount++;
}
#undef SAM4L_PM
#undef SAM4L_BPM
#undef SAM4L_BSCIF
#undef SAM4L_AST
#define SAM4L_AST (&s_Ast)
#undef SAM4L_EIC
#undef SAM4L_PDCA
#define SAM4L_PDCA (&s_Pdca)
#define SAM4L_PM (&s_Pm)
#define SAM4L_BPM (&s_Bpm)
#define SAM4L_BSCIF (&s_Bscif)
#define SAM4L_EIC (&s_Eic)
#include "../../ARM/Microchip/SAM4L/src/system_power_sam4l.cpp"

// The host register model retains the normal CPU rate across sleep.
uint32_t SystemCoreClockGet(void) { return 12000000U; }

// Host implementations of the optional driver sleep functions. These model
// successful preparation, refusal and rollback without linking MCU drivers.
bool Sam4lUartPrepareSleep(void)
{
	s_Hooks.UartPrepare++;
	return s_Hooks.RejectStage != 1U;
}

bool Sam4lUsbAllowDeepSleep(void)
{
	s_Hooks.UsbCheck++;
	return s_Hooks.RejectStage != 2U;
}

bool Sam4lI2CSuspendSleep(void)
{
	s_Hooks.I2CPrepare++;
	return s_Hooks.RejectStage != 3U;
}

bool Sam4lSpiSuspendSleep(void)
{
	s_Hooks.SpiPrepare++;
	return s_Hooks.RejectStage != 4U;
}

bool Sam4lUartSuspendSleep(void)
{
	s_Hooks.UartSuspend++;
	return s_Hooks.RejectStage != 5U;
}

void Sam4lUartResumeSleep(void)
{
	s_Hooks.UartResume++;
}

void Sam4lSpiResumeSleep(void)
{
	s_Hooks.SpiResume++;
}

void Sam4lI2CResumeSleep(void)
{
	s_Hooks.I2CResume++;
}

bool Sam4lTimerClockBusy(uint32_t ClockMask)
{
	return (ClockMask & s_TimerBusyMask) != 0U;
}

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
	s_TestSleep = true;
	s_Pm.PM_PBDMASK = 0;
	s_Bpm.BPM_PMCON = BPM_PMCON_BKUP | BPM_PMCON_RET | BPM_PMCON_SLEEP(3) | BPM_PMCON_CK32S;
	uint32_t saved = s_Bpm.BPM_PMCON;
	g_PowerTestScb.SCR = SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk | 0x10;
	s_Bscif.BSCIF_PCLKSR = BSCIF_PCLKSR_VREGOK;
	for (unsigned mask = 0; mask < 2; mask++)
	{
		g_PowerTestMask = mask;
		SystemSleep();
		assert(g_PowerTestMask == mask);
		assert(s_Pm.PM_PBDMASK == 0);
		assert(s_Bpm.BPM_PMCON == saved);
		assert(g_PowerTestScb.SCR == (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk | 0x10));
	}
	assert(s_SleepCount == 2);
	// Repeat retained sleep with EIC, AST, and an AWEN SleepWalking source.
	s_DeepSleep = true;
	for (unsigned source = 0; source < 3; source++)
	{
		s_Eic.EIC_CTRL = source == 0 ? 2U : 0U;
		s_Eic.EIC_IMR = 2U;
		s_Eic.EIC_ASYNC = 2U;
		s_Pm.PM_PBDMASK = source == 1 ? PM_PBDMASK_AST : 0U;
		s_Ast.AST_CR = AST_CR_EN;
		s_Ast.AST_WER = AST_WER_ALARM0;
		s_Pm.PM_AWEN = source == 2 ? (1U << PM_AWEN_TWIS0) : 0U;
		s_WaitSleep = source == 2;
		for (unsigned cycle = 0; cycle < 3; cycle++)
		{
			SystemSleep();
			assert(g_PowerTestIrqEnabled == 0);
			assert(g_PowerTestIrqPending == 0);
			assert(s_Pm.PM_IDR == PM_IDR_WAKE);
			assert(s_Pm.PM_ICR == PM_ICR_WAKE);
			assert(s_Bpm.BPM_PMCON == saved);
		}
	}
	assert(s_Hooks.UartPrepare == 11U);
	assert(s_Hooks.UsbCheck == 9U);
	assert(s_Hooks.I2CPrepare == 9U);
	assert(s_Hooks.SpiPrepare == 9U);
	assert(s_Hooks.UartSuspend == 9U);
	assert(s_Hooks.UartResume == 11U);
	assert(s_Hooks.SpiResume == 11U);
	assert(s_Hooks.I2CResume == 11U);

	// Refusal by each driver must prevent WFI and preserve system clocks.
	// The system rolls back every optional driver on a rejected deep sleep.
	s_Eic.EIC_CTRL = 2U;
	s_Eic.EIC_IMR = 2U;
	s_Eic.EIC_ASYNC = 2U;
	s_Pm.PM_AWEN = 0U;
	s_Pm.PM_PBDMASK = 0U;
	s_WaitSleep = false;
	for (unsigned stage = 1U; stage <= 5U; stage++)
	{
		s_Hooks = {};
		s_Hooks.RejectStage = stage;
		g_PowerTestMask = 0U;
		unsigned before = s_SleepCount;
		SystemSleep();
		assert(s_SleepCount == before);
		assert(g_PowerTestMask == 0U);
		assert(s_Pm.PM_PBDMASK == 0U);
		assert(s_Bpm.BPM_PMCON == saved);
		assert(g_PowerTestScb.SCR ==
			(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk | 0x10U));
		assert(s_Hooks.UartPrepare == 1U);
		assert(s_Hooks.UsbCheck == (stage >= 2U ? 1U : 0U));
		assert(s_Hooks.I2CPrepare == (stage >= 3U ? 1U : 0U));
		assert(s_Hooks.SpiPrepare == (stage >= 4U ? 1U : 0U));
		assert(s_Hooks.UartSuspend == (stage >= 5U ? 1U : 0U));
		assert(s_Hooks.UartResume == (stage > 1U ? 1U : 0U));
		assert(s_Hooks.SpiResume == (stage > 1U ? 1U : 0U));
		assert(s_Hooks.I2CResume == (stage > 1U ? 1U : 0U));
	}
	s_Hooks = {};

	// Shared DMA and running high-frequency timers must prevent RETENTION.
	// AST is allowed to keep running for the existing wake tests above.
	s_Pm.PM_AWEN = 0U;
	s_Eic.EIC_CTRL = 2U;
	s_Pm.PM_PBDMASK = 0U;
	s_WaitSleep = false;
	s_Pm.PM_HSBMASK = PM_HSBMASK_PDCA;
	s_Pm.PM_PBBMASK = 0U;
	unsigned before = s_SleepCount;
	SystemSleep();
	assert(s_SleepCount == before);
	assert(s_Pm.PM_PBDMASK == 0U);

	s_Pm.PM_HSBMASK = 0U;
	s_Pm.PM_PBBMASK = PM_PBBMASK_PDCA;
	SystemSleep();
	assert(s_SleepCount == before);

	s_Pm.PM_HSBMASK = PM_HSBMASK_PDCA;
	s_Pdca.PDCA_CHANNEL[5].PDCA_SR = PDCA_SR_TEN;
	SystemSleep();
	assert(s_SleepCount == before);
	assert(s_Pdca.PDCA_CHANNEL[5].PDCA_SR == PDCA_SR_TEN);
	s_Pdca.PDCA_CHANNEL[5].PDCA_SR = 0U;
	SystemSleep();
	assert(s_SleepCount == before + 1U);
	s_Pm.PM_HSBMASK = 0U;
	s_Pm.PM_PBBMASK = 0U;

	s_Pm.PM_PBAMASK = PM_PBAMASK_TC0;
	s_TimerBusyMask = PM_PBAMASK_TC0;
	before = s_SleepCount;
	SystemSleep();
	assert(s_SleepCount == before);
	s_TimerBusyMask = 0U;
	SystemSleep();
	assert(s_SleepCount == before + 1U);
	s_Pm.PM_PBAMASK = 0U;
	s_Pm.PM_PBDMASK = 0;
	s_Pm.PM_AWEN = 0;
	s_Eic.EIC_CTRL = 0;
	s_DeepSleep = false;

	s_Bscif.BSCIF_PCLKSR = 0;
	SystemSleep();
	assert(s_SleepCount == 13);
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
	std::puts("SAM4L reset, backup registers, wake pins, PDCA/timer sleep refusal, SLEEP0, WAIT, RETENTION and Backup entry PASS");
}

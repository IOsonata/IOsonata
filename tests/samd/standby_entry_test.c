// Register model for the production SAMD Standby entry and NMI handlers.
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define EIC_IRQn 4
#define IOPORTA 0
#define EIC_NMIFLAG_NMI_Msk 1U
#define EIC_NMICTRL_NMISENSE_Msk 7U
#define EIC_NMICTRL_NMISENSE_FALL 2U
#define EIC_NMICTRL_NMISENSE_RISE 1U
#define SCB_SCR_SLEEPDEEP_Msk 4U

static struct { uint32_t EIC_NMIFLAG, EIC_INTFLAG, EIC_INTENSET, EIC_NMICTRL; } s_Eic;
static struct { struct { uint32_t PORT_IN; } GROUP[1]; } s_Port;
static struct { uint32_t SCR; } s_Scb;
#define EIC_REGS (&s_Eic)
#define PORT_REGS (&s_Port)
#define SCB (&s_Scb)
static uint32_t s_SamdStandbyWakeLines;
static bool s_bSamdStandbyNmi;
static volatile bool s_bSamdStandbyNmiArmed;
static uint32_t s_SamdStandbyEicEnabled;
static uint32_t s_SamdStandbyEicPending;
static uint32_t s_Mask, s_Enabled, s_Pending, s_Inject, s_WfiCount;
static bool s_RealPin;
static uint32_t s_Isr;
static uint32_t s_RejectStage;
static uint32_t s_AdcPrepareCount;
static uint32_t s_AdcResumeCount;
static uint32_t s_I2cPrepareCount;
static uint32_t s_I2cResumeCount;
void NMI_Handler(void);

static uint32_t __get_IPSR(void) { return s_Isr; }
static uint32_t DisableInterrupt(void)
{
	uint32_t previous = s_Mask;
	s_Mask = 1;
	return previous;
}
static void EnableInterrupt(uint32_t State) { s_Mask = State; }
static uint32_t NVIC_GetEnableIRQ(int Irq) { (void)Irq; return s_Enabled; }
static uint32_t NVIC_GetPendingIRQ(int Irq) { (void)Irq; return s_Pending; }
static void NVIC_SetPendingIRQ(int Irq) { (void)Irq; s_Pending = 1; }
static void NVIC_ClearPendingIRQ(int Irq) { (void)Irq; s_Pending = 0; }
static void NVIC_DisableIRQ(int Irq) { (void)Irq; s_Enabled = 0; }
static void NVIC_EnableIRQ(int Irq)
{
	(void)Irq;
	s_Enabled = 1;
	if (s_Inject == 1) NMI_Handler();
}
static void __DSB(void)
{
	if (s_Inject == 2) NMI_Handler();
}
static void __ISB(void) {}
static bool SamdUsbAllowStandbyMock(void) { return s_RejectStage != 1U; }
static bool SamdDacAllowStandbyMock(void) { return s_RejectStage != 2U; }
static bool SamdAdcPrepareSleepMock(void)
{
	s_AdcPrepareCount++;
	return s_RejectStage != 3U;
}
static void SamdAdcResumeSleepMock(void) { s_AdcResumeCount++; }
static bool SamdI2cPrepareSleepMock(void)
{
	s_I2cPrepareCount++;
	return s_RejectStage != 4U;
}
static void SamdI2cResumeSleepMock(void) { s_I2cResumeCount++; }
static bool SamdSpiPrepareSleepMock(void) { return s_RejectStage != 5U; }
static void SamdSpiResumeSleepMock(void) {}
static bool SamdUartPrepareSleepMock(void) { return s_RejectStage != 6U; }
static void SamdUartResumeSleepMock(void) {}
static bool (*SamdUsbAllowStandby)(void) = SamdUsbAllowStandbyMock;
static bool (*SamdDacAllowStandby)(void) = SamdDacAllowStandbyMock;
static bool (*SamdAdcPrepareSleep)(void) = SamdAdcPrepareSleepMock;
static void (*SamdAdcResumeSleep)(void) = SamdAdcResumeSleepMock;
static bool (*SamdI2cPrepareSleep)(void) = SamdI2cPrepareSleepMock;
static void (*SamdI2cResumeSleep)(void) = SamdI2cResumeSleepMock;
static bool (*SamdSpiPrepareSleep)(void) = SamdSpiPrepareSleepMock;
static void (*SamdSpiResumeSleep)(void) = SamdSpiResumeSleepMock;
static bool (*SamdUartPrepareSleep)(void) = SamdUartPrepareSleepMock;
static void (*SamdUartResumeSleep)(void) = SamdUartResumeSleepMock;
static uint32_t SystemCoreClockGet(void) { return 48000000U; }
static void __WFI(void)
{
	assert(s_Mask == 1);
	assert((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0);
	if (s_Inject == 3) NMI_Handler(); // NMI returns just before WFI.
	if (s_RealPin)
	{
		s_Eic.EIC_INTFLAG = 2;
		s_Eic.EIC_INTENSET = 2;
	}
	assert(s_Enabled && s_Pending); // Otherwise the SW0 wake was lost.
	s_WfiCount++;
}

#include "standby_functions.h"

int main(void)
{
	for (unsigned mask = 0; mask < 2; mask++)
	for (unsigned enabled = 0; enabled < 2; enabled++)
	for (unsigned pending = 0; pending < 2; pending++)
	for (unsigned injection = 0; injection < 4; injection++)
	for (unsigned real = 0; real < 2; real++)
	for (unsigned high = 0; high < 2; high++)
	{
		memset(&s_Eic, 0, sizeof(s_Eic));
		s_Mask = mask;
		s_Enabled = enabled;
		s_Pending = pending;
		s_Inject = injection;
		s_RealPin = real;
		s_bSamdStandbyNmi = true;
		s_bSamdStandbyNmiArmed = false;
		s_Eic.EIC_NMICTRL = high ? EIC_NMICTRL_NMISENSE_RISE : EIC_NMICTRL_NMISENSE_FALL;
		// Without injected NMI, the button is already held on entry.
		s_Port.GROUP[0].PORT_IN = (high != (injection != 0)) ? (1UL << 14) : 0;
		s_Scb.SCR = 0x10;
		SystemSleep();
		assert(s_Mask == mask);
		assert(s_Enabled == enabled);
		assert(s_Pending == (pending || real));
		assert(s_Scb.SCR == 0x10);
		assert(!s_bSamdStandbyNmiArmed);
	}
	// Refused Standby must restore the earlier drivers and not execute WFI.
	for (unsigned stage = 1U; stage <= 6U; stage++)
	{
		s_RejectStage = stage;
		s_Mask = 0U;
		s_Isr = 0U;
		s_Scb.SCR = 0x10U;
		s_AdcPrepareCount = 0U;
		s_AdcResumeCount = 0U;
		s_I2cPrepareCount = 0U;
		s_I2cResumeCount = 0U;
		SystemSleep();
		assert(s_Mask == 0U);
		assert(s_Scb.SCR == 0x10U);
		assert(s_WfiCount == 128U);
		assert(s_AdcPrepareCount == (stage >= 3U ? 1U : 0U));
		assert(s_AdcResumeCount == 1U);
		assert(s_I2cPrepareCount == (stage >= 4U ? 1U : 0U));
		assert(s_I2cResumeCount == 1U);
	}
	// Never enter Standby while running inside a peripheral interrupt.
	s_RejectStage = 0U;
	s_Isr = 1U;
	s_AdcPrepareCount = 0U;
	SystemSleep();
	assert(s_AdcPrepareCount == 0U);
	assert(s_WfiCount == 128U);
	s_Isr = 0U;
	s_Pending = 0;
	NMI_Handler();
	assert(s_Pending == 0); // No synthetic IRQ outside the entry window.
	assert(s_WfiCount == 128);
	puts("SAMD11 NMI entry timing, refusal, and IRQ state: 128 wake cases PASS");
	return 0;
}

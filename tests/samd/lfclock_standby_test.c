// Host model for the production SAMD 32 kHz Standby clock request.
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SAMD_GCLK_LF 1U
#define GCLK_GENCTRL_GENEN_Msk (1UL << 16)
#define GCLK_GENCTRL_SRC_Msk (0x1FUL << 8)
#define GCLK_GENCTRL_SRC_OSC32K (4UL << 8)
#define GCLK_GENCTRL_SRC_XOSC32K (5UL << 8)

#define SYSCTRL_OSC32K_ENABLE_Msk (1UL << 1)
#define SYSCTRL_OSC32K_EN32K_Msk (1UL << 2)
#define SYSCTRL_OSC32K_RUNSTDBY_Msk (1UL << 6)
#define SYSCTRL_XOSC32K_ENABLE_Msk (1UL << 1)
#define SYSCTRL_XOSC32K_XTALEN_Msk (1UL << 2)
#define SYSCTRL_XOSC32K_EN32K_Msk (1UL << 3)
#define SYSCTRL_XOSC32K_RUNSTDBY_Msk (1UL << 6)
#define SYSCTRL_PCLKSR_OSC32KRDY_Msk (1UL << 7)
#define SYSCTRL_PCLKSR_XOSC32KRDY_Msk (1UL << 6)

static struct {
	volatile uint32_t GCLK_GENCTRL;
} s_Gclk;

static struct {
	volatile uint32_t SYSCTRL_OSC32K;
	volatile uint16_t SYSCTRL_XOSC32K;
	volatile uint32_t SYSCTRL_PCLKSR;
} s_Sysctrl;

#define GCLK_REGS (&s_Gclk)
#define SYSCTRL_REGS (&s_Sysctrl)

static uint32_t s_Generator;
static uint32_t s_Mask;
static unsigned s_ClockReads;

static uint32_t SamdGclkGenRead(volatile uint32_t *pReg, uint32_t Gen)
{
	assert(pReg == &GCLK_REGS->GCLK_GENCTRL);
	assert(Gen == SAMD_GCLK_LF);
	assert(s_Mask == 1U);
	s_ClockReads++;
	return s_Generator;
}

static uint32_t DisableInterrupt(void)
{
	uint32_t previous = s_Mask;

	s_Mask = 1U;
	return previous;
}

static void EnableInterrupt(uint32_t Previous)
{
	s_Mask = Previous;
}

#include "lfclock_function.h"

static void ResetModel(uint32_t Source, uint32_t Mask)
{
	s_Generator = GCLK_GENCTRL_GENEN_Msk | Source;
	s_Gclk.GCLK_GENCTRL = 0U;
	s_Sysctrl.SYSCTRL_OSC32K = 0U;
	s_Sysctrl.SYSCTRL_XOSC32K = 0U;
	s_Sysctrl.SYSCTRL_PCLKSR = 0U;
	s_Mask = Mask;
	s_ClockReads = 0U;
}

int main(void)
{
	for (uint32_t mask = 0U; mask < 2U; mask++)
	{
		ResetModel(GCLK_GENCTRL_SRC_OSC32K, mask);
		s_Sysctrl.SYSCTRL_OSC32K = 0x340000U |
			SYSCTRL_OSC32K_ENABLE_Msk | SYSCTRL_OSC32K_EN32K_Msk;
		s_Sysctrl.SYSCTRL_PCLKSR = SYSCTRL_PCLKSR_OSC32KRDY_Msk;
		assert(SamdLowFreqClockStandbyEnable());
		assert(s_Sysctrl.SYSCTRL_OSC32K == (0x340000U |
			SYSCTRL_OSC32K_ENABLE_Msk | SYSCTRL_OSC32K_EN32K_Msk |
			SYSCTRL_OSC32K_RUNSTDBY_Msk));
		assert(s_Sysctrl.SYSCTRL_XOSC32K == 0U);
		assert(s_Mask == mask && s_ClockReads == 1U);

		ResetModel(GCLK_GENCTRL_SRC_XOSC32K, mask);
		s_Sysctrl.SYSCTRL_XOSC32K = 0x500U |
			SYSCTRL_XOSC32K_ENABLE_Msk | SYSCTRL_XOSC32K_XTALEN_Msk |
			SYSCTRL_XOSC32K_EN32K_Msk;
		s_Sysctrl.SYSCTRL_PCLKSR = SYSCTRL_PCLKSR_XOSC32KRDY_Msk;
		assert(SamdLowFreqClockStandbyEnable());
		assert(s_Sysctrl.SYSCTRL_XOSC32K == (0x500U |
			SYSCTRL_XOSC32K_ENABLE_Msk | SYSCTRL_XOSC32K_XTALEN_Msk |
			SYSCTRL_XOSC32K_EN32K_Msk | SYSCTRL_XOSC32K_RUNSTDBY_Msk));
		assert(s_Sysctrl.SYSCTRL_OSC32K == 0U);
		assert(s_Mask == mask && s_ClockReads == 1U);
	}

	ResetModel(GCLK_GENCTRL_SRC_OSC32K, 0U);
	s_Sysctrl.SYSCTRL_OSC32K = SYSCTRL_OSC32K_ENABLE_Msk |
		SYSCTRL_OSC32K_EN32K_Msk;
	assert(!SamdLowFreqClockStandbyEnable());
	assert((s_Sysctrl.SYSCTRL_OSC32K & SYSCTRL_OSC32K_RUNSTDBY_Msk) == 0U);

	ResetModel(GCLK_GENCTRL_SRC_OSC32K, 0U);
	s_Sysctrl.SYSCTRL_PCLKSR = SYSCTRL_PCLKSR_OSC32KRDY_Msk;
	s_Sysctrl.SYSCTRL_OSC32K = SYSCTRL_OSC32K_ENABLE_Msk;
	assert(!SamdLowFreqClockStandbyEnable());

	ResetModel(GCLK_GENCTRL_SRC_XOSC32K, 0U);
	s_Sysctrl.SYSCTRL_PCLKSR = SYSCTRL_PCLKSR_XOSC32KRDY_Msk;
	s_Sysctrl.SYSCTRL_XOSC32K = SYSCTRL_XOSC32K_EN32K_Msk;
	assert(!SamdLowFreqClockStandbyEnable());

	ResetModel(GCLK_GENCTRL_SRC_XOSC32K, 0U);
	s_Sysctrl.SYSCTRL_XOSC32K = SYSCTRL_XOSC32K_ENABLE_Msk |
		SYSCTRL_XOSC32K_EN32K_Msk;
	assert(!SamdLowFreqClockStandbyEnable());

	ResetModel(GCLK_GENCTRL_SRC_OSC32K, 0U);
	s_Generator &= ~GCLK_GENCTRL_GENEN_Msk;
	assert(!SamdLowFreqClockStandbyEnable());

	ResetModel(GCLK_GENCTRL_SRC_OSC32K, 0U);
	s_Generator = UINT32_MAX;
	assert(!SamdLowFreqClockStandbyEnable());

	ResetModel(6UL << 8, 0U);
	assert(!SamdLowFreqClockStandbyEnable());
	assert(s_Gclk.GCLK_GENCTRL == 0U);
	assert(s_Mask == 0U);

	puts("SAMD 32 kHz Standby clock request: RC, crystal and rejection PASS");
	return 0;
}

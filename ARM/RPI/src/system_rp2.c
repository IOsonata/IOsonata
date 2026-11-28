// Clock tree for the 12 MHz crystal on Pico and Pico 2.
#include "rp2.h"
#include "hardware/regs/clocks.h"
#include "hardware/regs/pll.h"
#include "hardware/regs/xosc.h"
#include "hardware/regs/watchdog.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#ifdef RP2350
#include "hardware/regs/ticks.h"
#include "hardware/regs/powman.h"
#define RP_SYS_HZ 150000000UL
#define RP_PLL_FB 125U
#define RP_PLL_DIV1 5U
#else
#include "hardware/regs/vreg_and_chip_reset.h"
#define RP_SYS_HZ 125000000UL
#define RP_PLL_FB 125U
#define RP_PLL_DIV1 6U
#endif

uint32_t SystemCoreClock;
uint32_t SystemnsDelayFactor;
extern uint32_t SystemMicroSecLoopCnt;
__attribute__((weak)) McuOsc_t g_McuOsc = {
	{OSC_TYPE_XTAL, 12000000U}, {OSC_TYPE_XTAL, 12000000U}
};
static uint32_t s_Users[2];

void SystemCoreClockUpdate(void)
{
	uint32_t div = RP_REG(CLOCKS_BASE, CLOCKS_CLK_SYS_DIV_OFFSET) >> CLOCKS_CLK_SYS_DIV_INT_LSB;
	SystemCoreClock = div ? RP_SYS_HZ / div : 0U;
	// Initial estimates for the shared Arm delay loops; measure on the board.
	SystemMicroSecLoopCnt = SystemCoreClock / 13U;
	SystemnsDelayFactor = SystemCoreClock / 3U;
}

void SystemOscInit(void)
{
	RP_REG(XOSC_BASE, XOSC_STARTUP_OFFSET) = 47U;
	RP_REG(XOSC_BASE, XOSC_CTRL_OFFSET) = XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ |
		(XOSC_CTRL_ENABLE_VALUE_ENABLE << XOSC_CTRL_ENABLE_LSB);
	while (!(RP_REG(XOSC_BASE, XOSC_STATUS_OFFSET) & XOSC_STATUS_STABLE_BITS))
	{
	}
	RP_REG(CLOCKS_BASE, CLOCKS_CLK_REF_CTRL_OFFSET) = CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC;
	while (RP_REG(CLOCKS_BASE, CLOCKS_CLK_REF_SELECTED_OFFSET) != 4U)
	{
	}
	RP_REG(CLOCKS_BASE, CLOCKS_CLK_SYS_CTRL_OFFSET) = 0;
	while (RP_REG(CLOCKS_BASE, CLOCKS_CLK_SYS_SELECTED_OFFSET) != 1U)
	{
	}
	RP_SET(RESETS_BASE, RESETS_RESET_OFFSET) = RESETS_RESET_PLL_SYS_BITS;
	RpReleaseReset(RESETS_RESET_PLL_SYS_BITS);
	RP_REG(PLL_SYS_BASE, PLL_CS_OFFSET) = 1;
	RP_REG(PLL_SYS_BASE, PLL_FBDIV_INT_OFFSET) = RP_PLL_FB;
	RP_CLR(PLL_SYS_BASE, PLL_PWR_OFFSET) = PLL_PWR_VCOPD_BITS | PLL_PWR_PD_BITS;
	while (!(RP_REG(PLL_SYS_BASE, PLL_CS_OFFSET) & PLL_CS_LOCK_BITS))
	{
	}
	RP_REG(PLL_SYS_BASE, PLL_PRIM_OFFSET) = (RP_PLL_DIV1 << PLL_PRIM_POSTDIV1_LSB) | (2U << PLL_PRIM_POSTDIV2_LSB);
	RP_CLR(PLL_SYS_BASE, PLL_PWR_OFFSET) = PLL_PWR_POSTDIVPD_BITS;
	RP_REG(CLOCKS_BASE, CLOCKS_CLK_SYS_DIV_OFFSET) = 1U << CLOCKS_CLK_SYS_DIV_INT_LSB;
	RP_REG(CLOCKS_BASE, CLOCKS_CLK_SYS_CTRL_OFFSET) = CLOCKS_CLK_SYS_CTRL_SRC_BITS;
	while (RP_REG(CLOCKS_BASE, CLOCKS_CLK_SYS_SELECTED_OFFSET) != 2U)
	{
	}
	// Keep UART independent of the CPU divider.
	RP_REG(CLOCKS_BASE, CLOCKS_CLK_PERI_CTRL_OFFSET) = CLOCKS_CLK_PERI_CTRL_ENABLE_BITS |
		(CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS << CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB);
}

extern void (*const __Vectors[])(void);

void SystemInit(void)
{
	RP_REG(0xe000ed00UL, 8) = (uintptr_t)__Vectors;
	__DSB();
	__ISB();
	SystemOscInit();
#ifdef RP2350
	RP_REG(TICKS_BASE, TICKS_TIMER0_CYCLES_OFFSET) = 12U;
	RP_REG(TICKS_BASE, TICKS_TIMER0_CTRL_OFFSET) = TICKS_TIMER0_CTRL_ENABLE_BITS;
	RP_REG(TICKS_BASE, TICKS_TIMER1_CYCLES_OFFSET) = 12U;
	RP_REG(TICKS_BASE, TICKS_TIMER1_CTRL_OFFSET) = TICKS_TIMER1_CTRL_ENABLE_BITS;
#else
	RP_REG(WATCHDOG_BASE, WATCHDOG_TICK_OFFSET) = WATCHDOG_TICK_ENABLE_BITS | 12U;
#endif
	RpReleaseReset(RESETS_RESET_IO_BANK0_BITS | RESETS_RESET_PADS_BANK0_BITS);
	SystemCoreClockUpdate();
}

uint32_t SystemCoreClockGet(void) { return SystemCoreClock; }
uint32_t SystemPeriphClockGet(int idx)
{
	uint32_t ctrl = RP_REG(CLOCKS_BASE, CLOCKS_CLK_PERI_CTRL_OFFSET);
	uint32_t hz = RP_SYS_HZ;

	if (idx != 0 || !(ctrl & CLOCKS_CLK_PERI_CTRL_ENABLE_BITS)) return 0;
	if ((ctrl & CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS) ==
		(CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC << CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB))
	{
		hz = 12000000U;
	}
#ifdef RP2350
	uint32_t div = RP_REG(CLOCKS_BASE, CLOCKS_CLK_PERI_DIV_OFFSET) >> CLOCKS_CLK_PERI_DIV_INT_LSB;
	hz /= div ? div : 4U;
#endif
	return hz;
}

uint32_t SystemPeriphClockSet(int idx, uint32_t hz)
{
	static const uint32_t source[] = {RP_SYS_HZ, 12000000U};
	uint32_t state = DisableInterrupt();
	uint32_t current = SystemPeriphClockGet(idx);
	uint32_t error = current > hz ? current - hz : hz - current;
	uint32_t selected = 0;
	uint32_t divider = 0;
#ifdef RP2350
	const uint32_t maxDiv = 4;
#else
	const uint32_t maxDiv = 1;
#endif

	// A running UART holds this clock and its configured baud rate.
	if (current && hz && !s_Users[0])
	{
		for (unsigned i = 0; i < 2; i++)
		{
			for (unsigned d = 1; d <= maxDiv; d++)
			{
				uint32_t actual = source[i] / d;
				uint32_t diff = actual > hz ? actual - hz : hz - actual;
				if (diff < error)
				{
					error = diff;
					selected = i;
					divider = d;
				}
			}
		}
		if (divider)
		{
			uint32_t mux = selected ? CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC :
				CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS;
			RP_CLR(CLOCKS_BASE, CLOCKS_CLK_PERI_CTRL_OFFSET) = CLOCKS_CLK_PERI_CTRL_ENABLE_BITS;
			// Allow the asynchronous gate to close before changing the mux.
			for (unsigned i = 0; i < 32; i++) __NOP();
#ifdef RP2350
			RP_REG(CLOCKS_BASE, CLOCKS_CLK_PERI_DIV_OFFSET) = (divider % 4U) << CLOCKS_CLK_PERI_DIV_INT_LSB;
#endif
			RP_REG(CLOCKS_BASE, CLOCKS_CLK_PERI_CTRL_OFFSET) = mux << CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB;
			RP_SET(CLOCKS_BASE, CLOCKS_CLK_PERI_CTRL_OFFSET) = CLOCKS_CLK_PERI_CTRL_ENABLE_BITS;
			current = SystemPeriphClockGet(idx);
		}
	}
	EnableInterrupt(state);
	return current;
}
bool SystemCoreClockSelect(OSC_TYPE src, uint32_t hz) { return src == OSC_TYPE_XTAL && hz == 12000000U; }
bool SystemLowFreqClockSelect(OSC_TYPE src, uint32_t hz) { return src == OSC_TYPE_XTAL && hz == 12000000U; }
extern bool SysTickTimerEnabled(void) __attribute__((weak));

bool SystemPowerModeSet(SYSTEM_POWER_MODE mode)
{
	if (mode != SYSTEM_POWER_MODE_PERFORMANCE && mode != SYSTEM_POWER_MODE_LOW_POWER)
	{
		return false;
	}
	if (mode != SystemPowerModeGet() && SysTickTimerEnabled && SysTickTimerEnabled())
	{
		return false;
	}
	RP_REG(CLOCKS_BASE, CLOCKS_CLK_SYS_DIV_OFFSET) =
		(mode == SYSTEM_POWER_MODE_LOW_POWER ? 2U : 1U) << CLOCKS_CLK_SYS_DIV_INT_LSB;
	SystemCoreClockUpdate();
	return true;
}
SYSTEM_POWER_MODE SystemPowerModeGet(void)
{
	return SystemCoreClock == RP_SYS_HZ ? SYSTEM_POWER_MODE_PERFORMANCE : SYSTEM_POWER_MODE_LOW_POWER;
}
// Both roots also supply the running system. The last user leaves them running.
static bool RpClockRequest(unsigned idx, bool enable)
{
	uint32_t state = DisableInterrupt();
	bool ok = enable ? s_Users[idx] != UINT32_MAX : s_Users[idx] != 0U;
	if (ok)
	{
		s_Users[idx] += enable ? 1U : (uint32_t)-1;
	}
	EnableInterrupt(state);
	return ok;
}
bool SystemHighFreqClockEnable(void) { return RpClockRequest(0, true); }
void SystemHighFreqClockDisable(void) { RpClockRequest(0, false); }
bool SystemLowFreqClockEnable(void) { return RpClockRequest(1, true); }
void SystemLowFreqClockDisable(void) { RpClockRequest(1, false); }
uint32_t SystemHighFreqClockStartTime(void) { return 0; }
uint32_t SystemLowFreqClockStartTime(void) { return 0; }
bool SystemLowFreqClockCalibrate(void) { return false; }
bool SystemHighFreqClockCalibrate(void) { return false; }
uint32_t SystemPowerCapabilitiesGet(void)
{
	return SYSTEM_POWER_CAP_SLEEP | SYSTEM_POWER_CAP_TIMER_WAKE | SYSTEM_POWER_CAP_RET_REG | SYSTEM_POWER_CAP_WAKE_PIN | SYSTEM_POWER_CAP_LOW_POWER;
}
void SystemSleep(void) { __DSB(); __WFI(); }
void SystemPowerOff(void) {}
bool SystemWakePin(int port, int pin, int sense) { (void)port; (void)pin; (void)sense; return false; }
uint32_t SystemResetReason(void)
{
	uint32_t reason = RP_REG(WATCHDOG_BASE, WATCHDOG_REASON_OFFSET);
	uint32_t flags = 0;

	if (reason & WATCHDOG_REASON_TIMER_BITS) flags |= SYSTEM_RESET_WDT;
	if (reason & WATCHDOG_REASON_FORCE_BITS) flags |= SYSTEM_RESET_SOFT;
#ifdef RP2350
	uint32_t reset = RP_REG(POWMAN_BASE, POWMAN_CHIP_RESET_OFFSET);
	if (reset & (POWMAN_CHIP_RESET_HAD_POR_BITS | POWMAN_CHIP_RESET_HAD_BOR_BITS)) flags |= SYSTEM_RESET_POWERON;
	if (reset & POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS) flags |= SYSTEM_RESET_PIN;
	if (reset & POWMAN_CHIP_RESET_HAD_DP_RESET_REQ_BITS) flags |= SYSTEM_RESET_DEBUG;
#else
	uint32_t reset = RP_REG(VREG_AND_CHIP_RESET_BASE, VREG_AND_CHIP_RESET_CHIP_RESET_OFFSET);
	if (reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) flags |= SYSTEM_RESET_POWERON;
	if (reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS) flags |= SYSTEM_RESET_PIN;
	if (reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS) flags |= SYSTEM_RESET_DEBUG;
#endif
	return flags;
}
// The boot ROM uses scratch 4 through 7 for watchdog reboot.
int SystemRetRegCount(void) { return 4; }
uint32_t SystemRetRegRead(int idx) {
	return (unsigned)idx < 4U ? RP_REG(WATCHDOG_BASE, WATCHDOG_SCRATCH0_OFFSET + 4U * idx) : 0U;
}
bool SystemRetRegWrite(int idx, uint32_t value) {
	if ((unsigned)idx >= 4U) return false;
	RP_REG(WATCHDOG_BASE, WATCHDOG_SCRATCH0_OFFSET + 4U * idx) = value;
	return true;
}
bool SystemRamRetain(uintptr_t addr, uint32_t size, bool retain)
{
	(void)addr;
	(void)size;
	(void)retain;
	return false;
}

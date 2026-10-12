// Host regression of the LPC11Uxx CTIMER driver with a register model.
// The model counts with PR = 0, sets the match flags of the enabled match
// interrupts and calls the IRQ handler like the NVIC. It jumps from match to
// match so that counter wraps can be tested.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <climits>
#include <initializer_list>

#include "LPC11Uxx.h"

struct W1C {
	uint32_t value = 0;
	operator uint32_t() const { return value; }
	W1C &operator=(uint32_t v) { value &= ~v; return *this; }
};

// TCR with CRST holding the counters at 0
struct Tcr {
	uint32_t value = 0;
	uint32_t *pTc = nullptr;
	uint32_t *pPc = nullptr;
	operator uint32_t() const { return value; }
	Tcr &operator=(uint32_t v)
	{
		value = v;
		if (v & 2U)
		{
			*pTc = 0;
			*pPc = 0;
		}
		return *this;
	}
	Tcr &operator&=(uint32_t v) { return *this = value & v; }
	Tcr &operator|=(uint32_t v) { return *this = value | v; }
};

struct CtimerRegs {
	W1C IR;
	Tcr TCR;
	uint32_t TC = 0, PR = 0, PC = 0, MCR = 0, MR[4] = {}, CCR = 0, CR[4] = {}, EMR = 0;
	uint32_t CTCR = 0, PWMC = 0, MSR[4] = {};
	CtimerRegs() { TCR.pTc = &TC; TCR.pPc = &PC; }
};

static CtimerRegs s_Ct[4];
alignas(8) static uint8_t s_SysconMem[sizeof(LPC_SYSCON_Type)];
static uint32_t s_NvicEn = 0, s_NvicPend = 0, s_NvicPrio[64];
uint32_t g_IrqMasked = 0;
uint32_t SystemCoreClock = 48000000;

void NVIC_ClearPendingIRQ(IRQn_Type n) { s_NvicPend &= ~(1UL << (n & 31)); }
void NVIC_SetPendingIRQ(IRQn_Type n) { s_NvicPend |= 1UL << (n & 31); }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { s_NvicPrio[n] = p; }
void NVIC_EnableIRQ(IRQn_Type n) { s_NvicEn |= 1UL << (n & 31); }
void NVIC_DisableIRQ(IRQn_Type n) { s_NvicEn &= ~(1UL << (n & 31)); }

#undef LPC_CT16B0
#undef LPC_CT16B1
#undef LPC_CT32B0
#undef LPC_CT32B1
#undef LPC_SYSCON
#define LPC_CTxxBx_Type CtimerRegs
#define LPC_CT16B0 (&s_Ct[0])
#define LPC_CT16B1 (&s_Ct[1])
#define LPC_CT32B0 (&s_Ct[2])
#define LPC_CT32B1 (&s_Ct[3])
#define LPC_SYSCON ((LPC_SYSCON_Type *)s_SysconMem)

#include "../../../ARM/NXP/LPC11xx/src/timer_lpc11uxx.cpp"
#include "../../../ARM/NXP/src/timer_lpcxx.cpp"

static const int s_CtNo[4] = {0, 1, 2, 3};
static void (* const s_Handler[4])(void) = {
	TIMER16_0_IRQHandler, TIMER16_1_IRQHandler, TIMER32_0_IRQHandler, TIMER32_1_IRQHandler
};

static void Service(int DevNo)
{
	CtimerRegs &r = s_Ct[s_CtNo[DevNo]];
	uint32_t bit = 1UL << (g_LpcTimer[DevNo].IrqNo & 31);

	for (int i = 0; i < 4 && (s_NvicEn & bit) && g_IrqMasked == 0 && ((s_NvicPend & bit) || (r.IR.value & 15U)); i++)
	{
		s_NvicPend &= ~bit;
		s_Handler[DevNo]();
	}
}

// Advance the counter by Ticks, PR is 0 in the model
static void Advance(int DevNo, uint64_t Ticks)
{
	CtimerRegs &r = s_Ct[s_CtNo[DevNo]];

	assert(r.PR == 0);
	while (Ticks > 0)
	{
		if ((r.TCR & 1U) == 0 || (r.TCR & 2U))
		{
			return;
		}
		uint64_t step = Ticks;
		for (int n = 0; n < 4; n++)
		{
			if (r.MCR & (1U << (3 * n)))
			{
				uint64_t d = ((r.MR[n] - r.TC - 1U) & g_LpcTimer[DevNo].CounterMask) + 1ULL;
				if (d < step)
				{
					step = d;
				}
			}
		}
		r.TC = (uint32_t)(r.TC + step) & g_LpcTimer[DevNo].CounterMask;
		Ticks -= step;
		for (int n = 0; n < 4; n++)
		{
			if ((r.MCR & (1U << (3 * n))) && r.TC == r.MR[n])
			{
				r.IR.value |= 1UL << n;
			}
		}
		Service(DevNo);
	}
}

static unsigned s_Ovr = 0, s_Evt[3] = {}, s_Cb = 0;
static uint64_t s_EvtAt[3] = {};
static void EvtHandler(TimerDev_t * const pTimer, uint32_t Evt)
{
	if (Evt & TIMER_EVT_COUNTER_OVR)
	{
		s_Ovr++;
	}
	for (int n = 0; n < 3; n++)
	{
		if (Evt & TIMER_EVT_TRIGGER(n))
		{
			s_Evt[n]++;
			s_EvtAt[n] = pTimer->GetTickCount(pTimer);
		}
	}
}

static void ResetCounts(void)
{
	s_Ovr = 0;
	s_Cb = 0;
	for (int n = 0; n < 3; n++)
	{
		s_Evt[n] = 0;
		s_EvtAt[n] = 0;
	}
}

static void ResetHandler(TimerDev_t * const pTimer, int, void *)
{
	s_Cb++;
	pTimer->Reset(pTimer);
}

int main()
{
	Timer timers[4];
	TimerDev_t other = {};
	TimerCfg_t cfg = {};
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.IntPrio = 3;
	cfg.EvtHandler = EvtHandler;
	assert(TimerGetHighFreqDevCount() == 4 && TimerGetLowFreqDevCount() == 0);
	assert(TimerGetHighFreqDevNo() == 0);
	cfg.DevNo = 4;
	assert(!TimerInit(&other, &cfg));
	cfg.DevNo = 0;
	SystemCoreClock = 0;
	assert(!TimerInit(&other, &cfg));
	SystemCoreClock = 48000000;
	for (int d = 0; d < 4; d++)
	{
		cfg.DevNo = d;
		TimerDev_t &t = *(TimerDev_t *)timers[d];
		int maxprio = (1U << __NVIC_PRIO_BITS) - 1U;
		for (int priority : {INT_MIN, -1, 0, 1, 2, 3, maxprio, maxprio + 1, INT_MAX})
		{
			cfg.IntPrio = priority;
			uint32_t expected = priority < 0 ? 0U : priority > maxprio ? (uint32_t)maxprio : (uint32_t)priority;
		
			assert(TimerInit(&t, &cfg));
			assert(cfg.IntPrio == priority);
			assert(s_NvicPrio[g_LpcTimer[d].IrqNo] == expected);
			t.Disable(&t);
			assert(t.Enable(&t));
			assert(s_NvicPrio[g_LpcTimer[d].IrqNo] == expected);
		}
		cfg.IntPrio = 3;
		assert(TimerInit(&t, &cfg));
		assert(t.Freq == SystemCoreClock && s_Ct[d].PR == 0);
		assert(LPC_SYSCON->SYSAHBCLKCTRL & (1U << (7 + d)));
		assert(s_NvicPrio[g_LpcTimer[d].IrqNo] == 3);
		assert(!TimerInit(&other, &cfg));
		assert(t.GetMaxTrigger(&t) == 3);
		assert(!t.EnableExtTrigger(&t, 0, TIMER_EXTTRIG_SENSE_HIGH_TRANSITION));
		uint64_t cycle = (uint64_t)g_LpcTimer[d].CounterMask + 1;
		ResetCounts();
		Advance(d, 3 * cycle + 123);
		assert(t.GetTickCount(&t) == 3 * cycle + 123 && s_Ovr == 3);
		t.Reset(&t);
		ResetCounts();
		// One-shot longer than a 16-bit cycle, with no early callback.
		assert(t.EnableTrigger(&t, 0, 10000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 10000000);
		Advance(d, 479999);
		assert(s_Evt[0] == 0);
		Advance(d, 1);
		assert(s_Evt[0] == 1 && t.FindAvailTrigger(&t) == 0);
		// Masked ISR coalesces missed periods and keeps the original phase.
		t.Reset(&t);
		ResetCounts();
		t.EnableTrigger(&t, 0, 100000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
		g_IrqMasked = 1;
		Advance(d, 20000);
		g_IrqMasked = 0;
		Service(d);
		assert(s_Evt[0] == 1);
		Advance(d, 4000);
		assert(s_Evt[0] == 2 && s_EvtAt[0] == 24000);
		t.Disable(&t);
		Advance(d, 10000);
		assert(t.GetTickCount(&t) == 24000);
		assert(t.Enable(&t));
		Advance(d, 4800);
		assert(s_Evt[0] == 3);
		// Reset from one callback prevents stale callbacks for other matches.
		t.Reset(&t);
		ResetCounts();
		t.EnableTrigger(&t, 0, 100000, TIMER_TRIG_TYPE_SINGLE, ResetHandler, nullptr);
		t.EnableTrigger(&t, 1, 100000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr);
		Advance(d, 4800);
		assert(s_Cb == 1 && s_Evt[1] == 0 && t.GetTickCount(&t) == 0);
		Advance(d, 4800);
		assert(s_Evt[1] == 1);
		// Frequency selection must minimize Hz error, not divisor error.
		assert(t.SetFrequency(&t, 34000000) == 24000000 && s_Ct[d].PR == 1);
		assert(t.SetFrequency(&t, 36000000) == 48000000 && s_Ct[d].PR == 0);
		assert(t.SetFrequency(&t, UINT32_MAX) == 48000000);
		assert(t.SetFrequency(&t, 1) == (d < 2 ? 732U : 1U));
		assert(s_Ct[d].PR == (d < 2 ? 65535U : 47999999U));
		t.Disable(&t);
		assert(t.SetFrequency(&t, 1000000) == 1000000 && s_Ct[d].TCR == 0);
		t.EnableTrigger(&t, 2, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
		assert(t.SetFrequency(&t, 2000000) == 2000000);
		assert(g_LpcTimer[d].Trig[2].Ticks == 2000 && g_LpcTimer[d].Trig[2].Deadline == 2000);
		// Extreme duration conversion must not overflow into a short delay.
		assert(t.EnableTrigger(&t, 2, UINT64_MAX, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) > UINT64_MAX - 1000);
		assert(g_LpcTimer[d].Trig[2].Ticks > 10000000000000000ULL);
		// Automatic allocation must distinguish exhaustion from valid slot 0.
		assert(timers[d].Init(cfg));
		// A free slot is not success when the driver cannot arm it.
		t.Freq = 0;
		assert(timers[d].EnableTimerTrigger((uint64_t)1000000,
			TIMER_TRIG_TYPE_CONTINUOUS) == -1);
		assert(timers[d].EnableTimerTrigger((uint32_t)1,
			TIMER_TRIG_TYPE_CONTINUOUS) == -1);
		assert(timers[d].FindAvailTimerTrigger() == 0);
		t.Freq = SystemCoreClock;
		// A successful sub-millisecond period rounds to 0 ms, not failure.
		assert(timers[d].EnableTimerTrigger((uint32_t)0,
			TIMER_TRIG_TYPE_CONTINUOUS) == 0);
		assert(g_LpcTimer[d].Trig[0].bActive);
		timers[d].DisableTimerTrigger(0);
		for (int n = 0; n < 3; n++)
		{
			assert(timers[d].EnableTimerTrigger((uint64_t)1000000,
				TIMER_TRIG_TYPE_CONTINUOUS) == n);
		}
		assert(timers[d].EnableTimerTrigger((uint64_t)1000000,
			TIMER_TRIG_TYPE_CONTINUOUS) == -1);
		assert(timers[d].EnableTimerTrigger((uint32_t)1,
			TIMER_TRIG_TYPE_CONTINUOUS) == -1);
		timers[d].DisableTimerTrigger(0);
		assert(timers[d].EnableTimerTrigger((uint32_t)1,
			TIMER_TRIG_TYPE_CONTINUOUS) == 0);
		timers[d].Disable();
	}
	puts("lpc11uxx timer: PASS");
	return 0;
}

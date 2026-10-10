// Host regression of the LPC546xx CTIMER driver with a register model.
// The model counts with PR = 0, sets the match flags of the enabled match
// interrupts and calls the IRQ handler like the NVIC. It jumps from match to
// match so that counter wraps can be tested.
#include <cassert>
#include <cstdio>
#include <cstdint>

#include "LPC546xx.h"

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
		if (v & CTIMER_TCR_CRST_MASK)
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

static CtimerRegs s_Ct[5];
// The device header register structures have read-only members, plain
// memory stands for them.
alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
alignas(8) static uint8_t s_AsyncSysconMem[sizeof(ASYNC_SYSCON_Type)];
static uint32_t s_NvicEn = 0, s_NvicPend = 0, s_NvicPrio[64];
uint32_t g_IrqMasked = 0;
uint32_t SystemCoreClock = 220000000;

void NVIC_ClearPendingIRQ(IRQn_Type n) { s_NvicPend &= ~(1UL << (n & 31)); }
void NVIC_SetPendingIRQ(IRQn_Type n) { s_NvicPend |= 1UL << (n & 31); }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { s_NvicPrio[n] = p; }
void NVIC_EnableIRQ(IRQn_Type n) { s_NvicEn |= 1UL << (n & 31); }
void NVIC_DisableIRQ(IRQn_Type n) { s_NvicEn &= ~(1UL << (n & 31)); }

#undef CTIMER0
#undef CTIMER1
#undef CTIMER2
#undef CTIMER3
#undef CTIMER4
#undef SYSCON
#undef ASYNC_SYSCON
#define CTIMER_Type CtimerRegs
#define CTIMER0 (&s_Ct[0])
#define CTIMER1 (&s_Ct[1])
#define CTIMER2 (&s_Ct[2])
#define CTIMER3 (&s_Ct[3])
#define CTIMER4 (&s_Ct[4])
#define SYSCON ((SYSCON_Type *)s_SysconMem)
#define ASYNC_SYSCON ((ASYNC_SYSCON_Type *)s_AsyncSysconMem)

#include "../../ARM/NXP/LPC546xx/src/timer_lpc546xx.cpp"

// Virtual DevNo to CTIMER number and its handler
static const int s_CtNo[5] = {3, 4, 0, 1, 2};
static void (* const s_Handler[5])(void) = {
	CTIMER3_IRQHandler, CTIMER4_IRQHandler, CTIMER0_IRQHandler, CTIMER1_IRQHandler, CTIMER2_IRQHandler
};

static void Service(int DevNo)
{
	CtimerRegs &r = s_Ct[s_CtNo[DevNo]];
	uint32_t bit = 1UL << (s_Lpc546xxTimer[DevNo].IrqNo & 31);

	for (int i = 0; i < 4 && (s_NvicEn & bit) && g_IrqMasked == 0 && ((s_NvicPend & bit) || r.IR.value); i++)
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
		if ((r.TCR & CTIMER_TCR_CEN_MASK) == 0 || (r.TCR & CTIMER_TCR_CRST_MASK))
		{
			return;
		}
		uint64_t step = Ticks;
		for (int n = 0; n < 4; n++)
		{
			if (r.MCR & (CTIMER_MCR_MR0I_MASK << (3 * n)))
			{
				uint64_t d = (uint32_t)(r.MR[n] - r.TC - 1U) + 1ULL;
				if (d < step)
				{
					step = d;
				}
			}
		}
		r.TC = (uint32_t)(r.TC + step);
		Ticks -= step;
		for (int n = 0; n < 4; n++)
		{
			if ((r.MCR & (CTIMER_MCR_MR0I_MASK << (3 * n))) && r.TC == r.MR[n])
			{
				r.IR.value |= 1UL << n;
			}
		}
		Service(DevNo);
	}
}

static unsigned s_Ovr = 0, s_Evt[3] = {}, s_Cb = 0;
static uint64_t s_EvtAt[3] = {};
static TimerDev_t *s_Dev = nullptr;

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

static void TrigHandler(TimerDev_t * const pTimer, int TrigNo, void * const pCtx)
{
	assert(pTimer == s_Dev && TrigNo == 2 && pCtx == &s_Cb);
	s_Cb++;
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

int main()
{
	TimerDev_t t0 = {}, t2 = {}, tx = {};
	TimerCfg_t cfg = {};

	// Configuration checks
	cfg.DevNo = 0;
	cfg.IntPrio = 2;
	cfg.EvtHandler = EvtHandler;
	cfg.ClkSrc = TIMER_CLKSRC_LFXTAL;
	assert(!TimerInit(&t0, &cfg));
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.bTickInt = true;
	assert(!TimerInit(&t0, &cfg));
	cfg.bTickInt = false;
	cfg.IntPrio = 8;
	assert(!TimerInit(&t0, &cfg));
	cfg.IntPrio = 2;
	cfg.DevNo = 5;
	assert(!TimerInit(&t0, &cfg));
	assert(TimerGetLowFreqDevCount() == 0 && TimerGetHighFreqDevCount() == 5 && TimerGetHighFreqDevNo() == 0);

	// DevNo 0 is CTIMER3 on the async bridge, 12 MHz FRO, full rate
	cfg.DevNo = 0;
	cfg.Freq = 0;
	assert(TimerInit(&t0, &cfg));
	s_Dev = &t0;
	assert(t0.Freq == 12000000 && s_Ct[3].PR == 0);
	assert(SYSCON->ASYNCAPBCTRL == SYSCON_ASYNCAPBCTRL_ENABLE_MASK && ASYNC_SYSCON->ASYNCAPBCLKSELA == 1U);
	assert(s_Ct[3].TCR == CTIMER_TCR_CEN_MASK && s_Ct[3].MR[3] == 0x80000000UL);
	assert(s_NvicEn & (1UL << CTIMER3_IRQn));
	assert(s_NvicPrio[CTIMER3_IRQn] == 2);

	// Same object on a second CTIMER, or a second object on the same one
	cfg.DevNo = 1;
	assert(!TimerInit(&t0, &cfg));
	cfg.DevNo = 0;
	assert(!TimerInit(&tx, &cfg));
	assert(t0.GetMaxTrigger(&t0) == 3 && t0.FindAvailTrigger(&t0) == 0);
	assert(!t0.EnableExtTrigger(&t0, 0, TIMER_EXTTRIG_SENSE_HIGH_TRANSITION));

	// 64 bit count over three counter wraps
	Advance(0, 3ULL * 0x100000000ULL + 12345U);
	assert(t0.GetTickCount(&t0) == 3ULL * 0x100000000ULL + 12345U);
	assert(s_Ovr == 3);

	// Reset brings the count to 0
	t0.Reset(&t0);
	assert(t0.GetTickCount(&t0) == 0 && s_Ct[3].TC == 0 && (s_Ct[3].TCR & CTIMER_TCR_CEN_MASK));
	ResetCounts();

	// Continuous 1 ms trigger, 12000 ticks
	assert(t0.EnableTrigger(&t0, 0, 1000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr) == 1000000ULL);
	Advance(0, 10U * 12000U);
	assert(s_Evt[0] == 10 && s_EvtAt[0] == 10U * 12000U);

	// Single shot with a callback, the trigger is free again after it
	assert(t0.EnableTrigger(&t0, 2, 500000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, &s_Cb) == 500000ULL);
	assert(t0.FindAvailTrigger(&t0) == 1);
	Advance(0, 3U * 6000U);
	assert(s_Cb == 1 && t0.FindAvailTrigger(&t0) == 1);
	t0.DisableTrigger(&t0, 1);
	assert(s_Ct[3].MCR == ((CTIMER_MCR_MR0I_MASK << 9) | CTIMER_MCR_MR0I_MASK));

	// Period longer than a counter cycle, 400 s at 12 MHz. The match on its
	// low 32 bits one cycle earlier is not the trigger.
	t0.DisableTrigger(&t0, 0);
	t0.Reset(&t0);
	ResetCounts();
	assert(t0.EnableTrigger(&t0, 1, 400000000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr) == 400000000000ULL);
	Advance(0, 4800000000ULL - 1U);
	assert(s_Evt[1] == 0);
	Advance(0, 1U);
	assert(s_Evt[1] == 1 && s_EvtAt[1] == 4800000000ULL);
	Advance(0, 4800000000ULL);
	assert(s_Evt[1] == 2 && s_EvtAt[1] == 9600000000ULL);
	t0.DisableTrigger(&t0, 1);

	// Shortest period is LPC546XX_TIMER_TICKS_MIN ticks
	ResetCounts();
	assert(t0.EnableTrigger(&t0, 0, 1ULL, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 333ULL);
	assert(s_Lpc546xxTimer[0].Trig[0].Ticks == 4U);
	t0.DisableTrigger(&t0, 0);

	// A deadline already passed when the match register is written goes
	// through the pending interrupt
	g_IrqMasked = 1;
	assert(t0.EnableTrigger(&t0, 0, 1000ULL, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 1000ULL);
	s_Lpc546xxTimer[0].Trig[0].Deadline = t0.GetTickCount(&t0);
	Lpc546xxTimerArm(&s_Lpc546xxTimer[0], 0);
	g_IrqMasked = 0;
	assert(s_NvicPend & (1UL << CTIMER3_IRQn));
	Service(0);
	assert(s_Evt[0] == 1 && t0.FindAvailTrigger(&t0) == 0);

	// Disable stops the counter, Enable restarts it
	t0.Disable(&t0);
	uint64_t c = t0.GetTickCount(&t0);
	Advance(0, 1000U);
	assert(t0.GetTickCount(&t0) == c && (s_NvicEn & (1UL << CTIMER3_IRQn)) == 0);
	assert(t0.Enable(&t0));
	Advance(0, 1000U);
	assert(t0.GetTickCount(&t0) == c + 1000U);

	// DevNo 2 is CTIMER0 on the system clock. 10 kHz from 220 MHz.
	cfg.DevNo = 2;
	cfg.Freq = 10000;
	assert(TimerInit(&t2, &cfg));
	assert(t2.Freq == 10000 && s_Ct[0].PR == 21999U);
	assert(SYSCON->AHBCLKCTRLSET[1] == SYSCON_AHBCLKCTRL_CTIMER0_MASK);
	assert(SYSCON->PRESETCTRLCLR[1] == SYSCON_PRESETCTRL_CTIMER0_RST_MASK);

	// Closest frequency, the trigger keeps its period in time
	assert(t2.EnableTrigger(&t2, 0, 100000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr) == 100000000ULL);
	assert(s_Lpc546xxTimer[2].Trig[0].Ticks == 1000U);
	assert(t2.SetFrequency(&t2, 3000000) == 3013698U);
	assert(s_Ct[0].PR == 72U);
	assert(s_Lpc546xxTimer[2].Trig[0].Ticks == 301370U);
	assert(t2.SetFrequency(&t2, 1) == 1U && s_Ct[0].PR == 219999999U);

	printf("lpc546xx timer: PASS\n");

	return 0;
}

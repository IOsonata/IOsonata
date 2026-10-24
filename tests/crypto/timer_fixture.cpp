// Deterministic timer for crypto host/register-model tests only.
// Seven ticks per read, including a 32-bit rollover, at a non-integer MHz.
// Reported benchmark times are simulated, not performance measurements.
#include <cassert>
#include "coredev/timer.h"

bool g_CryptoTimerFail = false;
bool g_CryptoTimerStopped = true;
static uint64_t s_Count;

static uint64_t Read(TimerDev_t *)
{
	assert(!g_CryptoTimerStopped);
	s_Count += 7;
	return s_Count;
}

static void Stop(TimerDev_t *)
{
	g_CryptoTimerStopped = true;
}

extern "C" int TimerGetHighFreqDevNo(void)
{
	return 7;
}

extern "C" bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	assert(pCfg->DevNo == 7);
	assert(pCfg->Freq == 1000000U);
	assert(pCfg->ClkSrc == TIMER_CLKSRC_DEFAULT);
	assert(!pCfg->bTickInt);
	if (g_CryptoTimerFail)
	{
		return false;
	}
	pTimer->Freq = 32768U;
	pTimer->GetTickCount = Read;
	pTimer->Disable = Stop;
	s_Count = 0xfffffffcULL;
	g_CryptoTimerStopped = false;
	return true;
}

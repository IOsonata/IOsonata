/**-------------------------------------------------------------------------
@file nrf70_osal_time.cpp
@brief Nordic nRF70 timing using IOsonata board clock and delay.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "net/nrf70_osal_time.h"
#include "idelay.h"
#ifdef IO_ENABLE_NRF70_BM
#include "osal_ops.h"
#endif

static Nrf70OsalClock_t s_Clock;
static void *s_pContext;

extern "C" void Nrf70OsalTimeInit(Nrf70OsalClock_t Clock, void *pContext)
{
	s_Clock = Clock;
	s_pContext = pContext;
}

extern "C" int Nrf70OsalDelayUs(int MicroSeconds)
{
	if (MicroSeconds < 0)
		return -1;
	while (MicroSeconds > 100000)
	{
		usDelay(100000);
		MicroSeconds -= 100000;
	}
	if (MicroSeconds > 0)
		usDelay((uint32_t)MicroSeconds);
	return 0;
}

extern "C" int Nrf70OsalSleepMs(int MilliSeconds)
{
	if (MilliSeconds < 0)
		return -1;
	while (MilliSeconds > 100)
	{
		msDelay(100);
		MilliSeconds -= 100;
	}
	if (MilliSeconds > 0)
		msDelay((uint32_t)MilliSeconds);
	return 0;
}

extern "C" unsigned long Nrf70OsalCurrentUs(void)
{
	if (s_Clock == nullptr)
		return 0;
	return (unsigned long)s_Clock(s_pContext);
}

extern "C" unsigned int Nrf70OsalElapsedUs(unsigned long Start)
{
	return (unsigned int)(Nrf70OsalCurrentUs() - Start);
}

#ifdef IO_ENABLE_NRF70_BM
extern "C" bool Nrf70OsalInstallTime(struct nrf_wifi_osal_ops *pOps)
{
	if (pOps == nullptr || s_Clock == nullptr)
		return false;
	pOps->sleep_ms = Nrf70OsalSleepMs;
	pOps->delay_us = Nrf70OsalDelayUs;
	pOps->time_get_curr_us = Nrf70OsalCurrentUs;
	pOps->time_elapsed_us = Nrf70OsalElapsedUs;
	return true;
}
#endif

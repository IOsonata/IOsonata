/**-------------------------------------------------------------------------
@file nrf70_osal_time.h
@brief Nordic nRF70 timing callbacks backed by IOsonata timebase.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __NRF70_OSAL_TIME_H__
#define __NRF70_OSAL_TIME_H__
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Monotonic microseconds; supplied by an initialized board timer. */
typedef uint64_t (*Nrf70OsalClock_t)(void *pContext);
void Nrf70OsalTimeInit(Nrf70OsalClock_t Clock, void *pContext);
int Nrf70OsalSleepMs(int MilliSeconds);
int Nrf70OsalDelayUs(int MicroSeconds);
unsigned long Nrf70OsalCurrentUs(void);
unsigned int Nrf70OsalElapsedUs(unsigned long Start);
#ifdef IO_ENABLE_NRF70_BM
struct nrf_wifi_osal_ops;
bool Nrf70OsalInstallTime(struct nrf_wifi_osal_ops *pOps);
#endif
#ifdef __cplusplus
}
#endif
#endif

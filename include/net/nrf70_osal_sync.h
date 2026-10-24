/**-------------------------------------------------------------------------
@file nrf70_osal_sync.h
@brief Single-core bare-metal synchronization for Nordic nRF70.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __NRF70_OSAL_SYNC_H__
#define __NRF70_OSAL_SYNC_H__
#ifdef __cplusplus
extern "C" {
#endif
void *Nrf70OsalSpinAlloc(void);
void Nrf70OsalSpinFree(void *Lock);
void Nrf70OsalSpinInit(void *Lock);
void Nrf70OsalSpinTake(void *Lock);
void Nrf70OsalSpinRelease(void *Lock);
void Nrf70OsalSpinIrqTake(void *Lock, unsigned long *Flags);
void Nrf70OsalSpinIrqRelease(void *Lock, unsigned long *Flags);
#ifdef IO_ENABLE_NRF70_BM
struct nrf_wifi_osal_ops;
void Nrf70OsalInstallSync(struct nrf_wifi_osal_ops *Ops);
#endif
#ifdef __cplusplus
}
#endif
#endif

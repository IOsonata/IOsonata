/**-------------------------------------------------------------------------
@file nrf70_osal_sync.h
@brief Single-core bare-metal synchronization for Nordic nRF70.
@license MIT License
Copyright (c) 2026 I-SYST inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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

/**-------------------------------------------------------------------------
@file nrf70_tasklet.h
@brief Fixed-capacity Nordic host tasklet dispatcher using IOsonata AppEvt.
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
#ifndef __NRF70_TASKLET_H__
#define __NRF70_TASKLET_H__
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef void (*Nrf70TaskletCb_t)(unsigned long Context);
typedef struct __Nrf70_Tasklet
{
	Nrf70TaskletCb_t Callback;
	unsigned long Context;
	bool Pending;
	bool Queued;
	bool Active;
} Nrf70Tasklet_t;

/* Tasklets are caller-owned. Init accepts uninitialized storage and leaves
 * Active false if all 16 registry slots are occupied. Init cannot replace a
 * tasklet while its Queued flag is true.
 * Cancel releases an idle tasklet immediately. A cancelled queued tasklet
 * must remain alive until dispatch clears Queued; its callback is suppressed
 * and its registry slot is then released. Call Init again before reuse. */
void Nrf70TaskletInit(Nrf70Tasklet_t *pTask, Nrf70TaskletCb_t Callback,
	unsigned long Context);
bool Nrf70TaskletSchedule(Nrf70Tasklet_t *pTask);
void Nrf70TaskletCancel(Nrf70Tasklet_t *pTask);
void Nrf70TaskletCheckStatus(void);
bool Nrf70TaskletPending(void);
#ifdef __cplusplus
}
#endif
#endif

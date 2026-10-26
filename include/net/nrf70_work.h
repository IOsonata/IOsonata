/**-------------------------------------------------------------------------
@file nrf70_work.h
@brief Deferred nRF70 interrupt processing for bare-metal IOsonata.
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
#ifndef __NRF70_WORK_H__
#define __NRF70_WORK_H__
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef void (*Nrf70WorkHandler_t)(void *pContext);

/*
 * One nRF70 hardware interrupt source with a single foreground owner.
 * IRQ handler never executes Nordic bus or firmware code.
 * Call Nrf70WorkProcess() from the owning foreground AppEvt handler.
 */
void Nrf70WorkInit(Nrf70WorkHandler_t Handler, void *pContext);
void Nrf70WorkIrq(void);
bool Nrf70WorkPending(void);
void Nrf70WorkProcess(void);
/** Retry a refused nRF70 event when the shared application queue has space. */
void Nrf70WorkCheckStatus(void);
void Nrf70WorkClose(void);

#ifdef __cplusplus
}
#endif
#endif

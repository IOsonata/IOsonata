/**-------------------------------------------------------------------------
@file nrf70_work.h
@brief Deferred nRF70 interrupt processing for bare-metal IOsonata.
@license MIT License
Copyright (c) 2026 I-SYST inc.
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

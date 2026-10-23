/**-------------------------------------------------------------------------
@file nrf70_tasklet.h
@brief Fixed-capacity Nordic host tasklet dispatcher using IOsonata AppEvt.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __NRF70_TASKLET_H__
#define __NRF70_TASKLET_H__
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef void (*Nrf70TaskletCb_t)(unsigned long Context);
typedef struct __Nrf70_Tasklet {
	Nrf70TaskletCb_t Callback;
	unsigned long Context;
	bool Pending;
	bool Queued;
	bool Active;
} Nrf70Tasklet_t;

/* Tasklets are caller-owned, and must outlive any queued event. */
void Nrf70TaskletInit(Nrf70Tasklet_t *pTask, Nrf70TaskletCb_t Callback,
	unsigned long Context);
bool Nrf70TaskletSchedule(Nrf70Tasklet_t *pTask);
void Nrf70TaskletCancel(Nrf70Tasklet_t *pTask);
void Nrf70TaskletCheckStatus(void);
#ifdef __cplusplus
}
#endif
#endif

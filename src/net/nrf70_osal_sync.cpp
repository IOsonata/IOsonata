/**-------------------------------------------------------------------------
@file nrf70_osal_sync.cpp
@brief Nordic nRF70 synchronization on a single-core bare-metal owner.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include <stdlib.h>
#include "coredev/interrupt.h"
#include "net/nrf70_osal_sync.h"
#ifdef IO_ENABLE_NRF70_BM
#include "osal_ops.h"
#endif

typedef struct {
	uint32_t PreviousState;
} Nrf70OsalSpin_t;

/*
 * This implementation is restricted to one MCU core with IRQ callbacks
 * deferred into AppEvt. Never share these locks between the nRF5340 cores
 * or with an independent RTOS thread. In that case provide a port override.
 */
extern "C" void *Nrf70OsalSpinAlloc(void)
{
	return calloc(1, sizeof(Nrf70OsalSpin_t));
}

extern "C" void Nrf70OsalSpinFree(void *Lock)
{
	free(Lock);
}

extern "C" void Nrf70OsalSpinInit(void *Lock)
{
	Nrf70OsalSpin_t *pLock = (Nrf70OsalSpin_t *)Lock;
	if (pLock != nullptr)
		pLock->PreviousState = 0;
}

extern "C" void Nrf70OsalSpinTake(void *Lock)
{
	Nrf70OsalSpin_t *pLock = (Nrf70OsalSpin_t *)Lock;
	if (pLock != nullptr)
		pLock->PreviousState = DisableInterrupt();
}

extern "C" void Nrf70OsalSpinRelease(void *Lock)
{
	Nrf70OsalSpin_t *pLock = (Nrf70OsalSpin_t *)Lock;
	if (pLock != nullptr)
		EnableInterrupt(pLock->PreviousState);
}

extern "C" void Nrf70OsalSpinIrqTake(void *Lock, unsigned long *Flags)
{
	uint32_t state;
	(void)Lock;
	state = DisableInterrupt();
	if (Flags != nullptr)
		*Flags = (unsigned long)state;
}

extern "C" void Nrf70OsalSpinIrqRelease(void *Lock, unsigned long *Flags)
{
	(void)Lock;
	if (Flags != nullptr)
		EnableInterrupt((uint32_t)*Flags);
}

#ifdef IO_ENABLE_NRF70_BM
extern "C" void Nrf70OsalInstallSync(struct nrf_wifi_osal_ops *Ops)
{
	if (Ops == nullptr)
		return;
	Ops->spinlock_alloc = Nrf70OsalSpinAlloc;
	Ops->spinlock_free = Nrf70OsalSpinFree;
	Ops->spinlock_init = Nrf70OsalSpinInit;
	Ops->spinlock_take = Nrf70OsalSpinTake;
	Ops->spinlock_rel = Nrf70OsalSpinRelease;
	Ops->spinlock_irq_take = Nrf70OsalSpinIrqTake;
	Ops->spinlock_irq_rel = Nrf70OsalSpinIrqRelease;
}
#endif

/**-------------------------------------------------------------------------
@file	modem_os_nrf91_taktos.c

@brief	nRF91 Modem library glue, TaktOS waits, semaphores and mutex

Link this file into a TaktOS application that uses the modem, with the
TaktOS include paths. Its functions replace the weak bare metal ones of
modem_os_nrf91.c; memory, errno, logging, the IPC interrupt and
nRF91ModemInit stay the library ones. The pTimer and TimerTrigNo of the
modem configuration are not used: timeouts are kernel ticks.

Any number of threads may call the Modem library. A thread in
nrf_modem_os_timedwait waits on its own semaphore, given by
nrf_modem_os_event_notify when the context matches. Each thread also keeps
the event count it last saw, so a thread does not sleep through an event
that came after its last check: the same scheme as the nRF Connect SDK glue.

The Modem library needs a recursive mutex. The TaktOS mutex is not, so the
owner thread and the lock depth are kept here around it.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"

#ifndef NRF_TRUSTZONE_NONSECURE
#error "modem_os_nrf91_taktos.c is for a non secure build, the Modem library runs non secure"
#endif

#include "TaktOS.h"
#include "TaktKernel.h"
#include "TaktOSThread.h"
#include "TaktOSSem.h"
#include "TaktOSMutex.h"

#include "nrf_modem.h"
#include "nrf_modem_os.h"
#include "nrf_errno.h"
#include "coredev/interrupt.h"

/** @addtogroup LTE
  * @{
  */

// Threads that call the Modem library, for the event counts they last saw.
// The oldest entry is reused when a new thread calls.
#ifndef NRF91_MODEM_TAKTOS_THREAD_MAX
#define NRF91_MODEM_TAKTOS_THREAD_MAX	8
#endif

// Threads waiting in nrf_modem_os_timedwait at the same time
#ifndef NRF91_MODEM_TAKTOS_WAIT_MAX
#define NRF91_MODEM_TAKTOS_WAIT_MAX		8
#endif

typedef struct {
	hTaktOSThread_t hThread;		// NULL when unused
	uint32_t Cnt;					// Event count the thread saw last
} nRF91ModemTaktThread_t;

typedef struct {
	bool bUsed;
	uint32_t Context;
	TaktOSSem_t Sem;
} nRF91ModemTaktWait_t;

typedef struct {
	hTaktOSThread_t hOwner;			// Holder, valid while Depth > 0
	uint32_t Depth;					// Lock depth
	TaktOSMutex_t Mutex;
} nRF91ModemTaktMutex_t;

typedef struct {
	uint32_t EvtCnt;				// Event notifications so far
	nRF91ModemTaktThread_t Thread[NRF91_MODEM_TAKTOS_THREAD_MAX];
	nRF91ModemTaktWait_t Wait[NRF91_MODEM_TAKTOS_WAIT_MAX];
	int NbSem;
	TaktOSSem_t Sem[NRF_MODEM_OS_NUM_SEM_REQUIRED];
	int NbMutex;
	nRF91ModemTaktMutex_t Mutex[NRF_MODEM_OS_NUM_MUTEX_REQUIRED];
} nRF91ModemTakt_t;

static nRF91ModemTakt_t s_nRF91ModemTakt;

// Milliseconds to kernel ticks, rounded up. Negative is no timeout.
static uint32_t nRF91ModemTaktTicks(int32_t Ms)
{
	if (Ms < 0)
	{
		return TAKTOS_WAIT_FOREVER;
	}

	uint32_t hz = TaktOSGetTickHz();

	return (uint32_t)(((uint64_t)Ms * hz + 999U) / 1000U);
}

// Entry of the calling thread, called with interrupts masked
static nRF91ModemTaktThread_t *nRF91ModemTaktThread(hTaktOSThread_t hThread)
{
	nRF91ModemTaktThread_t *unused = NULL;
	nRF91ModemTaktThread_t *oldest = NULL;
	uint32_t oldestage = 0;

	for (int i = 0; i < NRF91_MODEM_TAKTOS_THREAD_MAX; i++)
	{
		nRF91ModemTaktThread_t *t = &s_nRF91ModemTakt.Thread[i];

		if (t->hThread == hThread)
		{
			return t;
		}

		if (t->hThread == NULL)
		{
			if (unused == NULL)
			{
				unused = t;
			}
		}
		else if (oldest == NULL || s_nRF91ModemTakt.EvtCnt - t->Cnt > oldestage)
		{
			oldest = t;
			oldestage = s_nRF91ModemTakt.EvtCnt - t->Cnt;
		}
	}

	nRF91ModemTaktThread_t *entry = unused != NULL ? unused : oldest;

	// A thread not seen before may have missed an event: it does not sleep
	// on its first wait.
	entry->hThread = hThread;
	entry->Cnt = s_nRF91ModemTakt.EvtCnt - 1;

	return entry;
}

int32_t nrf_modem_os_timedwait(uint32_t context, int32_t *timeout)
{
	if (!nrf_modem_is_initialized())
	{
		return -NRF_ESHUTDOWN;
	}

	if (*timeout == 0)
	{
		return -NRF_EAGAIN;
	}

	hTaktOSThread_t self = TaktOSCurrentThread();
	nRF91ModemTaktWait_t *wait = NULL;
	uint32_t state = DisableInterrupt();
	nRF91ModemTaktThread_t *thread = nRF91ModemTaktThread(self);

	// An event since this thread last checked: it checks again first
	if (thread->Cnt != s_nRF91ModemTakt.EvtCnt)
	{
		thread->Cnt = s_nRF91ModemTakt.EvtCnt;
		EnableInterrupt(state);

		return 0;
	}

	for (int i = 0; i < NRF91_MODEM_TAKTOS_WAIT_MAX; i++)
	{
		if (!s_nRF91ModemTakt.Wait[i].bUsed)
		{
			wait = &s_nRF91ModemTakt.Wait[i];
			wait->bUsed = true;
			wait->Context = context;
			(void)TaktOSSemInit(&wait->Sem, 0, 1);
			break;
		}
	}
	EnableInterrupt(state);

	uint32_t start = TaktOSTickCount();

	if (wait != NULL)
	{
		(void)TaktOSSemTake(&wait->Sem, true, nRF91ModemTaktTicks(*timeout));
	}
	else
	{
		// More waiters than slots: sleep at least 1 ms, counted against the
		// timeout, and let the library check again
		(void)TaktOSThreadSleep(self, 1);
	}

	uint32_t elapsed = TaktOSTickCount() - start;

	state = DisableInterrupt();
	if (wait != NULL)
	{
		wait->bUsed = false;
	}
	thread = nRF91ModemTaktThread(self);
	thread->Cnt = s_nRF91ModemTakt.EvtCnt;
	EnableInterrupt(state);

	if (!nrf_modem_is_initialized())
	{
		return -NRF_ESHUTDOWN;
	}

	if (*timeout < 0)
	{
		return 0;
	}

	uint64_t elapsedms = ((uint64_t)elapsed * 1000U) / TaktOSGetTickHz();

	*timeout = elapsedms >= (uint64_t)*timeout ? 0 : *timeout - (int32_t)elapsedms;

	return *timeout == 0 ? -NRF_EAGAIN : 0;
}

void nrf_modem_os_event_notify(uint32_t context)
{
	uint32_t state = DisableInterrupt();

	s_nRF91ModemTakt.EvtCnt++;

	for (int i = 0; i < NRF91_MODEM_TAKTOS_WAIT_MAX; i++)
	{
		nRF91ModemTaktWait_t *w = &s_nRF91ModemTakt.Wait[i];

		if (w->bUsed && (context == 0 || w->Context == 0 || w->Context == context))
		{
			(void)TaktOSSemGive(&w->Sem, false);
		}
	}

	EnableInterrupt(state);
}

int nrf_modem_os_sleep(uint32_t timeout)
{
	if (timeout == 0)
	{
		return 0;
	}

	return TaktOSThreadSleep(TaktOSCurrentThread(), timeout) == TAKTOS_OK ? 0 : -NRF_EAGAIN;
}

int nrf_modem_os_sem_init(void **sem, unsigned int initial_count, unsigned int limit)
{
	TaktOSSem_t *s = (TaktOSSem_t *)*sem;

	if (s < &s_nRF91ModemTakt.Sem[0] || s >= &s_nRF91ModemTakt.Sem[NRF_MODEM_OS_NUM_SEM_REQUIRED])
	{
		if (s_nRF91ModemTakt.NbSem >= NRF_MODEM_OS_NUM_SEM_REQUIRED)
		{
			return -NRF_ENOMEM;
		}
		s = &s_nRF91ModemTakt.Sem[s_nRF91ModemTakt.NbSem++];
		*sem = s;
	}

	return TaktOSSemInit(s, initial_count, limit) == TAKTOS_OK ? 0 : -NRF_EINVAL;
}

void nrf_modem_os_sem_give(void *sem)
{
	(void)TaktOSSemGive((TaktOSSem_t *)sem, false);
}

int nrf_modem_os_sem_take(void *sem, int timeout)
{
	bool block = timeout != NRF_MODEM_OS_NO_WAIT && !nrf_modem_os_is_in_isr();

	return TaktOSSemTake((TaktOSSem_t *)sem, block, nRF91ModemTaktTicks(timeout)) == TAKTOS_OK ?
		   0 : -NRF_EAGAIN;
}

unsigned int nrf_modem_os_sem_count_get(void *sem)
{
	return ((TaktOSSem_t *)sem)->Count;
}

int nrf_modem_os_mutex_init(void **mutex)
{
	nRF91ModemTaktMutex_t *m = (nRF91ModemTaktMutex_t *)*mutex;

	if (m < &s_nRF91ModemTakt.Mutex[0] || m >= &s_nRF91ModemTakt.Mutex[NRF_MODEM_OS_NUM_MUTEX_REQUIRED])
	{
		if (s_nRF91ModemTakt.NbMutex >= NRF_MODEM_OS_NUM_MUTEX_REQUIRED)
		{
			return -NRF_ENOMEM;
		}
		m = &s_nRF91ModemTakt.Mutex[s_nRF91ModemTakt.NbMutex++];
		*mutex = m;
	}

	m->hOwner = NULL;
	m->Depth = 0;

	return TaktOSMutexInit(&m->Mutex) == TAKTOS_OK ? 0 : -NRF_EINVAL;
}

int nrf_modem_os_mutex_lock(void *mutex, int timeout)
{
	nRF91ModemTaktMutex_t *m = (nRF91ModemTaktMutex_t *)mutex;
	hTaktOSThread_t self = TaktOSCurrentThread();

	if (nrf_modem_os_is_in_isr())
	{
		return -NRF_EAGAIN;
	}

	// Only the owner reads its own ownership here, nobody else can change it
	if (m->Depth > 0 && m->hOwner == self)
	{
		m->Depth++;

		return 0;
	}

	if (TaktOSMutexLock(&m->Mutex, timeout != NRF_MODEM_OS_NO_WAIT,
						nRF91ModemTaktTicks(timeout)) != TAKTOS_OK)
	{
		return -NRF_EAGAIN;
	}

	m->hOwner = self;
	m->Depth = 1;

	return 0;
}

int nrf_modem_os_mutex_unlock(void *mutex)
{
	nRF91ModemTaktMutex_t *m = (nRF91ModemTaktMutex_t *)mutex;

	if (m->Depth == 0)
	{
		return -NRF_EINVAL;
	}

	if (m->hOwner != TaktOSCurrentThread())
	{
		return -NRF_EPERM;
	}

	if (--m->Depth == 0)
	{
		m->hOwner = NULL;
		(void)TaktOSMutexUnlock(&m->Mutex);
	}

	return 0;
}

/** @} End of group LTE */

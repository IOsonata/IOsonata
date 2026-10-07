/**-------------------------------------------------------------------------
@file	modem_os_nrf91.c

@brief	nRF91 Modem library glue, bare metal

The nrf_modem_os functions the Modem library calls, the IPC interrupt and
nRF91ModemInit. See modem_nrf91.h.

Memory, errno, logging and the interrupt are the same with or without an
RTOS. The waits, the semaphores and the mutex here are the bare metal ones.
They are weak: modem_os_nrf91_taktos.c replaces all of them for TaktOS.

Bare metal there is one waiting context, the main loop. Interrupts give
semaphores and notify events, and every give or notify sets the event
register, so a wait that checked its condition just before the interrupt
does not sleep through it. A semaphore take or a mutex lock from an
interrupt does not wait.

The library heap and the TX area are first fit allocators over the memory of
the configuration, protected by masking interrupts. Each block has an 8 byte
header holding its size, bit 0 set while allocated. Free blocks are merged
with the free blocks after them when an allocation walks over them.

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
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <stdatomic.h>

#include "nrf.h"
#include "nrf_peripherals.h"

#include "nrf_modem.h"
#include "nrf_modem_os.h"
#include "nrf_errno.h"
#include "nrfx_ipc.h"
#include "idelay.h"
#include "coredev/interrupt.h"
#include "coredev/timer.h"
#include "syslog.h"
#include "modem_nrf91.h"

// The log goes to a SysLog only when the application has one. Weak, so that
// the log functions do not bring SysLog into an application without it.
#pragma weak SysLogPrintf
#pragma weak SysLogVPrintf

/** @addtogroup LTE
  * @{
  */

// Allocation granule, also the block header size, so that every block is 8
// byte aligned
#define NRF91_MODEM_ALLOC_UNIT			8U
#define NRF91_MODEM_ALLOC_USED			1U

// Control area size. The nRF9160 has no DECT NR+ firmware, the other nRF91
// take the larger of the two families.
#ifdef NRF9160_XXAA
#define NRF91_MODEM_SHM_CTRL_MAXSIZE	NRF_MODEM_CELLULAR_SHMEM_CTRL_SIZE
#else
#define NRF91_MODEM_SHM_CTRL_MAXSIZE	(NRF_MODEM_DECT_SHMEM_CTRL_SIZE > NRF_MODEM_CELLULAR_SHMEM_CTRL_SIZE ? \
										 NRF_MODEM_DECT_SHMEM_CTRL_SIZE : NRF_MODEM_CELLULAR_SHMEM_CTRL_SIZE)
#endif

// Bytes of a hex dump line of the log
#define NRF91_MODEM_LOG_DUMP_LINE		16

// Start of RAM, where SPU RAM region 0 is
#define NRF91_MODEM_RAM_START			0x20000000UL

// Longest single timer wait, within the counter range of the timer clocks
#define NRF91_MODEM_BM_PAUSE_MAX_MS		60000

#pragma pack(push, 4)

typedef struct {
	uint8_t *pStart;
	uint8_t *pEnd;
} nRF91ModemArena_t;

typedef struct {
	atomic_uint Count;
	unsigned int Limit;
} nRF91ModemSem_t;

typedef struct {
	uint32_t Owner;					// IPSR of the holder, 0 for the main loop
	uint32_t Count;					// Lock depth, 0 when free
} nRF91ModemMutex_t;

// Glue state, with or without an RTOS
typedef struct {
	const nRF91ModemCfg_t *pCfg;
	struct nrf_modem_init_params Params;
	uint32_t HeapSize;
	uint32_t TxSize;
	nRF91ModemArena_t Heap;
	nRF91ModemArena_t Tx;
	bool bFault;
	struct nrf_modem_fault_info Fault;
	uint32_t DfuResult;
} nRF91ModemOs_t;

// Bare metal wait state
typedef struct {
	uint32_t BusyMs;				// Time counted by the busy steps, no timer
	atomic_uint EvtCnt;				// Event notifications so far
	uint32_t SeenCnt;				// EvtCnt the waiting context has seen
	volatile uint32_t WaitCtx;		// Context of the waiting context
	volatile bool bWaiting;
	volatile bool bWoken;
	int NbSem;
	nRF91ModemSem_t Sem[NRF_MODEM_OS_NUM_SEM_REQUIRED];
	int NbMutex;
	nRF91ModemMutex_t Mutex[NRF_MODEM_OS_NUM_MUTEX_REQUIRED];
} nRF91ModemOsBm_t;

#pragma pack(pop)

// Shared memory and heap with the default sizes, replaced by the application
// definitions of the same names when it has its own, see modem_nrf91.h. Each
// in its own section, so that the linker drops a replaced one.
__attribute__((weak, section(".modem_shm.tx"), aligned(8)))
uint8_t g_nRF91ModemShmTx[NRF91_MODEM_SHM_TX_DEFAULT_SIZE];
__attribute__((weak, section(".modem_shm.rx"), aligned(8)))
uint8_t g_nRF91ModemShmRx[NRF91_MODEM_SHM_RX_DEFAULT_SIZE];
__attribute__((weak, aligned(8))) uint8_t g_nRF91ModemHeap[NRF91_MODEM_HEAP_DEFAULT_SIZE];

// Control area, sized for the larger of the two firmware families
__attribute__((section(".modem_shm.ctrl"), aligned(8)))
static uint8_t s_nRF91ModemShmCtrl[NRF91_MODEM_SHM_CTRL_MAXSIZE];

// The .modem_shm section: first in RAM, in whole SPU RAM regions below
// 128 KB (nrf9160_xxaa.ld, nrf9120_xxaa.ld)
extern const uint8_t __start_modem_shm[];
extern const uint8_t __stop_modem_shm[];

static const char s_nRF91ModemHex[] = "0123456789ABCDEF";

static nRF91ModemOs_t s_nRF91ModemOs;

static nRF91ModemOsBm_t s_nRF91ModemOsBm;

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

static void nRF91ModemFaultCb(struct nrf_modem_fault_info *pFaultInfo)
{
	s_nRF91ModemOs.Fault = *pFaultInfo;
	s_nRF91ModemOs.bFault = true;

	if (s_nRF91ModemOs.pCfg != NULL && s_nRF91ModemOs.pCfg->FaultHandler != NULL)
	{
		s_nRF91ModemOs.pCfg->FaultHandler(pFaultInfo);
	}
}

static void nRF91ModemDfuCb(uint32_t DfuResult)
{
	s_nRF91ModemOs.DfuResult = DfuResult;

	if (s_nRF91ModemOs.pCfg != NULL && s_nRF91ModemOs.pCfg->DfuHandler != NULL)
	{
		s_nRF91ModemOs.pCfg->DfuHandler(DfuResult);
	}
}

// True when Size bytes at pMem are not word aligned memory of the
// .modem_shm section
static bool nRF91ModemShmOut(const uint8_t *pMem, uint32_t Size)
{
	uintptr_t addr = (uintptr_t)pMem;

	return (addr & 3U) != 0 || addr < (uintptr_t)__start_modem_shm ||
		   addr > (uintptr_t)__stop_modem_shm ||
		   Size > (uintptr_t)__stop_modem_shm - addr;
}

// The application stays secure. Only what the modem side uses becomes non
// secure in the SPU: the RAM regions of the .modem_shm section, as the modem
// accesses are always non secure, and IPC, which the Modem library uses at
// its non secure address. The IPC interrupt stays secure.
static void nRF91ModemSpuOpen(void)
{
	NRF_SPU_Type * const spu = NRF_SPU_S;

	for (uintptr_t a = (uintptr_t)__start_modem_shm; a < (uintptr_t)__stop_modem_shm;
		 a += SPU_RAMREGION_SIZE)
	{
		const uintptr_t n = (a - NRF91_MODEM_RAM_START) / SPU_RAMREGION_SIZE;

		if (n < sizeof(spu->RAMREGION) / sizeof(spu->RAMREGION[0]))
		{
			spu->RAMREGION[n].PERM &= ~SPU_RAMREGION_PERM_SECATTR_Msk;
		}
	}

	spu->PERIPHID[IPC_IRQn].PERM &= ~SPU_PERIPHID_PERM_SECATTR_Msk;
}

int nRF91ModemInit(const nRF91ModemCfg_t * const pCfg)
{
	if (pCfg == NULL ||
		(pCfg->Fw != NRF91_MODEM_FW_CELLULAR && pCfg->Fw != NRF91_MODEM_FW_DECT))
	{
		return -NRF_EINVAL;
	}

	if (pCfg->pTimer != NULL)
	{
		TimerDev_t * const timer = pCfg->pTimer;

		// A timer that did not initialize has no functions
		if (timer->GetTickCount == NULL || timer->GetMaxTrigger == NULL ||
			timer->EnableTrigger == NULL || timer->DisableTrigger == NULL ||
			pCfg->TimerTrigNo < 0 || pCfg->TimerTrigNo >= timer->GetMaxTrigger(timer))
		{
			return -NRF_EINVAL;
		}
	}

	// The library keeps the parameters: leave them alone while it runs
	if (nrf_modem_is_initialized())
	{
		return -NRF_EPERM;
	}

	uint32_t ctrlsize = pCfg->Fw == NRF91_MODEM_FW_CELLULAR ?
						NRF_MODEM_CELLULAR_SHMEM_CTRL_SIZE : NRF_MODEM_DECT_SHMEM_CTRL_SIZE;
	uint32_t txsize = pCfg->ShmTxSize != 0 ? pCfg->ShmTxSize : NRF91_MODEM_SHM_TX_DEFAULT_SIZE;
	uint32_t rxsize = pCfg->ShmRxSize != 0 ? pCfg->ShmRxSize : NRF91_MODEM_SHM_RX_DEFAULT_SIZE;
	uint32_t tracesize = pCfg->pShmTrace != NULL ? pCfg->ShmTraceSize : 0;

	if (ctrlsize > sizeof(s_nRF91ModemShmCtrl) ||
		nRF91ModemShmOut(s_nRF91ModemShmCtrl, ctrlsize) ||
		nRF91ModemShmOut(g_nRF91ModemShmTx, txsize) ||
		nRF91ModemShmOut(g_nRF91ModemShmRx, rxsize) ||
		(tracesize != 0 && nRF91ModemShmOut(pCfg->pShmTrace, tracesize)))
	{
		return -NRF_EINVAL;
	}

	nRF91ModemSpuOpen();

	s_nRF91ModemOs.pCfg = pCfg;
	s_nRF91ModemOs.TxSize = txsize;
	s_nRF91ModemOs.HeapSize = pCfg->HeapSize != 0 ? pCfg->HeapSize : NRF91_MODEM_HEAP_DEFAULT_SIZE;
	s_nRF91ModemOs.bFault = false;
	s_nRF91ModemOs.DfuResult = 0;

	memset(s_nRF91ModemShmCtrl, 0, ctrlsize);

	s_nRF91ModemOs.Params.shmem.ctrl.base = (uint32_t)(uintptr_t)s_nRF91ModemShmCtrl;
	s_nRF91ModemOs.Params.shmem.ctrl.size = ctrlsize;
	s_nRF91ModemOs.Params.shmem.tx.base = (uint32_t)(uintptr_t)g_nRF91ModemShmTx;
	s_nRF91ModemOs.Params.shmem.tx.size = txsize;
	s_nRF91ModemOs.Params.shmem.rx.base = (uint32_t)(uintptr_t)g_nRF91ModemShmRx;
	s_nRF91ModemOs.Params.shmem.rx.size = rxsize;
	s_nRF91ModemOs.Params.shmem.trace.base = (uint32_t)(uintptr_t)pCfg->pShmTrace;
	s_nRF91ModemOs.Params.shmem.trace.size = tracesize;
	s_nRF91ModemOs.Params.ipc_irq_prio = (uint32_t)pCfg->IntPrio;
	s_nRF91ModemOs.Params.fault_handler = nRF91ModemFaultCb;
	s_nRF91ModemOs.Params.dfu_handler = nRF91ModemDfuCb;

	return nrf_modem_init(&s_nRF91ModemOs.Params);
}

bool nRF91ModemFault(struct nrf_modem_fault_info * const pInfo)
{
	if (s_nRF91ModemOs.bFault && pInfo != NULL)
	{
		*pInfo = s_nRF91ModemOs.Fault;
	}

	return s_nRF91ModemOs.bFault;
}

uint32_t nRF91ModemDfuResult(void)
{
	return s_nRF91ModemOs.DfuResult;
}

void IPC_IRQHandler(void)
{
	nrfx_ipc_irq_handler();
}

// ---------------------------------------------------------------------------
// Memory
// ---------------------------------------------------------------------------

static void nRF91ModemArenaInit(nRF91ModemArena_t * const pArena, uint8_t *pMem, uint32_t Size)
{
	uintptr_t start = ((uintptr_t)pMem + NRF91_MODEM_ALLOC_UNIT - 1) & ~(uintptr_t)(NRF91_MODEM_ALLOC_UNIT - 1);
	uintptr_t end = ((uintptr_t)pMem + Size) & ~(uintptr_t)(NRF91_MODEM_ALLOC_UNIT - 1);

	if (pMem == NULL || end < start + 2 * NRF91_MODEM_ALLOC_UNIT)
	{
		pArena->pStart = NULL;
		pArena->pEnd = NULL;

		return;
	}

	pArena->pStart = (uint8_t *)start;
	pArena->pEnd = (uint8_t *)end;
	*(uint32_t *)start = (uint32_t)(end - start);
}

static void *nRF91ModemArenaAlloc(nRF91ModemArena_t * const pArena, size_t Bytes)
{
	if (Bytes == 0 || pArena->pStart == NULL ||
		Bytes > (size_t)(pArena->pEnd - pArena->pStart))
	{
		return NULL;
	}

	uint32_t need = ((uint32_t)Bytes + 2 * NRF91_MODEM_ALLOC_UNIT - 1) & ~(NRF91_MODEM_ALLOC_UNIT - 1);
	uint32_t state = DisableInterrupt();
	uint8_t *p = pArena->pStart;

	while (p < pArena->pEnd)
	{
		uint32_t size = *(uint32_t *)p & ~(NRF91_MODEM_ALLOC_UNIT - 1);

		if (size == 0)
		{
			// Corrupted header, nothing safe to hand out
			break;
		}

		if ((*(uint32_t *)p & NRF91_MODEM_ALLOC_USED) == 0)
		{
			uint8_t *next = p + size;

			while (next < pArena->pEnd && (*(uint32_t *)next & NRF91_MODEM_ALLOC_USED) == 0)
			{
				uint32_t nsize = *(uint32_t *)next & ~(NRF91_MODEM_ALLOC_UNIT - 1);

				if (nsize == 0)
				{
					break;
				}
				size += nsize;
				next = p + size;
			}

			if (size >= need)
			{
				if (size - need >= 2 * NRF91_MODEM_ALLOC_UNIT)
				{
					*(uint32_t *)(p + need) = size - need;
					size = need;
				}
				*(uint32_t *)p = size | NRF91_MODEM_ALLOC_USED;
				EnableInterrupt(state);

				return p + NRF91_MODEM_ALLOC_UNIT;
			}

			*(uint32_t *)p = size;
		}

		p += size;
	}

	EnableInterrupt(state);

	return NULL;
}

static void nRF91ModemArenaFree(nRF91ModemArena_t * const pArena, void *pMem)
{
	if (pMem == NULL)
	{
		return;
	}

	uint8_t *p = (uint8_t *)pMem - NRF91_MODEM_ALLOC_UNIT;

	if (p < pArena->pStart || p >= pArena->pEnd)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	*(uint32_t *)p &= ~NRF91_MODEM_ALLOC_USED;

	EnableInterrupt(state);
}

void nrf_modem_os_init(void)
{
	nRF91ModemArenaInit(&s_nRF91ModemOs.Heap, g_nRF91ModemHeap, s_nRF91ModemOs.HeapSize);
	nRF91ModemArenaInit(&s_nRF91ModemOs.Tx, g_nRF91ModemShmTx, s_nRF91ModemOs.TxSize);
}

void nrf_modem_os_shutdown(void)
{
	// Every pending wait ends
	nrf_modem_os_event_notify(0);
}

void *nrf_modem_os_alloc(size_t bytes)
{
	return nRF91ModemArenaAlloc(&s_nRF91ModemOs.Heap, bytes);
}

void nrf_modem_os_free(void *mem)
{
	nRF91ModemArenaFree(&s_nRF91ModemOs.Heap, mem);
}

void *nrf_modem_os_shm_tx_alloc(size_t bytes)
{
	return nRF91ModemArenaAlloc(&s_nRF91ModemOs.Tx, bytes);
}

void nrf_modem_os_shm_tx_free(void *mem)
{
	nRF91ModemArenaFree(&s_nRF91ModemOs.Tx, mem);
}

// ---------------------------------------------------------------------------
// Context, errno and log
// ---------------------------------------------------------------------------

void nrf_modem_os_busywait(int32_t usec)
{
	if (usec > 0)
	{
		usDelay((uint32_t)usec);
	}
}

// The library errno values (nrf_errno.h) are kept as they are, the way the
// nRF Connect SDK does, so application code compares errno with NRF_ values.
void nrf_modem_os_errno_set(int errno_val)
{
	errno = errno_val;
}

bool nrf_modem_os_is_in_isr(void)
{
	return __get_IPSR() != 0;
}

void nrf_modem_os_log(int level, const char *fmt, ...)
{
	(void)level;

	if (s_nRF91ModemOs.pCfg == NULL || s_nRF91ModemOs.pCfg->pLog == NULL ||
		SysLogVPrintf == NULL)
	{
		return;
	}

	va_list args;

	va_start(args, fmt);
	(void)SysLogVPrintf(s_nRF91ModemOs.pCfg->pLog, fmt, args);
	va_end(args);
}

void nrf_modem_os_logdump(int level, const char *str, const void *data, size_t len)
{
	(void)level;

	if (s_nRF91ModemOs.pCfg == NULL || s_nRF91ModemOs.pCfg->pLog == NULL ||
		SysLogPrintf == NULL)
	{
		return;
	}

	SysLog_t *log = s_nRF91ModemOs.pCfg->pLog;
	const uint8_t *p = (const uint8_t *)data;
	char line[NRF91_MODEM_LOG_DUMP_LINE * 3 + 1];

	(void)SysLogPrintf(log, "%s\r\n", str);

	for (size_t i = 0; i < len; i += NRF91_MODEM_LOG_DUMP_LINE)
	{
		size_t n = len - i < NRF91_MODEM_LOG_DUMP_LINE ? len - i : NRF91_MODEM_LOG_DUMP_LINE;
		int k = 0;

		for (size_t j = 0; j < n; j++)
		{
			line[k++] = s_nRF91ModemHex[p[i + j] >> 4];
			line[k++] = s_nRF91ModemHex[p[i + j] & 0xF];
			line[k++] = ' ';
		}
		line[k] = 0;
		(void)SysLogPrintf(log, "%s\r\n", line);
	}
}

// ---------------------------------------------------------------------------
// Bare metal waits, semaphores and mutex. Weak, replaced for an RTOS.
// ---------------------------------------------------------------------------

static TimerDev_t *nRF91ModemBmTimer(void)
{
	return s_nRF91ModemOs.pCfg != NULL ? s_nRF91ModemOs.pCfg->pTimer : NULL;
}

static uint32_t nRF91ModemBmNow(void)
{
	TimerDev_t *timer = nRF91ModemBmTimer();

	return timer != NULL ? TimerGetMilisecond(timer) : s_nRF91ModemOsBm.BusyMs;
}

// Time left of a wait started at Start, -1 without timeout
static int32_t nRF91ModemBmRemain(uint32_t Start, int32_t Timeout)
{
	if (Timeout < 0)
	{
		return -1;
	}

	uint32_t elapsed = nRF91ModemBmNow() - Start;

	return elapsed >= (uint32_t)Timeout ? 0 : Timeout - (int32_t)elapsed;
}

// Wait for the next interrupt, at most Remain ms, -1 for no limit
static void nRF91ModemBmPause(int32_t Remain)
{
	TimerDev_t *timer = nRF91ModemBmTimer();

	if (Remain < 0)
	{
		__WFE();
	}
	else if (timer != NULL)
	{
		int trig = s_nRF91ModemOs.pCfg->TimerTrigNo;
		uint32_t ms = Remain > NRF91_MODEM_BM_PAUSE_MAX_MS ? NRF91_MODEM_BM_PAUSE_MAX_MS : (uint32_t)Remain;

		if (timer->EnableTrigger(timer, trig, (uint64_t)ms * 1000000ULL,
								 TIMER_TRIG_TYPE_SINGLE, NULL, NULL) != 0)
		{
			__WFE();
			timer->DisableTrigger(timer, trig);
		}
		else
		{
			// Trigger refused: a busy step, the timer still counts the time
			usDelay(1000);
		}
	}
	else
	{
		usDelay(1000);
		s_nRF91ModemOsBm.BusyMs++;
	}
}

__attribute__((weak)) int32_t nrf_modem_os_timedwait(uint32_t context, int32_t *timeout)
{
	if (!nrf_modem_is_initialized())
	{
		return -NRF_ESHUTDOWN;
	}

	if (*timeout == 0)
	{
		return -NRF_EAGAIN;
	}

	uint32_t state = DisableInterrupt();
	uint32_t cnt = atomic_load(&s_nRF91ModemOsBm.EvtCnt);

	// An event since the last wait: the library checks again before sleeping
	if (cnt != s_nRF91ModemOsBm.SeenCnt)
	{
		s_nRF91ModemOsBm.SeenCnt = cnt;
		EnableInterrupt(state);

		return 0;
	}

	s_nRF91ModemOsBm.WaitCtx = context;
	s_nRF91ModemOsBm.bWoken = false;
	s_nRF91ModemOsBm.bWaiting = true;
	EnableInterrupt(state);

	uint32_t start = nRF91ModemBmNow();
	int32_t remain = nRF91ModemBmRemain(start, *timeout);

	while (!s_nRF91ModemOsBm.bWoken && remain != 0)
	{
		nRF91ModemBmPause(remain);
		remain = nRF91ModemBmRemain(start, *timeout);
	}

	state = DisableInterrupt();
	s_nRF91ModemOsBm.bWaiting = false;
	s_nRF91ModemOsBm.SeenCnt = atomic_load(&s_nRF91ModemOsBm.EvtCnt);
	EnableInterrupt(state);

	if (!nrf_modem_is_initialized())
	{
		return -NRF_ESHUTDOWN;
	}

	if (*timeout < 0)
	{
		return 0;
	}

	*timeout = remain;

	return remain == 0 ? -NRF_EAGAIN : 0;
}

__attribute__((weak)) void nrf_modem_os_event_notify(uint32_t context)
{
	atomic_fetch_add(&s_nRF91ModemOsBm.EvtCnt, 1);

	if (s_nRF91ModemOsBm.bWaiting &&
		(context == 0 || s_nRF91ModemOsBm.WaitCtx == 0 || s_nRF91ModemOsBm.WaitCtx == context))
	{
		s_nRF91ModemOsBm.bWoken = true;
	}

	__SEV();
}

__attribute__((weak)) int nrf_modem_os_sleep(uint32_t timeout)
{
	// Kept in the signed range of the wait time
	int32_t ms = timeout > (uint32_t)INT32_MAX ? INT32_MAX : (int32_t)timeout;
	uint32_t start = nRF91ModemBmNow();
	int32_t remain = nRF91ModemBmRemain(start, ms);

	while (remain != 0)
	{
		nRF91ModemBmPause(remain);
		remain = nRF91ModemBmRemain(start, ms);
	}

	return 0;
}

__attribute__((weak)) int nrf_modem_os_sem_init(void **sem, unsigned int initial_count, unsigned int limit)
{
	nRF91ModemSem_t *s = (nRF91ModemSem_t *)*sem;

	if (s < &s_nRF91ModemOsBm.Sem[0] || s >= &s_nRF91ModemOsBm.Sem[NRF_MODEM_OS_NUM_SEM_REQUIRED])
	{
		if (s_nRF91ModemOsBm.NbSem >= NRF_MODEM_OS_NUM_SEM_REQUIRED)
		{
			return -NRF_ENOMEM;
		}
		s = &s_nRF91ModemOsBm.Sem[s_nRF91ModemOsBm.NbSem++];
		*sem = s;
	}

	s->Limit = limit;
	atomic_store(&s->Count, initial_count < limit ? initial_count : limit);

	return 0;
}

__attribute__((weak)) void nrf_modem_os_sem_give(void *sem)
{
	nRF91ModemSem_t *s = (nRF91ModemSem_t *)sem;
	unsigned int cnt = atomic_load(&s->Count);

	while (cnt < s->Limit && !atomic_compare_exchange_weak(&s->Count, &cnt, cnt + 1))
	{
	}

	__SEV();
}

__attribute__((weak)) int nrf_modem_os_sem_take(void *sem, int timeout)
{
	nRF91ModemSem_t *s = (nRF91ModemSem_t *)sem;
	uint32_t start = nRF91ModemBmNow();

	while (1)
	{
		unsigned int cnt = atomic_load(&s->Count);

		while (cnt > 0)
		{
			if (atomic_compare_exchange_weak(&s->Count, &cnt, cnt - 1))
			{
				return 0;
			}
		}

		if (timeout == NRF_MODEM_OS_NO_WAIT || nrf_modem_os_is_in_isr())
		{
			return -NRF_EAGAIN;
		}

		int32_t remain = nRF91ModemBmRemain(start, timeout);

		if (remain == 0)
		{
			return -NRF_EAGAIN;
		}

		nRF91ModemBmPause(remain);
	}
}

__attribute__((weak)) unsigned int nrf_modem_os_sem_count_get(void *sem)
{
	return atomic_load(&((nRF91ModemSem_t *)sem)->Count);
}

__attribute__((weak)) int nrf_modem_os_mutex_init(void **mutex)
{
	nRF91ModemMutex_t *m = (nRF91ModemMutex_t *)*mutex;

	if (m < &s_nRF91ModemOsBm.Mutex[0] || m >= &s_nRF91ModemOsBm.Mutex[NRF_MODEM_OS_NUM_MUTEX_REQUIRED])
	{
		if (s_nRF91ModemOsBm.NbMutex >= NRF_MODEM_OS_NUM_MUTEX_REQUIRED)
		{
			return -NRF_ENOMEM;
		}
		m = &s_nRF91ModemOsBm.Mutex[s_nRF91ModemOsBm.NbMutex++];
		*mutex = m;
	}

	m->Owner = 0;
	m->Count = 0;

	return 0;
}

__attribute__((weak)) int nrf_modem_os_mutex_lock(void *mutex, int timeout)
{
	nRF91ModemMutex_t *m = (nRF91ModemMutex_t *)mutex;
	uint32_t owner = __get_IPSR();
	uint32_t start = nRF91ModemBmNow();

	while (1)
	{
		uint32_t state = DisableInterrupt();

		if (m->Count == 0 || m->Owner == owner)
		{
			m->Owner = owner;
			m->Count++;
			EnableInterrupt(state);

			return 0;
		}
		EnableInterrupt(state);

		if (timeout == NRF_MODEM_OS_NO_WAIT || owner != 0)
		{
			return -NRF_EAGAIN;
		}

		int32_t remain = nRF91ModemBmRemain(start, timeout);

		if (remain == 0)
		{
			return -NRF_EAGAIN;
		}

		nRF91ModemBmPause(remain);
	}
}

__attribute__((weak)) int nrf_modem_os_mutex_unlock(void *mutex)
{
	nRF91ModemMutex_t *m = (nRF91ModemMutex_t *)mutex;
	uint32_t state = DisableInterrupt();
	int res = 0;

	if (m->Count == 0)
	{
		res = -NRF_EINVAL;
	}
	else if (m->Owner != __get_IPSR())
	{
		res = -NRF_EPERM;
	}
	else
	{
		m->Count--;
	}
	EnableInterrupt(state);

	__SEV();

	return res;
}

/** @} End of group LTE */

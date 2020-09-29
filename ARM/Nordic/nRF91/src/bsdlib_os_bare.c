/**-------------------------------------------------------------------------
@file	bsdlib_os_bare.c

@brief	bare metal port for nrf bsdlib


@author	Hoang Nguyen Hoan
@date	Sep. 26, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_errno.h"

#include "bsd_platform.h"
#include "bsd.h"

#include "idelay.h"
#include "bsdlib_os_bare.h"

extern void bsd_os_application_irq_handler(void);
extern void bsd_os_trace_irq_handler(void);

#ifndef ENOKEY
#define ENOKEY 2001
#endif

#ifndef EKEYEXPIRED
#define EKEYEXPIRED 2002
#endif

#ifndef EKEYREVOKED
#define EKEYREVOKED 2003
#endif

#ifndef EKEYREJECTED
#define EKEYREJECTED 2004
#endif

#define UNUSED_FLAGS 0

/* Handle modem traces from IRQ context with lower priority. */
#define TRACE_IRQ 			EGU2_IRQn
#define TRACE_IRQ_PRIORITY 	6

#define POWER_ID	5
#define IPC_ID		42
#define NS_RAMREGION_START		(BSD_RESERVED_MEMORY_ADDRESS / 8192U)
#define NS_RAMREGION_COUNT		((BSD_RESERVED_MEMORY_SIZE + 0x1FFF) / 8192U)


static UARTDEV *s_pUartDev = NULL;

__attribute__ ((section(".bsdlibData"), used))
uint8_t g_BsdResevedRam[BSD_RESERVED_MEMORY_SIZE];

int bsdlid_init(UARTDEV * const pUartDev, bool Trace)
{
	// Configure non secure peruph & ram for BSDLIB

	uint32_t r = NRF_SPU_S->PERIPHID[POWER_ID].PERM;
	r &= ~(0x13);
	NRF_SPU_S->PERIPHID[POWER_ID].PERM = r;
	r = NRF_SPU_S->PERIPHID[IPC_ID].PERM;
	r &= ~(0x13);
	NRF_SPU_S->PERIPHID[IPC_ID].PERM = r;

	for (int i = 0; i < NS_RAMREGION_COUNT; i++)
	{
		NRF_SPU_S->RAMREGION[i + NS_RAMREGION_START].PERM = 0x7;
	}

	NVIC_SetPriority(BSD_NETWORK_IRQ, BSD_NETWORK_IRQ_PRIORITY);

	const bsd_init_params_t init_params = {
		.trace_on = false,
		.bsd_memory_address = BSD_RESERVED_MEMORY_ADDRESS,
		.bsd_memory_size = BSD_RESERVED_MEMORY_SIZE
	};

	s_pUartDev = pUartDev;

	return bsd_init(&init_params);
}

/* @brief Put a thread to a sleep for a specific time or until an event occurs.
 *
 * @param[in]      context   A unique identifier assigned by the library to identify the context.
 * @param[in, out] p_timeout A pointer to the time-out value, in milliseconds. -1 for infinite
 *                           time-out. Contains the time-out value as input, remainig time to sleep
 *                           as output.
 *
 * @retval 0             If the procedure succeeded - it was interrupted by the RPC.
 * @retval NRF_ETIMEDOUT If a time-out condition occured.
 * @retval Other         Some other, OS specific error took place. The error code shall
 *                       belong to the nrf_errno error space.
 */
__WEAK int32_t bsd_os_timedwait(uint32_t context, int32_t *timeout)
{
#if 0
	msDelay(*timeout);
	*timeout = 0;
	return NRF_ETIMEDOUT;
#else
	return 0;
#endif
}

void bsd_os_errno_set(int err_code)
{
	switch (err_code) {
	case NRF_EPERM:
		errno = EPERM;
		break;
	case NRF_ENOENT:
		errno = ENOENT;
		break;
	case NRF_EIO:
		errno = EIO;
		break;
	case NRF_ENOEXEC:
		errno = ENOEXEC;
		break;
	case NRF_EBADF:
		errno = EBADF;
		break;
	case NRF_ENOMEM:
		errno = ENOMEM;
		break;
	case NRF_EACCES:
		errno = EACCES;
		break;
	case NRF_EFAULT:
		errno = EFAULT;
		break;
	case NRF_EINVAL:
		errno = EINVAL;
		break;
	case NRF_EMFILE:
		errno = EMFILE;
		break;
	case NRF_EAGAIN:
		errno = EAGAIN;
		break;
	case NRF_EDOM:
		errno = EDOM;
		break;
	case NRF_EPROTOTYPE:
		errno = EPROTOTYPE;
		break;
	case NRF_ENOPROTOOPT:
		errno = ENOPROTOOPT;
		break;
	case NRF_EPROTONOSUPPORT:
		errno = EPROTONOSUPPORT;
		break;
	case NRF_ESOCKTNOSUPPORT:
		//errno = ESOCKTNOSUPPORT;
		break;
	case NRF_EOPNOTSUPP:
		errno = EOPNOTSUPP;
		break;
	case NRF_EAFNOSUPPORT:
		errno = EAFNOSUPPORT;
		break;
	case NRF_EADDRINUSE:
		errno = EADDRINUSE;
		break;
	case NRF_ENETDOWN:
		errno = ENETDOWN;
		break;
	case NRF_ENETUNREACH:
		errno = ENETUNREACH;
		break;
	case NRF_ENETRESET:
		errno = ENETRESET;
		break;
	case NRF_ECONNRESET:
		errno = ECONNRESET;
		break;
	case NRF_EISCONN:
		errno = EISCONN;
		break;
	case NRF_ENOTCONN:
		errno = ENOTCONN;
		break;
	case NRF_ETIMEDOUT:
		errno = ETIMEDOUT;
		break;
	case NRF_ENOBUFS:
		errno = ENOBUFS;
		break;
	case NRF_EHOSTDOWN:
		errno = EHOSTDOWN;
		break;
	case NRF_EINPROGRESS:
		errno = EINPROGRESS;
		break;
	case NRF_EALREADY:
		errno = EALREADY;
		break;
	case NRF_ECANCELED:
		errno = ECANCELED;
		break;
	case NRF_ENOKEY:
		errno = ENOKEY;
		break;
	case NRF_EKEYEXPIRED:
		errno = EKEYEXPIRED;
		break;
	case NRF_EKEYREVOKED:
		errno = EKEYREVOKED;
		break;
	case NRF_EKEYREJECTED:
		errno = EKEYREJECTED;
		break;
	case NRF_EMSGSIZE:
		errno = EMSGSIZE;
		break;
	default:
		/* Catch untranslated errnos.
		 * Log the untranslated errno and return a magic value
		 * to make sure this situation is clearly distinguishable.
		 */
		//__ASSERT(false, "Untranslated errno %d set by bsdlib!", err_code);
		//LOG_ERR("Untranslated errno %d set by bsdlib!", err_code);
		errno = 0xBAADBAAD;
		break;
	}
}

void bsd_os_application_irq_set(void)
{
	NVIC_SetPendingIRQ(BSD_APPLICATION_IRQ);
}

void bsd_os_application_irq_clear(void)
{
	NVIC_ClearPendingIRQ(BSD_APPLICATION_IRQ);
}

void bsd_os_trace_irq_set(void)
{
	NVIC_SetPendingIRQ(TRACE_IRQ);
}

void bsd_os_trace_irq_clear(void)
{
	NVIC_ClearPendingIRQ(TRACE_IRQ);
}

void EGU1_IRQHandler()
{
	bsd_os_application_irq_handler();
}

void EGU2_IRQHandler()
{
	bsd_os_trace_irq_handler();
}

/* This function is called by bsd_init and must not be called explicitly. */
void bsd_os_init(void)
{
	NVIC_ClearPendingIRQ(BSD_APPLICATION_IRQ);
	NVIC_SetPriority(BSD_APPLICATION_IRQ, 6);
	NVIC_EnableIRQ(BSD_APPLICATION_IRQ);

	NVIC_ClearPendingIRQ(TRACE_IRQ);
	NVIC_SetPriority(TRACE_IRQ, 6);
	NVIC_EnableIRQ(TRACE_IRQ);
}

int32_t bsd_os_trace_put(const uint8_t * const Data, uint32_t Len)
{
	if (s_pUartDev)
	{
		UARTTx(s_pUartDev, (uint8_t*)Data, Len);
	}
	else
	{
		printf(Data);
	}
	return 0;
}

__WEAK void bsd_recoverable_error_handler(uint32_t error)
{
	printf("%x\n", error);
}

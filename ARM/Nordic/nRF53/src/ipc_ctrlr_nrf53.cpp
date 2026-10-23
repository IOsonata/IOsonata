/**-------------------------------------------------------------------------
@file	ipc_ctrlr_nrf53.cpp

@brief	IpcIntrf port for the nRF5340 IPC peripheral.

Each core of the nRF5340 has its own IPC peripheral, and the two share 16
channels. A SEND task of one core raises the channels listed in its SEND_CNF
register. A RECEIVE event of the other core fires when a channel listed in
its RECEIVE_CNF register is raised. This port ties send task n to channel n
and receive event n to channel n, so a TxChan on one core is the RxChan of
the other, and the number of the channel is also the number of the task and
of the event.

The same file serves the application core and the network core. The RAM
region of the application core that holds the shared rings is opened to the
network core here, on the application core only, as a non-secure region the
way the FICR copy region is opened in the system initialization.

Not validated on hardware.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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

#include "nrf.h"
#include "nrf_peripherals.h"

#include "ipc/ipc_intrf.h"

// The application core is built secure and uses the secure alias of the
// peripheral, the network core has the one alias it can reach.
#if defined(NRF_IPC_S) && !defined(NRF_TRUSTZONE_NONSECURE)
#define IPC_REGS		NRF_IPC_S
#else
#define IPC_REGS		NRF_IPC_NS
#endif

#define IPC_RAM_BASE	0x20000000UL

// Receiving interface of each channel
static IpcIntrfDev_t *s_pIpcDev[IPC_CH_NUM] = { nullptr, };

static inline bool IpcChanValid(int Chan)
{
	return Chan >= 0 && Chan < IPC_CH_NUM;
}

// Open the RAM regions that hold the device data and both rings to the
// network core. Only the application core has the SPU.
static void IpcShareRegion(IpcIntrfDev_t * const pDev)
{
#ifdef NRF_SPU_S
	uint32_t start = reinterpret_cast<uint32_t>(pDev->pHdr);
	uint32_t end = start + pDev->MemSize - 1U;

	if (start < IPC_RAM_BASE)
	{
		return;
	}

	uint32_t first = (start - IPC_RAM_BASE) / SPU_RAMREGION_SIZE;
	uint32_t last = (end - IPC_RAM_BASE) / SPU_RAMREGION_SIZE;

	for (uint32_t i = first; i <= last && i < 64U; i++)
	{
		uint32_t perm = NRF_SPU_S->RAMREGION[i].PERM;

		perm &= ~(SPU_RAMREGION_PERM_SECATTR_Msk | SPU_RAMREGION_PERM_EXECUTE_Msk);
		perm |= SPU_RAMREGION_PERM_READ_Msk | SPU_RAMREGION_PERM_WRITE_Msk;
		NRF_SPU_S->RAMREGION[i].PERM = perm;
	}
#else
	(void)pDev;
#endif
}

bool IpcCtrlrInit(IpcIntrfDev_t * const pDev)
{
	if (pDev == nullptr || pDev->pHdr == nullptr ||
		pDev->MemSize < sizeof(IpcIntrfHdr_t) ||
		!IpcChanValid(pDev->TxChan) || !IpcChanValid(pDev->RxChan))
	{
		return false;
	}

	if (s_pIpcDev[pDev->RxChan] != nullptr && s_pIpcDev[pDev->RxChan] != pDev)
	{
		return false;
	}

	IpcShareRegion(pDev);

#if defined(NRF5340_XXAA_APPLICATION)
	// The other end of the link is the network core. It is held off after
	// every reset until this core releases it, and nothing attaches to a
	// core that is not running. Releasing a core that already runs changes
	// nothing.
#if defined(NRF_RESET_S) && !defined(NRF_TRUSTZONE_NONSECURE)
	NRF_RESET_S->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;
#else
	NRF_RESET_NS->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;
#endif
#endif

	IPC_REGS->SEND_CNF[pDev->TxChan] = 1UL << pDev->TxChan;
	IPC_REGS->RECEIVE_CNF[pDev->RxChan] = 1UL << pDev->RxChan;
	IPC_REGS->EVENTS_RECEIVE[pDev->RxChan] = 0UL;

	s_pIpcDev[pDev->RxChan] = pDev;

	NVIC_ClearPendingIRQ(IPC_IRQn);
	NVIC_SetPriority(IPC_IRQn, pDev->DevIntrf.IntPrio);
	NVIC_EnableIRQ(IPC_IRQn);

	IPC_REGS->INTENSET = 1UL << pDev->RxChan;

	return true;
}

void IpcCtrlrSend(int Chan)
{
	if (!IpcChanValid(Chan))
	{
		return;
	}

	// The bytes written to the rings are visible before the signal.
	__DMB();
	IPC_REGS->TASKS_SEND[Chan] = 1UL;
}

void IpcCtrlrRxEnable(IpcIntrfDev_t * const pDev, bool bEnable)
{
	if (pDev == nullptr || !IpcChanValid(pDev->RxChan))
	{
		return;
	}

	if (bEnable)
	{
		IPC_REGS->INTENSET = 1UL << pDev->RxChan;
	}
	else
	{
		IPC_REGS->INTENCLR = 1UL << pDev->RxChan;
	}
}

// Clear the event before the work, so a signal that comes while the interface
// handles the previous one raises the interrupt again.
extern "C" void IPC_IRQHandler(void)
{
	uint32_t pend = IPC_REGS->INTPEND;

	for (int i = 0; i < IPC_CH_NUM && pend != 0U; i++)
	{
		uint32_t bit = 1UL << i;

		if ((pend & bit) == 0U)
		{
			continue;
		}

		pend &= ~bit;
		IPC_REGS->EVENTS_RECEIVE[i] = 0UL;
		(void)IPC_REGS->EVENTS_RECEIVE[i];

		if (s_pIpcDev[i] != nullptr)
		{
			IpcIntrfIrqHandler(s_pIpcDev[i]);
		}
	}
}

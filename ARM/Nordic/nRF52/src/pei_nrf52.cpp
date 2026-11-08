/**-------------------------------------------------------------------------
@file	pei_nrf52.cpp

@brief	Peripheral event interconnect matrix using Nordic nRF52 PPI

@license
MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include <stddef.h>
#include <stdint.h>

#include "nrf.h"
#include "coredev/interrupt.h"
#include "coredev/pei.h"

#if defined(SOFTDEVICE_PRESENT)
#include "nrf_soc.h"
#elif defined(NRFXLIB_SDC)
#include "sdc_soc.h"
#endif

// The Nordic MDK exposes the configurable PPI channels and groups as arrays.
// Their lengths are compile-time constants and differ across nRF52 devices.
enum {
	PEI_NRF52_CHANNEL_COUNT = sizeof(NRF_PPI->CH) / sizeof(NRF_PPI->CH[0]),
	PEI_NRF52_GROUP_COUNT = sizeof(NRF_PPI->CHG) / sizeof(NRF_PPI->CHG[0])
};
typedef char PeiNrf52ChannelCountCheck[
	(PEI_NRF52_CHANNEL_COUNT > 0 && PEI_NRF52_CHANNEL_COUNT <= 20) ? 1 : -1
];

// PPI channels 20-31 have fixed endpoints and are not allocated by PEI.
// Channels 0 and 1 are used directly by the nRF52 Direct Test Mode driver.
#define PEI_NRF52_DTM_RESERVED		3UL
#define PEI_NRF52_TOKEN_BITS		5U
#define PEI_NRF52_TOKEN_INDEX		31UL
#define PEI_NRF52_TOKEN_MAX_GEN	0x07FFFFFFUL

typedef struct {
	PeiConn_t *pOwner;
	uint32_t Token;
} PeiNrf52Slot_t;

static PeiNrf52Slot_t s_PeiSlot[PEI_NRF52_CHANNEL_COUNT];
static uint32_t s_PeiGeneration;

// Event registers occupy 0x100-0x1FC, task registers 0x000-0x0FC.
// PPI endpoints must address real nRF52 peripheral registers, not RAM.
static bool PeiNrf52EndpointValid(PeiEndpoint_t Addr, bool IsEvent)
{
	uintptr_t addr = (uintptr_t)Addr;
	uintptr_t offset = addr & 0xFFFU;

	if (addr < 0x40000000UL || addr >= 0x50000000UL || (addr & 3U) != 0U)
	{
		return false;
	}

	if (IsEvent)
	{
		return offset >= 0x100U && offset < 0x200U;
	}

	return offset < 0x100U;
}

// Restrict channels at library build time, not from an application board macro.
// Reserve channels used by the installed SoftDevice/SDC even before it starts.
static uint32_t PeiNrf52AvailableMask(void)
{
	uint32_t mask = ((1UL << PEI_NRF52_CHANNEL_COUNT) - 1UL) & ~PEI_NRF52_DTM_RESERVED;

#if defined(SOFTDEVICE_PRESENT)
	mask &= ~NRF_SOC_SD_PPI_CHANNELS_SD_ENABLED_MSK;
#elif defined(NRFXLIB_SDC)
	mask &= ~SDC_PPI_CHANNELS_USED_MASK;
#endif
#ifdef NRF_PPI_CHANNELS_USED
	mask &= ~NRF_PPI_CHANNELS_USED;
#endif
#ifdef NRFX_PPI_CHANNELS_USED
	mask &= ~NRFX_PPI_CHANNELS_USED;
#endif

	return mask;
}

static bool PeiNrf52ChannelFree(int Chan)
{
	uint32_t bit = 1UL << Chan;
	int i;

	if (s_PeiSlot[Chan].pOwner != NULL ||
		(NRF_PPI->CHEN & bit) != 0U ||
		NRF_PPI->CH[Chan].EEP != 0U ||
		NRF_PPI->CH[Chan].TEP != 0U ||
		NRF_PPI->FORK[Chan].TEP != 0U)
	{
		return false;
	}

	for (i = 0; i < PEI_NRF52_GROUP_COUNT; i++)
	{
		if ((NRF_PPI->CHG[i] & bit) != 0U)
		{
			return false;
		}
	}

	return true;
}

// On S132/S140 the SoC calls are required while the SoftDevice is enabled.
// Do not call the SVC operations with PRIMASK set.
static bool PeiNrf52Assign(int Chan, PeiEndpoint_t Event, PeiEndpoint_t Task)
{
#if defined(SOFTDEVICE_PRESENT)
	return sd_ppi_channel_assign((uint8_t)Chan,
		(const volatile void *)(uintptr_t)Event,
		(const volatile void *)(uintptr_t)Task) == 0U;
#else
	NRF_PPI->CH[Chan].EEP = (uint32_t)Event;
	NRF_PPI->CH[Chan].TEP = (uint32_t)Task;
	return true;
#endif
}

static bool PeiNrf52ChannelEnable(int Chan)
{
	uint32_t bit = 1UL << Chan;

#if defined(SOFTDEVICE_PRESENT)
	return sd_ppi_channel_enable_set(bit) == 0U;
#else
	NRF_PPI->CHENSET = bit;
	return true;
#endif
}

static bool PeiNrf52ChannelDisable(int Chan)
{
	uint32_t bit = 1UL << Chan;

#if defined(SOFTDEVICE_PRESENT)
	return sd_ppi_channel_enable_clr(bit) == 0U;
#else
	NRF_PPI->CHENCLR = bit;
	return true;
#endif
}

// A connection cannot be moved or copied: both its address and its opaque
// token must match. Callers serialize operations on one connection object.
static int PeiNrf52OwnedChannel(const PeiConn_t *pConn)
{
	uint32_t idx;
	int chan;

	if (pConn == NULL || pConn->Handle == 0U)
	{
		return -1;
	}

	idx = pConn->Handle & PEI_NRF52_TOKEN_INDEX;
	if (idx == 0U || idx > PEI_NRF52_CHANNEL_COUNT)
	{
		return -1;
	}
	chan = (int)idx - 1;

	if (s_PeiSlot[chan].pOwner != pConn ||
		s_PeiSlot[chan].Token != pConn->Handle)
	{
		return -1;
	}

	return chan;
}

bool PeiConnect(PeiConn_t * const pConn, const PeiCfg_t * const pCfg)
{
	uint32_t state;
	uint32_t allowed;
	uint32_t gen;
	uint32_t token;
	int chan;
	int i;

	if (pConn == NULL || pCfg == NULL || pConn->Handle != 0U ||
		pCfg->DevNo != 0 || pCfg->Options != 0U ||
		!PeiNrf52EndpointValid(pCfg->Event, true) ||
		!PeiNrf52EndpointValid(pCfg->Task, false))
	{
		return false;
	}

	allowed = PeiNrf52AvailableMask();
	chan = -1;
	state = DisableInterrupt();

	for (i = 0; i < PEI_NRF52_CHANNEL_COUNT; i++)
	{
		if ((allowed & (1UL << i)) != 0U && PeiNrf52ChannelFree(i))
		{
			chan = i;
			break;
		}
	}

	if (chan >= 0)
	{
		s_PeiGeneration = (s_PeiGeneration + 1U) & PEI_NRF52_TOKEN_MAX_GEN;
		if (s_PeiGeneration == 0U)
		{
			s_PeiGeneration = 1U;
		}
		gen = s_PeiGeneration;
		token = (gen << PEI_NRF52_TOKEN_BITS) | (uint32_t)(chan + 1);
		s_PeiSlot[chan].pOwner = pConn;
		s_PeiSlot[chan].Token = token;
	}

	EnableInterrupt(state);

	if (chan < 0)
	{
		return false;
	}

	if (!PeiNrf52Assign(chan, pCfg->Event, pCfg->Task))
	{
		state = DisableInterrupt();
		s_PeiSlot[chan].Token = 0;
		s_PeiSlot[chan].pOwner = NULL;
		EnableInterrupt(state);
		return false;
	}

	state = DisableInterrupt();
	pConn->Handle = token;
	EnableInterrupt(state);
	return true;
}

bool PeiEnable(PeiConn_t * const pConn)
{
	int chan = PeiNrf52OwnedChannel(pConn);

	if (chan < 0)
	{
		return false;
	}

	return PeiNrf52ChannelEnable(chan);
}

bool PeiDisable(PeiConn_t * const pConn)
{
	int chan = PeiNrf52OwnedChannel(pConn);

	if (chan < 0)
	{
		return false;
	}

	return PeiNrf52ChannelDisable(chan);
}

void PeiDisconnect(PeiConn_t * const pConn)
{
	uint32_t state;
	int chan;

	if (pConn == NULL)
	{
		return;
	}

	chan = PeiNrf52OwnedChannel(pConn);
	if (chan < 0)
	{
		pConn->Handle = 0U;
		return;
	}

	if (!PeiNrf52ChannelDisable(chan))
	{
		return;
	}
	if (!PeiNrf52Assign(chan, 0U, 0U))
	{
		return;
	}

	state = DisableInterrupt();
	s_PeiSlot[chan].Token = 0U;
	s_PeiSlot[chan].pOwner = NULL;
	pConn->Handle = 0U;
	EnableInterrupt(state);
}

// Software events on EGU channels. PEI owns the channels it allocates here:
// it never sets INTEN, so the SWIn_EGUn interrupt is not raised, and it
// skips a channel whose INTEN bit another user has set. TASKS_TRIGGER[n]
// fires EVENTS_TRIGGERED[n], which the PPI routes of the event carry to
// their tasks.
static NRF_EGU_Type * const s_PeiNrf52Egu[] = {
	NRF_EGU0,
#ifdef NRF_EGU1
	NRF_EGU1,
#endif
#ifdef NRF_EGU2
	NRF_EGU2,
#endif
#ifdef NRF_EGU3
	NRF_EGU3,
#endif
#ifdef NRF_EGU4
	NRF_EGU4,
#endif
#ifdef NRF_EGU5
	NRF_EGU5,
#endif
};

enum {
	PEI_NRF52_EGU_COUNT = sizeof(s_PeiNrf52Egu) / sizeof(s_PeiNrf52Egu[0]),
	PEI_NRF52_EGU_CH_COUNT = sizeof(NRF_EGU0->TASKS_TRIGGER) / sizeof(NRF_EGU0->TASKS_TRIGGER[0])
};
typedef char PeiNrf52EguChannelCountCheck[
	(PEI_NRF52_EGU_CH_COUNT > 0 && PEI_NRF52_EGU_CH_COUNT <= 32) ? 1 : -1
];

// Channels allocated as software events, one bit per channel
static uint32_t s_PeiEguOwned[PEI_NRF52_EGU_COUNT];

// Called by the generic matrix in src/coredev/pei.c through weak references
extern "C" {
bool PeiSwEventAllocMcu(int DevNo, PeiEndpoint_t *pEvent);
void PeiSwEventFreeMcu(PeiEndpoint_t Event);
bool PeiSwEventTriggerMcu(PeiEndpoint_t Event);
PeiEndpoint_t PeiSwEventTaskMcu(PeiEndpoint_t Event);
}

// EGU instances PEI may use, bit n for EGUn. With a SoftDevice, SWI1/EGU1 is
// the radio notification, SWI2/EGU2 the SoftDevice event interrupt, SWI4/EGU4
// and SWI5/EGU5 are reserved by the SoftDevice. SDC builds give SWI5/EGU5
// to MPSL as its low priority interrupt (nrf_mpsl.h). NRF_EGUS_USED reserves
// more instances at library build time.
static uint32_t PeiNrf52EguAvailableMask(void)
{
	uint32_t mask = (1UL << PEI_NRF52_EGU_COUNT) - 1UL;

#if defined(SOFTDEVICE_PRESENT)
	mask &= ~((1UL << 1) | (1UL << 2) | (1UL << 4) | (1UL << 5));
#elif defined(NRFXLIB_SDC)
	mask &= ~(1UL << 5);
#endif
#ifdef NRF_EGUS_USED
	mask &= ~(uint32_t)NRF_EGUS_USED;
#endif

	return mask;
}

// Instance and channel of an owned software event, from its event endpoint
static bool PeiNrf52EguOwned(PeiEndpoint_t Event, int *pInst, int *pChan)
{
	int i;

	for (i = 0; i < PEI_NRF52_EGU_COUNT; i++)
	{
		uintptr_t base = (uintptr_t)&s_PeiNrf52Egu[i]->EVENTS_TRIGGERED[0];

		if (Event < base || Event >= base + PEI_NRF52_EGU_CH_COUNT * 4U ||
			((Event - base) & 3U) != 0U)
		{
			continue;
		}

		int chan = (int)((Event - base) / 4U);

		if ((s_PeiEguOwned[i] & (1UL << chan)) == 0U)
		{
			return false;
		}
		*pInst = i;
		*pChan = chan;

		return true;
	}

	return false;
}

bool PeiSwEventAllocMcu(int DevNo, PeiEndpoint_t *pEvent)
{
	uint32_t allowed;
	uint32_t state;
	int inst = -1;
	int chan = -1;
	int i;
	int j;

	if (DevNo != 0 || pEvent == NULL)
	{
		return false;
	}

	allowed = PeiNrf52EguAvailableMask();
	state = DisableInterrupt();

	for (i = 0; i < PEI_NRF52_EGU_COUNT && inst < 0; i++)
	{
		if ((allowed & (1UL << i)) == 0U)
		{
			continue;
		}
		for (j = 0; j < PEI_NRF52_EGU_CH_COUNT; j++)
		{
			uint32_t bit = 1UL << j;

			if ((s_PeiEguOwned[i] & bit) == 0U && (s_PeiNrf52Egu[i]->INTEN & bit) == 0U)
			{
				s_PeiEguOwned[i] |= bit;
				inst = i;
				chan = j;
				break;
			}
		}
	}

	EnableInterrupt(state);

	if (inst < 0)
	{
		return false;
	}

	s_PeiNrf52Egu[inst]->EVENTS_TRIGGERED[chan] = 0;
	*pEvent = (PeiEndpoint_t)&s_PeiNrf52Egu[inst]->EVENTS_TRIGGERED[chan];

	return true;
}

void PeiSwEventFreeMcu(PeiEndpoint_t Event)
{
	uint32_t state;
	int inst;
	int chan;

	if (!PeiNrf52EguOwned(Event, &inst, &chan))
	{
		return;
	}

	s_PeiNrf52Egu[inst]->EVENTS_TRIGGERED[chan] = 0;

	state = DisableInterrupt();
	s_PeiEguOwned[inst] &= ~(1UL << chan);
	EnableInterrupt(state);
}

bool PeiSwEventTriggerMcu(PeiEndpoint_t Event)
{
	int inst;
	int chan;

	if (!PeiNrf52EguOwned(Event, &inst, &chan))
	{
		return false;
	}

	// The PPI routes run from the event itself. The register is cleared first
	// so that it shows this trigger only.
	s_PeiNrf52Egu[inst]->EVENTS_TRIGGERED[chan] = 0;
	s_PeiNrf52Egu[inst]->TASKS_TRIGGER[chan] = 1;

	return true;
}

PeiEndpoint_t PeiSwEventTaskMcu(PeiEndpoint_t Event)
{
	int inst;
	int chan;

	if (!PeiNrf52EguOwned(Event, &inst, &chan))
	{
		return 0U;
	}

	return (PeiEndpoint_t)&s_PeiNrf52Egu[inst]->TASKS_TRIGGER[chan];
}

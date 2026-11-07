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

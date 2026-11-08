/**-------------------------------------------------------------------------
@file	pei_nrf53.c

@brief	Peripheral event interconnect matrix using Nordic nRF53 DPPI

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

#if defined(NRF5340_XXAA_APPLICATION)
#if defined(NRF_TRUSTZONE_NONSECURE)
#define PEI_DPPIC	NRF_DPPIC_NS
#define PEI_ADDR_BEGIN	0x40000000UL
#define PEI_ADDR_END	0x50000000UL
#else
#define PEI_DPPIC	NRF_DPPIC_S
#define PEI_ADDR_BEGIN	0x50000000UL
#define PEI_ADDR_END	0x60000000UL
#endif
#elif defined(NRF5340_XXAA_NETWORK)
#define PEI_DPPIC	NRF_DPPIC_NS
#define PEI_ADDR_BEGIN	0x40000000UL
#define PEI_ADDR_END	0x50000000UL
#else
#error PEI nRF53 supports nRF5340 application and network cores
#endif

// One DPPIC with 32 channels per core. One publisher can feed multiple tasks.
#define PEI_NRF53_CHANNEL_COUNT	32
#define PEI_NRF53_ROUTE_COUNT	32
#define PEI_NRF53_TOKEN_INDEX	0x3FUL
#define PEI_NRF53_MAX_GENERATION	0x03FFFFFFUL
#define PEI_NRF53_ENDPOINT_ENABLE	0x80000000UL

enum {
	PEI_NRF53_GROUP_COUNT = sizeof(PEI_DPPIC->CHG) / sizeof(PEI_DPPIC->CHG[0])
};

typedef struct {
	PeiEndpoint_t Event;
	uint8_t Owners;
	uint8_t Enabled;
} PeiNrf53Channel_t;

typedef struct {
	PeiConn_t *pOwner;
	PeiEndpoint_t Event;
	PeiEndpoint_t Task;
	uint32_t Token;
	uint8_t Channel;
	bool Enabled;
} PeiNrf53Route_t;

static PeiNrf53Channel_t s_Channels[PEI_NRF53_CHANNEL_COUNT];
static PeiNrf53Route_t s_Routes[PEI_NRF53_ROUTE_COUNT];
static uint32_t s_Generation;

// Publish and subscribe registers are at +0x80 from event/task registers.
// A valid offset does not prove the endpoint register is implemented.
static bool PeiNrf53EndpointValid(PeiEndpoint_t Addr, bool IsEvent)
{
	uintptr_t offset;

	if (Addr < PEI_ADDR_BEGIN || Addr >= PEI_ADDR_END || (Addr & 3U) != 0U)
	{
		return false;
	}
	offset = (uintptr_t)Addr & 0xFFFU;
	if (IsEvent)
	{
		return offset >= 0x100U && offset < 0x180U;
	}
	return offset < 0x80U;
}

#ifdef PEI_NRF53_TEST
extern volatile uint32_t *PeiNrf53TestRegister(PeiEndpoint_t Addr);
#define PEI_NRF53_REG(addr)	PeiNrf53TestRegister(addr)
#else
#define PEI_NRF53_REG(addr)	((volatile uint32_t *)(uintptr_t)(addr))
#endif

static volatile uint32_t *PeiNrf53Publish(PeiEndpoint_t Event)
{
	return PEI_NRF53_REG(Event + 0x80U);
}

static volatile uint32_t *PeiNrf53Subscribe(PeiEndpoint_t Task)
{
	return PEI_NRF53_REG(Task + 0x80U);
}

static uint32_t PeiNrf53ReservedMask(void)
{
	uint32_t mask = 0U;

#ifdef NRF_DPPI_CHANNELS_USED
	mask |= NRF_DPPI_CHANNELS_USED;
#endif
#ifdef NRFX_DPPI_CHANNELS_USED
	mask |= NRFX_DPPI_CHANNELS_USED;
#endif
	return mask;
}

static bool PeiNrf53ChannelFree(int Chan)
{
	uint32_t bit = 1UL << Chan;
	int i;

	if (s_Channels[Chan].Owners != 0U ||
		(PEI_DPPIC->CHEN & bit) != 0U ||
		(PeiNrf53ReservedMask() & bit) != 0U)
	{
		return false;
	}
	for (i = 0; i < PEI_NRF53_GROUP_COUNT; i++)
	{
		if ((PEI_DPPIC->CHG[i] & bit) != 0U)
		{
			return false;
		}
	}
	return true;
}

// Called with interrupts disabled. Connection address and token must match.
static int PeiNrf53RouteIndex(const PeiConn_t *pConn)
{
	uint32_t idx;

	if (pConn == NULL || pConn->Handle == 0U)
	{
		return -1;
	}
	idx = pConn->Handle & PEI_NRF53_TOKEN_INDEX;
	if (idx == 0U || idx > PEI_NRF53_ROUTE_COUNT)
	{
		return -1;
	}
	idx--;
	if (s_Routes[idx].pOwner != pConn || s_Routes[idx].Token != pConn->Handle)
	{
		return -1;
	}
	return (int)idx;
}

static bool PeiNrf53DisableRoute(int Index)
{
	PeiNrf53Route_t *pRoute = &s_Routes[Index];
	PeiNrf53Channel_t *pChannel = &s_Channels[pRoute->Channel];
	volatile uint32_t *pSub = PeiNrf53Subscribe(pRoute->Task);
	uint32_t value = PEI_NRF53_ENDPOINT_ENABLE | pRoute->Channel;

	if (!pRoute->Enabled)
	{
		return *pSub == 0U;
	}
	if (*pSub != value || pChannel->Enabled == 0U)
	{
		return false;
	}
	*pSub = 0U;
	pRoute->Enabled = false;
	pChannel->Enabled--;
	if (pChannel->Enabled == 0U)
	{
		PEI_DPPIC->CHENCLR = 1UL << pRoute->Channel;
	}
	return true;
}

bool PeiConnect(PeiConn_t * const pConn, const PeiCfg_t * const pCfg)
{
	uint32_t state;
	uint32_t token;
	uint32_t gen;
	uint32_t value;
	int chan = -1;
	int route = -1;
	int i;

	if (pConn == NULL || pCfg == NULL || pCfg->DevNo != 0 ||
		pCfg->Options != 0U ||
		!PeiNrf53EndpointValid(pCfg->Event, true) ||
		!PeiNrf53EndpointValid(pCfg->Task, false))
	{
		return false;
	}
	state = DisableInterrupt();
	if (pConn->Handle != 0U || *PeiNrf53Subscribe(pCfg->Task) != 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	for (i = 0; i < PEI_NRF53_ROUTE_COUNT; i++)
	{
		if (s_Routes[i].pOwner == NULL && route < 0)
		{
			route = i;
		}
		if (s_Routes[i].pOwner != NULL && s_Routes[i].Task == pCfg->Task)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	if (route < 0)
	{
		EnableInterrupt(state);
		return false;
	}
	for (i = 0; i < PEI_NRF53_CHANNEL_COUNT; i++)
	{
		if (s_Channels[i].Owners != 0U && s_Channels[i].Event == pCfg->Event)
		{
			chan = i;
			break;
		}
	}
	if (chan >= 0)
	{
		value = PEI_NRF53_ENDPOINT_ENABLE | (uint32_t)chan;
		if (*PeiNrf53Publish(pCfg->Event) != value)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	else
	{
		if (*PeiNrf53Publish(pCfg->Event) != 0U)
		{
			EnableInterrupt(state);
			return false;
		}
		for (i = 0; i < PEI_NRF53_CHANNEL_COUNT; i++)
		{
			if (PeiNrf53ChannelFree(i))
			{
				chan = i;
				break;
			}
		}
		if (chan < 0)
		{
			EnableInterrupt(state);
			return false;
		}
		*PeiNrf53Publish(pCfg->Event) = PEI_NRF53_ENDPOINT_ENABLE | (uint32_t)chan;
		s_Channels[chan].Event = pCfg->Event;
	}
	s_Generation = (s_Generation + 1U) & PEI_NRF53_MAX_GENERATION;
	if (s_Generation == 0U)
	{
		s_Generation = 1U;
	}
	gen = s_Generation;
	token = (gen << 6U) | (uint32_t)(route + 1);
	s_Routes[route].pOwner = pConn;
	s_Routes[route].Event = pCfg->Event;
	s_Routes[route].Task = pCfg->Task;
	s_Routes[route].Token = token;
	s_Routes[route].Channel = (uint8_t)chan;
	s_Routes[route].Enabled = false;
	s_Channels[chan].Owners++;
	pConn->Handle = token;
	EnableInterrupt(state);
	return true;
}

bool PeiEnable(PeiConn_t * const pConn)
{
	uint32_t state = DisableInterrupt();
	int idx = PeiNrf53RouteIndex(pConn);
	PeiNrf53Route_t *pRoute;
	PeiNrf53Channel_t *pChannel;
	volatile uint32_t *pSub;
	uint32_t value;

	if (idx < 0)
	{
		EnableInterrupt(state);
		return false;
	}
	pRoute = &s_Routes[idx];
	pChannel = &s_Channels[pRoute->Channel];
	value = PEI_NRF53_ENDPOINT_ENABLE | pRoute->Channel;
	pSub = PeiNrf53Subscribe(pRoute->Task);
	if (*PeiNrf53Publish(pRoute->Event) != value ||
		(!pRoute->Enabled && *pSub != 0U) ||
		(pRoute->Enabled && *pSub != value))
	{
		EnableInterrupt(state);
		return false;
	}
	if (!pRoute->Enabled)
	{
		*pSub = value;
		pRoute->Enabled = true;
		pChannel->Enabled++;
		if (pChannel->Enabled == 1U)
		{
			PEI_DPPIC->CHENSET = 1UL << pRoute->Channel;
		}
	}
	EnableInterrupt(state);
	return true;
}

bool PeiDisable(PeiConn_t * const pConn)
{
	uint32_t state = DisableInterrupt();
	int idx = PeiNrf53RouteIndex(pConn);
	bool result;

	if (idx < 0)
	{
		EnableInterrupt(state);
		return false;
	}
	result = PeiNrf53DisableRoute(idx);
	EnableInterrupt(state);
	return result;
}

void PeiDisconnect(PeiConn_t * const pConn)
{
	uint32_t state;
	int idx;
	PeiNrf53Route_t *pRoute;
	PeiNrf53Channel_t *pChannel;

	if (pConn == NULL)
	{
		return;
	}
	state = DisableInterrupt();
	idx = PeiNrf53RouteIndex(pConn);
	if (idx < 0)
	{
		pConn->Handle = 0U;
		EnableInterrupt(state);
		return;
	}
	pRoute = &s_Routes[idx];
	pChannel = &s_Channels[pRoute->Channel];
	if (*PeiNrf53Publish(pRoute->Event) !=
		(PEI_NRF53_ENDPOINT_ENABLE | pRoute->Channel) ||
		!PeiNrf53DisableRoute(idx))
	{
		EnableInterrupt(state);
		return;
	}
	pChannel->Owners--;
	if (pChannel->Owners == 0U)
	{
		*PeiNrf53Publish(pRoute->Event) = 0U;
		pChannel->Event = 0U;
	}
	pRoute->pOwner = NULL;
	pRoute->Token = 0U;
	pRoute->Event = 0U;
	pRoute->Task = 0U;
	pRoute->Enabled = false;
	pConn->Handle = 0U;
	EnableInterrupt(state);
}

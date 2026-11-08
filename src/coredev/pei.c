/**-------------------------------------------------------------------------
@file	pei.c

@brief	Generic C PEI matrix ownership and event-to-task relationships

Hardware routing stays in the MCU-specific PeiConnect/PeiEnable/
PeiDisable/PeiDisconnect backend. A PeiDev_t owns fixed-size matrix
storage and is shared by C applications and the C++ Pei wrapper.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "coredev/pei.h"

// The MCU capability provider is optional. A weak reference cannot extract
// the Timer backend from a static archive when PEI is not used.
typedef struct __Timer_Device TimerDev_t;
#if defined(__GNUC__)
extern bool TimerPeiPrepareMcu(TimerDev_t *pTimer) __attribute__((weak));
#else
extern bool TimerPeiPrepareMcu(TimerDev_t *pTimer);
#endif

bool PeiTimerPrepare(TimerDev_t *pTimer)
{
	if (pTimer == NULL || TimerPeiPrepareMcu == NULL)
	{
		return false;
	}
	return TimerPeiPrepareMcu(pTimer);
}


static int PeiFindEvent(const PeiDev_t *pPei, PeiEvent_t Evt)
{
	uint32_t slot;

	if (pPei == NULL || Evt == 0U)
	{
		return -1;
	}
	slot = Evt & 63U;
	if (slot == 0U || slot > PEI_MAX_EVENTS)
	{
		return -1;
	}
	if (pPei->Events[slot - 1U].Token != Evt)
	{
		return -1;
	}
	return (int)slot - 1;
}

bool PeiInit(PeiDev_t *pPei, int DevNo)
{
	if (pPei == NULL || DevNo < 0)
	{
		return false;
	}
	// PeiDeinit must be called before reinitializing a live matrix.
	memset(pPei, 0, sizeof(*pPei));
	pPei->DevNo = DevNo;
	return true;
}

bool PeiDeinit(PeiDev_t *pPei)
{
	PeiEvent_t evt;
	int i;

	if (pPei == NULL)
	{
		return false;
	}
	for (i = 0; i < PEI_MAX_EVENTS; i++)
	{
		if (pPei->Events[i].Token != 0U)
		{
			evt = pPei->Events[i].Token;
			if (!PeiReleaseEvent(pPei, &evt))
			{
				return false;
			}
		}
	}
	return true;
}

PeiEndpoint_t PeiGetEventEndpoint(const PeiDev_t *pPei, PeiEvent_t Evt)
{
	int e;

	e = PeiFindEvent(pPei, Evt);
	if (e < 0)
	{
		return 0U;
	}
	return pPei->Events[e].Endpoint;
}

bool PeiBindEvent(PeiDev_t *pPei, PeiEvent_t *pEvt, PeiEndpoint_t Endpoint)
{
	PeiCfg_t cfg;
	PeiEvent_t evt;
	bool fresh;
	int e;
	int i;

	if (pPei == NULL || pEvt == NULL || Endpoint == 0U)
	{
		return false;
	}
	evt = *pEvt;
	fresh = evt == 0U;
	e = PeiFindEvent(pPei, evt);
	if (fresh)
	{
		for (i = 0; i < PEI_MAX_EVENTS; i++)
		{
			if (pPei->Events[i].Token == 0U)
			{
				e = i;
				break;
			}
		}
	}
	if (e < 0)
	{
		return false;
	}
	if (pPei->Events[e].Endpoint != 0U)
	{
		if (pPei->Events[e].Endpoint != Endpoint)
		{
			return false;
		}
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (pPei->Routes[i].Token == evt &&
				pPei->Routes[i].Conn.Handle == 0U)
			{
				return false;
			}
		}
		return true;
	}
	for (i = 0; i < PEI_MAX_EVENTS; i++)
	{
		if (i != e && pPei->Events[i].Endpoint == Endpoint)
		{
			return false;
		}
	}
	if (fresh)
	{
		pPei->Generation = (pPei->Generation + 1U) & 0x03FFFFFFU;
		if (pPei->Generation == 0U)
		{
			pPei->Generation = 1U;
		}
		evt = (pPei->Generation << 6U) | (uint32_t)(e + 1);
		pPei->Events[e].Token = evt;
		pPei->Events[e].Enabled = false;
		*pEvt = evt;
	}
	pPei->Events[e].Endpoint = Endpoint;
	cfg.DevNo = pPei->DevNo;
	cfg.Event = Endpoint;
	cfg.Options = 0U;
	for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
	{
		if (pPei->Routes[i].Token != evt)
		{
			continue;
		}
		cfg.Task = pPei->Routes[i].Task;
		if (!PeiConnect(&pPei->Routes[i].Conn, &cfg))
		{
			if (!PeiUnbindEvent(pPei, evt, Endpoint))
			{
				return false;
			}
			if (fresh)
			{
				PeiReleaseEvent(pPei, pEvt);
			}
			return false;
		}
		if (pPei->Events[e].Enabled &&
			!PeiEnable(&pPei->Routes[i].Conn))
		{
			if (!PeiUnbindEvent(pPei, evt, Endpoint))
			{
				return false;
			}
			if (fresh)
			{
				PeiReleaseEvent(pPei, pEvt);
			}
			return false;
		}
	}
	return true;
}

bool PeiUnbindEvent(PeiDev_t *pPei, PeiEvent_t Evt, PeiEndpoint_t Endpoint)
{
	int e;
	int i;

	e = PeiFindEvent(pPei, Evt);
	if (e < 0 || Endpoint == 0U || pPei->Events[e].Endpoint != Endpoint)
	{
		return false;
	}
	for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
	{
		if (pPei->Routes[i].Token == Evt &&
			pPei->Routes[i].Conn.Handle != 0U &&
			!PeiDisable(&pPei->Routes[i].Conn))
		{
			return false;
		}
	}
	for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
	{
		if (pPei->Routes[i].Token != Evt ||
			pPei->Routes[i].Conn.Handle == 0U)
		{
			continue;
		}
		PeiDisconnect(&pPei->Routes[i].Conn);
		if (pPei->Routes[i].Conn.Handle != 0U)
		{
			return false;
		}
	}
	pPei->Events[e].Endpoint = 0U;
	return true;
}

bool PeiConnectTask(PeiDev_t *pPei, PeiEvent_t Evt, PeiEndpoint_t Task)
{
	PeiCfg_t cfg;
	int e;
	int freeSlot;
	int i;

	e = PeiFindEvent(pPei, Evt);
	if (e < 0 || Task == 0U)
	{
		return false;
	}
	freeSlot = -1;
	for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
	{
		if (pPei->Routes[i].Token == Evt && pPei->Routes[i].Task == Task)
		{
			if (pPei->Events[e].Endpoint == 0U)
			{
				return true;
			}
			return pPei->Routes[i].Conn.Handle != 0U;
		}
		if (freeSlot < 0 && pPei->Routes[i].Token == 0U)
		{
			freeSlot = i;
		}
	}
	if (freeSlot < 0)
	{
		return false;
	}
	pPei->Routes[freeSlot].Token = Evt;
	pPei->Routes[freeSlot].Task = Task;
	if (pPei->Events[e].Endpoint != 0U)
	{
		cfg.DevNo = pPei->DevNo;
		cfg.Event = pPei->Events[e].Endpoint;
		cfg.Task = Task;
		cfg.Options = 0U;
		if (!PeiConnect(&pPei->Routes[freeSlot].Conn, &cfg))
		{
			if (pPei->Routes[freeSlot].Conn.Handle == 0U)
			{
				pPei->Routes[freeSlot].Token = 0U;
				pPei->Routes[freeSlot].Task = 0U;
			}
			return false;
		}
		if (pPei->Events[e].Enabled &&
			!PeiEnable(&pPei->Routes[freeSlot].Conn))
		{
			PeiDisconnect(&pPei->Routes[freeSlot].Conn);
			if (pPei->Routes[freeSlot].Conn.Handle == 0U)
			{
				pPei->Routes[freeSlot].Token = 0U;
				pPei->Routes[freeSlot].Task = 0U;
			}
			return false;
		}
	}
	return true;
}

bool PeiDisconnectTask(PeiDev_t *pPei, PeiEvent_t Evt, PeiEndpoint_t Task)
{
	int i;

	if (PeiFindEvent(pPei, Evt) < 0)
	{
		return false;
	}
	for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
	{
		if (pPei->Routes[i].Token != Evt || pPei->Routes[i].Task != Task)
		{
			continue;
		}
		if (pPei->Routes[i].Conn.Handle != 0U)
		{
			PeiDisconnect(&pPei->Routes[i].Conn);
			if (pPei->Routes[i].Conn.Handle != 0U)
			{
				return false;
			}
		}
		pPei->Routes[i].Token = 0U;
		pPei->Routes[i].Task = 0U;
		return true;
	}
	// Already removed is a successful idempotent cleanup.
	return true;
}

bool PeiEnableEvent(PeiDev_t *pPei, PeiEvent_t Evt)
{
	int e;
	int i;
	int j;

	e = PeiFindEvent(pPei, Evt);
	if (e < 0 || pPei->Events[e].Endpoint == 0U)
	{
		return false;
	}
	if (pPei->Events[e].Enabled)
	{
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (pPei->Routes[i].Token == Evt &&
				pPei->Routes[i].Conn.Handle == 0U)
			{
				return false;
			}
		}
		return true;
	}
	for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
	{
		if (pPei->Routes[i].Token != Evt)
		{
			continue;
		}
		if (pPei->Routes[i].Conn.Handle == 0U ||
			!PeiEnable(&pPei->Routes[i].Conn))
		{
			for (j = 0; j < i; j++)
			{
				if (pPei->Routes[j].Token == Evt &&
					pPei->Routes[j].Conn.Handle != 0U)
				{
					PeiDisable(&pPei->Routes[j].Conn);
				}
			}
			return false;
		}
	}
	pPei->Events[e].Enabled = true;
	return true;
}

bool PeiDisableEvent(PeiDev_t *pPei, PeiEvent_t Evt)
{
	int e;
	int i;
	int j;

	e = PeiFindEvent(pPei, Evt);
	if (e < 0)
	{
		return false;
	}
	if (!pPei->Events[e].Enabled || pPei->Events[e].Endpoint == 0U)
	{
		pPei->Events[e].Enabled = false;
		return true;
	}
	for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
	{
		if (pPei->Routes[i].Token != Evt)
		{
			continue;
		}
		if (pPei->Routes[i].Conn.Handle == 0U ||
			!PeiDisable(&pPei->Routes[i].Conn))
		{
			for (j = 0; j < i; j++)
			{
				if (pPei->Routes[j].Token == Evt &&
					pPei->Routes[j].Conn.Handle != 0U)
				{
					PeiEnable(&pPei->Routes[j].Conn);
				}
			}
			return false;
		}
	}
	pPei->Events[e].Enabled = false;
	return true;
}

bool PeiReleaseEvent(PeiDev_t *pPei, PeiEvent_t *pEvt)
{
	int e;
	int i;

	if (pEvt == NULL)
	{
		return false;
	}
	e = PeiFindEvent(pPei, *pEvt);
	if (e < 0)
	{
		return false;
	}
	if (pPei->Events[e].Endpoint != 0U)
	{
		if (!PeiUnbindEvent(pPei, *pEvt, pPei->Events[e].Endpoint))
		{
			return false;
		}
	}
	for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
	{
		if (pPei->Routes[i].Token == *pEvt)
		{
			pPei->Routes[i].Token = 0U;
			pPei->Routes[i].Task = 0U;
		}
	}
	pPei->Events[e].Token = 0U;
	pPei->Events[e].Enabled = false;
	*pEvt = 0U;
	return true;
}

/**-------------------------------------------------------------------------
@file	pei_matrix.h

@brief	C++ owner of a PEI logical event-to-task connection matrix

Hardware endpoint initialization belongs to peripheral drivers. This
object owns relationships only; no heap, DMA transfers or ISR forwarding.

----------------------------------------------------------------------------*/
#ifndef __PEI_MATRIX_H__
#define __PEI_MATRIX_H__

#ifndef PEI_MAX_EVENTS
#define PEI_MAX_EVENTS 16
#endif
#ifndef PEI_MAX_CONNECTIONS
#define PEI_MAX_CONNECTIONS 32
#endif
#if PEI_MAX_EVENTS < 1 || PEI_MAX_EVENTS > 63 || PEI_MAX_CONNECTIONS < 1
#error Invalid PEI logical matrix capacity
#endif

// One PEI device owns multiple logical events and event-to-task connections.
// Peripheral drivers provide already-configured event and task endpoints.
// Callers must serialize matrix changes; do not copy or move a live Pei.
class Pei {
public:
	explicit Pei(int DevNo = 0) : vDevNo(DevNo), vGeneration(0U)
	{
		int i;

		for (i = 0; i < PEI_MAX_EVENTS; i++)
		{
			vEvents[i].Token = 0U;
			vEvents[i].Endpoint = 0U;
			vEvents[i].Enabled = false;
		}
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			vRoutes[i].Token = 0U;
			vRoutes[i].Task = 0U;
			vRoutes[i].Conn.Handle = 0U;
		}
	}

	~Pei()
	{
		int i;
		PeiEvent_t evt;

		for (i = 0; i < PEI_MAX_EVENTS; i++)
		{
			if (vEvents[i].Token != 0U)
			{
				evt = vEvents[i].Token;
				ReleaseEvent(evt);
			}
		}
	}

	Pei(const Pei &) = delete;
	Pei &operator=(const Pei &) = delete;

	// Bind a prepared hardware event endpoint. When Evt is zero, allocate
	// a persistent logical event identity. Previously connected tasks survive
	// UnbindEvent and can be reconnected when another source is bound.
	bool BindEvent(PeiEvent_t &Evt, PeiEndpoint_t Endpoint)
	{
		int e;
		int i;
		bool fresh;
		PeiCfg_t cfg;

		if (Endpoint == 0U)
		{
			return false;
		}

		fresh = Evt == 0U;
		e = FindEvent(Evt);
		if (fresh)
		{
			for (i = 0; i < PEI_MAX_EVENTS; i++)
			{
				if (vEvents[i].Token == 0U)
				{
					e = i;
					break;
				}
			}
			if (e < 0)
			{
				return false;
			}
		}
		if (e < 0)
		{
			return false;
		}
		if (vEvents[e].Endpoint != 0U)
		{
			return vEvents[e].Endpoint == Endpoint;
		}
		for (i = 0; i < PEI_MAX_EVENTS; i++)
		{
			if (i != e && vEvents[i].Endpoint == Endpoint)
			{
				return false;
			}
		}

		if (fresh)
		{
			vGeneration = (vGeneration + 1U) & 0x03FFFFFFU;
			if (vGeneration == 0U)
			{
				vGeneration = 1U;
			}
			vEvents[e].Token = (vGeneration << 6U) | (uint32_t)(e + 1);
			vEvents[e].Enabled = false;
			Evt = vEvents[e].Token;
		}

		vEvents[e].Endpoint = Endpoint;
		cfg.DevNo = vDevNo;
		cfg.Event = Endpoint;
		cfg.Options = 0U;
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (vRoutes[i].Token != Evt)
			{
				continue;
			}
			cfg.Task = vRoutes[i].Task;
			if (!PeiConnect(&vRoutes[i].Conn, &cfg) ||
				(vEvents[e].Enabled && !PeiEnable(&vRoutes[i].Conn)))
			{
				// On a hardware teardown failure, keep the event bound so
				// its peripheral endpoint cannot be released unnoticed.
				if (!UnbindEvent(Evt, Endpoint))
				{
					return false;
				}
				if (fresh)
				{
					ReleaseEvent(Evt);
				}
				return false;
			}
		}
		return true;
	}

	// Call before the peripheral driver reuses its hardware event endpoint.
	// The task relationships and logical enable state remain registered.
	bool UnbindEvent(PeiEvent_t Evt, PeiEndpoint_t Endpoint)
	{
		int e;
		int i;

		e = FindEvent(Evt);
		if (e < 0 || vEvents[e].Endpoint != Endpoint || Endpoint == 0U)
		{
			return false;
		}
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (vRoutes[i].Token == Evt && vRoutes[i].Conn.Handle != 0U &&
				!PeiDisable(&vRoutes[i].Conn))
			{
				return false;
			}
		}
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (vRoutes[i].Token != Evt || vRoutes[i].Conn.Handle == 0U)
			{
				continue;
			}
			PeiDisconnect(&vRoutes[i].Conn);
			if (vRoutes[i].Conn.Handle != 0U)
			{
				return false;
			}
		}
		vEvents[e].Endpoint = 0U;
		return true;
	}

	// Add one relationship. A task can be attached to multiple events if
	// this hardware backend can implement that topology without extra edges.
	bool Connect(PeiEvent_t Evt, PeiEndpoint_t Task)
	{
		int e;
		int freeSlot;
		int i;
		PeiCfg_t cfg;

		e = FindEvent(Evt);
		if (e < 0 || Task == 0U)
		{
			return false;
		}
		freeSlot = -1;
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (vRoutes[i].Token == Evt && vRoutes[i].Task == Task)
			{
				return vEvents[e].Endpoint == 0U ||
					vRoutes[i].Conn.Handle != 0U;
			}
			if (freeSlot < 0 && vRoutes[i].Token == 0U)
			{
				freeSlot = i;
			}
		}
		if (freeSlot < 0)
		{
			return false;
		}
		vRoutes[freeSlot].Token = Evt;
		vRoutes[freeSlot].Task = Task;
		if (vEvents[e].Endpoint != 0U)
		{
			cfg.DevNo = vDevNo;
			cfg.Event = vEvents[e].Endpoint;
			cfg.Task = Task;
			cfg.Options = 0U;
			if (!PeiConnect(&vRoutes[freeSlot].Conn, &cfg))
			{
				if (vRoutes[freeSlot].Conn.Handle == 0U)
				{
					vRoutes[freeSlot].Token = 0U;
					vRoutes[freeSlot].Task = 0U;
				}
				return false;
			}
			if (vEvents[e].Enabled && !PeiEnable(&vRoutes[freeSlot].Conn))
			{
				PeiDisconnect(&vRoutes[freeSlot].Conn);
				if (vRoutes[freeSlot].Conn.Handle == 0U)
				{
					vRoutes[freeSlot].Token = 0U;
					vRoutes[freeSlot].Task = 0U;
				}
				return false;
			}
		}
		return true;
	}

	// Remove one relationship. Other events connected to the same task
	// and other tasks connected to the event remain unaffected.
	bool Disconnect(PeiEvent_t Evt, PeiEndpoint_t Task)
	{
		int i;

		if (FindEvent(Evt) < 0)
		{
			return false;
		}
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (vRoutes[i].Token != Evt || vRoutes[i].Task != Task)
			{
				continue;
			}
			if (vRoutes[i].Conn.Handle != 0U)
			{
				PeiDisconnect(&vRoutes[i].Conn);
				if (vRoutes[i].Conn.Handle != 0U)
				{
					return false;
				}
			}
			vRoutes[i].Token = 0U;
			vRoutes[i].Task = 0U;
			return true;
		}
		return false;
	}

	// Switch all of an event's routes without changing other events.
	bool Enable(PeiEvent_t Evt)
	{
		int e;
		int i;
		int j;

		e = FindEvent(Evt);
		if (e < 0 || vEvents[e].Endpoint == 0U)
		{
			return false;
		}
		if (vEvents[e].Enabled)
		{
			return true;
		}
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (vRoutes[i].Token == Evt && vRoutes[i].Conn.Handle != 0U &&
				!PeiEnable(&vRoutes[i].Conn))
			{
				for (j = 0; j < i; j++)
				{
					if (vRoutes[j].Token == Evt &&
						vRoutes[j].Conn.Handle != 0U)
					{
						PeiDisable(&vRoutes[j].Conn);
					}
				}
				return false;
			}
		}
		vEvents[e].Enabled = true;
		return true;
	}

	bool Disable(PeiEvent_t Evt)
	{
		int e;
		int i;

		e = FindEvent(Evt);
		if (e < 0)
		{
			return false;
		}
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (vRoutes[i].Token == Evt && vRoutes[i].Conn.Handle != 0U &&
				!PeiDisable(&vRoutes[i].Conn))
			{
				return false;
			}
		}
		vEvents[e].Enabled = false;
		return true;
	}

	// Remove the event identity and all of its connections.
	bool ReleaseEvent(PeiEvent_t &Evt)
	{
		int e;
		int i;

		e = FindEvent(Evt);
		if (e < 0)
		{
			return false;
		}
		if (vEvents[e].Endpoint != 0U &&
			!UnbindEvent(Evt, vEvents[e].Endpoint))
		{
			return false;
		}
		for (i = 0; i < PEI_MAX_CONNECTIONS; i++)
		{
			if (vRoutes[i].Token == Evt)
			{
				vRoutes[i].Token = 0U;
				vRoutes[i].Task = 0U;
			}
		}
		vEvents[e].Token = 0U;
		vEvents[e].Enabled = false;
		Evt = 0U;
		return true;
	}

private:
	struct EventState {
		PeiEvent_t Token;
		PeiEndpoint_t Endpoint;
		bool Enabled;
	};
	struct RouteState {
		PeiEvent_t Token;
		PeiEndpoint_t Task;
		PeiConn_t Conn;
	};

	int FindEvent(PeiEvent_t Evt) const
	{
		uint32_t slot;

		slot = Evt & 63U;
		if (slot == 0U || slot > (uint32_t)PEI_MAX_EVENTS)
		{
			return -1;
		}
		if (vEvents[slot - 1U].Token != Evt)
		{
			return -1;
		}
		return (int)slot - 1;
	}

	int vDevNo;
	uint32_t vGeneration;
	EventState vEvents[PEI_MAX_EVENTS];
	RouteState vRoutes[PEI_MAX_CONNECTIONS];
};

#endif // __PEI_MATRIX_H__

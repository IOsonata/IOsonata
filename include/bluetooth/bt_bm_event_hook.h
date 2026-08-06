/**-------------------------------------------------------------------------
@file	bt_bm_event_hook.h

@brief	sdk-nrf-bm application-visible connection callback tracking.

Included by app_evt_handler.h only for nRF54L sdk-nrf-bm builds after the
SoftDevice configuration is visible. A controller link that is rejected
because no BtPeer slot is available is deliberately not exposed through
BtAppEvtConnected. Its later disconnection event must therefore not be exposed
through BtAppEvtDisconnected either.
----------------------------------------------------------------------------*/
#ifndef __BT_BM_EVENT_HOOK_H__
#define __BT_BM_EVENT_HOOK_H__

#if defined(__cplusplus) && defined(CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT)

#include <stdint.h>

#include "bluetooth/bt_app.h"

#if CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT < 1
#error "CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT must be at least 1"
#endif

static uint16_t s_BtBmExposedConnHdl[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static bool s_BtBmExposedConnUsed[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];

static inline void BtBmAppEvtConnected(uint16_t ConnHdl)
{
	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_BtBmExposedConnUsed[i] &&
			s_BtBmExposedConnHdl[i] == ConnHdl)
		{
			BtAppEvtConnected(ConnHdl);
			return;
		}
	}

	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (!s_BtBmExposedConnUsed[i])
		{
			s_BtBmExposedConnHdl[i] = ConnHdl;
			s_BtBmExposedConnUsed[i] = true;
			BtAppEvtConnected(ConnHdl);
			return;
		}
	}

	// The tracker is sized from the same configured link count as the peer
	// table. Fail closed if those capacities ever diverge.
}

static inline void BtBmAppEvtDisconnected(uint16_t ConnHdl)
{
	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_BtBmExposedConnUsed[i] &&
			s_BtBmExposedConnHdl[i] == ConnHdl)
		{
			s_BtBmExposedConnUsed[i] = false;
			BtAppEvtDisconnected(ConnHdl);
			return;
		}
	}
}

#define BtAppEvtConnected(...) BtBmAppEvtConnected(__VA_ARGS__)
#define BtAppEvtDisconnected(...) BtBmAppEvtDisconnected(__VA_ARGS__)

#endif // C++ and sdk-nrf-bm link count

#endif // __BT_BM_EVENT_HOOK_H__

/**-------------------------------------------------------------------------
@file	bt_app_bm.cpp

@brief	nRF54 bare-metal Bluetooth application wrapper.

The implementation body remains in bt_app_bm_impl.inc.  This wrapper replaces
one sdk-nrf-bm Peer Manager convenience handler: a missing LTK is a recoverable
stale-bond condition, not a reason to disconnect immediately.  All other Peer
Manager failure handling remains unchanged.

----------------------------------------------------------------------------*/

// Rename only this helper while parsing the sdk-nrf-bm declaration and the
// implementation call site.  The other standard Peer Manager handlers remain
// supplied by sdk-nrf-bm.
#define pm_handler_disconnect_on_sec_failure BtPmDisconnectOnSecFailure
#include "bt_app_bm_impl.inc"
#undef pm_handler_disconnect_on_sec_failure

#if !defined(NDEBUG)
#include "syslog.h"
#define PM_DEBUG_PRINTF(...)	SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define PM_DEBUG_PRINTF(...)
#endif

enum BtPmStaleBondRepairState_t : uint8_t
{
	BT_PM_STALE_REPAIR_NONE = 0,
	BT_PM_STALE_REPAIR_WAIT_RESULT,
	BT_PM_STALE_REPAIR_QUEUED,
	BT_PM_STALE_REPAIR_RETRY,
};

static uint8_t s_StaleBondRepairState[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static uint16_t s_StaleBondRepairHdl[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static bool s_bStaleBondRepairPumpRegistered;

static void BtPmStaleBondRepair(uint32_t Evt, void *pCtx);
static void BtPmStaleBondRepairPump();

static int BtPmStaleBondRepairFind(uint16_t ConnHdl)
{
	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_StaleBondRepairState[i] != BT_PM_STALE_REPAIR_NONE &&
			s_StaleBondRepairHdl[i] == ConnHdl)
		{
			return i;
		}
	}

	return -1;
}

static int BtPmStaleBondRepairAllocate(uint16_t ConnHdl)
{
	int idx = BtPmStaleBondRepairFind(ConnHdl);
	if (idx >= 0)
	{
		return idx;
	}

	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_StaleBondRepairState[i] == BT_PM_STALE_REPAIR_NONE)
		{
			s_StaleBondRepairHdl[i] = ConnHdl;
			return i;
		}
	}

	return -1;
}

static void BtPmStaleBondRepairClear(uint16_t ConnHdl)
{
	uint32_t intState = DisableInterrupt();
	int idx = BtPmStaleBondRepairFind(ConnHdl);
	if (idx >= 0)
	{
		s_StaleBondRepairState[idx] = BT_PM_STALE_REPAIR_NONE;
		s_StaleBondRepairHdl[idx] = BLE_CONN_HANDLE_INVALID;
	}
	EnableInterrupt(intState);
}

static bool BtPmStaleBondRepairPumpEnsure()
{
	if (!s_bStaleBondRepairPumpRegistered)
	{
		s_bStaleBondRepairPumpRegistered =
			AppEvtHandlerIdleRegister(BtPmStaleBondRepairPump);
	}

	return s_bStaleBondRepairPumpRegistered;
}

static bool BtPmStaleBondRepairWait(uint16_t ConnHdl)
{
	if (!BtPmStaleBondRepairPumpEnsure())
	{
		return false;
	}

	uint32_t intState = DisableInterrupt();
	int idx = BtPmStaleBondRepairAllocate(ConnHdl);
	if (idx >= 0 && s_StaleBondRepairState[idx] == BT_PM_STALE_REPAIR_NONE)
	{
		s_StaleBondRepairState[idx] = BT_PM_STALE_REPAIR_WAIT_RESULT;
	}
	EnableInterrupt(intState);

	return idx >= 0;
}

static bool BtPmStaleBondRepairQueue(uint16_t ConnHdl)
{
	if (!BtPmStaleBondRepairPumpEnsure())
	{
		return false;
	}

	uint32_t intState = DisableInterrupt();
	int idx = BtPmStaleBondRepairAllocate(ConnHdl);
	if (idx < 0)
	{
		EnableInterrupt(intState);
		return false;
	}

	if (s_StaleBondRepairState[idx] == BT_PM_STALE_REPAIR_QUEUED ||
		s_StaleBondRepairState[idx] == BT_PM_STALE_REPAIR_RETRY)
	{
		EnableInterrupt(intState);
		return true;
	}

	s_StaleBondRepairState[idx] = BT_PM_STALE_REPAIR_QUEUED;
	EnableInterrupt(intState);

	if (AppEvtHandlerQue((uint32_t)ConnHdl, nullptr, BtPmStaleBondRepair))
	{
		return true;
	}

	// The application queue is temporarily full.  The idle pump runs after
	// queued handlers release their slots and will submit this repair again.
	intState = DisableInterrupt();
	if (s_StaleBondRepairState[idx] == BT_PM_STALE_REPAIR_QUEUED &&
		s_StaleBondRepairHdl[idx] == ConnHdl)
	{
		s_StaleBondRepairState[idx] = BT_PM_STALE_REPAIR_RETRY;
	}
	EnableInterrupt(intState);

	return true;
}

static void BtPmStaleBondRepair(uint32_t Evt, void *pCtx)
{
	(void)pCtx;
	uint16_t connHdl = (uint16_t)Evt;
	uint32_t intState = DisableInterrupt();
	int idx = BtPmStaleBondRepairFind(connHdl);

	if (idx < 0 || s_StaleBondRepairState[idx] != BT_PM_STALE_REPAIR_QUEUED)
	{
		EnableInterrupt(intState);
		return;
	}

	// RETRY also marks the slot as in progress, preventing a duplicate local
	// or remote failure report from enqueuing another callback concurrently.
	s_StaleBondRepairState[idx] = BT_PM_STALE_REPAIR_RETRY;
	EnableInterrupt(intState);

	if (BtPeerFindByHdl(connHdl) == nullptr)
	{
		BtPmStaleBondRepairClear(connHdl);
		return;
	}

	uint32_t err = pm_conn_secure(connHdl, true);
	PM_DEBUG_PRINTF("BM SEC: stale bond repair hdl=%u status=0x%08lx\r\n",
			(unsigned)connHdl, (unsigned long)err);

	if (err == NRF_SUCCESS)
	{
		BtPmStaleBondRepairClear(connHdl);
		return;
	}

	if (err == NRF_ERROR_BUSY || err == NRF_ERROR_INVALID_STATE)
	{
		return;	// idle pump retries once on the next main-loop pass
	}

	BtPmStaleBondRepairClear(connHdl);
	(void)sd_ble_gap_disconnect(connHdl,
								BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

static void BtPmStaleBondRepairPump()
{
	uint16_t connHdl = BLE_CONN_HANDLE_INVALID;
	int idx = -1;
	uint32_t intState = DisableInterrupt();

	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_StaleBondRepairState[i] == BT_PM_STALE_REPAIR_RETRY)
		{
			idx = i;
			connHdl = s_StaleBondRepairHdl[i];
			s_StaleBondRepairState[i] = BT_PM_STALE_REPAIR_QUEUED;
			break;
		}
	}
	EnableInterrupt(intState);

	if (idx < 0)
	{
		return;
	}

	if (!AppEvtHandlerQue((uint32_t)connHdl, nullptr, BtPmStaleBondRepair))
	{
		intState = DisableInterrupt();
		if (s_StaleBondRepairState[idx] == BT_PM_STALE_REPAIR_QUEUED &&
			s_StaleBondRepairHdl[idx] == connHdl)
		{
			s_StaleBondRepairState[idx] = BT_PM_STALE_REPAIR_RETRY;
		}
		EnableInterrupt(intState);
	}
}

extern "C" void BtPmDisconnectOnSecFailure(const struct pm_evt *pEvt)
{
	if (pEvt == nullptr)
	{
		return;
	}

	if (pEvt->evt_id == PM_EVT_CONN_SEC_SUCCEEDED)
	{
		BtPmStaleBondRepairClear(pEvt->conn_handle);
		return;
	}

	if (pEvt->evt_id != PM_EVT_CONN_SEC_FAILED)
	{
		return;
	}

	if (pEvt->conn_sec_failed.procedure == PM_CONN_SEC_PROCEDURE_ENCRYPTION &&
		pEvt->conn_sec_failed.error == PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING)
	{
		if (pEvt->conn_sec_failed.error_src == BLE_GAP_SEC_STATUS_SOURCE_LOCAL)
		{
			// SEC_INFO_REQUEST is not the end of the failed encryption procedure.
			// Record it and wait for S145's unencrypted CONN_SEC_UPDATE.
			if (BtPmStaleBondRepairWait(pEvt->conn_handle))
			{
				PM_DEBUG_PRINTF("BM SEC: stale bond wait hdl=%u\r\n",
						(unsigned)pEvt->conn_handle);
				return;
			}
		}
		else if (BtPmStaleBondRepairQueue(pEvt->conn_handle))
		{
			PM_DEBUG_PRINTF("BM SEC: stale bond repair queued hdl=%u src=%u\r\n",
					(unsigned)pEvt->conn_handle,
					(unsigned)pEvt->conn_sec_failed.error_src);
			return;
		}
	}

	BtPmStaleBondRepairClear(pEvt->conn_handle);

	// Queue exhaustion and every non-recoverable security failure retain the
	// stock fail-closed behavior.
	uint32_t err = sd_ble_gap_disconnect(
		pEvt->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	if (err != NRF_SUCCESS && err != NRF_ERROR_INVALID_STATE)
	{
		PM_DEBUG_PRINTF("BM SEC: failure disconnect hdl=%u status=0x%08lx\r\n",
				(unsigned)pEvt->conn_handle, (unsigned long)err);
	}
}

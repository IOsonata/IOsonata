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

#define BT_PM_STALE_REPAIR_DELAY_MS		20U
#define BT_PM_STALE_REPAIR_TIMEOUT_MS	2000U

static bool s_StaleBondRepairActive[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static uint16_t s_StaleBondRepairHdl[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static uint32_t s_StaleBondRepairNextMs[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static uint32_t s_StaleBondRepairDeadlineMs[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static bool s_bStaleBondRepairPumpRegistered;

static void BtPmStaleBondRepairPump();

static bool BtPmTimeReached(uint32_t Now, uint32_t Target)
{
	return (int32_t)(Now - Target) >= 0;
}

static int BtPmStaleBondRepairFind(uint16_t ConnHdl)
{
	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_StaleBondRepairActive[i] &&
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
		if (!s_StaleBondRepairActive[i])
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
		s_StaleBondRepairActive[idx] = false;
		s_StaleBondRepairHdl[idx] = BLE_CONN_HANDLE_INVALID;
		s_StaleBondRepairNextMs[idx] = 0;
		s_StaleBondRepairDeadlineMs[idx] = 0;
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

static bool BtPmStaleBondRepairSchedule(uint16_t ConnHdl)
{
	if (!BtPmStaleBondRepairPumpEnsure())
	{
		return false;
	}

	uint32_t now = s_BtAppSdGrtc3.mSecond();
	uint32_t intState = DisableInterrupt();
	int idx = BtPmStaleBondRepairAllocate(ConnHdl);

	if (idx >= 0)
	{
		if (!s_StaleBondRepairActive[idx])
		{
			s_StaleBondRepairDeadlineMs[idx] =
				now + BT_PM_STALE_REPAIR_TIMEOUT_MS;
		}

		s_StaleBondRepairHdl[idx] = ConnHdl;
		s_StaleBondRepairNextMs[idx] = now + BT_PM_STALE_REPAIR_DELAY_MS;
		s_StaleBondRepairActive[idx] = true;
	}

	EnableInterrupt(intState);
	return idx >= 0;
}

static void BtPmStaleBondRepairPump()
{
	uint32_t now = s_BtAppSdGrtc3.mSecond();
	uint16_t connHdl = BLE_CONN_HANDLE_INVALID;
	bool expired = false;
	uint32_t intState = DisableInterrupt();

	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (!s_StaleBondRepairActive[i])
		{
			continue;
		}

		if (BtPmTimeReached(now, s_StaleBondRepairDeadlineMs[i]))
		{
			connHdl = s_StaleBondRepairHdl[i];
			s_StaleBondRepairActive[i] = false;
			s_StaleBondRepairHdl[i] = BLE_CONN_HANDLE_INVALID;
			expired = true;
			break;
		}

		if (BtPmTimeReached(now, s_StaleBondRepairNextMs[i]))
		{
			connHdl = s_StaleBondRepairHdl[i];
			s_StaleBondRepairNextMs[i] =
				now + BT_PM_STALE_REPAIR_DELAY_MS;
			break;
		}
	}

	EnableInterrupt(intState);

	if (connHdl == BLE_CONN_HANDLE_INVALID)
	{
		return;
	}

	if (expired)
	{
		PM_DEBUG_PRINTF("BM SEC: stale bond repair timeout hdl=%u\r\n",
				(unsigned)connHdl);
		(void)sd_ble_gap_disconnect(connHdl,
								BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		return;
	}

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
		return;	// retry after BT_PM_STALE_REPAIR_DELAY_MS
	}

	BtPmStaleBondRepairClear(connHdl);
	(void)sd_ble_gap_disconnect(connHdl,
								BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
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
		// S145 does not reliably emit a terminal CONN_SEC_UPDATE after replying
		// without an LTK. Retry forced pairing from application context after
		// the failed encryption request has had time to unwind.
		if (BtPmStaleBondRepairSchedule(pEvt->conn_handle))
		{
			PM_DEBUG_PRINTF("BM SEC: stale bond repair scheduled hdl=%u src=%u\r\n",
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

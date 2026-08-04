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

static bool s_StaleBondRepairPending[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static uint16_t s_StaleBondRepairHdl[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];

static bool BtPmStaleBondRepairQueue(uint16_t ConnHdl);

static void BtPmStaleBondRepair(uint32_t Evt, void *pCtx)
{
	(void)pCtx;
	uint16_t connHdl = (uint16_t)Evt;
	bool found = false;
	uint32_t intState = DisableInterrupt();

	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_StaleBondRepairPending[i] &&
			s_StaleBondRepairHdl[i] == connHdl)
		{
			s_StaleBondRepairPending[i] = false;
			found = true;
			break;
		}
	}

	EnableInterrupt(intState);

	if (!found || BtPeerFindByHdl(connHdl) == nullptr)
	{
		return;
	}

	uint32_t err = pm_conn_secure(connHdl, true);
	PM_DEBUG_PRINTF("BM SEC: stale bond repair hdl=%u status=0x%08lx\r\n",
			(unsigned)connHdl, (unsigned long)err);

	if (err != NRF_SUCCESS && err != NRF_ERROR_BUSY &&
		err != NRF_ERROR_INVALID_STATE)
	{
		(void)sd_ble_gap_disconnect(connHdl,
									BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	}
}

static bool BtPmStaleBondRepairQueue(uint16_t ConnHdl)
{
	int freeIdx = -1;
	uint32_t intState = DisableInterrupt();

	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_StaleBondRepairPending[i])
		{
			if (s_StaleBondRepairHdl[i] == ConnHdl)
			{
				EnableInterrupt(intState);
				return true;	// local and remote missing-key reports may both arrive
			}
		}
		else if (freeIdx < 0)
		{
			freeIdx = i;
		}
	}

	if (freeIdx >= 0)
	{
		s_StaleBondRepairHdl[freeIdx] = ConnHdl;
		s_StaleBondRepairPending[freeIdx] = true;
	}

	EnableInterrupt(intState);

	if (freeIdx < 0)
	{
		return false;
	}

	if (AppEvtHandlerQue((uint32_t)ConnHdl, nullptr, BtPmStaleBondRepair))
	{
		return true;
	}

	intState = DisableInterrupt();
	if (s_StaleBondRepairPending[freeIdx] &&
		s_StaleBondRepairHdl[freeIdx] == ConnHdl)
	{
		s_StaleBondRepairPending[freeIdx] = false;
	}
	EnableInterrupt(intState);

	return false;
}

extern "C" void BtPmDisconnectOnSecFailure(const struct pm_evt *pEvt)
{
	if (pEvt == nullptr || pEvt->evt_id != PM_EVT_CONN_SEC_FAILED)
	{
		return;
	}

	if (pEvt->conn_sec_failed.procedure == PM_CONN_SEC_PROCEDURE_ENCRYPTION &&
		pEvt->conn_sec_failed.error == PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING)
	{
		// Either side can discover the stale bond first.  Let the failed
		// encryption event unwind, then request fresh pairing from application
		// context.  Duplicate local/remote failure reports share one request.
		if (BtPmStaleBondRepairQueue(pEvt->conn_handle))
		{
			PM_DEBUG_PRINTF(
				"BM SEC: stale bond repair queued hdl=%u src=%u\r\n",
				(unsigned)pEvt->conn_handle,
				(unsigned)pEvt->conn_sec_failed.error_src);
			return;
		}
	}

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

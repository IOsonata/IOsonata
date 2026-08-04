/**-------------------------------------------------------------------------
@file	bt_app_bm.cpp

@brief	nRF54 bare-metal Bluetooth application wrapper.

The implementation body remains in bt_app_bm_impl.inc.  This wrapper replaces
one sdk-nrf-bm Peer Manager convenience handler: a locally missing LTK is a
recoverable stale-bond condition, not a reason to disconnect immediately.
All other Peer Manager failure handling remains unchanged.

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

static void BtPmStaleBondRepair(uint32_t Evt, void *pCtx)
{
	(void)pCtx;
	uint16_t connHdl = (uint16_t)Evt;

	if (!pm_conn_state_valid(connHdl))
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

extern "C" void BtPmDisconnectOnSecFailure(const struct pm_evt *pEvt)
{
	if (pEvt == nullptr || pEvt->evt_id != PM_EVT_CONN_SEC_FAILED)
	{
		return;
	}

	if (pEvt->conn_sec_failed.procedure == PM_CONN_SEC_PROCEDURE_ENCRYPTION &&
		pEvt->conn_sec_failed.error == PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING &&
		pEvt->conn_sec_failed.error_src == BLE_GAP_SEC_STATUS_SOURCE_LOCAL)
	{
		// The peer attempted to use a bond that is absent locally.  Let the
		// failed encryption event unwind, then request fresh pairing from the
		// application context.  Do not disconnect the still-valid link.
		if (AppEvtHandlerQue((uint32_t)pEvt->conn_handle, nullptr,
							  BtPmStaleBondRepair))
		{
			PM_DEBUG_PRINTF("BM SEC: stale bond repair queued hdl=%u\r\n",
					(unsigned)pEvt->conn_handle);
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

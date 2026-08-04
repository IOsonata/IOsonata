/**-------------------------------------------------------------------------
@file	bt_app_bm.cpp

@brief	nRF54 bare-metal Bluetooth application wrapper.

The implementation body remains in bt_app_bm_impl.inc.  This wrapper replaces
one sdk-nrf-bm Peer Manager convenience handler: a missing remote LTK is not
immediately disconnected, giving the central a bounded opportunity to replace
its stale bond.  All other Peer Manager failure handling remains unchanged.

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

// A peripheral cannot send an SMP Pairing Request.  After replying without an
// LTK, allow the central time to abandon its failed encryption procedure and
// initiate fresh pairing.  If it does not, close the unusable unencrypted link.
#define BT_PM_STALE_BOND_GRACE_MS	2000U

static bool s_StaleBondActive[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static uint16_t s_StaleBondHdl[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static uint32_t s_StaleBondDeadlineMs[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static bool s_bStaleBondPumpRegistered;

static void BtPmStaleBondPump();

static bool BtPmTimeReached(uint32_t Now, uint32_t Target)
{
	return (int32_t)(Now - Target) >= 0;
}

static int BtPmStaleBondFind(uint16_t ConnHdl)
{
	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_StaleBondActive[i] && s_StaleBondHdl[i] == ConnHdl)
		{
			return i;
		}
	}

	return -1;
}

static int BtPmStaleBondAllocate(uint16_t ConnHdl)
{
	int idx = BtPmStaleBondFind(ConnHdl);
	if (idx >= 0)
	{
		return idx;
	}

	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (!s_StaleBondActive[i])
		{
			s_StaleBondHdl[i] = ConnHdl;
			return i;
		}
	}

	return -1;
}

static bool BtPmStaleBondIsActive(uint16_t ConnHdl)
{
	uint32_t intState = DisableInterrupt();
	bool active = BtPmStaleBondFind(ConnHdl) >= 0;
	EnableInterrupt(intState);

	return active;
}

static void BtPmStaleBondClear(uint16_t ConnHdl)
{
	uint32_t intState = DisableInterrupt();
	int idx = BtPmStaleBondFind(ConnHdl);
	if (idx >= 0)
	{
		s_StaleBondActive[idx] = false;
		s_StaleBondHdl[idx] = BLE_CONN_HANDLE_INVALID;
		s_StaleBondDeadlineMs[idx] = 0;
	}
	EnableInterrupt(intState);
}

static bool BtPmStaleBondPumpEnsure()
{
	if (!s_bStaleBondPumpRegistered)
	{
		s_bStaleBondPumpRegistered =
			AppEvtHandlerIdleRegister(BtPmStaleBondPump);
	}

	return s_bStaleBondPumpRegistered;
}

static bool BtPmStaleBondSchedule(uint16_t ConnHdl)
{
	if (!BtPmStaleBondPumpEnsure())
	{
		return false;
	}

	uint32_t now = s_BtAppSdGrtc3.mSecond();
	uint32_t intState = DisableInterrupt();
	int idx = BtPmStaleBondAllocate(ConnHdl);

	if (idx >= 0)
	{
		s_StaleBondHdl[idx] = ConnHdl;
		s_StaleBondDeadlineMs[idx] = now + BT_PM_STALE_BOND_GRACE_MS;
		s_StaleBondActive[idx] = true;
	}

	EnableInterrupt(intState);
	return idx >= 0;
}

static void BtPmStaleBondPump()
{
	uint32_t now = s_BtAppSdGrtc3.mSecond();
	uint16_t connHdl = BLE_CONN_HANDLE_INVALID;
	uint32_t intState = DisableInterrupt();

	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_StaleBondActive[i] &&
			BtPmTimeReached(now, s_StaleBondDeadlineMs[i]))
		{
			connHdl = s_StaleBondHdl[i];
			s_StaleBondActive[i] = false;
			s_StaleBondHdl[i] = BLE_CONN_HANDLE_INVALID;
			s_StaleBondDeadlineMs[i] = 0;
			break;
		}
	}

	EnableInterrupt(intState);

	if (connHdl == BLE_CONN_HANDLE_INVALID)
	{
		return;
	}

	if (BtPeerFindByHdl(connHdl) == nullptr)
	{
		return;
	}

	PM_DEBUG_PRINTF(
		"BM SEC: stale remote bond timeout hdl=%u; remove central bond\r\n",
		(unsigned)connHdl);
	(void)sd_ble_gap_disconnect(connHdl,
								BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

// The Peer Manager failure event does not carry the eventual GAP disconnect
// reason.  Keep a direct trace so stale-bond behavior can be distinguished
// from a local timeout or an unrelated link loss.
static void BtPmStaleBondBleEvt(const ble_evt_t *pBleEvt, void *pContext)
{
	(void)pContext;
	if (pBleEvt == nullptr || pBleEvt->header.evt_id != BLE_GAP_EVT_DISCONNECTED)
	{
		return;
	}

	uint16_t connHdl = pBleEvt->evt.gap_evt.conn_handle;
	bool stale = BtPmStaleBondIsActive(connHdl);
	uint8_t reason = pBleEvt->evt.gap_evt.params.disconnected.reason;

	PM_DEBUG_PRINTF("BM SEC: disconnected hdl=%u reason=0x%02x stale=%u\r\n",
			(unsigned)connHdl, (unsigned)reason, stale ? 1U : 0U);
	BtPmStaleBondClear(connHdl);
}

NRF_SDH_BLE_OBSERVER(s_BtPmStaleBondObserver,
					BtPmStaleBondBleEvt, NULL, USER);

extern "C" void BtPmDisconnectOnSecFailure(const struct pm_evt *pEvt)
{
	if (pEvt == nullptr)
	{
		return;
	}

	if (pEvt->evt_id == PM_EVT_CONN_SEC_SUCCEEDED)
	{
		BtPmStaleBondClear(pEvt->conn_handle);
		return;
	}

	if (pEvt->evt_id == PM_EVT_CONN_SEC_START &&
		pEvt->conn_sec_start.procedure != PM_CONN_SEC_PROCEDURE_ENCRYPTION)
	{
		if (BtPmStaleBondIsActive(pEvt->conn_handle))
		{
			PM_DEBUG_PRINTF("BM SEC: stale remote bond replacement started hdl=%u\r\n",
					(unsigned)pEvt->conn_handle);
		}
		BtPmStaleBondClear(pEvt->conn_handle);
		return;
	}

	if (pEvt->evt_id != PM_EVT_CONN_SEC_FAILED)
	{
		return;
	}

	if (pEvt->conn_sec_failed.procedure == PM_CONN_SEC_PROCEDURE_ENCRYPTION &&
		pEvt->conn_sec_failed.error == PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING)
	{
		if (BtPmStaleBondSchedule(pEvt->conn_handle))
		{
			PM_DEBUG_PRINTF(
				"BM SEC: stale remote bond hdl=%u src=%u; waiting for central repair\r\n",
				(unsigned)pEvt->conn_handle,
				(unsigned)pEvt->conn_sec_failed.error_src);
			return;
		}
	}

	BtPmStaleBondClear(pEvt->conn_handle);

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

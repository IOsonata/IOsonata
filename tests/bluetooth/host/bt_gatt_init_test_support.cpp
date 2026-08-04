// Host-test implementation of the production GATT initialization status API.
// Production builds get the same implementation from bt_gap.cpp. Tests that
// isolate DIS or an extracted BtAppInit body do not link the full GAP module.

#include <atomic>

#include "bluetooth/bt_gatt_init.h"

static std::atomic<bool> s_Active;
static std::atomic<uint8_t> s_Error;

extern "C" {

void BtGattInitStatusReset(void)
{
	s_Error.store(BT_GATT_INIT_ERROR_NONE, std::memory_order_relaxed);
	s_Active.store(true, std::memory_order_release);
}

void BtGattInitStatusFail(BtGattInitError_t Error)
{
	if (Error == BT_GATT_INIT_ERROR_NONE ||
		!s_Active.load(std::memory_order_acquire))
	{
		return;
	}

	uint8_t expected = BT_GATT_INIT_ERROR_NONE;
	s_Error.compare_exchange_strong(expected, (uint8_t)Error,
		std::memory_order_acq_rel, std::memory_order_acquire);
}

bool BtGattInitStatusActive(void)
{
	return s_Active.load(std::memory_order_acquire);
}

bool BtGattInitStatusOk(void)
{
	return s_Error.load(std::memory_order_acquire) ==
		BT_GATT_INIT_ERROR_NONE;
}

bool BtGattInitStatusComplete(void)
{
	bool ok = BtGattInitStatusOk();
	s_Active.store(false, std::memory_order_release);
	return ok;
}

BtGattInitError_t BtGattInitStatusErrorGet(void)
{
	return (BtGattInitError_t)s_Error.load(std::memory_order_acquire);
}

} // extern "C"

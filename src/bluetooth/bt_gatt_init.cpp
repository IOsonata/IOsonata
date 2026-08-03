/**-------------------------------------------------------------------------
@file	bt_gatt_init.cpp

@brief	GATT database initialization status.

@author	Hoang Nguyen Hoan
@date	Aug. 3, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <atomic>

#include "bluetooth/bt_gatt_init.h"

static std::atomic<bool> s_BtGattInitActive;
static std::atomic<uint8_t> s_BtGattInitError;

void BtGattInitStatusReset(void)
{
	s_BtGattInitError.store(BT_GATT_INIT_ERROR_NONE,
			std::memory_order_relaxed);
	s_BtGattInitActive.store(true, std::memory_order_release);
}

void BtGattInitStatusFail(BtGattInitError_t Error)
{
	if (Error == BT_GATT_INIT_ERROR_NONE ||
		!s_BtGattInitActive.load(std::memory_order_acquire))
	{
		return;
	}

	uint8_t expected = BT_GATT_INIT_ERROR_NONE;
	s_BtGattInitError.compare_exchange_strong(expected, (uint8_t)Error,
			std::memory_order_acq_rel, std::memory_order_acquire);
}

bool BtGattInitStatusActive(void)
{
	return s_BtGattInitActive.load(std::memory_order_acquire);
}

bool BtGattInitStatusOk(void)
{
	return s_BtGattInitError.load(std::memory_order_acquire) ==
		BT_GATT_INIT_ERROR_NONE;
}

bool BtGattInitStatusComplete(void)
{
	bool ok = BtGattInitStatusOk();
	s_BtGattInitActive.store(false, std::memory_order_release);
	return ok;
}

BtGattInitError_t BtGattInitStatusErrorGet(void)
{
	return (BtGattInitError_t)s_BtGattInitError.load(
			std::memory_order_acquire);
}

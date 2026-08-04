/**-------------------------------------------------------------------------
@file	bt_hci_gatt_test_stubs.cpp

@brief	GATT TX accounting stubs for HCI-only host tests.

		The HCI flow and compliance executables intentionally do not link the
		generic GATT implementation. These strong test definitions satisfy the
		HCI host's TX accounting references without adding test behavior to the
		production HCI object.

@author	Hoang Nguyen Hoan
@date	August 2026

@license MIT, (c) 2026 I-SYST. See bt_gatt.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "bluetooth/bt_gatt.h"

void BtGattTxPendRelease(uint16_t ConnHdl)
{
	(void)ConnHdl;
}

bool BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt)
{
	(void)ConnHdl;
	return NbPkt > 0;
}

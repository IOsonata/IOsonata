/**-------------------------------------------------------------------------
@file	bt_adv_select.cpp

@brief	Bluetooth advertising packet type selection

@license
MIT License

Copyright (c) 2026 I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/
#include "bluetooth/bt_adv.h"

bool BtAdvUseExtended(size_t AdvLen, size_t SrLen)
{
	return AdvLen > BT_ADV_LEGACY_DATA_MAX || SrLen > BT_ADV_LEGACY_DATA_MAX;
}

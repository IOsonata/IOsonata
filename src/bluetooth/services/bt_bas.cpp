/**-------------------------------------------------------------------------
@file	bt_bas.cpp

@brief	Bluetooth Battery Service (BAS) - generic implementation.

Builds a standard BAS instance using the generic GATT layer. The service exposes
Battery Level (0x2A19) as a single uint8 percentage value, readable and
notifiable.

@author	Hoang Nguyen Hoan
@date	Jun. 28, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdint.h>

#include "istddef.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/services/bt_bas.h"

static uint8_t s_BtBasLevel = 100;
static bool s_BtBasRegistered = false;

enum {
	BT_BAS_CHAR_IDX_BATTERY_LEVEL = 0,
	BT_BAS_CHAR_COUNT,
};

static BtGattChar_t s_BtBasChar[] = {
	BT_CHAR(BT_UUID_CHARACTERISTIC_BATTERY_LEVEL,
			sizeof(uint8_t),
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
			"Battery Level"),
};

static BtGattSrvc_t s_BtBasSrvc = BT_SRVC_STD(BT_UUID_GATT_SERVICE_BATTERY,
											  s_BtBasChar);

static uint8_t BtBasClampLevel(uint8_t Level)
{
	return Level > 100 ? 100 : Level;
}

bool BtBasInit(uint8_t Level)
{
	s_BtBasLevel = BtBasClampLevel(Level);

	if (s_BtBasRegistered == false)
	{
		if (BtGattSrvcAdd(&s_BtBasSrvc) == false)
		{
			return false;
		}
		s_BtBasRegistered = true;
	}

	return BtGattCharSetValue(&s_BtBasChar[BT_BAS_CHAR_IDX_BATTERY_LEVEL],
							  &s_BtBasLevel, sizeof(s_BtBasLevel));
}

bool BtBasSetLevel(uint16_t ConnHdl, uint8_t Level, bool bNotify)
{
	s_BtBasLevel = BtBasClampLevel(Level);

	if (BtGattCharSetValue(&s_BtBasChar[BT_BAS_CHAR_IDX_BATTERY_LEVEL],
						   &s_BtBasLevel, sizeof(s_BtBasLevel)) == false)
	{
		return false;
	}

	if (bNotify)
	{
		return BtGattCharNotify(ConnHdl,
								&s_BtBasChar[BT_BAS_CHAR_IDX_BATTERY_LEVEL],
								&s_BtBasLevel,
								sizeof(s_BtBasLevel));
	}

	return true;
}

uint8_t BtBasGetLevel(void)
{
	return s_BtBasLevel;
}

BtGattChar_t *BtBasBatteryLevelChar(void)
{
	return &s_BtBasChar[BT_BAS_CHAR_IDX_BATTERY_LEVEL];
}

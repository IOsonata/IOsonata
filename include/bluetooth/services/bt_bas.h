/**-------------------------------------------------------------------------
@file	bt_bas.h

@brief	Bluetooth Battery Service (BAS) - service UUID 0x180F.

Generic implementation exposing Battery Level (0x2A19), read + notify.

@author	Hoang Nguyen Hoan
@date	Jun. 28, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#ifndef __BT_BAS_H__
#define __BT_BAS_H__

#include <stdint.h>
#include <stdbool.h>

#include "bluetooth/bt_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif

bool BtBasInit(uint8_t Level);
bool BtBasSetLevel(uint16_t ConnHdl, uint8_t Level, bool bNotify);
uint8_t BtBasGetLevel(void);
BtGattChar_t *BtBasBatteryLevelChar(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_BAS_H__

/**-------------------------------------------------------------------------
@file	bt_gatt_init.h

@brief	Source-compatible GATT database initialization status.

Service registration hooks used by applications return void, so an ignored
BtGattSrvcAdd or initial value failure used to let BtAppInit continue with an
incomplete database. Tracking is active only during application startup; normal
runtime GATT errors do not poison later advertising operations.

@author	Hoang Nguyen Hoan
@date	Aug. 3, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __BT_GATT_INIT_H__
#define __BT_GATT_INIT_H__

#include <stdbool.h>
#include <stdint.h>

typedef enum __Bt_Gatt_Init_Error {
	BT_GATT_INIT_ERROR_NONE = 0,
	BT_GATT_INIT_ERROR_SERVICE_ADD,
	BT_GATT_INIT_ERROR_VALUE_SET,
	BT_GATT_INIT_ERROR_INVALID_CFG,
} BtGattInitError_t;

#ifdef __cplusplus
extern "C" {
#endif

void BtGattInitStatusReset(void);
void BtGattInitStatusFail(BtGattInitError_t Error);
bool BtGattInitStatusActive(void);
bool BtGattInitStatusOk(void);
bool BtGattInitStatusComplete(void);
BtGattInitError_t BtGattInitStatusErrorGet(void);

#ifdef __cplusplus
}
#endif

#endif

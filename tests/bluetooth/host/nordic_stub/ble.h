#ifndef __BT_TEST_BLE_H__
#define __BT_TEST_BLE_H__

#include <stdint.h>

#include "ble_gap.h"

typedef struct {
	uint16_t evt_id;
} ble_evt_header_t;

typedef union {
	ble_gap_evt_t gap_evt;
} ble_evt_data_t;

typedef struct {
	ble_evt_header_t header;
	ble_evt_data_t evt;
} ble_evt_t;

#endif

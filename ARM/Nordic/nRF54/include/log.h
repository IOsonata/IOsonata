/**-------------------------------------------------------------------------
@file	zephyr/logging/log.h

@brief	Zephyr logging shim for bare-metal sdk-nrf-bm build.

Routes Zephyr logging include to bm_compat and injects PHY config shim so
files like ble_conn_params/phy_mode.c get CONFIG_BLE_CONN_PARAMS_PHY_*.

@license MIT
----------------------------------------------------------------------------*/

#ifndef ZEPHYR_LOGGING_LOG_H__
#define ZEPHYR_LOGGING_LOG_H__

#include "bluetooth/bm_config_defaults.h"
#include "bluetooth/bm_compat_phy_shim.h"
#include "bluetooth/bm_compat.h"

#endif /* ZEPHYR_LOGGING_LOG_H__ */

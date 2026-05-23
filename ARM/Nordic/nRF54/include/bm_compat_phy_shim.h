/**-------------------------------------------------------------------------
@file	bm_compat_phy_shim.h

@brief	Compatibility shim for BLE PHY Kconfig-style symbols.

This header defines CONFIG_BLE_CONN_PARAMS_PHY_* symbols expected by
sdk-nrf-bm phy_mode.c when only CONFIG_BLE_CONN_PARAMS_PHY mask is present.

Include this before compiling sdk-nrf-bm BLE conn params sources, or include
it from bm_compat.h right after bm_config_defaults.h.

@license MIT
----------------------------------------------------------------------------*/

#ifndef BM_COMPAT_PHY_SHIM_H__
#define BM_COMPAT_PHY_SHIM_H__

/*
 * Expected by sdk-nrf-bm/lib/bluetooth/ble_conn_params/phy_mode.c:
 *   CONFIG_BLE_CONN_PARAMS_PHY_1MBPS
 *   CONFIG_BLE_CONN_PARAMS_PHY_2MBPS
 *   CONFIG_BLE_CONN_PARAMS_PHY_CODED
 *   CONFIG_BLE_CONN_PARAMS_PHY_AUTO
 */
#if !defined(CONFIG_BLE_CONN_PARAMS_PHY_1MBPS) && \
    !defined(CONFIG_BLE_CONN_PARAMS_PHY_2MBPS) && \
    !defined(CONFIG_BLE_CONN_PARAMS_PHY_CODED) && \
    !defined(CONFIG_BLE_CONN_PARAMS_PHY_AUTO)

  #if defined(CONFIG_BLE_CONN_PARAMS_PHY)

    #if ((CONFIG_BLE_CONN_PARAMS_PHY) & 0x01)
      #define CONFIG_BLE_CONN_PARAMS_PHY_1MBPS 1
    #endif

    #if ((CONFIG_BLE_CONN_PARAMS_PHY) & 0x02)
      #define CONFIG_BLE_CONN_PARAMS_PHY_2MBPS 1
    #endif

    #if ((CONFIG_BLE_CONN_PARAMS_PHY) & 0x04)
      #define CONFIG_BLE_CONN_PARAMS_PHY_CODED 1
    #endif

    #if !defined(CONFIG_BLE_CONN_PARAMS_PHY_1MBPS) && \
        !defined(CONFIG_BLE_CONN_PARAMS_PHY_2MBPS) && \
        !defined(CONFIG_BLE_CONN_PARAMS_PHY_CODED)
      #define CONFIG_BLE_CONN_PARAMS_PHY_AUTO 1
    #endif

  #else

    #define CONFIG_BLE_CONN_PARAMS_PHY_AUTO 1

  #endif
#endif

#endif /* BM_COMPAT_PHY_SHIM_H__ */

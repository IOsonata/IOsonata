/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

  /** @file Support functions
 *
 * @defgroup support_func Support functions
 * @{
 * @ingroup util
 * @brief Support functions.
 *
 */

#ifndef __SUPPORT_FUNC_H__
#define __SUPPORT_FUNC_H__

#include "sdk_errors.h"
#include "drv_ext_gpio.h"
#include <stdbool.h>

#define SUPPORT_FUNC_MAC_ADDR_STR_LEN ( (BLE_GAP_ADDR_LEN * 2) + 5 + 1) /**< 6 bytes + 5 colon separators + NUL termination */

/**@brief Function for printing the MAC addess of the device.
 *
 * @note The SoftDevice must be enabled before this call is made.
 *
 * @param[out] p_mac_addr    Pointer to char array of length SUPPORT_FUNC_MAC_ADDR_STR_LEN.
 *
 * @return NRF_SUCCESS                          If the call was successful.
 * @return NRF_ERROR_SOFTDEVICE_NOT_ENABLED     The SoftDevice has not been enabled.
 * @return Other codes from the underlying drivers.
 */
ret_code_t support_func_ble_mac_address_get(char * p_mac_addr);

/**@brief Function for checking if the device is in debug mode.
 *
 * @note Checks if the C_DEBUGEN flag is set in the DHCSR register (Debug Halting Control and Status Register).
 *
 * @return True     System is in debug mode.
 * @return False    System is not in debug mode.
 */
bool support_func_sys_halt_debug_enabled(void);

/**@brief Function for configuring nRF IO and the GPIO extender for startup.
 *
 * @param[in] p_ext_gpio_init    Pointer to IO extender configuration.
 *
 * @return NRF_SUCCESS           If the call was successful.
 * @return Other codes from the underlying drivers.
 */
ret_code_t support_func_configure_io_startup(drv_ext_gpio_init_t const * const p_ext_gpio_init);

/**@brief Function for configuring nRF IO and the GPIO extender for shutdown.
 *
 * @note For Thingy HW v1.0.0, this function will only return NRF_SUCCESS as errors are ignored.
 *
 * @return NRF_SUCCESS           If the call was successful.
 * @return Other codes from the underlying drivers.
 */
ret_code_t support_func_configure_io_shutdown(void);

#endif

/** @} */

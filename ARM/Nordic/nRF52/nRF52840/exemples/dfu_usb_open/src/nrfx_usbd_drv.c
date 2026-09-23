/**
 * @file nrfx_usbd_drv.c
 *
 * @brief Compilation wrapper for the nRF5 SDK nrfx USBD peripheral driver.
 *
 * The app_usbd library (app_usbd.c, app_usbd_core.c, app_usbd_cdc_acm.c) and
 * the DFU USB serial transport (nrf_dfu_serial_usb.c) call the nrfx_usbd_*
 * driver API, but nrfx_usbd.c was not part of this project, producing
 * "undefined reference to nrfx_usbd_*" link errors.
 *
 * This translation unit compiles the SDK driver in place. The quoted include
 * below is resolved by GCC against every -I directory; since
 * <nRF5_SDK>/modules/nrfx/drivers/include is already on the include path
 * (it provides nrfx_usbd.h), "../src/nrfx_usbd.c" resolves to
 * <nRF5_SDK>/modules/nrfx/drivers/src/nrfx_usbd.c.
 *
 * The driver body is guarded internally by NRFX_CHECK(NRFX_USBD_ENABLED), so
 * NRFX_USBD_ENABLED (or legacy USBD_ENABLED) must be 1 in sdk_config.h.
 */

#include "sdk_config.h"
#include "nrfx.h"

#if !NRFX_CHECK(NRFX_USBD_ENABLED)
#error "NRFX_USBD_ENABLED must be set to 1 in sdk_config.h for the USB DFU transport"
#endif

#include "../src/nrfx_usbd.c"

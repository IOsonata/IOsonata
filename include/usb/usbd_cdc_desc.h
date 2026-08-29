/**-------------------------------------------------------------------------
@file	usbd_cdc_desc.h

@brief	Native USB CDC ACM descriptor provider.

Builds the default IOsonata CDC ACM device, configuration and string
descriptors from UsbDevCfg_t using the common USB wire definitions.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __USBD_CDC_DESC_H__
#define __USBD_CDC_DESC_H__

#include "usb/usbd_core.h"

/** @addtogroup USBD
  * @{
  */

// Default CDC topology. One function owns two consecutive interfaces, one
// interrupt IN endpoint number and one bidirectional Bulk endpoint number.
// Endpoint 15 is the last USB endpoint number, so this numbering fits seven
// CDC functions on a controller with enough endpoints.
#define USBD_CDC_FUNC_MAXCNT			7
#define USBD_CDC_CTRL_INTRF(n)			((uint8_t)((n) * 2U))
#define USBD_CDC_DATA_INTRF(n)			((uint8_t)(USBD_CDC_CTRL_INTRF(n) + 1U))
#define USBD_CDC_NOTIF_EP(n)			USB_ENDPADDR_DIRIN(1U + ((n) * 2U))
#define USBD_CDC_DATA_OUT_EP(n)		USB_ENDPADDR_DIROUT(2U + ((n) * 2U))
#define USBD_CDC_DATA_IN_EP(n)			USB_ENDPADDR_DIRIN(2U + ((n) * 2U))

#define USBD_CDC_NOTIF_MPS				8U
#define USBD_CDC_BULK_FS_MPS			64U
#define USBD_CDC_BULK_HS_MPS			512U

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default CDC ACM descriptor provider for UsbdCore.
 *
 * The provider reads the stable configuration owned by UsbDev. Descriptor
 * memory is static and remains valid until a subsequent SETUP request.
 */
const uint8_t *UsbdCdcDescHandler(uint8_t DescType,
								  uint8_t DescIndex,
								  uint16_t LangId,
								  UsbdSpeed_t Speed,
								  uint16_t *pLength,
								  void *pContext);

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USBD_CDC_DESC_H__

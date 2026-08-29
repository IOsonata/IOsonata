/**-------------------------------------------------------------------------
@file	usbd_cdc_func.h

@brief	Native USB CDC ACM function.

Implements CDC ACM control requests and owns the notification and Bulk USB
endpoints. Application byte buffering remains in UsbdBulkIntrf.

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
#ifndef __USBD_CDC_FUNC_H__
#define __USBD_CDC_FUNC_H__

#include "usb/usb_cdcdef.h"
#include "usb/usbd_bulk_intrf.h"
#include "usb/usbd_cdc_desc.h"

/** @addtogroup USBD
  * @{
  */

#define USBD_CDC_NOTIFY_LEN		(sizeof(UsbCdcNotification_t) + 2U)
#define USBD_CDC_TRANS_WORDS		(USBD_CDC_BULK_HS_MPS / sizeof(uint32_t))
#define USBD_CDC_NOTIFY_WORDS		((USBD_CDC_NOTIFY_LEN + sizeof(uint32_t) - 1U) / sizeof(uint32_t))

typedef struct __Usbd_Cdc_Function {
	UsbdBulkDevIntrf_t *pBulk;		//!< Application data plane
	UsbCdcLineCoding_t LineCoding;	//!< Current ACM line coding
	UsbCdcLineCoding_t PendingLineCoding;
	uint16_t ControlLineState;		//!< Host DTR/RTS state
	uint16_t PendingControlLineState;
	uint16_t SerialState;			//!< Device SerialState notification bits
	uint16_t BulkMps;				//!< Current data endpoint packet size
	uint16_t TxLength;				//!< Bytes staged in TxTransfer
	int FunctionNo;					//!< CDC function index
	bool Configured;
	bool RxActive;
	bool TxActive;
	bool TxZlpActive;
	bool NotifActive;
	bool SerialStatePending;
	uint32_t RxTransfer[USBD_CDC_TRANS_WORDS];
	uint32_t TxTransfer[USBD_CDC_TRANS_WORDS];
	uint32_t NotifTransfer[USBD_CDC_NOTIFY_WORDS];
} UsbdCdcFunc_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register one CDC ACM function with the native device core.
 *
 * pBulk must already be initialized. This function becomes the owner of its
 * RxKick/TxKick callbacks but does not change the FIFO storage or policy.
 */
bool UsbdCdcFuncInit(UsbdCdcFunc_t *pFunc, int FunctionNo,
					 UsbdBulkDevIntrf_t *pBulk);

/** @brief true when configured and the host has asserted DTR. */
bool UsbdCdcFuncPortIsOpen(const UsbdCdcFunc_t *pFunc);

/** @brief Current host line-coding request. */
const UsbCdcLineCoding_t *UsbdCdcFuncLineCoding(const UsbdCdcFunc_t *pFunc);

/** @brief Current DTR/RTS bitmap from SET_CONTROL_LINE_STATE. */
uint16_t UsbdCdcFuncControlLineState(const UsbdCdcFunc_t *pFunc);

/**
 * @brief Update the ACM SerialState reported to the host.
 *
 * Only CDC SerialState bits defined by usb_cdcdef.h are retained.
 */
void UsbdCdcFuncSetSerialState(UsbdCdcFunc_t *pFunc, uint16_t SerialState);

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USBD_CDC_FUNC_H__

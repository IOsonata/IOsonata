/**-------------------------------------------------------------------------
@file	usbd_cdc_intrf.h

@brief	Nordic SDK CDC ACM DeviceIntrf adapter for the USB CDC benchmark

This adapter is local to the usb_cdc_acm example. It is used to compare the
IOsonata DeviceIntrf + CFifo data path against Nordic SDK 17.1.0 CDC ACM
without involving the native IOsonata USB stack.

@author	Hoang Nguyen Hoan
@date	May 2, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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
#ifndef __USBD_CDC_INTRF_H__
#define __USBD_CDC_INTRF_H__

#include "device_intrf.h"
#include "cfifo.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 4)

typedef struct __UsbdCdc_Interf_Config {
	bool bBlocking;				//!< true - Reject writes when FIFO is full
	int RxFifoMemSize;			//!< Total RX CFifo memory size in bytes
	uint8_t *pRxFifoMem;		//!< RX CFifo memory, NULL selects local default
	int TxFifoMemSize;			//!< Total TX CFifo memory size in bytes
	uint8_t *pTxFifoMem;		//!< TX CFifo memory, NULL selects local default
	DevIntrfEvtHandler_t EvtCB;	//!< Interface event callback
} UsbdCdcIntrfCfg_t;

#define USBD_CDC_INTRF_TRANSBUFF_MAXLEN	64

typedef struct __UsbdCdc_Dev_Interf {
	DevIntrf_t DevIntrf;
	hCFifo_t hRxFifo;
	hCFifo_t hTxFifo;
	uint32_t RxDropCnt;
	uint32_t TxDropCnt;
	uint8_t RxTransBuff[USBD_CDC_INTRF_TRANSBUFF_MAXLEN];
	uint8_t TxTransBuff[USBD_CDC_INTRF_TRANSBUFF_MAXLEN];
	int TxTransBuffLen;
	atomic_bool bPortOpen;
	atomic_bool bEnabled;
	atomic_bool bRxPending;
} UsbdCdcDevIntrf_t;

#pragma pack(pop)

bool UsbdCdcIntrfInit(UsbdCdcDevIntrf_t * const pIntrf,
					  const UsbdCdcIntrfCfg_t *pCfg);
bool UsbdCdcIntrfIsPortOpen(const UsbdCdcDevIntrf_t *pIntrf);

#ifdef __cplusplus
}
#endif

#endif	// __USBD_CDC_INTRF_H__

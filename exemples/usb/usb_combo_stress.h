/**-------------------------------------------------------------------------
@file	usb_combo_stress.h

@brief	Constants and data types for the USB combo stress examples.

Each benchmark owns its device state and implementation in its source file.

@author	Hoang Nguyen Hoan
@date	Oct. 2, 2026

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
#ifndef __USB_COMBO_STRESS_H__
#define __USB_COMBO_STRESS_H__

#include "usb/usbd_cdc.h"

#define USB_DEVNO			0
#define CDC_BUFFER_SIZE		USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK)

#define CDC_RXFIFO_PKTCNT	4
#define CDC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, CDC_BUFFER_SIZE)
#define LOOPBACK_TXFIFO_MEMSIZE	CFIFO_MEMSIZE(1024)
#define PRBS_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(2048)

#define COMBO_STR_INTERFACE	4U
#define HID_REPORT_SIZE			64U

#define INT_ALT_COUNT			3U
#define INT_MPS					64U

#define ISO_ALT_COUNT			6U
#define ISO_MAX_MPS			63U
#define ISO_REQ_GET_DIAG		0x5AU

#pragma pack(push, 1)
typedef struct __Combo_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} ComboAltDesc_t;

typedef struct __Combo_Int_Function_Descriptor {
	UsbIntrfDesc_t Alt0;
	ComboAltDesc_t Alt[INT_ALT_COUNT];
} ComboIntFunctionDesc_t;

typedef struct __Combo_Iso_Function_Descriptor {
	UsbIntrfDesc_t Alt0;
	ComboAltDesc_t Alt[ISO_ALT_COUNT];
} ComboIsoFunctionDesc_t;

typedef struct __Combo_Iso_Diag {
	uint32_t RxMissCnt;
	uint32_t TxMissCnt;
	uint32_t LoopbackDropCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
} ComboIsoDiag_t;
#pragma pack(pop)

typedef struct __Combo_Function_State {
	bool IntConfigured;
	uint8_t IntAlt;
	uint8_t IntInterfaceNo;
	uint8_t IntEpNo;
	bool IsoConfigured;
	uint8_t IsoAlt;
	uint8_t IsoInterfaceNo;
	uint8_t IsoEpNo;
	uint32_t IsoLoopbackDropCnt;
} ComboFunctionState_t;

#endif

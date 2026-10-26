/**-------------------------------------------------------------------------
@file	usbd_int.h

@brief	USB device vendor Interrupt function.

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
#ifndef __USBD_INT_H__
#define __USBD_INT_H__

#include "usb/usbd_periodic.h"
#include "usb/usb_int.h"

typedef struct __Usbd_Int_Config {
	UsbdPeriodicCfg_t Function;
	DevIntrfEvtHandler_t EvtCB;
	void *pContext;
	// Separate caller-owned aligned slots, USB_INT_INTRF_PKT_BLKSIZE each.
	uint8_t *pRxBuffer;
	uint8_t *pTxBuffer;
} UsbdIntCfg_t;

#ifdef __cplusplus
class UsbdInt : public UsbdPeriodic, public UsbIntIntrf {
public:
	UsbdInt();
	bool Init(const UsbdIntCfg_t &Cfg);
	void Reset(void) override;

protected:
	bool OpenTransport(uint16_t Mps, uint8_t Interval) override {
		return UsbIntIntrf::Open(Mps, Interval);
	}
	void CloseTransport(void) override;
};
#endif
#endif

/**-------------------------------------------------------------------------
@file	usbd_iso.h

@brief	USB device vendor Isochronous function.

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
#ifndef __USBD_ISO_H__
#define __USBD_ISO_H__

#include "usb/usbd_periodic.h"
#include "usb/usb_iso.h"

typedef struct __Usbd_Iso_Config {
	UsbdPeriodicCfg_t Function;
	DevIntrfEvtHandler_t EvtCB;
	void *pContext;
	// Both null uses the transport's internal storage. Otherwise supply
	// separate aligned USB_ISO_INTRF_FIFO_MEMSIZE(BufferSize) regions.
	uint16_t BufferSize;
	uint8_t *pRxFifoMem;
	uint8_t *pTxFifoMem;
} UsbdIsoCfg_t;

#ifdef __cplusplus
class UsbdIso : public UsbdPeriodic, public UsbIsoIntrf {
public:
	UsbdIso();
	bool Init(const UsbdIsoCfg_t &Cfg);
	void Reset(void) override;
	void Process(void) override;

protected:
	bool OpenTransport(uint16_t Mps, uint8_t Interval) override {
		return UsbIsoIntrf::Open(Mps, Interval);
	}
	void CloseTransport(void) override;
};
#endif
#endif

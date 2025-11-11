/**-------------------------------------------------------------------------
@file	usbd_cdc_intrf.h

@brief	Generic implementation of USBD CDC device interface


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

/** @addtogroup USBD
  * @{
  */

#pragma pack(push, 4)

typedef struct __UsbdCdc_Interf_Config {
    bool bBlocking;				//!< true - Blocking Fifo, false - Non blocking
	int RxFifoMemSize;			//!< Total memory size for CFIFO
	uint8_t *pRxFifoMem;		//!< Pointer to memory to be used by CFIFO
	int TxFifoMemSize;			//!< Total memory size for CFIFO
	uint8_t *pTxFifoMem;		//!< Pointer to memory to be used by CFIFO
	DevIntrfEvtHandler_t EvtCB;	//!< Event callback
} UsbdCdcIntrfCfg_t;

#define USBD_CDC_INTRF_TRANSBUFF_MAXLEN			64

// USBD CDC interf instance data
typedef struct __UsbdCdc_Dev_Interf {
	DevIntrf_t	DevIntrf;		//!< Base Device Interface
    hCFifo_t	hRxFifo;
    hCFifo_t	hTxFifo;
    uint32_t	RxDropCnt;
    uint32_t	TxDropCnt;
    uint8_t     TransBuff[USBD_CDC_INTRF_TRANSBUFF_MAXLEN];  //
    int         TransBuffLen;	//!< Data length
} UsbdCdcDevIntrf_t;

#pragma pack(pop)

#ifdef __cplusplus

class UsbdCdcIntrf : public DeviceIntrf {
public:
	bool Init(const UsbdCdcIntrfCfg_t &Cfg);

	operator DevIntrf_t * const () { return &vUsbDevIntrf.DevIntrf; }	// Get device interface data

	// Set data rate in bits/sec (Hz)
	virtual uint32_t Rate(uint32_t DataRate) { return DeviceIntrfSetRate(&vUsbDevIntrf.DevIntrf, DataRate); }
	// Get current data rate in bits/sec (Hz)
	virtual uint32_t Rate(void) { return DeviceIntrfGetRate(&vUsbDevIntrf.DevIntrf); }
	// Disable device for power reduction, re-enable with Enable() without
	// full init
	virtual void Disable(void) { DeviceIntrfDisable(&vUsbDevIntrf.DevIntrf); }
	// Enable device
	virtual void Enable(void) { DeviceIntrfEnable(&vUsbDevIntrf.DevIntrf); }

	// Initiate receive
	virtual bool StartRx(uint32_t DevAddr) { return DeviceIntrfStartRx(&vUsbDevIntrf.DevIntrf, DevAddr); }
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vUsbDevIntrf.DevIntrf, pBuff, BuffLen);
	}
	// Stop receive
	// BEWARE !!!!!
	// This functions MUST ONLY be called if StartRx returns true.
	virtual void StopRx(void) { DeviceIntrfStopRx(&vUsbDevIntrf.DevIntrf); }
	// Initiate transmit
	virtual bool StartTx(uint32_t DevAddr) {
		return DeviceIntrfStartTx(&vUsbDevIntrf.DevIntrf, DevAddr);
	}
	// Transmit Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vUsbDevIntrf.DevIntrf, pData, DataLen);
	}
	// Stop transmit
	// BEWARE !!!!!
	// This functions MUST ONLY be called if StartTx returns true.
	virtual void StopTx(void) { DeviceIntrfStopTx(&vUsbDevIntrf.DevIntrf); }

	virtual bool RequestToSend(int NbBytes);

private:

	UsbdCdcDevIntrf_t vUsbDevIntrf;
};

extern "C" {
#endif

bool UsbdCdcIntrfInit(UsbdCdcDevIntrf_t * const pUsbdDevIntrf, const UsbdCdcIntrfCfg_t *pCfg);

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USBD_CDC_INTRF_H__

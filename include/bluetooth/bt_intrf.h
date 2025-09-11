/**-------------------------------------------------------------------------
@file	bt_intrf.h

@brief	Implementation allow the creation of generic serial interface of
a custom Bluetooth service with multiple user defined characteristics.


@author	Hoang Nguyen Hoan
@date	Feb. 6, 2017

@license

MIT License

Copyright (c) 2017, I-SYST inc., all rights reserved

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

#ifndef __BT_INTRF_H__
#define __BT_INTRF_H__

#include "bluetooth/bt_gatt.h"
#include "device_intrf.h"
#include "cfifo.h"

/** @addtogroup Bluetooth
  * @{
  */

#define BTINTRF_TRANSBUFF_MAXLEN       512

/**
 * This structure define the CFIFO data packet
 * It is a variable length structure defined by user
 *
 * NOTE : Data element can be more than 1 byte. DO NOT use sizeof() to calculate
 * the size of this structure.  It should be mapped to appropriate buffer memory
 */
#pragma pack(push, 1)

typedef struct __Bt_Interf_Packet {
    uint16_t    Len;    // Valid data length
    uint8_t     Data[1];// Data container array
} BtIntrfPkt_t;

#pragma pack(pop)

/**
 * Calculate require mem
 */
#define BTINTRF_PKHDR_LEN							(sizeof(BtIntrfPkt_t) - 1)
#define BTINTRF_CFIFO_TOTAL_MEMSIZE(npk, pksize)	CFIFO_TOTAL_MEMSIZE(npk, pksize + BTINTRF_PKHDR_LEN)

#pragma pack(push, 4)

typedef struct __Bt_Interf_Config {
	BtGattSrvc_t *pSrvc;		//!< BLE Service
    int RxCharIdx;			//!< Write characteristic index (From BLE)
    int TxCharIdx;			//!< Read characteristic index (to BLE)
    int PacketSize;			//!< BLE packet size
    bool bBlocking;			//!< true - Blocking Fifo, false - Non blocking
	int RxFifoMemSize;		//!< Total memory size for CFIFO
	uint8_t *pRxFifoMem;	//!< Pointer to memory to be used by CFIFO
	int TxFifoMemSize;		//!< Total memory size for CFIFO
	uint8_t *pTxFifoMem;	//!< Pointer to memory to be used by CFIFO
	DevIntrfEvtHandler_t EvtCB;//!< Event callback
} BtIntrfCfg_t;

// BLE interf instance data
typedef struct __Bt_Dev_Interf {
//	BtDev_t		pDev;
	DevIntrf_t	DevIntrf;	//!< Base Device Interface
	BtGattSrvc_t	*pSrvc;	//!< BLE Service
    int			RxCharIdx;	//!< Write characteristic index (from BLE)
    int			TxCharIdx;	//!< Read characteristic index (to BLE)
    int			PacketSize;	//!< BLE packet size
    HCFIFO		hRxFifo;
    HCFIFO		hTxFifo;
    uint32_t	RxDropCnt;
    uint32_t	TxDropCnt;
    uint8_t     TransBuff[BTINTRF_TRANSBUFF_MAXLEN];  //
    int         TransBuffLen;   //!< Data length
} BtDevIntrf_t;

#pragma pack(pop)

#ifdef __cplusplus

class BtIntrf : public DeviceIntrf {
public:
	bool Init(const BtIntrfCfg_t &Cfg);

	operator DevIntrf_t * const () { return &vBtDevIntrf.DevIntrf; }	// Get device interface data
	operator BtGattSrvc_t * const () { return vBtDevIntrf.pSrvc; }
	// Set data rate in bits/sec (Hz)
	virtual uint32_t Rate(uint32_t DataRate) { return DeviceIntrfSetRate(&vBtDevIntrf.DevIntrf, DataRate); }
	// Get current data rate in bits/sec (Hz)
	virtual uint32_t Rate(void) { return DeviceIntrfGetRate(&vBtDevIntrf.DevIntrf); }
	// Disable device for power reduction, re-enable with Enable() without
	// full init
	virtual void Disable(void) { DeviceIntrfDisable(&vBtDevIntrf.DevIntrf); }
	// Enable device
	virtual void Enable(void) { DeviceIntrfEnable(&vBtDevIntrf.DevIntrf); }

	// Initiate receive
	virtual bool StartRx(uint32_t DevAddr) { return DeviceIntrfStartRx(&vBtDevIntrf.DevIntrf, DevAddr); }
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vBtDevIntrf.DevIntrf, pBuff, BuffLen);
	}
	// Stop receive
	// BEWARE !!!!!
	// This functions MUST ONLY be called if StartRx returns true.
	virtual void StopRx(void) { DeviceIntrfStopRx(&vBtDevIntrf.DevIntrf); }
	// Initiate transmit
	virtual bool StartTx(uint32_t DevAddr) {
		return DeviceIntrfStartTx(&vBtDevIntrf.DevIntrf, DevAddr);
	}
	// Transmit Data only, no Start/Stop condition
	virtual int TxData(const uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vBtDevIntrf.DevIntrf, pData, DataLen);
	}
	// Stop transmit
	// BEWARE !!!!!
	// This functions MUST ONLY be called if StartTx returns true.
	virtual void StopTx(void) { DeviceIntrfStopTx(&vBtDevIntrf.DevIntrf); }

	virtual bool RequestToSend(int NbBytes);

private:

	BtDevIntrf_t vBtDevIntrf;
};

extern "C" {
#endif

bool BtIntrfInit(BtDevIntrf_t * const pBtDevIntrf, const BtIntrfCfg_t *pCfg);

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BT_INTRF_H__

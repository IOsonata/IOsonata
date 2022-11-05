/**-------------------------------------------------------------------------
@file	ble_intrf.h

@brief	Implementation allow the creation of generic serial interface of
a custom Bluetooth Smart service with multiple user defined characteristics.


@author	Hoang Nguyen Hoan
@date	Feb. 6, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BLE_INTRF_H__
#define __BLE_INTRF_H__

#include "bluetooth/bt_gatt.h"
#include "device_intrf.h"
#include "cfifo.h"

/** @addtogroup Bluetooth
  * @{
  */

#define BLEINTRF_TRANSBUFF_MAXLEN       512

/**
 * This structure define the CFIFO data packet
 * It is a variable length structure defined by user
 *
 * NOTE : Data element can be more than 1 byte. DO NOT use sizeof() to calculate
 * the size of this structure.  It should be mapped to appropriate buffer memory
 */
#pragma pack(push, 1)
typedef struct __BleDeviceInterfPacket {
    uint16_t    Len;    // Valid data length
    uint8_t     Data[1];// Data container array
} BleIntrfPkt_t;

typedef BleIntrfPkt_t	BLEINTRF_PKT;

#pragma pack(pop)

/**
 * Calculate require mem
 */
#define BLEINTRF_PKHDR_LEN			(sizeof(BleIntrfPkt_t) - 1)
#define BLEINTRF_CFIFO_TOTAL_MEMSIZE(npk, pksize)	CFIFO_TOTAL_MEMSIZE(npk, pksize + BLEINTRF_PKHDR_LEN)

#pragma pack(push, 4)

typedef struct __BleDeviceInterfConfig {
	BtGattSrvc_t *pBleSrvc;		//!< BLE Service
    int RxCharIdx;			//!< Write characteristic index (From BLE)
    int TxCharIdx;			//!< Read characteristic index (to BLE)
    int PacketSize;			//!< BLE packet size
    bool bBlocking;			//!< true - Blocking Fifo, false - Non blocking
	int RxFifoMemSize;		//!< Total memory size for CFIFO
	uint8_t *pRxFifoMem;	//!< Pointer to memory to be used by CFIFO
	int TxFifoMemSize;		//!< Total memory size for CFIFO
	uint8_t *pTxFifoMem;	//!< Pointer to memory to be used by CFIFO
	DevIntrfEvtHandler_t EvtCB;//!< Event callback
} BleIntrfCfg_t;

typedef BleIntrfCfg_t	BLEINTRF_CFG;

// BLE interf instance data
typedef struct __BleDeviceInterf {
	DevIntrf_t	DevIntrf;	//!< Base Device Interface
	BtGattSrvc_t	*pBleSrvc;	//!< BLE Service
    int			RxCharIdx;	//!< Write characteristic index (from BLE)
    int			TxCharIdx;	//!< Read characteristic index (to BLE)
    int			PacketSize;	//!< BLE packet size
    HCFIFO		hRxFifo;
    HCFIFO		hTxFifo;
    uint32_t	RxDropCnt;
    uint32_t	TxDropCnt;
    uint8_t     TransBuff[BLEINTRF_TRANSBUFF_MAXLEN];  //
    int         TransBuffLen;   //!< Data length
} BleIntrf_t;

typedef BleIntrf_t	BLEINTRF;

#pragma pack(pop)

#ifdef __cplusplus

class BleIntrf : public DeviceIntrf {
public:
	bool Init(const BleIntrfCfg_t &Cfg);

	operator DevIntrf_t * const () { return &vBleIntrf.DevIntrf; }	// Get device interface data
	operator BtGattSrvc_t * const () { return vBleIntrf.pBleSrvc; }
	// Set data rate in bits/sec (Hz)
	virtual uint32_t Rate(uint32_t DataRate) { return DeviceIntrfSetRate(&vBleIntrf.DevIntrf, DataRate); }
	// Get current data rate in bits/sec (Hz)
	virtual uint32_t Rate(void) { return DeviceIntrfGetRate(&vBleIntrf.DevIntrf); }
	// Disable device for power reduction, re-enable with Enable() without
	// full init
	virtual void Disable(void) { DeviceIntrfDisable(&vBleIntrf.DevIntrf); }
	// Enable device
	virtual void Enable(void) { DeviceIntrfEnable(&vBleIntrf.DevIntrf); }

	// Initiate receive
	virtual bool StartRx(uint32_t DevAddr) { return DeviceIntrfStartRx(&vBleIntrf.DevIntrf, DevAddr); }
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vBleIntrf.DevIntrf, pBuff, BuffLen);
	}
	// Stop receive
	// BEWARE !!!!!
	// This functions MUST ONLY be called if StartRx returns true.
	virtual void StopRx(void) { DeviceIntrfStopRx(&vBleIntrf.DevIntrf); }
	// Initiate transmit
	virtual bool StartTx(uint32_t DevAddr) {
		return DeviceIntrfStartTx(&vBleIntrf.DevIntrf, DevAddr);
	}
	// Transmit Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vBleIntrf.DevIntrf, pData, DataLen);
	}
	// Stop transmit
	// BEWARE !!!!!
	// This functions MUST ONLY be called if StartTx returns true.
	virtual void StopTx(void) { DeviceIntrfStopTx(&vBleIntrf.DevIntrf); }

	virtual bool RequestToSend(int NbBytes);

private:

	BleIntrf_t vBleIntrf;
};

extern "C" {
#endif

bool BleIntrfInit(BleIntrf_t * const pBleIntrf, const BleIntrfCfg_t *pCfg);

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BLE_INTRF_H__

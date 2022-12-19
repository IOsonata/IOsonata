/**-------------------------------------------------------------------------
@file	bt_ctlr.h

@brief	Generic implementation of Bluetooth controller device.


@author	Hoang Nguyen Hoan
@date	Nov. 30, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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
#ifndef __BT_CTLR_H__
#define __BT_CTLR_H__

#include "device_intrf.h"
#include "cfifo.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_hcievt.h"

/** @addtogroup Bluetooth
  * @{
  */

#define BT_CTLR_MTU_MAX									512

#define BT_CTLR_PHY_1MBITS								(1<<0)
#define BT_CTLR_PHY_2MBITS								(1<<1)
#define BT_CTLR_PHY_CODED								(1<<2)

#pragma pack(push, 4)

typedef struct __Bt_Ctlr_Dev		BtCtlrDev_t;

typedef void (*AttDataHandler_t)(BtCtlrDev_t * const pDev, uint16_t ConnHdl, BtL2CapPdu_t * const pL2Frame);
typedef void (*SmpDataHandler_t)(BtCtlrDev_t * const pDev, BtL2CapPdu_t * const pL2Frame);

typedef struct __Bt_Ctlr_Config {
	uint16_t MaxMtu;
	size_t PacketSize;
	uint8_t *pRxFifoMem;
	int RxFifoMemSize;
	DevIntrfEvtHandler_t EvtHandler;
	AttDataHandler_t AttHandler;
	SmpDataHandler_t SmpHandler;
//	uint32_t (*Send)(BtCtlrDev_t * const pDev, void * const pData, uint32_t Len);
//	uint32_t (*Receive)(BtCtlrDev_t * const pDev, void * const pData, uint32_t Len);
} BtCtlrCfg_t;

struct __Bt_Ctlr_Dev {
	uint16_t MaxMtu;
	size_t PacketSize;
	uint8_t Phy;
	uint32_t Rate;
	DevIntrf_t DevIntrf;
	void *pObj;
	uint16_t ConnHdl;				//<! Connection handle
	uint16_t ValHdl;				//<! Characteristic value handle
	HCFIFO hRxFifo;
	AttDataHandler_t AttHandler;
	SmpDataHandler_t SmpHandler;
	size_t (*Send)(BtCtlrDev_t * const pDev, void * const pData, size_t Len);
	size_t (*Receive)(BtCtlrDev_t * const pDev, uint16_t Hdl, void * const pData, size_t Len);
};// BtCtlrDev_t;

/// NOTE: Variable length
typedef struct __Bt_Ctlr_Packet {
	uint16_t ValHdl;				//!< Char value handle
	uint16_t Len;					//!< Data length in bytes
	uint16_t Off;					//!< Data start offset for fragmented packet
	uint8_t Data[1];				//!< Variable length data
} BtCtlrPkt_t;

/**
 * Calculate require mem
 */
#define BTCTLR_PKTHDR_LEN								(sizeof(BtCtlrPkt_t) - 1)
#define BTCTLR_PKT_CFIFO_TOTAL_MEMSIZE(npk, pksize)		CFIFO_TOTAL_MEMSIZE(npk, pksize + BTCTLR_PKTHDR_LEN)

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 * @param pDev
 * @param pCfg
 * @return
 */
bool BtCtlrInit(BtCtlrDev_t * const pDev, const BtCtlrCfg_t *pCfg);
void BtCtlrProcessAttData(BtCtlrDev_t * const pDev, uint16_t ConnHdl, BtL2CapPdu_t * const pRcvPdu);
void BtCtlrProcessSmpData(BtCtlrDev_t * const pDev, BtL2CapPdu_t * const pRcvPdu);
void BtCtlrProcessEvent(BtCtlrDev_t * const pDev, BtHciEvtPacket_t * const pEvtPkt);
void BtCtlrProcessData(BtCtlrDev_t * const pDev, BtHciACLDataPacket_t * const pPkt);

#ifdef __cplusplus
}

class BtCtlr : public DeviceIntrf {
public:
	operator BtCtlrDev_t *  const () { return &vDevData; }
	operator DevIntrf_t * const () { return &vDevData.DevIntrf; }

	virtual bool Init(const BtCtlrCfg_t &Cfg) { return BtCtlrInit(&vDevData, &Cfg); }
	// Set data baudrate
	virtual uint32_t Rate(uint32_t DataRate) { return DeviceIntrfSetRate(&vDevData.DevIntrf, DataRate); }
	// Get current data baudrate
	virtual uint32_t Rate(void) { return vDevData.Rate; }
    void Enable(void) { DeviceIntrfEnable(&vDevData.DevIntrf); }
    void Disable(void) { DeviceIntrfDisable(&vDevData.DevIntrf); }
	virtual int Rx(uint8_t *pBuff, int Len) { return DeviceIntrfRx(&vDevData.DevIntrf, 0, pBuff, Len); }
	// Initiate receive
	virtual bool StartRx(uint32_t DevAddr) { return DeviceIntrfStartRx(&vDevData.DevIntrf, DevAddr); }
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}
	// Stop receive
	virtual void StopRx(void) { DeviceIntrfStopRx(&vDevData.DevIntrf); }
	virtual int Tx(uint8_t *pData, uint32_t Len) { return DeviceIntrfTx(&vDevData.DevIntrf, 0, pData, Len); }
	// Initiate transmit
	virtual bool StartTx(uint32_t DevAddr) { return DeviceIntrfStartTx(&vDevData.DevIntrf, DevAddr); }
	// Transmit Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}
	// Stop transmit
	virtual void StopTx(void) { DeviceIntrfStopTx(&vDevData.DevIntrf); }

protected:
private:
	BtCtlrDev_t vDevData;

	BtCtlr(&BtCtlr);
};

#endif

/** @} end group Bluetooth */

#endif // __BT_CTLR_H__

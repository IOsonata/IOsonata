/**-------------------------------------------------------------------------
@file	bt_hci_ctlr.h

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
#ifndef __BT_HCI_CTLR_H__
#define __BT_HCI_CTLR_H__

#include "device_intrf.h"
#include "cfifo.h"
#include "bluetooth/bt_hcievt.h"

/** @addtogroup Bluetooth
  * @{
  */

#define BT_HCI_CTLR_MTU_MAX									512

#define BT_HCI_CTLR_PHY_1MBITS								(1<<0)
#define BT_HCI_CTLR_PHY_2MBITS								(1<<1)
#define BT_HCI_CTLR_PHY_CODED								(1<<2)

#pragma pack(push, 4)

typedef struct __Bt_Hci_Ctlr_Dev		BtHciCtlrDev_t;

// HCI receive callback. The controller fires this for each HCI packet pulled
// from the wrapped stack. bIsEvent selects the host entry: true for an HCI
// event packet, false for an ACL data packet. The host layer wires this to
// its process entry; the controller holds no host type.
typedef void (*BtHciCtlrRxHandler_t)(BtHciCtlrDev_t * const pDev, bool bIsEvent, uint8_t *pPacket);

typedef struct __Bt_Hci_Ctlr_Config {
	uint16_t MaxMtu;
	size_t PacketSize;
	uint8_t *pRxFifoMem;
	int RxFifoMemSize;
	DevIntrfEvtHandler_t EvtHandler;
	BtHciCtlrRxHandler_t RxHandler;	//!< HCI receive handler, host wires it to its process entry
	uint16_t Role;					//!< BT_GAP_ROLE_* bitmask the controller must support
	uint8_t PeriLinkCount;			//!< Peripheral link count for resource sizing
	uint8_t CentLinkCount;			//!< Central link count for resource sizing
	uint8_t RxPktCount;				//!< Controller RX ACL packet count
	uint8_t TxPktCount;				//!< Controller TX ACL packet count
	uint16_t MaxDataLen;			//!< ACL data length for buffer sizing
	void (*OnWake)(void);			//!< Fired from the controller receive context to wake a host waiter
//	uint32_t (*Send)(BtHciCtlrDev_t * const pDev, void * const pData, uint32_t Len);
//	uint32_t (*Receive)(BtHciCtlrDev_t * const pDev, void * const pData, uint32_t Len);
} BtHciCtlrCfg_t;

struct __Bt_Hci_Ctlr_Dev {
	uint16_t MaxMtu;
	size_t PacketSize;
	uint8_t Phy;
	uint32_t Rate;
	DevIntrf_t DevIntrf;
	void *pObj;
	uint16_t ConnHdl;				//<! Connection handle
	uint16_t ValHdl;				//<! Characteristic value handle
	hCFifo_t hRxFifo;
	BtHciCtlrRxHandler_t RxHandler;	//!< HCI receive handler set from config
	void (*OnWake)(void);			//!< Host waiter wake, set from config
	size_t (*Send)(BtHciCtlrDev_t * const pDev, void * const pData, size_t Len);
	size_t (*SendCommand)(BtHciCtlrDev_t * const pDev, void * const pData, size_t Len);	//!< Send a formed HCI command down to the controller
	size_t (*Receive)(BtHciCtlrDev_t * const pDev, uint16_t Hdl, void * const pData, size_t Len);
};// BtHciCtlrDev_t;

/// NOTE: Variable length
typedef struct __Bt_Hci_Ctlr_Packet {
	uint16_t ValHdl;				//!< Char value handle
	uint16_t Len;					//!< Data length in bytes
	uint16_t Off;					//!< Data start offset for fragmented packet
	uint8_t Data[1];				//!< Variable length data
} BtHciCtlrPkt_t;

/**
 * Calculate require mem
 */
#define BTHCICTLR_PKTHDR_LEN								(sizeof(BtHciCtlrPkt_t) - 1)
#define BTHCICTLR_PKT_CFIFO_TOTAL_MEMSIZE(npk, pksize)		CFIFO_TOTAL_MEMSIZE(npk, pksize + BTHCICTLR_PKTHDR_LEN)

#define BTHCICTLR_CMD_HDR_LEN								3		// HCI command header: opcode (2) plus parameter length (1)
#define BTHCICTLR_CMD_PARAM_MAX								255		// Max HCI command parameter length

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
bool BtHciCtlrInit(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg);

/**
 * @brief	Bring up the controller: initialize the underlying stack, apply
 *			role and resource configuration, and enable it.
 *
 * For the SDC controller this runs sdc_init, the role-gated sdc_support_*
 * calls, sdc_cfg_set resource sizing, MpslInit, and sdc_enable, then wires
 * the device through BtHciCtlrInit. Call once at startup before the pump.
 *
 * @param	pDev	Controller device.
 * @param	pCfg	Controller configuration.
 *
 * @return	true on success.
 */
bool BtHciCtlrEnable(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg);

/**
 * @brief	Drain all HCI packets currently queued by the controller.
 *
 * Fires RxHandler once per packet. Call from the controller receive context.
 * For the SDC controller that is the low priority callback and the main loop.
 *
 * @param	pDev	Controller device.
 */
void BtHciCtlrProcess(BtHciCtlrDev_t * const pDev);

/**
 * @brief	Frame a standard HCI command and send it to the controller.
 *
 * Builds the standard HCI command packet (opcode little endian, parameter
 * length, parameters) and sends it through the controller SendCommand op. This
 * is the send only step. The response arrives asynchronously as a Command
 * Complete or Command Status event on the RX path.
 *
 * @param	pDev		Controller device.
 * @param	OpCode		16 bit HCI opcode.
 * @param	pParam		Command parameters, or NULL when ParamLen is 0.
 * @param	ParamLen	Parameter length in bytes.
 *
 * @return	Bytes sent, 0 on failure.
 */
size_t BtHciCtlrSendCommand(BtHciCtlrDev_t * const pDev, uint16_t OpCode, const void *pParam, uint8_t ParamLen);

#ifdef __cplusplus
}

class BtHciCtlr : public DeviceIntrf {
public:
	operator BtHciCtlrDev_t *  const () { return &vDevData; }
	operator DevIntrf_t * const () { return &vDevData.DevIntrf; }

	virtual bool Init(const BtHciCtlrCfg_t &Cfg) { return BtHciCtlrInit(&vDevData, &Cfg); }
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
	BtHciCtlrDev_t vDevData;

	BtHciCtlr(&BtHciCtlr);
};

#endif

/** @} end group Bluetooth */

#endif // __BT_HCI_CTLR_H__

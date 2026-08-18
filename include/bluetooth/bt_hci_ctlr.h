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
	// Periodic advertising resources. The controller allocates none of these by
	// default, so a device that leaves them zero behaves as before and every
	// periodic advertising command is refused. Each one costs controller RAM,
	// which is why the count is the application's to give rather than implied
	// by the role.
	uint8_t PeriodicAdvCount;		//!< Periodic advertising sets to transmit, advertiser role
	uint8_t PeriodicSyncCount;		//!< Trains that can be synchronized to at once, observer role
	uint8_t PawrAdvCount;			//!< Periodic advertising with responses sets, advertiser role
	uint8_t PawrSyncCount;			//!< Periodic advertising with responses trains answered, responder role
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
size_t BtHciCtlrSdcSend(void *pData, size_t Len);
uint8_t BtHciCmdSdc(BtHciDevice_t * const pDev, uint16_t OpCode, const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen);

/**
 * @brief	Bring up the controller.
 *
 * Generic, one implementation for every port. Validates the arguments, runs
 * BtHciCtlrInit and stops if it fails, then hands over to BtHciCtlrStart for
 * whatever the target's controller needs.
 *
 * The order matters and is why this is not left to each port. BtHciCtlrInit
 * assigns RxHandler, Receive and the interface table, and returns early
 * without them on a bad packet size, a misaligned or undersized RX FIFO
 * buffer, or a CFifoInit failure. A port that starts its controller anyway
 * reports an enabled controller with no receive path, and no HCI packet is
 * ever delivered.
 *
 * @param	pDev	Controller device.
 * @param	pCfg	Controller configuration.
 *
 * @return	true on success.
 */
bool BtHciCtlrEnable(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg);

/// Why the controller refused to start. A failed bring-up returns one bool to
/// its caller and that reaches the application as a single failure, so the
/// reason has to be recorded to be recoverable. The port traces are compiled
/// out unless the build defines DEBUG_ENABLE, which no project here does, so a
/// trace is not a substitute for this.
typedef enum __Bt_Hci_Ctlr_Error {
	BT_HCI_CTLR_ERROR_NONE = 0,		//!< No failure recorded
	BT_HCI_CTLR_ERROR_MPSL_INIT,	//!< Multiprotocol service layer refused, Value is its code
	BT_HCI_CTLR_ERROR_CTLR_INIT,	//!< Target controller init refused, Value is its code
	BT_HCI_CTLR_ERROR_CFG_SET,		//!< A resource configuration was refused, Value is the tag
	BT_HCI_CTLR_ERROR_MEM_POOL,		//!< Pool too small, Value is the size asked for in bytes
	BT_HCI_CTLR_ERROR_ARBITER,		//!< Memory arbiter refused, Value is its code
	BT_HCI_CTLR_ERROR_CTLR_ENABLE,	//!< Target controller enable refused, Value is its code
} BtHciCtlrError_t;

/**
 * @brief	Record why controller bring-up failed.
 *
 * Called by the port at each point it gives up. The meaning of Value depends
 * on Error and is given with each enumerator.
 *
 * @param	Error	Which step refused.
 * @param	Value	The number that step reported.
 */
void BtHciCtlrErrorSet(BtHciCtlrError_t Error, int32_t Value);

/**
 * @brief	The step that refused the last bring-up.
 *
 * @return	BT_HCI_CTLR_ERROR_NONE when no failure has been recorded.
 */
BtHciCtlrError_t BtHciCtlrErrorGet(void);

/**
 * @brief	The number reported by the step that refused.
 *
 * For BT_HCI_CTLR_ERROR_MEM_POOL this is the pool size the configuration
 * needs, which is what the pool constant should be set from.
 *
 * @return	The recorded value, zero when nothing has been recorded.
 */
int32_t BtHciCtlrErrorValueGet(void);

/**
 * @brief	Target controller bring-up, called by BtHciCtlrEnable.
 *
 * Everything vendor specific belongs here: for the SDC port that is sdc_init,
 * the role-gated sdc_support_* calls, sdc_cfg_set resource sizing, MpslInit
 * and sdc_enable. A port that needs nothing beyond the generic wiring, such
 * as a plain HCI transport over UART or USB, can leave the weak default,
 * which succeeds.
 *
 * Called only after BtHciCtlrInit has succeeded, so the device is wired
 * before any controller traffic can arrive.
 *
 * @param	pDev	Controller device, already initialized.
 * @param	pCfg	Controller configuration.
 *
 * @return	true on success.
 */
bool BtHciCtlrStart(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg);

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
	operator BtHciCtlrDev_t * () { return &vDevData; }
	operator DevIntrf_t * () { return &vDevData.DevIntrf; }

	virtual bool Init(const BtHciCtlrCfg_t &Cfg) { return BtHciCtlrInit(&vDevData, &Cfg); }
	// Set data baudrate
	virtual uint32_t Rate(uint32_t DataRate) { return DeviceIntrfSetRate(&vDevData.DevIntrf, DataRate); }
	// Get current data baudrate
	virtual uint32_t Rate(void) { return vDevData.Rate; }
    void Enable(void) { DeviceIntrfEnable(&vDevData.DevIntrf); }
    void Disable(void) { DeviceIntrfDisable(&vDevData.DevIntrf); }
	// Keep the base class 3 argument Rx/Tx visible alongside the 2 argument
	// convenience forms below, so neither set hides the other.
	using DeviceIntrf::Rx;
	using DeviceIntrf::Tx;
	virtual int Rx(uint8_t *pBuff, int Len) { return DeviceIntrfRx(&vDevData.DevIntrf, 0, pBuff, Len); }
	// Initiate receive
	virtual int Tx(uint8_t *pData, uint32_t Len) { return DeviceIntrfTx(&vDevData.DevIntrf, 0, pData, Len); }

protected:
private:
	BtHciCtlrDev_t vDevData;

public:
	// vDevData holds interface state the C layer points into; a copied
	// object would alias it. Not copyable, still default-constructible.
	BtHciCtlr() = default;
	BtHciCtlr(const BtHciCtlr&) = delete;
	BtHciCtlr &operator=(const BtHciCtlr&) = delete;
};

#endif

/** @} end group Bluetooth */

#endif // __BT_HCI_CTLR_H__

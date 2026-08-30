/**-------------------------------------------------------------------------
@file	usbd_cdc.h

@brief	USB CDC device interface

One CDC ACM port. The instance holds everything the port needs : the Bulk
data plane it is built on, the ACM protocol state, the staging the endpoints
transfer out of, and the endpoint bookkeeping.

UsbdBulkDevIntrf_t is the first member, so a UsbdCdcDevIntrf_t is a Bulk
interface and a DevIntrf_t at the same address. There is no second interface
to forward through and no table mapping a port number back to its state.

@author	Hoang Nguyen Hoan
@date	May 2, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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
#ifndef __USBD_CDC_H__
#define __USBD_CDC_H__

#include <stdbool.h>
#include <stdint.h>

#include "cfifo.h"
#include "device_intrf.h"
#include "usb/usb_cdcdef.h"
#include "usb/usbd.h"
#include "usb/usbd_bulk_intrf.h"
#include "usb/usbd_ctrlr.h"

/** @addtogroup USBD
  * @{
  */

//
// CDC topology. One port owns two consecutive interfaces, one interrupt IN
// endpoint number and one bidirectional Bulk endpoint number. Endpoint 15 is
// the last USB endpoint number, so this numbering fits seven ports on a
// controller with enough endpoints.
//
#define USBD_CDC_FUNC_MAXCNT			7
#define USBD_CDC_CTRL_INTRF(n)			((uint8_t)((n) * 2U))
#define USBD_CDC_DATA_INTRF(n)			((uint8_t)(USBD_CDC_CTRL_INTRF(n) + 1U))
#define USBD_CDC_NOTIF_EP(n)			USB_ENDPADDR_DIRIN(1U + ((n) * 2U))
#define USBD_CDC_DATA_OUT_EP(n)			USB_ENDPADDR_DIROUT(2U + ((n) * 2U))
#define USBD_CDC_DATA_IN_EP(n)			USB_ENDPADDR_DIRIN(2U + ((n) * 2U))

#define USBD_CDC_CONFIG_VALUE			1U

#define USBD_CDC_NOTIF_MPS				8U
#define USBD_CDC_BULK_FS_MPS			64U
#define USBD_CDC_BULK_HS_MPS			512U

#define USBD_CDC_NOTIFY_LEN				(sizeof(UsbCdcNotification_t) + 2U)
#define USBD_CDC_TRANS_WORDS			(USBD_CDC_BULK_HS_MPS / sizeof(uint32_t))
#define USBD_CDC_NOTIFY_WORDS \
	((USBD_CDC_NOTIFY_LEN + sizeof(uint32_t) - 1U) / sizeof(uint32_t))

//
// Two receive staging slots. The interrupt fills one while the pump drains
// the other, which is what lets the Bulk OUT endpoint stay armed while the
// application is behind.
//
#define USBD_CDC_RX_STAGE_COUNT			2U

#pragma pack(push, 4)

typedef struct __Usbd_Cdc_Rx_Stage {
	uint16_t Length;
	uint16_t Reserved;
	uint32_t Data[USBD_CDC_TRANS_WORDS];
} UsbdCdcRxStage_t;

typedef struct __UsbdCdc_Interf_Config {
	bool bBlocking;				//!< true - Blocking Fifo, false - Non blocking
	int RxFifoMemSize;			//!< Total memory size for CFIFO
	uint8_t *pRxFifoMem;		//!< Pointer to memory to be used by CFIFO
	int TxFifoMemSize;			//!< Total memory size for CFIFO
	uint8_t *pTxFifoMem;		//!< Pointer to memory to be used by CFIFO
	int ItfNo;					//!< CDC port index, 0 for the first port
	DevIntrfEvtHandler_t EvtCB;	//!< Event callback
} UsbdCdcIntrfCfg_t;

// USBD CDC interface instance data
typedef struct __UsbdCdc_Dev_Interf {
	UsbdBulkDevIntrf_t Bulk;		//!< Bulk data plane, must be first
	UsbCdcLineCoding_t LineCoding;	//!< Current ACM line coding
	UsbCdcLineCoding_t PendingLineCoding;
	uint16_t ControlLineState;		//!< Host DTR/RTS state
	uint16_t PendingControlLineState;
	uint16_t SerialState;			//!< Device SerialState notification bits
	uint16_t BulkMps;				//!< Current data endpoint packet size
	uint16_t TxLength;				//!< Bytes staged in TxTransfer
	uint16_t TxCompleteLength;		//!< Last completed IN byte count
	uint16_t TxCompleteExpected;	//!< Expected bytes for completed IN request
	UsbdCtrlrXferResult_t TxCompleteResult;
	uint32_t RxPut;					//!< Published RX staging producer index
	uint32_t RxGet;					//!< Released RX staging consumer index
	uint32_t RxDropCnt;
	uint32_t TxDropCnt;
	int ItfNo;						//!< CDC port index this instance serves
	bool Configured;
	bool RxActive;
	bool TxActive;
	bool TxZlpActive;
	bool TxZlpRequired;
	bool TxCompletePending;
	bool NotifActive;
	bool SerialStatePending;
	bool ReportedOpen;				//!< Last port state reported to DeviceIntrf
	UsbdCdcRxStage_t RxStage[USBD_CDC_RX_STAGE_COUNT];
	uint32_t TxTransfer[USBD_CDC_TRANS_WORDS];
	uint32_t NotifTransfer[USBD_CDC_NOTIFY_WORDS];
} UsbdCdcDevIntrf_t;

#pragma pack(pop)

#ifdef __cplusplus

class UsbdCdcIntrf : public DeviceIntrf {
public:
	bool Init(const UsbdCdcIntrfCfg_t &Cfg);

	operator DevIntrf_t * () { return &vUsbDevIntrf.Bulk.DevIntrf; }

	// Set data rate in bits/sec (Hz)
	virtual uint32_t Rate(uint32_t DataRate) { return DeviceIntrfSetRate(&vUsbDevIntrf.Bulk.DevIntrf, DataRate); }
	// Get current data rate in bits/sec (Hz)
	virtual uint32_t Rate(void) { return DeviceIntrfGetRate(&vUsbDevIntrf.Bulk.DevIntrf); }
	// Disable device for power reduction, re-enable with Enable() without
	// full init
	virtual void Disable(void) { DeviceIntrfDisable(&vUsbDevIntrf.Bulk.DevIntrf); }
	// Enable device
	virtual void Enable(void) { DeviceIntrfEnable(&vUsbDevIntrf.Bulk.DevIntrf); }

	virtual bool RequestToSend(int NbBytes);

	// true when the host has opened this port
	bool IsPortOpen(void);

	// Line coding the host last set
	const UsbCdcLineCoding_t *LineCoding(void);

	// DTR and RTS as the host last set them
	uint16_t ControlLineState(void);

	// Report modem and error state to the host over the notification endpoint
	void SetSerialState(uint16_t SerialState);

private:

	UsbdCdcDevIntrf_t vUsbDevIntrf;
};

extern "C" {
#endif

bool UsbdCdcIntrfInit(UsbdCdcDevIntrf_t * const pIntrf,
					  const UsbdCdcIntrfCfg_t *pCfg);

/**
 * @brief	Move data in both directions for this port.
 *
 * Called by UsbDevProcess for every registered interface. An application that
 * pumps the device itself does not call this.
 *
 * @param	pIntrf : Interface instance
 */
void UsbdCdcIntrfProcess(UsbdCdcDevIntrf_t * const pIntrf);

/**
 * @brief	Whether the host has opened this port.
 *
 * The device is configured and something on the host has asserted DTR.
 *
 * @param	pIntrf : Interface instance
 *
 * @return	true - Open
 */
bool UsbdCdcIntrfPortIsOpen(const UsbdCdcDevIntrf_t * const pIntrf);

/**
 * @brief	Line coding the host last set on this port.
 *
 * @param	pIntrf : Interface instance
 *
 * @return	Pointer to the line coding, NULL for a bad instance
 */
const UsbCdcLineCoding_t *UsbdCdcIntrfLineCoding(
									const UsbdCdcDevIntrf_t * const pIntrf);

/**
 * @brief	DTR and RTS as the host last set them.
 *
 * @param	pIntrf : Interface instance
 *
 * @return	Control line state bits
 */
uint16_t UsbdCdcIntrfControlLineState(const UsbdCdcDevIntrf_t * const pIntrf);

/**
 * @brief	Report modem and error state to the host.
 *
 * Sent on the notification endpoint. Bits outside the CDC SerialState set are
 * discarded.
 *
 * @param	pIntrf : Interface instance
 * @param	SerialState : CDC SerialState bits
 */
void UsbdCdcIntrfSetSerialState(UsbdCdcDevIntrf_t * const pIntrf,
								uint16_t SerialState);

/**
 * @brief	Descriptor provider for the CDC ACM device.
 *
 * Builds device, configuration, string and qualifier descriptors from
 * UsbDevCfg_t. Passed to UsbdCoreInit by the device layer.
 */
const uint8_t *UsbdCdcDescHandler(uint8_t DescType,
								  uint8_t DescIndex,
								  uint16_t LangId,
								  UsbdSpeed_t Speed,
								  uint16_t *pLength,
								  void *pContext);

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USBD_CDC_H__

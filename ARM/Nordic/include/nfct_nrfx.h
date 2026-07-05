/**-------------------------------------------------------------------------
@file	nfct_nrfx.h

@brief	Nordic NFCT frame transport, all NFCT equipped nRF series

This object is a DeviceIntrf over the Nordic NFCT peripheral in target mode.
It applies to all NFCT equipped parts served by the nrfx_nfct driver:
nRF52832, nRF52833, nRF52840, nRF5340 application core and the nRF54L
series. Series differences are contained in nrfx and its errata handling.
It moves raw NFC-A frames only. Activation and anticollision run in the NFCT
hardware with automatic collision resolution. CRC generation and check and
the frame delay timer are hardware as well. Nothing above the frame level is
handled here, no ISO-DEP, no APDU, no tag memory.

Transport capability set for RFTagCfg_t XCap:
	RFTAG_XCAP_ANTICOL | RFTAG_XCAP_CRC | RFTAG_XCAP_FDT

Usage with RFTag:
	Each received reader frame is passed to the frame callback set in the
	configuration. The callback normally calls RFTagProcessFrame, which runs
	the configured protocol and sends the response back through this
	transport. The response goes out through DeviceIntrfTx, which maps to
	nrfx_nfct_tx with the window grid delay mode.

Integration notes for the target build:
	HFCLK must run from the crystal before Enable, NFCT is clocked from it.
	NRFX_NFCT_ENABLED must be set in the nrfx configuration and the NFCT
	interrupt vector must reach nrfx_nfct_irq_handler.
	On parts with shared NFC pins the UICR setting must select the antenna
	function. Pin handling details differ per series, see the part datasheet.

@author	Hoang Nguyen Hoan
@date	Jul. 5, 2026

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
#ifndef __NFCT_NRFX_H__
#define __NFCT_NRFX_H__

#include <stdint.h>
#include <stdbool.h>

#include "device_intrf.h"

// Largest NFC-A frame the transport buffers. Covers FSD 256 frames.
#define NFCT_INTRF_FRAME_MAX	260

typedef enum {
	NFCT_INTRF_EVT_FIELD_ON,	//!< Reader field detected
	NFCT_INTRF_EVT_FIELD_OFF,	//!< Reader field lost
	NFCT_INTRF_EVT_SELECTED,	//!< Tag selected by the reader, anticollision done
	NFCT_INTRF_EVT_ERROR,		//!< Frame delay timeout or transfer error
} NFCT_INTRF_EVT;

typedef struct __NfctIntrf_Device NfctIntrfDev_t;

// Frame callback. Runs in the NFCT interrupt context. pFrame holds one
// received reader frame of FrameLen bytes with the CRC already checked and
// stripped by hardware. Keep processing short, the reply window is timed.
typedef void (*NFCTINTRFFRAMECB)(NfctIntrfDev_t * const pDev, const uint8_t *pFrame, int FrameLen);

// Transport event callback. Runs in the NFCT interrupt context.
typedef void (*NFCTINTRFEVTCB)(NfctIntrfDev_t * const pDev, NFCT_INTRF_EVT Evt);

#pragma pack(push, 4)

typedef struct __NfctIntrf_Config {
	uint8_t NfcId1[10];			//!< NFCID1 value
	uint8_t IdLen;				//!< NFCID1 length 4, 7 or 10. 0 keeps the hardware default
	uint8_t SelRes;				//!< Protocol bits of SEL_RES. 0x20 announces ISO-DEP
	NFCTINTRFFRAMECB pFrameCB;	//!< Received frame handler
	NFCTINTRFEVTCB pEvtCB;		//!< Transport event handler, may be null
	void *pCtx;					//!< User context for the callbacks
} NfctIntrfCfg_t;

struct __NfctIntrf_Device {
	DevIntrf_t DevIntrf;		//!< Base device interface
	NFCTINTRFFRAMECB pFrameCB;
	NFCTINTRFEVTCB pEvtCB;
	void *pCtx;
	volatile bool bFieldOn;
	volatile bool bSelected;
	volatile bool bTxPending;	//!< A response frame was queued for the current exchange
	int RxLen;					//!< Length of the frame in RxFrame
	uint8_t RxFrame[NFCT_INTRF_FRAME_MAX];
	uint8_t TxFrame[NFCT_INTRF_FRAME_MAX];
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize the NFCT frame transport.
 *
 * Sets up the nrfx NFCT driver, programs NFCID1 and SEL_RES when given, and
 * enables automatic collision resolution. The peripheral is not enabled here,
 * call DeviceIntrfEnable when the high frequency clock is running.
 *
 * @param	pDev	Pointer to a device state instance
 * @param	pCfg	Pointer to a configuration instance
 *
 * @return	true on success
 */
bool NfctIntrfInit(NfctIntrfDev_t * const pDev, const NfctIntrfCfg_t * const pCfg);

#ifdef __cplusplus
}

class NfctIntrf : public DeviceIntrf {
public:
	bool Init(const NfctIntrfCfg_t &Cfg) {
		return NfctIntrfInit(&vDevData, &Cfg);
	}

	operator DevIntrf_t * const () {
		return &vDevData.DevIntrf;
	}

	operator NfctIntrfDev_t * const () {
		return &vDevData;
	}

	virtual uint32_t Rate(uint32_t RateHz) {
		return DeviceIntrfSetRate(&vDevData.DevIntrf, RateHz);
	}

	virtual uint32_t Rate(void) {
		return vDevData.DevIntrf.GetRate(&vDevData.DevIntrf);
	}

	virtual bool StartRx(uint32_t DevAddr) {
		return DeviceIntrfStartRx(&vDevData.DevIntrf, DevAddr);
	}

	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}

	virtual void StopRx(void) {
		DeviceIntrfStopRx(&vDevData.DevIntrf);
	}

	virtual bool StartTx(uint32_t DevAddr) {
		return DeviceIntrfStartTx(&vDevData.DevIntrf, DevAddr);
	}

	virtual int TxData(const uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}

	virtual void StopTx(void) {
		DeviceIntrfStopTx(&vDevData.DevIntrf);
	}

protected:
	NfctIntrfDev_t vDevData;
};

#endif

#endif	// __NFCT_NRFX_H__

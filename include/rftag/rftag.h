/**-------------------------------------------------------------------------
@file	rftag.h

@brief	RF tag memory device

This module provides an EEPROM-like RF tag object. The physical access is provided by DeviceIntrf.

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
#ifndef __RFTAG_H__
#define __RFTAG_H__

#include <stdint.h>
#include <stdbool.h>

#include "device_intrf.h"
#include "coredev/iopincfg.h"

#pragma pack(push, 4)

typedef enum {
	RFTAG_NDEF_FMT_NONE = 0,		//!< Raw NDEF bytes at NdefAddr
	RFTAG_NDEF_FMT_NLEN16,		//!< 2-byte big-endian length then NDEF bytes
	RFTAG_NDEF_FMT_TLV,			//!< NFC Forum TLV: 0x03 LEN DATA 0xFE
} RFTAG_NDEF_FMT;

typedef enum {
	RFTAG_EVT_FIELD_ON,
	RFTAG_EVT_FIELD_OFF,
	RFTAG_EVT_SELECTED,
	RFTAG_EVT_DESELECTED,
	RFTAG_EVT_READ,
	RFTAG_EVT_WRITE,
	RFTAG_EVT_MEM_CHANGED,
	RFTAG_EVT_ERROR,
} RFTAG_EVT;

typedef struct {
	RFTAG_EVT Evt;
	uint32_t Addr;
	uint32_t Len;
	uint32_t Flags;
} RFTagEvt_t;

typedef void (*RFTAGCB)(void *pCtx, const RFTagEvt_t *pEvt);

typedef bool (*RFTAGDEVOP)(int DevAddr, DevIntrf_t * const pIntrf);

typedef struct __RFTag_Config {
	uint8_t DevAddr;				//!< Device address or selection id
	uint8_t AddrLen;				//!< Memory address length in bytes
	uint16_t PageSize;				//!< Write page size in bytes
	uint32_t Size;					//!< Tag memory size in bytes
	uint32_t WrDelay;				//!< Write delay in msec
	uint32_t NdefAddr;				//!< NDEF storage offset
	uint32_t NdefMaxLen;			//!< Max NDEF payload length. 0 uses Size - NdefAddr
	RFTAG_NDEF_FMT NdefFmt;			//!< NDEF storage layout
	IOPinCfg_t FdPin;				//!< Field-detect pin, optional
	IOPinCfg_t WrProtPin;			//!< Write-protect pin, optional
	RFTAGDEVOP pInitCB;				//!< Optional target init hook
	RFTAGDEVOP pWaitCB;				//!< Optional write wait hook
	RFTAGCB pEvtCB;					//!< Optional RF tag event callback
	void *pCtx;						//!< Callback context
} RFTagCfg_t;

typedef struct __RFTag_Device {
	uint8_t DevAddr;
	uint8_t AddrLen;
	uint16_t PageSize;
	uint32_t Size;
	uint32_t WrDelay;
	uint32_t NdefAddr;
	uint32_t NdefMaxLen;
	RFTAG_NDEF_FMT NdefFmt;
	IOPinCfg_t FdPin;
	IOPinCfg_t WrProtPin;
	DevIntrf_t *pIntrf;
	RFTAGDEVOP pWaitCB;
	RFTAGCB pEvtCB;
	void *pCtx;
} RFTagDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool RFTagInit(RFTagDev_t * const pDev, const RFTagCfg_t * const pCfg, DevIntrf_t * const pIntrf);
int RFTagRead(RFTagDev_t * const pDev, uint32_t Addr, uint8_t *pBuff, int Len);
int RFTagWrite(RFTagDev_t * const pDev, uint32_t Addr, const uint8_t *pData, int Len);

uint32_t RFTagGetSize(RFTagDev_t * const pDev);
uint16_t RFTagGetPageSize(RFTagDev_t * const pDev);

bool RFTagSetNdef(RFTagDev_t * const pDev, const uint8_t *pNdef, uint16_t Len);
int RFTagGetNdef(RFTagDev_t * const pDev, uint8_t *pNdef, uint16_t Len);

void RFTagSetWriteProt(RFTagDev_t * const pDev, bool bVal);
void RFTagEvtDispatch(RFTagDev_t * const pDev, RFTAG_EVT Evt, uint32_t Addr, uint32_t Len, uint32_t Flags);

#ifdef __cplusplus
}

class RFTag {
public:
	RFTag();
	virtual ~RFTag();
	RFTag(RFTag&);		// copy ctor not allowed

	virtual bool Init(const RFTagCfg_t &Cfg, DeviceIntrf * const pIntrf) {
		return RFTagInit(&vDevData, &Cfg, *pIntrf);
	}

	virtual int Read(uint32_t Addr, uint8_t *pBuff, int Len) {
		return RFTagRead(&vDevData, Addr, pBuff, Len);
	}

	virtual int Write(uint32_t Addr, const uint8_t *pData, int Len) {
		return RFTagWrite(&vDevData, Addr, pData, Len);
	}

	virtual bool SetNdef(const uint8_t *pNdef, uint16_t Len) {
		return RFTagSetNdef(&vDevData, pNdef, Len);
	}

	virtual int GetNdef(uint8_t *pNdef, uint16_t Len) {
		return RFTagGetNdef(&vDevData, pNdef, Len);
	}

	uint32_t GetSize() {
		return RFTagGetSize(&vDevData);
	}

	uint16_t GetPageSize() {
		return RFTagGetPageSize(&vDevData);
	}

	void SetWriteProt(bool bVal) {
		RFTagSetWriteProt(&vDevData, bVal);
	}

	operator RFTagDev_t* const () {
		return &vDevData;
	}

protected:
	RFTagDev_t vDevData;
};

#endif

#endif	// __RFTAG_H__

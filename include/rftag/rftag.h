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
	RFTAG_PROTO_NONE = 0,		//!< Raw memory tag, host access over the transport
	RFTAG_PROTO_NFC_T2,			//!< NFC Forum Type 2 Tag
	RFTAG_PROTO_NFC_T4,			//!< NFC Forum Type 4 Tag
	RFTAG_PROTO_ISO15693,		//!< ISO15693 vicinity tag, NFC Forum Type 5
	RFTAG_PROTO_EPC_GEN2,		//!< UHF EPC Gen2, ISO18000-63. Remote or chip tag only
	RFTAG_PROTO_VENDOR,			//!< Vendor specific protocol
} RFTAG_PROTO;

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

// Transport capability flags. They state which protocol layers the transport
// hardware or firmware already performs, so the protocol module can skip them
// instead of running them again in software.
#define RFTAG_XCAP_ANTICOL		(1 << 0)	//!< Activation and anticollision in hardware
#define RFTAG_XCAP_CRC			(1 << 1)	//!< RX frames arrive CRC checked, TX CRC appended
#define RFTAG_XCAP_FDT			(1 << 2)	//!< Reply timing enforced in hardware
#define RFTAG_XCAP_ISODEP		(1 << 3)	//!< Transport delivers bare APDUs, ISO-DEP in the chip
#define RFTAG_XCAP_TAGFULL		(1 << 4)	//!< Chip runs the whole tag, host only syncs memory

// Size of the protocol scratch area kept inside the tag object. Must hold the
// largest protocol state. Protocol modules verify this at compile time.
#define RFTAG_PROTO_STATE_SIZE	64

// Largest response frame a protocol may build.
#define RFTAG_TX_FRAME_MAX		260

typedef struct __RFTag_Device RFTagDev_t;

// Pluggable per protocol behavior, selected by RFTagCfg_t Proto at init.
// OnFrame and OnApdu are the two target entry levels. The transport XCap
// decides which one runs: RFTAG_XCAP_ISODEP routes to OnApdu, otherwise
// OnFrame handles the raw frame including the ISO-DEP layer.
typedef struct {
	bool (*Init)(RFTagDev_t * const pDev);	//!< Per protocol setup, may be null
	// Target path, raw frame level. Process one reader frame in pRx, build a
	// response in pTx. Returns the response length in bytes, 0 for no response.
	int (*OnFrame)(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen,
	               uint8_t *pTx, int TxCap);
	// Target path, APDU level, for transports that run ISO-DEP themselves.
	int (*OnApdu)(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen,
	              uint8_t *pTx, int TxCap);
} RFTagProto_t;

typedef struct __RFTag_Config {
	RFTAG_PROTO Proto;				//!< Tag protocol, selects the tag behavior
	uint32_t XCap;					//!< Transport capability flags, RFTAG_XCAP_*
	uint8_t *pMem;					//!< Local tag memory for target protocols. Null for bus tags
	uint32_t MemSize;				//!< Local tag memory size in bytes
	uint8_t DevAddr;				//!< Device address or selection id
	uint8_t AddrLen;				//!< Memory address length in bytes
	uint16_t PageSize;				//!< Write page size in bytes
	uint32_t Size;					//!< Tag memory size in bytes
	uint32_t WrDelay;				//!< Write delay in msec
	uint32_t NdefAddr;				//!< NDEF storage offset
	uint32_t NdefMaxLen;			//!< Max NDEF area in bytes incl format framing. 0 uses Size - NdefAddr
	RFTAG_NDEF_FMT NdefFmt;			//!< NDEF storage layout
	IOPinCfg_t FdPin;				//!< Field-detect pin, optional
	IOPinCfg_t WrProtPin;			//!< Write-protect pin, optional
	RFTAGDEVOP pInitCB;				//!< Optional target init hook
	RFTAGDEVOP pWaitCB;				//!< Optional write wait hook
	RFTAGCB pEvtCB;					//!< Optional RF tag event callback
	void *pCtx;						//!< Callback context
} RFTagCfg_t;

struct __RFTag_Device {
	RFTAG_PROTO Proto;
	uint32_t XCap;
	uint8_t *pMem;
	uint32_t MemSize;
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
	const RFTagProto_t *pProto;
	uint8_t ProtoState[RFTAG_PROTO_STATE_SIZE];
	uint8_t TxFrame[RFTAG_TX_FRAME_MAX];
};

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

// Target entry. Feed one reader frame from the transport. Runs the configured
// protocol and sends the response frame back through the transport.
int RFTagProcessFrame(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen);

// Protocol bind hook provided by the Type 4 module. Referenced from RFTagInit
// when Proto is RFTAG_PROTO_NFC_T4 so the module object is pulled from the archive.
bool RFTagProtoT4tBind(RFTagDev_t * const pDev);

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

	virtual int ProcessFrame(const uint8_t *pRx, int RxLen) {
		return RFTagProcessFrame(&vDevData, pRx, RxLen);
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

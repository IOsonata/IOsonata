/**-------------------------------------------------------------------------
@file	rftag_controller.h

@brief	RF tag controller object

The controller is the active RF device used to detect and access remote tags.

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
#ifndef __RFTAG_CONTROLLER_H__
#define __RFTAG_CONTROLLER_H__

#include <stdint.h>
#include <stdbool.h>

#include "device_intrf.h"

#pragma pack(push, 4)

typedef enum {
	RF_PROTO_NONE     = 0,
	RF_PROTO_NFCA     = (1u << 0),
	RF_PROTO_NFCB     = (1u << 1),
	RF_PROTO_NFCF     = (1u << 2),
	RF_PROTO_ISO15693 = (1u << 3),
	RF_PROTO_T4T      = (1u << 4),
	RF_PROTO_T5T      = (1u << 5),
	RF_PROTO_MIFARE   = (1u << 6),
} RF_PROTO;

typedef enum {
	RFTAGCTRL_EVT_FIELD_ON,
	RFTAGCTRL_EVT_FIELD_OFF,
	RFTAGCTRL_EVT_TAG_DETECTED,
	RFTAGCTRL_EVT_TAG_REMOVED,
	RFTAGCTRL_EVT_RX_DATA,
	RFTAGCTRL_EVT_TX_DONE,
	RFTAGCTRL_EVT_ERROR,
} RFTAGCTRL_EVT;

typedef struct {
	RFTAGCTRL_EVT Evt;
	uint32_t Flags;
	const uint8_t *pData;
	int Len;
} RFTagCtrlEvt_t;

typedef void (*RFTAGCTRLCB)(void *pCtx, const RFTagCtrlEvt_t *pEvt);

typedef struct {
	uint32_t Proto;
	uint8_t Uid[16];
	uint8_t UidLen;
	uint8_t Type;
	uint32_t Size;
	uint16_t BlockSize;
	uint32_t Flags;
} RFTagInfo_t;

typedef struct {
	uint8_t DevAddr;
	uint32_t ProtoMask;
	uint32_t Bitrate;
	uint32_t TimeoutMs;
	RFTAGCTRLCB pEvtCB;
	void *pCtx;
} RFTagControllerCfg_t;

typedef struct {
	uint8_t DevAddr;
	uint32_t ProtoMask;
	uint32_t Bitrate;
	uint32_t TimeoutMs;
	DevIntrf_t *pIntrf;
	RFTAGCTRLCB pEvtCB;
	void *pCtx;
} RFTagControllerDev_t;

/**
 * @brief IOsonata RFTagController adapter command code.
 *
 * These commands are used by adapter implementations that expose an RF tag
 * controller through DeviceIntrf. RFTagControllerTransceive() is raw and does
 * not wrap data with one of these command values.
 */
typedef enum {
	RFTAGCTRL_CMD_DETECT = 1,
	RFTAGCTRL_CMD_SELECT,
	RFTAGCTRL_CMD_TAG_READ,
	RFTAGCTRL_CMD_TAG_WRITE,
} RFTAGCTRL_CMD;

/**
 * @brief IOsonata adapter command for remote tag memory access.
 *
 * This is not a chip command format. PN7160, PN532, ST25R and vendor API
 * ports translate this structure to their own command sequence.
 */
typedef struct {
	uint8_t Cmd;
	uint8_t UidLen;
	uint16_t Len;
	uint32_t Proto;
	uint32_t Addr;
	uint8_t Uid[16];
} RFTagControllerMemCmd_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool RFTagControllerInit(RFTagControllerDev_t * const pDev,
                         const RFTagControllerCfg_t * const pCfg,
                         DevIntrf_t * const pIntrf);

void RFTagControllerEnable(RFTagControllerDev_t * const pDev);
void RFTagControllerDisable(RFTagControllerDev_t * const pDev);

bool RFTagControllerDetect(RFTagControllerDev_t * const pDev,
                           RFTagInfo_t * const pTag);

bool RFTagControllerSelect(RFTagControllerDev_t * const pDev,
                           const RFTagInfo_t * const pTag);

int RFTagControllerTagRead(RFTagControllerDev_t * const pDev,
                           const RFTagInfo_t * const pTag,
                           uint32_t Addr,
                           uint8_t *pBuff,
                           int Len);

int RFTagControllerTagWrite(RFTagControllerDev_t * const pDev,
                            const RFTagInfo_t * const pTag,
                            uint32_t Addr,
                            const uint8_t *pData,
                            int Len);

int RFTagControllerTransceive(RFTagControllerDev_t * const pDev,
                              const uint8_t *pTx,
                              int TxLen,
                              uint8_t *pRx,
                              int RxLen);

void RFTagControllerEvtDispatch(RFTagControllerDev_t * const pDev,
                                RFTAGCTRL_EVT Evt,
                                const uint8_t *pData,
                                int Len,
                                uint32_t Flags);

#ifdef __cplusplus
}

class RFTagController {
public:
	RFTagController();
	virtual ~RFTagController();
	RFTagController(RFTagController&);	// copy ctor not allowed

	virtual bool Init(const RFTagControllerCfg_t &Cfg, DeviceIntrf * const pIntrf) {
		return RFTagControllerInit(&vDevData, &Cfg, *pIntrf);
	}

	virtual void Enable() {
		RFTagControllerEnable(&vDevData);
	}

	virtual void Disable() {
		RFTagControllerDisable(&vDevData);
	}

	virtual bool Detect(RFTagInfo_t * const pTag) {
		return RFTagControllerDetect(&vDevData, pTag);
	}

	virtual bool Select(const RFTagInfo_t * const pTag) {
		return RFTagControllerSelect(&vDevData, pTag);
	}

	virtual int TagRead(const RFTagInfo_t * const pTag, uint32_t Addr, uint8_t *pBuff, int Len) {
		return RFTagControllerTagRead(&vDevData, pTag, Addr, pBuff, Len);
	}

	virtual int TagWrite(const RFTagInfo_t * const pTag, uint32_t Addr, const uint8_t *pData, int Len) {
		return RFTagControllerTagWrite(&vDevData, pTag, Addr, pData, Len);
	}

	virtual int Transceive(const uint8_t *pTx, int TxLen, uint8_t *pRx, int RxLen) {
		return RFTagControllerTransceive(&vDevData, pTx, TxLen, pRx, RxLen);
	}

	operator RFTagControllerDev_t* const () {
		return &vDevData;
	}

protected:
	RFTagControllerDev_t vDevData;
};

#endif

#endif	// __RFTAG_CONTROLLER_H__

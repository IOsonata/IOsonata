/*--------------------------------------------------------------------------
File   : rftag_controller.cpp

Author : Hoang Nguyen Hoan          Jul. 5, 2026

Desc   : RF tag controller implementation

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <string.h>

#include "istddef.h"
#include "rftag/rftag_controller.h"

RFTagController::RFTagController()
{
	memset(&vDevData, 0, sizeof(RFTagControllerDev_t));
}

RFTagController::~RFTagController()
{
}

bool RFTagControllerInit(RFTagControllerDev_t * const pDev,
                         const RFTagControllerCfg_t * const pCfg,
                         DevIntrf_t * const pIntrf)
{
	if (pDev == nullptr || pCfg == nullptr || pIntrf == nullptr)
	{
		return false;
	}

	memset(pDev, 0, sizeof(RFTagControllerDev_t));

	pDev->DevAddr = pCfg->DevAddr;
	pDev->ProtoMask = pCfg->ProtoMask;
	pDev->Bitrate = pCfg->Bitrate;
	pDev->TimeoutMs = pCfg->TimeoutMs;
	pDev->pIntrf = pIntrf;
	pDev->pEvtCB = pCfg->pEvtCB;
	pDev->pCtx = pCfg->pCtx;

	if (pCfg->Bitrate > 0)
	{
		DeviceIntrfSetRate(pIntrf, pCfg->Bitrate);
	}

	return true;
}

void RFTagControllerEnable(RFTagControllerDev_t * const pDev)
{
	if (pDev == nullptr || pDev->pIntrf == nullptr)
	{
		return;
	}

	DeviceIntrfEnable(pDev->pIntrf);
}

void RFTagControllerDisable(RFTagControllerDev_t * const pDev)
{
	if (pDev == nullptr || pDev->pIntrf == nullptr)
	{
		return;
	}

	DeviceIntrfDisable(pDev->pIntrf);
}

bool RFTagControllerDetect(RFTagControllerDev_t * const pDev,
                           RFTagInfo_t * const pTag)
{
	if (pDev == nullptr || pDev->pIntrf == nullptr || pTag == nullptr)
	{
		return false;
	}

	uint8_t cmd[5];

	cmd[0] = RFTAGCTRL_CMD_DETECT;
	cmd[1] = (uint8_t)pDev->ProtoMask;
	cmd[2] = (uint8_t)(pDev->ProtoMask >> 8);
	cmd[3] = (uint8_t)(pDev->ProtoMask >> 16);
	cmd[4] = (uint8_t)(pDev->ProtoMask >> 24);

	memset(pTag, 0, sizeof(RFTagInfo_t));

	int l = DeviceIntrfRead(pDev->pIntrf, pDev->DevAddr, cmd, sizeof(cmd), (uint8_t*)pTag, sizeof(RFTagInfo_t));

	if (l == (int)sizeof(RFTagInfo_t) && pTag->Proto != RF_PROTO_NONE)
	{
		RFTagControllerEvtDispatch(pDev, RFTAGCTRL_EVT_TAG_DETECTED, (const uint8_t*)pTag, sizeof(RFTagInfo_t), pTag->Proto);
		return true;
	}

	return false;
}

bool RFTagControllerSelect(RFTagControllerDev_t * const pDev,
                           const RFTagInfo_t * const pTag)
{
	if (pDev == nullptr || pDev->pIntrf == nullptr || pTag == nullptr)
	{
		return false;
	}

	uint8_t cmd = RFTAGCTRL_CMD_SELECT;

	int l = DeviceIntrfWrite(pDev->pIntrf, pDev->DevAddr, &cmd, sizeof(cmd), (const uint8_t*)pTag, sizeof(RFTagInfo_t));

	return l == (int)sizeof(RFTagInfo_t);
}

static void RFTagControllerFillMemCmd(RFTagControllerMemCmd_t *pCmd,
                                      uint8_t Cmd,
                                      const RFTagInfo_t * const pTag,
                                      uint32_t Addr,
                                      uint16_t Len)
{
	memset(pCmd, 0, sizeof(RFTagControllerMemCmd_t));

	pCmd->Cmd = Cmd;
	pCmd->Len = Len;
	pCmd->Proto = pTag ? pTag->Proto : RF_PROTO_NONE;
	pCmd->Addr = Addr;

	if (pTag)
	{
		pCmd->UidLen = pTag->UidLen > sizeof(pCmd->Uid) ? sizeof(pCmd->Uid) : pTag->UidLen;
		memcpy(pCmd->Uid, pTag->Uid, pCmd->UidLen);
	}
}

int RFTagControllerTagRead(RFTagControllerDev_t * const pDev,
                           const RFTagInfo_t * const pTag,
                           uint32_t Addr,
                           uint8_t *pBuff,
                           int Len)
{
	if (pDev == nullptr || pDev->pIntrf == nullptr || pTag == nullptr || pBuff == nullptr || Len <= 0 || Len > UINT16_MAX)
	{
		return 0;
	}

	RFTagControllerMemCmd_t cmd;

	RFTagControllerFillMemCmd(&cmd, RFTAGCTRL_CMD_TAG_READ, pTag, Addr, (uint16_t)Len);

	int l = DeviceIntrfRead(pDev->pIntrf, pDev->DevAddr, (const uint8_t*)&cmd, sizeof(cmd), pBuff, Len);

	if (l > 0)
	{
		RFTagControllerEvtDispatch(pDev, RFTAGCTRL_EVT_RX_DATA, pBuff, l, 0);
	}

	return l;
}

int RFTagControllerTagWrite(RFTagControllerDev_t * const pDev,
                            const RFTagInfo_t * const pTag,
                            uint32_t Addr,
                            const uint8_t *pData,
                            int Len)
{
	if (pDev == nullptr || pDev->pIntrf == nullptr || pTag == nullptr || pData == nullptr || Len <= 0 || Len > UINT16_MAX)
	{
		return 0;
	}

	RFTagControllerMemCmd_t cmd;

	RFTagControllerFillMemCmd(&cmd, RFTAGCTRL_CMD_TAG_WRITE, pTag, Addr, (uint16_t)Len);

	int l = DeviceIntrfWrite(pDev->pIntrf, pDev->DevAddr, (const uint8_t*)&cmd, sizeof(cmd), pData, Len);

	if (l > 0)
	{
		RFTagControllerEvtDispatch(pDev, RFTAGCTRL_EVT_TX_DONE, nullptr, l, 0);
	}

	return l;
}

int RFTagControllerTransceive(RFTagControllerDev_t * const pDev,
                              const uint8_t *pTx,
                              int TxLen,
                              uint8_t *pRx,
                              int RxLen)
{
	if (pDev == nullptr || pDev->pIntrf == nullptr || pTx == nullptr || TxLen <= 0)
	{
		return 0;
	}

	if (pRx == nullptr || RxLen <= 0)
	{
		return DeviceIntrfTx(pDev->pIntrf, pDev->DevAddr, pTx, TxLen);
	}

	int l = DeviceIntrfRead(pDev->pIntrf, pDev->DevAddr, pTx, TxLen, pRx, RxLen);

	if (l > 0)
	{
		RFTagControllerEvtDispatch(pDev, RFTAGCTRL_EVT_RX_DATA, pRx, l, 0);
	}

	return l;
}

void RFTagControllerEvtDispatch(RFTagControllerDev_t * const pDev,
                                RFTAGCTRL_EVT Evt,
                                const uint8_t *pData,
                                int Len,
                                uint32_t Flags)
{
	if (pDev == nullptr || pDev->pEvtCB == nullptr)
	{
		return;
	}

	RFTagCtrlEvt_t e = {
		.Evt = Evt,
		.Flags = Flags,
		.pData = pData,
		.Len = Len,
	};

	pDev->pEvtCB(pDev->pCtx, &e);
}

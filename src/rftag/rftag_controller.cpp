/**-------------------------------------------------------------------------
@file	rftag_controller.cpp

@brief	RF tag controller facet implementation

Reader side over a DeviceIntrf transport. Detect scans the field and fills a
tag description, Select activates a tag, TagRead and TagWrite move remote tag
memory through the adapter memory command, Transceive sends raw frames. Reader
activity is reported through the per instance EvtHandler.

@author	Hoang Nguyen Hoan
@date	Jul. 7, 2026

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
#include <string.h>

#include "istddef.h"
#include "rftag/rftag_controller.h"

bool RFTagController::Init(const RFTagControllerCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	Valid(false);
	Interface(nullptr);
	memset(&vCfg, 0, sizeof(vCfg));

	if (pIntrf == nullptr)
	{
		return false;
	}

	vCfg = Cfg;
	Interface(pIntrf);

	if (vCfg.Bitrate > 0)
	{
		DeviceIntrfSetRate((DevIntrf_t *)*pIntrf, vCfg.Bitrate);
	}

	Valid(true);

	return true;
}

bool RFTagController::Enable()
{
	if (Valid() == false || vpIntrf == nullptr)
	{
		return false;
	}

	DeviceIntrfEnable((DevIntrf_t *)*vpIntrf);

	return true;
}

void RFTagController::Disable()
{
	if (Valid() == false || vpIntrf == nullptr)
	{
		return;
	}

	DeviceIntrfDisable((DevIntrf_t *)*vpIntrf);
}

void RFTagController::Reset()
{
	if (Valid() == false || vpIntrf == nullptr)
	{
		return;
	}

	DeviceIntrfReset((DevIntrf_t *)*vpIntrf);
}

bool RFTagController::Detect(RFTagInfo_t * const pTag)
{
	if (Valid() == false || vpIntrf == nullptr || pTag == nullptr)
	{
		return false;
	}

	uint8_t cmd[5];

	cmd[0] = RFTAGCTRL_CMD_DETECT;
	cmd[1] = (uint8_t)vCfg.ProtoMask;
	cmd[2] = (uint8_t)(vCfg.ProtoMask >> 8);
	cmd[3] = (uint8_t)(vCfg.ProtoMask >> 16);
	cmd[4] = (uint8_t)(vCfg.ProtoMask >> 24);

	memset(pTag, 0, sizeof(RFTagInfo_t));

	int l = DeviceIntrfRead((DevIntrf_t *)*vpIntrf, vCfg.DevAddr, cmd, sizeof(cmd),
							(uint8_t *)pTag, sizeof(RFTagInfo_t));

	if (l == (int)sizeof(RFTagInfo_t) && pTag->Proto != RF_PROTO_NONE)
	{
		EvtHandler(RFTAGCTRL_EVT_TAG_DETECTED, pTag->Proto, 0);
		return true;
	}

	return false;
}

bool RFTagController::Select(const RFTagInfo_t * const pTag)
{
	if (Valid() == false || vpIntrf == nullptr || pTag == nullptr)
	{
		return false;
	}

	uint8_t cmd = RFTAGCTRL_CMD_SELECT;

	int l = DeviceIntrfWrite((DevIntrf_t *)*vpIntrf, vCfg.DevAddr, &cmd, sizeof(cmd),
							 (const uint8_t *)pTag, sizeof(RFTagInfo_t));

	return l == (int)sizeof(RFTagInfo_t);
}

// Fill the adapter memory command from the target tag and the access range.
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

int RFTagController::TagRead(const RFTagInfo_t * const pTag, uint32_t Addr, uint8_t *pBuff, int Len)
{
	if (Valid() == false || vpIntrf == nullptr || pTag == nullptr || pBuff == nullptr || Len <= 0 || Len > UINT16_MAX)
	{
		return 0;
	}

	RFTagControllerMemCmd_t cmd;

	RFTagControllerFillMemCmd(&cmd, RFTAGCTRL_CMD_TAG_READ, pTag, Addr, (uint16_t)Len);

	int l = DeviceIntrfRead((DevIntrf_t *)*vpIntrf, vCfg.DevAddr,
							(const uint8_t *)&cmd, sizeof(cmd), pBuff, Len);

	if (l > 0)
	{
		EvtHandler(RFTAGCTRL_EVT_RX_DATA, (uint32_t)l, 0);
	}

	return l;
}

int RFTagController::TagWrite(const RFTagInfo_t * const pTag, uint32_t Addr, const uint8_t *pData, int Len)
{
	if (Valid() == false || vpIntrf == nullptr || pTag == nullptr || pData == nullptr || Len <= 0 || Len > UINT16_MAX)
	{
		return 0;
	}

	RFTagControllerMemCmd_t cmd;

	RFTagControllerFillMemCmd(&cmd, RFTAGCTRL_CMD_TAG_WRITE, pTag, Addr, (uint16_t)Len);

	int l = DeviceIntrfWrite((DevIntrf_t *)*vpIntrf, vCfg.DevAddr,
							 (const uint8_t *)&cmd, sizeof(cmd), pData, Len);

	if (l > 0)
	{
		EvtHandler(RFTAGCTRL_EVT_TX_DONE, (uint32_t)l, 0);
	}

	return l;
}

int RFTagController::Transceive(const uint8_t *pTx, int TxLen, uint8_t *pRx, int RxLen)
{
	if (Valid() == false || vpIntrf == nullptr || pTx == nullptr || TxLen <= 0)
	{
		return 0;
	}

	if (pRx == nullptr || RxLen <= 0)
	{
		return DeviceIntrfTx((DevIntrf_t *)*vpIntrf, vCfg.DevAddr, pTx, TxLen);
	}

	int l = DeviceIntrfRead((DevIntrf_t *)*vpIntrf, vCfg.DevAddr, pTx, TxLen, pRx, RxLen);

	if (l > 0)
	{
		EvtHandler(RFTAGCTRL_EVT_RX_DATA, (uint32_t)l, 0);
	}

	return l;
}

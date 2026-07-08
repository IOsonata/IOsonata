/**-------------------------------------------------------------------------
@file	rfreader_pn532.cpp

@brief	PN532 NFC reader port of the RFTagController facet

Maps the controller methods to PN532 command frames over the inherited
DeviceIntrf. BuildFrame assembles a host to PN532 frame, Command sends it and
reads the response after the ACK, DataExchange runs InDataExchange to the
activated target. Detect uses InListPassiveTarget, Transceive passes a payload
straight through to the target.

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

#include "device_intrf.h"
#include "rftag/rfreader_pn532.h"

int RfReaderPn532::BuildFrame(uint8_t Cmd, const uint8_t *pData, int DataLen,
							  uint8_t *pFrame, int Cap)
{
	// Frame is preamble, start codes, LEN, LCS, TFI, command, data, DCS,
	// postamble. LEN counts TFI, the command and the data bytes.
	int len = 2 + DataLen;			// TFI and command, plus data
	int need = 7 + DataLen + 2;		// header 6, LEN body, DCS, postamble

	if (pFrame == nullptr || Cap < need || DataLen < 0 || len > 0xFF)
	{
		return -1;
	}

	int n = 0;

	pFrame[n++] = PN532_PREAMBLE;
	pFrame[n++] = PN532_STARTCODE1;
	pFrame[n++] = PN532_STARTCODE2;
	pFrame[n++] = (uint8_t)len;
	pFrame[n++] = (uint8_t)(-len);			// LCS, LEN plus LCS is zero
	pFrame[n++] = PN532_HOSTTOPN532;
	pFrame[n++] = Cmd;

	uint8_t sum = PN532_HOSTTOPN532 + Cmd;

	for (int i = 0; i < DataLen; i++)
	{
		pFrame[n++] = pData[i];
		sum += pData[i];
	}

	pFrame[n++] = (uint8_t)(-sum);			// DCS, TFI plus data plus DCS is zero
	pFrame[n++] = PN532_POSTAMBLE;

	return n;
}

int RfReaderPn532::Command(uint8_t Cmd, const uint8_t *pData, int DataLen,
						   uint8_t *pRsp, int RspCap)
{
	uint8_t frame[PN532_FRAME_DATA_MAX + 12];
	uint8_t rd[PN532_FRAME_DATA_MAX + 12];

	if (vpIntrf == nullptr)
	{
		return -1;
	}

	int flen = BuildFrame(Cmd, pData, DataLen, frame, sizeof(frame));

	if (flen < 0)
	{
		return -1;
	}

	// Send the command frame, then read the ACK and response frame. The wire
	// handshake, ACK wait and the interface specific read length are the
	// bus port work. This is the transport hook to fill in for a bus.
	if (DeviceIntrfTx((DevIntrf_t *)*vpIntrf, DevAddr(), frame, flen) != flen)
	{
		return -1;
	}

	int rl = DeviceIntrfRx((DevIntrf_t *)*vpIntrf, DevAddr(), rd, (int)sizeof(rd));

	if (rl <= 0)
	{
		return -1;
	}

	// The read holds the ACK frame 00 00 FF 00 FF 00 then the response frame.
	// Skip a leading ACK when present, then locate the response frame start.
	int i = 0;
	static const uint8_t ack[6] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00 };

	if (rl >= 6 && memcmp(rd, ack, sizeof(ack)) == 0)
	{
		i = 6;
	}

	// Locate the frame start code 00 FF, skipping any preamble zeros.
	while (i + 2 < rl && !(rd[i] == PN532_STARTCODE1 && rd[i + 1] == PN532_STARTCODE2))
	{
		i++;
	}

	if (i + 4 >= rl)
	{
		return -1;
	}

	i += 2;								// past the start codes 00 FF
	int len = rd[i++];
	i++;								// skip LCS

	if (len < 2 || i + len > rl)
	{
		return -1;
	}

	if (rd[i++] != PN532_PN532TOHOST || rd[i++] != PN532_RSP_CODE(Cmd))
	{
		return -1;
	}

	int dl = len - 2;				// data after TFI and response command

	if (pRsp != nullptr && dl > 0)
	{
		int c = dl < RspCap ? dl : RspCap;
		memcpy(pRsp, &rd[i], c);
		return c;
	}

	return dl;
}

int RfReaderPn532::DataExchange(const uint8_t *pTx, int TxLen, uint8_t *pRx, int RxCap)
{
	uint8_t d[PN532_FRAME_DATA_MAX];
	uint8_t r[PN532_FRAME_DATA_MAX];

	if (pTx == nullptr || TxLen <= 0 || TxLen + 1 > (int)sizeof(d))
	{
		return -1;
	}

	// InDataExchange data is the target number then the payload.
	d[0] = vTg;
	memcpy(&d[1], pTx, TxLen);

	int rl = Command(PN532_CMD_INDATAEXCHANGE, d, TxLen + 1, r, sizeof(r));

	if (rl < 1)
	{
		return -1;
	}

	// r[0] is the status byte, 0 on success. The rest is the tag answer.
	if (r[0] != 0x00)
	{
		return -1;
	}

	int dl = rl - 1;

	if (pRx != nullptr && dl > 0)
	{
		int c = dl < RxCap ? dl : RxCap;
		memcpy(pRx, &r[1], c);
		return c;
	}

	return dl;
}

bool RfReaderPn532::Init(const RFTagControllerCfg_t &Cfg, const RfReaderPn532Cfg_t &Pn,
						 DeviceIntrf * const pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}

	vCfg = Cfg;
	Interface(pIntrf);

	vTargetType = Pn.TargetType;
	vMaxTargets = Pn.MaxTargets != 0 ? Pn.MaxTargets : 1;
	vbSamNormal = Pn.bSamNormal;
	vTg = 1;

	if (vCfg.Bitrate > 0)
	{
		DeviceIntrfSetRate((DevIntrf_t *)*pIntrf, vCfg.Bitrate);
	}

	// Presence check, read the firmware version.
	uint8_t ver[4];

	if (Command(PN532_CMD_GETFIRMWAREVERSION, nullptr, 0, ver, sizeof(ver)) < 0)
	{
		return false;
	}

	if (vbSamNormal)
	{
		// SAMConfiguration normal mode, no timeout, no IRQ.
		uint8_t sam[3] = { 0x01, 0x00, 0x00 };

		if (Command(PN532_CMD_SAMCONFIGURATION, sam, sizeof(sam), nullptr, 0) < 0)
		{
			return false;
		}
	}

	Valid(true);

	return true;
}

bool RfReaderPn532::Init(const RFTagControllerCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	RfReaderPn532Cfg_t pn;

	pn.TargetType = PN532_BRTY_106K_TYPEA;
	pn.MaxTargets = 1;
	pn.bSamNormal = true;

	return Init(Cfg, pn, pIntrf);
}

bool RfReaderPn532::Enable()
{
	if (vpIntrf == nullptr)
	{
		return false;
	}

	DeviceIntrfEnable((DevIntrf_t *)*vpIntrf);

	return true;
}

void RfReaderPn532::Disable()
{
	if (vpIntrf == nullptr)
	{
		return;
	}

	DeviceIntrfDisable((DevIntrf_t *)*vpIntrf);
}

void RfReaderPn532::Reset()
{
	if (vpIntrf == nullptr)
	{
		return;
	}

	DeviceIntrfReset((DevIntrf_t *)*vpIntrf);
}

bool RfReaderPn532::Detect(RFTagInfo_t * const pTag)
{
	uint8_t d[2];
	uint8_t r[PN532_FRAME_DATA_MAX];

	if (pTag == nullptr)
	{
		return false;
	}

	// InListPassiveTarget: MaxTg then BrTy. Some target types add init data.
	d[0] = vMaxTargets;
	d[1] = vTargetType;

	int rl = Command(PN532_CMD_INLISTPASSIVETARGET, d, sizeof(d), r, sizeof(r));

	if (rl < 1 || r[0] == 0)
	{
		// r[0] is NbTg, the number of targets found.
		return false;
	}

	memset(pTag, 0, sizeof(RFTagInfo_t));

	// Type A target data layout, r[1] is Tg number, then SENS_RES, SEL_RES,
	// UID length and UID. The exact layout depends on the target type, this
	// is the Type A case.
	if (vTargetType == PN532_BRTY_106K_TYPEA && rl >= 6)
	{
		vTg = r[1];
		pTag->Proto = RF_PROTO_NFCA;
		int uidLen = r[5];

		if (uidLen > (int)sizeof(pTag->Uid))
		{
			uidLen = (int)sizeof(pTag->Uid);
		}

		if (6 + uidLen <= rl)
		{
			pTag->UidLen = (uint8_t)uidLen;
			memcpy(pTag->Uid, &r[6], uidLen);
		}
	}
	else if (vTargetType == PN532_BRTY_ISO15693)
	{
		vTg = r[1];
		pTag->Proto = RF_PROTO_ISO15693;
	}

	EvtHandler(RFTAGCTRL_EVT_TAG_DETECTED, pTag->Proto, 0);

	return true;
}

bool RfReaderPn532::Select(const RFTagInfo_t * const pTag)
{
	// InListPassiveTarget already activated the target. Record the target
	// number for InDataExchange. A multi target port would match pTag to a
	// stored list, this single target port uses the last activated one.
	(void)pTag;

	return vTg != 0;
}

int RfReaderPn532::TagRead(const RFTagInfo_t * const pTag, uint32_t Addr, uint8_t *pBuff, int Len)
{
	// Building the read command is protocol specific. For a Type 4 tag it is a
	// ReadBinary APDU, for a Type 2 tag a READ command, for ISO15693 a Read
	// Single Block. That per protocol command construction is the initiator
	// side mirror of the tag engines and belongs in a follow on. The wire path
	// is Transceive of the built command.
	(void)pTag; (void)Addr; (void)pBuff; (void)Len;

	return 0;
}

int RfReaderPn532::TagWrite(const RFTagInfo_t * const pTag, uint32_t Addr, const uint8_t *pData, int Len)
{
	// Protocol specific write command, see TagRead. Follow on work.
	(void)pTag; (void)Addr; (void)pData; (void)Len;

	return 0;
}

int RfReaderPn532::Transceive(const uint8_t *pTx, int TxLen, uint8_t *pRx, int RxLen)
{
	int l = DataExchange(pTx, TxLen, pRx, RxLen);

	if (l < 0)
	{
		return 0;
	}

	if (l > 0)
	{
		EvtHandler(RFTAGCTRL_EVT_RX_DATA, (uint32_t)l, 0);
	}

	return l;
}

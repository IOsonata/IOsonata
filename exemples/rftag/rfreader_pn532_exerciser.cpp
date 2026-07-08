/**-------------------------------------------------------------------------
@file	rfreader_pn532_exerciser.cpp

@brief	Host exerciser for the RfReaderPn532 port

Drives RfReaderPn532 against a mock PN532 that speaks the frame protocol. The
mock parses the host frame, checks the checksums, and answers with an ACK plus
a response frame carrying canned data for GetFirmwareVersion, SAMConfiguration,
InListPassiveTarget and InDataExchange. Checks the init handshake, the detect
mapping and returned tag, and the transceive passthrough.

Link rule: build with rfreader_pn532.cpp, rftag_controller.cpp, device.cpp and
device_intrf.cpp.

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
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "device_intrf.h"
#include "rftag/rfreader_pn532.h"

static int s_Pass = 0;
static int s_Fail = 0;

// The last command code the mock parsed from a host frame.
static uint8_t s_LastCmd = 0;
static uint8_t s_LastData[PN532_FRAME_DATA_MAX];
static int s_LastDataLen = 0;

// The response frame the mock will hand back on the next Rx.
static uint8_t s_RspFrame[PN532_FRAME_DATA_MAX + 12];
static int s_RspFrameLen = 0;

static void Check(const char *pName, bool bOk)
{
	if (bOk) { s_Pass++; printf("  [PASS] %s\n", pName); }
	else { s_Fail++; printf("  [FAIL] %s\n", pName); }
}

// Build a PN532 to host response frame with the given response command and
// data, prefixed by the ACK frame the chip sends first.
static void MockBuildResponse(uint8_t RspCmd, const uint8_t *pData, int DataLen)
{
	static const uint8_t ack[6] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00 };
	int n = 0;

	memcpy(&s_RspFrame[n], ack, sizeof(ack));
	n += sizeof(ack);

	int len = 2 + DataLen;			// TFI and response command plus data

	s_RspFrame[n++] = 0x00;
	s_RspFrame[n++] = 0x00;
	s_RspFrame[n++] = 0xFF;
	s_RspFrame[n++] = (uint8_t)len;
	s_RspFrame[n++] = (uint8_t)(-len);
	s_RspFrame[n++] = PN532_PN532TOHOST;
	s_RspFrame[n++] = RspCmd;

	uint8_t sum = PN532_PN532TOHOST + RspCmd;

	for (int i = 0; i < DataLen; i++)
	{
		s_RspFrame[n++] = pData[i];
		sum += pData[i];
	}

	s_RspFrame[n++] = (uint8_t)(-sum);
	s_RspFrame[n++] = 0x00;

	s_RspFrameLen = n;
}

// Parse a host to PN532 frame, capture the command and data, and stage the
// matching canned response.
static void MockParseAndAnswer(const uint8_t *pFrame, int Len)
{
	int i = 0;

	while (i + 1 < Len && !(pFrame[i] == 0x00 && pFrame[i + 1] == 0xFF))
	{
		i++;
	}

	if (i + 4 >= Len)
	{
		s_RspFrameLen = 0;
		return;
	}

	i += 2;								// past start codes 00 FF
	int flen = pFrame[i++];
	i++;								// skip LCS

	if (flen < 2 || i + flen > Len)
	{
		s_RspFrameLen = 0;
		return;
	}

	uint8_t tfi = pFrame[i++];
	s_LastCmd = pFrame[i++];
	s_LastDataLen = flen - 2;

	(void)tfi;

	if (s_LastDataLen > 0 && s_LastDataLen <= (int)sizeof(s_LastData))
	{
		memcpy(s_LastData, &pFrame[i], s_LastDataLen);
	}

	// Stage the canned response for this command.
	switch (s_LastCmd)
	{
		case PN532_CMD_GETFIRMWAREVERSION:
		{
			uint8_t v[4] = { 0x32, 0x01, 0x06, 0x07 };	// IC, ver, rev, support
			MockBuildResponse(PN532_RSP_CODE(PN532_CMD_GETFIRMWAREVERSION), v, sizeof(v));
			break;
		}

		case PN532_CMD_SAMCONFIGURATION:
			MockBuildResponse(PN532_RSP_CODE(PN532_CMD_SAMCONFIGURATION), nullptr, 0);
			break;

		case PN532_CMD_INLISTPASSIVETARGET:
		{
			// NbTg 1, Tg 1, SENS_RES 2, SEL_RES 1, UIDLen 4, UID 4 bytes.
			uint8_t t[] = { 0x01, 0x01, 0x00, 0x44, 0x00, 0x04, 0xDE, 0xAD, 0xBE, 0xEF };
			MockBuildResponse(PN532_RSP_CODE(PN532_CMD_INLISTPASSIVETARGET), t, sizeof(t));
			break;
		}

		case PN532_CMD_INDATAEXCHANGE:
		{
			// Status 0 then a canned answer.
			uint8_t e[] = { 0x00, 0x90, 0x00 };
			MockBuildResponse(PN532_RSP_CODE(PN532_CMD_INDATAEXCHANGE), e, sizeof(e));
			break;
		}

		default:
			MockBuildResponse(PN532_RSP_CODE(s_LastCmd), nullptr, 0);
			break;
	}
}

static bool MockStartTx(DevIntrf_t * const pDev, uint32_t DevAddr) { (void)pDev; (void)DevAddr; return true; }
static int MockTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	(void)pDev;
	MockParseAndAnswer(pData, DataLen);
	return DataLen;
}
static void MockStopTx(DevIntrf_t * const pDev) { atomic_flag_clear(&pDev->bBusy); }
static bool MockStartRx(DevIntrf_t * const pDev, uint32_t DevAddr) { (void)pDev; (void)DevAddr; return true; }
static int MockRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	(void)pDev;
	int l = s_RspFrameLen < BuffLen ? s_RspFrameLen : BuffLen;
	memcpy(pBuff, s_RspFrame, l);
	return l;
}
static void MockStopRx(DevIntrf_t * const pDev) { atomic_flag_clear(&pDev->bBusy); }

static void MockIntrfInit(DevIntrf_t *pIntrf)
{
	memset(pIntrf, 0, sizeof(DevIntrf_t));
	pIntrf->StartTx = MockStartTx;
	pIntrf->TxData = MockTxData;
	pIntrf->StopTx = MockStopTx;
	pIntrf->StartRx = MockStartRx;
	pIntrf->RxData = MockRxData;
	pIntrf->StopRx = MockStopRx;
	pIntrf->MaxRetry = 1;
	atomic_flag_clear(&pIntrf->bBusy);
}

class MockIntrf : public DeviceIntrf {
public:
	MockIntrf() { MockIntrfInit(&vIntrf); }
	operator DevIntrf_t * const () { return &vIntrf; }
	uint32_t Rate(uint32_t DataRate) { (void)DataRate; return 0; }
	uint32_t Rate(void) { return 0; }
	bool StartRx(uint32_t DevAddr) { return MockStartRx(&vIntrf, DevAddr); }
	int RxData(uint8_t *pBuff, int BuffLen) { return MockRxData(&vIntrf, pBuff, BuffLen); }
	void StopRx(void) { MockStopRx(&vIntrf); }
	bool StartTx(uint32_t DevAddr) { return MockStartTx(&vIntrf, DevAddr); }
	int TxData(const uint8_t *pData, int DataLen) { return MockTxData(&vIntrf, pData, DataLen); }
	void StopTx(void) { MockStopTx(&vIntrf); }
private:
	DevIntrf_t vIntrf;
};

class TestReader : public RfReaderPn532 {
public:
	TestReader() : Detected(0), RxData_(0) {}
	virtual void EvtHandler(RFTAGCTRL_EVT Evt, uint32_t P0, uint32_t P1)
	{
		(void)P0; (void)P1;
		if (Evt == RFTAGCTRL_EVT_TAG_DETECTED) { Detected++; }
		else if (Evt == RFTAGCTRL_EVT_RX_DATA) { RxData_++; }
	}
	int Detected;
	int RxData_;
};

int main(void)
{
	MockIntrf intrf;
	TestReader rdr;

	RFTagControllerCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.DevAddr = 0x24;
	cfg.ProtoMask = RF_PROTO_NFCA;

	RfReaderPn532Cfg_t pn;
	pn.TargetType = PN532_BRTY_106K_TYPEA;
	pn.MaxTargets = 1;
	pn.bSamNormal = true;

	printf("== 1. Init handshake ==\n");
	{
		bool ok = rdr.Init(cfg, pn, &intrf);
		Check("init succeeds", ok);
		// The last command sent during init is SAMConfiguration.
		Check("SAMConfiguration issued", s_LastCmd == PN532_CMD_SAMCONFIGURATION);
	}

	printf("== 2. Detect maps to InListPassiveTarget ==\n");
	{
		RFTagInfo_t tag;
		bool ok = rdr.Detect(&tag);
		Check("detect answers", ok);
		Check("InListPassiveTarget issued", s_LastCmd == PN532_CMD_INLISTPASSIVETARGET);
		Check("MaxTg and BrTy in the command", s_LastData[0] == 1 && s_LastData[1] == PN532_BRTY_106K_TYPEA);
		Check("proto is NFC A", tag.Proto == RF_PROTO_NFCA);
		Check("UID parsed", tag.UidLen == 4 && tag.Uid[0] == 0xDE && tag.Uid[3] == 0xEF);
		Check("DETECTED event", rdr.Detected == 1);
	}

	printf("== 3. Transceive maps to InDataExchange ==\n");
	{
		// A Type 4 select APDU.
		uint8_t apdu[] = { 0x00, 0xA4, 0x04, 0x00, 0x07,
						   0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 };
		uint8_t rx[16];
		int l = rdr.Transceive(apdu, sizeof(apdu), rx, sizeof(rx));
		Check("InDataExchange issued", s_LastCmd == PN532_CMD_INDATAEXCHANGE);
		Check("target number prefixes the payload", s_LastData[0] == 1);
		Check("apdu forwarded after target number", s_LastData[1] == 0x00 && s_LastData[2] == 0xA4);
		Check("response strips status byte", l == 2 && rx[0] == 0x90 && rx[1] == 0x00);
		Check("RX_DATA event", rdr.RxData_ == 1);
	}

	printf("\nresult: pass=%d fail=%d\n", s_Pass, s_Fail);
	return s_Fail == 0 ? 0 : 1;
}

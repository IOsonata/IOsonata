/**-------------------------------------------------------------------------
@file	rftag_controller_exerciser.cpp

@brief	Host exerciser for the RFTagController facet

Drives RFTagController against a mock reader adapter over a DeviceIntrf. The
mock records the last command and answers Detect with a canned tag and reads
with canned data. Checks the detect command layout and the tag description,
the select payload, the memory command layout for read and write, the raw
transceive send only and send with response paths, and the per instance
EvtHandler dispatch. The mock is wrapped in a small DeviceIntrf so the facet
reaches it the same way a real transport does.

Link rule: build with rftag_controller.cpp, device.cpp and device_intrf.cpp.

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
#include "rftag/rftag_controller.h"

static int s_Pass = 0;
static int s_Fail = 0;

// Mock transaction capture.
static uint32_t s_LastDevAddr = 0;
static uint8_t s_LastCmd[64];
static int s_LastCmdLen = 0;
static uint8_t s_LastTx[64];
static int s_LastTxLen = 0;

// Canned tag answered by a detect read.
static RFTagInfo_t s_CannedTag;
static bool s_ProvideTag = false;

static void Check(const char *pName, bool bOk)
{
	if (bOk)
	{
		s_Pass++;
		printf("  [PASS] %s\n", pName);
	}
	else
	{
		s_Fail++;
		printf("  [FAIL] %s\n", pName);
	}
}

static bool MockStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	s_LastDevAddr = DevAddr;
	return true;
}

// DeviceIntrfWrite combines the command phase and the payload into a single
// TxData buffer, command bytes first. Capture the whole buffer here, the tests
// split it by the known command length.
static int s_CmdSplit = 0;

static int MockTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	(void)pDev;
	int cl = DataLen < (int)sizeof(s_LastCmd) ? DataLen : (int)sizeof(s_LastCmd);
	memcpy(s_LastCmd, pData, cl);
	s_LastCmdLen = DataLen;

	// The payload follows the command phase in the same buffer.
	if (s_CmdSplit > 0 && DataLen > s_CmdSplit)
	{
		int pl = DataLen - s_CmdSplit;
		if (pl > (int)sizeof(s_LastTx)) { pl = (int)sizeof(s_LastTx); }
		memcpy(s_LastTx, pData + s_CmdSplit, pl);
		s_LastTxLen = DataLen - s_CmdSplit;
	}
	return DataLen;
}

static void MockStopTx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static bool MockStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	s_LastDevAddr = DevAddr;
	return true;
}

static int MockRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	(void)pDev;
	if (s_ProvideTag && BuffLen == (int)sizeof(RFTagInfo_t))
	{
		memcpy(pBuff, &s_CannedTag, sizeof(RFTagInfo_t));
		return BuffLen;
	}

	// Canned memory read pattern.
	for (int i = 0; i < BuffLen; i++)
	{
		pBuff[i] = (uint8_t)(0xA0 + i);
	}
	return BuffLen;
}

static void MockStopRx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

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

// The command phase of a Read arrives through the TxData path as well. Reset
// the capture before each operation so the layout checks read the right bytes.
static void ResetCapture(void)
{
	s_LastCmdLen = 0;
	s_LastTxLen = 0;
	s_CmdSplit = 0;
	memset(s_LastCmd, 0, sizeof(s_LastCmd));
	memset(s_LastTx, 0, sizeof(s_LastTx));
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

// A controller that counts the events it observes.
class TestController : public RFTagController {
public:
	TestController() : Detected(0), RxData_(0), TxDone(0) {}

	virtual void EvtHandler(RFTAGCTRL_EVT Evt, uint32_t P0, uint32_t P1)
	{
		(void)P0; (void)P1;
		if (Evt == RFTAGCTRL_EVT_TAG_DETECTED) { Detected++; }
		else if (Evt == RFTAGCTRL_EVT_RX_DATA) { RxData_++; }
		else if (Evt == RFTAGCTRL_EVT_TX_DONE) { TxDone++; }
	}

	int Detected;
	int RxData_;
	int TxDone;
};

int main(void)
{
	MockIntrf intrf;
	TestController ctrl;

	RFTagControllerCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.DevAddr = 0x24;
	cfg.ProtoMask = RF_PROTO_ISO15693 | RF_PROTO_T4T;
	cfg.Bitrate = 0;
	cfg.TimeoutMs = 100;

	printf("== 1. Init ==\n");
	{
		bool ok = ctrl.Init(cfg, &intrf);
		Check("init succeeds", ok);
		Check("dev addr recorded", ctrl.DevAddr() == 0x24);
		Check("proto mask recorded", ctrl.ProtoMask() == (RF_PROTO_ISO15693 | RF_PROTO_T4T));
	}

	printf("== 2. Detect command and tag description ==\n");
	{
		ResetCapture();
		memset(&s_CannedTag, 0, sizeof(s_CannedTag));
		s_CannedTag.Proto = RF_PROTO_ISO15693;
		s_CannedTag.UidLen = 8;
		for (int i = 0; i < 8; i++) { s_CannedTag.Uid[i] = (uint8_t)(0xE0 + i); }
		s_CannedTag.Size = 256;
		s_CannedTag.BlockSize = 4;
		s_ProvideTag = true;

		RFTagInfo_t tag;
		bool ok = ctrl.Detect(&tag);
		s_ProvideTag = false;

		Check("detect answers", ok);
		// Command layout: DETECT then 4 byte proto mask little endian.
		bool layout = s_LastCmd[0] == RFTAGCTRL_CMD_DETECT
			&& s_LastCmd[1] == (uint8_t)cfg.ProtoMask
			&& s_LastCmd[2] == (uint8_t)(cfg.ProtoMask >> 8)
			&& s_LastCmd[3] == (uint8_t)(cfg.ProtoMask >> 16)
			&& s_LastCmd[4] == (uint8_t)(cfg.ProtoMask >> 24);
		Check("detect command layout", layout);
		Check("tag proto returned", tag.Proto == RF_PROTO_ISO15693);
		Check("tag UID returned", tag.UidLen == 8 && tag.Uid[0] == 0xE0 && tag.Uid[7] == 0xE7);
		Check("DETECTED event dispatched once", ctrl.Detected == 1);
		Check("detect used the config dev addr", s_LastDevAddr == 0x24);
	}

	printf("== 3. Detect no tag ==\n");
	{
		ResetCapture();
		s_ProvideTag = true;
		memset(&s_CannedTag, 0, sizeof(s_CannedTag));	// Proto NONE
		RFTagInfo_t tag;
		int before = ctrl.Detected;
		bool ok = ctrl.Detect(&tag);
		s_ProvideTag = false;
		Check("no tag reported", ok == false);
		Check("no DETECTED event", ctrl.Detected == before);
	}

	printf("== 4. Select ==\n");
	{
		ResetCapture();
		RFTagInfo_t tag;
		memset(&tag, 0, sizeof(tag));
		tag.Proto = RF_PROTO_T4T;
		tag.UidLen = 7;
		for (int i = 0; i < 7; i++) { tag.Uid[i] = (uint8_t)(i + 1); }

		s_CmdSplit = 1;			// select command is one byte, tag info follows
		bool ok = ctrl.Select(&tag);
		Check("select succeeds", ok);
		Check("select command byte", s_LastCmd[0] == RFTAGCTRL_CMD_SELECT);
		// Select writes the tag info as payload.
		bool payload = s_LastTxLen == (int)sizeof(RFTagInfo_t);
		Check("select sends tag info payload", payload);
	}

	printf("== 5. Tag read memory command ==\n");
	{
		ResetCapture();
		RFTagInfo_t tag;
		memset(&tag, 0, sizeof(tag));
		tag.Proto = RF_PROTO_ISO15693;
		tag.UidLen = 8;
		for (int i = 0; i < 8; i++) { tag.Uid[i] = (uint8_t)(0xE0 + i); }

		uint8_t buff[16];
		int l = ctrl.TagRead(&tag, 0x0104, buff, sizeof(buff));
		Check("tag read returns data", l == (int)sizeof(buff));
		Check("read data is the canned pattern", buff[0] == 0xA0 && buff[1] == 0xA1);

		// The memory command is the read command phase.
		RFTagControllerMemCmd_t *pc = (RFTagControllerMemCmd_t *)s_LastCmd;
		bool layout = s_LastCmdLen == (int)sizeof(RFTagControllerMemCmd_t)
			&& pc->Cmd == RFTAGCTRL_CMD_TAG_READ
			&& pc->Len == sizeof(buff)
			&& pc->Proto == RF_PROTO_ISO15693
			&& pc->Addr == 0x0104
			&& pc->UidLen == 8
			&& pc->Uid[0] == 0xE0 && pc->Uid[7] == 0xE7;
		Check("read memory command layout", layout);
		Check("RX_DATA event on read", ctrl.RxData_ >= 1);
	}

	printf("== 6. Tag write memory command ==\n");
	{
		ResetCapture();
		RFTagInfo_t tag;
		memset(&tag, 0, sizeof(tag));
		tag.Proto = RF_PROTO_T4T;
		tag.UidLen = 4;
		for (int i = 0; i < 4; i++) { tag.Uid[i] = (uint8_t)(0x10 + i); }

		uint8_t data[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
		int before = ctrl.TxDone;
		s_CmdSplit = (int)sizeof(RFTagControllerMemCmd_t);	// mem command then payload
		int l = ctrl.TagWrite(&tag, 0x0200, data, sizeof(data));
		Check("tag write returns count", l == (int)sizeof(data));

		// Write combines the memory command and the payload in one buffer,
		// so the captured length is the command plus the data.
		RFTagControllerMemCmd_t *pc = (RFTagControllerMemCmd_t *)s_LastCmd;
		bool layout = s_LastCmdLen == (int)(sizeof(RFTagControllerMemCmd_t) + sizeof(data))
			&& pc->Cmd == RFTAGCTRL_CMD_TAG_WRITE
			&& pc->Len == sizeof(data)
			&& pc->Proto == RF_PROTO_T4T
			&& pc->Addr == 0x0200
			&& pc->UidLen == 4;
		Check("write memory command layout", layout);
		Check("write payload is the data", s_LastTx[0] == 0xDE && s_LastTx[3] == 0xEF);
		Check("TX_DONE event on write", ctrl.TxDone == before + 1);
	}

	printf("== 7. Transceive send only ==\n");
	{
		ResetCapture();
		uint8_t tx[3] = { 0x01, 0x02, 0x03 };
		int l = ctrl.Transceive(tx, sizeof(tx), nullptr, 0);
		Check("send only returns sent count", l == (int)sizeof(tx));
	}

	printf("== 8. Transceive send with response ==\n");
	{
		ResetCapture();
		int before = ctrl.RxData_;
		uint8_t tx[2] = { 0xAA, 0xBB };
		uint8_t rx[8];
		int l = ctrl.Transceive(tx, sizeof(tx), rx, sizeof(rx));
		Check("send with response returns rx count", l == (int)sizeof(rx));
		Check("response is the canned pattern", rx[0] == 0xA0 && rx[1] == 0xA1);
		Check("RX_DATA event on transceive", ctrl.RxData_ == before + 1);
	}

	printf("\nresult: pass=%d fail=%d\n", s_Pass, s_Fail);
	return s_Fail == 0 ? 0 : 1;
}

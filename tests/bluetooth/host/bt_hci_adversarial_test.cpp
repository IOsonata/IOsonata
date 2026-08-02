// Adversarial HCI host tests.
//
// Variable-length HCI events are controller input. The count field itself must
// be in bounds before it is read, and a dropped extended advertising fragment
// must never be delivered later as a complete report.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_smp.h"
#include "syslog.h"

namespace {

bttest::Context s_Test("HCI adversarial host tests");

int s_CompletedCount = 0;
int s_ScanCount = 0;
size_t s_LastScanLen = 0;
uint8_t s_LastScanData[64] = {};

void Completed(uint16_t, uint16_t)
{
	s_CompletedCount++;
}

bool ScanReport(int8_t, uint8_t, uint8_t[6], size_t Len, uint8_t *pData)
{
	s_ScanCount++;
	s_LastScanLen = Len;
	if (pData != nullptr && Len <= sizeof(s_LastScanData))
	{
		std::memcpy(s_LastScanData, pData, Len);
	}
	return true;
}

void TestCompletedEventNeedsCountByte()
{
	BtHciDevice_t dev = {};
	dev.SendCompleted = Completed;
	s_CompletedCount = 0;

	alignas(4) std::array<uint8_t, sizeof(BtHciEvtPacketHdr_t)> raw = {};
	BtHciEvtPacket_t *pEvt = (BtHciEvtPacket_t *)raw.data();
	pEvt->Hdr.Evt = BT_HCI_EVT_NB_COMPLETED_PACKET;
	pEvt->Hdr.Len = 0;

	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_CompletedCount == 0);
}

void TestLegacyAdvEventNeedsCountByte()
{
	BtHciDevice_t dev = {};
	dev.ScanReport = ScanReport;
	s_ScanCount = 0;

	alignas(4) std::array<uint8_t,
			sizeof(BtHciEvtPacketHdr_t) + 1> raw = {};
	BtHciEvtPacket_t *pEvt = (BtHciEvtPacket_t *)raw.data();
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = 1;
	pEvt->Data[0] = BT_HCI_EVT_LE_ADV_REPORT;

	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_ScanCount == 0);
}

void TestExtendedAdvEventNeedsCountByte()
{
	BtHciDevice_t dev = {};
	dev.ScanReport = ScanReport;
	s_ScanCount = 0;

	alignas(4) std::array<uint8_t,
			sizeof(BtHciEvtPacketHdr_t) + 1> raw = {};
	BtHciEvtPacket_t *pEvt = (BtHciEvtPacket_t *)raw.data();
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = 1;
	pEvt->Data[0] = BT_HCI_EVT_LE_EXT_ADV_REPORT;

	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_ScanCount == 0);
}

void FeedExtAdv(BtHciDevice_t *pDev, const uint8_t Addr[6],
				uint8_t Sid, uint8_t DataStatus,
				const uint8_t *pData, uint8_t DataLen)
{
	alignas(4) std::array<uint8_t,
			sizeof(BtHciEvtPacketHdr_t) + 1 + 1 + 24 + 32> raw = {};
	BtHciEvtPacket_t *pEvt = (BtHciEvtPacket_t *)raw.data();
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = (uint8_t)(1 + 1 + 24 + DataLen);

	BtHciLeEvtPacket_t *pLe = (BtHciLeEvtPacket_t *)pEvt->Data;
	pLe->Evt = BT_HCI_EVT_LE_EXT_ADV_REPORT;
	uint8_t *pBody = (uint8_t *)pLe->Data;
	pBody[0] = 1;

	BtExtAdvReport_t *pReport = (BtExtAdvReport_t *)(pBody + 1);
	pReport->Type = (uint16_t)(DataStatus << 5);
	pReport->AddrType = 0;
	std::memcpy(pReport->Addr, Addr, 6);
	pReport->AdvSid = Sid;
	pReport->Rssi = -30;
	pReport->DataLen = DataLen;
	if (DataLen > 0)
	{
		std::memcpy(pReport->Data, pData, DataLen);
	}

	BtHciProcessEvent(pDev, pEvt);
}

void TestDroppedReassemblyCannotBecomeComplete()
{
	BtHciDevice_t dev = {};
	dev.ScanReport = ScanReport;
	s_ScanCount = 0;
	s_LastScanLen = 0;

	const uint8_t addrA[6] = { 1, 0, 0, 0, 0, 0 };
	const uint8_t addrB[6] = { 2, 0, 0, 0, 0, 0 };
	const uint8_t addrC[6] = { 3, 0, 0, 0, 0, 0 };
	const uint8_t first = 0xA1;
	const uint8_t last = 0xC2;

	FeedExtAdv(&dev, addrA, 1, 1, &first, 1);
	FeedExtAdv(&dev, addrB, 2, 1, &first, 1);
	BT_CHECK(s_Test, s_ScanCount == 0);

	FeedExtAdv(&dev, addrC, 3, 1, &first, 1);
	FeedExtAdv(&dev, addrC, 3, 0, &last, 1);
	BT_CHECK(s_Test, s_ScanCount == 0);
}

} // namespace

extern "C" {

uint32_t BtAttProcessReq(uint16_t, BtAttReqRsp_t * const, int,
						 BtAttReqRsp_t * const)
{
	return 0;
}

void BtAttProcessRsp(uint16_t, BtAttReqRsp_t * const, int)
{
}

uint32_t BtL2CapProcessSignal(BtHciDevice_t * const, uint16_t,
							  BtL2CapPdu_t const * const, uint16_t,
							  BtL2CapPdu_t * const)
{
	return 0;
}

void BtProcessSmpData(BtHciDevice_t * const, uint16_t,
					  BtL2CapSmp_t * const, size_t)
{
}

void BtSmpProcessLtkRequest(BtHciDevice_t * const, uint16_t, uint64_t, uint16_t)
{
}

void BtSmpLocalPubKeyReady(BtHciDevice_t * const, uint8_t,
						   const uint8_t *, const uint8_t *)
{
}

void BtSmpDhKeyReady(BtHciDevice_t * const, uint8_t, const uint8_t *)
{
}

void BtSmpEncryptionChanged(BtHciDevice_t * const, uint16_t, uint8_t, uint8_t)
{
}

void BtSmpDisconnected(uint16_t)
{
}

SysLog_t *SysLogGet(void)
{
	static SysLog_t log = {};
	return &log;
}

int SysLogPrintf(SysLog_t * const, const char *, ...)
{
	return 0;
}

} // extern "C"

int main()
{
	s_Test.Run("completed-packet count boundary",
			   TestCompletedEventNeedsCountByte);
	s_Test.Run("legacy advertising count boundary",
			   TestLegacyAdvEventNeedsCountByte);
	s_Test.Run("extended advertising count boundary",
			   TestExtendedAdvEventNeedsCountByte);
	s_Test.Run("dropped extended advertising reassembly",
			   TestDroppedReassemblyCannotBecomeComplete);
	return s_Test.Finish();
}

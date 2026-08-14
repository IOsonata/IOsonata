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
#include "bluetooth/bt_adv.h"
#include "syslog.h"

namespace {

bttest::Context s_Test("HCI adversarial host tests");

int s_CompletedCount = 0;
int s_ScanCount = 0;
int s_ScanTimeoutCount = 0;
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

	// The storage is a whole BtHciEvtPacket_t: casting a smaller buffer to it
	// and touching a member is undefined on its own, which is a defect in the
	// test rather than in the code under test. Hdr.Len is what says the event
	// is truncated, and that is what the parser has to honour.
	alignas(4) std::array<uint8_t, sizeof(BtHciEvtPacket_t)> raw = {};
	BtHciEvtPacket_t *pEvt = (BtHciEvtPacket_t *)raw.data();
	pEvt->Hdr.Evt = BT_HCI_EVT_NB_COMPLETED_PACKET;
	pEvt->Hdr.Len = 0;
	pEvt->Data[0] = 0xFF;			// a count the parser must not read

	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_CompletedCount == 0);
}

void TestLegacyAdvEventNeedsCountByte()
{
	BtHciDevice_t dev = {};
	dev.ScanReport = ScanReport;
	s_ScanCount = 0;

	// Room for the subevent byte and the count byte after it; Hdr.Len says
	// only the subevent byte is present, so the count byte is outside the
	// event even though it is inside the allocation.
	alignas(4) std::array<uint8_t,
			sizeof(BtHciEvtPacket_t) + 1> raw = {};
	BtHciEvtPacket_t *pEvt = (BtHciEvtPacket_t *)raw.data();
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = 1;
	pEvt->Data[0] = BT_HCI_EVT_LE_ADV_REPORT;
	pEvt->Data[1] = 0xFF;			// a count the parser must not read

	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_ScanCount == 0);
}

void TestExtendedAdvEventNeedsCountByte()
{
	BtHciDevice_t dev = {};
	dev.ScanReport = ScanReport;
	s_ScanCount = 0;

	alignas(4) std::array<uint8_t,
			sizeof(BtHciEvtPacket_t) + 1> raw = {};
	BtHciEvtPacket_t *pEvt = (BtHciEvtPacket_t *)raw.data();
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = 1;
	pEvt->Data[0] = BT_HCI_EVT_LE_EXT_ADV_REPORT;
	pEvt->Data[1] = 0xFF;			// a count the parser must not read

	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_ScanCount == 0);
}


// --- Periodic advertising report layouts ---
//
// v1 and v2 of the periodic advertising report are different layouts, not one
// with a tail: v2 puts the periodic event counter and the subevent between
// CteType and DataStatus. Parsing a v2 report with the v1 fixed size reads
// DataStatus out of the low half of the event counter and the length out of
// its high half, which is a defect this file exists to make impossible.

int s_PeriodicFragCount = 0;
uint16_t s_PeriodicFragHdl = 0;
uint8_t s_PeriodicFragStatus = 0xFF;
size_t s_PeriodicFragLen = 0;
uint8_t s_PeriodicFragData[64] = {};
int8_t s_PeriodicFragRssi = 0;

int s_SubeventRptCount = 0;
uint16_t s_SubeventRptEvent = 0;
uint8_t s_SubeventRptSubevent = 0xFF;
uint8_t s_SubeventRptStatus = 0xFF;
size_t s_SubeventRptLen = 0;
uint8_t s_SubeventRptData[64] = {};

int s_EstCount = 0;
uint16_t s_EstHdl = 0;
int s_LostCount = 0;
int s_DataReqCount = 0;
uint8_t s_DataReqStart = 0xFF;
int s_RspRptCount = 0;
uint8_t s_RspRptNum = 0xFF;
size_t s_RspRptLen = 0;
int s_AdvTermCount = 0;
uint8_t s_AdvTermStatus = 0xFF;
uint8_t s_AdvTermHdl = 0xFF;
uint16_t s_AdvTermConnHdl = 0xFFFF;

void ResetPeriodic()
{
	s_PeriodicFragCount = 0;
	s_PeriodicFragHdl = 0;
	s_PeriodicFragStatus = 0xFF;
	s_PeriodicFragLen = 0;
	s_PeriodicFragRssi = 0;
	s_SubeventRptCount = 0;
	s_SubeventRptEvent = 0;
	s_SubeventRptSubevent = 0xFF;
	s_SubeventRptStatus = 0xFF;
	s_SubeventRptLen = 0;
	s_EstCount = 0;
	s_EstHdl = 0;
	s_LostCount = 0;
	s_DataReqCount = 0;
	s_DataReqStart = 0xFF;
	s_RspRptCount = 0;
	s_RspRptNum = 0xFF;
	s_RspRptLen = 0;
	s_AdvTermCount = 0;
	s_AdvTermStatus = 0xFF;
	s_AdvTermHdl = 0xFF;
	s_AdvTermConnHdl = 0xFFFF;
}

// Build an LE meta event from a subevent code and its parameter octets.
BtHciEvtPacket_t *MakeLeEvent(uint8_t *pRaw, size_t RawLen, uint8_t Subevent,
	const uint8_t *pParams, uint8_t ParamLen)
{
	std::memset(pRaw, 0, RawLen);
	BtHciEvtPacket_t *pEvt = (BtHciEvtPacket_t *)pRaw;
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = (uint8_t)(1 + ParamLen);
	pEvt->Data[0] = Subevent;
	if (ParamLen > 0)
	{
		std::memcpy(&pEvt->Data[1], pParams, ParamLen);
	}
	return pEvt;
}

void TestPeriodicReportV1Layout()
{
	BtHciDevice_t dev = {};
	ResetPeriodic();

	// SyncHdl 0x0007, TxPower -10, RSSI -40, CteType 0, DataStatus complete,
	// length 3, then the data.
	const uint8_t params[] = {
		0x07, 0x00, 0xF6, 0xD8, 0x00, 0x00, 0x03, 0xAA, 0xBB, 0xCC,
	};
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacket_t) + sizeof(params)] = {};
	BtHciEvtPacket_t *pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V1, params, sizeof(params));

	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_PeriodicFragCount == 1);
	BT_CHECK(s_Test, s_PeriodicFragHdl == 0x0007);
	BT_CHECK(s_Test, s_PeriodicFragRssi == -40);
	BT_CHECK(s_Test, s_PeriodicFragStatus == 0x00);
	BT_CHECK(s_Test, s_PeriodicFragLen == 3);
	BT_CHECK(s_Test, s_PeriodicFragData[0] == 0xAA);
	BT_CHECK(s_Test, s_PeriodicFragData[2] == 0xCC);

	// v1 must not reach the PAwR path.
	BT_CHECK(s_Test, s_SubeventRptCount == 0);
}

void TestPeriodicReportV2Layout()
{
	BtHciDevice_t dev = {};
	ResetPeriodic();

	// SyncHdl 0x0007, TxPower -10, RSSI -40, CteType 0, event counter 0x1234,
	// subevent 5, DataStatus complete, length 3, then the data. Read as v1
	// this would take DataStatus 0x34 and length 0x12.
	const uint8_t params[] = {
		0x07, 0x00, 0xF6, 0xD8, 0x00, 0x34, 0x12, 0x05, 0x00, 0x03,
		0xAA, 0xBB, 0xCC,
	};
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacket_t) + sizeof(params)] = {};
	BtHciEvtPacket_t *pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V2, params, sizeof(params));

	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_SubeventRptCount == 1);
	BT_CHECK(s_Test, s_SubeventRptEvent == 0x1234);
	BT_CHECK(s_Test, s_SubeventRptSubevent == 5);
	BT_CHECK(s_Test, s_SubeventRptStatus == 0x00);
	BT_CHECK(s_Test, s_SubeventRptLen == 3);
	BT_CHECK(s_Test, s_SubeventRptData[0] == 0xAA);
	BT_CHECK(s_Test, s_SubeventRptData[2] == 0xCC);

	// v2 must not reach the plain path.
	BT_CHECK(s_Test, s_PeriodicFragCount == 0);
}

// A length the event does not contain, and a fixed part cut short by Hdr.Len.
void TestPeriodicReportBounds()
{
	BtHciDevice_t dev = {};

	ResetPeriodic();
	const uint8_t shortV2[] = { 0x07, 0x00, 0xF6, 0xD8, 0x00, 0x34, 0x12 };
	alignas(4) uint8_t raw1[sizeof(BtHciEvtPacket_t) + sizeof(shortV2)] = {};
	BtHciEvtPacket_t *pEvt = MakeLeEvent(raw1, sizeof(raw1),
		BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V2, shortV2, sizeof(shortV2));
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_SubeventRptCount == 0);

	ResetPeriodic();
	// Announces 0x40 octets and holds one.
	const uint8_t overrunV2[] = {
		0x07, 0x00, 0xF6, 0xD8, 0x00, 0x34, 0x12, 0x05, 0x00, 0x40, 0xAA,
	};
	alignas(4) uint8_t raw2[sizeof(BtHciEvtPacket_t) + sizeof(overrunV2)] = {};
	pEvt = MakeLeEvent(raw2, sizeof(raw2),
		BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V2, overrunV2, sizeof(overrunV2));
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_SubeventRptCount == 0);

	ResetPeriodic();
	const uint8_t overrunV1[] = { 0x07, 0x00, 0xF6, 0xD8, 0x00, 0x00, 0x40, 0xAA };
	alignas(4) uint8_t raw3[sizeof(BtHciEvtPacket_t) + sizeof(overrunV1)] = {};
	pEvt = MakeLeEvent(raw3, sizeof(raw3),
		BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V1, overrunV1, sizeof(overrunV1));
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_PeriodicFragCount == 0);
}

void TestPeriodicSyncEvents()
{
	BtHciDevice_t dev = {};
	ResetPeriodic();

	// Status 0, SyncHdl 0x0009, sid, address type, address, phy, interval,
	// clock accuracy.
	const uint8_t est[] = {
		0x00, 0x09, 0x00, 0x03, 0x01, 1, 2, 3, 4, 5, 6, 0x01, 0x80, 0x00, 0x00,
	};
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacket_t) + sizeof(est)] = {};
	BtHciEvtPacket_t *pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_ESTABLISHED_V1, est, sizeof(est));
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_EstCount == 1);
	BT_CHECK(s_Test, s_EstHdl == 0x0009);

	// A truncated establish is refused.
	ResetPeriodic();
	pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_ESTABLISHED_V1, est, 4);
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_EstCount == 0);

	const uint8_t lost[] = { 0x09, 0x00 };
	alignas(4) uint8_t raw2[sizeof(BtHciEvtPacket_t) + sizeof(lost)] = {};
	pEvt = MakeLeEvent(raw2, sizeof(raw2),
		BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_LOST, lost, sizeof(lost));
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_LostCount == 1);
}

void TestPawrEvents()
{
	BtHciDevice_t dev = {};
	ResetPeriodic();

	const uint8_t req[] = { 0x01, 0x03, 0x02 };
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacket_t) + sizeof(req)] = {};
	BtHciEvtPacket_t *pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_PERIODIC_ADV_DATA_REQ, req, sizeof(req));
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_DataReqCount == 1);
	BT_CHECK(s_Test, s_DataReqStart == 3);

	// Two octets is not enough for the three the event has.
	ResetPeriodic();
	pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_PERIODIC_ADV_DATA_REQ, req, 2);
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_DataReqCount == 0);

	// Advertising handle, subevent, TX status, response count, then responses.
	ResetPeriodic();
	const uint8_t rsp[] = {
		0x01, 0x02, 0x00, 0x01,
		0x00, 0xD6, 0x00, 0x04, 0x00, 0x02, 0x11, 0x22,
	};
	alignas(4) uint8_t raw2[sizeof(BtHciEvtPacket_t) + sizeof(rsp)] = {};
	pEvt = MakeLeEvent(raw2, sizeof(raw2),
		BT_HCI_EVT_LE_PERIODIC_ADV_RESP_REPORT, rsp, sizeof(rsp));
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_RspRptCount == 1);
	BT_CHECK(s_Test, s_RspRptNum == 1);
	BT_CHECK(s_Test, s_RspRptLen == 8);
}

// Core Vol 4 Part E 7.7.65.18. The event has five octets and the host
// used to read none of them, so the parse is what has to be pinned here: the
// policy that acts on Status and the handle lives in bt_adv_hci and is
// covered by bt_adv_hci_test.
void TestAdvSetTerminatedLayout()
{
	BtHciDevice_t dev = {};
	ResetPeriodic();

	// Advertising Timeout on set 0, connection handle unused, no completed
	// extended advertising events.
	const uint8_t timeout[] = { 0x3C, 0x00, 0x00, 0x00, 0x00 };
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacket_t) + sizeof(timeout)] = {};
	BtHciEvtPacket_t *pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_ADV_SET_TERMINATED, timeout, sizeof(timeout));
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_AdvTermCount == 1);
	BT_CHECK(s_Test, s_AdvTermStatus == 0x3C);
	BT_CHECK(s_Test, s_AdvTermHdl == 0);
	BT_CHECK(s_Test, s_AdvTermConnHdl == 0);

	// A connection ended it: status 0, set 3, connection handle 0x0042.
	ResetPeriodic();
	const uint8_t conn[] = { 0x00, 0x03, 0x42, 0x00, 0x07 };
	pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_ADV_SET_TERMINATED, conn, sizeof(conn));
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_AdvTermCount == 1);
	BT_CHECK(s_Test, s_AdvTermStatus == 0x00);
	BT_CHECK(s_Test, s_AdvTermHdl == 3);
	BT_CHECK(s_Test, s_AdvTermConnHdl == 0x0042);

	// Four octets is not enough for the five the event has.
	ResetPeriodic();
	pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_ADV_SET_TERMINATED, timeout, 4);
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_AdvTermCount == 0);

	// And no parameters at all is not either.
	ResetPeriodic();
	pEvt = MakeLeEvent(raw, sizeof(raw),
		BT_HCI_EVT_LE_ADV_SET_TERMINATED, nullptr, 0);
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_AdvTermCount == 0);
}

void TestScanTimeoutEventDispatch()
{
	BtHciDevice_t dev = {};
	s_ScanTimeoutCount = 0;

	alignas(4) std::array<uint8_t, sizeof(BtHciEvtPacket_t)> raw = {};
	BtHciEvtPacket_t *pEvt = (BtHciEvtPacket_t *)raw.data();
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = 1;
	pEvt->Data[0] = BT_HCI_EVT_LE_SCAN_TIMEOUT;

	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_ScanTimeoutCount == 1);

	// A zero-length LE Meta event has no subevent byte and must not dispatch.
	pEvt->Hdr.Len = 0;
	BtHciProcessEvent(&dev, pEvt);
	BT_CHECK(s_Test, s_ScanTimeoutCount == 1);
}

void FeedExtAdv(BtHciDevice_t *pDev, const uint8_t Addr[6],
				uint8_t Sid, uint8_t DataStatus,
				const uint8_t *pData, uint8_t DataLen)
{
	alignas(4) std::array<uint8_t,
			sizeof(BtHciEvtPacket_t) + 1 + 1 + 24 + 32> raw = {};
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

// Strong overrides of the periodic advertising seams bt_hci_host declares weak.
void BtAdvPeriodicReportFragment(uint16_t SyncHdl, int8_t, int8_t Rssi,
	uint8_t DataStatus, size_t Len, const uint8_t *pData)
{
	s_PeriodicFragCount++;
	s_PeriodicFragHdl = SyncHdl;
	s_PeriodicFragRssi = Rssi;
	s_PeriodicFragStatus = DataStatus;
	s_PeriodicFragLen = Len;
	if (Len > 0 && pData != nullptr && Len <= sizeof(s_PeriodicFragData))
	{
		std::memcpy(s_PeriodicFragData, pData, Len);
	}
}

void BtAdvPawrSubeventReportEvt(uint16_t, uint16_t EventCounter,
	uint8_t Subevent, int8_t, int8_t, uint8_t DataStatus, size_t Len,
	const uint8_t *pData)
{
	s_SubeventRptCount++;
	s_SubeventRptEvent = EventCounter;
	s_SubeventRptSubevent = Subevent;
	s_SubeventRptStatus = DataStatus;
	s_SubeventRptLen = Len;
	if (Len > 0 && pData != nullptr && Len <= sizeof(s_SubeventRptData))
	{
		std::memcpy(s_SubeventRptData, pData, Len);
	}
}

void BtAdvPeriodicSyncEstablishedEvt(uint8_t, uint16_t SyncHdl, uint8_t,
	uint8_t, const uint8_t[6], uint8_t, uint16_t)
{
	s_EstCount++;
	s_EstHdl = SyncHdl;
}

void BtAdvPeriodicSyncLostEvt(uint16_t)
{
	s_LostCount++;
}

void BtAdvSetTerminatedEvt(uint8_t Status, uint8_t AdvHdl, uint16_t ConnHdl)
{
	s_AdvTermCount++;
	s_AdvTermStatus = Status;
	s_AdvTermHdl = AdvHdl;
	s_AdvTermConnHdl = ConnHdl;
}

void BtAdvPawrSubeventDataRequestEvt(uint8_t, uint8_t SubeventStart, uint8_t)
{
	s_DataReqCount++;
	s_DataReqStart = SubeventStart;
}

void BtAdvPawrResponseReportEvt(uint8_t, uint8_t, uint8_t,
	uint8_t NumResponses, const uint8_t *, size_t ResponsesLen)
{
	s_RspRptCount++;
	s_RspRptNum = NumResponses;
	s_RspRptLen = ResponsesLen;
}

// Strong override of the weak HCI host timeout hook.
void BtHciScanTimeout(BtHciDevice_t * const)
{
	s_ScanTimeoutCount++;
}

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
	s_Test.Run("advertising set terminated layout",
			   TestAdvSetTerminatedLayout);
	s_Test.Run("scan timeout event dispatch",
			   TestScanTimeoutEventDispatch);
	s_Test.Run("dropped extended advertising reassembly",
			   TestDroppedReassemblyCannotBecomeComplete);
	s_Test.Run("periodic advertising report v1 layout",
			   TestPeriodicReportV1Layout);
	s_Test.Run("periodic advertising report v2 layout",
			   TestPeriodicReportV2Layout);
	s_Test.Run("periodic advertising report bounds",
			   TestPeriodicReportBounds);
	s_Test.Run("periodic advertising sync events",
			   TestPeriodicSyncEvents);
	s_Test.Run("PAwR subevent request and response report",
			   TestPawrEvents);
	return s_Test.Finish();
}

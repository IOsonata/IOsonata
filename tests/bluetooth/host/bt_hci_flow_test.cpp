#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_smp.h"
#include "syslog.h"

namespace {

int s_Failures = 0;
int s_Checks = 0;

#define CHECK(expr) do { \
    ++s_Checks; \
    if (!(expr)) { \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        ++s_Failures; \
    } \
} while (0)

struct CapturedPacket {
    size_t Len;
    std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE> Data;
};

std::array<CapturedPacket, 12> s_AclPackets = {};
size_t s_AclPacketCount = 0;

std::array<uint8_t, BTHCICTLR_CMD_HDR_LEN + BTHCICTLR_CMD_PARAM_MAX> s_Command = {};
size_t s_CommandLen = 0;

struct SmpCapture {
    uint16_t ConnHdl;
    size_t Len;
    std::array<uint8_t, 64> Data;
};

std::array<SmpCapture, 8> s_SmpPackets = {};
size_t s_SmpPacketCount = 0;

uint16_t s_CompletedHdl = 0;
uint16_t s_CompletedCount = 0;
int s_CompletedCallbacks = 0;
int s_ScanReports = 0;
int s_SmpDisconnectCount = 0;
uint16_t s_SmpDisconnectHdl = 0;

void ResetAclCapture()
{
    s_AclPacketCount = 0;
    for (auto &pkt : s_AclPackets)
    {
        pkt.Len = 0;
        pkt.Data.fill(0);
    }
}

void ResetSmpCapture()
{
    s_SmpPacketCount = 0;
    for (auto &pkt : s_SmpPackets)
    {
        pkt.ConnHdl = 0;
        pkt.Len = 0;
        pkt.Data.fill(0);
    }
}

uint32_t CaptureAcl(void *pData, uint32_t Len)
{
    if (s_AclPacketCount >= s_AclPackets.size() ||
        Len > s_AclPackets[s_AclPacketCount].Data.size())
    {
        return 0;
    }

    CapturedPacket &pkt = s_AclPackets[s_AclPacketCount++];
    pkt.Len = Len;
    std::memcpy(pkt.Data.data(), pData, Len);
    return Len;
}

size_t CaptureCommand(BtHciCtlrDev_t * const, void * const pData, size_t Len)
{
    if (Len > s_Command.size())
    {
        return 0;
    }

    s_CommandLen = Len;
    std::memcpy(s_Command.data(), pData, Len);
    return Len;
}

void CaptureCompleted(uint16_t ConnHdl, uint16_t Count)
{
    s_CompletedHdl = ConnHdl;
    s_CompletedCount = Count;
    ++s_CompletedCallbacks;
}

bool CaptureScanReport(int8_t, uint8_t, uint8_t[6], size_t, uint8_t *)
{
    ++s_ScanReports;
    return true;
}

void FeedAclFragment(BtHciDevice_t *pDev, uint16_t ConnHdl, uint8_t PbFlag,
                     const uint8_t *pData, uint16_t Len)
{
    alignas(4) std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE> raw = {};
    BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw.data());

    pAcl->Hdr.ConnHdl = ConnHdl;
    pAcl->Hdr.PBFlag = PbFlag;
    pAcl->Hdr.BCFlag = BT_HCI_BCFLAG_POINT_TO_POINT;
    pAcl->Hdr.Len = Len;
    std::memcpy(pAcl->Data, pData, Len);

    BtHciProcessData(pDev, pAcl);
}

size_t BuildSmpPdu(uint8_t *pOut, uint8_t Code, const uint8_t *pData, size_t Len)
{
    BtL2CapPdu_t *pPdu = reinterpret_cast<BtL2CapPdu_t *>(pOut);
    pPdu->Hdr.Len = static_cast<uint16_t>(Len + 1U);
    pPdu->Hdr.Cid = BT_L2CAP_CID_SEC_MNGR;
    pPdu->Smp.Code = Code;
    if (Len > 0)
    {
        std::memcpy(pPdu->Smp.Data, pData, Len);
    }
    return sizeof(BtL2CapHdr_t) + Len + 1U;
}

void TestCommandFraming()
{
    BtHciCtlrDev_t ctlr = {};
    ctlr.SendCommand = CaptureCommand;
    s_CommandLen = 0;
    s_Command.fill(0);

    const uint8_t param[] = { 0x11, 0x22, 0x33 };
    const uint16_t opcode = 0x2042;

    CHECK(BtHciCtlrSendCommand(&ctlr, opcode, param, sizeof(param)) ==
          BTHCICTLR_CMD_HDR_LEN + sizeof(param));
    CHECK(s_CommandLen == BTHCICTLR_CMD_HDR_LEN + sizeof(param));
    CHECK(s_Command[0] == 0x42);
    CHECK(s_Command[1] == 0x20);
    CHECK(s_Command[2] == sizeof(param));
    CHECK(std::memcmp(&s_Command[3], param, sizeof(param)) == 0);

    CHECK(BtHciCtlrSendCommand(&ctlr, opcode, nullptr, 1) == 0);
}

void TestAclFragmentationAndCredits()
{
    BtHciDevice_t dev = {};
    dev.SendData = CaptureAcl;
    ResetAclCapture();

    alignas(4) std::array<uint8_t, 64> raw = {};
    BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw.data());
    pAcl->Hdr.ConnHdl = 0x123;
    pAcl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
    pAcl->Hdr.BCFlag = BT_HCI_BCFLAG_POINT_TO_POINT;
    pAcl->Hdr.Len = 25;
    for (uint16_t i = 0; i < pAcl->Hdr.Len; ++i)
    {
        pAcl->Data[i] = static_cast<uint8_t>(i);
    }

    BtHciSetLeAclBuffer(&dev, 10, 3);
    CHECK(dev.AclMaxLen == 10);
    CHECK(dev.AclCreditMax == 3);
    CHECK(dev.AclCredit == 3);
    CHECK(dev.CmdCredit == 1);

    CHECK(BtHciSendAcl(&dev, pAcl) == pAcl->Hdr.Len + sizeof(pAcl->Hdr));
    CHECK(s_AclPacketCount == 3);
    CHECK(dev.AclCredit == 0);

    const uint16_t expectedLen[] = { 10, 10, 5 };
    uint16_t offset = 0;
    for (size_t i = 0; i < s_AclPacketCount; ++i)
    {
        const BtHciACLDataPacket_t *p =
            reinterpret_cast<const BtHciACLDataPacket_t *>(s_AclPackets[i].Data.data());
        CHECK(p->Hdr.ConnHdl == 0x123);
        CHECK(p->Hdr.Len == expectedLen[i]);
        CHECK(p->Hdr.PBFlag == (i == 0 ? BT_HCI_PBFLAG_START_NONFLUSHABLE
                                       : BT_HCI_PBFLAG_CONTINUING_FRAGMENT));
        CHECK(s_AclPackets[i].Len == expectedLen[i] + sizeof(p->Hdr));
        CHECK(std::memcmp(p->Data, &pAcl->Data[offset], expectedLen[i]) == 0);
        offset = static_cast<uint16_t>(offset + expectedLen[i]);
    }

    CHECK(BtHciSendAcl(&dev, pAcl) == 0);
    CHECK(s_AclPacketCount == 3);
}

void TestAclCompletionCredits()
{
    BtHciDevice_t dev = {};
    dev.AclCreditMax = 3;
    dev.AclCredit = 0;
    dev.SendCompleted = CaptureCompleted;
    s_CompletedCallbacks = 0;

    alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 5] = {};
    BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
    pEvt->Hdr.Evt = BT_HCI_EVT_NB_COMPLETED_PACKET;
    pEvt->Hdr.Len = 5;
    BtHciEvtNbCompletedPkt_t *p =
        reinterpret_cast<BtHciEvtNbCompletedPkt_t *>(pEvt->Data);
    p->NbHdl = 1;
    p->Completed[0].Hdl = 0x123;
    p->Completed[0].NbPkt = 2;

    BtHciProcessEvent(&dev, pEvt);
    CHECK(dev.AclCredit == 2);
    CHECK(s_CompletedCallbacks == 1);
    CHECK(s_CompletedHdl == 0x123);
    CHECK(s_CompletedCount == 2);

    p->Completed[0].NbPkt = 9;
    BtHciProcessEvent(&dev, pEvt);
    CHECK(dev.AclCredit == 3);
}

// A Number Of Completed Packets event whose NbHdl count exceeds what the event
// length can hold must be bounded to the payload; the buffer is sized to exactly
// one entry so an unbounded walk would run off the end (caught by ASan).
void TestCompletedPacketsBounded()
{
    BtHciDevice_t dev = {};
    dev.AclCreditMax = 20;
    dev.AclCredit = 0;
    dev.SendCompleted = CaptureCompleted;
    s_CompletedCallbacks = 0;

    // Payload holds NbHdl(1) + exactly one Completed entry (4) = 5 bytes.
    alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 5] = {};
    BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
    pEvt->Hdr.Evt = BT_HCI_EVT_NB_COMPLETED_PACKET;
    pEvt->Hdr.Len = 5;
    BtHciEvtNbCompletedPkt_t *p =
        reinterpret_cast<BtHciEvtNbCompletedPkt_t *>(pEvt->Data);
    p->NbHdl = 10;					// lies: only one entry actually present
    p->Completed[0].Hdl = 0x321;
    p->Completed[0].NbPkt = 4;

    BtHciProcessEvent(&dev, pEvt);

    // Only the single in-bounds entry is processed.
    CHECK(dev.AclCredit == 4);
    CHECK(s_CompletedCallbacks == 1);
    CHECK(s_CompletedHdl == 0x321);
}

void TestCommandCreditsAndCompletion()
{
    BtHciDevice_t dev = {};
    uint8_t ret[2] = {};
    const uint16_t opcode = 0x201C;

    dev.CmdCredit = 0;
    dev.CmdOpCode = opcode;
    dev.pCmdRet = ret;
    dev.CmdRetLen = sizeof(ret);
    dev.CmdDone = false;

    alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 6] = {};
    BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
    pEvt->Hdr.Evt = BT_HCI_EVT_COMMAND_COMPLETE;
    pEvt->Hdr.Len = 6;
    BtHciEvtCmdComplete_t *p = reinterpret_cast<BtHciEvtCmdComplete_t *>(pEvt->Data);
    p->NbCmdPacket = 3;
    p->CmdCode = opcode;
    p->RetParam[0] = 0x0C;
    p->RetParam[1] = 0xA5;
    p->RetParam[2] = 0x5A;

    BtHciProcessEvent(&dev, pEvt);
    CHECK(dev.CmdCredit == 3);
    CHECK(dev.CmdDone);
    CHECK(dev.CmdStatus == 0x0C);
    CHECK(ret[0] == 0xA5 && ret[1] == 0x5A);

    alignas(4) uint8_t statusRaw[sizeof(BtHciEvtPacketHdr_t) + sizeof(BtHciEvtCmdStatus_t)] = {};
    BtHciEvtPacket_t *pStatusEvt = reinterpret_cast<BtHciEvtPacket_t *>(statusRaw);
    pStatusEvt->Hdr.Evt = BT_HCI_EVT_COMMAND_STATUS;
    pStatusEvt->Hdr.Len = sizeof(BtHciEvtCmdStatus_t);
    BtHciEvtCmdStatus_t *pStatus =
        reinterpret_cast<BtHciEvtCmdStatus_t *>(pStatusEvt->Data);
    pStatus->Status = 0x01;
    pStatus->NbCmdPacket = 1;
    pStatus->CmdCode = static_cast<uint16_t>(opcode + 1U);

    dev.CmdDone = false;
    BtHciProcessEvent(&dev, pStatusEvt);
    CHECK(dev.CmdCredit == 1);
    CHECK(!dev.CmdDone);
}

void TestAclReassembly()
{
    BtHciDevice_t dev = {};
    ResetSmpCapture();

    uint8_t pdu[32] = {};
    const uint8_t payload[] = { 1, 2, 3, 4, 5 };
    size_t pduLen = BuildSmpPdu(pdu, 0x0B, payload, sizeof(payload));

    FeedAclFragment(&dev, 0x201, BT_HCI_PBFLAG_START_NONFLUSHABLE, pdu, 6);
    CHECK(s_SmpPacketCount == 0);
    FeedAclFragment(&dev, 0x201, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
                    &pdu[6], static_cast<uint16_t>(pduLen - 6));

    CHECK(s_SmpPacketCount == 1);
    CHECK(s_SmpPackets[0].ConnHdl == 0x201);
    CHECK(s_SmpPackets[0].Len == sizeof(payload) + 1U);
    CHECK(s_SmpPackets[0].Data[0] == 0x0B);
    CHECK(std::memcmp(&s_SmpPackets[0].Data[1], payload, sizeof(payload)) == 0);
}

void TestInterleavedReassembly()
{
    BtHciDevice_t dev = {};
    ResetSmpCapture();

    uint8_t pduA[32] = {};
    uint8_t pduB[32] = {};
    const uint8_t dataA[] = { 0xA1, 0xA2, 0xA3 };
    const uint8_t dataB[] = { 0xB1, 0xB2, 0xB3, 0xB4 };
    size_t lenA = BuildSmpPdu(pduA, 0x01, dataA, sizeof(dataA));
    size_t lenB = BuildSmpPdu(pduB, 0x02, dataB, sizeof(dataB));

    FeedAclFragment(&dev, 0x301, BT_HCI_PBFLAG_START_NONFLUSHABLE, pduA, 5);
    FeedAclFragment(&dev, 0x302, BT_HCI_PBFLAG_START_NONFLUSHABLE, pduB, 5);
    CHECK(s_SmpPacketCount == 0);

    FeedAclFragment(&dev, 0x302, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
                    &pduB[5], static_cast<uint16_t>(lenB - 5));
    FeedAclFragment(&dev, 0x301, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
                    &pduA[5], static_cast<uint16_t>(lenA - 5));

    CHECK(s_SmpPacketCount == 2);
    CHECK(s_SmpPackets[0].ConnHdl == 0x302);
    CHECK(s_SmpPackets[0].Data[0] == 0x02);
    CHECK(s_SmpPackets[1].ConnHdl == 0x301);
    CHECK(s_SmpPackets[1].Data[0] == 0x01);
}

// A Disconnection Complete event must free the SMP link record via
// BtSmpDisconnected, so cleanup does not depend on the application callback.
void TestDisconnectFreesSmp()
{
    BtHciDevice_t dev = {};
    s_SmpDisconnectCount = 0;
    s_SmpDisconnectHdl = 0;

    alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) +
                          sizeof(BtHciEvtDisconComplete_t)] = {};
    BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
    pEvt->Hdr.Evt = BT_HCI_EVT_DISCONN_COMPLETE;
    pEvt->Hdr.Len = sizeof(BtHciEvtDisconComplete_t);
    BtHciEvtDisconComplete_t *p =
        reinterpret_cast<BtHciEvtDisconComplete_t *>(pEvt->Data);
    p->Status = 0;
    p->ConnHdl = 0x0042;
    p->Reason = 0x13;

    BtHciProcessEvent(&dev, pEvt);

    CHECK(s_SmpDisconnectCount == 1);
    CHECK(s_SmpDisconnectHdl == 0x0042);
}

void TestMalformedFragmentsAndEvents()
{
    BtHciDevice_t dev = {};
    ResetSmpCapture();

    const uint8_t orphan[] = { 1, 2, 3 };
    FeedAclFragment(&dev, 0x401, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
                    orphan, sizeof(orphan));
    CHECK(s_SmpPacketCount == 0);

    uint8_t pdu[16] = {};
    const uint8_t payload[] = { 1, 2, 3 };
    size_t pduLen = BuildSmpPdu(pdu, 0x03, payload, sizeof(payload));
    FeedAclFragment(&dev, 0x401, BT_HCI_PBFLAG_START_NONFLUSHABLE, pdu, 5);

    const uint8_t tooLong[] = { 9, 9, 9, 9, 9 };
    FeedAclFragment(&dev, 0x401, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
                    tooLong, sizeof(tooLong));
    FeedAclFragment(&dev, 0x401, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
                    &pdu[5], static_cast<uint16_t>(pduLen - 5));
    CHECK(s_SmpPacketCount == 0);

    s_ScanReports = 0;
    dev.ScanReport = CaptureScanReport;
    alignas(4) uint8_t evtRaw[sizeof(BtHciEvtPacketHdr_t) + 11] = {};
    BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(evtRaw);
    pEvt->Hdr.Evt = BT_HCI_EVT_LE;
    pEvt->Hdr.Len = 11;
    BtHciLeEvtPacket_t *pLe = reinterpret_cast<BtHciLeEvtPacket_t *>(pEvt->Data);
    pLe->Evt = BT_HCI_EVT_LE_ADV_REPORT;
    BtHciLeEvtAdvReport_t *pReport =
        reinterpret_cast<BtHciLeEvtAdvReport_t *>(pLe->Data);
    pReport->NbReport = 1;
    pReport->Report[0].DataLen = 20;

    BtHciProcessEvent(&dev, pEvt);
    CHECK(s_ScanReports == 0);
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

void BtProcessSmpData(BtHciDevice_t * const, uint16_t ConnHdl,
                      BtL2CapSmp_t * const pSmp, size_t Len)
{
    if (s_SmpPacketCount >= s_SmpPackets.size() ||
        Len > s_SmpPackets[s_SmpPacketCount].Data.size())
    {
        return;
    }

    SmpCapture &pkt = s_SmpPackets[s_SmpPacketCount++];
    pkt.ConnHdl = ConnHdl;
    pkt.Len = Len;
    std::memcpy(pkt.Data.data(), pSmp, Len);
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

void BtSmpDisconnected(uint16_t ConnHdl)
{
    ++s_SmpDisconnectCount;
    s_SmpDisconnectHdl = ConnHdl;
}

SysLog_t * const SysLogGet(void)
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
    TestCommandFraming();
    TestAclFragmentationAndCredits();
    TestAclCompletionCredits();
    TestCompletedPacketsBounded();
    TestCommandCreditsAndCompletion();
    TestAclReassembly();
    TestInterleavedReassembly();
    TestMalformedFragmentsAndEvents();
    TestDisconnectFreesSmp();

    if (s_Failures != 0)
    {
        std::printf("Bluetooth HCI flow tests: %d failure(s), %d checks\n",
                    s_Failures, s_Checks);
        return 1;
    }

    std::printf("Bluetooth HCI flow tests: PASS (%d checks)\n", s_Checks);
    return 0;
}

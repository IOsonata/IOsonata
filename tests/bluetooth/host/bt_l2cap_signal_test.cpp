#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_peer.h"

namespace {

constexpr uint16_t kFrameHeaderLen = sizeof(BtL2CapCFrame_t) - 1U;
constexpr uint16_t kResponseMax =
    BT_HCI_BUFFER_MAX_SIZE - sizeof(BtHciACLDataPacketHdr_t) - sizeof(BtL2CapHdr_t);

int s_Failures = 0;
int s_Checks = 0;

#define CHECK(expr) do { \
    ++s_Checks; \
    if (!(expr)) { \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        ++s_Failures; \
    } \
} while (0)

struct HookState {
    bool ConnParamAccept;
    int ConnParamReqCount;
    uint16_t ConnParamConnHdl;
    BtL2CapConnParamUpdateReq_t ConnParamReq;

    int ConnParamRspCount;
    uint8_t ConnParamRspId;
    uint16_t ConnParamRspResult;

    int LeCocReqCount;
    uint16_t LeCocConnHdl;
    BtL2CapLeCreditBasedConnReq_t LeCocReq;
    BtL2CapLeCreditBasedConnRsp_t LeCocRsp;

    bool FlowCreditAccept;
    int FlowCreditCount;
    uint16_t FlowCreditConnHdl;
    uint16_t FlowCreditCid;
    uint16_t FlowCreditCredits;

    bool DisconnectAccept;
    int DisconnectReqCount;
    uint16_t DisconnectConnHdl;
    uint16_t DisconnectDcid;
    uint16_t DisconnectScid;

    int DisconnectRspCount;
    uint16_t DisconnectRspConnHdl;
    uint16_t DisconnectRspDcid;
    uint16_t DisconnectRspScid;

    int ReconfigureReqCount;
    uint16_t ReconfigureConnHdl;
    uint16_t ReconfigureLen;
    BtL2CapCreditBasedReconfReq_t ReconfigureReq;
    uint16_t ReconfigureResult;

    int ReconfigureRspCount;
    uint16_t ReconfigureRspConnHdl;
    uint16_t ReconfigureRspResult;
};

HookState s_Hooks = {};

// Role reported by the BtPeerRole stub below. Defaults to unknown so the
// existing tests keep the ungated behavior.
uint8_t s_PeerRole = BT_CONN_ROLE_UNKNOWN;

void ResetHooks()
{
    std::memset(&s_Hooks, 0, sizeof(s_Hooks));
    s_PeerRole = BT_CONN_ROLE_UNKNOWN;
    s_Hooks.LeCocRsp.Dcid = 0x0042;
    s_Hooks.LeCocRsp.Mtu = 128;
    s_Hooks.LeCocRsp.Mps = 64;
    s_Hooks.LeCocRsp.InitialCredits = 7;
    s_Hooks.LeCocRsp.Result = BT_L2CAP_LE_CBFC_RESULT_SUCCESS;
    s_Hooks.ReconfigureResult = BT_L2CAP_RECONFIG_RESULT_SUCCESS;
}

uint16_t AppendFrame(uint8_t *pOut, uint16_t Offset, uint8_t Code, uint8_t Id,
                     const void *pPayload, uint16_t PayloadLen)
{
    BtL2CapCFrame_t *pFrame = reinterpret_cast<BtL2CapCFrame_t *>(pOut + Offset);
    pFrame->Code = Code;
    pFrame->Id = Id;
    pFrame->Len = PayloadLen;
    if (PayloadLen > 0)
    {
        std::memcpy(pFrame->Data, pPayload, PayloadLen);
    }
    return static_cast<uint16_t>(Offset + kFrameHeaderLen + PayloadLen);
}

struct SignalExchange {
    alignas(4) std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE> ReqRaw = {};
    alignas(4) std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE> RspRaw = {};
    BtL2CapPdu_t *Req = reinterpret_cast<BtL2CapPdu_t *>(ReqRaw.data());
    BtL2CapPdu_t *Rsp = reinterpret_cast<BtL2CapPdu_t *>(RspRaw.data());
    uint16_t ReqLen = 0;

    uint8_t *Commands()
    {
        return reinterpret_cast<uint8_t *>(&Req->CFrame);
    }

    uint16_t Append(uint8_t Code, uint8_t Id, const void *pPayload = nullptr,
                    uint16_t PayloadLen = 0)
    {
        ReqLen = AppendFrame(Commands(), ReqLen, Code, Id, pPayload, PayloadLen);
        return ReqLen;
    }

    uint32_t Run(uint16_t ConnHdl = 0x0123)
    {
        Req->Hdr.Cid = BT_L2CAP_CID_SIGNAL;
        Req->Hdr.Len = ReqLen;
        return BtL2CapProcessSignal(nullptr, ConnHdl, Req, ReqLen, Rsp);
    }

    const BtL2CapCFrame_t *Frame(uint16_t Offset = 0) const
    {
        if (Offset + kFrameHeaderLen > Rsp->Hdr.Len)
        {
            return nullptr;
        }
        return reinterpret_cast<const BtL2CapCFrame_t *>(
            reinterpret_cast<const uint8_t *>(&Rsp->CFrame) + Offset);
    }
};

uint16_t FrameTotalLen(const BtL2CapCFrame_t *pFrame)
{
    return pFrame == nullptr ? 0 : static_cast<uint16_t>(kFrameHeaderLen + pFrame->Len);
}

void CheckReject(const BtL2CapCFrame_t *pFrame, uint8_t Id, uint16_t Reason)
{
    CHECK(pFrame != nullptr);
    if (pFrame == nullptr)
    {
        return;
    }
    CHECK(pFrame->Code == BT_L2CAP_CODE_COMMAND_REJECT_RSP);
    CHECK(pFrame->Id == Id);
    CHECK(pFrame->Len >= sizeof(uint16_t));
    if (pFrame->Len >= sizeof(uint16_t))
    {
        uint16_t reason = 0;
        std::memcpy(&reason, pFrame->Data, sizeof(reason));
        CHECK(reason == Reason);
    }
}

void TestNullAndShortInput()
{
    SignalExchange x;
    CHECK(BtL2CapProcessSignal(nullptr, 1, nullptr, 0, x.Rsp) == 0);
    CHECK(BtL2CapProcessSignal(nullptr, 1, x.Req, 0, nullptr) == 0);
    CHECK(BtL2CapProcessSignal(nullptr, 1, x.Req, kFrameHeaderLen - 1, x.Rsp) == 0);
}

void TestUnknownAndTruncatedCommand()
{
    SignalExchange unknown;
    unknown.Append(0xFF, 0x31);
    CHECK(unknown.Run() == kFrameHeaderLen + sizeof(uint16_t));
    CheckReject(unknown.Frame(), 0x31, BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD);

    SignalExchange truncated;
    uint8_t *p = truncated.Commands();
    BtL2CapCFrame_t *frame = reinterpret_cast<BtL2CapCFrame_t *>(p);
    frame->Code = BT_L2CAP_CODE_DISCONNECTION_REQ;
    frame->Id = 0x32;
    frame->Len = sizeof(BtL2CapDisconnReq_t);
    truncated.ReqLen = kFrameHeaderLen + 1;
    CHECK(truncated.Run() == kFrameHeaderLen + sizeof(uint16_t));
    CheckReject(truncated.Frame(), 0x32, BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD);
}

void TestConnectionParameterRequest()
{
    ResetHooks();
    SignalExchange accepted;
    BtL2CapConnParamUpdateReq_t req = { 24, 40, 0, 200 };
    s_Hooks.ConnParamAccept = true;
    accepted.Append(BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_REQ, 0x40,
                    &req, sizeof(req));
    CHECK(accepted.Run(0x0201) == kFrameHeaderLen + sizeof(BtL2CapConnParamUpdateRsp_t));
    CHECK(s_Hooks.ConnParamReqCount == 1);
    CHECK(s_Hooks.ConnParamConnHdl == 0x0201);
    CHECK(std::memcmp(&s_Hooks.ConnParamReq, &req, sizeof(req)) == 0);

    const BtL2CapCFrame_t *rspFrame = accepted.Frame();
    CHECK(rspFrame != nullptr);
    CHECK(rspFrame->Code == BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_RSP);
    CHECK(rspFrame->Id == 0x40);
    CHECK(rspFrame->Len == sizeof(BtL2CapConnParamUpdateRsp_t));
    if (rspFrame != nullptr && rspFrame->Len == sizeof(BtL2CapConnParamUpdateRsp_t))
    {
        const BtL2CapConnParamUpdateRsp_t *rsp =
            reinterpret_cast<const BtL2CapConnParamUpdateRsp_t *>(rspFrame->Data);
        CHECK(rsp->Result == BT_L2CAP_CONN_PARAM_ACCEPTED);
    }

    ResetHooks();
    SignalExchange invalid;
    req.IntervalMin = 50;
    req.IntervalMax = 40;
    s_Hooks.ConnParamAccept = true;
    invalid.Append(BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_REQ, 0x41,
                   &req, sizeof(req));
    invalid.Run();
    CHECK(s_Hooks.ConnParamReqCount == 0);
    rspFrame = invalid.Frame();
    CHECK(rspFrame != nullptr);
    if (rspFrame != nullptr)
    {
        const BtL2CapConnParamUpdateRsp_t *rsp =
            reinterpret_cast<const BtL2CapConnParamUpdateRsp_t *>(rspFrame->Data);
        CHECK(rsp->Result == BT_L2CAP_CONN_PARAM_REJECTED);
    }

    ResetHooks();
    SignalExchange wrongLength;
    wrongLength.Append(BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_REQ, 0x42,
                       &req, sizeof(req) - 1);
    wrongLength.Run();
    CHECK(s_Hooks.ConnParamReqCount == 0);
    rspFrame = wrongLength.Frame();
    CHECK(rspFrame != nullptr);
    if (rspFrame != nullptr)
    {
        const BtL2CapConnParamUpdateRsp_t *rsp =
            reinterpret_cast<const BtL2CapConnParamUpdateRsp_t *>(rspFrame->Data);
        CHECK(rsp->Result == BT_L2CAP_CONN_PARAM_REJECTED);
    }
}

// The Connection Parameter Update Request is peripheral to central only
// (Vol 3 Part A 4.20). When this device is the peripheral on the link the
// request gets a Command Reject and never reaches the accept hook; when it is
// the central (or the role is unknown) the normal update response path runs.
void TestConnParamRoleGate()
{
    BtL2CapConnParamUpdateReq_t req = { 24, 40, 0, 200 };

    // As peripheral: Command Reject, hook not called.
    ResetHooks();
    s_PeerRole = BT_CONN_ROLE_PERIPHERAL;
    s_Hooks.ConnParamAccept = true;
    SignalExchange periph;
    periph.Append(BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_REQ, 0x43,
                  &req, sizeof(req));
    CHECK(periph.Run(0x0205) == kFrameHeaderLen + sizeof(uint16_t));
    CheckReject(periph.Frame(), 0x43, BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD);
    CHECK(s_Hooks.ConnParamReqCount == 0);

    // As central: normal accepted response.
    ResetHooks();
    s_PeerRole = BT_CONN_ROLE_CENTRAL;
    s_Hooks.ConnParamAccept = true;
    SignalExchange central;
    central.Append(BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_REQ, 0x44,
                   &req, sizeof(req));
    CHECK(central.Run(0x0206) == kFrameHeaderLen + sizeof(BtL2CapConnParamUpdateRsp_t));
    CHECK(s_Hooks.ConnParamReqCount == 1);
    const BtL2CapCFrame_t *frame = central.Frame();
    CHECK(frame != nullptr);
    if (frame != nullptr)
    {
        CHECK(frame->Code == BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_RSP);
    }
}

void TestConnectionParameterResponseCallback()
{
    ResetHooks();
    SignalExchange x;
    BtL2CapConnParamUpdateRsp_t rsp = { BT_L2CAP_CONN_PARAM_ACCEPTED };
    x.Append(BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_RSP, 0x51,
             &rsp, sizeof(rsp));
    CHECK(x.Run(0x0202) == 0);
    CHECK(s_Hooks.ConnParamRspCount == 1);
    CHECK(s_Hooks.ConnParamRspId == 0x51);
    CHECK(s_Hooks.ConnParamRspResult == BT_L2CAP_CONN_PARAM_ACCEPTED);
}

void TestLeCreditBasedConnection()
{
    ResetHooks();
    SignalExchange valid;
    BtL2CapLeCreditBasedConnReq_t req = { 0x0025, 0x0041, 128, 64, 5 };
    valid.Append(BT_L2CAP_CODE_LE_CREDIT_BASED_CONNECTION_REQ, 0x60,
                 &req, sizeof(req));
    CHECK(valid.Run(0x0203) == kFrameHeaderLen + sizeof(BtL2CapLeCreditBasedConnRsp_t));
    CHECK(s_Hooks.LeCocReqCount == 1);
    CHECK(s_Hooks.LeCocConnHdl == 0x0203);
    CHECK(std::memcmp(&s_Hooks.LeCocReq, &req, sizeof(req)) == 0);

    const BtL2CapCFrame_t *frame = valid.Frame();
    CHECK(frame != nullptr);
    CHECK(frame->Code == BT_L2CAP_CODE_LE_CREDIT_BASED_CONNECTION_RSP);
    CHECK(frame->Id == 0x60);
    CHECK(frame->Len == sizeof(BtL2CapLeCreditBasedConnRsp_t));
    if (frame != nullptr && frame->Len == sizeof(BtL2CapLeCreditBasedConnRsp_t))
    {
        CHECK(std::memcmp(frame->Data, &s_Hooks.LeCocRsp, sizeof(s_Hooks.LeCocRsp)) == 0);
    }

    ResetHooks();
    SignalExchange malformed;
    malformed.Append(BT_L2CAP_CODE_LE_CREDIT_BASED_CONNECTION_REQ, 0x61,
                     &req, sizeof(req) - 1);
    malformed.Run();
    CHECK(s_Hooks.LeCocReqCount == 0);
    frame = malformed.Frame();
    CHECK(frame != nullptr);
    if (frame != nullptr)
    {
        const BtL2CapLeCreditBasedConnRsp_t *rsp =
            reinterpret_cast<const BtL2CapLeCreditBasedConnRsp_t *>(frame->Data);
        CHECK(rsp->Result == BT_L2CAP_LE_CBFC_RESULT_INVALID_PARAMETERS);
    }
}

void TestFlowControlCredit()
{
    ResetHooks();
    SignalExchange accepted;
    BtL2CapFlowControlCreditInd_t ind = { 0x0042, 9 };
    s_Hooks.FlowCreditAccept = true;
    accepted.Append(BT_L2CAP_CODE_FLOW_CONTROL_CREDIT_IND, 0x70,
                    &ind, sizeof(ind));
    CHECK(accepted.Run(0x0204) == 0);
    CHECK(s_Hooks.FlowCreditCount == 1);
    CHECK(s_Hooks.FlowCreditConnHdl == 0x0204);
    CHECK(s_Hooks.FlowCreditCid == ind.Cid);
    CHECK(s_Hooks.FlowCreditCredits == ind.Credits);

    ResetHooks();
    SignalExchange invalidCid;
    invalidCid.Append(BT_L2CAP_CODE_FLOW_CONTROL_CREDIT_IND, 0x71,
                      &ind, sizeof(ind));
    invalidCid.Run();
    CHECK(s_Hooks.FlowCreditCount == 1);
    const BtL2CapCFrame_t *frame = invalidCid.Frame();
    CheckReject(frame, 0x71, BT_L2CAP_CMD_REJECT_REASON_INVALID_CID);
    CHECK(frame != nullptr && frame->Len == 6);
    if (frame != nullptr && frame->Len == 6)
    {
        BtL2CapInvalidCidRejData_t rej = {};
        std::memcpy(&rej, &frame->Data[2], sizeof(rej));
        // Local CID (Dcid) then remote CID (Scid). The channel is known only by
        // the peer's source CID, so the local slot is 0 (Vol 3 Part A 4.1).
        CHECK(rej.Dcid == 0);
        CHECK(rej.Scid == ind.Cid);
    }

    ResetHooks();
    SignalExchange malformed;
    malformed.Append(BT_L2CAP_CODE_FLOW_CONTROL_CREDIT_IND, 0x72,
                     &ind, sizeof(ind) - 1);
    malformed.Run();
    CHECK(s_Hooks.FlowCreditCount == 0);
    CheckReject(malformed.Frame(), 0x72, BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD);
}

void TestDisconnectSignaling()
{
    ResetHooks();
    SignalExchange accepted;
    BtL2CapDisconnReq_t req = { 0x0042, 0x0041 };
    s_Hooks.DisconnectAccept = true;
    accepted.Append(BT_L2CAP_CODE_DISCONNECTION_REQ, 0x80, &req, sizeof(req));
    accepted.Run(0x0205);
    CHECK(s_Hooks.DisconnectReqCount == 1);
    CHECK(s_Hooks.DisconnectConnHdl == 0x0205);
    CHECK(s_Hooks.DisconnectDcid == req.Dcid);
    CHECK(s_Hooks.DisconnectScid == req.Scid);
    const BtL2CapCFrame_t *frame = accepted.Frame();
    CHECK(frame != nullptr);
    CHECK(frame->Code == BT_L2CAP_CODE_DISCONNECTION_RSP);
    CHECK(frame->Id == 0x80);
    CHECK(frame->Len == sizeof(BtL2CapDisconnRsp_t));
    CHECK(std::memcmp(frame->Data, &req, sizeof(req)) == 0);

    ResetHooks();
    SignalExchange rejected;
    rejected.Append(BT_L2CAP_CODE_DISCONNECTION_REQ, 0x81, &req, sizeof(req));
    rejected.Run();
    CheckReject(rejected.Frame(), 0x81, BT_L2CAP_CMD_REJECT_REASON_INVALID_CID);

    ResetHooks();
    SignalExchange response;
    response.Append(BT_L2CAP_CODE_DISCONNECTION_RSP, 0x82, &req, sizeof(req));
    CHECK(response.Run(0x0206) == 0);
    CHECK(s_Hooks.DisconnectRspCount == 1);
    CHECK(s_Hooks.DisconnectRspConnHdl == 0x0206);
    CHECK(s_Hooks.DisconnectRspDcid == req.Dcid);
    CHECK(s_Hooks.DisconnectRspScid == req.Scid);
}

void TestEnhancedCreditBasedRejection()
{
    SignalExchange request;
    request.Append(BT_L2CAP_CODE_CREDIT_BASED_CONNECTION_REQ, 0x90);
    request.Run();
    CheckReject(request.Frame(), 0x90, BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD);

    SignalExchange response;
    response.Append(BT_L2CAP_CODE_CREDIT_BASED_CONNECTION_RSP, 0x91);
    CHECK(response.Run() == 0);
}

void TestReconfigureSignaling()
{
    ResetHooks();
    SignalExchange valid;
    struct ReconfigurePayload {
        uint16_t Mtu;
        uint16_t Mps;
        uint16_t Dcid[2];
    } payload = { 200, 100, { 0x0042, 0x0043 } };
    valid.Append(BT_L2CAP_CODE_CREDIT_BASED_RECONFIGURE_REQ, 0xA0,
                 &payload, sizeof(payload));
    valid.Run(0x0207);
    CHECK(s_Hooks.ReconfigureReqCount == 1);
    CHECK(s_Hooks.ReconfigureConnHdl == 0x0207);
    CHECK(s_Hooks.ReconfigureLen == sizeof(payload));
    CHECK(s_Hooks.ReconfigureReq.Mtu == payload.Mtu);
    CHECK(s_Hooks.ReconfigureReq.Mps == payload.Mps);
    CHECK(s_Hooks.ReconfigureReq.Dcid[0] == payload.Dcid[0]);
    const BtL2CapCFrame_t *frame = valid.Frame();
    CHECK(frame != nullptr);
    CHECK(frame->Code == BT_L2CAP_CODE_CREDIT_BASED_RECONFIGURE_RSP);
    if (frame != nullptr)
    {
        const BtL2CapCreditBasedReconfRsp_t *rsp =
            reinterpret_cast<const BtL2CapCreditBasedReconfRsp_t *>(frame->Data);
        CHECK(rsp->Result == BT_L2CAP_RECONFIG_RESULT_SUCCESS);
    }

    ResetHooks();
    SignalExchange shortReq;
    shortReq.Append(BT_L2CAP_CODE_CREDIT_BASED_RECONFIGURE_REQ, 0xA1,
                    &payload, 4);
    shortReq.Run();
    CHECK(s_Hooks.ReconfigureReqCount == 0);
    frame = shortReq.Frame();
    CHECK(frame != nullptr);
    if (frame != nullptr)
    {
        const BtL2CapCreditBasedReconfRsp_t *rsp =
            reinterpret_cast<const BtL2CapCreditBasedReconfRsp_t *>(frame->Data);
        CHECK(rsp->Result == BT_L2CAP_RECONFIG_RESULT_INVALID_DCID);
    }

    ResetHooks();
    SignalExchange response;
    BtL2CapCreditBasedReconfRsp_t rsp = { BT_L2CAP_RECONFIG_RESULT_MPS_REDUCTION };
    response.Append(BT_L2CAP_CODE_CREDIT_BASED_RECONFIGURE_RSP, 0xA2,
                    &rsp, sizeof(rsp));
    CHECK(response.Run(0x0208) == 0);
    CHECK(s_Hooks.ReconfigureRspCount == 1);
    CHECK(s_Hooks.ReconfigureRspConnHdl == 0x0208);
    CHECK(s_Hooks.ReconfigureRspResult == BT_L2CAP_RECONFIG_RESULT_MPS_REDUCTION);
}

void TestMultipleCommands()
{
    ResetHooks();
    SignalExchange x;
    BtL2CapConnParamUpdateReq_t req = { 24, 40, 0, 200 };
    s_Hooks.ConnParamAccept = true;
    x.Append(0xFE, 0xB0);
    x.Append(BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_REQ, 0xB1,
             &req, sizeof(req));
    uint32_t len = x.Run();
    // The LE signaling channel carries one command per C-frame (Vol 3 Part A
    // 4): only the first command (the unknown 0xFE) is processed and rejected;
    // the trailing Connection Parameter Update is ignored.
    CHECK(len == kFrameHeaderLen + sizeof(uint16_t));
    const BtL2CapCFrame_t *first = x.Frame();
    CheckReject(first, 0xB0, BT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD);
    CHECK(x.Frame(FrameTotalLen(first)) == nullptr);
    CHECK(s_Hooks.ConnParamReqCount == 0);
}

void TestResponseBound()
{
    SignalExchange x;
    while (x.ReqLen + kFrameHeaderLen <= kResponseMax)
    {
        x.Append(0xFD, static_cast<uint8_t>(x.ReqLen / kFrameHeaderLen + 1U));
    }

    uint32_t len = x.Run();
    CHECK(len <= kResponseMax);
    CHECK(x.Rsp->Hdr.Len == len);

    // However many commands are packed into the C-frame, only the first is
    // processed, so the response is a single Command Reject and stays well
    // within the response bound.
    CHECK(len == kFrameHeaderLen + sizeof(uint16_t));
    const BtL2CapCFrame_t *frame = x.Frame();
    CHECK(frame != nullptr);
    if (frame != nullptr)
    {
        CHECK(frame->Code == BT_L2CAP_CODE_COMMAND_REJECT_RSP);
        CHECK(frame->Id == 0x01);
    }
    CHECK(x.Frame(FrameTotalLen(frame)) == nullptr);
}

} // namespace

uint8_t BtPeerRole(uint16_t)
{
    return s_PeerRole;
}

bool BtL2CapConnParamUpdateAccept(uint16_t ConnHdl,
                                  const BtL2CapConnParamUpdateReq_t *pReq)
{
    ++s_Hooks.ConnParamReqCount;
    s_Hooks.ConnParamConnHdl = ConnHdl;
    s_Hooks.ConnParamReq = *pReq;
    return s_Hooks.ConnParamAccept;
}

void BtL2CapConnParamUpdateRsp(uint16_t, uint8_t Id, uint16_t Result)
{
    ++s_Hooks.ConnParamRspCount;
    s_Hooks.ConnParamRspId = Id;
    s_Hooks.ConnParamRspResult = Result;
}

uint16_t BtL2CapLeCreditBasedConnectionReq(uint16_t ConnHdl,
                                            const BtL2CapLeCreditBasedConnReq_t *pReq,
                                            BtL2CapLeCreditBasedConnRsp_t *pRsp)
{
    ++s_Hooks.LeCocReqCount;
    s_Hooks.LeCocConnHdl = ConnHdl;
    s_Hooks.LeCocReq = *pReq;
    *pRsp = s_Hooks.LeCocRsp;
    return pRsp->Result;
}

bool BtL2CapFlowControlCreditInd(uint16_t ConnHdl, uint16_t Cid, uint16_t Credits)
{
    ++s_Hooks.FlowCreditCount;
    s_Hooks.FlowCreditConnHdl = ConnHdl;
    s_Hooks.FlowCreditCid = Cid;
    s_Hooks.FlowCreditCredits = Credits;
    return s_Hooks.FlowCreditAccept;
}

bool BtL2CapDisconnectReq(uint16_t ConnHdl, uint16_t Dcid, uint16_t Scid)
{
    ++s_Hooks.DisconnectReqCount;
    s_Hooks.DisconnectConnHdl = ConnHdl;
    s_Hooks.DisconnectDcid = Dcid;
    s_Hooks.DisconnectScid = Scid;
    return s_Hooks.DisconnectAccept;
}

void BtL2CapDisconnectRsp(uint16_t ConnHdl, uint16_t Dcid, uint16_t Scid)
{
    ++s_Hooks.DisconnectRspCount;
    s_Hooks.DisconnectRspConnHdl = ConnHdl;
    s_Hooks.DisconnectRspDcid = Dcid;
    s_Hooks.DisconnectRspScid = Scid;
}

uint16_t BtL2CapCreditBasedReconfigureReq(uint16_t ConnHdl,
                                           const BtL2CapCreditBasedReconfReq_t *pReq,
                                           uint16_t Len)
{
    ++s_Hooks.ReconfigureReqCount;
    s_Hooks.ReconfigureConnHdl = ConnHdl;
    s_Hooks.ReconfigureLen = Len;
    s_Hooks.ReconfigureReq = *pReq;
    return s_Hooks.ReconfigureResult;
}

void BtL2CapCreditBasedReconfigureRsp(uint16_t ConnHdl, uint16_t Result)
{
    ++s_Hooks.ReconfigureRspCount;
    s_Hooks.ReconfigureRspConnHdl = ConnHdl;
    s_Hooks.ReconfigureRspResult = Result;
}

int main()
{
    TestNullAndShortInput();
    TestUnknownAndTruncatedCommand();
    TestConnectionParameterRequest();
    TestConnParamRoleGate();
    TestConnectionParameterResponseCallback();
    TestLeCreditBasedConnection();
    TestFlowControlCredit();
    TestDisconnectSignaling();
    TestEnhancedCreditBasedRejection();
    TestReconfigureSignaling();
    TestMultipleCommands();
    TestResponseBound();

    if (s_Failures == 0)
    {
        std::printf("PASS bt_l2cap_signal_test: %d checks\n", s_Checks);
        return 0;
    }

    std::printf("FAIL bt_l2cap_signal_test: %d of %d checks failed\n",
                s_Failures, s_Checks);
    return 1;
}

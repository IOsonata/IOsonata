// Adversarial LE signaling tests.
//
// A signaling error response is never answered with another Command Reject.
// Oversized requests are rejected, but oversized identified responses are
// discarded.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_peer.h"

namespace {

bttest::Context s_Test("L2CAP adversarial host tests");

constexpr uint16_t kFrameHeaderLen =
		(uint16_t)(sizeof(BtL2CapCFrame_t) - 1U);

uint32_t RunFrame(uint8_t Code, uint8_t Id,
				  const uint8_t *pPayload, uint16_t PayloadLen,
				  BtL2CapPdu_t *pRsp)
{
	alignas(4) std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE> raw = {};
	BtL2CapPdu_t *pReq = (BtL2CapPdu_t *)raw.data();
	pReq->Hdr.Cid = BT_L2CAP_CID_SIGNAL;
	pReq->CFrame.Code = Code;
	pReq->CFrame.Id = Id;
	pReq->CFrame.Len = PayloadLen;
	if (PayloadLen > 0)
	{
		std::memcpy(pReq->CFrame.Data, pPayload, PayloadLen);
	}
	uint16_t reqLen = (uint16_t)(kFrameHeaderLen + PayloadLen);
	pReq->Hdr.Len = reqLen;

	return BtL2CapProcessSignal(nullptr, 0x0042,
			pReq, reqLen, pRsp);
}

void TestOversizedIdentifiedResponsesAreDiscarded()
{
	alignas(4) std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE> rspRaw = {};
	BtL2CapPdu_t *pRsp = (BtL2CapPdu_t *)rspRaw.data();
	uint8_t payload[24] = {};

	uint32_t n = RunFrame(
			BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_RSP,
			0x21, payload, sizeof(payload), pRsp);
	BT_CHECK(s_Test, n == 0);
	BT_CHECK(s_Test, pRsp->Hdr.Len == 0);

	std::memset(rspRaw.data(), 0, rspRaw.size());

	n = RunFrame(BT_L2CAP_CODE_COMMAND_REJECT_RSP,
			0x22, payload, sizeof(payload), pRsp);
	BT_CHECK(s_Test, n == 0);
	BT_CHECK(s_Test, pRsp->Hdr.Len == 0);
}

void TestOversizedRequestStillGetsMtuReject()
{
	alignas(4) std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE> rspRaw = {};
	BtL2CapPdu_t *pRsp = (BtL2CapPdu_t *)rspRaw.data();
	uint8_t payload[24] = {};

	uint32_t n = RunFrame(
			BT_L2CAP_CODE_CONNECTION_PARAMETER_UPDATE_REQ,
			0x31, payload, sizeof(payload), pRsp);
	BT_CHECK(s_Test, n ==
			kFrameHeaderLen + sizeof(uint16_t) + sizeof(uint16_t));
	BT_CHECK(s_Test,
			pRsp->CFrame.Code == BT_L2CAP_CODE_COMMAND_REJECT_RSP);
	BT_CHECK(s_Test, pRsp->CFrame.Id == 0x31);

	uint16_t reason = 0;
	uint16_t mtu = 0;
	std::memcpy(&reason, pRsp->CFrame.Data, sizeof(reason));
	std::memcpy(&mtu, pRsp->CFrame.Data + sizeof(reason), sizeof(mtu));
	BT_CHECK(s_Test,
			reason == BT_L2CAP_CMD_REJECT_REASON_MTU_EXCEEDED);
	BT_CHECK(s_Test, mtu == BT_L2CAP_LE_SIG_MTU);
}

} // namespace

uint8_t BtPeerRole(uint16_t)
{
	return BT_CONN_ROLE_UNKNOWN;
}

bool BtL2CapConnParamUpdateAccept(
		uint16_t, const BtL2CapConnParamUpdateReq_t *)
{
	return false;
}

void BtL2CapConnParamUpdateRsp(uint16_t, uint8_t, uint16_t)
{
}

uint16_t BtL2CapLeCreditBasedConnectionReq(
		uint16_t, const BtL2CapLeCreditBasedConnReq_t *,
		BtL2CapLeCreditBasedConnRsp_t *)
{
	return BT_L2CAP_LE_CBFC_RESULT_INVALID_PARAMETERS;
}

bool BtL2CapFlowControlCreditInd(uint16_t, uint16_t, uint16_t)
{
	return false;
}

bool BtL2CapDisconnectReq(uint16_t, uint16_t, uint16_t)
{
	return false;
}

void BtL2CapDisconnectRsp(uint16_t, uint16_t, uint16_t)
{
}

uint16_t BtL2CapCreditBasedReconfigureReq(
		uint16_t, const BtL2CapCreditBasedReconfReq_t *, uint16_t)
{
	return BT_L2CAP_RECONFIG_RESULT_INVALID_DCID;
}

void BtL2CapCreditBasedReconfigureRsp(uint16_t, uint16_t)
{
}

int main()
{
	s_Test.Run("oversized identified responses",
			   TestOversizedIdentifiedResponsesAreDiscarded);
	s_Test.Run("oversized request MTU rejection",
			   TestOversizedRequestStillGetsMtuReject);
	return s_Test.Finish();
}

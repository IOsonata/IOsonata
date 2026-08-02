// Adversarial GATT tests.
//
// These tests cross the GATT/ATT boundary and verify that initialization,
// subscription teardown and transmit completion bookkeeping preserve state
// under failure and mixed traffic.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_peer.h"

namespace {

bttest::Context s_Test("GATT adversarial host tests");

constexpr uint16_t kConnHdl = 0x0044;

BtDevice_t s_Peer;
BtHciDevice_t s_Hci;
int s_HciPacketCount = 0;
int s_TxCompleteCount = 0;
int s_NotifyTransitions = 0;
int s_IndicateTransitions = 0;
bool s_LastNotifyState = false;
bool s_LastIndicateState = false;
int s_LongWriteCallbacks = 0;
int s_LongWriteOffset = -1;
int s_LongWriteLen = -1;
uint8_t s_LongWriteData[16];
bool s_LongWriteSawPeerFinal = false;
BtGattChar_t *s_LongWritePeerChar = nullptr;

BtGattChar_t s_InitialChar;
BtGattSrvc_t s_InitialService;
BtGattChar_t s_SubChar;
BtGattSrvc_t s_SubService;
uint8_t s_InitialData[4] = { 0x11, 0x22, 0x33, 0x44 };

uint32_t DummySendData(void *, uint32_t Len)
{
	return Len;
}

void TxComplete(BtGattChar_t *, int)
{
	s_TxCompleteCount++;
}

void NotifyChanged(BtGattChar_t *, bool Enabled, uint16_t)
{
	s_NotifyTransitions++;
	s_LastNotifyState = Enabled;
}

void IndicateChanged(BtGattChar_t *, bool Enabled, uint16_t)
{
	s_IndicateTransitions++;
	s_LastIndicateState = Enabled;
}

void LongWriteChanged(BtGattChar_t *, uint8_t *pData, int Offset, int Len)
{
	s_LongWriteCallbacks++;
	s_LongWriteOffset = Offset;
	s_LongWriteLen = Len;
	if (Len > 0 && Len <= (int)sizeof(s_LongWriteData))
	{
		std::memcpy(s_LongWriteData, pData, (size_t)Len);
	}
	static const uint8_t expected[] = { 0x91, 0x92, 0x93 };
	s_LongWriteSawPeerFinal = s_LongWritePeerChar != nullptr &&
		s_LongWritePeerChar->ValueLen == sizeof(expected) &&
		std::memcmp(s_LongWritePeerChar->pValue,
					expected, sizeof(expected)) == 0;
}

void ResetPeer()
{
	std::memset(&s_Peer, 0, sizeof(s_Peer));
	std::memset(&s_Hci, 0, sizeof(s_Hci));
	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.pHciDev = &s_Hci;
	s_Hci.SendData = DummySendData;
	s_HciPacketCount = 0;
	s_TxCompleteCount = 0;
	s_NotifyTransitions = 0;
	s_IndicateTransitions = 0;
	s_LastNotifyState = false;
	s_LastIndicateState = false;
}

void BuildInitialService()
{
	std::memset(&s_InitialChar, 0, sizeof(s_InitialChar));
	std::memset(&s_InitialService, 0, sizeof(s_InitialService));

	s_InitialChar.Uuid = 0xFFF1;
	s_InitialChar.MaxDataLen = sizeof(s_InitialData);
	s_InitialChar.Property = BT_GATT_CHAR_PROP_READ;
	s_InitialChar.pValue = s_InitialData;
	s_InitialChar.ValueLen = sizeof(s_InitialData);

	s_InitialService.UuidSrvc = 0xFFE0;
	s_InitialService.NbChar = 1;
	s_InitialService.pCharArray = &s_InitialChar;
}

void BuildSubscriptionService()
{
	std::memset(&s_SubChar, 0, sizeof(s_SubChar));
	std::memset(&s_SubService, 0, sizeof(s_SubService));

	s_SubChar.Uuid = 0xFFF2;
	s_SubChar.MaxDataLen = 8;
	s_SubChar.Property =
			BT_GATT_CHAR_PROP_READ |
			BT_GATT_CHAR_PROP_NOTIFY |
			BT_GATT_CHAR_PROP_INDICATE;
	s_SubChar.SetNotifCB = NotifyChanged;
	s_SubChar.SetIndCB = IndicateChanged;
	s_SubChar.TxCompleteCB = TxComplete;

	s_SubService.UuidSrvc = 0xFFE1;
	s_SubService.NbChar = 1;
	s_SubService.pCharArray = &s_SubChar;
}

void TestInitialValueIsCopied()
{
	BuildInitialService();
	BT_CHECK(s_Test, BtGattSrvcAdd(&s_InitialService));
	BT_CHECK(s_Test, s_InitialChar.pValue != nullptr);
	BT_CHECK(s_Test, s_InitialChar.pValue != s_InitialData);
	BT_CHECK(s_Test, s_InitialChar.ValueLen == sizeof(s_InitialData));
	BT_CHECK(s_Test,
			std::memcmp(s_InitialChar.pValue,
						s_InitialData, sizeof(s_InitialData)) == 0);
}

void TestDisconnectReportsSubscriptionDisable()
{
	BuildSubscriptionService();
	BT_CHECK(s_Test, BtGattSrvcAdd(&s_SubService));
	BT_CHECK(s_Test, s_SubChar.CccdHdl != BT_ATT_HANDLE_INVALID);

	s_NotifyTransitions = 0;
	s_IndicateTransitions = 0;
	BT_CHECK(s_Test, BtGattCccdSet(kConnHdl, s_SubChar.CccdHdl,
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION |
			BT_DESC_CLIENT_CHAR_CONFIG_INDICATION));
	BT_CHECK(s_Test, s_NotifyTransitions == 1);
	BT_CHECK(s_Test, s_IndicateTransitions == 1);
	BT_CHECK(s_Test, s_LastNotifyState);
	BT_CHECK(s_Test, s_LastIndicateState);

	s_NotifyTransitions = 0;
	s_IndicateTransitions = 0;
	BtGattCccdClear(kConnHdl);
	BT_CHECK(s_Test, s_Peer.Conn.NbCccd == 0);
	BT_CHECK(s_Test, s_NotifyTransitions == 1);
	BT_CHECK(s_Test, s_IndicateTransitions == 1);
	BT_CHECK(s_Test, !s_LastNotifyState);
	BT_CHECK(s_Test, !s_LastIndicateState);
}

void TestMixedPacketCompletionOrdering()
{
	s_Peer.TxPendHead = 0;
	s_Peer.TxPendCount = 0;
	s_TxCompleteCount = 0;

	// An ATT response goes out on this link. It owns no GATT completion, so
	// the layer that sent it reports the packet; bt_hci_host.cpp does this
	// for real, and the stub below only counts.
	BtHciACLDataPacket_t untracked = {};
	BT_CHECK(s_Test, BtHciSendAcl(&s_Hci, &untracked) != 0);
	BtGattTxPendUntracked(kConnHdl, 1);

	BtGattTxPendingAdd(kConnHdl, &s_SubChar);
	BT_CHECK(s_Test, s_Peer.TxPendCount == 2);

	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 0);
	BT_CHECK(s_Test, s_Peer.TxPendCount == 1);

	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 1);
	BT_CHECK(s_Test, s_Peer.TxPendCount == 0);
}

void TestFragmentCompletionWaitsForFinalPacket()
{
	s_Peer.TxPendHead = 0;
	s_Peer.TxPendCount = 0;
	s_TxCompleteCount = 0;

	// One notification that the transport split into three ACL packets. The
	// send path derives the count from the PDU size and the controller's ACL
	// length; here it is stated directly.
	BT_CHECK(s_Test, BtGattTxPendReserve(kConnHdl, &s_SubChar, 3));
	BT_CHECK(s_Test, s_Peer.TxPendCount == 1);

	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 0);
	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 0);
	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 1);
}

void TestFullCompletionQueueRejectsSend()
{
	s_Peer.Conn.NbCccd = 1;
	s_Peer.Conn.Cccd[0].Hdl = s_SubChar.CccdHdl;
	s_Peer.Conn.Cccd[0].Value =
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION;
	s_Peer.TxPendHead = 0;
	s_Peer.TxPendCount = BT_DEV_TXPEND_MAX;
	s_HciPacketCount = 0;

	uint8_t value[2] = { 0xAA, 0x55 };
	bool ok = BtGattCharNotify(kConnHdl, &s_SubChar,
			value, sizeof(value));

	BT_CHECK(s_Test, !ok);
	BT_CHECK(s_Test, s_HciPacketCount == 0);
	BT_CHECK(s_Test, s_Peer.TxPendCount == BT_DEV_TXPEND_MAX);
}


void TestExecuteWriteCallbacksUseCanonicalRuns()
{
	static BtGattChar_t chars[2];
	static BtGattSrvc_t service;
	static bool registered = false;
	static uint8_t queue[96];

	if (!registered)
	{
		std::memset(chars, 0, sizeof(chars));
		std::memset(&service, 0, sizeof(service));
		for (int i = 0; i < 2; i++)
		{
			chars[i].Uuid = (uint16_t)(0xFFC0 + i);
			chars[i].MaxDataLen = 16;
			chars[i].Property = BT_GATT_CHAR_PROP_READ |
							 BT_GATT_CHAR_PROP_WRITE;
		}
		chars[0].WrCB = LongWriteChanged;
		service.UuidSrvc = 0xFFBF;
		service.NbChar = 2;
		service.pCharArray = chars;
		BT_CHECK(s_Test, BtGattSrvcAdd(&service));
		registered = true;
	}

	BT_CHECK(s_Test, BtGattCharSetValue(&chars[0], nullptr, 0));
	BT_CHECK(s_Test, BtGattCharSetValue(&chars[1], nullptr, 0));

	s_Peer.Conn.pLongWrBuff = queue;
	s_Peer.Conn.LongWrBuffSize = sizeof(queue);
	s_Peer.Conn.LongWrLen = 0;
	s_LongWriteCallbacks = 0;
	s_LongWriteOffset = -1;
	s_LongWriteLen = -1;
	std::memset(s_LongWriteData, 0, sizeof(s_LongWriteData));
	s_LongWriteSawPeerFinal = false;
	s_LongWritePeerChar = &chars[1];

	static const uint8_t first[] = { 1, 2, 3 };
	static const uint8_t second[] = { 4, 5, 6, 7 };
	static const uint8_t peer[] = { 0x91, 0x92, 0x93 };
	struct Fragment {
		uint16_t Hdl;
		uint16_t Offset;
		const uint8_t *pData;
		uint16_t Len;
	};
	const Fragment fragments[] = {
		{ chars[0].ValHdl, 0, first, sizeof(first) },
		{ chars[0].ValHdl, sizeof(first), second, sizeof(second) },
		{ chars[1].ValHdl, 0, peer, sizeof(peer) },
	};

	for (const Fragment &f : fragments)
	{
		uint8_t request[16] = {};
		uint8_t response[16] = {};
		request[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
		request[1] = (uint8_t)(f.Hdl & 0xFF);
		request[2] = (uint8_t)(f.Hdl >> 8);
		request[3] = (uint8_t)(f.Offset & 0xFF);
		request[4] = (uint8_t)(f.Offset >> 8);
		std::memcpy(request + 5, f.pData, f.Len);
		uint32_t len = BtAttProcessReq(kConnHdl,
			(BtAttReqRsp_t *)request, (uint16_t)(5 + f.Len),
			(BtAttReqRsp_t *)response);
		BT_CHECK(s_Test, len == (uint32_t)(5 + f.Len));
		BT_CHECK(s_Test,
			response[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);
	}

	BtAttReqRsp_t execute = {};
	BtAttReqRsp_t response = {};
	execute.OpCode = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	execute.ExecuteWriteReq.Flag = 1;
	uint32_t len = BtAttProcessReq(kConnHdl, &execute, 2, &response);

	static const uint8_t expected[] = { 1, 2, 3, 4, 5, 6, 7 };
	BT_CHECK(s_Test, len == 1);
	BT_CHECK(s_Test,
		response.OpCode == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP);
	BT_CHECK(s_Test, s_LongWriteCallbacks == 1);
	BT_CHECK(s_Test, s_LongWriteOffset == 0);
	BT_CHECK(s_Test, s_LongWriteLen == (int)sizeof(expected));
	BT_CHECK(s_Test,
		std::memcmp(s_LongWriteData, expected, sizeof(expected)) == 0);
	BT_CHECK(s_Test, chars[0].ValueLen == sizeof(expected));
	BT_CHECK(s_Test,
		std::memcmp(chars[0].pValue, expected, sizeof(expected)) == 0);
	BT_CHECK(s_Test, s_LongWriteSawPeerFinal);
}

void TestExecuteWriteCccdCapacityIsAtomic()
{
	static BtGattChar_t chars[2];
	static BtGattSrvc_t service;
	static bool registered = false;
	static uint8_t queue[64];

	if (!registered)
	{
		std::memset(chars, 0, sizeof(chars));
		std::memset(&service, 0, sizeof(service));
		for (int i = 0; i < 2; i++)
		{
			chars[i].Uuid = (uint16_t)(0xFFD0 + i);
			chars[i].MaxDataLen = 1;
			chars[i].Property = BT_GATT_CHAR_PROP_READ |
							 BT_GATT_CHAR_PROP_NOTIFY;
			chars[i].SetNotifCB = NotifyChanged;
		}
		service.UuidSrvc = 0xFFCF;
		service.NbChar = 2;
		service.pCharArray = chars;
		BT_CHECK(s_Test, BtGattSrvcAdd(&service));
		registered = true;
	}

	s_Peer.Conn.pLongWrBuff = queue;
	s_Peer.Conn.LongWrBuffSize = sizeof(queue);
	s_Peer.Conn.LongWrLen = 0;
	s_Peer.Conn.NbCccd = BT_GATT_CCCD_STATE_MAX - 1;
	for (uint8_t i = 0; i < s_Peer.Conn.NbCccd; i++)
	{
		s_Peer.Conn.Cccd[i].Hdl = (uint16_t)(0x7000 + i);
		s_Peer.Conn.Cccd[i].Value =
				BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION;
	}
	s_NotifyTransitions = 0;

	for (int i = 0; i < 2; i++)
	{
		uint8_t request[7] = {};
		uint8_t response[7] = {};
		request[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
		request[1] = (uint8_t)(chars[i].CccdHdl & 0xFF);
		request[2] = (uint8_t)(chars[i].CccdHdl >> 8);
		request[3] = 0;
		request[4] = 0;
		request[5] = 1;
		request[6] = 0;
		uint32_t len = BtAttProcessReq(
				kConnHdl,
				(BtAttReqRsp_t *)request,
				sizeof(request),
				(BtAttReqRsp_t *)response);
		BT_CHECK(s_Test, len == sizeof(response));
		BT_CHECK(s_Test,
				 response[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);
	}

	BtAttReqRsp_t execute = {};
	BtAttReqRsp_t response = {};
	execute.OpCode = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	execute.ExecuteWriteReq.Flag = 1;
	uint32_t len = BtAttProcessReq(kConnHdl, &execute, 2, &response);

	BT_CHECK(s_Test, len == sizeof(BtAttErrorRsp_t) + 1);
	BT_CHECK(s_Test, response.OpCode == BT_ATT_OPCODE_ATT_ERROR_RSP);
	BT_CHECK(s_Test, response.ErrorRsp.Error == BT_ATT_ERROR_INSUF_RESOURCE);
	BT_CHECK(s_Test, s_Peer.Conn.NbCccd == BT_GATT_CCCD_STATE_MAX - 1);
	BT_CHECK(s_Test, BtGattCccdGet(kConnHdl, chars[0].CccdHdl) == 0);
	BT_CHECK(s_Test, BtGattCccdGet(kConnHdl, chars[1].CccdHdl) == 0);
	BT_CHECK(s_Test, s_NotifyTransitions == 0);
}

} // namespace

extern "C" {

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	return Hdl == kConnHdl ? &s_Peer : nullptr;
}

uint16_t BtPeerCount(void)
{
	return 1;
}

BtDevice_t *BtPeerSlot(uint16_t Idx)
{
	return Idx == 0 ? &s_Peer : nullptr;
}

size_t BtPeerGetConnectedHandles(uint16_t *pHdl, size_t MaxCount)
{
	if (pHdl != nullptr && MaxCount > 0)
	{
		pHdl[0] = kConnHdl;
		return 1;
	}
	return 0;
}

void BtSmpBondCccdSave(uint16_t, uint16_t, uint16_t)
{
}

uint8_t BtSmpBondCccdGet(uint16_t, uint16_t *, uint16_t *, uint8_t)
{
	return 0;
}

bool BtSmpSignVerify(uint16_t, const uint8_t *, size_t, const uint8_t *)
{
	return false;
}

bool BtGapConnSecGet(uint16_t, BtConnSec_t *)
{
	return false;
}

void BtGattSrvcEvtHandler(BtGattSrvc_t * const, uint32_t, void * const)
{
}

uint32_t BtHciSendAcl(BtHciDevice_t * const,
					  BtHciACLDataPacket_t * const pPacket)
{
	s_HciPacketCount++;
	return pPacket != nullptr ?
			(uint32_t)pPacket->Hdr.Len + sizeof(pPacket->Hdr) : 1;
}

void BtGattClientNotified(uint16_t, uint16_t, uint8_t *, uint16_t)
{
}

} // extern "C"

int main()
{
	ResetPeer();
	BtAttDBInit(4096);

	s_Test.Run("initial value copy", TestInitialValueIsCopied);
	s_Test.Run("subscription teardown callbacks",
			   TestDisconnectReportsSubscriptionDisable);
	s_Test.Run("mixed packet completion ordering",
			   TestMixedPacketCompletionOrdering);
	s_Test.Run("fragment completion ordering",
			   TestFragmentCompletionWaitsForFinalPacket);
	s_Test.Run("completion queue exhaustion",
			   TestFullCompletionQueueRejectsSend);
	s_Test.Run("execute write canonical callback",
			   TestExecuteWriteCallbacksUseCanonicalRuns);
	s_Test.Run("execute write CCCD capacity atomicity",
			   TestExecuteWriteCccdCapacityIsAtomic);
	return s_Test.Finish();
}

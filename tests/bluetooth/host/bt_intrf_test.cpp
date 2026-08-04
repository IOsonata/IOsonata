// Bluetooth DeviceIntrf boundary and queue-ownership tests.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "bluetooth/bt_intrf.h"
#include "bluetooth/bt_peer.h"

namespace {

bttest::Context s_Test("Bluetooth interface host tests");
constexpr uint16_t kConnHdl = 0x0042;

BtDevice_t s_Peer;
bool s_PeerEnabled;
bool s_NotifyEnabled;
bool s_NotifyResult;
int s_NotifyCount;
uint8_t s_Notified[8][BTINTRF_TRANSBUFF_MAXLEN];
size_t s_NotifiedLen[8];
int s_RxEventCount;

int RxEvent(DevIntrf_t *, DEVINTRF_EVT Evt, uint8_t *, int)
{
	if (Evt == DEVINTRF_EVT_RX_DATA)
	{
		s_RxEventCount++;
	}
	return 0;
}

struct Fixture {
	BtGattChar_t Char[2];
	BtGattSrvc_t Service;
	BtDevIntrf_t Intrf;
	alignas(CFifo_t) uint8_t RxMem[BTINTRF_CFIFO_TOTAL_MEMSIZE(4, 8)];
	alignas(CFifo_t) uint8_t TxMem[BTINTRF_CFIFO_TOTAL_MEMSIZE(2, 8)];

	Fixture()
	{
		std::memset(Char, 0, sizeof(Char));
		std::memset(&Service, 0, sizeof(Service));
		Service.pCharArray = Char;
		Service.NbChar = 2;

		// BtGattSrvcAdd establishes this back-pointer during normal service
		// registration. This isolated test does not call a platform GATT port,
		// so mirror the registered-service state explicitly before invoking
		// characteristic callbacks.
		Char[0].pSrvc = &Service;
		Char[1].pSrvc = &Service;
		Char[0].Property = BT_GATT_CHAR_PROP_WRITE;
		Char[0].MaxDataLen = 64;
		Char[1].Property = BT_GATT_CHAR_PROP_NOTIFY;
		Char[1].MaxDataLen = 8;
	}

	BtIntrfCfg_t Config()
	{
		BtIntrfCfg_t cfg = {};
		cfg.pSrvc = &Service;
		cfg.RxCharIdx = 0;
		cfg.TxCharIdx = 1;
		cfg.PacketSize = 8;
		cfg.bBlocking = true;
		cfg.pRxFifoMem = RxMem;
		cfg.RxFifoMemSize = sizeof(RxMem);
		cfg.pTxFifoMem = TxMem;
		cfg.TxFifoMemSize = sizeof(TxMem);
		cfg.EvtCB = RxEvent;
		return cfg;
	}
};

void ResetLink()
{
	std::memset(&s_Peer, 0, sizeof(s_Peer));
	s_Peer.Conn.Hdl = kConnHdl;
	s_PeerEnabled = true;
	s_NotifyEnabled = true;
	s_NotifyResult = true;
	s_NotifyCount = 0;
	std::memset(s_Notified, 0, sizeof(s_Notified));
	std::memset(s_NotifiedLen, 0, sizeof(s_NotifiedLen));
	s_RxEventCount = 0;
}

void TestInitializationValidation()
{
	ResetLink();
	Fixture f;
	BtIntrfCfg_t cfg = f.Config();

	BT_CHECK(s_Test, !BtIntrfInit(nullptr, &cfg));
	BT_CHECK(s_Test, !BtIntrfInit(&f.Intrf, nullptr));

	BtGattSrvc_t *savedService = cfg.pSrvc;
	cfg.pSrvc = nullptr;
	BT_CHECK(s_Test, !BtIntrfInit(&f.Intrf, &cfg));
	cfg.pSrvc = savedService;

	BtGattChar_t *savedChars = cfg.pSrvc->pCharArray;
	cfg.pSrvc->pCharArray = nullptr;
	BT_CHECK(s_Test, !BtIntrfInit(&f.Intrf, &cfg));
	cfg.pSrvc->pCharArray = savedChars;

	cfg.RxCharIdx = cfg.pSrvc->NbChar;
	BT_CHECK(s_Test, !BtIntrfInit(&f.Intrf, &cfg));
	cfg = f.Config();
	cfg.TxCharIdx = -2;
	BT_CHECK(s_Test, !BtIntrfInit(&f.Intrf, &cfg));

	cfg = f.Config();
	cfg.PacketSize = BTINTRF_TRANSBUFF_MAXLEN + 1;
	BT_CHECK(s_Test, !BtIntrfInit(&f.Intrf, &cfg));

	cfg = f.Config();
	cfg.pTxFifoMem = f.TxMem + 1;
	cfg.TxFifoMemSize--;
	BT_CHECK(s_Test, !BtIntrfInit(&f.Intrf, &cfg));

	cfg = f.Config();
	cfg.TxFifoMemSize = sizeof(CFifo_t) + 7;
	BT_CHECK(s_Test, !BtIntrfInit(&f.Intrf, &cfg));

	cfg = f.Config();
	BT_CHECK(s_Test, BtIntrfInit(&f.Intrf, &cfg));
	BT_CHECK(s_Test, f.Service.pContext == &f.Intrf);
	BT_CHECK(s_Test, f.Char[0].WrCB != nullptr);
	BT_CHECK(s_Test, f.Char[1].TxCompleteCB != nullptr);
	BT_CHECK(s_Test, f.Intrf.DevIntrf.TxSrData != nullptr);
	BT_CHECK(s_Test, f.Intrf.DevIntrf.Reset != nullptr);
	BT_CHECK(s_Test, f.Intrf.DevIntrf.PowerOff != nullptr);
	BT_CHECK(s_Test, f.Intrf.DevIntrf.GetHandle != nullptr);
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.GetHandle(&f.Intrf.DevIntrf) == &f.Intrf);
}

void TestInvalidDataArguments()
{
	ResetLink();
	Fixture f;
	BtIntrfCfg_t cfg = f.Config();
	BT_CHECK(s_Test, BtIntrfInit(&f.Intrf, &cfg));

	uint8_t byte = 0;
	BT_CHECK(s_Test, f.Intrf.DevIntrf.RxData(nullptr, &byte, 1) == 0);
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.RxData(&f.Intrf.DevIntrf, nullptr, 1) == 0);
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.RxData(&f.Intrf.DevIntrf, &byte, -1) == 0);
	BT_CHECK(s_Test, f.Intrf.DevIntrf.TxData(nullptr, &byte, 1) == 0);
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.TxData(&f.Intrf.DevIntrf, nullptr, 1) == 0);
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.TxData(&f.Intrf.DevIntrf, &byte, -1) == 0);
}

void TestRxPacketOwnership()
{
	ResetLink();
	Fixture f;
	BtIntrfCfg_t cfg = f.Config();
	BT_CHECK(s_Test, BtIntrfInit(&f.Intrf, &cfg));

	uint8_t incoming[12];
	for (size_t i = 0; i < sizeof(incoming); i++)
	{
		incoming[i] = (uint8_t)(0x30U + i);
	}
	f.Char[0].WrCB(&f.Char[0], incoming, 0, sizeof(incoming));
	BT_CHECK(s_Test, s_RxEventCount == 1);

	uint8_t out[8] = {};
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.RxData(&f.Intrf.DevIntrf, out, sizeof(out)) == 8);
	BT_CHECK(s_Test, std::memcmp(out, incoming, 8) == 0);
	std::memset(out, 0, sizeof(out));
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.RxData(&f.Intrf.DevIntrf, out, sizeof(out)) == 4);
	BT_CHECK(s_Test, std::memcmp(out, incoming + 8, 4) == 0);
}

void TestTxRetryAndReset()
{
	ResetLink();
	Fixture f;
	BtIntrfCfg_t cfg = f.Config();
	BT_CHECK(s_Test, BtIntrfInit(&f.Intrf, &cfg));

	uint8_t data[12];
	for (size_t i = 0; i < sizeof(data); i++)
	{
		data[i] = (uint8_t)(0x80U + i);
	}

	s_NotifyResult = false;
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.TxData(&f.Intrf.DevIntrf, data, sizeof(data)) == 12);
	BT_CHECK(s_Test, s_NotifyCount == 1);
	BT_CHECK(s_Test, f.Intrf.TransBuffLen == 8);
	BT_CHECK(s_Test, CFifoUsed(f.Intrf.hTxFifo) == 1);

	s_NotifyResult = true;
	f.Char[1].TxCompleteCB(&f.Char[1], 1);
	BT_CHECK(s_Test, s_NotifyCount == 3);
	BT_CHECK(s_Test, s_NotifiedLen[1] == 8);
	BT_CHECK(s_Test, std::memcmp(s_Notified[1], data, 8) == 0);
	BT_CHECK(s_Test, s_NotifiedLen[2] == 4);
	BT_CHECK(s_Test, std::memcmp(s_Notified[2], data + 8, 4) == 0);
	BT_CHECK(s_Test, f.Intrf.TransBuffLen == 0);
	BT_CHECK(s_Test, CFifoUsed(f.Intrf.hTxFifo) == 0);

	s_NotifyResult = false;
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.TxData(&f.Intrf.DevIntrf, data, 8) == 8);
	BT_CHECK(s_Test, f.Intrf.TransBuffLen == 8);
	f.Intrf.DevIntrf.Reset(&f.Intrf.DevIntrf);
	BT_CHECK(s_Test, f.Intrf.TransBuffLen == 0);
	BT_CHECK(s_Test, CFifoUsed(f.Intrf.hTxFifo) == 0);
	BT_CHECK(s_Test, CFifoUsed(f.Intrf.hRxFifo) == 0);
}

void TestNoSubscribedPeerKeepsQueue()
{
	ResetLink();
	Fixture f;
	BtIntrfCfg_t cfg = f.Config();
	BT_CHECK(s_Test, BtIntrfInit(&f.Intrf, &cfg));

	s_NotifyEnabled = false;
	uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	BT_CHECK(s_Test,
		f.Intrf.DevIntrf.TxData(&f.Intrf.DevIntrf, data, sizeof(data)) == 8);
	BT_CHECK(s_Test, s_NotifyCount == 0);
	BT_CHECK(s_Test, CFifoUsed(f.Intrf.hTxFifo) == 1);
}

} // namespace

extern "C" {

uint16_t BtPeerCount(void)
{
	return s_PeerEnabled ? 1 : 0;
}

BtDevice_t *BtPeerSlot(uint16_t Idx)
{
	return s_PeerEnabled && Idx == 0 ? &s_Peer : nullptr;
}

bool BtGattCharNotifyEnabled(uint16_t ConnHdl, BtGattChar_t *)
{
	return ConnHdl == kConnHdl && s_NotifyEnabled;
}

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *,
		void * const pVal, size_t Len)
{
	if (ConnHdl != kConnHdl || (Len > 0 && pVal == nullptr))
	{
		return false;
	}

	int idx = s_NotifyCount++;
	if (idx < (int)(sizeof(s_NotifiedLen) / sizeof(s_NotifiedLen[0])))
	{
		s_NotifiedLen[idx] = Len;
		if (Len <= sizeof(s_Notified[idx]))
		{
			std::memcpy(s_Notified[idx], pVal, Len);
		}
	}
	return s_NotifyResult;
}

} // extern "C"

int main()
{
	s_Test.Run("initialization validation", TestInitializationValidation);
	s_Test.Run("invalid data arguments", TestInvalidDataArguments);
	s_Test.Run("RX packet ownership", TestRxPacketOwnership);
	s_Test.Run("TX retry and reset", TestTxRetryAndReset);
	s_Test.Run("unsubscribed peer keeps queue", TestNoSubscribedPeerKeepsQueue);
	return s_Test.Finish();
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci_ctlr.h"

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

constexpr size_t kPayloadSize = 64;
constexpr size_t kPacketSize = kPayloadSize + BTHCICTLR_PKTHDR_LEN;
constexpr size_t kFifoSize = CFIFO_TOTAL_MEMSIZE(2, kPacketSize);

void TestDefaultFifo()
{
	BtHciCtlrDev_t dev = {};
	BtHciCtlrCfg_t cfg = {};

	CHECK(BtHciCtlrInit(&dev, &cfg));
	CHECK(dev.hRxFifo != nullptr);
	if (dev.hRxFifo != nullptr)
	{
		CHECK(CFifoBlockSize(dev.hRxFifo) ==
			  BT_HCI_CTLR_MTU_MAX + BTHCICTLR_PKTHDR_LEN);
		CHECK(CFifoAvail(dev.hRxFifo) == 4);
	}
}

void TestCustomFifo()
{
	alignas(CFifo_t) std::array<uint8_t, kFifoSize> fifoMem = {};
	BtHciCtlrDev_t dev = {};
	BtHciCtlrCfg_t cfg = {};
	cfg.PacketSize = kPayloadSize;
	cfg.pRxFifoMem = fifoMem.data();
	cfg.RxFifoMemSize = static_cast<int>(fifoMem.size());

	CHECK(BtHciCtlrInit(&dev, &cfg));
	CHECK(dev.hRxFifo == reinterpret_cast<hCFifo_t>(fifoMem.data()));
	CHECK(dev.PacketSize == kPacketSize);
	CHECK(CFifoBlockSize(dev.hRxFifo) == kPacketSize);
	CHECK(CFifoAvail(dev.hRxFifo) == 2);

	const uint8_t payload[] = { 0x11, 0x22, 0x33, 0x44 };
	CHECK(dev.Receive != nullptr);
	CHECK(dev.Receive(&dev, 0x1234, const_cast<uint8_t *>(payload),
					   sizeof(payload)) == sizeof(payload));
	CHECK(CFifoUsed(dev.hRxFifo) == 1);

	BtHciCtlrPkt_t *packet =
		reinterpret_cast<BtHciCtlrPkt_t *>(CFifoGet(dev.hRxFifo));
	CHECK(packet != nullptr);
	if (packet != nullptr)
	{
		CHECK(packet->ValHdl == 0x1234);
		CHECK(packet->Len == sizeof(payload));
		CHECK(packet->Off == 0);
		CHECK(std::memcmp(packet->Data, payload, sizeof(payload)) == 0);
	}
}

void TestInvalidCustomFifo()
{
	alignas(CFifo_t) std::array<uint8_t, kFifoSize> fifoMem = {};
	BtHciCtlrCfg_t cfg = {};
	cfg.PacketSize = kPayloadSize;
	cfg.pRxFifoMem = fifoMem.data();

	BtHciCtlrDev_t noSize = {};
	cfg.RxFifoMemSize = 0;
	CHECK(!BtHciCtlrInit(&noSize, &cfg));

	// One byte short of a single complete packet, so the FIFO cannot hold even
	// one element. kFifoSize holds two packets, so kFifoSize - 1 still holds
	// one and would be accepted; size against the one-packet minimum instead.
	alignas(CFifo_t) std::array<uint8_t, CFIFO_TOTAL_MEMSIZE(1, kPacketSize) - 1> shortMem = {};
	BtHciCtlrDev_t tooSmall = {};
	cfg.pRxFifoMem = shortMem.data();
	cfg.RxFifoMemSize = static_cast<int>(shortMem.size());
	CHECK(!BtHciCtlrInit(&tooSmall, &cfg));

	alignas(CFifo_t) std::array<uint8_t, kFifoSize + alignof(CFifo_t)> raw = {};
	BtHciCtlrDev_t misaligned = {};
	cfg.pRxFifoMem = raw.data() + 1;
	cfg.RxFifoMemSize = static_cast<int>(kFifoSize);
	CHECK(!BtHciCtlrInit(&misaligned, &cfg));

	BtHciCtlrDev_t oversized = {};
	cfg = {};
	cfg.PacketSize = static_cast<size_t>(UINT32_MAX);
	CHECK(!BtHciCtlrInit(&oversized, &cfg));
}

} // namespace

int main()
{
	TestDefaultFifo();
	TestCustomFifo();
	TestInvalidCustomFifo();

	if (s_Failures != 0)
	{
		std::printf("HCI controller init host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("HCI controller init host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}

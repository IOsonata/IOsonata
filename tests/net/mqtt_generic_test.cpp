/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Actual generic MQTT example and shared client through two DeviceIntrf ports.
 * The peers model stream I/O, not a modem, radio or Ethernet controller.
 */
#include <assert.h>
#include <stdio.h>
#include <algorithm>
#include <deque>
#include <string>
#include <vector>
#define main GenericMqttDemoMain
#include "../../exemples/net/mqtt_generic.cpp"
#undef main

class DemoRng : public RngEngine
{
public:
	uint8_t Seed = 0;
	unsigned Calls = 0;
	bool Works = true;

	bool Enable() override
	{
		return Works;
	}
	void Disable() override
	{
	}
	void Reset() override
	{
	}
	CRYPTO_STATUS Random(uint8_t *pData, size_t Len) override
	{
		Calls++;
		if (!Works)
		{
			return CRYPTO_STATUS_FAIL;
		}
		for (size_t i = 0; i < Len; i++)
		{
			pData[i] = Seed++;
		}
		return CRYPTO_STATUS_OK;
	}
};

struct WirePacket
{
	std::vector<uint8_t> Bytes;
	uint32_t Time;
	unsigned Session;
	unsigned Medium;
};

struct StreamState
{
	std::deque<uint8_t> Input;
	std::vector<uint8_t> Output;
	unsigned Reads = 0;
	unsigned Writes = 0;
	int Chunk = 1024;
	bool BlockTx = false;
	bool Connected = false;
};

static DemoRng s_TestRng;
static RngEngine *s_TestRngPointer = &s_TestRng;
static uint32_t s_TestNow;
static uint32_t s_TestOpenDuration;
static uint32_t s_TestCloseDuration;
static uint32_t s_TestWriteDuration;
static unsigned s_TestOpens;
static unsigned s_TestCloses;
static unsigned s_TestStarts;
static unsigned s_TestSelected;
static int s_TestActive = -1;
static bool s_TestReady;
static bool s_TestOpenWorks;
static bool s_TestInitWorks;
static bool s_TestStartWorks;
static void (*s_TestHandler)(uint32_t);
static StreamState s_TestStreams[2];
static std::vector<WirePacket> s_TestPackets;
static std::string s_TestLog;

static StreamState &Current()
{
	return s_TestStreams[s_TestSelected];
}

static size_t Header(const std::vector<uint8_t> &Bytes)
{
	for (size_t i = 1; i < Bytes.size() && i < 5; i++)
	{
		if (!(Bytes[i] & 128U))
		{
			return i + 1U;
		}
	}
	return 0;
}

static uint16_t Word(const std::vector<uint8_t> &Bytes, size_t Offset)
{
	assert(Offset + 2U <= Bytes.size());
	return (uint16_t)((uint16_t)Bytes[Offset] << 8 | Bytes[Offset + 1U]);
}

static std::string String(const std::vector<uint8_t> &Bytes, size_t Offset)
{
	size_t len = Word(Bytes, Offset);

	assert(Offset + 2U + len <= Bytes.size());
	return std::string(Bytes.begin() + Offset + 2U, Bytes.begin() + Offset + 2U + len);
}

// Two concrete C callback tables and independently owned I/O state. Neither
// class is SockIntrf, and MQTT must use the object supplied by NetDemoOpen.
template<unsigned Index> class DemoStream : public DeviceIntrf
{
public:
	DevIntrf_t Dev = {};

	DemoStream()
	{
		Dev.pDevData = &s_TestStreams[Index];
		Dev.StartRx = Dev.StartTx = Start;
		Dev.StopRx = Dev.StopTx = Stop;
		Dev.RxData = Receive;
		Dev.TxData = Send;
		Dev.EnCnt = 1;
		Dev.MaxRetry = 0;
		atomic_flag_clear(&Dev.bBusy);
	}

	operator DevIntrf_t *() override
	{
		return &Dev;
	}
	uint32_t Rate(uint32_t) override
	{
		return 0;
	}
	uint32_t Rate() override
	{
		return 0;
	}

private:
	static bool Start(DevIntrf_t *pDev, uint32_t Addr)
	{
		assert(Addr == 0 && pDev->pDevData == &s_TestStreams[Index]);
		return s_TestStreams[Index].Connected;
	}
	static void Stop(DevIntrf_t *)
	{
	}
	static int Receive(DevIntrf_t *, uint8_t *pData, int Len)
	{
		StreamState &stream = s_TestStreams[Index];

		stream.Reads++;
		Len = std::min(Len, std::min(stream.Chunk, (int)stream.Input.size()));
		for (int i = 0; i < Len; i++)
		{
			pData[i] = stream.Input.front();
			stream.Input.pop_front();
		}
		return Len;
	}
	static int Send(DevIntrf_t *, const uint8_t *pData, int Len)
	{
		StreamState &stream = s_TestStreams[Index];

		stream.Writes++;
		s_TestNow += s_TestWriteDuration;
		if (stream.BlockTx)
		{
			return 0;
		}
		Len = std::min(Len, stream.Chunk);
		stream.Output.insert(stream.Output.end(), pData, pData + Len);
		while (Header(stream.Output))
		{
			size_t header = Header(stream.Output);
			size_t size = 0;
			size_t factor = 1;

			for (size_t i = 1; i < header; i++)
			{
				size += (stream.Output[i] & 127U) * factor;
				factor *= 128U;
			}
			size += header;
			assert(size <= MQTT_PACKET_SIZE + 5U);
			if (stream.Output.size() < size)
			{
				break;
			}
			s_TestPackets.push_back({std::vector<uint8_t>(stream.Output.begin(),
				stream.Output.begin() + size), s_TestNow, s_TestOpens, Index});
			stream.Output.erase(stream.Output.begin(), stream.Output.begin() + size);
		}
		return Len;
	}
};

static DemoStream<0> s_TestStreamA;
static DemoStream<1> s_TestStreamB;
static DeviceIntrf *s_TestInterfaces[] = {&s_TestStreamA, &s_TestStreamB};

bool NetDemoInit()
{
	return s_TestInitWorks;
}

bool NetDemoStart(void (*pHandler)(uint32_t))
{
	s_TestStarts++;
	s_TestHandler = pHandler;
	return s_TestStartWorks;
}

uint32_t NetDemoNow()
{
	return s_TestNow;
}

RngEngine *NetDemoRng()
{
	return s_TestRngPointer;
}

const char *NetDemoMediumName()
{
	return "test stream";
}

const char *NetDemoSecurityName()
{
#if defined(NET_DEMO_MODEM_TLS) && NET_DEMO_MODEM_TLS
	return "TLS 1.2, CA/hostname verified";
#else
	return "plaintext";
#endif
}

bool NetDemoReady()
{
	return s_TestReady;
}

DeviceIntrf *NetDemoOpen(bool Datagram, const char *pHost, uint16_t Port)
{
	assert(!Datagram && pHost && strcmp(pHost, MQTT_DEMO_HOST) == 0);
	assert(Port == MQTT_DEMO_PORT && s_TestActive == -1 && s_TestReady);
	s_TestOpens++;
	s_TestNow += s_TestOpenDuration;
	if (!s_TestOpenWorks)
	{
		return nullptr;
	}
	s_TestActive = (int)s_TestSelected;
	Current().Input.clear();
	Current().Output.clear();
	Current().Connected = true;
	return s_TestInterfaces[s_TestSelected];
}

bool NetDemoConnected()
{
	return s_TestActive >= 0 && s_TestReady && s_TestStreams[s_TestActive].Connected;
}

void NetDemoClose()
{
	if (s_TestActive >= 0)
	{
		s_TestCloses++;
		s_TestNow += s_TestCloseDuration;
		s_TestStreams[s_TestActive].Connected = false;
		s_TestActive = -1;
	}
}

extern "C"
{
void AppRun(void)
{
}

SysLog_t *SysLogGet(void)
{
	return nullptr;
}

int SysLogPrintf(SysLog_t *, const char *pFormat, ...)
{
	char line[256];
	va_list args;
	int length;

	va_start(args, pFormat);
	length = vsnprintf(line, sizeof(line), pFormat, args);
	va_end(args);
	assert(length >= 0 && (size_t)length < sizeof(line));
	s_TestLog += line;
	return length;
}
}

static void Reset(uint32_t Now, int Chunk = 1024)
{
	s_Mqtt.Reset();
	NetDemoClose();
	s_TestNow = Now;
	s_TestOpenDuration = s_TestCloseDuration = s_TestWriteDuration = 0;
	s_TestOpens = s_TestCloses = s_TestStarts = s_TestSelected = 0;
	s_TestReady = s_TestOpenWorks = s_TestInitWorks = s_TestStartWorks = true;
	s_TestHandler = nullptr;
	for (StreamState &stream : s_TestStreams)
	{
		stream = {};
		stream.Chunk = Chunk;
	}
	atomic_flag_clear(&s_TestStreamA.Dev.bBusy);
	atomic_flag_clear(&s_TestStreamB.Dev.bBusy);
	s_TestPackets.clear();
	s_TestLog.clear();
	s_TestRng.Seed = 0;
	s_TestRng.Calls = 0;
	s_TestRng.Works = true;
	s_TestRngPointer = &s_TestRng;
	s_pIntrf = nullptr;
	s_Started = s_SubscribeSent = s_Subscribed = s_Waiting = false;
	s_EchoReceived = s_SessionChecked = s_RecoveryWaiting = false;
	s_Checked = s_Failures = s_Sequence = 0;
	s_SentAt = s_Retry = 0;
	memcpy(s_Payload, "IOsonata MQTT 00000000", sizeof(s_Payload));
	assert(CreateIdentity());
}

static void Pump(uint32_t Now, unsigned Times = 1)
{
	s_TestNow = Now;
	while (Times--)
	{
		unsigned reads = s_TestStreams[0].Reads + s_TestStreams[1].Reads;
		unsigned writes = s_TestStreams[0].Writes + s_TestStreams[1].Writes;

		Report(s_TestNow);
		assert(s_TestStreams[0].Reads + s_TestStreams[1].Reads - reads <= 1U);
		assert(s_TestStreams[0].Writes + s_TestStreams[1].Writes - writes <= 1U);
	}
}

static size_t Count(uint8_t Type)
{
	size_t count = 0;

	for (const WirePacket &packet : s_TestPackets)
	{
		if (packet.Bytes[0] >> 4 == Type)
		{
			count++;
		}
	}
	return count;
}

static WirePacket Packet(uint8_t Type, size_t Index)
{
	for (const WirePacket &packet : s_TestPackets)
	{
		if (packet.Bytes[0] >> 4 == Type)
		{
			if (Index == 0)
			{
				return packet;
			}
			Index--;
		}
	}
	assert(false);
	return {};
}

static uint16_t PacketId(const WirePacket &Packet)
{
	size_t offset = Header(Packet.Bytes);

	if (Packet.Bytes[0] >> 4 == 3)
	{
		offset += 2U + Word(Packet.Bytes, offset);
	}
	return Word(Packet.Bytes, offset);
}

static void Feed(const std::vector<uint8_t> &Bytes)
{
	Current().Input.insert(Current().Input.end(), Bytes.begin(), Bytes.end());
}

static void FeedAck(uint16_t Id)
{
	Feed({0x40, 2, (uint8_t)(Id >> 8), (uint8_t)Id});
}

static void FeedEcho(const WirePacket &Publish, uint16_t Id, bool Duplicate = false)
{
	std::vector<uint8_t> body(Publish.Bytes.begin() + Header(Publish.Bytes), Publish.Bytes.end());
	size_t id = 2U + Word(body, 0);
	size_t remaining = body.size();
	std::vector<uint8_t> bytes = {(uint8_t)(Duplicate ? 0x3a : 0x32)};

	body[id] = (uint8_t)(Id >> 8);
	body[id + 1U] = (uint8_t)Id;
	do
	{
		uint8_t digit = (uint8_t)(remaining & 127U);

		remaining >>= 7;
		bytes.push_back(digit | (remaining ? 128U : 0U));
	} while (remaining);
	bytes.insert(bytes.end(), body.begin(), body.end());
	Feed(bytes);
}


static void CheckConnect(const WirePacket &Connect)
{
	static const uint8_t protocol[] = {0, 4, 'M', 'Q', 'T', 'T', 4, 2, 0, 60};
	size_t header = Header(Connect.Bytes);

	assert(Connect.Bytes[0] == 0x10);
	assert(Connect.Bytes.size() >= header + sizeof(protocol) + 2U);
	assert(memcmp(Connect.Bytes.data() + header, protocol, sizeof(protocol)) == 0);
	assert(String(Connect.Bytes, header + sizeof(protocol)) == s_ClientId);
	assert(strlen(s_ClientId) == 22U);
	assert(Connect.Bytes.size() == header + sizeof(protocol) + 2U + strlen(s_ClientId));
}

static WirePacket Handshake(uint32_t Now, unsigned Session)
{
	size_t publications = Count(3);
	WirePacket subscribe;
	WirePacket publish;
	size_t header;
	uint16_t id;

	Pump(Now, 256);
	assert(Count(1) == Session && s_Started);
	assert(s_TestLog.find(std::string(", ") + NetDemoSecurityName() + ", client ") !=
		std::string::npos);
	CheckConnect(Packet(1, Session - 1U));
	Feed({0x20, 2, 0, 0});
	Pump(Now, 256);
	assert(Count(8) == Session && Count(3) == publications);
	subscribe = Packet(8, Session - 1U);
	header = Header(subscribe.Bytes);
	assert(subscribe.Bytes[0] == 0x82 && subscribe.Bytes.back() == 1);
	assert(String(subscribe.Bytes, header + 2U) == s_Topic);
	id = PacketId(subscribe);
	assert(id != 0);
	Feed({0x90, 3, (uint8_t)(id >> 8), (uint8_t)id, 1});
	Pump(Now, 256);
	assert(s_Subscribed && s_Waiting && !s_EchoReceived);
	assert(Count(3) == publications + 1U);
	publish = Packet(3, publications);
	assert(publish.Bytes[0] == 0x32 && publish.Session == Session);
	assert(String(publish.Bytes, Header(publish.Bytes)) == s_Topic);
	assert(PacketId(publish) != 0);
	return publish;
}

static WirePacket Begin(uint32_t Now, int Chunk = 1024)
{
	Reset(Now, Chunk);
	return Handshake(Now, 1);
}

static void Complete(const WirePacket &Publish, uint32_t Now, bool EchoFirst)
{
	uint32_t checked = s_Checked;
	size_t acks = Count(4);

	if (EchoFirst)
	{
		FeedEcho(Publish, 0x4567);
	}
	else
	{
		FeedAck(PacketId(Publish));
	}
	Pump(Now, 256);
	assert(s_Checked == checked && s_Waiting && s_Failures == 0);
	if (EchoFirst)
	{
		assert(s_EchoReceived && Count(4) == acks + 1U);
		assert(PacketId(Packet(4, acks)) == 0x4567);
		FeedAck(PacketId(Publish));
	}
	else
	{
		assert(!s_EchoReceived && Count(4) == acks);
		FeedEcho(Publish, 0x4567);
	}
	Pump(Now, 256);
	assert(s_Checked == checked + 1U && !s_Waiting && !s_Mqtt.Busy());
	assert(Count(4) == acks + 1U && PacketId(Packet(4, acks)) == 0x4567);
}

static void Ordering(uint32_t Now, int Chunk, bool EchoFirst)
{
	WirePacket first = Begin(Now, Chunk);
	WirePacket next;
	size_t acks;

	Complete(first, Now, EchoFirst);
	assert(s_TestRng.Calls == 1);
	// An already completed QoS 1 delivery is acknowledged without recounting.
	FeedEcho(first, 0x4567, true);
	Pump(Now, 256);
	assert(s_Checked == 1 && s_Failures == 0 && !s_Waiting && Count(3) == 1);
	Pump(Now + MQTT_DEMO_INTERVAL - 1U, 256);
	assert(Count(3) == 1);
	Pump(Now + MQTT_DEMO_INTERVAL, 256);
	assert(Count(3) == 2 && s_Waiting);
	next = Packet(3, 1);
	assert(next.Bytes != first.Bytes);
	// An old echo cannot satisfy the next publication, even with a new id.
	acks = Count(4);
	FeedEcho(first, 0x6789, true);
	Pump(Now + MQTT_DEMO_INTERVAL, 256);
	assert(s_Checked == 1 && s_Waiting && !s_EchoReceived && s_Failures == 0);
	assert(Count(4) == acks + 1U && PacketId(Packet(4, acks)) == 0x6789);
	Complete(next, Now + MQTT_DEMO_INTERVAL, !EchoFirst);
	assert(s_Checked == 2);
	assert(s_TestLog.find("MQTT echo rejected:") == std::string::npos);
}


static void Timeouts(void)
{
	const uint32_t now = 10000U;
	WirePacket publish;

	for (unsigned mode = 0; mode < 6; mode++)
	{
		if (mode < 3)
		{
			Reset(now);
			Current().BlockTx = mode == 2;
			Pump(now, 256);
			if (mode == 1)
			{
				Feed({0x20, 2, 0, 0});
				Pump(now, 256);
				assert(Count(8) == 1);
			}
		}
		else
		{
			publish = Begin(now);
			if (mode == 4)
			{
				FeedAck(PacketId(publish));
			}
			else if (mode == 5)
			{
				FeedEcho(publish, 0x3456);
			}
			Pump(now, 256);
		}
		Pump(now + MQTT_DEMO_TIMEOUT_MS - 1U, 256);
		assert(s_Started && s_Failures == 0 && s_Checked == 0);
		Pump(now + MQTT_DEMO_TIMEOUT_MS);
		assert(!s_Started && !NetDemoConnected() && s_TestCloses == 1);
		assert(s_Failures == 1 && s_Checked == 0 && s_RecoveryWaiting);
	}
}

static void Refused(void)
{
	const uint32_t now = 10000U;
	uint16_t id;

	for (uint8_t grant : {0, 0x80})
	{
		Reset(now);
		Pump(now, 256);
		Feed({0x20, 2, 0, 0});
		Pump(now, 256);
		id = PacketId(Packet(8, 0));
		Feed({0x90, 3, (uint8_t)(id >> 8), (uint8_t)id, grant});
		Pump(now, 256);
		assert(s_Failures == 1 && !s_Started && Count(3) == 0);
		assert(s_TestLog.find("did not grant QoS 1") != std::string::npos);
	}
	Reset(now);
	Pump(now, 256);
	Feed({0x20, 2, 0, 5});
	Pump(now, 256);
	assert(s_Failures == 1 && !s_Started && Count(8) == 0);
	assert(s_TestLog.find("result=5") != std::string::npos);
}


static void SequenceWrap(void)
{
	const uint32_t now = 10000U;
	WirePacket first;
	WirePacket next;

	Reset(now);
	s_Sequence = 0xfffffffeU;
	first = Handshake(now, 1);
	assert(std::string(first.Bytes.end() - 8, first.Bytes.end()) == "ffffffff");
	Complete(first, now, true);
	Pump(now + MQTT_DEMO_INTERVAL, 256);
	next = Packet(3, 1);
	assert(std::string(next.Bytes.end() - 8, next.Bytes.end()) == "00000000");
	FeedEcho(first, 0x1234, true);
	Pump(now + MQTT_DEMO_INTERVAL, 256);
	assert(s_Checked == 1 && s_Waiting && !s_EchoReceived && s_Failures == 0);
	Complete(next, now + MQTT_DEMO_INTERVAL, false);
}

static void IncomingAckBackpressure(void)
{
	const uint32_t now = 10000U;
	WirePacket publish = Begin(now);

	FeedAck(PacketId(publish));
	Pump(now, 256);
	Current().BlockTx = true;
	FeedEcho(publish, 0x3456);
	Pump(now, 256);
	// Both payload and broker PUBACK arrived, but our echo PUBACK is blocked.
	assert(s_Waiting && s_EchoReceived && s_Checked == 0 && s_Mqtt.Busy());
	assert(Count(4) == 0);
	Current().BlockTx = false;
	Pump(now, 256);
	assert(s_Checked == 1 && !s_Waiting && s_Failures == 0);
	assert(Count(4) == 1 && PacketId(Packet(4, 0)) == 0x3456);
}

static void PublicTopicMessages(void)
{
	const uint32_t now = 10000U;
	WirePacket publish = Begin(now);
	WirePacket unrelated = publish;

	FeedAck(PacketId(publish));
	Pump(now, 256);
	FeedEcho(publish, 0x1111);
	Current().Input.front() |= 1U;
	Pump(now, 256);
	assert(s_Checked == 0 && s_Waiting && !s_EchoReceived && s_Failures == 0);
	assert(Count(4) == 1 && PacketId(Packet(4, 0)) == 0x1111);
	unrelated.Bytes[Header(unrelated.Bytes) + 2U] = 'x';
	FeedEcho(unrelated, 0x2222);
	Pump(now, 256);
	assert(s_Checked == 0 && s_Waiting && !s_EchoReceived && s_Failures == 0);
	assert(Count(4) == 2 && PacketId(Packet(4, 1)) == 0x2222);
	FeedEcho(publish, 0x3333);
	Pump(now, 256);
	assert(s_Checked == 1 && !s_Waiting && s_Failures == 0);
	assert(Count(4) == 3 && PacketId(Packet(4, 2)) == 0x3333);
	assert(s_TestLog.find("MQTT echo rejected:") == std::string::npos);
}

static void RejectedMessages(void)
{
	struct RejectCase
	{
		const char *pReason;
		std::string Payload;
		const char *pHex;
		uint8_t Qos;
		bool Duplicate;
	};
	const uint32_t now = 10000U;
	std::string binary;

	for (unsigned i = 0; i < 70; i++)
	{
		binary.push_back((char)i);
	}
	const RejectCase cases[] = {
		{"sequence encoding", "IOsonata MQTT 0000000g",
			"494f736f6e617461204d515454203030303030303067", 1, false},
		{"sequence ahead", "IOsonata MQTT 00000002",
			"494f736f6e617461204d515454203030303030303032", 1, true},
		{"QoS", "IOsonata MQTT 00000001",
			"494f736f6e617461204d515454203030303030303031", 0, false},
		{"length", "IOsonata MQTT 0000000",
			"494f736f6e617461204d5154542030303030303030", 1, false},
		{"prefix", "XOsonata MQTT 00000001",
			"584f736f6e617461204d515454203030303030303031", 1, true},
		{"length", binary,
			"000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", 1, true},
		{"length", "", "", 1, false},
	};

	for (const RejectCase &test : cases)
	{
		WirePacket publish = Begin(now);
		size_t header = Header(publish.Bytes);
		size_t payload = header + 2U + Word(publish.Bytes, header) + 2U;
		char metadata[160];

		publish.Bytes.resize(payload);
		publish.Bytes.insert(publish.Bytes.end(), test.Payload.begin(), test.Payload.end());
		if (test.Qos == 0)
		{
			// This is a valid MQTT QoS 0 PUBLISH on our exact topic. The
			// application rejects it while the MQTT parser still reports no error.
			std::vector<uint8_t> body(publish.Bytes.begin() + header, publish.Bytes.end());
			size_t id = 2U + Word(body, 0);

			body.erase(body.begin() + id, body.begin() + id + 2U);
			assert(body.size() < 128U);
			Feed({0x30, (uint8_t)body.size()});
			Feed(body);
		}
		else
		{
			FeedEcho(publish, 0x4444, test.Duplicate);
		}
		Pump(now, 256);
		assert(s_Checked == 0 && s_Failures == 1 && !s_Started);
		assert(!s_Mqtt.Message());
		snprintf(metadata, sizeof(metadata),
			"MQTT echo rejected: %s qos=%u dup=%u len=%lu expected_seq=00000001\r\n",
			test.pReason, (unsigned)test.Qos, (unsigned)test.Duplicate,
			(unsigned long)test.Payload.size());
		std::string sample = "MQTT echo payload hex (" +
			std::to_string(std::min((size_t)32, test.Payload.size())) + "/" +
			std::to_string(test.Payload.size()) + " bytes): " + test.pHex + "\r\n";
		size_t rejected = s_TestLog.find(metadata);
		size_t snapshot = s_TestLog.find(sample);
		size_t failure = s_TestLog.find("MQTT FAIL: echo payload or QoS mismatch, "
			"mqtt_error=0 mqtt_result=0 checked=0 failures=1\r\n");

		// The snapshot must preserve the received bytes before Release/Reset
		// wipes them. Binary NUL, CR, LF and ESC are encoded and capped at 32 bytes.
		assert(rejected != std::string::npos && snapshot != std::string::npos);
		assert(failure != std::string::npos && rejected < snapshot && snapshot < failure);
		assert(s_TestLog.find('\x1b') == std::string::npos);
		if (strcmp(test.pReason, "sequence ahead") == 0)
		{
			size_t received = s_TestLog.find("MQTT echo sequence received=00000002\r\n");

			assert(received != std::string::npos && received < rejected);
		}
		else
		{
			assert(s_TestLog.find("MQTT echo sequence received=") == std::string::npos);
		}
	}
}

static void SwitchInterface(uint32_t Now, unsigned Kind, bool Idle)
{
	WirePacket first = Begin(Now, 7);
	WirePacket next;
	std::string client = s_ClientId;
	std::string topic = s_Topic;
	uint32_t finished;
	unsigned oldReads;
	unsigned oldWrites;
	uint32_t checked = Idle ? 1U : 0U;

	assert(s_pIntrf == &s_TestStreamA && first.Medium == 0);
	if (Idle)
	{
		Complete(first, Now, true);
	}
	s_TestNow = Now + 100U;
	s_TestCloseDuration = 700U;
	if (Kind == 0)
	{
		Current().Connected = false;
	}
	else if (Kind == 1)
	{
		s_TestReady = false;
	}
	else
	{
		// A setup callback can close an endpoint before the protocol poll.
		// Reopening the same C++ object must still begin a fresh MQTT session.
		NetDemoClose();
	}
	Pump(s_TestNow);
	finished = s_TestNow;
	assert(!s_Started && !NetDemoConnected() && s_TestCloses == 1);
	assert(!s_pIntrf && !s_Subscribed && !s_SubscribeSent && !s_Waiting);
	assert(!s_EchoReceived && s_Failures == 1 && s_Checked == checked);
	oldReads = s_TestStreams[0].Reads;
	oldWrites = s_TestStreams[0].Writes;
	FeedEcho(first, 0x1234);
	s_TestSelected = Kind == 2 ? 0U : 1U;
	s_TestReady = true;
	Pump(finished + MQTT_DEMO_RESTART_MS - 1U);
	assert(s_TestOpens == 1 && s_TestCloses == 1);
	finished += MQTT_DEMO_RESTART_MS;
	next = Handshake(finished, 2);
	assert(s_pIntrf == s_TestInterfaces[s_TestSelected]);
	assert(next.Medium == s_TestSelected && next.Bytes != first.Bytes);
	assert(s_Checked == checked && s_Failures == 1 && s_TestOpens == 2);
	assert(s_TestRng.Calls == 1 && client == s_ClientId && topic == s_Topic);
	if (Kind != 2)
	{
		assert(s_TestStreams[0].Reads == oldReads && s_TestStreams[0].Writes == oldWrites);
		assert(!s_TestStreams[0].Input.empty());
	}
	// Even if old application data reaches the new stream, it cannot satisfy
	// a publication from the new session. The identity survives, the sequence advances.
	FeedEcho(first, 0x1111, true);
	Pump(finished, 256);
	assert(s_Checked == checked && s_Waiting && !s_EchoReceived);
	FeedAck(PacketId(next));
	FeedEcho(next, 0x2222);
	Pump(finished, 256);
	assert(s_Checked == checked + 1U && s_Failures == 1 && !s_Waiting);
	assert(s_TestLog.find("MQTT QoS 1 echo checked " + std::to_string(checked + 1U)) !=
		std::string::npos);
}

static void OpenRetry()
{
	const uint32_t start = 0xfffff000U;
	uint32_t finished;

	Reset(start);
	s_TestOpenWorks = false;
	s_TestOpenDuration = 1700U;
	Pump(start);
	finished = s_TestNow;
	assert(!s_Started && s_TestOpens == 1 && s_Failures == 1);
	assert(s_TestPackets.empty() && Current().Reads == 0 && Current().Writes == 0);
	Pump(finished + MQTT_DEMO_RESTART_MS - 1U);
	assert(s_TestOpens == 1);
	Pump(finished + MQTT_DEMO_RESTART_MS);
	finished = s_TestNow;
	assert(s_TestOpens == 2 && s_Failures == 2 && !s_Started);
	assert(s_TestPackets.empty() && Current().Reads == 0 && Current().Writes == 0);
	s_TestOpenWorks = true;
	Pump(finished + MQTT_DEMO_RESTART_MS - 1U);
	assert(s_TestOpens == 2);
	Pump(finished + MQTT_DEMO_RESTART_MS);
	finished = s_TestNow;
	assert(s_TestOpens == 3 && s_Started && Count(1) == 1);
	// DNS/connect time must not consume the new MQTT CONNACK deadline.
	Pump(finished + MQTT_DEMO_TIMEOUT_MS - 1U, 256);
	assert(s_Started && s_Failures == 2);
	Pump(finished + MQTT_DEMO_TIMEOUT_MS);
	assert(!s_Started && s_Failures == 3);
}

static void WriteClock()
{
	const uint32_t now = 0xfffff000U;
	WirePacket first = Begin(now);

	Complete(first, now, false);
	Pump(now + MQTT_DEMO_INTERVAL);
	assert(s_Waiting && Count(3) == 1);
	s_TestWriteDuration = MQTT_DEMO_TIMEOUT_MS;
	Pump(now + MQTT_DEMO_INTERVAL);
	// The application must observe time spent inside a blocking transport write.
	assert(!s_Started && s_Failures == 1 && s_Checked == 1);
	assert(s_TestLog.find("PUBACK or echo timeout") != std::string::npos);
	assert(s_Retry == s_TestNow);
}

static void Startup()
{
	Reset(0);
	assert(strcmp(s_ClientId, "iosnet0001020304050607") == 0);
	assert(strcmp(s_Topic, "iosonata/demo/0001020304050607/echo") == 0);
	assert(CreateIdentity());
	assert(strcmp(s_ClientId, "iosnet08090a0b0c0d0e0f") == 0);
	s_TestRng.Works = false;
	assert(!CreateIdentity());
	assert(GenericMqttDemoMain() == 0 && s_TestStarts == 0 && s_TestOpens == 0);
	assert(s_TestLog.find("random identity initialization failed") != std::string::npos);
	s_TestRngPointer = nullptr;
	assert(!CreateIdentity());
	assert(GenericMqttDemoMain() == 0 && s_TestStarts == 0 && s_TestOpens == 0);
	Reset(0);
	s_TestInitWorks = false;
	assert(GenericMqttDemoMain() == 1 && s_TestStarts == 0);
	Reset(0);
	s_TestStartWorks = false;
	assert(GenericMqttDemoMain() == 0 && s_TestStarts == 1 && s_TestOpens == 0);
	assert(s_TestLog.find("startup failed") != std::string::npos);
	Reset(0);
	assert(GenericMqttDemoMain() == 0 && s_TestStarts == 1 && s_TestHandler == Report);
	s_TestReady = false;
	s_TestHandler(0);
	assert(s_TestOpens == 0);
	s_TestReady = true;
	s_TestHandler(0);
	assert(s_TestOpens == 1 && s_pIntrf == &s_TestStreamA);
}

int main()
{
	if (!MQTT_DEMO_HOST[0])
	{
		Reset(0);
		assert(GenericMqttDemoMain() == 0 && s_TestStarts == 0 && s_TestOpens == 0);
		assert(s_TestLog.find("Set MQTT_DEMO_HOST") != std::string::npos);
		puts("Generic MQTT application: empty host rejected before network start PASS");
		return 0;
	}
	Startup();
	for (int chunk : {1, 7, 1024})
	{
		Ordering(10000U, chunk, true);
		Ordering(0xfffff000U, chunk, false);
	}
	for (unsigned kind = 0; kind < 3; kind++)
	{
		SwitchInterface(10000U, kind, false);
		SwitchInterface(0xfffff000U, kind, true);
	}
	Timeouts();
	Refused();
	SequenceWrap();
	IncomingAckBackpressure();
	PublicTopicMessages();
	RejectedMessages();
	OpenRetry();
	WriteClock();
	puts("Generic MQTT application: interface switching, QoS 1, reconnect and deadlines PASS");
}


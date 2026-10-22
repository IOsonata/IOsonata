/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * LTE application checks using the real shared MQTT client and fake stream I/O.
 * No modem registers, cellular network or radio behavior are exercised here.
 */
#include <assert.h>
#include <stdio.h>
#include <algorithm>
#include <deque>
#include <string>
#include <vector>
#define main LteMqttDemoMain
#include "../../exemples/lte/lte_mqtt.cpp"
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
};

static DemoRng s_TestRng;
static uint32_t s_TestNow;
static uint32_t s_TestLteDuration;
static uint32_t s_TestSocketDuration;
static uint32_t s_TestCloseDuration;
static unsigned s_TestLteInits;
static unsigned s_TestSocketInits;
static unsigned s_TestSocketCloses;
static unsigned s_TestQueueCalls;
static unsigned s_TestReadCalls;
static unsigned s_TestWriteCalls;
static int s_TestChunk;
static bool s_TestRegistered;
static bool s_TestLteWorks;
static bool s_TestSocketWorks;
static bool s_TestQueueWorks;
static bool s_TestBlockTx;
static AppEvtHandler_t s_TestQueued;
static std::deque<uint8_t> s_TestInput;
static std::vector<uint8_t> s_TestOutput;
static std::vector<WirePacket> s_TestPackets;
static std::string s_TestLog;

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

static uint64_t Ticks(TimerDev_t *)
{
	return s_TestNow;
}

static bool ReceiveStart(DevIntrf_t *pDev, uint32_t)
{
	return ((SockIntrfDev_t *)pDev->pDevData)->Hdl >= 0;
}

static bool SendStart(DevIntrf_t *pDev, uint32_t)
{
	SockIntrfDev_t *socket = (SockIntrfDev_t *)pDev->pDevData;

	return socket->Hdl >= 0 && socket->bConnected;
}

static void TransferStop(DevIntrf_t *)
{
}

static int Receive(DevIntrf_t *, uint8_t *pData, int Len)
{
	s_TestReadCalls++;
	Len = std::min(Len, std::min(s_TestChunk, (int)s_TestInput.size()));
	for (int i = 0; i < Len; i++)
	{
		pData[i] = s_TestInput.front();
		s_TestInput.pop_front();
	}
	return Len;
}

static int Send(DevIntrf_t *, const uint8_t *pData, int Len)
{
	s_TestWriteCalls++;
	if (s_TestBlockTx)
	{
		return 0;
	}
	Len = std::min(Len, s_TestChunk);
	s_TestOutput.insert(s_TestOutput.end(), pData, pData + Len);
	while (Header(s_TestOutput))
	{
		size_t header = Header(s_TestOutput);
		size_t size = 0;
		size_t factor = 1;

		for (size_t i = 1; i < header; i++)
		{
			size += (s_TestOutput[i] & 127U) * factor;
			factor *= 128U;
		}
		size += header;
		assert(size <= MQTT_PACKET_SIZE + 5U);
		if (s_TestOutput.size() < size)
		{
			break;
		}
		s_TestPackets.push_back({std::vector<uint8_t>(s_TestOutput.begin(),
			s_TestOutput.begin() + size), s_TestNow, s_TestSocketInits});
		s_TestOutput.erase(s_TestOutput.begin(), s_TestOutput.begin() + size);
	}
	return Len;
}

CryptoRngNrf *CryptoRngNrfInstance(void)
{
	return nullptr;
}

extern "C"
{
bool LteInit(const LteCfg_t *pCfg)
{
	assert(!pCfg->bPsm && !pCfg->bRai && pCfg->EdrxCycle == 0);
	s_TestLteInits++;
	s_TestNow += s_TestLteDuration;
	return s_TestLteWorks;
}

bool LteRegistered(void)
{
	return s_TestRegistered;
}

bool LteGetInfo(LTE_INFO, char *, int)
{
	return false;
}

bool SockIntrfInit(SockIntrfDev_t *pDev, const SockIntrfCfg_t *pCfg)
{
	DevIntrf_t *intrf = &pDev->DevIntrf;

	assert(pCfg->Proto == SOCKINTRF_PROTO_TCP && pCfg->EvtCB == nullptr);
	assert(pCfg->Timeout == 100U);
	s_TestSocketInits++;
	s_TestNow += s_TestSocketDuration;
	s_TestInput.clear();
	s_TestOutput.clear();
	intrf->pDevData = pDev;
	intrf->StartRx = ReceiveStart;
	intrf->StartTx = SendStart;
	intrf->StopRx = intrf->StopTx = TransferStop;
	intrf->RxData = Receive;
	intrf->TxData = Send;
	intrf->EnCnt = 1;
	intrf->MaxRetry = 0;
	atomic_flag_clear(&intrf->bBusy);
	pDev->Hdl = s_TestSocketWorks ? 1 : -1;
	pDev->bConnected = s_TestSocketWorks;
	return s_TestSocketWorks;
}

void SockIntrfClose(SockIntrfDev_t *pDev)
{
	if (pDev->Hdl >= 0)
	{
		s_TestSocketCloses++;
		s_TestNow += s_TestCloseDuration;
	}
	pDev->Hdl = -1;
	pDev->bConnected = false;
}

bool AppEvtHandlerInit(uint8_t *, size_t)
{
	return true;
}

bool AppEvtHandlerQue(uint32_t, void *, AppEvtHandler_t Handler)
{
	s_TestQueueCalls++;
	if (s_TestQueueWorks)
	{
		assert(s_TestQueued == nullptr);
		s_TestQueued = Handler;
	}
	return s_TestQueueWorks;
}

void AppRun(void)
{
}

bool TimerInit(TimerDev_t *pDev, const TimerCfg_t *)
{
	pDev->Freq = 1000;
	pDev->GetTickCount = Ticks;
	return true;
}

bool UARTInit(UARTDev_t *, const UARTCfg_t *)
{
	return true;
}

void UARTSetCtrlLineState(UARTDev_t *, uint32_t)
{
}

int CFifoAvail(hCFifo_t)
{
	return 0;
}

SysLog_t *SysLogGet(void)
{
	return nullptr;
}

bool SysLogInit(SysLog_t *, const SysLogCfg_t *, DevIntrf_t *, uint32_t,
	TimerDev_t *, uint32_t)
{
	return true;
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
	s_TestInput.insert(s_TestInput.end(), Bytes.begin(), Bytes.end());
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

static void Reset(uint32_t Now, int Chunk = 1024)
{
	s_Mqtt.Reset();
	s_Socket.Close();
	s_TestNow = Now;
	s_TestLteDuration = s_TestSocketDuration = s_TestCloseDuration = 0;
	s_TestLteInits = s_TestSocketInits = s_TestSocketCloses = s_TestQueueCalls = 0;
	s_TestReadCalls = s_TestWriteCalls = 0;
	s_TestChunk = Chunk;
	s_TestRegistered = s_TestLteWorks = s_TestSocketWorks = s_TestQueueWorks = true;
	s_TestBlockTx = false;
	s_TestQueued = nullptr;
	s_TestInput.clear();
	s_TestOutput.clear();
	s_TestPackets.clear();
	s_TestLog.clear();
	s_TestRng.Seed = 0;
	s_TestRng.Calls = 0;
	s_TestRng.Works = true;
	s_pRng = &s_TestRng;
	s_Started = s_SubscribeSent = s_Subscribed = s_Waiting = false;
	s_EchoReceived = s_SessionChecked = s_RecoveryWaiting = s_ReportQueued = false;
	s_LteStarted = true;
	s_Checked = s_Failures = s_Sequence = 0;
	s_SentAt = s_Retry = s_LteRetry = 0;
	memcpy(s_Payload, "IOsonata LTE MQTT 00000000", sizeof(s_Payload));
	assert(CreateIdentity());
	TimerInit(&s_Timer, &s_TimerCfg);
}

static void Pump(uint32_t Now, unsigned Times = 1)
{
	s_TestNow = Now;
	while (Times--)
	{
		unsigned reads = s_TestReadCalls;
		unsigned writes = s_TestWriteCalls;

		Report(0, nullptr);
		// One foreground turn must not spin on an empty or blocked stream.
		assert(s_TestReadCalls - reads <= 1U && s_TestWriteCalls - writes <= 1U);
	}
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
}

static void Recovery(uint32_t Now, unsigned Kind)
{
	WirePacket first = Begin(Now, 7);
	WirePacket next;
	std::string client = s_ClientId;
	std::string topic = s_Topic;
	LteStatus_t status = {};
	uint32_t finished;

	s_TestNow = Now + 100U;
	s_TestCloseDuration = 700U;
	status.RegStat = LTE_REG_SEARCHING;
	if (Kind == 0)
	{
		((SockIntrfDev_t *)s_Socket)->bConnected = false;
		Pump(s_TestNow);
	}
	else if (Kind == 1)
	{
		s_TestRegistered = false;
		LteEvent(LTE_EVT_UNREGISTERED, &status);
	}
	else if (Kind == 2)
	{
		// Registration can also be noticed by the normal foreground poll.
		s_TestRegistered = false;
		Pump(s_TestNow);
	}
	else
	{
		s_TestRegistered = false;
		LteEvent(LTE_EVT_MODEM_FAULT, &status);
	}
	finished = s_TestNow;
	assert(!s_Started && !s_Socket.Connected() && s_TestSocketCloses == 1);
	assert(!s_Subscribed && !s_SubscribeSent && !s_Waiting && !s_EchoReceived);
	assert(s_Failures == 1 && s_Checked == 0);
	Pump(finished + LTE_MQTT_RESTART_MS - 1U);
	assert(s_TestSocketInits == 1 && s_TestLteInits == 0);
	if (Kind == 3)
	{
		s_TestLteDuration = 900U;
		Pump(finished + LTE_MQTT_RESTART_MS);
		assert(s_TestLteInits == 1 && s_LteStarted && s_TestSocketInits == 1);
		finished = s_TestNow;
		Pump(finished);
		assert(s_TestSocketInits == 1);
	}
	else
	{
		finished += LTE_MQTT_RESTART_MS;
	}
	s_TestRegistered = true;
	next = Handshake(finished, 2);
	assert(s_Failures == 1 && s_TestSocketInits == 2 && s_Checked == 0);
	assert(s_TestRng.Calls == 1 && client == s_ClientId && topic == s_Topic);
	assert(next.Bytes != first.Bytes);
	FeedEcho(first, 0x1111, true);
	Pump(finished, 256);
	assert(s_Checked == 0 && s_Waiting && !s_EchoReceived);
	FeedAck(PacketId(next));
	FeedEcho(next, 0x2222);
	Pump(finished, 256);
	assert(s_Checked == 1 && s_Failures == 1 && !s_Waiting);
	assert(s_TestLog.find("MQTT QoS 1 echo checked 1") != std::string::npos);
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
			s_TestBlockTx = mode == 2;
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
		Pump(now + LTE_MQTT_TIMEOUT_MS - 1U, 256);
		assert(s_Started && s_Failures == 0 && s_Checked == 0);
		Pump(now + LTE_MQTT_TIMEOUT_MS);
		assert(!s_Started && !s_Socket.Connected() && s_TestSocketCloses == 1);
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

static void SocketRetry(void)
{
	const uint32_t start = 0xfffff000U;
	uint32_t finished;

	Reset(start);
	s_TestSocketWorks = false;
	s_TestSocketDuration = 1700U;
	Pump(start);
	finished = s_TestNow;
	assert(!s_Started && s_TestSocketInits == 1 && s_Failures == 1);
	Pump(finished + LTE_MQTT_RESTART_MS - 1U);
	assert(s_TestSocketInits == 1);
	Pump(finished + LTE_MQTT_RESTART_MS);
	finished = s_TestNow;
	assert(s_TestSocketInits == 2 && s_Failures == 2 && !s_Started);
	s_TestSocketWorks = true;
	Pump(finished + LTE_MQTT_RESTART_MS - 1U);
	assert(s_TestSocketInits == 2);
	Pump(finished + LTE_MQTT_RESTART_MS);
	finished = s_TestNow;
	assert(s_TestSocketInits == 3 && s_Started && Count(1) == 1);
	// MQTT's CONNACK deadline starts after DNS/connect has returned.
	Pump(finished + LTE_MQTT_TIMEOUT_MS - 1U, 256);
	assert(s_Started && s_Failures == 2);
	Pump(finished + LTE_MQTT_TIMEOUT_MS);
	assert(!s_Started && s_Failures == 3);
}

static void ModemRetry(void)
{
	const uint32_t start = 0xfffff000U;
	uint32_t finished;

	Reset(start);
	s_TestRegistered = false;
	s_TestLteWorks = false;
	s_TestLteDuration = 700U;
	StartLte();
	finished = s_TestNow;
	assert(!s_LteStarted && s_TestLteInits == 1);
	Pump(finished + LTE_MQTT_RESTART_MS - 1U);
	assert(s_TestLteInits == 1);
	Pump(finished + LTE_MQTT_RESTART_MS);
	finished = s_TestNow;
	assert(s_TestLteInits == 2 && !s_LteStarted);
	s_TestLteWorks = true;
	Pump(finished + LTE_MQTT_RESTART_MS - 1U);
	assert(s_TestLteInits == 2);
	Pump(finished + LTE_MQTT_RESTART_MS);
	assert(s_TestLteInits == 3 && s_LteStarted);
	Pump(s_TestNow + 20U);
	assert(s_TestSocketInits == 0);
}

static void QueueRecovery(void)
{
	AppEvtHandler_t queued;

	Reset(10000U);
	s_TestQueueWorks = false;
	ReportTimer(&s_Timer, LTE_MQTT_TRIG_REPORT, nullptr);
	assert(!s_ReportQueued && s_TestQueueCalls == 1);
	s_TestQueueWorks = true;
	ReportTimer(&s_Timer, LTE_MQTT_TRIG_REPORT, nullptr);
	assert(s_ReportQueued && s_TestQueued);
	for (unsigned i = 0; i < 100; i++)
	{
		ReportTimer(&s_Timer, LTE_MQTT_TRIG_REPORT, nullptr);
	}
	assert(s_TestQueueCalls == 2 && s_TestSocketInits == 0);
	assert(s_TestReadCalls == 0 && s_TestWriteCalls == 0 && s_TestLteInits == 0);
	queued = s_TestQueued;
	s_TestQueued = nullptr;
	queued(0, nullptr);
	assert(!s_ReportQueued && s_TestSocketInits == 1 && Count(1) == 1);
}

static void Identity(void)
{
	std::string first;
	std::string topic;

	Reset(0);
	assert(strcmp(s_ClientId, "ioslte0001020304050607") == 0);
	assert(strcmp(s_Topic, "iosonata/lte/0001020304050607/echo") == 0);
	first = s_ClientId;
	topic = s_Topic;
	assert(CreateIdentity() && first != s_ClientId && topic != s_Topic);
	assert(strcmp(s_ClientId, "ioslte08090a0b0c0d0e0f") == 0);
	s_TestRng.Works = false;
	assert(!CreateIdentity());
	s_pRng = nullptr;
	assert(!CreateIdentity());
	// The startup path must stop before modem or socket activity without RNG.
	assert(LteMqttDemoMain() == 0);
	assert(s_TestLteInits == 0 && s_TestSocketInits == 0);
	assert(s_TestLog.find("random identity initialization failed") != std::string::npos);
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
	s_TestBlockTx = true;
	FeedEcho(publish, 0x3456);
	Pump(now, 256);
	// Both payload and broker PUBACK arrived, but our echo PUBACK is blocked.
	assert(s_Waiting && s_EchoReceived && s_Checked == 0 && s_Mqtt.Busy());
	assert(Count(4) == 0);
	s_TestBlockTx = false;
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
	s_TestInput.front() |= 1U;
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

	for (unsigned mode = 0; mode < 3; mode++)
	{
		publish = Begin(now);
		if (mode == 2)
		{
			std::vector<uint8_t> body(publish.Bytes.begin() + Header(publish.Bytes),
				publish.Bytes.end());
			size_t id = 2U + Word(body, 0);

			body.erase(body.begin() + id, body.begin() + id + 2U);
			assert(body.size() < 128U);
			Feed({0x30, (uint8_t)body.size()});
			Feed(body);
		}
		else
		{
			// A malformed suffix or a future sequence must never count as ours.
			publish.Bytes.back() = mode == 0 ? 'g' : '2';
			FeedEcho(publish, 0x4444);
		}
		Pump(now, 256);
		assert(s_Checked == 0 && s_Failures == 1 && !s_Started);
		assert(s_TestLog.find("echo payload or QoS mismatch") != std::string::npos);
	}
}

int main()
{
	Identity();
	for (int chunk : {1, 7, 1024})
	{
		Ordering(10000U, chunk, true);
		Ordering(0xfffff000U, chunk, false);
	}
	for (unsigned kind = 0; kind < 4; kind++)
	{
		Recovery(10000U, kind);
		Recovery(0xfffff000U, kind);
	}
	Timeouts();
	Refused();
	SequenceWrap();
	IncomingAckBackpressure();
	PublicTopicMessages();
	SocketRetry();
	ModemRetry();
	QueueRecovery();
	puts("LTE MQTT application: QoS 1 ordering, reconnect, identity, timeouts and deferred pump PASS");
}

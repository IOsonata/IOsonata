/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * The actual generic application on two independent datagram interfaces.
 * Host containers simulate packets; no radio, modem or Ethernet hardware runs.
 */
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <deque>
#include <string>
#include <vector>
#define main CoapGenericMain
#include "../../exemples/net/coap_generic.cpp"
#undef main

class DemoRng : public RngEngine
{
public:
	uint8_t Draw = 0;

	bool Enable() override
	{
		return true;
	}
	void Disable() override
	{
	}
	void Reset() override
	{
	}
	CRYPTO_STATUS Random(uint8_t *pData, size_t Len) override
	{
		memset(pData, ++Draw, Len);
		if (Len == COAP_TOKEN_SIZE + 2U)
		{
			pData[COAP_TOKEN_SIZE] = pData[COAP_TOKEN_SIZE + 1U] = 0;
		}
		return CRYPTO_STATUS_OK;
	}
};

static bool TransferStart(DevIntrf_t *, uint32_t)
{
	return true;
}

static void TransferStop(DevIntrf_t *)
{
}

// This interface receives from a queue of complete packets.
class QueueDatagram : public DeviceIntrf
{
public:
	DevIntrf_t Dev = {};
	std::deque<std::vector<uint8_t>> In;
	std::vector<std::vector<uint8_t>> Out;
	unsigned Reads = 0;
	int Capacity = 0;
	bool Block = false;
	bool WriteFault = false;

	QueueDatagram()
	{
		Dev.pDevData = this;
		Dev.Type = DEVINTRF_TYPE_ETH;
		Dev.EnCnt = 1;
		Dev.StartRx = Dev.StartTx = TransferStart;
		Dev.StopRx = Dev.StopTx = TransferStop;
		Dev.RxData = ReadPacket;
		Dev.TxData = WritePacket;
		atomic_flag_clear(&Dev.bBusy);
	}
	operator DevIntrf_t *() override
	{
		return &Dev;
	}
	uint32_t Rate(uint32_t Value) override
	{
		return Value;
	}
	uint32_t Rate() override
	{
		return 0;
	}
	static int ReadPacket(DevIntrf_t *pDev, uint8_t *pData, int Len)
	{
		QueueDatagram *self = (QueueDatagram *)pDev->pDevData;
		size_t count;

		self->Reads++;
		self->Capacity = Len;
		if (self->In.empty())
		{
			return 0;
		}
		count = self->In.front().size();
		if (count > (size_t)Len)
		{
			count = (size_t)Len;
		}
		if (count)
		{
			memcpy(pData, self->In.front().data(), count);
		}
		self->In.pop_front();
		return (int)count;
	}
	static int WritePacket(DevIntrf_t *pDev, const uint8_t *pData, int Len)
	{
		QueueDatagram *self = (QueueDatagram *)pDev->pDevData;

		if (self->WriteFault)
		{
			return -1;
		}
		if (self->Block)
		{
			return 0;
		}
		self->Out.emplace_back(pData, pData + Len);
		return Len;
	}
};

// A different object layout and function table, with a one-packet mailbox.
// Its C handle is deliberately not its object's first field.
class MailboxDatagram : public DeviceIntrf
{
public:
	std::vector<uint8_t> Packet;
	std::vector<std::vector<uint8_t>> Out;
	bool Full = false;
	unsigned Reads = 0;
	int Capacity = 0;
	DevIntrf_t Dev = {};

	MailboxDatagram()
	{
		Dev.pDevData = this;
		Dev.Type = DEVINTRF_TYPE_WIFI;
		Dev.EnCnt = 1;
		Dev.StartRx = Dev.StartTx = TransferStart;
		Dev.StopRx = Dev.StopTx = TransferStop;
		Dev.RxData = ReceivePacket;
		Dev.TxData = SendPacket;
		atomic_flag_clear(&Dev.bBusy);
	}
	operator DevIntrf_t *() override
	{
		return &Dev;
	}
	uint32_t Rate(uint32_t Value) override
	{
		return Value;
	}
	uint32_t Rate() override
	{
		return 0;
	}
	static int ReceivePacket(DevIntrf_t *pDev, uint8_t *pData, int Len)
	{
		MailboxDatagram *self = (MailboxDatagram *)pDev->pDevData;
		size_t count;

		self->Reads++;
		self->Capacity = Len;
		if (!self->Full)
		{
			return 0;
		}
		count = self->Packet.size() < (size_t)Len ? self->Packet.size() : (size_t)Len;
		if (count)
		{
			memcpy(pData, self->Packet.data(), count);
		}
		self->Packet.clear();
		self->Full = false;
		return (int)count;
	}
	static int SendPacket(DevIntrf_t *pDev, const uint8_t *pData, int Len)
	{
		MailboxDatagram *self = (MailboxDatagram *)pDev->pDevData;

		self->Out.emplace_back(pData, pData + Len);
		return Len;
	}
};

static QueueDatagram s_Queue;
static MailboxDatagram s_Mailbox;
static DemoRng s_TestRng;
static DeviceIntrf *s_Selected;
static DeviceIntrf *s_Active;
static uint32_t s_TestNow;
static uint32_t s_OpenDuration;
static uint32_t s_CloseDuration;
static unsigned s_OpenCalls;
static unsigned s_CloseCalls;
static unsigned s_StartCalls;
static unsigned s_InitCalls;
static unsigned s_RunCalls;
static bool s_Ready;
static bool s_Connected;
static bool s_OpenWorks;
static bool s_InitWorks;
static void (*s_Handler)(uint32_t);
static std::string s_Log;

bool NetDemoInit()
{
	s_InitCalls++;
	return s_InitWorks;
}

bool NetDemoStart(void (*pHandler)(uint32_t))
{
	s_StartCalls++;
	s_Handler = pHandler;
	return true;
}

uint32_t NetDemoNow()
{
	return s_TestNow;
}

RngEngine *NetDemoRng()
{
	return &s_TestRng;
}

bool NetDemoReady()
{
	return s_Ready;
}

DeviceIntrf *NetDemoOpen(bool Datagram, const char *pHost, uint16_t Port)
{
	const char *host = COAP_DEMO_HOST;

	assert(Datagram && pHost && host && strcmp(pHost, host) == 0 && Port == COAP_DEMO_PORT);
	assert(!s_Active && s_Ready);
	s_OpenCalls++;
	s_TestNow += s_OpenDuration;
	if (!s_OpenWorks)
	{
		return nullptr;
	}
	s_Active = s_Selected;
	s_Connected = true;
	return s_Active;
}

bool NetDemoConnected()
{
	return s_Active && s_Connected;
}

void NetDemoClose()
{
	s_CloseCalls++;
	s_TestNow += s_CloseDuration;
	s_Active = nullptr;
	s_Connected = false;
}

extern "C"
{
void AppRun()
{
	s_RunCalls++;
}

SysLog_t *SysLogGet()
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
	s_Log += line;
	return length;
}
}

static void Reset(uint32_t Now)
{
	s_Coap.Reset();
	s_pIntrf = nullptr;
	s_pRng = &s_TestRng;
	s_TestRng.Draw = 0;
	s_Queue.In.clear();
	s_Queue.Out.clear();
	s_Queue.Block = s_Queue.WriteFault = false;
	s_Queue.Reads = 0;
	s_Queue.Capacity = 0;
	s_Queue.Dev.MaxRetry = 0;
	atomic_flag_clear(&s_Queue.Dev.bBusy);
	s_Mailbox.Packet.clear();
	s_Mailbox.Out.clear();
	s_Mailbox.Full = false;
	s_Mailbox.Reads = 0;
	s_Mailbox.Capacity = 0;
	atomic_flag_clear(&s_Mailbox.Dev.bBusy);
	s_Selected = &s_Queue;
	s_Active = nullptr;
	s_TestNow = Now;
	s_OpenDuration = s_CloseDuration = 0;
	s_OpenCalls = s_CloseCalls = s_StartCalls = s_InitCalls = s_RunCalls = 0;
	s_Ready = s_OpenWorks = s_InitWorks = true;
	s_Connected = false;
	s_Handler = nullptr;
	s_Log.clear();
	s_Started = s_Waiting = s_RecoveryWaiting = false;
	s_Checked = s_Failures = s_Cycle = 0;
	s_Retry = s_ProbeStarted = s_ProbeDelay = 0;
	s_Step = s_PayloadLen = 0;
	s_LastRequest = Now - COAP_DEMO_INTERVAL;
}

static void Pump(uint32_t Now)
{
	s_TestNow = Now;
	Report(Now);
}

static std::vector<std::vector<uint8_t>> &Packets()
{
	return s_Selected == &s_Queue ? s_Queue.Out : s_Mailbox.Out;
}

static void Deliver(const std::vector<uint8_t> &Packet)
{
	if (s_Selected == &s_Queue)
	{
		s_Queue.In.push_back(Packet);
	}
	else
	{
		assert(!s_Mailbox.Full);
		s_Mailbox.Packet = Packet;
		s_Mailbox.Full = true;
	}
}

static std::vector<uint8_t> FirstPayload()
{
	const char *text = COAP_DEMO_PUBLIC_TEST ? "public response" : "IOsonata CoAP\n";

	return std::vector<uint8_t>(text, text + strlen(text));
}

static uint16_t Mid(const std::vector<uint8_t> &Packet)
{
	return (uint16_t)((Packet[2] << 8) | Packet[3]);
}

// Frame responses independently of the protocol's encoder.
static std::vector<uint8_t> Reply(const std::vector<uint8_t> &Request,
	uint8_t Code, const std::vector<uint8_t> &Payload,
	COAP_TYPE Type = COAP_ACK, uint16_t MessageId = 700)
{
	uint16_t id = Type == COAP_ACK ? Mid(Request) : MessageId;
	std::vector<uint8_t> packet = {
		(uint8_t)(0x48U | ((unsigned)Type << 4)), Code,
		(uint8_t)(id >> 8), (uint8_t)id
	};

	assert(Request.size() >= 12U && (Request[0] & 15U) == 8U);
	packet.insert(packet.end(), Request.begin() + 4, Request.begin() + 12);
	if (!Payload.empty())
	{
		packet.push_back(0xff);
		packet.insert(packet.end(), Payload.begin(), Payload.end());
	}
	return packet;
}

static std::vector<uint8_t> Begin(uint32_t Now)
{
	Pump(Now);
	Pump(s_TestNow);
	assert(s_Started && s_Waiting && !Packets().empty());
	return Packets().back();
}

static void CheckFirstRequest(const std::vector<uint8_t> &Packet)
{
	CoapMessage_t request = {};
	size_t path = COAP_DEMO_PUBLIC_TEST ? 1U : 0U;
	const char *expected = COAP_DEMO_PUBLIC_TEST ? COAP_DEMO_RESOURCE : "hello";

	assert(CoapDecode(&request, Packet.data(), Packet.size()) == COAP_ERROR_NONE);
	assert(request.Type == COAP_CON && request.Code == COAP_GET);
	assert(request.PayloadLen == 0 && request.OptionCount == path + 1U);
	if (COAP_DEMO_PUBLIC_TEST)
	{
		assert(request.Options[0].Number == COAP_OPTION_URI_HOST);
		assert(request.Options[0].Len == strlen(COAP_DEMO_HOST));
		assert(memcmp(request.Options[0].pValue, COAP_DEMO_HOST,
			request.Options[0].Len) == 0);
	}
	assert(request.Options[path].Number == COAP_OPTION_URI_PATH);
	assert(request.Options[path].Len == strlen(expected));
	assert(memcmp(request.Options[path].pValue, expected, strlen(expected)) == 0);
}

static void CompletedSwitch()
{
	const uint32_t now = 10000U;
	std::vector<uint8_t> oldRequest;
	std::vector<uint8_t> newRequest;
	unsigned reads;
	size_t writes;

	Reset(now);
	oldRequest = Begin(now);
	CheckFirstRequest(oldRequest);
	Deliver(Reply(oldRequest, COAP_CONTENT, FirstPayload()));
	Pump(now + 20U);
	assert(s_Checked == 1 && !s_Waiting);
	s_Ready = false;
	Pump(now + 40U);
	assert(!s_Started && !s_pIntrf && !s_Active && s_CloseCalls == 1);
	s_Queue.In.push_back(Reply(oldRequest, COAP_CONTENT, FirstPayload()));
	reads = s_Queue.Reads;
	writes = s_Queue.Out.size();
	s_Selected = &s_Mailbox;
	s_Ready = true;
	newRequest = Begin(now + COAP_DEMO_INTERVAL);
	CheckFirstRequest(newRequest);
	assert(s_pIntrf == &s_Mailbox && s_OpenCalls == 2);
	assert(memcmp(oldRequest.data() + 4, newRequest.data() + 4, 8U) != 0);
	Deliver(Reply(oldRequest, COAP_CONTENT, FirstPayload()));
	Pump(now + COAP_DEMO_INTERVAL + 20U);
	assert(s_Checked == 1 && s_Waiting);
	Deliver(Reply(newRequest, COAP_CONTENT, FirstPayload()));
	Pump(now + COAP_DEMO_INTERVAL + 40U);
	assert(s_Checked == 2 && !s_Waiting);
	assert(s_Queue.Reads == reads && s_Queue.Out.size() == writes);
	assert(s_Queue.In.size() == 1 && s_Mailbox.Capacity == COAP_MESSAGE_SIZE + 1U);
}

static void OutstandingSwitch(uint32_t Now, bool SameObject)
{
	std::vector<uint8_t> oldRequest;
	std::vector<uint8_t> newRequest;
	uint32_t debt;
	unsigned reads;

	Reset(Now);
	if (!COAP_DEMO_PUBLIC_TEST)
	{
		s_Step = 2;
		s_PayloadLen = sizeof(s_Payload);
		memset(s_Payload, 0xa5, s_PayloadLen);
	}
	oldRequest = Begin(Now);
	debt = (uint32_t)oldRequest.size() * 1000U;
	assert(s_Coap.RetryAfter(Now) == debt);
	s_Ready = false;
	Pump(Now + 100U);
	assert(!s_Started && s_ProbeDelay == debt - 100U && s_CloseCalls == 1);
	Pump(Now + 1000U);
	Pump(Now + 2000U);
	assert(s_CloseCalls == 1 && s_ProbeDelay == debt - 100U);
	// A second explicit detach after Reset must retain the remaining wait.
	CoapDown(Now + 2000U, "repeated setup loss", true);
	assert(s_ProbeDelay == debt - 2000U);
	reads = s_Queue.Reads;
	s_Selected = SameObject ? (DeviceIntrf *)&s_Queue : (DeviceIntrf *)&s_Mailbox;
	s_Ready = true;
	Pump(Now + debt - 1U);
	assert(s_OpenCalls == 1 && !s_Started);
	newRequest = Begin(Now + debt);
	assert(s_OpenCalls == 2 && s_Step == 0 && s_Waiting);
	CheckFirstRequest(newRequest);
	assert(memcmp(oldRequest.data() + 4, newRequest.data() + 4, 8U) != 0);
	Deliver(Reply(oldRequest, COAP_CONTENT, FirstPayload(), COAP_CON, 701));
	Pump(Now + debt + 20U);
	assert(s_Checked == 0 && s_Waiting);
	assert(Packets().back() == std::vector<uint8_t>({0x70, 0, 2, 189}));
	Deliver(Reply(newRequest, COAP_CONTENT, FirstPayload()));
	Pump(Now + debt + 40U);
	assert(s_Checked == 1 && !s_Waiting);
	if (!SameObject)
	{
		assert(s_Queue.Reads == reads && s_Queue.Out.size() == 1);
	}
}

static void LiveTimeout()
{
	const uint32_t now = 10000U;
	std::vector<uint8_t> request;
	uint32_t debt;
	uint32_t deadline = COAP_DEMO_PUBLIC_TEST ? 62000U : 95000U;
	size_t count;

	Reset(now);
	if (!COAP_DEMO_PUBLIC_TEST)
	{
		s_Step = 2;
		s_PayloadLen = sizeof(s_Payload);
		memset(s_Payload, 0x5a, s_PayloadLen);
	}
	request = Begin(now);
	debt = (uint32_t)request.size() * 1000U;
	if (COAP_DEMO_PUBLIC_TEST)
	{
		for (uint32_t age : {2000U, 6000U, 14000U, 30000U})
		{
			Pump(now + age);
			assert(s_Queue.Out.back() == request);
		}
		debt *= 5U;
	}
	count = s_Queue.Out.size();
	Pump(now + deadline);
	assert(s_Failures == 1 && s_Started && s_Active == &s_Queue);
	assert(s_CloseCalls == 0 && s_OpenCalls == 1 && !s_Coap.Busy());
	assert(s_ProbeDelay == debt - deadline && !s_Waiting);
	Pump(now + debt - 1U);
	assert(s_Queue.Out.size() == count && !s_Waiting);
	Begin(now + debt);
	assert(s_OpenCalls == 1 && s_CloseCalls == 0);
	assert(Mid(s_Queue.Out.back()) != Mid(request));
}

static void ControlBackpressure()
{
	const uint32_t now = 10000U;
	std::vector<uint8_t> request;
	std::vector<uint8_t> response;
	unsigned reads;

	Reset(now);
	request = Begin(now);
	response = Reply(request, COAP_CONTENT, FirstPayload(), COAP_CON, 900);
	s_Queue.Block = true;
	Deliver(response);
	Pump(now + 20U);
	assert(s_Checked == 1 && !s_Waiting && s_Coap.Busy());
	assert(s_Queue.Out.size() == 1);
	Deliver(response);
	reads = s_Queue.Reads;
	Pump(now + COAP_DEMO_INTERVAL);
	assert(s_Queue.Reads == reads && !s_Waiting && s_Coap.Busy());
	s_Queue.Block = false;
	Pump(now + COAP_DEMO_INTERVAL + 20U);
	assert(s_Queue.Out[1] == std::vector<uint8_t>({0x60, 0, 3, 132}));
	Pump(now + COAP_DEMO_INTERVAL + 40U);
	assert(s_Checked == 1 && s_Queue.Out[2] == s_Queue.Out[1]);
	Pump(now + COAP_DEMO_INTERVAL + 60U);
	assert(s_Waiting && s_Queue.Out.size() == 4);
	assert(s_CloseCalls == 0 && s_OpenCalls == 1);
}

static void AcceptedResponseThenTransportError()
{
	const uint32_t now = 10000U;
	std::vector<uint8_t> request;

	Reset(now);
	request = Begin(now);
	Deliver(Reply(request, COAP_CONTENT, FirstPayload(), COAP_CON, 901));
	s_Queue.WriteFault = true;
	Pump(now + 20U);
	assert(s_Checked == 1 && s_Failures == 1);
	assert(!s_Started && !s_Waiting && !s_pIntrf && s_CloseCalls == 1);
	assert(!atomic_flag_test_and_set(&s_Queue.Dev.bBusy));
	atomic_flag_clear(&s_Queue.Dev.bBusy);
	assert(s_Log.find("transport closed after response") != std::string::npos);
}

static void DatagramBoundaries()
{
	const uint32_t now = 10000U;
	std::vector<uint8_t> request;
	std::vector<uint8_t> response;
	size_t split;

	Reset(now);
	request = Begin(now);
	response = Reply(request, COAP_CONTENT, FirstPayload());
	// Split inside the token: neither fragment is a complete CoAP packet.
	// A split within the payload could itself be a valid shorter datagram.
	split = 7U;
	Deliver(std::vector<uint8_t>(response.begin(), response.begin() + split));
	Deliver(std::vector<uint8_t>(response.begin() + split, response.end()));
	Pump(now + 20U);
	Pump(now + 40U);
	assert(s_Checked == 0 && s_Waiting);
	Deliver(response);
	Pump(now + 60U);
	assert(s_Checked == 1);
	Reset(now);
	s_Selected = &s_Mailbox;
	request = Begin(now);
	response = Reply(request, COAP_CONTENT, std::vector<uint8_t>(1200U, 0xa5));
	Deliver(response);
	Pump(now + 20U);
	assert(s_Mailbox.Capacity == COAP_MESSAGE_SIZE + 1U && !s_Mailbox.Full);
	assert(s_Checked == 0 && s_Failures == 1 && s_Started && !s_Waiting);
	assert(s_OpenCalls == 1 && s_CloseCalls == 0);
	request = Begin(now + COAP_DEMO_INTERVAL);
	Deliver(Reply(request, COAP_CONTENT, FirstPayload()));
	Pump(now + COAP_DEMO_INTERVAL + 20U);
	assert(s_Checked == 1 && s_Failures == 1);
}

static void OpenTiming()
{
	const uint32_t start = 0xfffff000U;
	uint32_t finished;

	Reset(start);
	s_OpenWorks = false;
	s_OpenDuration = 700U;
	Pump(start);
	finished = s_TestNow;
	assert(s_OpenCalls == 1 && s_Retry == finished && !s_Started);
	Pump(finished + COAP_DEMO_RECOVERY_MS - 1U);
	assert(s_OpenCalls == 1);
	s_OpenWorks = true;
	Begin(finished + COAP_DEMO_RECOVERY_MS);
	assert(s_OpenCalls == 2 && s_LastRequest == s_TestNow);
	assert(s_Coap.RetryAfter(s_TestNow) == s_Queue.Out[0].size() * 1000U);
}

static void CloseTiming()
{
	const uint32_t start = 0xfffff000U;
	std::vector<uint8_t> request;
	uint32_t debt;
	uint32_t finished;
	uint32_t wait;

	for (bool expireDebt : {false, true})
	{
		Reset(start);
		request = Begin(start);
		debt = (uint32_t)request.size() * 1000U;
		s_CloseDuration = expireDebt ? debt + 5000U : 6000U;
		s_Ready = false;
		Pump(start + 100U);
		finished = s_TestNow;
		assert(finished < start && s_Retry == finished);
		assert(s_ProbeStarted == start + 100U && s_ProbeDelay == debt - 100U);
		assert(!s_Started && s_CloseCalls == 1 && !s_pIntrf);
		wait = expireDebt ? COAP_DEMO_RECOVERY_MS : debt - 100U - s_CloseDuration;
		assert(wait >= COAP_DEMO_RECOVERY_MS);
		s_CloseDuration = 0;
		s_Ready = true;
		Pump(finished + wait - 1U);
		assert(s_OpenCalls == 1 && !s_Started);
		Begin(finished + wait);
		assert(s_OpenCalls == 2 && s_CloseCalls == 1 && !s_ProbeDelay);
	}
}

static void PublicProfile()
{
	const uint32_t now = 10000U;
	std::vector<uint8_t> request;
	std::string resource(255U, 'x');

	assert(PublicResourceValid(resource.c_str()));
	resource += 'x';
	assert(!PublicResourceValid(resource.c_str()) && !PublicResourceValid(nullptr));
	for (const char *bad : {"", "/test", "a/b", "test?x", "test#x", ".", ".."})
	{
		assert(!PublicResourceValid(bad));
	}
	for (size_t length : {0U, 17U, 1024U, 1025U})
	{
		Reset(now);
		request = Begin(now);
		CheckFirstRequest(request);
		Deliver(Reply(request, COAP_CONTENT, std::vector<uint8_t>(length, 0x42)));
		Pump(now + 20U);
		assert(s_Started && !s_Waiting && s_CloseCalls == 0);
		assert(s_Checked == (length <= 1024U ? 1U : 0U));
		assert(s_Failures == (length <= 1024U ? 0U : 1U));
		assert(s_Log.find("methods, CON/NON and separate response verified") == std::string::npos);
		Pump(now + COAP_DEMO_INTERVAL - 1U);
		assert(!s_Waiting && s_Queue.Out.size() == 1);
		Begin(now + COAP_DEMO_INTERVAL);
		assert(s_Queue.Out.size() == 2);
	}
	Reset(now);
	request = Begin(now);
	Deliver(Reply(request, COAP_NOT_FOUND, {'n', 'o'}));
	Pump(now + 20U);
	assert(s_Checked == 0 && s_Failures == 1 && s_Started);
	request = Begin(now + COAP_DEMO_INTERVAL);
	Deliver(Reply(request, COAP_CONTENT, {'o', 'k'}));
	Pump(now + COAP_DEMO_INTERVAL + 20U);
	assert(s_Checked == 1 && s_Failures == 1);
}

static void HelperProfile()
{
	const uint8_t methods[] = {1, 2, 2, 3, 1, 4, 1, 1};
	const uint8_t codes[] = {69, 69, 69, 68, 69, 66, 132, 69};
	const char *paths[] = {"hello", "echo", "echo", "value", "value", "value", "value", "slow"};
	const size_t lengths[] = {1, 16, 257, 900};
	std::vector<uint8_t> binary;
	std::vector<uint8_t> request;
	std::vector<uint8_t> payload;
	CoapMessage_t message = {};
	uint32_t now = 10000U;
	unsigned cycle;
	unsigned step;

	Reset(now);
	for (cycle = 0; cycle < 4; cycle++)
	{
		for (step = 0; step < 8; step++)
		{
			request = Begin(now);
			assert(CoapDecode(&message, request.data(), request.size()) == COAP_ERROR_NONE);
			assert(message.Code == methods[step]);
			assert(message.Type == (step == 2 ? COAP_NON : COAP_CON));
			assert(message.Options[0].Number == COAP_OPTION_URI_PATH);
			assert(message.Options[0].Len == strlen(paths[step]));
			assert(memcmp(message.Options[0].pValue, paths[step], strlen(paths[step])) == 0);
			payload.clear();
			if (step >= 1 && step <= 3)
			{
				assert(message.PayloadLen == lengths[cycle] && message.OptionCount == 2);
				assert(message.Options[1].Number == COAP_OPTION_CONTENT_FORMAT);
				assert(message.Options[1].Len == 1 && message.Options[1].pValue[0] == 42);
				if (step == 1)
				{
					binary.assign(message.pPayload, message.pPayload + message.PayloadLen);
				}
				else
				{
					assert(memcmp(message.pPayload, binary.data(), binary.size()) == 0);
				}
			}
			else
			{
				assert(message.PayloadLen == 0 && message.OptionCount == 1);
			}
			if (step == 0)
			{
				payload = FirstPayload();
			}
			else if (step == 1 || step == 2 || step == 4)
			{
				payload = binary;
			}
			else if (step == 7)
			{
				const char text[] = "separate response\n";

				payload.assign(text, text + sizeof(text) - 1U);
				Deliver({0x60, 0, request[2], request[3]});
				Pump(now + 20U);
				assert(s_Waiting);
			}
			Deliver(Reply(request, codes[step], payload,
				step == 7 ? COAP_CON : step == 2 ? COAP_NON : COAP_ACK,
				(uint16_t)(900U + cycle)));
			Pump(now + 40U);
			assert(s_Checked == cycle * 8U + step + 1U && s_Failures == 0);
			assert(!s_Waiting);
			now += COAP_DEMO_INTERVAL;
		}
	}
	assert(s_Cycle == 4 && s_OpenCalls == 1 && s_CloseCalls == 0);
	assert(s_Log.find("CoAP methods, CON/NON and separate response verified") != std::string::npos);
}

static void Startup()
{
	const char *host = COAP_DEMO_HOST;

	Reset(0);
	assert(CoapGenericMain() == 0);
	assert(s_InitCalls == 1 && s_RunCalls == 1 && s_OpenCalls == 0);
	if (!host || !host[0])
	{
		assert(s_StartCalls == 0 && !s_Handler);
		assert(s_Log.find("Set COAP_DEMO_HOST") != std::string::npos);
	}
	else
	{
		assert(s_StartCalls == 1 && s_Handler == Report);
		s_TestNow = COAP_DEMO_INTERVAL;
		s_Handler(s_TestNow);
		assert(s_OpenCalls == 1);
	}
	Reset(0);
	s_InitWorks = false;
	assert(CoapGenericMain() == 1);
	assert(s_StartCalls == 0 && s_RunCalls == 0 && s_OpenCalls == 0);
}

int main()
{
	const char *host = COAP_DEMO_HOST;

	Startup();
	if (!host || !host[0])
	{
		puts("Generic CoAP empty host: network remains stopped PASS");
		return 0;
	}
	CompletedSwitch();
	OutstandingSwitch(10000U, false);
	OutstandingSwitch(0xfffff000U, false);
	OutstandingSwitch(10000U, true);
	LiveTimeout();
	ControlBackpressure();
	AcceptedResponseThenTransportError();
	DatagramBoundaries();
	OpenTiming();
	CloseTiming();
	if (COAP_DEMO_PUBLIC_TEST)
	{
		PublicProfile();
	}
	else
	{
		HelperProfile();
	}
	puts(COAP_DEMO_PUBLIC_TEST ?
		"Generic CoAP public GET: two interfaces, switching, recovery and datagrams PASS" :
		"Generic CoAP helper: all steps, two interfaces, switching and pacing PASS");
	return 0;
}

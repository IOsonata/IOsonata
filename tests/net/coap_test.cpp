/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Independent RFC 7252 wire fixtures and datagram exchange fault tests.
 * Host-only containers do not appear in the embedded implementation.
 */
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <deque>
#include <vector>
#include "net/coap.h"

class TestRng : public RngEngine
{
public:
	uint8_t Byte = 0;
	bool Increment = false;
	bool Works = true;

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
		if (!Works)
		{
			return CRYPTO_STATUS_FAIL;
		}
		memset(pData, Byte, Len);
		if (Increment)
		{
			Byte++;
		}
		return CRYPTO_STATUS_OK;
	}
};

struct Peer
{
	DevIntrf_t Dev = {};
	std::deque<std::vector<uint8_t>> In;
	std::vector<std::vector<uint8_t>> Out;
	bool Block = false;
	bool StartWorks = true;
	int WriteError = 0;
	int ReadError = 0;
	int ReadCapacity = 0;
	unsigned Reads = 0;
	unsigned Writes = 0;
};

static bool Start(DevIntrf_t *pDev, uint32_t)
{
	Peer *p = (Peer *)pDev->pDevData;

	return p->StartWorks;
}

static void Stop(DevIntrf_t *)
{
}

static int Read(DevIntrf_t *pDev, uint8_t *pData, int Len)
{
	Peer *p = (Peer *)pDev->pDevData;
	size_t count;

	p->Reads++;
	p->ReadCapacity = Len;
	if (p->ReadError)
	{
		return p->ReadError < 0 ? -1 : Len + 1;
	}
	if (p->In.empty())
	{
		return 0;
	}
	count = std::min((size_t)Len, p->In.front().size());
	if (count)
	{
		memcpy(pData, p->In.front().data(), count);
	}
	// UDP consumes the whole datagram, including any truncated remainder.
	p->In.pop_front();
	return (int)count;
}

static int Write(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	Peer *p = (Peer *)pDev->pDevData;

	p->Writes++;
	if (p->WriteError)
	{
		if (p->WriteError < 0)
		{
			return -1;
		}
		return p->WriteError == 1 ? Len - 1 : Len + 1;
	}
	if (p->Block)
	{
		return 0;
	}
	p->Out.emplace_back(pData, pData + Len);
	return Len;
}

static void Setup(Peer &Peer)
{
	Peer.Dev.pDevData = &Peer;
	Peer.Dev.StartRx = Peer.Dev.StartTx = Start;
	Peer.Dev.StopRx = Peer.Dev.StopTx = Stop;
	Peer.Dev.RxData = Read;
	Peer.Dev.TxData = Write;
	Peer.Dev.EnCnt = 1;
	atomic_flag_clear(&Peer.Dev.bBusy);
}

struct Fixture
{
	Peer PeerData;
	TestRng Rng;
	CoapDev_t Dev = {};

	Fixture(uint32_t Now = 0, uint32_t Timeout = 100000, uint32_t ProbingRate = 1152000)
	{
		CoapCfg_t cfg = {&Rng, Timeout, ProbingRate};

		Setup(PeerData);
		assert(CoapInit(&Dev, &cfg, &PeerData.Dev, Now));
		assert(Dev.State == COAP_READY);
	}
};

static void Pump(Fixture &F, uint32_t Now = 0, unsigned Count = 3)
{
	while (Count--)
	{
		CoapProcess(&F.Dev, Now, true);
	}
}

static std::vector<uint8_t> Begin(Fixture &F, uint32_t Now = 0, COAP_TYPE Type = COAP_CON,
								uint8_t Method = COAP_GET)
{
	const uint8_t path[] = {'x'};
	CoapOption_t option = {COAP_OPTION_URI_PATH, path, sizeof(path)};
	CoapRequest_t request = {Type, Method, &option, 1, nullptr, 0};

	assert(CoapRequest(&F.Dev, &request, Now));
	Pump(F, Now);
	assert(F.PeerData.Out.size() == 1);
	assert(F.PeerData.Out[0].size() == 4 + COAP_TOKEN_SIZE + 2);
	assert((F.PeerData.Out[0][0] & 15U) == COAP_TOKEN_SIZE);
	assert(F.PeerData.Out[0][1] == Method);
	assert(F.PeerData.Out[0][4 + COAP_TOKEN_SIZE] == 0xb1);
	assert(F.PeerData.Out[0].back() == 'x');
	assert(CoapBusy(&F.Dev));
	return F.PeerData.Out[0];
}

static uint16_t MessageId(const std::vector<uint8_t> &Packet)
{
	assert(Packet.size() >= 4);
	return (uint16_t)((Packet[2] << 8) | Packet[3]);
}

static std::vector<uint8_t> Empty(COAP_TYPE Type, uint16_t Id)
{
	return {(uint8_t)(0x40U | ((unsigned)Type << 4)), 0, (uint8_t)(Id >> 8), (uint8_t)Id};
}

// Deliberately independent of CoapEncode/CoapDecode: copy only the request's
// opaque token and choose explicit type, code, MID, option bytes and payload.
static std::vector<uint8_t> Reply(const std::vector<uint8_t> &Request, COAP_TYPE Type = COAP_ACK,
								uint8_t Code = COAP_CONTENT, uint16_t Id = 0,
								std::vector<uint8_t> Options = {},
								std::vector<uint8_t> Payload = {'o', 0, 'k'})
{
	size_t tokenLen = Request[0] & 15U;
	uint16_t id = Type == COAP_ACK ? MessageId(Request) : Id;
	std::vector<uint8_t> result = {(uint8_t)(0x40U | ((unsigned)Type << 4) | tokenLen), Code,
								   (uint8_t)(id >> 8), (uint8_t)id};

	result.insert(result.end(), Request.begin() + 4, Request.begin() + 4 + tokenLen);
	result.insert(result.end(), Options.begin(), Options.end());
	if (!Payload.empty())
	{
		result.push_back(0xff);
		result.insert(result.end(), Payload.begin(), Payload.end());
	}
	return result;
}

static void CheckPayload(const Fixture &F)
{
	assert(F.Dev.State == COAP_RESPONSE);
	assert(F.Dev.Error == COAP_ERROR_NONE);
	assert(F.Dev.Message.PayloadLen == 3);
	assert(memcmp(F.Dev.Message.pPayload, "o\0k", 3) == 0);
	assert(CoapBusy(&F.Dev));
}

static void CodecFixtures()
{
	std::vector<std::vector<uint8_t>> fixtures = {
		{0x40, 0, 0x12, 0x34},
		{0x60, 0, 0x12, 0x34},
		{0x70, 0, 0x12, 0x34},
		{0x40, 1, 0x12, 0x34},
		{0x58, 2, 0xab, 0xcd, 0, 1, 2, 3, 4, 5, 6, 7, 0xff, 0, 0xff, 0x55},
		{0x64, 69, 0x12, 0x34, 0xde, 0xad, 0xbe, 0xef, 0xc1, 42, 0xff, 'o', 0, 'k'},
		{0x40, 1, 0x12, 0x34, 0xb3, 'f', 'o', 'o', 0x00, 0x03, 'b', 'a', 'r', 0x40},
		{0x40, 1, 0x12, 0x34, 0xd1, 0, 0xff},
		{0x40, 1, 0x12, 0x34, 0xe1, 0, 0, 7}
	};
	CoapMessage_t message = {};
	uint8_t encoded[COAP_MESSAGE_SIZE];
	size_t count;
	size_t i;
	std::vector<uint8_t> extended = {0x40, 1, 0x12, 0x34, 0x0e, 0, 0};

	extended.insert(extended.end(), 269, 0xa5);
	fixtures.push_back(extended);
	for (const auto &packet : fixtures)
	{
		assert(CoapDecode(&message, packet.data(), packet.size()) == COAP_ERROR_NONE);
		assert(message.MessageId == MessageId(packet));
		assert(message.Type == (COAP_TYPE)((packet[0] >> 4) & 3U));
		assert(message.Code == packet[1]);
		count = 0;
		assert(CoapEncode(&message, encoded, sizeof(encoded), &count) == COAP_ERROR_NONE);
		assert(count == packet.size());
		assert(memcmp(encoded, packet.data(), count) == 0);
		for (i = 0; i < count; i++)
		{
			assert(CoapEncode(&message, encoded, i, &count) != COAP_ERROR_NONE);
			count = packet.size();
		}
	}
	assert(CoapDecode(&message, fixtures[6].data(), fixtures[6].size()) == COAP_ERROR_NONE);
	assert(message.OptionCount == 4 && message.Options[1].Number == 11);
	assert(message.Options[1].Len == 0 && message.Options[3].Number == 15);
	assert(CoapDecode(&message, fixtures[7].data(), fixtures[7].size()) == COAP_ERROR_NONE);
	assert(message.Options[0].Number == 13 && message.Options[0].pValue[0] == 0xff);
	assert(CoapDecode(&message, fixtures[8].data(), fixtures[8].size()) == COAP_ERROR_NONE);
	assert(message.Options[0].Number == 269);
	assert(CoapDecode(&message, extended.data(), extended.size()) == COAP_ERROR_NONE);
	assert(message.Options[0].Len == 269);
	printf("CoAP codec: independent valid wire fixtures and exact buffer boundaries PASS\n");
}

static void CodecErrors()
{
	std::vector<std::vector<uint8_t>> malformed = {
		{}, {0x40}, {0x40, 1, 0}, {0, 1, 0, 1}, {0x80, 1, 0, 1},
		{0x49, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0x48, 1, 0, 1}, {0x40, 1, 0, 1, 0xff},
		{0x40, 1, 0, 1, 0xf0}, {0x40, 1, 0, 1, 0x0f},
		{0x40, 1, 0, 1, 0xd0}, {0x40, 1, 0, 1, 0xe0, 0},
		{0x40, 1, 0, 1, 0xe0, 0xff, 0xff},
		{0x40, 1, 0, 1, 0x0d}, {0x40, 1, 0, 1, 0x0e, 0},
		{0x40, 1, 0, 1, 0xb5, 'x'},
		{0x60, 0, 0, 1, 0}, {0x61, 0, 0, 1, 0},
		{0x60, 1, 0, 1}, {0x70, 69, 0, 1}, {0x50, 0, 0, 1},
		{0x40, 32, 0, 1}, {0x40, 224, 0, 1}
	};
	CoapMessage_t message = {};
	uint8_t data[COAP_MESSAGE_SIZE + 1U] = {};
	size_t count = 0;
	std::vector<uint8_t> options = {0x40, 1, 0, 1};

	for (const auto &packet : malformed)
	{
		assert(CoapDecode(&message, packet.data(), packet.size()) != COAP_ERROR_NONE);
	}
	options.insert(options.end(), COAP_MAX_OPTIONS + 1U, 0);
	assert(CoapDecode(&message, options.data(), options.size()) == COAP_ERROR_LIMIT);
	data[0] = 0x40;
	data[1] = 1;
	data[4] = 0xff;
	assert(CoapDecode(&message, data, sizeof(data)) == COAP_ERROR_LIMIT);
	message = {};
	message.Type = COAP_CON;
	message.Code = COAP_GET;
	message.TokenLen = 9;
	message.pToken = data;
	assert(CoapEncode(&message, data, sizeof(data), &count) != COAP_ERROR_NONE);
	message.TokenLen = 1;
	message.pToken = nullptr;
	assert(CoapEncode(&message, data, sizeof(data), &count) != COAP_ERROR_NONE);
	message.TokenLen = 0;
	message.OptionCount = 2;
	message.Options[0] = {12, nullptr, 0};
	message.Options[1] = {11, nullptr, 0};
	assert(CoapEncode(&message, data, sizeof(data), &count) != COAP_ERROR_NONE);
	message.OptionCount = 0;
	message.PayloadLen = 1;
	assert(CoapEncode(&message, data, sizeof(data), &count) != COAP_ERROR_NONE);
	assert(CoapDecode(nullptr, data, 4) != COAP_ERROR_NONE);
	assert(CoapEncode(nullptr, data, sizeof(data), &count) != COAP_ERROR_NONE);
	printf("CoAP codec: malformed headers/options, limits and invalid views PASS\n");
}

static void ParserBounds()
{
	uint32_t random = 0x735a18d1U;
	uint8_t bytes[96];
	CoapMessage_t message = {};
	size_t len;
	size_t i;
	unsigned pass;

	// Fixed seed and bounded length exercise parser bounds under sanitizers.
	for (pass = 0; pass < 8000; pass++)
	{
		len = pass % sizeof(bytes);
		for (i = 0; i < len; i++)
		{
			random = random * 1664525U + 1013904223U;
			bytes[i] = (uint8_t)(random >> 24);
		}
		if (len >= 4 && (pass & 1U))
		{
			bytes[0] = 0x40;
			bytes[1] = 1;
		}
		if (CoapDecode(&message, bytes, len) == COAP_ERROR_NONE)
		{
			assert(message.TokenLen <= COAP_TOKEN_SIZE);
			assert(message.OptionCount <= COAP_MAX_OPTIONS);
			for (i = 0; i < message.OptionCount; i++)
			{
				assert(message.Options[i].pValue >= bytes);
				assert(message.Options[i].pValue + message.Options[i].Len <= bytes + len);
			}
			if (message.PayloadLen)
			{
				assert(message.pPayload >= bytes);
				assert(message.pPayload + message.PayloadLen == bytes + len);
			}
		}
	}
}

static void MatchingAndOwnership()
{
	Fixture f;
	std::vector<uint8_t> request = Begin(f);
	std::vector<uint8_t> wrong = Reply(request);
	CoapRequest_t another = {COAP_CON, COAP_GET, nullptr, 0, nullptr, 0};
	const uint8_t *payload;
	unsigned reads;
	unsigned writes;

	assert(!CoapRequest(&f.Dev, &another, 0));
	wrong[3] ^= 1;
	f.PeerData.In.push_back(wrong);
	Pump(f, 1);
	assert(CoapBusy(&f.Dev));
	wrong = Reply(request);
	wrong[4] ^= 1;
	f.PeerData.In.push_back(wrong);
	f.PeerData.In.push_back(Empty(COAP_ACK, (uint16_t)(MessageId(request) + 1U)));
	f.PeerData.In.push_back(Empty(COAP_RST, (uint16_t)(MessageId(request) + 1U)));
	Pump(f, 2);
	assert(CoapBusy(&f.Dev) && f.PeerData.Out.size() == 1);
	f.PeerData.In.push_back(Reply(request));
	reads = f.PeerData.Reads;
	writes = f.PeerData.Writes;
	Pump(f, 3, 1);
	assert(f.PeerData.Reads - reads <= 1 && f.PeerData.Writes - writes <= 1);
	CheckPayload(f);
	payload = f.Dev.Message.pPayload;
	f.PeerData.In.push_back(Reply(request, COAP_CON, COAP_CONTENT, 700));
	Pump(f, 4);
	assert(f.Dev.Message.pPayload == payload);
	CheckPayload(f);
	assert(!CoapRequest(&f.Dev, &another, 4));
	CoapRelease(&f.Dev);
	assert(f.Dev.State == COAP_READY && !CoapBusy(&f.Dev));
	f.PeerData.Out.clear();
	f.Rng.Byte = 1;
	request = Begin(f, 5);
	f.PeerData.In.push_back(Reply(request, COAP_ACK, COAP_NOT_FOUND, 0, {}, {}));
	Pump(f, 6);
	assert(f.Dev.State == COAP_RESPONSE && f.Dev.Message.Code == COAP_NOT_FOUND);
	assert(f.Dev.Error == COAP_ERROR_NONE);
	printf("CoAP exchange: MID/token matching, held response and ordinary 4.04 PASS\n");
}

static void SeparateAndDuplicate()
{
	Fixture f;
	std::vector<uint8_t> request = Begin(f);
	std::vector<uint8_t> response;
	std::vector<uint8_t> next;
	size_t output;

	f.PeerData.In.push_back(Empty(COAP_ACK, MessageId(request)));
	Pump(f, 1);
	assert(f.Dev.State == COAP_WAIT_RESPONSE);
	Pump(f, 5000);
	assert(f.PeerData.Out.size() == 1);
	response = Reply(request, COAP_CON, COAP_CONTENT, 0x4567);
	f.PeerData.In.push_back(response);
	Pump(f, 5001);
	CheckPayload(f);
	assert(f.PeerData.Out.back() == Empty(COAP_ACK, 0x4567));
	output = f.PeerData.Out.size();
	f.PeerData.In.push_back(response);
	Pump(f, 5002);
	CheckPayload(f);
	assert(f.PeerData.Out.size() == output + 1);
	assert(f.PeerData.Out.back() == Empty(COAP_ACK, 0x4567));
	CoapRelease(&f.Dev);
	f.PeerData.Out.clear();
	f.Rng.Byte = 5;
	next = Begin(f, 5003);
	f.PeerData.In.push_back(response);
	Pump(f, 5004);
	assert(CoapBusy(&f.Dev));
	assert(f.PeerData.Out.back() == Empty(COAP_ACK, 0x4567));
	f.PeerData.In.push_back(Reply(next, COAP_NON, COAP_CONTENT, 0x6789));
	Pump(f, 5005);
	CheckPayload(f);
	printf("CoAP exchange: empty ACK, separate response, duplicate ACK and next request PASS\n");
}

static void TypePermutations()
{
	COAP_TYPE requestType;
	COAP_TYPE responseType;

	for (requestType = COAP_CON; requestType <= COAP_NON;
		 requestType = (COAP_TYPE)(requestType + 1))
	{
		for (responseType = COAP_CON; responseType <= COAP_NON;
			 responseType = (COAP_TYPE)(responseType + 1))
		{
			Fixture f;
			std::vector<uint8_t> request = Begin(f, 0, requestType);

			// A separate response can arrive before the ACK of a CON request.
			f.PeerData.In.push_back(Reply(request, responseType, COAP_CONTENT, 91));
			Pump(f, 1);
			CheckPayload(f);
			assert(f.PeerData.Out.size() == (responseType == COAP_CON ? 2U : 1U));
			Pump(f, 4000);
			assert(f.PeerData.Out.size() == (responseType == COAP_CON ? 2U : 1U));
		}
	}
}

static void Retransmission()
{
	Fixture f;
	std::vector<uint8_t> request = Begin(f);
	const uint32_t times[] = {2000, 6000, 14000, 30000};
	size_t i;

	Pump(f, 1999);
	assert(f.PeerData.Out.size() == 1);
	for (i = 0; i < sizeof(times) / sizeof(times[0]); i++)
	{
		Pump(f, times[i]);
		assert(f.PeerData.Out.size() == i + 2);
		assert(f.PeerData.Out.back() == request);
		assert(CoapBusy(&f.Dev));
	}
	Pump(f, 61999);
	assert(CoapBusy(&f.Dev));
	Pump(f, 62000);
	assert(f.Dev.State == COAP_FAILED && f.Dev.Error == COAP_ERROR_TIMEOUT);
	assert(f.PeerData.Out.size() == 5);
	{
		Fixture stalled;
		std::vector<uint8_t> initial = Begin(stalled);

		Pump(stalled, COAP_MAX_TRANSMIT_SPAN + 1);
		assert(stalled.PeerData.Out.size() == 1);
		(void)initial;
	}
	{
		Fixture blocked(0, 5000);
		CoapRequest_t requestData = {COAP_CON, COAP_GET, nullptr, 0, nullptr, 0};

		blocked.PeerData.Block = true;
		assert(CoapRequest(&blocked.Dev, &requestData, 0));
		Pump(blocked, 0);
		Pump(blocked, 5000);
		assert(blocked.Dev.Error == COAP_ERROR_TIMEOUT && blocked.PeerData.Out.empty());
	}
	printf("CoAP exchange: identical retransmissions, exponential delay and stalled pump PASS\n");
}

static void ResetCancelAndWrap()
{
	Fixture reset;
	std::vector<uint8_t> request = Begin(reset);
	CoapRequest_t next = {COAP_CON, COAP_GET, nullptr, 0, nullptr, 0};
	uint16_t id = MessageId(request);
	size_t i;

	reset.PeerData.In.push_back(Empty(COAP_RST, id));
	Pump(reset, 1);
	assert(reset.Dev.State == COAP_FAILED && reset.Dev.Error == COAP_ERROR_RESET);
	CoapRelease(&reset.Dev);
	assert(reset.Dev.State == COAP_READY);
	assert(CoapRequest(&reset.Dev, &next, 2));
	Pump(reset, 2);
	assert(MessageId(reset.PeerData.Out.back()) != id);
	CoapCancel(&reset.Dev);
	assert(reset.Dev.State == COAP_READY && !CoapBusy(&reset.Dev));
	assert(reset.Dev.Error == COAP_ERROR_CANCELLED);
	CoapReset(&reset.Dev);
	assert(reset.Dev.State == COAP_IDLE && reset.Dev.pTransport == nullptr);
	for (i = 0; i < sizeof(reset.Dev.Data); i++)
	{
		assert(reset.Dev.Data[i] == 0);
	}
	for (i = 0; i < sizeof(reset.Dev.Rx); i++)
	{
		assert(reset.Dev.Rx[i] == 0);
	}
	{
		Fixture wrapped(0xfffffff0U, 1000);
		std::vector<uint8_t> packet = Begin(wrapped, 0xfffffff0U);

		Pump(wrapped, 0x3d7U);
		assert(CoapBusy(&wrapped.Dev));
		Pump(wrapped, 0x3d8U);
		assert(wrapped.Dev.Error == COAP_ERROR_TIMEOUT);
		(void)packet;
	}
}

static void DatagramBoundaries()
{
	Fixture f;
	std::vector<uint8_t> request = Begin(f);
	std::vector<uint8_t> oversized = Reply(request);
	size_t first;

	oversized.resize(COAP_MESSAGE_SIZE + 40U, 0x55);
	f.PeerData.In.push_back(oversized);
	f.PeerData.In.push_back(Reply(request));
	Pump(f, 1, 1);
	assert(f.PeerData.ReadCapacity == (int)COAP_MESSAGE_SIZE + 1);
	assert(f.PeerData.In.size() == 1);
	assert(f.Dev.State == COAP_FAILED && f.Dev.Error == COAP_ERROR_LIMIT);
	CoapRelease(&f.Dev);
	f.PeerData.Out.clear();
	f.Rng.Byte = 3;
	request = Begin(f, 2);
	// The old response remains a separate stale datagram after Release.
	f.PeerData.In.push_back(Reply(request));
	Pump(f, 2);
	CheckPayload(f);
	CoapRelease(&f.Dev);
	f.PeerData.Out.clear();
	request = Begin(f, 3);
	f.PeerData.In.push_back({});
	f.PeerData.In.push_back(Reply(request));
	Pump(f, 4);
	CheckPayload(f);
	CoapRelease(&f.Dev);
	f.PeerData.Out.clear();
	request = Begin(f, 5);
	oversized = Reply(request);
	first = oversized.size() / 2;
	f.PeerData.In.emplace_back(oversized.begin(), oversized.begin() + first);
	f.PeerData.In.emplace_back(oversized.begin() + first, oversized.end());
	Pump(f, 6);
	assert(f.Dev.State != COAP_RESPONSE);
	f.PeerData.In.push_back(Reply(request));
	Pump(f, 7);
	CheckPayload(f);
	printf("CoAP datagrams: truncation failure/recovery, zero-length and no reassembly PASS\n");
}

static void ProtocolRejection()
{
	Fixture f;
	std::vector<uint8_t> request = Begin(f);
	std::vector<uint8_t> bad;
	size_t output = f.PeerData.Out.size();

	// Invalid ACK and RST must never produce an ACK or RST in reply.
	f.PeerData.In.push_back({0x60, 1, request[2], request[3]});
	f.PeerData.In.push_back({0x70, 69, request[2], request[3]});
	Pump(f, 1);
	assert(CoapBusy(&f.Dev) && f.PeerData.Out.size() == output);
	bad = Reply(request, COAP_CON, COAP_CONTENT, 333);
	bad[0] &= 0x3f;
	f.PeerData.In.push_back(bad);
	Pump(f, 2);
	assert(f.PeerData.Out.size() == output);
	bad = Reply(request, COAP_CON, COAP_CONTENT, 334);
	bad[4] ^= 1;
	f.PeerData.In.push_back(bad);
	Pump(f, 3);
	assert(CoapBusy(&f.Dev) && f.PeerData.Out.back() == Empty(COAP_RST, 334));
	f.PeerData.In.push_back({0x40, 0, 1, 79});
	Pump(f, 4);
	assert(f.PeerData.Out.back() == Empty(COAP_RST, 335));
	f.PeerData.In.push_back(Reply(request));
	Pump(f, 5);
	CheckPayload(f);
	{
		Fixture elective;
		std::vector<uint8_t> packet = Begin(elective);

		// Unknown elective option 30 is ignored by the client.
		elective.PeerData.In.push_back(Reply(packet, COAP_ACK, COAP_CONTENT, 0, {0xd1, 17, 7}));
		Pump(elective, 1);
		CheckPayload(elective);
	}
	{
		Fixture critical;
		std::vector<uint8_t> packet = Begin(critical);

		// Block2 is critical and unsupported. Never deliver a partial body.
		critical.PeerData.In.push_back(Reply(packet, COAP_CON, COAP_CONTENT, 700, {0xd1, 10, 0}));
		Pump(critical, 1);
		assert(critical.Dev.State != COAP_RESPONSE);
		assert(critical.PeerData.Out.back() == Empty(COAP_RST, 700));
	}
	printf("CoAP rejection: wrong version, invalid controls, unmatched CON and option classes PASS\n");
}

static void TransportAndConfigErrors()
{
	const int faults[] = {-1, 1, 2};
	int fault;
	size_t i;

	for (i = 0; i < sizeof(faults) / sizeof(faults[0]); i++)
	{
		Fixture f;
		CoapRequest_t request = {COAP_CON, COAP_GET, nullptr, 0, nullptr, 0};

		fault = faults[i];
		f.PeerData.WriteError = fault;
		assert(CoapRequest(&f.Dev, &request, 0));
		Pump(f);
		assert(f.Dev.State == COAP_FAILED && f.Dev.Error == COAP_ERROR_TRANSPORT);
		assert(f.PeerData.Out.empty());
	}
	{
		Fixture f;
		std::vector<uint8_t> request = Begin(f);

		CoapProcess(&f.Dev, 1, false);
		assert(f.Dev.Error == COAP_ERROR_TRANSPORT);
		(void)request;
	}
	{
		Fixture f;
		CoapRequest_t request = {COAP_ACK, COAP_GET, nullptr, 0, nullptr, 0};
		CoapCfg_t cfg = {&f.Rng, 1000, 1152000};
		CoapDev_t invalid = {};
		uint8_t data[COAP_MESSAGE_SIZE] = {};

		assert(!CoapRequest(&f.Dev, &request, 0));
		request.Type = COAP_CON;
		request.Code = 99;
		assert(!CoapRequest(&f.Dev, &request, 0));
		request.Code = COAP_GET;
		request.pPayload = data;
		request.PayloadLen = sizeof(data);
		assert(!CoapRequest(&f.Dev, &request, 0));
		assert(f.PeerData.Out.empty());
		f.PeerData.Dev.MaxRetry = 1;
		assert(!CoapInit(&invalid, &cfg, &f.PeerData.Dev, 0));
		f.PeerData.Dev.MaxRetry = 0;
		cfg.Timeout = 0;
		assert(!CoapInit(&invalid, &cfg, &f.PeerData.Dev, 0));
		cfg.Timeout = (uint32_t)INT_MAX + 1U;
		assert(!CoapInit(&invalid, &cfg, &f.PeerData.Dev, 0));
		cfg.Timeout = 1000;
		cfg.pRng = nullptr;
		assert(!CoapInit(&invalid, &cfg, &f.PeerData.Dev, 0));
		cfg.pRng = &f.Rng;
		f.Rng.Works = false;
		assert(!CoapInit(&invalid, &cfg, &f.PeerData.Dev, 0));
	}
}

static void RequestStorageAndControlPressure()
{
	Fixture f;
	uint8_t path[] = {'t', 'e', 's', 't'};
	uint8_t payload[] = {0, 0xff, 0x42};
	CoapOption_t option = {COAP_OPTION_URI_PATH, path, sizeof(path)};
	CoapRequest_t request = {COAP_CON, COAP_POST, &option, 1, payload, sizeof(payload)};
	std::vector<uint8_t> packet;
	std::vector<uint8_t> response;
	unsigned reads;
	const uint8_t expected[] = {0xb4, 't', 'e', 's', 't', 0xff, 0, 0xff, 0x42};

	assert(CoapRequest(&f.Dev, &request, 0));
	memset(path, 0, sizeof(path));
	memset(payload, 0, sizeof(payload));
	Pump(f);
	assert(f.PeerData.Out.size() == 1);
	packet = f.PeerData.Out[0];
	assert(packet.size() == 4 + COAP_TOKEN_SIZE + sizeof(expected));
	assert(memcmp(packet.data() + 4 + COAP_TOKEN_SIZE, expected, sizeof(expected)) == 0);
	response = Reply(packet, COAP_CON, COAP_CONTENT, 0x3456);
	f.PeerData.Block = true;
	f.PeerData.In.push_back(response);
	Pump(f, 1);
	CheckPayload(f);
	assert(f.Dev.bControl && f.PeerData.Out.size() == 1);
	f.PeerData.In.push_back(response);
	reads = f.PeerData.Reads;
	Pump(f, 2);
	assert(f.PeerData.Reads == reads && f.PeerData.In.size() == 1);
	CoapRelease(&f.Dev);
	assert(CoapBusy(&f.Dev));
	assert(!CoapRequest(&f.Dev, &request, 2));
	f.PeerData.Block = false;
	Pump(f, 3);
	assert(f.PeerData.Out.size() == 3);
	assert(f.PeerData.Out[1] == Empty(COAP_ACK, 0x3456));
	assert(f.PeerData.Out[2] == Empty(COAP_ACK, 0x3456));
	assert(f.Dev.State == COAP_READY && !CoapBusy(&f.Dev));
	printf("CoAP ownership: request copies and unsent ACK retained under backpressure PASS\n");
}

static void JitterAndBlockedSend()
{
	Fixture jitter;
	Fixture blocked;
	std::vector<uint8_t> packet;
	CoapRequest_t request = {COAP_CON, COAP_GET, nullptr, 0, nullptr, 0};

	jitter.Rng.Byte = 255;
	packet = Begin(jitter);
	Pump(jitter, 2999);
	assert(jitter.PeerData.Out.size() == 1);
	Pump(jitter, 3000);
	assert(jitter.PeerData.Out.size() == 2 && jitter.PeerData.Out.back() == packet);
	blocked.PeerData.Block = true;
	assert(CoapRequest(&blocked.Dev, &request, 0));
	Pump(blocked, 1000);
	assert(blocked.PeerData.Out.empty() && blocked.Dev.State == COAP_SENDING);
	blocked.PeerData.Block = false;
	Pump(blocked, 1500);
	assert(blocked.PeerData.Out.size() == 1);
	packet = blocked.PeerData.Out[0];
	Pump(blocked, 3499);
	assert(blocked.PeerData.Out.size() == 1);
	Pump(blocked, 3500);
	assert(blocked.PeerData.Out.size() == 2 && blocked.PeerData.Out.back() == packet);
}

static void DuplicateExpiryAndMidRollover()
{
	Fixture expiry;
	Fixture rollover(0, 500000);
	std::vector<uint8_t> packet = Begin(expiry);
	std::vector<uint8_t> response = Reply(packet, COAP_CON, COAP_CONTENT, 911);
	CoapRequest_t request = {COAP_CON, COAP_GET, nullptr, 0, nullptr, 0};

	expiry.PeerData.In.push_back(response);
	Pump(expiry, 1);
	CoapRelease(&expiry.Dev);
	expiry.PeerData.In.push_back(response);
	Pump(expiry, COAP_EXCHANGE_LIFETIME + 1);
	assert(expiry.PeerData.Out.back() == Empty(COAP_RST, 911));
	assert(expiry.Dev.State == COAP_READY);
	packet = Begin(rollover);
	Pump(rollover, 2000);
	assert(rollover.PeerData.Out.size() == 2);
	CoapCancel(&rollover.Dev);
	// Reach the full ID-cycle guard without creating 65536 artificial requests.
	// A retransmission moves the quiet-period origin, not just original sends.
	rollover.Dev.IdCount = 65536;
	assert(!CoapRequest(&rollover.Dev, &request, COAP_EXCHANGE_LIFETIME + 1000));
	assert(rollover.Dev.Error == COAP_ERROR_LIMIT);
	assert(CoapRequest(&rollover.Dev, &request, COAP_EXCHANGE_LIFETIME + 2000));
	Pump(rollover, COAP_EXCHANGE_LIFETIME + 2000);
	assert(rollover.PeerData.Out.size() == 3);
}

static void OptionSemantics()
{
	Fixture repeated;
	std::vector<uint8_t> packet = Begin(repeated);
	Fixture malformed;
	Fixture critical;
	Fixture paths;
	uint8_t dot[] = {'.', '.'};
	uint8_t format = 42;
	CoapOption_t option = {COAP_OPTION_URI_PATH, dot, 1};
	CoapOption_t accept[2] = {{COAP_OPTION_ACCEPT, &format, 1},
							 {COAP_OPTION_ACCEPT, &format, 1}};
	CoapRequest_t request = {COAP_CON, COAP_GET, &option, 1, nullptr, 0};

	// A second nonrepeatable elective occurrence is ignored, preserving first.
	repeated.PeerData.In.push_back(Reply(packet, COAP_ACK, COAP_CONTENT, 0,
									  {0xc1, 42, 0x01, 43}));
	Pump(repeated, 1);
	CheckPayload(repeated);
	assert(repeated.Dev.Message.OptionCount == 1);
	assert(repeated.Dev.Message.Options[0].Number == COAP_OPTION_CONTENT_FORMAT);
	assert(repeated.Dev.Message.Options[0].pValue[0] == 42);
	packet = Begin(malformed);
	// Content-Format length 3 is outside its 0..2 range, so ignore this elective.
	malformed.PeerData.In.push_back(Reply(packet, COAP_ACK, COAP_CONTENT, 0,
									   {0xc3, 0, 0, 42}));
	Pump(malformed, 1);
	CheckPayload(malformed);
	assert(malformed.Dev.Message.OptionCount == 0);
	packet = Begin(critical);
	critical.PeerData.In.push_back(Reply(packet, COAP_ACK, COAP_CONTENT, 0,
									  {0xd1, 10, 0}));
	Pump(critical, 1);
	assert(critical.Dev.State == COAP_FAILED && critical.Dev.Error == COAP_ERROR_OPTION);
	assert(critical.PeerData.Out.size() == 1);
	assert(!CoapRequest(&paths.Dev, &request, 0));
	assert(paths.Dev.Error == COAP_ERROR_OPTION);
	option.Len = 2;
	assert(!CoapRequest(&paths.Dev, &request, 0));
	option.Len = 0;
	assert(CoapRequest(&paths.Dev, &request, 0));
	CoapCancel(&paths.Dev);
	request.pOptions = accept;
	request.OptionCount = 2;
	assert(!CoapRequest(&paths.Dev, &request, 0));
	assert(paths.Dev.Error == COAP_ERROR_OPTION);
	printf("CoAP options: singleton/length handling, critical rejection and path validity PASS\n");
}

static void FullDatagramAndTransportFailure()
{
	Fixture full;
	std::vector<uint8_t> packet = Begin(full);
	std::vector<uint8_t> payload(COAP_MESSAGE_SIZE - 4U - COAP_TOKEN_SIZE - 1U, 0x56);
	std::vector<uint8_t> response = Reply(packet, COAP_ACK, COAP_CONTENT, 0, {}, payload);
	const uint8_t *view;
	int fault;

	assert(response.size() == COAP_MESSAGE_SIZE);
	full.PeerData.In.push_back(response);
	Pump(full, 1);
	assert(full.Dev.State == COAP_RESPONSE);
	assert(full.Dev.Message.PayloadLen == payload.size());
	assert(memcmp(full.Dev.Message.pPayload, payload.data(), payload.size()) == 0);
	view = full.Dev.Message.pPayload;
	CoapProcess(&full.Dev, 2, false);
	assert(full.Dev.State == COAP_RESPONSE && full.Dev.Error == COAP_ERROR_TRANSPORT);
	assert(full.Dev.Message.pPayload == view);
	full.PeerData.ReadError = -1;
	Pump(full, 2);
	assert(full.Dev.State == COAP_RESPONSE && full.Dev.Error == COAP_ERROR_TRANSPORT);
	assert(full.Dev.Message.pPayload == view);
	assert(memcmp(view, payload.data(), payload.size()) == 0);
	assert(!atomic_flag_test_and_set(&full.PeerData.Dev.bBusy));
	atomic_flag_clear(&full.PeerData.Dev.bBusy);
	full.PeerData.ReadError = 0;
	CoapRelease(&full.Dev);
	full.PeerData.Out.clear();
	packet = Begin(full, 3);
	payload.push_back(0x56);
	response = Reply(packet, COAP_ACK, COAP_CONTENT, 0, {}, payload);
	assert(response.size() == COAP_MESSAGE_SIZE + 1U);
	full.PeerData.In.push_back(response);
	Pump(full, 4);
	assert(full.Dev.State == COAP_FAILED && full.Dev.Error == COAP_ERROR_LIMIT);
	for (fault = -1; fault <= 1; fault += 2)
	{
		Fixture badRead;
		std::vector<uint8_t> initial = Begin(badRead);

		badRead.PeerData.ReadError = fault;
		Pump(badRead, 1);
		assert(badRead.Dev.State == COAP_FAILED && badRead.Dev.Error == COAP_ERROR_TRANSPORT);
		assert(!atomic_flag_test_and_set(&badRead.PeerData.Dev.bBusy));
		atomic_flag_clear(&badRead.PeerData.Dev.bBusy);
		(void)initial;
	}
	{
		Fixture control;
		std::vector<uint8_t> initial = Begin(control);
		const uint8_t *held;

		control.PeerData.Block = true;
		control.PeerData.In.push_back(Reply(initial, COAP_CON, COAP_CONTENT, 777));
		Pump(control, 1);
		CheckPayload(control);
		held = control.Dev.Message.pPayload;
		control.PeerData.WriteError = -1;
		Pump(control, 2);
		assert(control.Dev.State == COAP_RESPONSE && control.Dev.Error == COAP_ERROR_TRANSPORT);
		assert(control.Dev.Message.pPayload == held);
		assert(memcmp(held, "o\0k", 3) == 0);
		assert(!atomic_flag_test_and_set(&control.PeerData.Dev.bBusy));
		atomic_flag_clear(&control.PeerData.Dev.bBusy);
	}
	printf("CoAP limits: full datagram, held response after transport failure, released locks PASS\n");
}

static void ReservedLocationOptions()
{
	const uint16_t reserved[] = {128, 132, 136, 140};
	const uint16_t location[] = {COAP_OPTION_LOCATION_PATH, COAP_OPTION_LOCATION_QUERY};
	size_t i;
	size_t j;
	COAP_TYPE type;

	for (i = 0; i < sizeof(reserved) / sizeof(reserved[0]); i++)
	{
		for (j = 0; j < sizeof(location) / sizeof(location[0]); j++)
		{
			for (type = COAP_CON; type <= COAP_ACK; type = (COAP_TYPE)(type + 1))
			{
				Fixture f;
				std::vector<uint8_t> request = Begin(f, 0, COAP_CON, COAP_POST);
				std::vector<uint8_t> options;
				size_t expectedWrites = type == COAP_CON ? 2U : 1U;

				if (location[j] == COAP_OPTION_LOCATION_PATH)
				{
					options = {0x81, 'r'};
				}
				else
				{
					options = {0xd1, 7, 'q'};
				}
				options.push_back(0xd0);
				options.push_back((uint8_t)(reserved[i] - location[j] - 13U));
				f.PeerData.In.push_back(Reply(request, type, COAP_CREATED, 888, options, {}));
				Pump(f, 1);
				assert(f.Dev.State == COAP_FAILED && f.Dev.Error == COAP_ERROR_OPTION);
				assert(f.PeerData.Out.size() == expectedWrites);
				if (type == COAP_CON)
				{
					assert(f.PeerData.Out.back() == Empty(COAP_RST, 888));
				}
			}
		}
		{
			Fixture alone;
			std::vector<uint8_t> request = Begin(alone);

			// Without Location-*, the same even-numbered unknown option is elective.
			alone.PeerData.In.push_back(Reply(request, COAP_ACK, COAP_CONTENT, 0,
											{0xd0, (uint8_t)(reserved[i] - 13U)}));
			Pump(alone, 1);
			CheckPayload(alone);
		}
	}
	printf("CoAP erratum 4949: unsupported Location extensions reject CON/NON/ACK PASS\n");
}

static void ProbePacing()
{
	Fixture non(0, 1000, 0);
	std::vector<uint8_t> packet = Begin(non, 0, COAP_NON);
	CoapRequest_t request = {COAP_NON, COAP_GET, nullptr, 0, nullptr, 0};
	uint32_t debt = (uint32_t)packet.size() * 1000U;

	assert(CoapRetryAfter(&non.Dev, 0) == debt);
	Pump(non, 1000);
	assert(non.Dev.State == COAP_FAILED && non.Dev.Error == COAP_ERROR_TIMEOUT);
	CoapRelease(&non.Dev);
	assert(!CoapRequest(&non.Dev, &request, 1000));
	assert(non.Dev.State == COAP_READY && non.Dev.Error == COAP_ERROR_RATE);
	assert(!CoapBusy(&non.Dev));
	assert(CoapRetryAfter(&non.Dev, debt - 1U) == 1);
	assert(CoapRequest(&non.Dev, &request, debt));
	Pump(non, debt);
	packet = non.PeerData.Out.back();
	assert(CoapRetryAfter(&non.Dev, debt) == packet.size() * 1000U);
	non.PeerData.In.push_back(Reply(packet, COAP_NON, COAP_CONTENT, 810));
	Pump(non, debt + 1U);
	CheckPayload(non);
	assert(CoapRetryAfter(&non.Dev, debt + 1U) == 0);
	CoapRelease(&non.Dev);
	assert(CoapRequest(&non.Dev, &request, debt + 1U));
	{
		Fixture cancelled(0, 1000, 1);
		std::vector<uint8_t> original = Begin(cancelled, 0, COAP_NON);

		CoapCancel(&cancelled.Dev);
		assert(!CoapRequest(&cancelled.Dev, &request, 1));
		assert(cancelled.Dev.Error == COAP_ERROR_RATE);
		assert(CoapRetryAfter(&cancelled.Dev, 1) == original.size() * 1000U - 1U);
		CoapRelease(&cancelled.Dev);
		assert(!CoapRequest(&cancelled.Dev, &request, 2));
	}
	{
		Fixture blocked(0, 1000, 1);

		blocked.PeerData.Block = true;
		assert(CoapRequest(&blocked.Dev, &request, 0));
		Pump(blocked, 500);
		assert(CoapRetryAfter(&blocked.Dev, 500) == 0);
		CoapCancel(&blocked.Dev);
		assert(CoapRequest(&blocked.Dev, &request, 500));
		blocked.PeerData.Block = false;
		Pump(blocked, 500);
		assert(blocked.PeerData.Out.size() == 1);
		assert(CoapRetryAfter(&blocked.Dev, 500) == blocked.PeerData.Out[0].size() * 1000U);
	}
	{
		Fixture failed(0, 1000, 1);

		failed.PeerData.WriteError = -1;
		assert(CoapRequest(&failed.Dev, &request, 0));
		Pump(failed, 0);
		assert(failed.Dev.Error == COAP_ERROR_TRANSPORT);
		assert(CoapRetryAfter(&failed.Dev, 0) == 0);
		CoapRelease(&failed.Dev);
		assert(CoapRequest(&failed.Dev, &request, 0));
	}
	{
		Fixture retry(0, 100000, 1);
		std::vector<uint8_t> original = Begin(retry);
		uint32_t total;

		Pump(retry, 2000);
		Pump(retry, 6000);
		assert(retry.PeerData.Out.size() == 3);
		total = (uint32_t)original.size() * 3000U;
		assert(CoapRetryAfter(&retry.Dev, 6000) == total - 6000U);
		retry.PeerData.In.push_back(Empty(COAP_ACK, MessageId(original)));
		Pump(retry, 6001);
		assert(retry.Dev.State == COAP_WAIT_RESPONSE);
		assert(CoapRetryAfter(&retry.Dev, 6001) == total - 6001U);
		retry.PeerData.In.push_back(Reply(original, COAP_CON, COAP_CONTENT, 811));
		Pump(retry, 6002);
		CheckPayload(retry);
		assert(CoapRetryAfter(&retry.Dev, 6002) == 0);
	}
	{
		Fixture wrapped(0xfffffff0U, 1000, 1);
		std::vector<uint8_t> original = Begin(wrapped, 0xfffffff0U, COAP_NON);
		uint32_t deadline = 0xfffffff0U + (uint32_t)original.size() * 1000U;

		Pump(wrapped, 0x3d8U);
		CoapRelease(&wrapped.Dev);
		assert(!CoapRequest(&wrapped.Dev, &request, 0x3d8U));
		assert(CoapRetryAfter(&wrapped.Dev, deadline - 1U) == 1);
		assert(CoapRequest(&wrapped.Dev, &request, deadline));
	}
	{
		Fixture reset(0, 1000, 1);
		std::vector<uint8_t> original = Begin(reset);

		reset.PeerData.In.push_back(Empty(COAP_RST, MessageId(original)));
		Pump(reset, 1);
		assert(reset.Dev.Error == COAP_ERROR_RESET);
		assert(CoapRetryAfter(&reset.Dev, 1) == 0);
		CoapRelease(&reset.Dev);
		assert(CoapRequest(&reset.Dev, &request, 1));
	}
	printf("CoAP pacing: retained byte debt, release/cancel, empty ACK, retransmits and wrap PASS\n");
}

class PeerInterface : public DeviceIntrf
{
public:
	Peer *pPeer;

	explicit PeerInterface(Peer &Peer) : pPeer(&Peer)
	{
	}
	operator DevIntrf_t *() override
	{
		return &pPeer->Dev;
	}
	uint32_t Rate(uint32_t Value) override
	{
		return Value;
	}
	uint32_t Rate() override
	{
		return 0;
	}
};

static void WrapperLifecycle()
{
	Peer peer;
	PeerInterface transport(peer);
	TestRng rng;
	Coap coap;
	CoapCfg_t cfg = {&rng, 1000, 1152000};
	CoapRequest_t request = {COAP_CON, COAP_GET, nullptr, 0, nullptr, 0};

	Setup(peer);
	assert(coap.Init(cfg, &transport, 0));
	assert(coap.Valid());
	assert(coap.Request(request, 0));
	coap.Disable();
	coap.Process(1, true);
	assert(peer.Out.empty());
	assert(!coap.Request(request, 1));
	assert(coap.Enable());
	coap.Process(2, true);
	assert(peer.Out.size() == 1);
	peer.In.push_back(Reply(peer.Out[0]));
	coap.Process(3, true);
	assert(coap.Response() && coap.Response()->Code == COAP_CONTENT);
	coap.Release();
	assert(!coap.Response());
	assert(coap.Request(request, 4));
	coap.Cancel();
	assert(!coap.Busy());
	coap.PowerOff();
	assert(!coap.Valid() && !coap.Enable());
	printf("CoAP C++ wrapper: same engine, pause/resume, ownership and reset PASS\n");
}

int main()
{
	CodecFixtures();
	CodecErrors();
	ParserBounds();
	MatchingAndOwnership();
	SeparateAndDuplicate();
	TypePermutations();
	Retransmission();
	ResetCancelAndWrap();
	DatagramBoundaries();
	ProtocolRejection();
	TransportAndConfigErrors();
	RequestStorageAndControlPressure();
	JitterAndBlockedSend();
	DuplicateExpiryAndMidRollover();
	OptionSemantics();
	FullDatagramAndTransportFailure();
	ReservedLocationOptions();
	ProbePacing();
	WrapperLifecycle();
	printf("CoAP host wire and datagram fault tests: PASS\n");
	return 0;
}

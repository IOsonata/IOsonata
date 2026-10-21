/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Independent aiocoap interoperability through a host-only UDP DeviceIntrf.
 */
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include "net/coap.h"

class HostRng : public RngEngine
{
public:
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
	bool IsSecure() const override
	{
		return true;
	}
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override
	{
		size_t offset = 0;
		ssize_t count;

		while (offset < Len)
		{
			count = getrandom(pOut + offset, Len - offset, 0);
			if (count < 0 && errno == EINTR)
			{
				continue;
			}
			if (count <= 0)
			{
				return CRYPTO_STATUS_FAIL;
			}
			offset += (size_t)count;
		}
		return CRYPTO_STATUS_OK;
	}
};

struct Transport
{
	DevIntrf_t Dev = {};
	int Fd = -1;
	bool Open = true;
	unsigned EmptyAcks = 0;
	unsigned SeparateResponses = 0;
};

static bool Start(DevIntrf_t *, uint32_t)
{
	return true;
}

static void Stop(DevIntrf_t *)
{
}

static int Read(DevIntrf_t *pDev, uint8_t *pOut, int Len)
{
	Transport *p = (Transport *)pDev->pDevData;
	int count;

	// recv consumes exactly one datagram and drops any excess. The production
	// client reserves one extra byte so an oversized response is never accepted.
	count = recv(p->Fd, pOut, (size_t)Len, 0);
	if (count < 0)
	{
		if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
		{
			return 0;
		}
		p->Open = false;
		return -1;
	}
	if (count >= 4 && (pOut[0] & 0x30U) == 0 && pOut[1] >= 64)
	{
		p->SeparateResponses++;
	}
	return count;
}

static int Write(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	Transport *p = (Transport *)pDev->pDevData;
	int count;

	count = send(p->Fd, pData, (size_t)Len, MSG_NOSIGNAL);
	if (count < 0)
	{
		if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
		{
			return 0;
		}
		p->Open = false;
		return -1;
	}
	if (count == 4 && pData[0] == 0x60 && pData[1] == 0)
	{
		p->EmptyAcks++;
	}
	return count;
}

static uint32_t Now()
{
	using namespace std::chrono;
	return (uint32_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static void Pump(CoapDev_t &Dev, Transport &Peer)
{
	CoapProcess(&Dev, Now(), Peer.Open);
	assert(Dev.Error == COAP_ERROR_NONE);
	usleep(1000);
}

static void Exchange(CoapDev_t &Dev, Transport &Peer, uint8_t Method, const char *pPath,
					 const uint8_t *pPayload, size_t PayloadLen, uint8_t ExpectedCode,
					 const uint8_t *pExpected, size_t ExpectedLen, bool Non = false)
{
	const uint8_t format = 42;
	CoapOption_t options[2] = {{11, (const uint8_t *)pPath, strlen(pPath)}, {12, &format, 1}};
	CoapRequest_t request = {(COAP_TYPE)(Non ? 1 : 0), Method, options,
							 PayloadLen ? 2U : 1U, pPayload, PayloadLen};
	uint32_t started = Now();

	assert(CoapRequest(&Dev, &request, started));
	do
	{
		Pump(Dev, Peer);
	} while (Dev.State != COAP_RESPONSE && Dev.State != COAP_FAILED &&
			 Now() - started < 12000);
	assert(Dev.State == COAP_RESPONSE);
	assert(Dev.Message.Code == ExpectedCode);
	if (pExpected != nullptr)
	{
		assert(Dev.Message.PayloadLen == ExpectedLen);
		if (ExpectedLen)
		{
			assert(memcmp(Dev.Message.pPayload, pExpected, ExpectedLen) == 0);
		}
	}
	CoapRelease(&Dev);
}

int main(int argc, char **argv)
{
	Transport peer;
	CoapDev_t dev = {};
	HostRng rng;
	CoapCfg_t cfg = {&rng, 15000, 0};
	sockaddr_in address = {};
	uint8_t payload[1000];
	uint32_t started;
	size_t i;
	bool loss;

	assert(argc == 2 || argc == 3);
	loss = argc == 3;
	peer.Fd = socket(AF_INET, SOCK_DGRAM, 0);
	assert(peer.Fd >= 0);
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	address.sin_port = htons((uint16_t)atoi(argv[1]));
	assert(connect(peer.Fd, (sockaddr *)&address, sizeof(address)) == 0);
	assert(fcntl(peer.Fd, F_SETFL, O_NONBLOCK) == 0);
	peer.Dev.pDevData = &peer;
	peer.Dev.StartRx = peer.Dev.StartTx = Start;
	peer.Dev.StopRx = peer.Dev.StopTx = Stop;
	peer.Dev.RxData = Read;
	peer.Dev.TxData = Write;
	peer.Dev.EnCnt = 1;
	atomic_flag_clear(&peer.Dev.bBusy);
	assert(CoapInit(&dev, &cfg, &peer.Dev, Now()));
	for (i = 0; i < sizeof(payload); i++)
	{
		payload[i] = (uint8_t)(i * 73U);
	}
	Exchange(dev, peer, 1, "hello", nullptr, 0, 69,
			 (const uint8_t *)"IOsonata CoAP\n", 14);
	Exchange(dev, peer, 2, "echo", payload, sizeof(payload), 69, payload, sizeof(payload));
	Exchange(dev, peer, 2, "echo", payload, 257, 69, payload, 257, true);
	Exchange(dev, peer, 3, "value", payload, 513, 68, (const uint8_t *)"", 0);
	Exchange(dev, peer, 1, "value", nullptr, 0, 69, payload, 513);
	Exchange(dev, peer, 4, "value", nullptr, 0, 66, (const uint8_t *)"", 0);
	Exchange(dev, peer, 1, "value", nullptr, 0, 132, nullptr, 0);
	Exchange(dev, peer, 1, "missing", nullptr, 0, 132, nullptr, 0);
	Exchange(dev, peer, 1, "slow", nullptr, 0, 69,
			 (const uint8_t *)"separate response\n", 18);
	assert(peer.SeparateResponses >= 1 && peer.EmptyAcks >= 1);
	if (loss)
	{
		// The proxy drops the first ACK. Keep pumping after Release so the
		// server's retransmitted CON is acknowledged without another delivery.
		started = Now();
		while (Now() - started < 4000)
		{
			Pump(dev, peer);
			assert(dev.State == COAP_READY);
		}
		assert(peer.SeparateResponses >= 2 && peer.EmptyAcks >= 2);
	}
	CoapReset(&dev);
	close(peer.Fd);
	printf("aiocoap %s: CON/NON binary echo, all methods, 4.04, separate response: PASS\n",
		   loss ? "with loss" : "direct");
	return 0;
}

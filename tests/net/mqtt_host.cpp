/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Real Mosquitto interoperability through a host-only DeviceIntrf adapter.
 */
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <chrono>
#include "net/mqtt.h"
#include "net/tls.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"
#include <sys/random.h>

static TlsDev_t s_Tls;
static bool s_Secure;
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
	CRYPTO_STATUS Random(uint8_t *p, size_t n) override
	{
		return getrandom(p, n, 0) == (ssize_t)n ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
};
struct Transport
{
	DevIntrf_t Dev = {};
	int Fd;
	bool Open = true;
};
static bool Start(DevIntrf_t *, uint32_t)
{
	return true;
}
static void Stop(DevIntrf_t *)
{
}
static int Read(DevIntrf_t *p, uint8_t *out, int n)
{
	Transport *t = (Transport *)p->pDevData;
	int r = recv(t->Fd, out, n > 7 ? 7 : n, 0);
	if (!r || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
	{
		t->Open = false;
	}
	return r < 0 ? 0 : r;
}
static int Write(DevIntrf_t *p, const uint8_t *in, int n)
{
	Transport *t = (Transport *)p->pDevData;
	int r = send(t->Fd, in, n > 7 ? 7 : n, MSG_NOSIGNAL);
	if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
	{
		t->Open = false;
	}
	return r < 0 ? 0 : r;
}
static uint32_t Now()
{
	using namespace std::chrono;
	return (uint32_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
static void Pump(MqttDev_t &d, Transport &t)
{
	if (s_Secure)
	{
		assert(TlsProcess(&s_Tls, Now(), t.Open) == TLS_READY);
	}
	MqttProcess(&d, Now(), s_Secure ? s_Tls.State == TLS_READY : t.Open);
	assert(d.State != MQTT_FAILED);
	usleep(100);
}
static void Wait(MqttDev_t &d, Transport &t)
{
	uint32_t start = Now();
	while (MqttBusy(&d) && Now() - start < 5000)
	{
		Pump(d, t);
	}
	assert(!MqttBusy(&d));
}
int main(int argc, char **argv)
{
	assert(argc == 2 || argc == 3);
	s_Secure = argc == 3;
	for (int repeat = 0; repeat < (s_Secure ? 1 : 3); repeat++)
	{
		Transport t;
		MqttDev_t d = {};
		MqttCfg_t cfg = {"iosonata-host-test", nullptr, nullptr, 0, 1, 3000};
		sockaddr_in addr = {};
		uint8_t payload[900];
		uint32_t start;
		CryptoSoftSha256 hash;
		CryptoSoftAes aes;
		CryptoUecc agree;
		HostRng rng;
		uint8_t psk[32];
		const uint8_t identity[] = "IOsonata";
		TlsCfg_t tlsCfg = {&hash,	&hash, &aes,  &rng, psk, sizeof(psk), identity, sizeof(identity) - 1,
						   nullptr, 5000,  &agree, nullptr};
		t.Fd = socket(AF_INET, SOCK_STREAM, 0);
		assert(t.Fd >= 0);
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		addr.sin_port = htons((uint16_t)atoi(argv[1]));
		start = Now();
		while (connect(t.Fd, (sockaddr *)&addr, sizeof(addr)) != 0)
		{
			assert(Now() - start < 3000);
			close(t.Fd);
			usleep(20000);
			t.Fd = socket(AF_INET, SOCK_STREAM, 0);
			assert(t.Fd >= 0);
		}
		fcntl(t.Fd, F_SETFL, O_NONBLOCK);
		t.Dev.pDevData = &t;
		t.Dev.StartRx = t.Dev.StartTx = Start;
		t.Dev.StopRx = t.Dev.StopTx = Stop;
		t.Dev.RxData = Read;
		t.Dev.TxData = Write;
		t.Dev.EnCnt = 1;
		atomic_flag_clear(&t.Dev.bBusy);
		if (s_Secure)
		{
			hash.Enable();
			aes.Enable();
			agree.SetRng(&rng);
			agree.Enable();
			for (size_t i = 0; i < sizeof(psk); i++)
			{
				psk[i] = (uint8_t)i;
			}
			assert(TlsInit(&s_Tls, &tlsCfg, &t.Dev, Now()));
			start = Now();
			while (s_Tls.State != TLS_READY && s_Tls.State != TLS_FAILED && Now() - start < 6000)
			{
				TlsProcess(&s_Tls, Now(), t.Open);
				usleep(100);
			}
			printf("MQTT TLS state=%d error=%d\n", s_Tls.State, s_Tls.Error);
			assert(s_Tls.State == TLS_READY);
		}
		assert(MqttInit(&d, &cfg, s_Secure ? &s_Tls.DevIntrf : &t.Dev, Now()));
		Wait(d, t);
		assert(MqttSubscribe(&d, "iosonata/test", 1, Now()));
		Wait(d, t);
		assert(d.Result == 1);
		for (unsigned qos = 0; qos < 2; qos++)
		{
			for (size_t i = 0; i < sizeof(payload); i++)
			{
				payload[i] = (uint8_t)(i + qos);
			}
			assert(MqttPublish(&d, "iosonata/test", payload, sizeof(payload), qos, false, Now()));
			start = Now();
			while (!d.bMessage && Now() - start < 5000)
			{
				Pump(d, t);
			}
			assert(d.bMessage && d.Message.PayloadLen == sizeof(payload) && d.Message.Qos == qos);
			assert(memcmp(d.Message.pPayload, payload, sizeof(payload)) == 0);
			MqttRelease(&d);
			Wait(d, t);
		}
		// Stay idle through several PINGREQ/PINGRESP cycles.
		start = Now();
		while (Now() - start < 2500)
		{
			Pump(d, t);
		}
		assert(d.State == MQTT_READY);
		Wait(d, t);
		assert(MqttUnsubscribe(&d, "iosonata/test", Now()));
		Wait(d, t);
		assert(MqttDisconnect(&d, Now()));
		while (d.State == MQTT_CLOSING)
		{
			Pump(d, t);
		}
		assert(d.State == MQTT_CLOSED);
		if (s_Secure)
		{
			while (s_Tls.TxLen)
			{
				TlsProcess(&s_Tls, Now(), t.Open);
			}
			assert(TlsClose(&s_Tls));
			while (s_Tls.State == TLS_CLOSING)
			{
				TlsProcess(&s_Tls, Now(), t.Open);
			}
			assert(s_Tls.State == TLS_CLOSED);
			TlsReset(&s_Tls);
		}
		close(t.Fd);
		MqttReset(&d);
	}
	printf(
		"%s Mosquitto: QoS 0/1 binary echo, subscribe/unsubscribe, keepalive, partial I/O: PASS\n",
		s_Secure ? "TLS" : "TCP");
}

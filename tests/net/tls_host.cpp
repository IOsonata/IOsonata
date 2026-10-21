/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Host transport only; production TLS and crypto sources are linked unchanged.
 */
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/random.h>
#include <netinet/in.h>
#include <unistd.h>
#include <chrono>
#include <initializer_list>
#include "net/tls.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

class HostRng : public RngEngine {
public:
	bool Enable() override { return true; }
	void Disable() override {}
	void Reset() override {}
	bool IsSecure() const override { return true; }
	CRYPTO_STATUS Random(uint8_t *out, size_t n) override
	{
		return getrandom(out, n, 0) == (ssize_t)n ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
};
struct Transport {
	DevIntrf_t Dev = {};
	int Fd = -1;
	int Chunk = 65536;
	bool Open = true;
};
static bool Start(DevIntrf_t *, uint32_t) { return true; }
static void Stop(DevIntrf_t *) {}
static int Read(DevIntrf_t *p, uint8_t *out, int n)
{
	Transport *t = (Transport *)p->pDevData;
	if (n > t->Chunk) n = t->Chunk;
	int r = recv(t->Fd, out, n, 0);
	if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) t->Open = false;
	return r < 0 ? 0 : r;
}
static int Write(DevIntrf_t *p, const uint8_t *in, int n)
{
	Transport *t = (Transport *)p->pDevData;
	if (n > t->Chunk) n = t->Chunk;
	int r = send(t->Fd, in, n, MSG_NOSIGNAL);
	if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) t->Open = false;
	return r < 0 ? 0 : r;
}
static uint32_t Now()
{
	using namespace std::chrono;
	return (uint32_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
int main(int argc, char **argv)
{
	assert(argc >= 2);
	uint64_t previousKey[8] = {};
	int repeats = argc > 5 ? atoi(argv[5]) : 2;
	assert(repeats > 0 && repeats <= 1000);
	for (int repeat = 0; repeat < repeats; repeat++)
	{
	Transport t;
	t.Chunk = argc > 2 ? atoi(argv[2]) : 65536;
	t.Fd = socket(AF_INET, SOCK_STREAM, 0);
	assert(t.Fd >= 0);
	sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons((uint16_t)atoi(argv[1]));
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	assert(connect(t.Fd, (sockaddr *)&addr, sizeof(addr)) == 0);
	fcntl(t.Fd, F_SETFL, O_NONBLOCK);
	t.Dev.pDevData = &t;
	t.Dev.StartRx = t.Dev.StartTx = Start;
	t.Dev.StopRx = t.Dev.StopTx = Stop;
	t.Dev.RxData = Read;
	t.Dev.TxData = Write;
	t.Dev.EnCnt = 1;
	atomic_flag_clear(&t.Dev.bBusy);
	CryptoSoftSha256 hash;
	CryptoSoftAes aes;
	HostRng rng;
	CryptoUecc agree;
	agree.SetRng(&rng);
	agree.Enable();
	bool dhe = argc > 3 && strcmp(argv[3], "dhe") == 0;
	bool bad = argc > 4 && strcmp(argv[4], "bad-key") == 0;
	hash.Enable();
	aes.Enable();
	uint8_t psk[32];
	for (unsigned i = 0; i < sizeof(psk); i++) psk[i] = (uint8_t)i;
	if (bad) psk[0] ^= 1;
	const uint8_t identity[] = "IOsonata";
	TlsCfg_t cfg = {&hash, &hash, &aes, &rng, psk, sizeof(psk),
		identity, sizeof(identity) - 1, nullptr, 5000, dhe ? &agree : nullptr};
	static TlsDev_t tls = {};
	assert(TlsInit(&tls, &cfg, &t.Dev, Now()));
	if (dhe)
	{
		assert(memcmp(previousKey, tls.KeyCtx, sizeof(previousKey)) != 0);
		memcpy(previousKey, tls.KeyCtx, sizeof(previousKey));
	}
	uint32_t start = Now();
	while (tls.State != TLS_READY && tls.State != TLS_FAILED && Now() - start < 6000)
	{
		TlsProcess(&tls, Now(), t.Open);
		usleep(100);
	}
	printf("%s handshake state=%d error=%d storage=%zu\n",
		dhe ? "PSK-DHE" : "PSK-only", tls.State, tls.Error, sizeof(tls));
	if (bad)
	{
		assert(tls.State == TLS_FAILED);
		close(t.Fd);
		return 0;
	}
	assert(tls.State == TLS_READY);
	for (auto c : tls.KeyCtx) assert(c == 0);
	for (size_t length : {19U, 1024U, 4096U, 16000U})
	{
		uint8_t msg[16000];
		uint8_t result[16000] = {};
		for (size_t i = 0; i < length - 1; i++)
		{
			msg[i] = (uint8_t)('A' + i % 26);
		}
		msg[length - 1] = '\n';
		size_t sent = 0;
		size_t got = 0;
		start = Now();
		while (got < length && tls.State != TLS_FAILED && Now() - start < 10000)
		{
			TlsProcess(&tls, Now(), t.Open);
			if (sent < length)
			{
				sent += DeviceIntrfTx(&tls.DevIntrf, 0, msg + sent, length - sent);
			}
			got += DeviceIntrfRx(&tls.DevIntrf, 0, result + got, length - got);
			usleep(50);
		}
		assert(got == length && sent == length);
		for (size_t i = 0; i < length - 1; i++)
		{
			assert(result[i] == msg[length - 2 - i]);
		}
		assert(result[length - 1] == '\n');
		printf("OpenSSL reverse echo PASS %zu bytes\n", got);
	}
	assert(TlsClose(&tls));
	while (tls.State == TLS_CLOSING) TlsProcess(&tls, Now(), t.Open);
	assert(tls.State == TLS_CLOSED);
	TlsReset(&tls);
	close(t.Fd);
	}
}

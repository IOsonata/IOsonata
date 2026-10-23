/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Localhost transport for the unmodified shared TLS and crypto sources.
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
#include <vector>

#include "net/tls.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

class HostRng : public RngEngine {
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
		return getrandom(pOut, Len, 0) == (ssize_t)Len ?
			CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
};

struct Transport {
	DevIntrf_t Dev = {};
	int Fd = -1;
	int Chunk = 65536;
	bool Open = true;
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
	Transport *pTransport = (Transport *)pDev->pDevData;
	int count;

	if (Len > pTransport->Chunk)
	{
		Len = pTransport->Chunk;
	}
	count = recv(pTransport->Fd, pOut, Len, 0);
	if (count == 0 || (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
	{
		pTransport->Open = false;
	}
	return count < 0 ? 0 : count;
}

static int Write(DevIntrf_t *pDev, const uint8_t *pIn, int Len)
{
	Transport *pTransport = (Transport *)pDev->pDevData;
	int count;

	if (Len > pTransport->Chunk)
	{
		Len = pTransport->Chunk;
	}
	count = send(pTransport->Fd, pIn, Len, MSG_NOSIGNAL);
	if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
	{
		pTransport->Open = false;
	}
	return count < 0 ? 0 : count;
}

static uint32_t Now()
{
	using namespace std::chrono;
	return (uint32_t)duration_cast<milliseconds>(
		steady_clock::now().time_since_epoch()).count();
}

static std::vector<uint8_t> ReadFile(const char *pPath)
{
	FILE *pFile = fopen(pPath, "rb");
	long length;
	std::vector<uint8_t> data;

	assert(pFile != nullptr);
	assert(fseek(pFile, 0, SEEK_END) == 0);
	length = ftell(pFile);
	assert(length > 0 && length <= X509_MAX_CERT_SIZE);
	assert(fseek(pFile, 0, SEEK_SET) == 0);
	data.resize((size_t)length);
	assert(fread(data.data(), 1, data.size(), pFile) == data.size());
	assert(fclose(pFile) == 0);
	return data;
}

static void Echo(TlsDev_t &Tls, Transport &Stream, size_t Length)
{
	uint8_t message[16000];
	uint8_t result[16000] = {};
	size_t sent = 0;
	size_t received = 0;
	uint32_t start = Now();
	int count;

	assert(Length > 1 && Length <= sizeof(message));
	for (size_t i = 0; i < Length - 1; i++)
	{
		message[i] = (uint8_t)('A' + i % 26);
	}
	message[Length - 1] = '\n';
	while (received < Length && Tls.State != TLS_FAILED && Now() - start < 10000)
	{
		TlsProcess(&Tls, Now(), Stream.Open);
		if (sent < Length)
		{
			count = DeviceIntrfTx(&Tls.DevIntrf, 0, message + sent, Length - sent);
			assert(count >= 0);
			sent += (size_t)count;
		}
		count = DeviceIntrfRx(&Tls.DevIntrf, 0, result + received, Length - received);
		assert(count >= 0);
		received += (size_t)count;
		usleep(50);
	}
	assert(received == Length && sent == Length);
	for (size_t i = 0; i < Length - 1; i++)
	{
		assert(result[i] == message[Length - 2 - i]);
	}
	assert(result[Length - 1] == '\n');
}

int main(int argc, char **argv)
{
	static TlsDev_t tls = {};
	static uint8_t handshake[8192];
	static uint8_t transcript[8192];
	std::vector<uint8_t> root;
	uint64_t previousKey[8] = {};
	int repeats;
	bool certificateFailure;
	bool limitFailure;

	assert(argc == 9);
	root = ReadFile(argv[3]);
	repeats = atoi(argv[7]);
	certificateFailure = strcmp(argv[6], "certificate") == 0;
	limitFailure = strcmp(argv[6], "limit") == 0;
	assert(certificateFailure || limitFailure || strcmp(argv[6], "ok") == 0);
	assert(repeats > 0 && repeats <= 1000);
	for (int repeat = 0; repeat < repeats; repeat++)
	{
		Transport stream;
		sockaddr_in address = {};
		CryptoSoftSha256 hash;
		CryptoSoftAes aes;
		HostRng rng;
		CryptoUecc ecc;
		X509Cert_t anchor = {root.data(), (uint32_t)root.size()};
		TlsCertCfg_t cert = {};
		TlsCfg_t config = {};
		uint32_t start;
		uint8_t sample = 0;

		stream.Chunk = atoi(argv[2]);
		assert(stream.Chunk > 0);
		stream.Fd = socket(AF_INET, SOCK_STREAM, 0);
		assert(stream.Fd >= 0);
		address.sin_family = AF_INET;
		address.sin_port = htons((uint16_t)atoi(argv[1]));
		address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		assert(connect(stream.Fd, (sockaddr *)&address, sizeof(address)) == 0);
		assert(fcntl(stream.Fd, F_SETFL, O_NONBLOCK) == 0);
		stream.Dev.pDevData = &stream;
		stream.Dev.StartRx = stream.Dev.StartTx = Start;
		stream.Dev.StopRx = stream.Dev.StopTx = Stop;
		stream.Dev.RxData = Read;
		stream.Dev.TxData = Write;
		stream.Dev.EnCnt = 1;
		atomic_flag_clear(&stream.Dev.bBusy);
		ecc.SetRng(&rng);
		ecc.Enable();
		hash.Enable();
		aes.Enable();
		cert.pSign = &ecc;
		cert.pTrustAnchors = &anchor;
		cert.TrustAnchorCount = 1;
		cert.UnixTime = strtoull(argv[5], nullptr, 10);
		if (strcmp(argv[8], "external") == 0)
		{
			cert.pHandshakeBuffer = handshake;
			cert.HandshakeBufferSize = sizeof(handshake);
			cert.pTranscriptBuffer = transcript;
			cert.TranscriptBufferSize = sizeof(transcript);
		}
		config.pHash = &hash;
		config.pMac = &hash;
		config.pAead = &aes;
		config.pRng = &rng;
		config.pServerName = argv[4];
		config.Timeout = 5000;
		config.pKeyAgree = &ecc;
		config.pCert = &cert;
		assert(TlsInit(&tls, &config, &stream.Dev, Now()));
		assert(memcmp(previousKey, tls.KeyCtx, sizeof(previousKey)) != 0);
		memcpy(previousKey, tls.KeyCtx, sizeof(previousKey));
		assert(DeviceIntrfTx(&tls.DevIntrf, 0, &sample, 1) == 0);
		assert(DeviceIntrfRx(&tls.DevIntrf, 0, &sample, 1) == 0);
		start = Now();
		while (tls.State != TLS_READY && tls.State != TLS_FAILED && Now() - start < 6000)
		{
			TlsProcess(&tls, Now(), stream.Open);
			usleep(100);
		}
		printf("Certificate TLS handshake %d state=%d error=%d certificate=%d\n",
			repeat + 1, tls.State, tls.Error, tls.CertificateError);
		fflush(stdout);
		if (certificateFailure || limitFailure)
		{
			assert(tls.State == TLS_FAILED);
			assert(tls.Error == (certificateFailure ? TLS_ERROR_CERTIFICATE : TLS_ERROR_LIMIT));
			if (certificateFailure)
			{
				assert(tls.CertificateError != X509_STATUS_OK);
			}
			assert(DeviceIntrfTx(&tls.DevIntrf, 0, &sample, 1) == 0);
			assert(DeviceIntrfRx(&tls.DevIntrf, 0, &sample, 1) == 0);
		}
		else
		{
			assert(tls.State == TLS_READY);
			assert(tls.Error == TLS_ERROR_NONE);
			assert(tls.CertificateError == X509_STATUS_OK);
			for (auto value : tls.KeyCtx)
			{
				assert(value == 0);
			}
			for (size_t length : {19U, 1024U, 4096U, 16000U})
			{
				Echo(tls, stream, length);
			}
			assert(TlsClose(&tls));
			start = Now();
			while (tls.State == TLS_CLOSING && Now() - start < 5000)
			{
				TlsProcess(&tls, Now(), stream.Open);
			}
			assert(tls.State == TLS_CLOSED);
			printf("OpenSSL authenticated reverse echoes 19/1024/4096/16000 bytes PASS\n");
		}
		TlsReset(&tls);
		assert(close(stream.Fd) == 0);
	}
	return 0;
}

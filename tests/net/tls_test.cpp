/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 */
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include "net/tls.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

class TestRng : public RngEngine {
public:
	bool Secure = true;
	bool Works = true;
	bool Enable() override { return true; }
	void Disable() override {}
	void Reset() override {}
	bool IsSecure() const override { return Secure; }
	CRYPTO_STATUS Random(uint8_t *p, size_t n) override
	{
		memset(p, 0x42, n);
		return Works ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
};

class FailedAgree : public KeyAgreeEngine {
public:
	size_t Size = CRYPTO_KEYCTX_MAX;
	size_t Alignment = 8;
	bool Async = false;
	bool Enable() override { return true; }
	void Disable() override {}
	void Reset() override {}
	bool IsAsync() const override { return Async; }
	size_t KeyCtxSize() const override { return Size; }
	size_t KeyCtxAlign() const override { return Alignment; }
	void KeyReset(void *p) override { CryptoSecureWipe(p, CRYPTO_KEYCTX_MAX); }
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE, void *p, uint8_t *) override
	{
		memset(p, 0x77, CRYPTO_KEYCTX_MAX);
		return CRYPTO_STATUS_BUSY;
	}
};

struct Fixture {
	TlsDev_t Tls = {};
	DevIntrf_t Transport = {};
	CryptoSoftSha256 Hash;
	CryptoSoftAes Aes;
	TestRng Rng;
	CryptoUecc Agree;
	std::vector<uint8_t> Input;
	std::vector<uint8_t> Output;
	bool Block = false;
	int Chunk = 65536;
	uint8_t Psk[32] = {};
	TlsCfg_t Config = {};
	static bool Start(DevIntrf_t *, uint32_t) { return true; }
	static void Stop(DevIntrf_t *) {}
	static int Read(DevIntrf_t *d, uint8_t *p, int n)
	{
		Fixture &f = *(Fixture *)d->pDevData;
		if ((size_t)n > f.Input.size()) n = f.Input.size();
		if (n > f.Chunk) n = f.Chunk;
		if (n != 0) memcpy(p, f.Input.data(), n);
		f.Input.erase(f.Input.begin(), f.Input.begin() + n);
		return n;
	}
	static int Write(DevIntrf_t *d, const uint8_t *p, int n)
	{
		Fixture &f = *(Fixture *)d->pDevData;
		if (f.Block) return 0;
		if (n > f.Chunk) n = f.Chunk;
		f.Output.insert(f.Output.end(), p, p + n);
		return n;
	}
	Fixture()
	{
		Agree.SetRng(&Rng);
		Agree.Enable();
		Hash.Enable();
		Aes.Enable();
		Transport.pDevData = this;
		Transport.StartRx = Transport.StartTx = Start;
		Transport.StopRx = Transport.StopTx = Stop;
		Transport.RxData = Read;
		Transport.TxData = Write;
		Transport.EnCnt = 1;
		atomic_flag_clear(&Transport.bBusy);
		Config = {&Hash, &Hash, &Aes, &Rng, Psk, sizeof(Psk),
			(const uint8_t *)"IOsonata", 8, nullptr, 1000, nullptr, nullptr};
		assert(TlsInit(&Tls, &Config, &Transport, 0));
		TlsProcess(&Tls, 0, true);
	}
	~Fixture() { TlsReset(&Tls); }
	void Pump()
	{
		for (int i = 0; i < 3000 && (!Input.empty() || Tls.TxLen); i++)
		{
			TlsProcess(&Tls, 1, true);
			if (Tls.State == TLS_FAILED) break;
		}
	}
	void Feed(uint8_t type, const std::vector<uint8_t> &data)
	{
		Input.insert(Input.end(), {type, 3, 3, (uint8_t)(data.size() >> 8), (uint8_t)data.size()});
		Input.insert(Input.end(), data.begin(), data.end());
		Pump();
	}
	void Encrypted(uint8_t type, const std::vector<uint8_t> &data, bool corrupt = false)
	{
		std::vector<uint8_t> record(5 + data.size() + 17);
		record[0] = 23;
		record[1] = record[2] = 3;
		record[3] = (uint8_t)((record.size() - 5) >> 8);
		record[4] = (uint8_t)(record.size() - 5);
		if (!data.empty()) memcpy(record.data() + 5, data.data(), data.size());
		record[5 + data.size()] = type;
		uint8_t nonce[12];
		memcpy(nonce, Tls.RxIv, 12);
		uint64_t seq = Tls.RxSequence;
		for (int i = 0; i < 8; i++)
		{
			nonce[11 - i] ^= (uint8_t)seq;
			seq >>= 8;
		}
		CryptoKey key = {};
		key.Type = CRYPTO_KEY_AES_128;
		key.Loc = CRYPTO_KEY_LOC_PLAIN;
		key.Usage = CRYPTO_KEY_USE_ENCRYPT;
		key.Plain.pData = Tls.RxKey;
		key.Plain.Len = 16;
		assert(Aes.Seal(CRYPTO_AEAD_AES_GCM, key, nonce, 12, record.data(), 5,
			record.data() + 5, data.size() + 1, record.data() + 5,
			record.data() + 6 + data.size(), 16) == CRYPTO_STATUS_OK);
		if (corrupt) record.back() ^= 1;
		Input.insert(Input.end(), record.begin(), record.end());
		Pump();
	}
	void Hello(unsigned mutation = 0)
	{
		std::vector<uint8_t> hello = {2, 0, 0, 52, 3, 3};
		hello.resize(38, 0x55);
		hello.insert(hello.end(), {0, 0x13, 1, 0, 0, 12,
			0, 43, 0, 2, 3, 4, 0, 41, 0, 2, 0, 0});
		if (mutation == 1) hello[40] = 2; // Unoffered cipher.
		if (mutation == 2) hello[55] = 1; // Unoffered PSK identity.
		if (mutation == 3) hello[49] = 3; // Version downgrade.
		if (mutation == 4) hello[41] = 1; // Compression.
		if (mutation == 5)
		{
			hello[51] = 43; // Duplicate supported_versions.
			hello[54] = 3;
			hello[55] = 4;
		}
		Feed(22, hello);
		if (mutation == 0 && Config.pKeyAgree == nullptr) assert(Tls.State == TLS_ENCRYPTED_EXTENSIONS);
	}
	void Dhe()
	{
		Config.pKeyAgree = &Agree;
		assert(TlsInit(&Tls, &Config, &Transport, 0));
		TlsProcess(&Tls, 0, true);
	}
	void DheHello(unsigned mutation)
	{
		uint64_t key[8] = {};
		uint8_t pub[64];
		assert(Agree.KeyGen(CRYPTO_CURVE_P256, key, pub) == CRYPTO_STATUS_OK);
		Agree.KeyReset(key);
		std::vector<uint8_t> hello = {2, 0, 0, 125, 3, 3};
		hello.resize(38, 0x55);
		hello.insert(hello.end(), {0, 0x13, 1, 0, 0, 85,
			0, 43, 0, 2, 3, 4, 0, 41, 0, 2, 0, 0,
			0, 51, 0, 69, 0, 23, 0, 65, 4});
		hello.insert(hello.end(), pub, pub + sizeof(pub));
		if (mutation == 1) hello[61] = 24;
		if (mutation == 2) hello[64] = 2;
		if (mutation == 3) hello[63] = 64;
		if (mutation == 4) memset(hello.data() + 65, 0, 64);
		if (mutation == 5)
		{
			std::vector<uint8_t> share(hello.begin() + 56, hello.end());
			hello.insert(hello.end(), share.begin(), share.end());
			hello[3] += 73;
			hello[43] += 73;
		}
		Feed(22, hello);
	}
	void Ready()
	{
		// Record-layer tests isolate post-handshake behavior. Real handshakes
		// (including these traffic keys) are independently tested with OpenSSL.
		Tls.State = TLS_READY;
		Tls.bEncrypted = true;
	}
	void Failed(TLS_ERROR error)
	{
		assert(Tls.State == TLS_FAILED && Tls.Error == error);
		uint8_t zeros[32] = {};
		assert(memcmp(Tls.Secret, zeros, 32) == 0);
		assert(memcmp(Tls.ClientSecret, zeros, 32) == 0);
		assert(memcmp(Tls.ServerSecret, zeros, 32) == 0);
		assert(memcmp(Tls.RxKey, zeros, 16) == 0);
		assert(memcmp(Tls.TxKey, zeros, 16) == 0);
		for (auto c : Tls.KeyCtx) assert(c == 0);
		for (auto c : Tls.Rx) assert(c == 0);
		for (auto c : Tls.Tx) assert(c == 0);
		assert(Tls.PlainLen == 0 && Tls.TxLen == 0);
	}
};

int main()
{
	for (int mode = 0; mode < 4; mode++)
	{
		FailedAgree agree;
		Fixture f;
		if (mode == 1) agree.Size++;
		if (mode == 2) agree.Alignment = 16;
		if (mode == 3) agree.Async = true;
		f.Config.pKeyAgree = &agree;
		assert(!TlsInit(&f.Tls, &f.Config, &f.Transport, 0));
		f.Failed(mode == 0 ? TLS_ERROR_CRYPTO : TLS_ERROR_CONFIG);
	}

	for (unsigned mutation = 0; mutation <= 5; mutation++)
	{
		Fixture f;
		f.Dhe();
		f.Chunk = 1;
		f.DheHello(mutation);
		if (mutation == 0)
		{
			assert(f.Tls.State == TLS_ENCRYPTED_EXTENSIONS);
			for (auto c : f.Tls.KeyCtx) assert(c == 0);
			f.Encrypted(22, {8,0,0,10,0,8,0,10,0,4,0,2,0,23});
			assert(f.Tls.State == TLS_SERVER_FINISHED);
		}
		else
		{
			f.Failed(mutation == 4 ? TLS_ERROR_CRYPTO : TLS_ERROR_PROTOCOL);
		}
	}
	{
		Fixture f;
		f.Dhe();
		f.Hello(); // PSK-only response must not downgrade a DHE request.
		f.Failed(TLS_ERROR_AUTH);
	}
	{
		Fixture f;
		f.DheHello(0); // Unsolicited key share in PSK-only mode.
		f.Failed(TLS_ERROR_PROTOCOL);
	}
	{
		Fixture f;
		f.Dhe();
		TlsProcess(&f.Tls, 1000, true);
		f.Failed(TLS_ERROR_TIMEOUT);
	}

	for (unsigned mutation = 1; mutation <= 5; mutation++)
	{
		Fixture f;
		f.Hello(mutation);
		f.Failed(TLS_ERROR_PROTOCOL);
	}
	{
		Fixture f;
		f.Chunk = 1;
		f.Hello();
		f.Encrypted(22, {8,0});
		f.Encrypted(22, {0,2,0,0});
		assert(f.Tls.State == TLS_SERVER_FINISHED);
		std::vector<uint8_t> badFinished(36);
		badFinished[0] = 20;
		badFinished[3] = 32;
		f.Encrypted(22, badFinished);
		f.Failed(TLS_ERROR_AUTH);
	}
	{
		Fixture f;
		f.Hello();
		f.Encrypted(22, {8,0,0,2,0,0}, true);
		f.Failed(TLS_ERROR_AUTH);
	}
	{
		Fixture f;
		f.Hello();
		f.Feed(22, {8,0,0,2,0,0});
		f.Failed(TLS_ERROR_PROTOCOL);
	}
	{
		Fixture f;
		f.Feed(22, {2,0,0xff,0xff});
		f.Failed(TLS_ERROR_LIMIT);
	}
	{
		Fixture f;
		f.Input = {23,3,3,0xff,0xff};
		f.Pump();
		f.Failed(TLS_ERROR_PROTOCOL);
	}
	{
		Fixture f;
		f.Feed(23, {'p','l','a','i','n'});
		f.Failed(TLS_ERROR_PROTOCOL);
	}
	{
		Fixture f;
		TlsProcess(&f.Tls, 1000, true);
		f.Failed(TLS_ERROR_TIMEOUT);
		assert(TlsInit(&f.Tls, &f.Config, &f.Transport, 0xfffffff0U));
		TlsProcess(&f.Tls, 0x3d8U, true);
		f.Failed(TLS_ERROR_TIMEOUT);
	}
	{
		Fixture f;
		TlsProcess(&f.Tls, 1, false);
		f.Failed(TLS_ERROR_TRANSPORT);
	}
	{
		Fixture f;
		f.Rng.Secure = false;
		assert(!TlsInit(&f.Tls, &f.Config, &f.Transport, 0));
		f.Failed(TLS_ERROR_CONFIG);
		f.Rng.Secure = true;
		f.Rng.Works = false;
		assert(!TlsInit(&f.Tls, &f.Config, &f.Transport, 0));
		f.Failed(TLS_ERROR_CRYPTO);
	}
	{
		Fixture f;
		f.Ready();
		f.Block = true;
		const uint8_t msg[] = {1,2,3};
		assert(DeviceIntrfTx(&f.Tls.DevIntrf, 0, msg, 3) == 3);
		assert(DeviceIntrfTx(&f.Tls.DevIntrf, 0, msg, 3) == 0);
		assert(f.Tls.TxSequence == 1);
		TlsProcess(&f.Tls, 1000, true);
		f.Failed(TLS_ERROR_TIMEOUT);
	}
	{
		Fixture f;
		f.Ready();
		f.Encrypted(23, {1,2,3});
		uint8_t out[4] = {};
		assert(DeviceIntrfRx(&f.Tls.DevIntrf, 0, out, 2) == 2);
		assert(DeviceIntrfRx(&f.Tls.DevIntrf, 0, out + 2, 2) == 1);
		assert(out[0] == 1 && out[1] == 2 && out[2] == 3 && out[3] == 0);
		f.Encrypted(22, {24,0,0,1,1});
		assert(f.Tls.State == TLS_READY && f.Tls.RxSequence == 0 && f.Tls.TxSequence == 0);
		f.Encrypted(23, {4,5,6});
		assert(DeviceIntrfRx(&f.Tls.DevIntrf, 0, out, 4) == 3 && out[0] == 4);
		f.Encrypted(21, {1,0});
		assert(f.Tls.State == TLS_CLOSED);
	}
	{
		Fixture f;
		f.Ready();
		f.Encrypted(22, {24,0,0,1,2});
		f.Failed(TLS_ERROR_PROTOCOL);
	}
	{
		Fixture f;
		f.Ready();
		f.Tls.RxSequence = 1048576;
		f.Encrypted(23, {1});
		f.Failed(TLS_ERROR_PROTOCOL);
	}
	{
		Fixture f;
		f.Hello();
		f.Encrypted(22, {8,0,0,6,0,4,0,16,0,0});
		f.Failed(TLS_ERROR_PROTOCOL);
	}
	puts("TLS parser, authentication, fragmentation, key update, bounds, timeout, EOF and wipe checks PASS");
}

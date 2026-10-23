/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Certificate and TLS parser tests with production SHA-256/AES/P-256 engines.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <initializer_list>
#include <map>
#include <string>
#include <vector>

#include "net/tls.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"
#include "crypto/x509.h"

class TestRng : public RngEngine {
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
		memset(pOut, 0x42, Len);
		return CRYPTO_STATUS_OK;
	}
};

class FailedSign : public SignEngine {
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
	CRYPTO_STATUS Verify(CRYPTO_CURVE, const uint8_t *, const uint8_t *,
						 size_t, const uint8_t *) override
	{
		return CRYPTO_STATUS_BUSY;
	}
};

struct Credentials {
	std::string Directory;
	std::map<std::string, std::vector<uint8_t>> Files;

	explicit Credentials(const char *pDirectory) : Directory(pDirectory)
	{
	}

	const std::vector<uint8_t> &Load(const char *pName)
	{
		std::string path = Directory + "/" + pName;
		std::vector<uint8_t> &data = Files[pName];
		FILE *pFile;
		long length;

		if (!data.empty())
		{
			return data;
		}
		pFile = fopen(path.c_str(), "rb");
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

	X509Cert_t Cert(const char *pName)
	{
		const std::vector<uint8_t> &data = Load(pName);
		return {data.data(), (uint32_t)data.size()};
	}
};

static bool Zero(const void *pData, size_t Len)
{
	const uint8_t *pBytes = (const uint8_t *)pData;

	for (size_t i = 0; i < Len; i++)
	{
		if (pBytes[i] != 0)
		{
			return false;
		}
	}
	return true;
}

static void CheckChain(const char *pName, const X509VerifyCfg_t &Cfg,
					   const std::vector<X509Cert_t> &Chain, X509_STATUS Expected)
{
	uint8_t key[64];
	X509_STATUS status;

	memset(key, 0xa5, sizeof(key));
	status = X509VerifyChain(&Cfg, Chain.data(), (uint32_t)Chain.size(), key);
	if (status != Expected)
	{
		fprintf(stderr, "X.509 %s: status=%d expected=%d\n", pName, status, Expected);
	}
	assert(status == Expected);
	if (Expected != X509_STATUS_OK)
	{
		assert(Zero(key, sizeof(key)));
	}
}

static void CheckRejected(const X509VerifyCfg_t &Cfg, const X509Cert_t *pChain,
						  uint32_t Count)
{
	uint8_t key[64];

	memset(key, 0xa5, sizeof(key));
	assert(X509VerifyChain(&Cfg, pChain, Count, key) != X509_STATUS_OK);
	assert(Zero(key, sizeof(key)));
}

static void X509Tests(Credentials &Files, uint64_t Now)
{
	CryptoSoftSha256 hash;
	CryptoUecc ecc;
	TestRng rng;
	FailedSign failedSign;
	X509Cert_t roots[] = {Files.Cert("root.der"), Files.Cert("other_root.der")};
	X509Cert_t shortRoot = Files.Cert("short_root.der");
	X509VerifyCfg_t cfg = {&hash, &ecc, roots, 1, "localhost", Now};
	std::vector<X509Cert_t> chain = {Files.Cert("server.der"), Files.Cert("intermediate.der")};
	std::vector<uint8_t> malformed;
	const std::vector<uint8_t> &server = Files.Load("server.der");
	const std::vector<uint8_t> &scalar = Files.Load("server.scalar");
	uint8_t actualKey[64];
	uint8_t expectedKey[64];
	uint64_t keyContext[8] = {};
	X509Cert_t malformedCert;
	X509VerifyCfg_t changed;
	X509Cert_t excessive[X509_MAX_CHAIN_CERTS + 1] = {};
	X509Cert_t invalidRoot;
	const uint8_t pointHeader[] = {3, 0x42, 0, 4};
	size_t pointOffset;

	hash.Enable();
	ecc.SetRng(&rng);
	ecc.Enable();
	CheckChain("CA/intermediate/leaf", cfg, chain, X509_STATUS_OK);
	assert(X509VerifyChain(&cfg, chain.data(), 2, actualKey) == X509_STATUS_OK);
	assert(scalar.size() == 32);
	assert(ecc.KeyImport(CRYPTO_CURVE_P256, keyContext, scalar.data(), expectedKey) == CRYPTO_STATUS_OK);
	assert(memcmp(actualKey, expectedKey, sizeof(actualKey)) == 0);
	ecc.KeyReset(keyContext);
	CheckChain("included root", cfg,
		{chain[0], chain[1], roots[0]}, X509_STATUS_OK);
	CheckChain("direct issuer", cfg, {Files.Cert("direct.der")}, X509_STATUS_OK);
	changed = cfg;
	changed.TrustAnchorCount = 2;
	CheckChain("multiple configured roots", changed, chain, X509_STATUS_OK);
	changed.pTrustAnchors = roots + 1;
	changed.TrustAnchorCount = 1;
	CheckChain("untrusted root", changed, chain, X509_STATUS_UNTRUSTED);
	CheckChain("missing issuer", cfg, {chain[0]}, X509_STATUS_UNTRUSTED);
	changed = cfg;
	changed.pHostName = "LOCALHOST";
	CheckChain("case-insensitive DNS", changed, chain, X509_STATUS_OK);
	changed.pHostName = "server.iosonata.test";
	CheckChain("second SAN name", changed, chain, X509_STATUS_OK);
	changed.pHostName = "wrong.test";
	CheckChain("wrong DNS name", changed, chain, X509_STATUS_HOSTNAME);
	CheckChain("SAN required", cfg,
		{Files.Cert("no_san.der"), chain[1]}, X509_STATUS_HOSTNAME);
	CheckChain("CN is not a SAN fallback", cfg,
		{Files.Cert("wrong_name.der"), chain[1]}, X509_STATUS_HOSTNAME);
	changed.pHostName = "device.mqtt.example.test";
	CheckChain("wildcard single label", changed,
		{Files.Cert("wildcard.der"), chain[1]}, X509_STATUS_OK);
	changed.pHostName = "MQTT.EXAMPLE.TEST";
	CheckChain("wildcard needs one label", changed,
		{Files.Cert("wildcard.der"), chain[1]}, X509_STATUS_HOSTNAME);
	changed.pHostName = "a.b.mqtt.example.test";
	CheckChain("wildcard cannot cross labels", changed,
		{Files.Cert("wildcard.der"), chain[1]}, X509_STATUS_HOSTNAME);
	changed = cfg;
	changed.UnixTime = Now - 3600;
	CheckChain("not yet valid leaf", changed, chain, X509_STATUS_TIME);
	changed.UnixTime = Now + 31 * 86400;
	CheckChain("expired leaf", changed, chain, X509_STATUS_TIME);
	changed.UnixTime = Now + 2 * 86400;
	CheckChain("expired intermediate", changed,
		{Files.Cert("short_ca_server.der"), Files.Cert("short_ca.der")}, X509_STATUS_TIME);
	changed.pTrustAnchors = &shortRoot;
	CheckChain("expired configured root", changed,
		{Files.Cert("short_root_server.der")}, X509_STATUS_TIME);
	CheckChain("leaf key usage", cfg,
		{Files.Cert("wrong_usage.der"), chain[1]}, X509_STATUS_CONSTRAINT);
	CheckChain("leaf extended key usage", cfg,
		{Files.Cert("wrong_eku.der"), chain[1]}, X509_STATUS_CONSTRAINT);
	CheckChain("CA flag", cfg,
		{Files.Cert("non_ca_server.der"), Files.Cert("not_ca.der")}, X509_STATUS_CONSTRAINT);
	CheckChain("CA key usage", cfg,
		{Files.Cert("no_ca_usage_server.der"), Files.Cert("no_ca_usage.der")}, X509_STATUS_CONSTRAINT);
	CheckChain("CA extended key usage", cfg,
		{Files.Cert("client_ca_server.der"), Files.Cert("client_ca.der")}, X509_STATUS_CONSTRAINT);
	CheckChain("CA path length", cfg,
		{Files.Cert("pathlen_server.der"), Files.Cert("sub_intermediate.der"), chain[1]},
		X509_STATUS_CONSTRAINT);
	CheckChain("unsupported name constraints", cfg,
		{Files.Cert("constrained_server.der"), Files.Cert("name_constrained.der")},
		X509_STATUS_UNSUPPORTED);
	CheckChain("unknown critical extension", cfg,
		{Files.Cert("critical_extension.der"), chain[1]}, X509_STATUS_UNSUPPORTED);
	CheckChain("unknown noncritical extension", cfg,
		{Files.Cert("noncritical_extension.der"), chain[1]}, X509_STATUS_OK);
	CheckChain("AKI issuer and serial", cfg,
		{Files.Cert("aki_issuer.der"), chain[1]}, X509_STATUS_OK);
	CheckChain("empty subject with critical SAN", cfg,
		{Files.Cert("empty_subject_critical_san.der"), chain[1]}, X509_STATUS_OK);
	CheckChain("pre-1970 and GeneralizedTime validity", cfg,
		{Files.Cert("wide_validity.der"), chain[1]}, X509_STATUS_OK);
	for (const char *pName : {"duplicate_extension.der", "invalid_calendar.der",
		"empty_subject_noncritical_san.der", "nul_san.der", "invalid_wildcard.der"})
	{
		X509Cert_t invalidChain[] = {Files.Cert(pName), chain[1]};

		CheckRejected(cfg, invalidChain, 2);
	}
	CheckChain("noncritical CA basic constraints", cfg,
		{Files.Cert("noncritical_ca_server.der"), Files.Cert("noncritical_ca.der")},
		X509_STATUS_CONSTRAINT);
	CheckChain("noncritical name constraints", cfg,
		{Files.Cert("noncritical_constraint_server.der"), Files.Cert("noncritical_constraint.der")},
		X509_STATUS_UNSUPPORTED);
	CheckChain("path length without key usage", cfg,
		{Files.Cert("pathlen_no_usage_server.der"), Files.Cert("pathlen_no_usage.der")},
		X509_STATUS_CONSTRAINT);
	CheckChain("altered certificate signature", cfg,
		{Files.Cert("bad_signature.der"), chain[1]}, X509_STATUS_SIGNATURE);
	// The certificate carries a signed but invalid affine point. Its issuer
	// signature is valid; the leaf point must fail when used for CertificateVerify.
	CheckChain("signed invalid leaf point", cfg,
		{Files.Cert("invalid_point.der"), chain[1]}, X509_STATUS_OK);
	malformed = Files.Load("root.der");
	pointOffset = 0;
	while (pointOffset + 68 < malformed.size() &&
		memcmp(malformed.data() + pointOffset, pointHeader, sizeof(pointHeader)) != 0)
	{
		pointOffset++;
	}
	assert(pointOffset + 68 < malformed.size());
	memset(malformed.data() + pointOffset + 4, 0, 64);
	invalidRoot = {malformed.data(), (uint32_t)malformed.size()};
	changed = cfg;
	changed.pTrustAnchors = &invalidRoot;
	CheckChain("invalid configured issuer public key", changed,
		{Files.Cert("direct.der")}, X509_STATUS_SIGNATURE);
	changed = cfg;
	changed.pSign = &failedSign;
	CheckChain("signature engine busy", changed, chain, X509_STATUS_CRYPTO);
	for (size_t length = 0; length < server.size(); length++)
	{
		malformedCert = {server.data(), (uint32_t)length};
		CheckRejected(cfg, &malformedCert, 1);
	}
	malformed = server;
	malformed.push_back(0);
	malformedCert = {malformed.data(), (uint32_t)malformed.size()};
	CheckRejected(cfg, &malformedCert, 1);
	malformed = server;
	malformed[1] = 0x80;
	malformedCert = {malformed.data(), (uint32_t)malformed.size()};
	CheckRejected(cfg, &malformedCert, 1);
	assert(server[1] == 0x82);
	malformed = server;
	malformed[1] = 0x83;
	malformed.insert(malformed.begin() + 2, 0);
	malformedCert = {malformed.data(), (uint32_t)malformed.size()};
	CheckRejected(cfg, &malformedCert, 1);
	malformedCert = {server.data(), X509_MAX_CERT_SIZE + 1};
	CheckRejected(cfg, &malformedCert, 1);
	CheckRejected(cfg, nullptr, 1);
	CheckRejected(cfg, chain.data(), 0);
	CheckRejected(cfg, excessive, X509_MAX_CHAIN_CERTS + 1);
	changed = cfg;
	changed.UnixTime = 0;
	CheckRejected(changed, chain.data(), 2);
	changed = cfg;
	changed.pHash = nullptr;
	CheckRejected(changed, chain.data(), 2);
	changed = cfg;
	changed.pSign = nullptr;
	CheckRejected(changed, chain.data(), 2);
	changed = cfg;
	changed.TrustAnchorCount = 0;
	CheckRejected(changed, chain.data(), 2);
	changed.TrustAnchorCount = X509_MAX_TRUST_ANCHORS + 1;
	CheckRejected(changed, chain.data(), 2);
	for (const char *pName : {"", "localhost.", "127.0.0.1", "a..test", "-a.test", "*.test"})
	{
		changed = cfg;
		changed.pHostName = pName;
		CheckRejected(changed, chain.data(), 2);
	}
	printf("X.509 chain, DNS, time, usage, signature and malformed DER checks PASS\n");
}

static void SignatureDerTests()
{
	const uint8_t good[] = {0x30, 6, 2, 1, 1, 2, 1, 2};
	const uint8_t zero[] = {0x30, 6, 2, 1, 0, 2, 1, 2};
	const uint8_t negative[] = {0x30, 6, 2, 1, 0x80, 2, 1, 2};
	const uint8_t overlong[] = {0x30, 7, 2, 2, 0, 1, 2, 1, 2};
	const uint8_t trailing[] = {0x30, 6, 2, 1, 1, 2, 1, 2, 0};
	const uint8_t longLength[] = {0x30, 0x81, 6, 2, 1, 1, 2, 1, 2};
	const uint8_t order[32] = {
		0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xbc, 0xe6, 0xfa, 0xad, 0xa7, 0x17, 0x9e, 0x84,
		0xf3, 0xb9, 0xca, 0xc2, 0xfc, 0x63, 0x25, 0x51};
	std::vector<uint8_t> outOfRange = {0x30, 38, 2, 33, 0};
	uint8_t raw[64];
	std::vector<std::vector<uint8_t>> bad = {
		{zero, zero + sizeof(zero)},
		{negative, negative + sizeof(negative)},
		{overlong, overlong + sizeof(overlong)},
		{trailing, trailing + sizeof(trailing)},
		{longLength, longLength + sizeof(longLength)}};

	assert(X509EcdsaP256Decode(good, sizeof(good), raw));
	assert(raw[31] == 1 && raw[63] == 2);
	assert(Zero(raw, 31) && Zero(raw + 32, 31));
	outOfRange.insert(outOfRange.end(), order, order + sizeof(order));
	outOfRange.insert(outOfRange.end(), {2, 1, 1});
	bad.push_back(outOfRange);
	for (const auto &der : bad)
	{
		memset(raw, 0xa5, sizeof(raw));
		assert(!X509EcdsaP256Decode(der.data(), (uint32_t)der.size(), raw));
		assert(Zero(raw, sizeof(raw)));
	}
	for (size_t length = 0; length < sizeof(good); length++)
	{
		memset(raw, 0xa5, sizeof(raw));
		assert(!X509EcdsaP256Decode(good, (uint32_t)length, raw));
		assert(Zero(raw, sizeof(raw)));
	}
	printf("ECDSA DER integer, length and scalar bounds checks PASS\n");
}

static void Put24(std::vector<uint8_t> &Data, size_t Offset, size_t Value)
{
	Data[Offset] = (uint8_t)(Value >> 16);
	Data[Offset + 1] = (uint8_t)(Value >> 8);
	Data[Offset + 2] = (uint8_t)Value;
}

static std::vector<uint8_t> CertificateMessage(Credentials &Files,
											 const char *pLeaf = "server.der",
											 bool IncludeRoot = false)
{
	std::vector<uint8_t> message = {11, 0, 0, 0, 0, 0, 0, 0};
	std::vector<const char *> names = {pLeaf, "intermediate.der"};
	size_t offset;

	if (IncludeRoot)
	{
		names.push_back("root.der");
	}
	for (const char *pName : names)
	{
		const std::vector<uint8_t> &der = Files.Load(pName);

		offset = message.size();
		message.resize(offset + 3);
		Put24(message, offset, der.size());
		message.insert(message.end(), der.begin(), der.end());
		message.insert(message.end(), {0, 0});
	}
	Put24(message, 1, message.size() - 4);
	Put24(message, 5, message.size() - 8);
	return message;
}

struct Fixture {
	TlsDev_t Tls = {};
	DevIntrf_t Transport = {};
	CryptoSoftSha256 Hash;
	CryptoSoftAes Aes;
	TestRng Rng;
	CryptoUecc Ecc;
	X509Cert_t Root;
	TlsCertCfg_t Cert = {};
	TlsCfg_t Config = {};
	std::vector<uint8_t> Input;
	std::vector<uint8_t> Output;
	std::vector<uint8_t> Handshake = std::vector<uint8_t>(8192);
	std::vector<uint8_t> Transcript = std::vector<uint8_t>(8192);
	int Chunk = 65536;
	bool Block = false;

	static bool Start(DevIntrf_t *, uint32_t)
	{
		return true;
	}
	static void Stop(DevIntrf_t *)
	{
	}
	static int Read(DevIntrf_t *pDev, uint8_t *pOut, int Len)
	{
		Fixture &fixture = *(Fixture *)pDev->pDevData;

		if ((size_t)Len > fixture.Input.size())
		{
			Len = (int)fixture.Input.size();
		}
		if (Len > fixture.Chunk)
		{
			Len = fixture.Chunk;
		}
		if (Len != 0)
		{
			memcpy(pOut, fixture.Input.data(), Len);
		}
		fixture.Input.erase(fixture.Input.begin(), fixture.Input.begin() + Len);
		return Len;
	}
	static int Write(DevIntrf_t *pDev, const uint8_t *pIn, int Len)
	{
		Fixture &fixture = *(Fixture *)pDev->pDevData;

		if (fixture.Block)
		{
			return 0;
		}
		if (Len > fixture.Chunk)
		{
			Len = fixture.Chunk;
		}
		fixture.Output.insert(fixture.Output.end(), pIn, pIn + Len);
		return Len;
	}

	Fixture(Credentials &Files, uint64_t Now, bool External = false)
	{
		Root = Files.Cert("root.der");
		Ecc.SetRng(&Rng);
		Ecc.Enable();
		Hash.Enable();
		Aes.Enable();
		Transport.pDevData = this;
		Transport.StartRx = Transport.StartTx = Start;
		Transport.StopRx = Transport.StopTx = Stop;
		Transport.RxData = Read;
		Transport.TxData = Write;
		Transport.EnCnt = 1;
		atomic_flag_clear(&Transport.bBusy);
		Cert.pSign = &Ecc;
		Cert.pTrustAnchors = &Root;
		Cert.TrustAnchorCount = 1;
		Cert.UnixTime = Now;
		if (External)
		{
			Cert.pHandshakeBuffer = Handshake.data();
			Cert.HandshakeBufferSize = Handshake.size();
			Cert.pTranscriptBuffer = Transcript.data();
			Cert.TranscriptBufferSize = Transcript.size();
		}
		Config.pHash = &Hash;
		Config.pMac = &Hash;
		Config.pAead = &Aes;
		Config.pRng = &Rng;
		Config.pServerName = "localhost";
		Config.Timeout = 1000;
		Config.pKeyAgree = &Ecc;
		Config.pCert = &Cert;
		assert(TlsInit(&Tls, &Config, &Transport, 0));
		TlsProcess(&Tls, 0, true);
	}
	~Fixture()
	{
		TlsReset(&Tls);
	}
	void Pump()
	{
		for (unsigned i = 0; i < 20000 && (!Input.empty() || Tls.TxLen != 0); i++)
		{
			TlsProcess(&Tls, 1, true);
			if (Tls.State == TLS_FAILED)
			{
				break;
			}
		}
	}
	void Feed(uint8_t Type, const std::vector<uint8_t> &Data)
	{
		Input.insert(Input.end(), {Type, 3, 3, (uint8_t)(Data.size() >> 8), (uint8_t)Data.size()});
		Input.insert(Input.end(), Data.begin(), Data.end());
		Pump();
	}
	void Encrypted(uint8_t Type, const std::vector<uint8_t> &Data)
	{
		std::vector<uint8_t> record(5 + Data.size() + 17);
		uint8_t nonce[12];
		uint64_t sequence = Tls.RxSequence;
		CryptoKey key = {};

		record[0] = 23;
		record[1] = record[2] = 3;
		record[3] = (uint8_t)((record.size() - 5) >> 8);
		record[4] = (uint8_t)(record.size() - 5);
		if (!Data.empty())
		{
			memcpy(record.data() + 5, Data.data(), Data.size());
		}
		record[5 + Data.size()] = Type;
		memcpy(nonce, Tls.RxIv, sizeof(nonce));
		for (unsigned i = 0; i < 8; i++)
		{
			nonce[11 - i] ^= (uint8_t)sequence;
			sequence >>= 8;
		}
		key.Type = CRYPTO_KEY_AES_128;
		key.Loc = CRYPTO_KEY_LOC_PLAIN;
		key.Usage = CRYPTO_KEY_USE_ENCRYPT;
		key.Plain.pData = Tls.RxKey;
		key.Plain.Len = 16;
		assert(Aes.Seal(CRYPTO_AEAD_AES_GCM, key, nonce, sizeof(nonce),
			record.data(), 5, record.data() + 5, Data.size() + 1,
			record.data() + 5, record.data() + 6 + Data.size(), 16) == CRYPTO_STATUS_OK);
		Input.insert(Input.end(), record.begin(), record.end());
		Pump();
	}
	void Hello(unsigned Mutation = 0)
	{
		uint64_t keyContext[8] = {};
		uint8_t publicKey[64];
		std::vector<uint8_t> hello = {2, 0, 0, 119, 3, 3};

		assert(Ecc.KeyGen(CRYPTO_CURVE_P256, keyContext, publicKey) == CRYPTO_STATUS_OK);
		Ecc.KeyReset(keyContext);
		hello.resize(38, 0x55);
		hello.insert(hello.end(), {0, 0x13, 1, 0, 0, 79,
			0, 43, 0, 2, 3, 4, 0, 51, 0, 69, 0, 23, 0, 65, 4});
		hello.insert(hello.end(), publicKey, publicKey + sizeof(publicKey));
		if (Mutation == 1)
		{
			hello.insert(hello.end(), {0, 41, 0, 2, 0, 0});
			hello[3] += 6;
			hello[43] += 6;
		}
		if (Mutation == 2)
		{
			hello.resize(50);
			hello[3] = 46;
			hello[43] = 6;
		}
		Feed(22, hello);
		if (Mutation == 0)
		{
			assert(Tls.State == TLS_ENCRYPTED_EXTENSIONS);
		}
	}
	void Extensions()
	{
		Encrypted(22, {8, 0, 0, 2, 0, 0});
		assert(Tls.State == TLS_SERVER_CERTIFICATE);
	}
	void Certificate(Credentials &Files)
	{
		Encrypted(22, CertificateMessage(Files));
		assert(Tls.State == TLS_SERVER_CERT_VERIFY);
	}
	std::vector<uint8_t> CertificateVerify(Credentials &Files)
	{
		static const char context[] = "TLS 1.3, server CertificateVerify";
		const std::vector<uint8_t> &privateKey = Files.Load("server.scalar");
		uint8_t signedData[64 + sizeof(context) + 32];
		uint8_t digest[32];
		uint8_t signature[64];
		CryptoKey key = {};
		std::vector<uint8_t> der = {0x30, 0};
		std::vector<uint8_t> message = {15, 0, 0, 0, 4, 3, 0, 0};
		size_t offset;
		size_t count;
		bool padding;

		memset(signedData, 0x20, 64);
		memcpy(signedData + 64, context, sizeof(context));
		assert(Hash.Hash(CRYPTO_HASH_SHA256, Tls.pTranscript, Tls.TranscriptLen,
			signedData + 64 + sizeof(context)) == CRYPTO_STATUS_OK);
		assert(Hash.Hash(CRYPTO_HASH_SHA256, signedData, sizeof(signedData), digest) == CRYPTO_STATUS_OK);
		key.Type = CRYPTO_KEY_ECC_P256;
		key.Loc = CRYPTO_KEY_LOC_PLAIN;
		key.Usage = CRYPTO_KEY_USE_SIGN;
		key.Plain.pData = privateKey.data();
		key.Plain.Len = privateKey.size();
		assert(Ecc.Sign(CRYPTO_CURVE_P256, key, digest, sizeof(digest), signature) == CRYPTO_STATUS_OK);
		for (unsigned scalar = 0; scalar < 2; scalar++)
		{
			offset = scalar * 32;
			count = 32;
			while (count > 1 && signature[offset] == 0)
			{
				offset++;
				count--;
			}
			padding = (signature[offset] & 0x80) != 0;
			der.push_back(2);
			der.push_back((uint8_t)(count + (padding ? 1 : 0)));
			if (padding)
			{
				der.push_back(0);
			}
			der.insert(der.end(), signature + offset, signature + offset + count);
		}
		der[1] = (uint8_t)(der.size() - 2);
		message[7] = (uint8_t)der.size();
		message.insert(message.end(), der.begin(), der.end());
		Put24(message, 1, message.size() - 4);
		return message;
	}
	void Finished()
	{
		const uint8_t info[] = {0, 32, 14, 't', 'l', 's', '1', '3', ' ',
			'f', 'i', 'n', 'i', 's', 'h', 'e', 'd', 0, 1};
		uint8_t digest[32];
		uint8_t finishedKey[32];
		CryptoKey key = {};
		std::vector<uint8_t> message(36);

		message[0] = 20;
		message[3] = 32;
		assert(Hash.Hash(CRYPTO_HASH_SHA256, Tls.pTranscript, Tls.TranscriptLen, digest) == CRYPTO_STATUS_OK);
		key.Type = CRYPTO_KEY_HMAC;
		key.Loc = CRYPTO_KEY_LOC_PLAIN;
		key.Usage = CRYPTO_KEY_USE_SIGN;
		key.Plain.pData = Tls.ServerSecret;
		key.Plain.Len = 32;
		assert(Hash.Mac(CRYPTO_MAC_HMAC, key, info, sizeof(info), finishedKey, 32) == CRYPTO_STATUS_OK);
		key.Plain.pData = finishedKey;
		assert(Hash.Mac(CRYPTO_MAC_HMAC, key, digest, sizeof(digest), message.data() + 4, 32) == CRYPTO_STATUS_OK);
		Encrypted(22, message);
		assert(Tls.State == TLS_READY);
	}
	void Failed(TLS_ERROR Error)
	{
		if (Tls.State != TLS_FAILED || Tls.Error != Error)
		{
			fprintf(stderr, "TLS failure: state=%d error=%d expected=%d certificate=%d\n",
				Tls.State, Tls.Error, Error, Tls.CertificateError);
		}
		assert(Tls.State == TLS_FAILED && Tls.Error == Error);
		assert(Zero(Tls.Secret, sizeof(Tls.Secret)));
		assert(Zero(Tls.ServerSecret, sizeof(Tls.ServerSecret)));
		assert(Zero(Tls.ClientSecret, sizeof(Tls.ClientSecret)));
		assert(Zero(Tls.KeyCtx, sizeof(Tls.KeyCtx)));
		assert(Zero(Tls.RxKey, sizeof(Tls.RxKey)));
		assert(Zero(Tls.TxKey, sizeof(Tls.TxKey)));
		assert(Zero(Tls.ServerPublicKey, sizeof(Tls.ServerPublicKey)));
		assert(Zero(Tls.Rx, sizeof(Tls.Rx)));
		assert(Zero(Tls.Tx, sizeof(Tls.Tx)));
		assert(Zero(Handshake.data(), Handshake.size()));
		assert(Zero(Transcript.data(), Transcript.size()));
		assert(Tls.PlainLen == 0 && Tls.TxLen == 0);
	}
};

static void ClientHelloTests(Credentials &Files, uint64_t Now)
{
	Fixture fixture(Files, Now);
	const std::vector<uint8_t> &hello = fixture.Output;
	size_t offset = 5 + 4 + 2 + 32;
	size_t end;
	size_t length;
	unsigned type;
	unsigned extensions = 0;

	assert(hello.size() > offset + 1 && hello[0] == 22 && hello[5] == 1);
	offset += 1 + hello[offset];
	length = ((size_t)hello[offset] << 8) | hello[offset + 1];
	offset += 2 + length;
	offset += 1 + hello[offset];
	length = ((size_t)hello[offset] << 8) | hello[offset + 1];
	offset += 2;
	end = offset + length;
	assert(end == hello.size());
	while (offset < end)
	{
		assert(end - offset >= 4);
		type = ((unsigned)hello[offset] << 8) | hello[offset + 1];
		length = ((size_t)hello[offset + 2] << 8) | hello[offset + 3];
		offset += 4;
		assert(length <= end - offset);
		assert(type != 41 && type != 45);
		if (type == 13)
		{
			assert(length == 4 && hello[offset] == 0 && hello[offset + 1] == 2);
			assert(hello[offset + 2] == 4 && hello[offset + 3] == 3);
			extensions |= 1;
		}
		if (type == 10)
		{
			extensions |= 2;
		}
		if (type == 51)
		{
			extensions |= 4;
		}
		if (type == 0)
		{
			extensions |= 8;
		}
		offset += length;
	}
	assert(extensions == 15);
	printf("Certificate ClientHello signature, group, key share and SNI checks PASS\n");
}

static void TlsTests(Credentials &Files, uint64_t Now)
{
	for (unsigned mode = 0; mode < 10; mode++)
	{
		Fixture fixture(Files, Now);
		uint8_t psk[32] = {};

		switch (mode)
		{
			case 0:
				fixture.Cert.pSign = nullptr;
				break;
			case 1:
				fixture.Cert.UnixTime = 0;
				break;
			case 2:
				fixture.Cert.TrustAnchorCount = 0;
				break;
			case 3:
				fixture.Cert.pTrustAnchors = nullptr;
				break;
			case 4:
				fixture.Config.pKeyAgree = nullptr;
				break;
			case 5:
				fixture.Config.pServerName = nullptr;
				break;
			case 6:
				fixture.Config.pPsk = psk;
				fixture.Config.PskLen = sizeof(psk);
				break;
			case 7:
				fixture.Cert.pHandshakeBuffer = fixture.Handshake.data();
				break;
			case 8:
				fixture.Cert.TranscriptBufferSize = 8192;
				break;
			case 9:
				fixture.Cert.TrustAnchorCount = X509_MAX_TRUST_ANCHORS + 1;
				break;
		}
		assert(!TlsInit(&fixture.Tls, &fixture.Config, &fixture.Transport, 0));
		fixture.Failed(TLS_ERROR_CONFIG);
	}
	for (unsigned mode = 0; mode < 9; mode++)
	{
		Fixture fixture(Files, Now);
		X509Cert_t root = fixture.Root;
		X509Cert_t configuredRoot;
		std::vector<uint8_t> original(root.pDer, root.pDer + root.Len);
		uint8_t *pOverlap;
		size_t length;

		pOverlap = mode < 2 ? (uint8_t *)&fixture.Root : const_cast<uint8_t *>(root.pDer);
		length = mode < 2 ? sizeof(fixture.Root) : root.Len;
		if (mode < 4)
		{
			if ((mode & 1) == 0)
			{
				fixture.Cert.pHandshakeBuffer = pOverlap;
				fixture.Cert.HandshakeBufferSize = length;
			}
			else
			{
				fixture.Cert.pTranscriptBuffer = pOverlap;
				fixture.Cert.TranscriptBufferSize = length;
			}
		}
		else if (mode == 4)
		{
			fixture.Cert.UnixTime = 253402300800ULL;
		}
		else if (mode == 5)
		{
			fixture.Cert.pHandshakeBuffer = fixture.Tls.Handshake;
			fixture.Cert.HandshakeBufferSize = sizeof(fixture.Tls.Handshake);
		}
		else if (mode == 6)
		{
			fixture.Cert.pHandshakeBuffer = fixture.Handshake.data();
			fixture.Cert.HandshakeBufferSize = fixture.Handshake.size();
			fixture.Cert.pTranscriptBuffer = fixture.Handshake.data() + 1;
			fixture.Cert.TranscriptBufferSize = fixture.Handshake.size() - 1;
		}
		else if (mode == 7)
		{
			fixture.Root.pDer = fixture.Tls.Handshake;
		}
		else
		{
			fixture.Cert.pTrustAnchors = (const X509Cert_t *)fixture.Tls.KeyCtx;
		}
		configuredRoot = fixture.Root;
		assert(!TlsInit(&fixture.Tls, &fixture.Config, &fixture.Transport, 0));
		fixture.Failed(TLS_ERROR_CONFIG);
		assert(fixture.Root.pDer == configuredRoot.pDer && fixture.Root.Len == configuredRoot.Len);
		assert(memcmp(root.pDer, original.data(), original.size()) == 0);
	}
	ClientHelloTests(Files, Now);
	for (unsigned mutation = 1; mutation <= 2; mutation++)
	{
		Fixture fixture(Files, Now);

		fixture.Hello(mutation);
		assert(fixture.Tls.State == TLS_FAILED);
		assert(fixture.Tls.Error == TLS_ERROR_PROTOCOL || fixture.Tls.Error == TLS_ERROR_AUTH);
	}
	for (unsigned state = 0; state < 3; state++)
	{
		Fixture fixture(Files, Now, true);

		fixture.Hello();
		if (state > 0)
		{
			fixture.Extensions();
		}
		if (state > 1)
		{
			fixture.Certificate(Files);
		}
		TlsProcess(&fixture.Tls, 1000, true);
		fixture.Failed(TLS_ERROR_TIMEOUT);
	}
	for (unsigned mutation = 0; mutation < 7; mutation++)
	{
		Fixture fixture(Files, Now, true);
		std::vector<uint8_t> message = CertificateMessage(Files);
		size_t extensionOffset = 11 + Files.Load("server.der").size();

		fixture.Hello();
		fixture.Extensions();
		switch (mutation)
		{
			case 0:
				message = {11, 0, 0, 4, 0, 0, 0, 0};
				break;
			case 1:
				message[4] = 1;
				break;
			case 2:
				message[7]--;
				break;
			case 3:
				message[8] = message[9] = message[10] = 0;
				break;
			case 4:
				message[extensionOffset + 1] = 4;
				message.insert(message.begin() + extensionOffset + 2, {0, 5, 0, 0});
				Put24(message, 1, message.size() - 4);
				Put24(message, 5, message.size() - 8);
				break;
			case 5:
				message.pop_back();
				Put24(message, 1, message.size() - 4);
				Put24(message, 5, message.size() - 8);
				break;
			case 6:
				message = {20, 0, 0, 32};
				message.resize(36);
				break;
		}
		fixture.Encrypted(22, message);
		assert(fixture.Tls.State == TLS_FAILED);
		assert(fixture.Tls.Error == TLS_ERROR_PROTOCOL || fixture.Tls.Error == TLS_ERROR_CERTIFICATE);
	}
	for (unsigned mutation = 0; mutation < 5; mutation++)
	{
		Fixture fixture(Files, Now, true);
		std::vector<uint8_t> message;

		fixture.Hello();
		fixture.Extensions();
		fixture.Certificate(Files);
		message = fixture.CertificateVerify(Files);
		switch (mutation)
		{
			case 0:
				message.back() ^= 1;
				break;
			case 1:
				message[5] = 1;
				break;
			case 2:
				message[7]--;
				break;
			case 3:
				message.push_back(0);
				Put24(message, 1, message.size() - 4);
				break;
			case 4:
				message = {20, 0, 0, 32};
				message.resize(36);
				break;
		}
		fixture.Encrypted(22, message);
		assert(fixture.Tls.State == TLS_FAILED);
		assert(fixture.Tls.Error == TLS_ERROR_AUTH || fixture.Tls.Error == TLS_ERROR_PROTOCOL ||
			fixture.Tls.Error == TLS_ERROR_CERTIFICATE);
	}
	for (bool external : {false, true})
	{
		Fixture fixture(Files, Now, external);
		std::vector<uint8_t> certificate = CertificateMessage(Files, "server.der", true);
		std::vector<uint8_t> verify;
		uint8_t byte = 0;

		fixture.Chunk = 1;
		fixture.Hello();
		fixture.Extensions();
		fixture.Feed(20, {1});
		assert(fixture.Tls.State == TLS_SERVER_CERTIFICATE);
		for (size_t offset = 0; offset < certificate.size(); offset += 13)
		{
			size_t end = offset + 13 < certificate.size() ? offset + 13 : certificate.size();

			fixture.Encrypted(22, {certificate.begin() + offset, certificate.begin() + end});
		}
		assert(fixture.Tls.State == TLS_SERVER_CERT_VERIFY);
		assert(DeviceIntrfTx(&fixture.Tls.DevIntrf, 0, &byte, 1) == 0);
		fixture.Feed(20, {1});
		assert(fixture.Tls.State == TLS_SERVER_CERT_VERIFY);
		verify = fixture.CertificateVerify(Files);
		fixture.Encrypted(22, {verify.begin(), verify.begin() + 3});
		fixture.Encrypted(22, {verify.begin() + 3, verify.end()});
		assert(fixture.Tls.State == TLS_SERVER_FINISHED);
		assert(DeviceIntrfRx(&fixture.Tls.DevIntrf, 0, &byte, 1) == 0);
		fixture.Finished();
		assert(Zero(fixture.Handshake.data(), fixture.Handshake.size()));
		assert(Zero(fixture.Transcript.data(), fixture.Transcript.size()));
	}
	{
		Fixture fixture(Files, Now, true);
		std::vector<uint8_t> verify;

		fixture.Hello();
		fixture.Extensions();
		fixture.Encrypted(22, CertificateMessage(Files, "invalid_point.der"));
		assert(fixture.Tls.State == TLS_SERVER_CERT_VERIFY);
		verify = fixture.CertificateVerify(Files);
		fixture.Encrypted(22, verify);
		fixture.Failed(TLS_ERROR_CERTIFICATE);
		assert(fixture.Tls.CertificateError == X509_STATUS_SIGNATURE);
	}
	{
		Fixture fixture(Files, Now);

		fixture.Hello();
		fixture.Extensions();
		fixture.Encrypted(22, {11, 0, 0x10, 1});
		fixture.Failed(TLS_ERROR_LIMIT);
	}
	{
		Fixture fixture(Files, Now, true);

		fixture.Hello();
		fixture.Extensions();
		fixture.Encrypted(23, {'x'});
		fixture.Failed(TLS_ERROR_PROTOCOL);
	}
	{
		Fixture fixture(Files, Now, true);

		fixture.Hello();
		fixture.Extensions();
		fixture.Certificate(Files);
		TlsProcess(&fixture.Tls, 1, false);
		fixture.Failed(TLS_ERROR_TRANSPORT);
	}
	printf("TLS certificate ordering, fragmentation, timeout, buffer and authentication checks PASS\n");
}

int main(int argc, char **argv)
{
	uint64_t now;
	Credentials files(argc > 1 ? argv[1] : "");

	assert(argc == 3);
	now = strtoull(argv[2], nullptr, 10);
	SignatureDerTests();
	X509Tests(files, now);
	TlsTests(files, now);
	return 0;
}

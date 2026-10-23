/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * TLS 1.3 PSK and P-256 certificate client. RFC 8446 sections 4, 5 and 7.
 */
#include <string.h>
#include "net/tls.h"

namespace {

const uint8_t s_Zero[32] = {};
const uint64_t s_RecordLimit = 1048576U;

struct Scratch {
	uint8_t Data[128] = {};
	~Scratch()
	{
		CryptoSecureWipe(Data, sizeof(Data));
	}
};

uint16_t Get16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] << 8 | p[1]);
}

size_t Get24(const uint8_t *p)
{
	return (size_t)p[0] << 16 | (size_t)p[1] << 8 | p[2];
}

void Put16(uint8_t *p, size_t n)
{
	p[0] = (uint8_t)(n >> 8);
	p[1] = (uint8_t)n;
}

bool IsHandshake(TLS_STATE state)
{
	switch (state)
	{
	case TLS_SERVER_HELLO:
	case TLS_ENCRYPTED_EXTENSIONS:
	case TLS_SERVER_CERTIFICATE:
	case TLS_SERVER_CERT_VERIFY:
	case TLS_SERVER_FINISHED:
	case TLS_CLIENT_FINISHED:
		return true;
	default:
		return false;
	}
}

bool Overlap(const void *a, size_t an, const void *b, size_t bn)
{
	uintptr_t ap = (uintptr_t)a;
	uintptr_t bp = (uintptr_t)b;

	if (an > UINTPTR_MAX - ap || bn > UINTPTR_MAX - bp)
	{
		return true;
	}
	return ap < bp + bn && bp < ap + an;
}

void WipeWork(TlsDev_t *d)
{
	if (d->pTranscript != nullptr && d->pTranscript != d->Transcript)
	{
		CryptoSecureWipe(d->pTranscript, d->TranscriptSize);
	}
	if (d->pHandshake != nullptr && d->pHandshake != d->Handshake)
	{
		CryptoSecureWipe(d->pHandshake, d->HandshakeSize);
	}
	CryptoSecureWipe(d->Transcript, sizeof(d->Transcript));
	CryptoSecureWipe(d->Handshake, sizeof(d->Handshake));
	CryptoSecureWipe(d->ServerPublicKey, sizeof(d->ServerPublicKey));
	d->TranscriptLen = 0;
	d->HandshakeLen = 0;
}

void WipeEphemeral(TlsDev_t *d)
{
	if (d->pKeyAgree != nullptr)
	{
		d->pKeyAgree->KeyReset(d->KeyCtx);
	}
	CryptoSecureWipe(d->KeyCtx, sizeof(d->KeyCtx));
}

void WipeKeys(TlsDev_t *d)
{
	WipeEphemeral(d);
	CryptoSecureWipe(d->Secret, sizeof(d->Secret));
	CryptoSecureWipe(d->ClientSecret, sizeof(d->ClientSecret));
	CryptoSecureWipe(d->ServerSecret, sizeof(d->ServerSecret));
	CryptoSecureWipe(d->TxKey, sizeof(d->TxKey));
	CryptoSecureWipe(d->RxKey, sizeof(d->RxKey));
	CryptoSecureWipe(d->TxIv, sizeof(d->TxIv));
	CryptoSecureWipe(d->RxIv, sizeof(d->RxIv));
	WipeWork(d);
}

bool Fail(TlsDev_t *d, TLS_ERROR error)
{
	WipeKeys(d);
	CryptoSecureWipe(d->Rx, sizeof(d->Rx));
	CryptoSecureWipe(d->Tx, sizeof(d->Tx));
	d->TxLen = 0;
	d->TxOffset = 0;
	d->PlainLen = 0;
	d->RxCount = 0;
	d->State = TLS_FAILED;
	d->Error = error;
	return false;
}

bool CertificateFail(TlsDev_t *d, X509_STATUS status)
{
	d->CertificateError = status;
	return Fail(d, TLS_ERROR_CERTIFICATE);
}

bool Hash(TlsDev_t *d, const uint8_t *p, size_t n, uint8_t *out)
{
	return d->pHash->Hash(CRYPTO_HASH_SHA256, p, n, out) == CRYPTO_STATUS_OK ||
		Fail(d, TLS_ERROR_CRYPTO);
}

bool Mac(TlsDev_t *d, const uint8_t *key, size_t keylen,
		 const uint8_t *p, size_t n, uint8_t *out)
{
	CryptoKey k = {};
	k.Type = CRYPTO_KEY_HMAC;
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_SIGN;
	k.Plain.pData = key;
	k.Plain.Len = keylen;
	return d->pMac->Mac(CRYPTO_MAC_HMAC, k, p, n, out, 32) == CRYPTO_STATUS_OK ||
		Fail(d, TLS_ERROR_CRYPTO);
}

// Every expansion in this SHA-256 profile is at most one HKDF block.
// This is TLS's HkdfLabel encoding, not a second hash or HMAC implementation.
bool Expand(TlsDev_t *d, const uint8_t *secret, const char *label,
			const uint8_t *context, size_t ctxlen, uint8_t *out, size_t n)
{
	Scratch scratch;
	uint8_t info[96] = {};
	size_t len = strlen(label);
	if (len > 32 || ctxlen > 32 || n > 32)
	{
		return Fail(d, TLS_ERROR_LIMIT);
	}
	Put16(info, n);
	info[2] = (uint8_t)(6 + len);
	memcpy(info + 3, "tls13 ", 6);
	memcpy(info + 9, label, len);
	info[9 + len] = (uint8_t)ctxlen;
	if (ctxlen != 0)
	{
		memcpy(info + 10 + len, context, ctxlen);
	}
	info[10 + len + ctxlen] = 1;
	if (!Mac(d, secret, 32, info, 11 + len + ctxlen, scratch.Data))
	{
		return false;
	}
	memcpy(out, scratch.Data, n);
	return true;
}

bool Append(TlsDev_t *d, const uint8_t *p, size_t n)
{
	if (d->TranscriptLen > d->TranscriptSize ||
		n > d->TranscriptSize - d->TranscriptLen)
	{
		return Fail(d, TLS_ERROR_LIMIT);
	}
	memcpy(d->pTranscript + d->TranscriptLen, p, n);
	d->TranscriptLen += n;
	return true;
}

bool Keys(TlsDev_t *d, bool tx)
{
	const uint8_t *secret = tx ? d->ClientSecret : d->ServerSecret;
	return Expand(d, secret, "key", nullptr, 0, tx ? d->TxKey : d->RxKey, 16) &&
		Expand(d, secret, "iv", nullptr, 0, tx ? d->TxIv : d->RxIv, 12);
}

void Nonce(uint8_t out[12], const uint8_t iv[12], uint64_t sequence)
{
	memcpy(out, iv, 12);
	for (int i = 0; i < 8; i++)
	{
		out[11 - i] ^= (uint8_t)sequence;
		sequence >>= 8;
	}
}

bool Record(TlsDev_t *d, uint8_t type, const uint8_t *p, size_t n)
{
	uint8_t nonce[12];
	CryptoKey key = {};
	if (d->TxLen != 0 || n > 1024 || d->TxSequence >= s_RecordLimit)
	{
		return Fail(d, TLS_ERROR_LIMIT);
	}
	d->Tx[0] = 23;
	d->Tx[1] = 3;
	d->Tx[2] = 3;
	Put16(d->Tx + 3, n + 17);
	memcpy(d->Tx + 5, p, n);
	d->Tx[5 + n] = type;
	key.Type = CRYPTO_KEY_AES_128;
	key.Loc = CRYPTO_KEY_LOC_PLAIN;
	key.Usage = CRYPTO_KEY_USE_ENCRYPT;
	key.Plain.pData = d->TxKey;
	key.Plain.Len = 16;
	Nonce(nonce, d->TxIv, d->TxSequence);
	if (d->pAead->Seal(CRYPTO_AEAD_AES_GCM, key, nonce, sizeof(nonce),
		d->Tx, 5, d->Tx + 5, n + 1, d->Tx + 5, d->Tx + 6 + n, 16) != CRYPTO_STATUS_OK)
	{
		return Fail(d, TLS_ERROR_CRYPTO);
	}
	d->TxSequence++;
	d->TxLen = n + 22;
	d->TxOffset = 0;
	d->TxStarted = d->Now;
	return true;
}

bool ServerHello(TlsDev_t *d, const uint8_t *p, size_t n)
{
	static const uint8_t retry[32] = {
		0xcf,0x21,0xad,0x74,0xe5,0x9a,0x61,0x11,0xbe,0x1d,0x8c,0x02,0x1e,0x65,0xb8,0x91,
		0xc2,0xa2,0x11,0x16,0x7a,0xbb,0x8c,0x5e,0x07,0x9e,0x09,0xe2,0xc8,0xa8,0x33,0x9c
	};
	if (n < 40 || Get16(p) != 0x0303 || memcmp(p + 2, retry, 32) == 0 ||
		p[34] != 0 || Get16(p + 35) != 0x1301 || p[37] != 0 || Get16(p + 38) != n - 40)
	{
		return Fail(d, TLS_ERROR_PROTOCOL);
	}
	unsigned seen = 0;
	const uint8_t *peer = nullptr;
	Scratch shared;
	for (size_t i = 40; i < n;)
	{
		if (n - i < 4)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		unsigned type = Get16(p + i);
		size_t len = Get16(p + i + 2);
		i += 4;
		if (len > n - i)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		unsigned bit;
		if (type == 43 && len == 2 && Get16(p + i) == 0x0304)
		{
			bit = 1;
		}
		else if (type == 41 && !d->bCertificate && len == 2 && Get16(p + i) == 0)
		{
			bit = 2;
		}
		else if (type == 51 && d->pKeyAgree != nullptr && len == 69 &&
			Get16(p + i) == 23 && Get16(p + i + 2) == 65 && p[i + 4] == 4)
		{
			bit = 4;
			peer = p + i + 5;
		}
		else
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		if (seen & bit)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		seen |= bit;
		i += len;
	}
	if (seen != (d->bCertificate ? 5U : (d->pKeyAgree != nullptr ? 7U : 3U)))
	{
		return Fail(d, TLS_ERROR_AUTH);
	}
	if (d->pKeyAgree != nullptr)
	{
		CRYPTO_STATUS status = d->pKeyAgree->Agree(CRYPTO_CURVE_P256,
			d->KeyCtx, peer, shared.Data, false);
		WipeEphemeral(d);
		if (status != CRYPTO_STATUS_OK)
		{
			return Fail(d, TLS_ERROR_CRYPTO);
		}
	}
	Scratch s;
	if (!Hash(d, nullptr, 0, s.Data) ||
		!Expand(d, d->Secret, "derived", s.Data, 32, s.Data + 32, 32) ||
		!Mac(d, s.Data + 32, 32, shared.Data, 32, d->Secret) ||
		!Hash(d, d->pTranscript, d->TranscriptLen, s.Data) ||
		!Expand(d, d->Secret, "c hs traffic", s.Data, 32, d->ClientSecret, 32) ||
		!Expand(d, d->Secret, "s hs traffic", s.Data, 32, d->ServerSecret, 32) ||
		!Keys(d, true) || !Keys(d, false))
	{
		return false;
	}
	d->bEncrypted = true;
	d->State = TLS_ENCRYPTED_EXTENSIONS;
	return true;
}

bool Finished(TlsDev_t *d, const uint8_t *p, size_t n)
{
	Scratch s;
	uint8_t finished[36] = {20, 0, 0, 32};
	if (n != 32)
	{
		return Fail(d, TLS_ERROR_PROTOCOL);
	}
	if (!Hash(d, d->pTranscript, d->TranscriptLen, s.Data) ||
		!Expand(d, d->ServerSecret, "finished", nullptr, 0, s.Data + 32, 32) ||
		!Mac(d, s.Data + 32, 32, s.Data, 32, s.Data + 64))
	{
		return false;
	}
	uint8_t diff = 0;
	for (size_t i = 0; i < 32; i++)
	{
		diff |= p[i] ^ s.Data[64 + i];
	}
	if (diff != 0)
	{
		return Fail(d, TLS_ERROR_AUTH);
	}
	if (!Append(d, d->pHandshake, 36) ||
		!Hash(d, d->pTranscript, d->TranscriptLen, s.Data) ||
		!Expand(d, d->ClientSecret, "finished", nullptr, 0, s.Data + 32, 32) ||
		!Mac(d, s.Data + 32, 32, s.Data, 32, finished + 4) ||
		!Record(d, 22, finished, sizeof(finished)))
	{
		CryptoSecureWipe(finished, sizeof(finished));
		return false;
	}
	CryptoSecureWipe(finished, sizeof(finished));
	if (!Hash(d, nullptr, 0, s.Data + 32) ||
		!Expand(d, d->Secret, "derived", s.Data + 32, 32, s.Data + 64, 32) ||
		!Mac(d, s.Data + 64, 32, s_Zero, 32, d->Secret) ||
		!Expand(d, d->Secret, "c ap traffic", s.Data, 32, d->ClientSecret, 32) ||
		!Expand(d, d->Secret, "s ap traffic", s.Data, 32, d->ServerSecret, 32) ||
		!Keys(d, true) || !Keys(d, false))
	{
		return false;
	}
	CryptoSecureWipe(d->Secret, sizeof(d->Secret));
	WipeWork(d);
	d->TxSequence = 0;
	d->RxSequence = 0;
	d->State = TLS_CLIENT_FINISHED;
	return true;
}

bool Certificate(TlsDev_t *d, const uint8_t *p, size_t n)
{
	X509Cert_t certs[X509_MAX_CHAIN_CERTS] = {};
	X509VerifyCfg_t cfg = d->CertificateCfg;
	X509_STATUS status;
	size_t i = 4;
	size_t len;
	uint32_t count = 0;
	uint64_t elapsed = (uint32_t)(d->Now - d->Started) / 1000U;

	if (n < 4 || p[0] != 0 || Get24(p + 1) != n - 4 || n == 4)
	{
		return Fail(d, TLS_ERROR_PROTOCOL);
	}
	while (i < n)
	{
		if (n - i < 5)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		if (count == X509_MAX_CHAIN_CERTS)
		{
			return CertificateFail(d, X509_STATUS_LIMIT);
		}
		len = Get24(p + i);
		i += 3;
		if (len == 0 || len > n - i - 2)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		if (len > X509_MAX_CERT_SIZE)
		{
			return CertificateFail(d, X509_STATUS_LIMIT);
		}
		certs[count].pDer = p + i;
		certs[count].Len = (uint32_t)len;
		count++;
		i += len;
		// No CertificateEntry extensions were requested in ClientHello.
		if (Get16(p + i) != 0)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		i += 2;
	}
	if (elapsed > UINT64_MAX - cfg.UnixTime)
	{
		return CertificateFail(d, X509_STATUS_TIME);
	}
	cfg.UnixTime += elapsed;
	status = X509VerifyChain(&cfg, certs, count, d->ServerPublicKey);
	if (status != X509_STATUS_OK)
	{
		return CertificateFail(d, status);
	}
	if (!Append(d, d->pHandshake, d->HandshakeLen))
	{
		return false;
	}
	d->State = TLS_SERVER_CERT_VERIFY;
	return true;
}

bool CertificateVerify(TlsDev_t *d, const uint8_t *p, size_t n)
{
	static const char context[] = "TLS 1.3, server CertificateVerify";
	uint8_t input[64 + sizeof(context) + 32] = {};
	Scratch scratch;
	CRYPTO_STATUS status;

	if (n < 4 || Get16(p) != 0x0403 || Get16(p + 2) != n - 4)
	{
		return Fail(d, TLS_ERROR_PROTOCOL);
	}
	if (!X509EcdsaP256Decode(p + 4, (uint32_t)(n - 4), scratch.Data))
	{
		return CertificateFail(d, X509_STATUS_FORMAT);
	}
	memset(input, 0x20, 64);
	memcpy(input + 64, context, sizeof(context));
	if (!Hash(d, d->pTranscript, d->TranscriptLen, input + 64 + sizeof(context)) ||
		!Hash(d, input, sizeof(input), scratch.Data + 64))
	{
		CryptoSecureWipe(input, sizeof(input));
		return false;
	}
	CryptoSecureWipe(input, sizeof(input));
	status = d->CertificateCfg.pSign->Verify(CRYPTO_CURVE_P256,
		d->ServerPublicKey, scratch.Data + 64, 32, scratch.Data);
	if (status != CRYPTO_STATUS_OK)
	{
		return CertificateFail(d, status == CRYPTO_STATUS_FAIL ?
			X509_STATUS_SIGNATURE : X509_STATUS_CRYPTO);
	}
	if (!Append(d, d->pHandshake, d->HandshakeLen))
	{
		return false;
	}
	CryptoSecureWipe(d->ServerPublicKey, sizeof(d->ServerPublicKey));
	d->State = TLS_SERVER_FINISHED;
	return true;
}

bool Message(TlsDev_t *d)
{
	uint8_t type = d->pHandshake[0];
	const uint8_t *p = d->pHandshake + 4;
	size_t n = d->HandshakeLen - 4;
	if (d->State == TLS_SERVER_HELLO && type == 2)
	{
		return Append(d, d->pHandshake, d->HandshakeLen) && ServerHello(d, p, n);
	}
	if (d->State == TLS_ENCRYPTED_EXTENSIONS && type == 8)
	{
		unsigned seen = 0;
		if (n < 2 || Get16(p) != n - 2)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		for (size_t i = 2; i < n;)
		{
			unsigned bit = 0;
			if (n - i < 4)
			{
				return Fail(d, TLS_ERROR_PROTOCOL);
			}
			unsigned type = Get16(p + i);
			size_t len = Get16(p + i + 2);
			i += 4;
			if (len > n - i)
			{
				return Fail(d, TLS_ERROR_PROTOCOL);
			}
			if (type == 0 && d->bServerName && len == 0)
			{
				bit = 1;
			}
			else if (type == 10 && d->pKeyAgree != nullptr && len >= 4 &&
				(len & 1U) == 0 && Get16(p + i) == len - 2)
			{
				// Server group preferences do not change the negotiated P-256 share.
				bit = 2;
			}
			if (bit == 0 || (seen & bit) != 0)
			{
				return Fail(d, TLS_ERROR_PROTOCOL);
			}
			seen |= bit;
			i += len;
		}
		if (!Append(d, d->pHandshake, d->HandshakeLen))
		{
			return false;
		}
		d->State = d->bCertificate ? TLS_SERVER_CERTIFICATE : TLS_SERVER_FINISHED;
		return true;
	}
	if (d->State == TLS_SERVER_CERTIFICATE && type == 11)
	{
		return Certificate(d, p, n);
	}
	if (d->State == TLS_SERVER_CERT_VERIFY && type == 15)
	{
		return CertificateVerify(d, p, n);
	}
	if (d->State == TLS_SERVER_FINISHED && type == 20)
	{
		return Finished(d, p, n);
	}
	if (d->State == TLS_READY && type == 24 && n == 1 && p[0] <= 1)
	{
		bool reply = p[0] == 1;
		if (!Expand(d, d->ServerSecret, "traffic upd", nullptr, 0, d->ServerSecret, 32) ||
			!Keys(d, false))
		{
			return false;
		}
		d->RxSequence = 0;
		if (reply)
		{
			const uint8_t response[5] = {24, 0, 0, 1, 0};
			if (!Record(d, 22, response, sizeof(response)) ||
				!Expand(d, d->ClientSecret, "traffic upd", nullptr, 0, d->ClientSecret, 32) ||
				!Keys(d, true))
			{
				return false;
			}
			d->TxSequence = 0;
		}
		return true;
	}
	if (d->State == TLS_READY && type == 4)
	{
		// Validate but do not retain NewSessionTicket: no resumption in this profile.
		if (n < 13)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		size_t i = 9U + p[8];
		if (i > n - 4 || Get16(p + i) == 0 || Get16(p + i) > n - i - 4)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		i += 2 + Get16(p + i);
		if (Get16(p + i) != n - i - 2)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		i += 2;
		// Only the early_data ticket extension is defined for this profile.
		if (i != n && !(n - i == 8 && Get16(p + i) == 42 && Get16(p + i + 2) == 4))
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		return true;
	}
	return Fail(d, TLS_ERROR_PROTOCOL);
}

bool Handshake(TlsDev_t *d, const uint8_t *p, size_t n)
{
	while (n != 0)
	{
		size_t need = d->HandshakeLen < 4 ? 4 : 4 + Get24(d->pHandshake + 1);
		if (need > d->HandshakeSize)
		{
			return Fail(d, TLS_ERROR_LIMIT);
		}
		size_t copy = need - d->HandshakeLen;
		if (copy > n)
		{
			copy = n;
		}
		memcpy(d->pHandshake + d->HandshakeLen, p, copy);
		d->HandshakeLen += copy;
		p += copy;
		n -= copy;
		if (d->HandshakeLen < 4)
		{
			continue;
		}
		need = 4 + Get24(d->pHandshake + 1);
		if (need > d->HandshakeSize)
		{
			return Fail(d, TLS_ERROR_LIMIT);
		}
		if (d->HandshakeLen == need)
		{
			// Messages changing keys must end at a record boundary.
			if (n != 0 && (d->pHandshake[0] == 2 || d->pHandshake[0] == 20 || d->pHandshake[0] == 24))
			{
				return Fail(d, TLS_ERROR_PROTOCOL);
			}
			if (!Message(d))
			{
				return false;
			}
			CryptoSecureWipe(d->pHandshake, d->HandshakeLen);
			d->HandshakeLen = 0;
		}
	}
	return true;
}

bool ReceiveRecord(TlsDev_t *d)
{
	uint8_t type = d->Rx[0];
	uint8_t *p = d->Rx + 5;
	size_t n = d->RxNeed - 5;
	if (type == 20 && IsHandshake(d->State) && d->State != TLS_CLIENT_FINISHED &&
		n == 1 && p[0] == 1)
	{
		return true;
	}
	if (d->bEncrypted)
	{
		if (type != 23 || n < 17 || d->RxSequence >= s_RecordLimit)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		uint8_t nonce[12];
		CryptoKey key = {};
		key.Type = CRYPTO_KEY_AES_128;
		key.Loc = CRYPTO_KEY_LOC_PLAIN;
		key.Usage = CRYPTO_KEY_USE_DECRYPT;
		key.Plain.pData = d->RxKey;
		key.Plain.Len = 16;
		Nonce(nonce, d->RxIv, d->RxSequence);
		if (d->pAead->Open(CRYPTO_AEAD_AES_GCM, key, nonce, sizeof(nonce),
			d->Rx, 5, p, n - 16, p, p + n - 16, 16) != CRYPTO_STATUS_OK)
		{
			return Fail(d, TLS_ERROR_AUTH);
		}
		d->RxSequence++;
		n -= 16;
		while (n != 0 && p[n - 1] == 0)
		{
			n--;
		}
		if (n == 0 || n > 16385)
		{
			return Fail(d, TLS_ERROR_PROTOCOL);
		}
		type = p[--n];
	}
	else if (n > 16384 || (type != 22 && type != 21))
	{
		return Fail(d, TLS_ERROR_PROTOCOL);
	}
	if (type != 22 && d->HandshakeLen != 0)
	{
		return Fail(d, TLS_ERROR_PROTOCOL);
	}
	if (type == 22)
	{
		return n != 0 ? Handshake(d, p, n) : Fail(d, TLS_ERROR_PROTOCOL);
	}
	if (type == 23 && d->State == TLS_READY)
	{
		d->PlainOffset = 5;
		d->PlainLen = n;
		return true;
	}
	if (type == 21 && n == 2)
	{
		if (p[1] == 0 && d->State == TLS_READY)
		{
			return TlsClose(d);
		}
		return Fail(d, TLS_ERROR_PEER);
	}
	return Fail(d, TLS_ERROR_PROTOCOL);
}

void NoOp(DevIntrf_t *)
{
}
void Off(DevIntrf_t *p)
{
	TlsReset((TlsDev_t *)p->pDevData);
}
void *Handle(DevIntrf_t *p)
{
	return p->pDevData;
}
uint32_t Rate(DevIntrf_t *)
{
	return 0;
}
uint32_t SetRate(DevIntrf_t *, uint32_t)
{
	return 0;
}
bool StartRx(DevIntrf_t *, uint32_t)
{
	return true;
}
bool StartTx(DevIntrf_t *p, uint32_t)
{
	TlsDev_t *d = (TlsDev_t *)p->pDevData;
	return d->State == TLS_READY;
}
int Rx(DevIntrf_t *p, uint8_t *out, int len)
{
	TlsDev_t *d = (TlsDev_t *)p->pDevData;
	if (out == nullptr || len <= 0 || d->State == TLS_FAILED)
	{
		return 0;
	}
	size_t n = (size_t)len < d->PlainLen ? (size_t)len : d->PlainLen;
	memcpy(out, d->Rx + d->PlainOffset, n);
	CryptoSecureWipe(d->Rx + d->PlainOffset, n);
	d->PlainOffset += n;
	d->PlainLen -= n;
	return (int)n;
}
int Tx(DevIntrf_t *p, const uint8_t *in, int len)
{
	TlsDev_t *d = (TlsDev_t *)p->pDevData;
	if (in == nullptr || len <= 0 || d->State != TLS_READY || d->TxLen != 0)
	{
		return 0;
	}
	size_t n = len < 1024 ? (size_t)len : 1024U;
	return Record(d, 23, in, n) ? (int)n : 0;
}

} // namespace

void TlsReset(TlsDev_t *d)
{
	if (d == nullptr)
	{
		return;
	}
	WipeKeys(d);
	CryptoSecureWipe(d->Rx, sizeof(d->Rx));
	CryptoSecureWipe(d->Tx, sizeof(d->Tx));
	d->State = TLS_IDLE;
	d->Error = TLS_ERROR_NONE;
	d->CertificateError = X509_STATUS_OK;
	d->CertificateCfg = {};
	CryptoSecureWipe(d->ServerName, sizeof(d->ServerName));
	d->pTransport = nullptr;
	d->pKeyAgree = nullptr;
	d->pHandshake = d->Handshake;
	d->HandshakeSize = sizeof(d->Handshake);
	d->pTranscript = d->Transcript;
	d->TranscriptSize = sizeof(d->Transcript);
	d->TxLen = d->TxOffset = d->RxCount = d->PlainOffset = d->PlainLen = 0;
	d->RxNeed = 5;
	d->RxSequence = d->TxSequence = 0;
	d->bEncrypted = false;
	d->bCertificate = false;
	d->bServerName = false;
}

bool TlsInit(TlsDev_t *d, const TlsCfg_t *cfg, DevIntrf_t *transport, uint32_t now)
{
	static const uint8_t versions[] = {0,43,0,3,2,3,4};
	static const uint8_t groups[] = {0,10,0,4,0,2,0,23};
	static const uint8_t signatures[] = {0,13,0,4,0,2,4,3};
	const TlsCertCfg_t *cert;
	DevIntrf_t *dev;
	uint8_t *p;
	uint8_t *handshake;
	uint8_t *transcript;
	size_t handshakeSize;
	size_t transcriptSize;
	size_t hostlen = 0;
	size_t i = 47;
	size_t truncated;
	uint32_t anchor;
	Scratch s;

	if (d == nullptr)
	{
		return false;
	}
	TlsReset(d);
	if (cfg == nullptr || transport == nullptr || transport == &d->DevIntrf ||
		transport->RxData == nullptr || transport->TxData == nullptr ||
		transport->StartRx == nullptr || transport->StartTx == nullptr ||
		transport->StopRx == nullptr || transport->StopTx == nullptr ||
		cfg->pHash == nullptr || cfg->pMac == nullptr || cfg->pAead == nullptr ||
		cfg->pRng == nullptr || !cfg->pRng->IsSecure() || cfg->pHash->IsAsync() ||
		cfg->pMac->IsAsync() || cfg->pAead->IsAsync() || cfg->pRng->IsAsync() ||
		cfg->Timeout == 0 || cfg->Timeout > 0x7fffffffU)
	{
		return Fail(d, TLS_ERROR_CONFIG);
	}
	if (cfg->pKeyAgree != nullptr && (cfg->pKeyAgree->IsAsync() ||
		cfg->pKeyAgree->KeyCtxSize() == 0 ||
		cfg->pKeyAgree->KeyCtxSize() > sizeof(d->KeyCtx) ||
		cfg->pKeyAgree->KeyCtxAlign() == 0 ||
		alignof(uint64_t) % cfg->pKeyAgree->KeyCtxAlign() != 0))
	{
		return Fail(d, TLS_ERROR_CONFIG);
	}
	if (cfg->pServerName != nullptr)
	{
		while (hostlen <= X509_MAX_HOSTNAME_SIZE && cfg->pServerName[hostlen] != 0)
		{
			hostlen++;
		}
		if (hostlen > X509_MAX_HOSTNAME_SIZE)
		{
			return Fail(d, TLS_ERROR_CONFIG);
		}
	}
	cert = cfg->pCert;
	if (cert == nullptr)
	{
		if (cfg->pPsk == nullptr || cfg->PskLen < 16 || cfg->PskLen > 64 ||
			cfg->pIdentity == nullptr || cfg->IdentityLen == 0 || cfg->IdentityLen > 128)
		{
			return Fail(d, TLS_ERROR_CONFIG);
		}
	}
	else
	{
		if (cfg->pPsk != nullptr || cfg->PskLen != 0 || cfg->pIdentity != nullptr ||
			cfg->IdentityLen != 0 || cfg->pKeyAgree == nullptr || hostlen == 0 ||
			cert->pSign == nullptr || cert->pSign->IsAsync() || cert->UnixTime == 0 ||
			cert->UnixTime > 253402300799ULL ||
			cert->pTrustAnchors == nullptr || cert->TrustAnchorCount == 0 ||
			cert->TrustAnchorCount > X509_MAX_TRUST_ANCHORS ||
			Overlap(cert->pTrustAnchors, cert->TrustAnchorCount * sizeof(X509Cert_t),
				d, sizeof(*d)) ||
			(cert->pHandshakeBuffer == nullptr) != (cert->HandshakeBufferSize == 0) ||
			(cert->pTranscriptBuffer == nullptr) != (cert->TranscriptBufferSize == 0))
		{
			return Fail(d, TLS_ERROR_CONFIG);
		}
		for (anchor = 0; anchor < cert->TrustAnchorCount; anchor++)
		{
			if (cert->pTrustAnchors[anchor].pDer == nullptr ||
				cert->pTrustAnchors[anchor].Len == 0 ||
				cert->pTrustAnchors[anchor].Len > X509_MAX_CERT_SIZE ||
				Overlap(cert->pTrustAnchors[anchor].pDer,
					cert->pTrustAnchors[anchor].Len, d, sizeof(*d)))
			{
				return Fail(d, TLS_ERROR_CONFIG);
			}
		}
		handshake = cert->pHandshakeBuffer;
		handshakeSize = cert->HandshakeBufferSize;
		transcript = cert->pTranscriptBuffer;
		transcriptSize = cert->TranscriptBufferSize;
		if ((handshake != nullptr && (handshakeSize < 4 ||
			handshakeSize > TLS_HANDSHAKE_MAX_SIZE ||
			Overlap(handshake, handshakeSize, d, sizeof(*d)))) ||
			(transcript != nullptr && (transcriptSize < 4 ||
			transcriptSize > TLS_TRANSCRIPT_MAX_SIZE ||
			Overlap(transcript, transcriptSize, d, sizeof(*d)))) ||
			(handshake != nullptr && transcript != nullptr &&
			Overlap(handshake, handshakeSize, transcript, transcriptSize)))
		{
			return Fail(d, TLS_ERROR_CONFIG);
		}
		if ((handshake != nullptr && Overlap(handshake, handshakeSize,
			cert->pTrustAnchors, cert->TrustAnchorCount * sizeof(X509Cert_t))) ||
			(transcript != nullptr && Overlap(transcript, transcriptSize,
			cert->pTrustAnchors, cert->TrustAnchorCount * sizeof(X509Cert_t))))
		{
			return Fail(d, TLS_ERROR_CONFIG);
		}
		for (anchor = 0; anchor < cert->TrustAnchorCount; anchor++)
		{
			if ((handshake != nullptr && Overlap(handshake, handshakeSize,
				cert->pTrustAnchors[anchor].pDer, cert->pTrustAnchors[anchor].Len)) ||
				(transcript != nullptr && Overlap(transcript, transcriptSize,
				cert->pTrustAnchors[anchor].pDer, cert->pTrustAnchors[anchor].Len)))
			{
				return Fail(d, TLS_ERROR_CONFIG);
			}
		}
		d->pHandshake = handshake != nullptr ? handshake : d->Handshake;
		d->HandshakeSize = handshake != nullptr ? handshakeSize : sizeof(d->Handshake);
		d->pTranscript = transcript != nullptr ? transcript : d->Transcript;
		d->TranscriptSize = transcript != nullptr ? transcriptSize : sizeof(d->Transcript);
		d->CertificateCfg.pHash = cfg->pHash;
		d->CertificateCfg.pSign = cert->pSign;
		d->CertificateCfg.pTrustAnchors = cert->pTrustAnchors;
		d->CertificateCfg.TrustAnchorCount = cert->TrustAnchorCount;
		d->CertificateCfg.pHostName = d->ServerName;
		d->CertificateCfg.UnixTime = cert->UnixTime;
		d->bCertificate = true;
	}
	d->pTransport = transport;
	d->pHash = cfg->pHash;
	d->pMac = cfg->pMac;
	d->pAead = cfg->pAead;
	d->pKeyAgree = cfg->pKeyAgree;
	d->Timeout = cfg->Timeout;
	d->Started = d->Now = d->TxStarted = now;
	d->bServerName = hostlen != 0;
	if (hostlen != 0)
	{
		memcpy(d->ServerName, cfg->pServerName, hostlen);
	}
	d->ServerName[hostlen] = 0;
	dev = &d->DevIntrf;
	dev->pDevData = d;
	dev->Type = DEVINTRF_TYPE_UNKOWN;
	dev->Enable = NoOp;
	dev->Disable = NoOp;
	dev->PowerOff = Off;
	dev->Reset = Off;
	dev->GetHandle = Handle;
	dev->GetRate = Rate;
	dev->SetRate = SetRate;
	dev->StartRx = StartRx;
	dev->StartTx = StartTx;
	dev->StopRx = dev->StopTx = NoOp;
	dev->RxData = Rx;
	dev->TxData = Tx;
	dev->TxSrData = Tx;
	dev->EvtCB = nullptr;
	dev->MaxRetry = 0;
	dev->IntPrio = 0;
	dev->bDma = false;
	dev->bIntEn = false;
	dev->bNoStop = false;
	atomic_store(&dev->bTxReady, true);
	dev->EnCnt = 1;
	atomic_flag_clear(&dev->bBusy);

	p = d->Tx + 5;
	p[0] = 1;
	p[1] = 0;
	Put16(p + 4, 0x0303);
	if (cfg->pRng->Random(p + 6, 32) != CRYPTO_STATUS_OK)
	{
		return Fail(d, TLS_ERROR_CRYPTO);
	}
	p[38] = 0;
	Put16(p + 39, 2);
	Put16(p + 41, 0x1301);
	p[43] = 1;
	p[44] = 0;
	memcpy(p + i, versions, sizeof(versions));
	i += sizeof(versions);
	if (d->bCertificate)
	{
		memcpy(p + i, signatures, sizeof(signatures));
		i += sizeof(signatures);
	}
	else
	{
		Put16(p + i, 45);
		Put16(p + i + 2, 2);
		p[i + 4] = 1;
		p[i + 5] = d->pKeyAgree != nullptr ? 1 : 0;
		i += 6;
	}
	if (d->pKeyAgree != nullptr)
	{
		memcpy(p + i, groups, sizeof(groups));
		i += sizeof(groups);
		Put16(p + i, 51);
		Put16(p + i + 2, 71);
		Put16(p + i + 4, 69);
		Put16(p + i + 6, 23);
		Put16(p + i + 8, 65);
		p[i + 10] = 4;
		if (d->pKeyAgree->KeyGen(CRYPTO_CURVE_P256, d->KeyCtx,
			p + i + 11) != CRYPTO_STATUS_OK)
		{
			return Fail(d, TLS_ERROR_CRYPTO);
		}
		i += 75;
	}
	if (hostlen != 0)
	{
		Put16(p + i, 0);
		Put16(p + i + 2, hostlen + 5);
		Put16(p + i + 4, hostlen + 3);
		p[i + 6] = 0;
		Put16(p + i + 7, hostlen);
		memcpy(p + i + 9, d->ServerName, hostlen);
		i += 9 + hostlen;
	}
	if (d->bCertificate)
	{
		Put16(p + 2, i - 4);
		Put16(p + 45, i - 47);
		if (!Mac(d, s_Zero, 32, s_Zero, 32, d->Secret))
		{
			return false;
		}
	}
	else
	{
		// pre_shared_key must be the final extension; its binder covers the share.
		Put16(p + i, 41);
		Put16(p + i + 2, cfg->IdentityLen + 43);
		Put16(p + i + 4, cfg->IdentityLen + 6);
		Put16(p + i + 6, cfg->IdentityLen);
		memcpy(p + i + 8, cfg->pIdentity, cfg->IdentityLen);
		i += 8 + cfg->IdentityLen;
		memset(p + i, 0, 4);
		i += 4;
		truncated = i;
		Put16(p + i, 33);
		p[i + 2] = 32;
		i += 35;
		Put16(p + 2, i - 4);
		Put16(p + 45, i - 47);
		if (!Mac(d, s_Zero, 32, cfg->pPsk, cfg->PskLen, d->Secret) ||
			!Hash(d, nullptr, 0, s.Data) ||
			!Expand(d, d->Secret, "ext binder", s.Data, 32, s.Data + 32, 32) ||
			!Expand(d, s.Data + 32, "finished", nullptr, 0, s.Data + 64, 32) ||
			!Hash(d, p, truncated, s.Data) ||
			!Mac(d, s.Data + 64, 32, s.Data, 32, p + truncated + 3))
		{
			return false;
		}
	}
	if (!Append(d, p, i))
	{
		return false;
	}
	d->Tx[0] = 22;
	d->Tx[1] = 3;
	d->Tx[2] = 3;
	Put16(d->Tx + 3, i);
	d->TxLen = i + 5;
	d->State = TLS_SERVER_HELLO;
	return true;
}

TLS_STATE TlsProcess(TlsDev_t *d, uint32_t now, bool open)
{
	if (d == nullptr)
	{
		return TLS_FAILED;
	}
	if (d->State == TLS_IDLE || d->State == TLS_FAILED || d->State == TLS_CLOSED)
	{
		return d->State;
	}
	d->Now = now;
	if ((IsHandshake(d->State) && (uint32_t)(now - d->Started) >= d->Timeout) ||
		(d->TxLen != 0 && (uint32_t)(now - d->TxStarted) >= d->Timeout))
	{
		Fail(d, TLS_ERROR_TIMEOUT);
		return d->State;
	}
	if (d->TxLen != 0)
	{
		int count = DeviceIntrfTx(d->pTransport, 0, d->Tx + d->TxOffset,
			(int)(d->TxLen - d->TxOffset));
		if (count < 0 || (size_t)count > d->TxLen - d->TxOffset || !open)
		{
			Fail(d, TLS_ERROR_TRANSPORT);
			return d->State;
		}
		d->TxOffset += (size_t)count;
		if (d->TxOffset != d->TxLen)
		{
			return d->State;
		}
		CryptoSecureWipe(d->Tx, d->TxLen);
		d->TxLen = d->TxOffset = 0;
		if (d->State == TLS_CLIENT_FINISHED)
		{
			d->State = TLS_READY;
		}
		if (d->State == TLS_CLOSING)
		{
			WipeKeys(d);
			d->State = TLS_CLOSED;
			return d->State;
		}
	}
	if (d->PlainLen != 0)
	{
		return d->State;
	}
	int count = DeviceIntrfRx(d->pTransport, 0, d->Rx + d->RxCount,
		(int)(d->RxNeed - d->RxCount));
	if (count < 0 || (size_t)count > d->RxNeed - d->RxCount || (count == 0 && !open))
	{
		Fail(d, TLS_ERROR_TRANSPORT);
		return d->State;
	}
	d->RxCount += (size_t)count;
	if (d->RxCount == 5 && d->RxNeed == 5)
	{
		d->RxNeed = 5U + Get16(d->Rx + 3);
		if (d->Rx[1] != 3 || d->Rx[2] != 3 || d->RxNeed > sizeof(d->Rx))
		{
			Fail(d, TLS_ERROR_PROTOCOL);
			return d->State;
		}
	}
	if (d->RxCount == d->RxNeed)
	{
		ReceiveRecord(d);
		d->RxCount = 0;
		d->RxNeed = 5;
	}
	return d->State;
}

bool TlsClose(TlsDev_t *d)
{
	if (d == nullptr || d->State != TLS_READY || d->TxLen != 0)
	{
		return false;
	}
	const uint8_t close[2] = {1, 0};
	if (!Record(d, 21, close, sizeof(close)))
	{
		return false;
	}
	d->State = TLS_CLOSING;
	return true;
}

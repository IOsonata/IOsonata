/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * TLS 1.3 external PSK client. RFC 8446 sections 4, 5 and 7.
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
	CryptoSecureWipe(d->Transcript, sizeof(d->Transcript));
	CryptoSecureWipe(d->Handshake, sizeof(d->Handshake));
	d->TranscriptLen = 0;
	d->HandshakeLen = 0;
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
	if (n > sizeof(d->Transcript) - d->TranscriptLen)
	{
		return Fail(d, TLS_ERROR_LIMIT);
	}
	memcpy(d->Transcript + d->TranscriptLen, p, n);
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
		else if (type == 41 && len == 2 && Get16(p + i) == 0)
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
	if (seen != (d->pKeyAgree != nullptr ? 7U : 3U))
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
		!Hash(d, d->Transcript, d->TranscriptLen, s.Data) ||
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
	if (!Hash(d, d->Transcript, d->TranscriptLen, s.Data) ||
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
	if (!Append(d, d->Handshake, 36) ||
		!Hash(d, d->Transcript, d->TranscriptLen, s.Data) ||
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
	CryptoSecureWipe(d->Transcript, sizeof(d->Transcript));
	d->TranscriptLen = 0;
	d->TxSequence = 0;
	d->RxSequence = 0;
	d->State = TLS_CLIENT_FINISHED;
	return true;
}

bool Message(TlsDev_t *d)
{
	uint8_t type = d->Handshake[0];
	const uint8_t *p = d->Handshake + 4;
	size_t n = d->HandshakeLen - 4;
	if (d->State == TLS_SERVER_HELLO && type == 2)
	{
		return Append(d, d->Handshake, d->HandshakeLen) && ServerHello(d, p, n);
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
		if (!Append(d, d->Handshake, d->HandshakeLen))
		{
			return false;
		}
		d->State = TLS_SERVER_FINISHED;
		return true;
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
		size_t need = d->HandshakeLen < 4 ? 4 : 4 + Get24(d->Handshake + 1);
		if (need > sizeof(d->Handshake))
		{
			return Fail(d, TLS_ERROR_LIMIT);
		}
		size_t copy = need - d->HandshakeLen;
		if (copy > n)
		{
			copy = n;
		}
		memcpy(d->Handshake + d->HandshakeLen, p, copy);
		d->HandshakeLen += copy;
		p += copy;
		n -= copy;
		if (d->HandshakeLen < 4)
		{
			continue;
		}
		need = 4 + Get24(d->Handshake + 1);
		if (need > sizeof(d->Handshake))
		{
			return Fail(d, TLS_ERROR_LIMIT);
		}
		if (d->HandshakeLen == need)
		{
			// Messages changing keys must end at a record boundary.
			if (n != 0 && (d->Handshake[0] == 2 || d->Handshake[0] == 20 || d->Handshake[0] == 24))
			{
				return Fail(d, TLS_ERROR_PROTOCOL);
			}
			if (!Message(d))
			{
				return false;
			}
			CryptoSecureWipe(d->Handshake, d->HandshakeLen);
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
	if (type == 20 && d->State >= TLS_SERVER_HELLO && d->State <= TLS_SERVER_FINISHED &&
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
	d->pTransport = nullptr;
	d->pKeyAgree = nullptr;
	d->TxLen = d->TxOffset = d->RxCount = d->PlainOffset = d->PlainLen = 0;
	d->RxNeed = 5;
	d->RxSequence = d->TxSequence = 0;
	d->bEncrypted = false;
}

bool TlsInit(TlsDev_t *d, const TlsCfg_t *cfg, DevIntrf_t *transport, uint32_t now)
{
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
		cfg->pPsk == nullptr || cfg->PskLen < 16 || cfg->PskLen > 64 ||
		cfg->pIdentity == nullptr || cfg->IdentityLen == 0 || cfg->IdentityLen > 128 ||
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
	size_t hostlen = 0;
	if (cfg->pServerName != nullptr)
	{
		while (hostlen <= 253 && cfg->pServerName[hostlen] != 0)
		{
			hostlen++;
		}
		if (hostlen > 253)
		{
			return Fail(d, TLS_ERROR_CONFIG);
		}
	}
	d->pTransport = transport;
	d->pHash = cfg->pHash;
	d->pMac = cfg->pMac;
	d->pAead = cfg->pAead;
	d->pKeyAgree = cfg->pKeyAgree;
	d->Timeout = cfg->Timeout;
	d->Started = d->Now = d->TxStarted = now;
	d->bServerName = hostlen != 0;
	DevIntrf_t *dev = &d->DevIntrf;
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

	uint8_t *p = d->Tx + 5;
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
	size_t i = 47;
	const uint8_t extensions[] = {0,43,0,3,2,3,4, 0,45,0,2,1,0};
	memcpy(p + i, extensions, sizeof(extensions));
	i += sizeof(extensions);
	if (d->pKeyAgree != nullptr)
	{
		// Offer only psk_dhe_ke and secp256r1; the PSK binder covers this share.
		p[i - 1] = 1;
		const uint8_t groups[] = {0,10,0,4,0,2,0,23};
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
		memcpy(p + i + 9, cfg->pServerName, hostlen);
		i += 9 + hostlen;
	}
	Put16(p + i, 41);
	Put16(p + i + 2, cfg->IdentityLen + 43);
	Put16(p + i + 4, cfg->IdentityLen + 6);
	Put16(p + i + 6, cfg->IdentityLen);
	memcpy(p + i + 8, cfg->pIdentity, cfg->IdentityLen);
	i += 8 + cfg->IdentityLen;
	memset(p + i, 0, 4);
	i += 4;
	size_t truncated = i;
	Put16(p + i, 33);
	p[i + 2] = 32;
	i += 35;
	Put16(p + 2, i - 4);
	Put16(p + 45, i - 47);
	Scratch s;
	if (!Mac(d, s_Zero, 32, cfg->pPsk, cfg->PskLen, d->Secret) ||
		!Hash(d, nullptr, 0, s.Data) ||
		!Expand(d, d->Secret, "ext binder", s.Data, 32, s.Data + 32, 32) ||
		!Expand(d, s.Data + 32, "finished", nullptr, 0, s.Data + 64, 32) ||
		!Hash(d, p, truncated, s.Data) ||
		!Mac(d, s.Data + 64, 32, s.Data, 32, p + truncated + 3) ||
		!Append(d, p, i))
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
	if ((d->State < TLS_READY && (uint32_t)(now - d->Started) >= d->Timeout) ||
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

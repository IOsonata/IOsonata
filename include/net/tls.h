/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * TLS 1.3 external-PSK client over an existing synchronous stream DeviceIntrf.
 * Profiles: psk_ke or P-256 psk_dhe_ke, AES-128-GCM/SHA-256.
 * No certificates, early data or resumption. Not DTLS.
 */
#ifndef __NET_TLS_H__
#define __NET_TLS_H__

#include "crypto/icrypto.h"

#define TLS_RX_SIZE 16645U
#define TLS_TX_SIZE 1046U
#define TLS_HANDSHAKE_SIZE 2048U

typedef enum {
	TLS_IDLE,
	TLS_SERVER_HELLO,
	TLS_ENCRYPTED_EXTENSIONS,
	TLS_SERVER_FINISHED,
	TLS_CLIENT_FINISHED,
	TLS_READY,
	TLS_CLOSING,
	TLS_CLOSED,
	TLS_FAILED
} TLS_STATE;

typedef enum {
	TLS_ERROR_NONE,
	TLS_ERROR_CONFIG,
	TLS_ERROR_CRYPTO,
	TLS_ERROR_PROTOCOL,
	TLS_ERROR_AUTH,
	TLS_ERROR_LIMIT,
	TLS_ERROR_TIMEOUT,
	TLS_ERROR_TRANSPORT,
	TLS_ERROR_PEER
} TLS_ERROR;

typedef struct {
	HashEngine *pHash;
	MacEngine *pMac;
	AeadEngine *pAead;
	RngEngine *pRng;
	const uint8_t *pPsk;
	size_t PskLen;
	const uint8_t *pIdentity;
	size_t IdentityLen;
	const char *pServerName;
	uint32_t Timeout; /* Handshake and pending-output deadline, ms, 1..INT32_MAX. */
	/* Optional. Non-null requires P-256 PSK-DHE; no fallback to PSK-only.
	 * Enable/configure this synchronous engine with secure RNG before Init. */
	KeyAgreeEngine *pKeyAgree;
} TlsCfg_t;

/* Zero-initialize before first use. All storage belongs to the caller. Do not
 * copy a live instance. Engines and transport must outlive the connection.
 * Credentials/name are consumed by TlsInit and need not remain live afterward.
 * One foreground owner; never call from an ISR or a transport callback.
 */
typedef struct {
	DevIntrf_t DevIntrf;
	DevIntrf_t *pTransport;
	HashEngine *pHash;
	MacEngine *pMac;
	AeadEngine *pAead;
	KeyAgreeEngine *pKeyAgree;
	uint64_t KeyCtx[(CRYPTO_KEYCTX_MAX + 7U) / 8U];
	TLS_STATE State;
	TLS_ERROR Error;
	uint32_t Started;
	uint32_t Now;
	uint32_t Timeout;
	uint32_t TxStarted;
	size_t RxCount;
	size_t RxNeed;
	size_t PlainOffset;
	size_t PlainLen;
	size_t TxOffset;
	size_t TxLen;
	size_t HandshakeLen;
	size_t TranscriptLen;
	uint64_t RxSequence;
	uint64_t TxSequence;
	bool bServerName;
	bool bEncrypted;
	uint8_t Secret[32];
	uint8_t ClientSecret[32];
	uint8_t ServerSecret[32];
	uint8_t TxKey[16];
	uint8_t RxKey[16];
	uint8_t TxIv[12];
	uint8_t RxIv[12];
	uint8_t Rx[TLS_RX_SIZE];
	uint8_t Tx[TLS_TX_SIZE];
	uint8_t Handshake[TLS_HANDSHAKE_SIZE];
	uint8_t Transcript[TLS_HANDSHAKE_SIZE];
} TlsDev_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Transport must already be connected (e.g. SockIntrf with TCP). Its Rx/Tx
 * return an accepted byte count, or zero for would-block. Async -1 is unsupported.
 * Init queues ClientHello; success does not mean authenticated/connected yet.
 */
bool TlsInit(TlsDev_t *pDev, const TlsCfg_t *pCfg, DevIntrf_t *pTransport, uint32_t Now);
/* Bounded pump: one write and one read per call. Pass socket Connected() for
 * bTransportOpen. Buffered bytes are drained before an EOF is treated as truncation.
 */
TLS_STATE TlsProcess(TlsDev_t *pDev, uint32_t Now, bool bTransportOpen);
/* Queue authenticated close_notify once pending output has drained. */
bool TlsClose(TlsDev_t *pDev);
/* Abort/wipe without closing the caller-owned transport. */
void TlsReset(TlsDev_t *pDev);

#ifdef __cplusplus
}

class Tls : public DeviceIntrf {
public:
	Tls() = default;
	~Tls()
	{
		TlsReset(&vDev);
	}
	Tls(const Tls &) = delete;
	Tls &operator=(const Tls &) = delete;
	bool Init(const TlsCfg_t &Cfg, DevIntrf_t *pTransport, uint32_t Now)
	{
		return TlsInit(&vDev, &Cfg, pTransport, Now);
	}
	TLS_STATE Process(uint32_t Now, bool bTransportOpen)
	{
		return TlsProcess(&vDev, Now, bTransportOpen);
	}
	bool Connected() const
	{
		return vDev.State == TLS_READY;
	}
	uint32_t Rate(uint32_t Value) override
	{
		return DeviceIntrfSetRate(*this, Value);
	}
	uint32_t Rate() override
	{
		return DeviceIntrfGetRate(*this);
	}
	TLS_ERROR Error() const
	{
		return vDev.Error;
	}
	bool Close()
	{
		return TlsClose(&vDev);
	}
	void Reset()
	{
		TlsReset(&vDev);
	}
	operator DevIntrf_t *()
	{
		return &vDev.DevIntrf;
	}
	operator TlsDev_t *()
	{
		return &vDev;
	}

private:
	TlsDev_t vDev = {};
};
#endif
#endif

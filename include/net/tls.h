/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * TLS 1.3 client over an existing synchronous stream DeviceIntrf.
 * Profiles: external PSK, P-256 PSK-DHE, or P-256 server certificates.
 * AES-128-GCM/SHA-256. No client certificates, early data or resumption. Not DTLS.
 */
#ifndef __NET_TLS_H__
#define __NET_TLS_H__

#include "crypto/icrypto.h"
#include "crypto/x509.h"

#define TLS_RX_SIZE 16645U
#define TLS_TX_SIZE 1046U
#define TLS_HANDSHAKE_SIZE 2048U
#define TLS_HANDSHAKE_MAX_SIZE 32768U
#define TLS_TRANSCRIPT_MAX_SIZE 65536U

typedef enum {
	TLS_IDLE,
	TLS_SERVER_HELLO,
	TLS_ENCRYPTED_EXTENSIONS,
	TLS_SERVER_FINISHED,
	TLS_CLIENT_FINISHED,
	TLS_READY,
	TLS_CLOSING,
	TLS_CLOSED,
	TLS_FAILED,
	TLS_SERVER_CERTIFICATE,
	TLS_SERVER_CERT_VERIFY
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
	TLS_ERROR_PEER,
	TLS_ERROR_CERTIFICATE
} TLS_ERROR;

/* Certificate profile: P-256 ECDSA/SHA-256, ordered DER chain, DNS SAN match.
 * UnixTime is trusted UTC seconds at TlsInit; zero is not accepted. The caller
 * supplies time from a trusted clock or provisioning, not an unchecked network
 * response. See crypto/x509.h for certificate/path restrictions.
 * Optional work buffers use the internal 2048-byte arrays when pointer and size
 * are both zero. Larger chains require larger caller-owned buffers. Buffers must
 * be disjoint from each other, TlsDev_t, credentials and engine/transport storage.
 * Handshake storage is limited to TLS_HANDSHAKE_MAX_SIZE; transcript storage to
 * TLS_TRANSCRIPT_MAX_SIZE. Active buffers are wiped on finish, failure and reset.
 */
typedef struct {
	SignEngine *pSign;
	const X509Cert_t *pTrustAnchors;
	uint32_t TrustAnchorCount;
	uint64_t UnixTime;
	uint8_t *pHandshakeBuffer;
	size_t HandshakeBufferSize;
	uint8_t *pTranscriptBuffer;
	size_t TranscriptBufferSize;
} TlsCertCfg_t;

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
	/* Optional for PSK; required for certificates. Non-null requires P-256 DHE.
	 * Enable/configure this synchronous engine with secure RNG before Init. */
	KeyAgreeEngine *pKeyAgree;
	/* Null selects external PSK. Non-null requires pServerName, pKeyAgree,
	 * empty PSK/identity fields and a valid certificate configuration. No fallback.
	 */
	const TlsCertCfg_t *pCert;
} TlsCfg_t;

/* Zero-initialize before first use. All storage belongs to the caller. Do not
 * copy a live instance. Engines and transport must outlive the connection.
 * PSK/identity/name and configuration values are consumed/copied by TlsInit.
 * CA array/DER and optional work buffers must remain live until TlsReset.
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
	X509_STATUS CertificateError;
	X509VerifyCfg_t CertificateCfg;
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
	bool bCertificate;
	uint8_t *pHandshake;
	size_t HandshakeSize;
	uint8_t *pTranscript;
	size_t TranscriptSize;
	char ServerName[X509_MAX_HOSTNAME_SIZE + 1U];
	uint8_t ServerPublicKey[64];
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
	X509_STATUS CertificateError() const
	{
		return vDev.CertificateError;
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

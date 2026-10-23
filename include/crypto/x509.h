/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 */
#ifndef __CRYPTO_X509_H__
#define __CRYPTO_X509_H__

#include <stdbool.h>
#include <stdint.h>

#include "crypto/icrypto.h"

#define X509_MAX_CHAIN_CERTS		4U
#define X509_MAX_TRUST_ANCHORS		8U
#define X509_MAX_CERT_SIZE			4096U
#define X509_MAX_HOSTNAME_SIZE		253U

typedef enum {
	X509_STATUS_OK = 0,
	X509_STATUS_ARG = -1,
	X509_STATUS_FORMAT = -2,
	X509_STATUS_UNSUPPORTED = -3,
	X509_STATUS_UNTRUSTED = -4,
	X509_STATUS_TIME = -5,
	X509_STATUS_HOSTNAME = -6,
	X509_STATUS_CONSTRAINT = -7,
	X509_STATUS_SIGNATURE = -8,
	X509_STATUS_CRYPTO = -9,
	X509_STATUS_LIMIT = -10
} X509_STATUS;

typedef struct {
	const uint8_t *pDer;
	uint32_t Len;
} X509Cert_t;

typedef struct {
	HashEngine *pHash;
	SignEngine *pSign;
	const X509Cert_t *pTrustAnchors;
	uint32_t TrustAnchorCount;
	const char *pHostName;
	uint64_t UnixTime;
} X509VerifyCfg_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Validate an ordered leaf-to-root server chain with synchronous injected
 * SHA-256 and ECDSA engines. All input storage belongs to the caller and must
 * remain unchanged during this call. No allocation, network access or global
 * state. pLeafKey receives the 64-byte P-256 X || Y only on success; it is
 * cleared on failure. It must not overlap the input certificates/configuration.
 *
 * Supported: DER X.509 v3, uncompressed P-256 public keys, ECDSA/SHA-256
 * certificate signatures, basicConstraints/pathLen, keyUsage, serverAuth EKU,
 * and DNS subjectAltName. A wildcard may occupy only the complete leftmost
 * label and matches one label; its suffix must have at least two labels.
 * ASCII DNS names are compared without case; IP literals, trailing dots,
 * Unicode names and CN fallback are unsupported. Other supported SAN fields
 * are rfc822Name, URI, IP address and registeredID; they do not match the
 * configured DNS name. Other SAN forms and issuer/subject unique IDs are
 * rejected. Issuer/subject names must have identical DER encodings.
 *
 * The leaf SPKI encoding is checked here. Curve membership of that leaf key
 * is checked by SignEngine when TLS verifies CertificateVerify. A valid CA
 * signature over the leaf certificate is not proof that its public point is
 * on the curve; callers must authenticate a signature using the returned key.
 *
 * Every configured anchor is a CA certificate in the same profile. A selected
 * anchor's validity dates, CA flag, key usage, EKU and pathLen are enforced.
 * The anchor is trusted by configuration; its self-signature is not required.
 * UnixTime must be a nonzero, trusted UTC time in seconds since 1970-01-01.
 *
 * This does not build alternate paths or fetch certificates/revocation data.
 * Name constraints, policy constraints/mappings and inhibitAnyPolicy are
 * rejected even when noncritical. Unknown critical extensions are rejected.
 * Unknown noncritical extensions may be ignored, including CRL/AIA pointers;
 * successful verification does not report certificate revocation status.
 */
X509_STATUS X509VerifyChain(const X509VerifyCfg_t *pCfg,
						 const X509Cert_t *pPeerCerts, uint32_t PeerCount,
						 uint8_t pLeafKey[64]);

/* Decode one canonical DER ECDSA signature into 32-byte r || 32-byte s.
 * Both scalars must be in [1, n - 1]. The output is cleared on failure.
 * Input and output must not overlap. Used by TLS CertificateVerify as well
 * as certificate signatures; this function performs no signature check.
 */
bool X509EcdsaP256Decode(const uint8_t *pDer, uint32_t Len,
						 uint8_t pSignature[64]);

#ifdef __cplusplus
}
#endif

#endif

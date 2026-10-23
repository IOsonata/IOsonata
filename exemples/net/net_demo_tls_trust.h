/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Caller-owned DER trust anchors for the native TLS example profile.
 */
#ifndef __NET_DEMO_TLS_TRUST_H__
#define __NET_DEMO_TLS_TRUST_H__

#include "crypto/x509.h"

extern const X509Cert_t g_NetDemoTlsTrustAnchors[];
extern const uint32_t g_NetDemoTlsTrustAnchorCount;

#endif

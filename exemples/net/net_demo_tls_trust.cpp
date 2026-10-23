/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 */
#include "net_demo_tls_trust.h"

// Generate this public-only header beside the project's board.h with
// Python/mqtt_cert_demo.py. Missing provisioning compiles with no trust anchor
// and the setup reports the missing configuration before starting a network.
#if __has_include("net_demo_tls_trust_generated.h")
#include "net_demo_tls_trust_generated.h"
#else
const X509Cert_t g_NetDemoTlsTrustAnchors[] = {{nullptr, 0}};
const uint32_t g_NetDemoTlsTrustAnchorCount = 0;
#endif

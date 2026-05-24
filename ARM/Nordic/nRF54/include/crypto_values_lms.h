// File: include/crypto_values_lms.h
// Stub header for bare-metal builds.
//
// Upstream Peer Manager LESC module includes PSA crypto headers which, in some
// configurations, pull in LMS-related definitions/test vectors via this header.
// The IOsonata project does not implement/require LMS (Leighton–Micali
// Signature) for BLE Secure Connections (ECDH on secp256r1).
//
// This stub exists only to satisfy the include and allow compilation.
// If you need LMS support, integrate a proper PSA/MbedTLS backend and replace
// this file with the vendor-provided header.

#ifndef CRYPTO_VALUES_LMS_H
#define CRYPTO_VALUES_LMS_H

// Intentionally empty.

#endif // CRYPTO_VALUES_LMS_H

/**-------------------------------------------------------------------------
@file	bt_lesc.h

@brief	LE Secure Connections ECDH support for the SoftDevice stacks.

The SoftDevice (nRF52) and sdk-nrf-bm (nRF54) own the SMP state machine but
delegate the P-256 work to the host: the local key pair, the ECDH shared
secret, and the LESC OOB confirm value. This module supplies that work over the
IOsonata CryptoDev_t engine, replacing the SDK nrf_ble_lesc module and its
dependency on nrf_crypto.

The engine is injected by the application before the stack initialises its
security layer (BtLescSetCryptoEngine, then the stack's security init). The
application owns the CryptoDev_t and chooses the engine through CryptoInit,
exactly as it does for the SDC pairing path in bt_app_sdc.cpp.

Byte order: the SoftDevice holds public keys and the DH key little-endian, one
32 byte coordinate at a time. CryptoDev_t works big-endian. This module does the
per-coordinate inversion in both directions.

Two operations differ per stack and are supplied by the port (bt_app_nrf52.cpp,
bt_app_bm.cpp): BtLescDhKeyReply wraps sd_ble_gap_lesc_dhkey_reply, whose arity
differs between the s132 SoftDevice and sdk-nrf-bm, and BtLescLinkCount reports
the combined peripheral and central link count.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __BT_LESC_H__
#define __BT_LESC_H__

#include <stdint.h>

#include "ble_gap.h"

#include "crypto/crypto.h"

/// Peer OOB data lookup. The stack calls this on an OOB DHKey request to obtain
/// the peer OOB set for a connection; return NULL when none is held.
typedef ble_gap_lesc_oob_data_t * (*BtLescOobPeerHandler_t)(uint16_t ConnHdl);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Inject the ECDH engine before the stack security layer initialises.
 *
 * The engine must provide CRYPTO_CAP_ECDH_P256. The application owns it.
 *
 * @param	pEcdh	Engine from CryptoInit.
 */
void BtLescSetCryptoEngine(CryptoDev_t * const pEcdh);

/**
 * @brief	Initialise the module and generate the local key pair.
 *
 * @return	true on success. false when no capable engine was injected.
 */
bool BtLescInit(void);

/**
 * @brief	Generate a fresh local P-256 key pair.
 *
 * @return	true on success.
 */
bool BtLescKeyPairGen(void);

/**
 * @brief	Local public key, little-endian, for the security-params reply.
 *
 * @return	Pointer to the key, or NULL if none has been generated.
 */
ble_gap_lesc_p256_pk_t * BtLescPubKeyGet(void);

/**
 * @brief	Generate the local LESC OOB data set for OOB pairing.
 *
 * @return	true on success.
 */
bool BtLescOobLocalGen(void);

/**
 * @brief	Local LESC OOB data set.
 *
 * @return	Pointer to the set, or NULL if none has been generated.
 */
ble_gap_lesc_oob_data_t * BtLescOobLocalGet(void);

/**
 * @brief	Set the peer OOB data lookup used on an OOB DHKey request.
 *
 * @param	Handler	Lookup, or NULL to clear.
 */
void BtLescOobPeerHandlerSet(BtLescOobPeerHandler_t Handler);

/**
 * @brief	Run any deferred DHKey computation. Call from the main loop.
 *
 * The ECDH is deferred out of the event handler so it does not run in the
 * stack callback context.
 *
 * @return	true on success.
 */
bool BtLescRequestHandler(void);

/**
 * @brief	Feed a BLE event to the module.
 *
 * @param	pEvt	The BLE event.
 */
void BtLescOnBleEvt(const ble_evt_t * pEvt);

//
// Port-supplied. Implemented per stack in bt_app_nrf52.cpp / bt_app_bm.cpp.
//

/**
 * @brief	Reply to a LESC DHKey request. Wraps sd_ble_gap_lesc_dhkey_reply,
 * 			whose arity differs between the s132 SoftDevice and sdk-nrf-bm.
 *
 * @param	ConnHdl		Connection handle.
 * @param	SecStatus	BLE_GAP_SEC_STATUS_* result. Ignored on stacks whose
 * 						reply takes no status argument.
 * @param	pDhKey		DH key (little-endian), or NULL to fail the request.
 *
 * @return	NRF_SUCCESS on success.
 */
uint32_t BtLescDhKeyReply(uint16_t ConnHdl, uint8_t SecStatus,
						  const ble_gap_lesc_dhkey_t * pDhKey);

/**
 * @brief	Combined peripheral and central link count for this build.
 *
 * @return	Number of concurrent links the peer-key table must cover.
 */
int BtLescLinkCount(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_LESC_H__

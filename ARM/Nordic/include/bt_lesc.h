/**-------------------------------------------------------------------------
@file	bt_lesc.h

@brief	LE Secure Connections ECDH support for the SoftDevice stacks.

The SoftDevice (nRF52) and sdk-nrf-bm (nRF54) own the SMP state machine but
delegate the P-256 work to the host: the local key pair, the ECDH shared
secret, and the LESC OOB confirm value. This module supplies that work over the
IOsonata KeyAgreeEngine, replacing the SDK nrf_ble_lesc module and its
dependency on nrf_crypto.

The engine is injected by the application before the stack initialises its
security layer (BtLescSetCryptoEngine, then the stack's security init). The
application owns the engine and constructs it (Ba414ep or CryptoUecc),
exactly as it does for the SDC pairing path in bt_app_sdc.cpp.

Byte order: the SoftDevice holds public keys and the DH key little-endian, one
32 byte coordinate at a time. The engine works big-endian. This module does the
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

#include "ble.h"
#include "ble_gap.h"

#include "crypto/icrypto.h"

// Temporary nRF54 BM reconnect trace. The BM security replacement includes this
// header after the Peer Manager APIs, so these wrappers observe the exact calls
// made by bt_sec_bm.cpp without changing its security decisions. No key bytes
// are printed. Set BT_BM_SECURITY_TRACE to 0 after the hardware fault is found.
#if defined(NRF54L15_XXAA) && defined(__cplusplus)
#ifndef BT_BM_SECURITY_TRACE
#define BT_BM_SECURITY_TRACE	1
#endif

#if BT_BM_SECURITY_TRACE
#include <string.h>
#include <bm/bluetooth/peer_manager/peer_manager_types.h>
#include <modules/id_manager.h>
#include <modules/peer_data_storage.h>
#include "syslog.h"

static inline bool BtBmTraceIrkPresent(const ble_gap_irk_t *pIrk)
{
	if (pIrk == nullptr)
	{
		return false;
	}

	uint8_t present = 0;
	for (uint32_t i = 0; i < BLE_GAP_SEC_KEY_LEN; i++)
	{
		present |= pIrk->irk[i];
	}
	return present != 0;
}

static inline void BtBmTracePeerLookup(uint16_t ConnHdl, uint16_t PeerId)
{
	// Peer Manager asks for the same mapping while emitting several events.
	// Print only when the handle or result changes so one connection produces
	// one useful identity report instead of a line for every PM callback.
	static uint16_t s_LastConnHdl = BLE_CONN_HANDLE_INVALID;
	static uint16_t s_LastPeerId = PM_PEER_ID_INVALID;

	if (ConnHdl == s_LastConnHdl && PeerId == s_LastPeerId)
	{
		return;
	}
	s_LastConnHdl = ConnHdl;
	s_LastPeerId = PeerId;

	ble_gap_addr_t connAddr;
	memset(&connAddr, 0, sizeof(connAddr));
	uint32_t addrStatus = im_ble_addr_get(ConnHdl, &connAddr);

	if (addrStatus == NRF_SUCCESS)
	{
		SysLogPrintf(SysLogGet(),
				"BM ID: hdl=%u peer=%u type=%u addr=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
				(unsigned)ConnHdl, (unsigned)PeerId,
				(unsigned)connAddr.addr_type,
				connAddr.addr[5], connAddr.addr[4], connAddr.addr[3],
				connAddr.addr[2], connAddr.addr[1], connAddr.addr[0]);
	}
	else
	{
		SysLogPrintf(SysLogGet(),
				"BM ID: hdl=%u peer=%u address read failed 0x%08lx\r\n",
				(unsigned)ConnHdl, (unsigned)PeerId,
				(unsigned long)addrStatus);
	}

	if (PeerId != PM_PEER_ID_INVALID)
	{
		return;
	}

	// No connection mapping. Show every stored identity considered by the
	// identity manager and whether it matches directly or resolves the RPA.
	uint16_t peerId;
	uint16_t iter;
	struct pm_peer_data_const peerData;
	uint8_t buffer[PM_PEER_DATA_MAX_SIZE];

	memset(&peerData, 0, sizeof(peerData));
	memset(buffer, 0, sizeof(buffer));
	peerData.all_data = buffer;
	pds_peer_data_iterate_prepare(&iter);

	while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId,
								 &peerData, &iter))
	{
		const ble_gap_id_key_t *pId = &peerData.bonding_data->peer_ble_id;
		bool addrMatch = addrStatus == NRF_SUCCESS &&
			connAddr.addr_type == pId->id_addr_info.addr_type &&
			memcmp(connAddr.addr, pId->id_addr_info.addr,
					BLE_GAP_ADDR_LEN) == 0;
		bool rpaMatch = addrStatus == NRF_SUCCESS &&
			connAddr.addr_type == BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE &&
			BtBmTraceIrkPresent(&pId->id_info) &&
			im_address_resolve(&connAddr, &pId->id_info);

		SysLogPrintf(SysLogGet(),
				"BM ID: candidate=%u type=%u addr=%02x:%02x:%02x:%02x:%02x:%02x "
				"irk=%u direct=%u rpa=%u\r\n",
				(unsigned)peerId, (unsigned)pId->id_addr_info.addr_type,
				pId->id_addr_info.addr[5], pId->id_addr_info.addr[4],
				pId->id_addr_info.addr[3], pId->id_addr_info.addr[2],
				pId->id_addr_info.addr[1], pId->id_addr_info.addr[0],
				BtBmTraceIrkPresent(&pId->id_info) ? 1U : 0U,
				addrMatch ? 1U : 0U, rpaMatch ? 1U : 0U);
	}
}

static inline void BtBmTraceBondRead(uint16_t PeerId, uint32_t DataId,
								 const struct pm_peer_data *pPeerData,
								 uint32_t Status)
{
	if (DataId != PM_PEER_DATA_ID_BONDING)
	{
		return;
	}

	if (Status != NRF_SUCCESS || pPeerData == nullptr ||
		pPeerData->bonding_data == nullptr)
	{
		SysLogPrintf(SysLogGet(),
				"BM SEC: bond read peer=%u status=0x%08lx\r\n",
				(unsigned)PeerId, (unsigned long)Status);
		return;
	}

	const struct pm_peer_data_bonding *pBond = pPeerData->bonding_data;
	const ble_gap_addr_t *pAddr = &pBond->peer_ble_id.id_addr_info;

	SysLogPrintf(SysLogGet(),
			"BM SEC: bond peer=%u own(sc=%u len=%u) peer(sc=%u len=%u) "
			"idtype=%u id=%02x:%02x:%02x:%02x:%02x:%02x irk=%u\r\n",
			(unsigned)PeerId,
			(unsigned)pBond->own_ltk.enc_info.lesc,
			(unsigned)pBond->own_ltk.enc_info.ltk_len,
			(unsigned)pBond->peer_ltk.enc_info.lesc,
			(unsigned)pBond->peer_ltk.enc_info.ltk_len,
			(unsigned)pAddr->addr_type,
			pAddr->addr[5], pAddr->addr[4], pAddr->addr[3],
			pAddr->addr[2], pAddr->addr[1], pAddr->addr[0],
			BtBmTraceIrkPresent(&pBond->peer_ble_id.id_info) ? 1U : 0U);
}

static inline void BtBmTraceEncrypt(uint16_t ConnHdl,
								const ble_gap_master_id_t *pMasterId,
								const ble_gap_enc_info_t *pEncInfo,
								uint32_t Status)
{
	SysLogPrintf(SysLogGet(),
			"BM SEC: encrypt hdl=%u sc=%u len=%u ediv=0x%04x status=0x%08lx\r\n",
			(unsigned)ConnHdl,
			pEncInfo != nullptr ? (unsigned)pEncInfo->lesc : 0U,
			pEncInfo != nullptr ? (unsigned)pEncInfo->ltk_len : 0U,
			pMasterId != nullptr ? (unsigned)pMasterId->ediv : 0U,
			(unsigned long)Status);
}

// Function-like macros preserve the original call signatures. A macro is not
// expanded recursively while its replacement is rescanned, so the call inside
// each lambda reaches the real sdk-nrf-bm function.
#define im_peer_id_get_by_conn_handle(ConnHdl) \
	([&]() -> uint16_t { \
		const uint16_t traceConnHdl = (ConnHdl); \
		const uint16_t tracePeerId = im_peer_id_get_by_conn_handle(traceConnHdl); \
		BtBmTracePeerLookup(traceConnHdl, tracePeerId); \
		return tracePeerId; \
	}())

#define pds_peer_data_read(PeerId, DataId, pPeerData, pBufferSize) \
	([&]() -> uint32_t { \
		const uint16_t tracePeerId = (PeerId); \
		const auto traceDataId = (DataId); \
		struct pm_peer_data *tracePeerData = (pPeerData); \
		const uint32_t *traceBufferSize = (pBufferSize); \
		const uint32_t traceStatus = pds_peer_data_read(tracePeerId, traceDataId, \
				tracePeerData, traceBufferSize); \
		BtBmTraceBondRead(tracePeerId, (uint32_t)traceDataId, tracePeerData, \
				traceStatus); \
		return traceStatus; \
	}())

#define sd_ble_gap_encrypt(ConnHdl, pMasterId, pEncInfo) \
	([&]() -> uint32_t { \
		const uint16_t traceConnHdl = (ConnHdl); \
		const ble_gap_master_id_t *traceMasterId = (pMasterId); \
		const ble_gap_enc_info_t *traceEncInfo = (pEncInfo); \
		const uint32_t traceStatus = sd_ble_gap_encrypt(traceConnHdl, \
				traceMasterId, traceEncInfo); \
		BtBmTraceEncrypt(traceConnHdl, traceMasterId, traceEncInfo, traceStatus); \
		return traceStatus; \
	}())
#endif // BT_BM_SECURITY_TRACE
#endif // NRF54L15_XXAA && __cplusplus

/// Peer OOB data lookup. The stack calls this on an OOB DHKey request to obtain
/// the peer OOB set for a connection; return NULL when none is held.
typedef ble_gap_lesc_oob_data_t * (*BtLescOobPeerHandler_t)(uint16_t ConnHdl);

/**
 * @brief	Inject the P-256 ECDH engine before the stack security layer
 *			initialises. The application owns it. C++ linkage: the engine is a
 *			KeyAgreeEngine on the OO crypto tree (CryptoUecc or Ba414ep).
 *
 * @param	pEcdh	KeyAgreeEngine providing synchronous P-256 ECDH.
 */
void BtLescSetCryptoEngine(KeyAgreeEngine *pEcdh);

#ifdef __cplusplus
extern "C" {
#endif

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
 * @brief	Generate a fresh local P-256 key pair and preserve retryable status.
 *
 * @return	CRYPTO_STATUS_OK, BUSY, or a permanent failure status.
 */
CRYPTO_STATUS BtLescKeyPairGenStatus(void);

/**
 * @brief	Local public key, little-endian.
 *
 * Each connection that receives this key in a security-params reply is
 * registered as a user from its BLE_GAP_EVT_SEC_PARAMS_REQUEST event; the key
 * pair is not replaced while any registered connection is still pairing.
 *
 * @return	Pointer to the key, or NULL if none has been generated.
 */
ble_gap_lesc_p256_pk_t *BtLescPubKeyGet(void);

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
ble_gap_lesc_oob_data_t *BtLescOobLocalGet(void);

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
 * stack callback context. BUSY operations remain queued for the next call.
 *
 * @return	true when processing may continue; false on a permanent local error.
 */
bool BtLescRequestHandler(void);

/**
 * @brief	Feed a BLE event to the module.
 *
 * @param	pEvt	The BLE event.
 */
void BtLescOnBleEvt(const ble_evt_t *pEvt);

//
// Port-supplied. Implemented per stack in bt_app_nrf52.cpp / bt_app_bm.cpp.
//

/**
 * @brief	Reply to a LESC DHKey request. Wraps sd_ble_gap_lesc_dhkey_reply,
 * 			whose arity differs between the s132 SoftDevice and sdk-nrf-bm.
 *
 * @param	ConnHdl		Connection handle.
 * @param	SecStatus	BLE_GAP_SEC_STATUS_* result. Ignored on stacks whose
 * 					reply takes no status argument.
 * @param	pDhKey		DH key (little-endian), or NULL to fail the request.
 *
 * @return	NRF_SUCCESS on success.
 */
uint32_t BtLescDhKeyReply(uint16_t ConnHdl, uint8_t SecStatus,
						  const ble_gap_lesc_dhkey_t *pDhKey);

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

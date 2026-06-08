/**-------------------------------------------------------------------------
@file	nrf_ble_lesc.c (IOsonata replacement)

@brief	LE Secure Connections key agreement for the bm peer_manager, backed by
		the IOsonata CryptoDev_t abstraction instead of the PSA crypto stack.

		This keeps the exact nrf_ble_lesc public API the peer_manager and the
		application expect (nrf_ble_lesc_init, _keypair_generate,
		_request_handler, _public_key_get, _on_ble_evt, OOB helpers), so nothing
		above it changes. The only difference from the stock Nordic file is the
		crypto backend: the five psa_* calls are replaced by calls through a
		CryptoDev_t (crypto/crypto.h). The engine is App-owned and injected via
		BtLescSetCryptoEngine before nrf_ble_lesc_init, exactly as bt_app_sdc.cpp
		owns a CryptoDev_t and injects it into BtSmpInit.

		Backend independence is the point: today the injected engine is software
		P-256 (CryptoUeccInit). On a target with a hardware accelerator (nRF54L
		CRACEN, CryptoCell), the App injects a hardware-backed CryptoDev_t
		instead and this file does not change. The accelerator is reached through
		the same abstraction, not bypassed.

		Key byte order (must match what the SoftDevice expects):
		- BLE carries P-256 public keys and the DHKey little-endian, per
		  coordinate (BLE_GAP_LESC_* buffers).
		- The CryptoDev_t contract (crypto.h) is big-endian X||Y for public keys
		  and big-endian X for the DHKey.
		So every crossing inverts byte order per 32-byte coordinate. There is no
		0x04 uncompressed-format marker on the CryptoDev_t side (that was a PSA
		serialization detail); uecc and the BLE buffers are both raw X||Y.

@author	Hoang Nguyen Hoan
@date	Jun 04, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ble_gap.h>
#include <nrf_error.h>
#include <bm/softdevice_handler/nrf_sdh_ble.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <bm/bluetooth/peer_manager/nrf_ble_lesc.h>

#include "crypto/crypto.h"
#include "syslog.h"

#define LESC_TRACE(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)

LOG_MODULE_DECLARE(peer_manager, CONFIG_PEER_MANAGER_LOG_LEVEL);

/** @brief Descriptor of the peer public key. */
struct lesc_peer_pub_key {
	/** @brief Peer public key. Stored in little-endian. */
	uint8_t value[BLE_GAP_LESC_P256_PK_LEN];
	/** @brief Peer connection handle. */
	uint16_t conn_handle;
	/** @brief Flag indicating the public key was requested to compute DH key. */
	bool is_requested;
};

#define NRF_BLE_LESC_LINK_COUNT CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT

#define COORD_SIZE (BLE_GAP_LESC_P256_PK_LEN / 2)

/** LESC ECC Public Key (little-endian, as BLE carries it). */
__ALIGN(4) static ble_gap_lesc_p256_pk_t lesc_public_key;
/** LESC ECC DH Key (little-endian, as BLE carries it). */
__ALIGN(4) static ble_gap_lesc_dhkey_t lesc_dh_key;

static bool ble_lesc_internal_error;
static bool keypair_generated;

static struct lesc_peer_pub_key peer_keys[NRF_BLE_LESC_LINK_COUNT];

static bool lesc_oobd_own_generated;
static ble_gap_lesc_oob_data_t ble_lesc_oobd_own;
static nrf_ble_lesc_peer_oob_data_handler lesc_oobd_peer_handler;

/** App-injected crypto engine. Must provide CRYPTO_CAP_ECDH_P256. */
static CryptoDev_t *s_pLescCrypto;

/* Convert a secp256r1 public key between big-endian and little-endian. The two
 * 32-byte coordinates are byte-reversed individually.
 */
static void ecc_public_key_byte_order_invert(const uint8_t *raw_key_in, uint8_t *raw_key_out)
{
	sys_memcpy_swap(raw_key_out, raw_key_in, COORD_SIZE);
	sys_memcpy_swap(&raw_key_out[COORD_SIZE], &raw_key_in[COORD_SIZE], COORD_SIZE);
}

/* App injects the crypto engine here before nrf_ble_lesc_init. The App owns the
 * CryptoDev_t instance (CryptoUeccInit or a hardware engine) and passes its
 * pointer in, mirroring the BtSmpInit injection model.
 */
void BtLescSetCryptoEngine(CryptoDev_t *pDev)
{
	s_pLescCrypto = pDev;
}

uint32_t nrf_ble_lesc_init(void)
{
	memset((void *)peer_keys, 0, sizeof(peer_keys));

	if (!CryptoIsCapable(s_pLescCrypto, CRYPTO_CAP_ECDH_P256)) {
		LOG_ERR("LESC crypto engine missing or lacks ECDH P-256");
		return NRF_ERROR_INTERNAL;
	}

	ble_lesc_internal_error = false;
	keypair_generated = false;

	/* Generate the single local ECDH key pair used by this module. */
	return nrf_ble_lesc_keypair_generate();
}

uint32_t nrf_ble_lesc_keypair_generate(void)
{
	uint8_t pub_key_be[64];		/* CryptoDev_t public key: big-endian X||Y */

	/* Do not regenerate while a DH computation is pending. */
	for (uint16_t i = 0; i < ARRAY_SIZE(peer_keys); i++) {
		if (peer_keys[i].is_requested) {
			return NRF_ERROR_BUSY;
		}
	}

	keypair_generated = false;
	lesc_oobd_own_generated = false;

	/* Generate a fresh P-256 key pair. The private key is retained inside the
	 * engine for the matching ECDH call and never crosses the interface.
	 */
	if (CryptoEcdhP256KeyGen(s_pLescCrypto, pub_key_be, NULL) != CRYPTO_STATUS_OK) {
		LOG_ERR("LESC keypair generate failed");
		return NRF_ERROR_INTERNAL;
	}

	/* BLE wants the public key little-endian per coordinate. */
	ecc_public_key_byte_order_invert(pub_key_be, lesc_public_key.pk);

	keypair_generated = true;

	LESC_TRACE("lesc: keypair generated ok\r\n");

	return NRF_SUCCESS;
}

uint32_t nrf_ble_lesc_own_oob_data_generate(void)
{
	uint32_t nrf_err = NRF_ERROR_INVALID_STATE;

	lesc_oobd_own_generated = false;

	if (keypair_generated) {
		nrf_err = sd_ble_gap_lesc_oob_data_get(BLE_CONN_HANDLE_INVALID, &lesc_public_key,
						       &ble_lesc_oobd_own);
		if (nrf_err == NRF_SUCCESS) {
			lesc_oobd_own_generated = true;
		}
	}

	return nrf_err;
}

ble_gap_lesc_p256_pk_t *nrf_ble_lesc_public_key_get(void)
{
	if (!keypair_generated) {
		LOG_ERR("LESC public key accessed before generation.");
		return NULL;
	}
	return &lesc_public_key;
}

ble_gap_lesc_oob_data_t *nrf_ble_lesc_own_oob_data_get(void)
{
	if (!lesc_oobd_own_generated) {
		LOG_ERR("LESC OOB data accessed before generation.");
		return NULL;
	}
	return &ble_lesc_oobd_own;
}

void nrf_ble_lesc_peer_oob_data_handler_set(nrf_ble_lesc_peer_oob_data_handler handler)
{
	lesc_oobd_peer_handler = handler;
}

/* Compute the DH key from the peer public key and answer the DHKey request. */
static uint32_t compute_and_give_dhkey(struct lesc_peer_pub_key *peer_public_key)
{
	uint8_t peer_be[64];		/* peer public key, big-endian for CryptoDev_t */
	uint8_t dh_be[32];			/* DH key, big-endian from CryptoDev_t */
	ble_gap_lesc_dhkey_t *p_dh_key = NULL;
	uint8_t sec_status = BLE_GAP_SEC_STATUS_DHKEY_FAILURE;

	if (!keypair_generated) {
		return NRF_ERROR_INTERNAL;
	}

	/* Reject a peer presenting our own public key (reflection). Compare the X
	 * coordinate only, both little-endian here.
	 */
	if (memcmp(lesc_public_key.pk, peer_public_key->value, COORD_SIZE) == 0) {
		LOG_WRN("Remote peer is using identical public key.");
	} else {
		/* Peer key arrives little-endian; CryptoDev_t wants big-endian X||Y. */
		ecc_public_key_byte_order_invert(peer_public_key->value, peer_be);

		if (CryptoEcdhP256(s_pLescCrypto, peer_be, dh_be, NULL) == CRYPTO_STATUS_OK) {
			/* CryptoDev_t returns the DH key big-endian; BLE wants it
			 * little-endian.
			 */
			for (int i = 0; i < BLE_GAP_LESC_DHKEY_LEN; i++) {
				lesc_dh_key.key[i] = dh_be[BLE_GAP_LESC_DHKEY_LEN - 1 - i];
			}
			p_dh_key = &lesc_dh_key;
			sec_status = BLE_GAP_SEC_STATUS_SUCCESS;
		} else {
			LOG_ERR("LESC ECDH failed");
		}
	}

	LOG_INF("sd_ble_gap_lesc_dhkey_reply(sec_status: %#x) conn_handle: %d",
		sec_status, peer_public_key->conn_handle);

	uint32_t reply_ret = sd_ble_gap_lesc_dhkey_reply(peer_public_key->conn_handle,
							 sec_status, p_dh_key);
	LESC_TRACE("lesc: dhkey_reply sec=%#x ret=%#x\r\n", sec_status, reply_ret);
	return reply_ret;
}

uint32_t nrf_ble_lesc_request_handler(void)
{
	uint32_t nrf_err;

	if (ble_lesc_internal_error) {
		return NRF_ERROR_INTERNAL;
	}

	for (uint16_t i = 0; i < NRF_BLE_LESC_LINK_COUNT; i++) {
		if (peer_keys[i].is_requested) {
			LESC_TRACE("lesc: handler computing idx=%d\r\n", i);
			nrf_err = compute_and_give_dhkey(&peer_keys[i]);
			peer_keys[i].is_requested = false;

			if (nrf_err) {
				return nrf_err;
			}
		}
	}

	return NRF_SUCCESS;
}

static void on_dhkey_request(uint16_t conn_handle, int idx,
			     const ble_gap_evt_lesc_dhkey_request_t *dhkey_request)
{
	const uint8_t *const public_raw = dhkey_request->p_pk_peer->pk;

	memcpy(peer_keys[idx].value, public_raw, BLE_GAP_LESC_P256_PK_LEN);
	peer_keys[idx].conn_handle = conn_handle;
	peer_keys[idx].is_requested = true;
}

static uint32_t lesc_oob_data_set(uint16_t conn_handle)
{
	ble_gap_lesc_oob_data_t *lesc_oobd_own;
	ble_gap_lesc_oob_data_t *lesc_oobd_peer;

	lesc_oobd_own = (lesc_oobd_own_generated) ? &ble_lesc_oobd_own : NULL;
	lesc_oobd_peer =
		(lesc_oobd_peer_handler != NULL) ? lesc_oobd_peer_handler(conn_handle) : NULL;

	return sd_ble_gap_lesc_oob_data_set(conn_handle, lesc_oobd_own, lesc_oobd_peer);
}

void nrf_ble_lesc_on_ble_evt(const ble_evt_t *ble_evt)
{
	__ASSERT(ble_evt, "ble_evt is NULL");

	uint32_t nrf_err = NRF_SUCCESS;
	const uint16_t conn_handle = ble_evt->evt.gap_evt.conn_handle;
	const int idx = nrf_sdh_ble_idx_get(conn_handle);

	LESC_TRACE("lesc: on_ble_evt id=%#x conn=%d idx=%d\r\n",
		ble_evt->header.evt_id, conn_handle, idx);

	__ASSERT(idx >= 0, "Invalid idx %d for conn_handle %#x, evt_id %#x",
		 idx, conn_handle, ble_evt->header.evt_id);

	switch (ble_evt->header.evt_id) {
	case BLE_GAP_EVT_DISCONNECTED:
		peer_keys[idx].is_requested = false;
		break;

	case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
		if (ble_evt->evt.gap_evt.params.lesc_dhkey_request.oobd_req) {
			nrf_err = lesc_oob_data_set(conn_handle);
			if (nrf_err) {
				LOG_ERR("sd_ble_gap_lesc_oob_data_set() returned %#x.", nrf_err);
				ble_lesc_internal_error = true;
			}
		}

		on_dhkey_request(conn_handle, idx,
				 &ble_evt->evt.gap_evt.params.lesc_dhkey_request);
		LESC_TRACE("lesc: dhkey_req queued idx=%d\r\n", idx);
		break;

#if defined(CONFIG_PM_LESC_GENERATE_NEW_KEYS)
	case BLE_GAP_EVT_AUTH_STATUS:
		nrf_err = nrf_ble_lesc_keypair_generate();
		if (nrf_err) {
			ble_lesc_internal_error = true;
		}
		break;
#endif

	default:
		break;
	}
}

/**-------------------------------------------------------------------------
@file	bt_lesc_pm_shim.cpp

@brief	nrf_ble_lesc ABI shim over bt_lesc, for the SDK peer_manager.

The SDK peer_manager (security_manager.c) calls the LESC module by fixed symbol
name: nrf_ble_lesc_init, nrf_ble_lesc_on_ble_evt, nrf_ble_lesc_public_key_get.
This file provides those symbols by forwarding to the bt_lesc implementation, so
the SDK security layer runs on the IOsonata CryptoDev_t engine instead of
nrf_crypto. The remaining nrf_ble_lesc_* entry points that the application calls
directly are forwarded too, so an app can migrate to the Bt* names at its pace.

This shim exists only while the SDK peer_manager is in the build. It is unlinked
when peer_manager is replaced; bt_lesc.cpp then stands on its own with the Bt*
API and no SDK symbol names.

The engine is injected through BtLescSetCryptoEngine before pm_init, because
security_manager's sm_init calls nrf_ble_lesc_init() with no arguments.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include "ble_gap.h"
#include "nrf_error.h"

#include "bt_lesc.h"

extern "C" {

uint32_t nrf_ble_lesc_init(void)
{
	return BtLescInit() ? NRF_SUCCESS : NRF_ERROR_INTERNAL;
}

uint32_t nrf_ble_lesc_keypair_generate(void)
{
	return BtLescKeyPairGen() ? NRF_SUCCESS : NRF_ERROR_INTERNAL;
}

ble_gap_lesc_p256_pk_t * nrf_ble_lesc_public_key_get(void)
{
	return BtLescPubKeyGet();
}

uint32_t nrf_ble_lesc_own_oob_data_generate(void)
{
	return BtLescOobLocalGen() ? NRF_SUCCESS : NRF_ERROR_INTERNAL;
}

ble_gap_lesc_oob_data_t * nrf_ble_lesc_own_oob_data_get(void)
{
	return BtLescOobLocalGet();
}

void nrf_ble_lesc_peer_oob_data_handler_set(BtLescOobPeerHandler_t Handler)
{
	BtLescOobPeerHandlerSet(Handler);
}

uint32_t nrf_ble_lesc_request_handler(void)
{
	return BtLescRequestHandler() ? NRF_SUCCESS : NRF_ERROR_INTERNAL;
}

void nrf_ble_lesc_on_ble_evt(const ble_evt_t * p_ble_evt)
{
	BtLescOnBleEvt(p_ble_evt);
}

}

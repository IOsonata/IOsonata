#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_lesc.h"
#include "bt_test_harness.h"
#include "nrf_error.h"

namespace {

class ScriptedKeyAgree final : public KeyAgreeEngine {
public:
	ScriptedKeyAgree()
	{
		ResetScript();
	}

	void ResetScript()
	{
		vStatus[0] = CRYPTO_STATUS_OK;
		vStatus[1] = CRYPTO_STATUS_OK;
		vStatusCount = 1;
		vKeyGenCalls = 0;
		vAgreeCalls = 0;
	}

	void Script(CRYPTO_STATUS First, CRYPTO_STATUS Second)
	{
		vStatus[0] = First;
		vStatus[1] = Second;
		vStatusCount = 2;
		vKeyGenCalls = 0;
		vAgreeCalls = 0;
	}

	int KeyGenCalls() const { return vKeyGenCalls; }
	int AgreeCalls() const { return vAgreeCalls; }

	bool Enable() override { return true; }
	void Disable() override {}
	void Reset() override {}
	int SelfTest() override { return 0; }

	size_t KeyCtxSize() const override { return 32; }
	size_t KeyCtxAlign() const override { return alignof(uint32_t); }

	void KeyReset(void *pKeyCtx) override
	{
		if (pKeyCtx != nullptr)
		{
			std::memset(pKeyCtx, 0, KeyCtxSize());
		}
	}

	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPublicKey) override
	{
		if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
			pPublicKey == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}

		int index = vKeyGenCalls < vStatusCount ? vKeyGenCalls : vStatusCount - 1;
		CRYPTO_STATUS status = vStatus[index];
		vKeyGenCalls++;
		if (status != CRYPTO_STATUS_OK)
		{
			std::memset(pPublicKey, 0, BLE_GAP_LESC_P256_PK_LEN);
			return status;
		}

		for (size_t i = 0; i < BLE_GAP_LESC_P256_PK_LEN; i++)
		{
			pPublicKey[i] = (uint8_t)(i + 1U);
		}
		std::memset(pKeyCtx, 0xA5, KeyCtxSize());
		return CRYPTO_STATUS_OK;
	}

	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
						const uint8_t *pPeerPublicKey,
						uint8_t *pSharedX, bool) override
	{
		if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
			pPeerPublicKey == nullptr || pSharedX == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}
		vAgreeCalls++;
		for (size_t i = 0; i < BLE_GAP_LESC_DHKEY_LEN; i++)
		{
			pSharedX[i] = (uint8_t)(pPeerPublicKey[i] ^ 0x5AU);
		}
		return CRYPTO_STATUS_OK;
	}

private:
	CRYPTO_STATUS vStatus[2];
	int vStatusCount;
	int vKeyGenCalls;
	int vAgreeCalls;
};

class UnsupportedKeyAgree final : public KeyAgreeEngine {
public:
	bool Enable() override { return true; }
	void Disable() override {}
	void Reset() override {}
};

static uint32_t s_DhReplyCount;
static uint8_t s_LastDhStatus;
static bool s_LastDhHadKey;

static void ResetReplies()
{
	s_DhReplyCount = 0;
	s_LastDhStatus = 0xFF;
	s_LastDhHadKey = false;
}

static ble_evt_t MakeDhRequest(uint16_t ConnHdl,
							 ble_gap_lesc_p256_pk_t *pPeer)
{
	ble_evt_t evt = {};
	evt.header.evt_id = BLE_GAP_EVT_LESC_DHKEY_REQUEST;
	evt.evt.gap_evt.conn_handle = ConnHdl;
	evt.evt.gap_evt.params.lesc_dhkey_request.p_pk_peer = pPeer;
	return evt;
}

} // namespace

extern "C" {

void CryptoSecureWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = static_cast<volatile uint8_t *>(pData);
	while (Len-- != 0)
	{
		*p++ = 0;
	}
}

uint32_t sd_ble_gap_lesc_oob_data_get(uint16_t,
								 ble_gap_lesc_p256_pk_t *,
								 ble_gap_lesc_oob_data_t *pData)
{
	if (pData == nullptr)
	{
		return NRF_ERROR_INVALID_STATE;
	}
	std::memset(pData, 0x3C, sizeof(*pData));
	return NRF_SUCCESS;
}

uint32_t sd_ble_gap_lesc_oob_data_set(uint16_t,
								 ble_gap_lesc_oob_data_t *,
								 ble_gap_lesc_oob_data_t *)
{
	return NRF_SUCCESS;
}

uint32_t BtLescDhKeyReply(uint16_t, uint8_t SecStatus,
						  const ble_gap_lesc_dhkey_t *pDhKey)
{
	s_DhReplyCount++;
	s_LastDhStatus = SecStatus;
	s_LastDhHadKey = pDhKey != nullptr;
	return NRF_SUCCESS;
}

int BtLescLinkCount(void)
{
	return 2;
}

} // extern "C"

int main()
{
	bttest::Context ctx("SoftDevice LESC port tests");
	ScriptedKeyAgree engine;

	ctx.Run("missing engine remains fatal", [&]() {
		BtLescSetCryptoEngine(nullptr);
		BT_CHECK(ctx, !BtLescInit());
		BT_CHECK(ctx, BtLescPubKeyGet() == nullptr);
	});

	ctx.Run("unsupported engine remains fatal", [&]() {
		UnsupportedKeyAgree unsupported;
		BtLescSetCryptoEngine(&unsupported);
		BT_CHECK(ctx, !BtLescInit());
		BT_CHECK(ctx, BtLescPubKeyGet() == nullptr);
	});

	ctx.Run("busy initial key generation is retried", [&]() {
		engine.Script(CRYPTO_STATUS_BUSY, CRYPTO_STATUS_OK);
		BtLescSetCryptoEngine(&engine);
		BT_CHECK(ctx, BtLescInit());
		BT_CHECK(ctx, engine.KeyGenCalls() == 1);
		BT_CHECK(ctx, BtLescPubKeyGet() == nullptr);
		BT_CHECK(ctx, BtLescRequestHandler());
		BT_CHECK(ctx, engine.KeyGenCalls() == 2);
		BT_CHECK(ctx, BtLescPubKeyGet() != nullptr);
	});

	ctx.Run("startup RNG failure is retried", [&]() {
		engine.Script(CRYPTO_STATUS_FAIL, CRYPTO_STATUS_OK);
		BtLescSetCryptoEngine(&engine);
		BT_CHECK(ctx, BtLescInit());
		BT_CHECK(ctx, engine.KeyGenCalls() == 1);
		BT_CHECK(ctx, BtLescPubKeyGet() == nullptr);
		BT_CHECK(ctx, BtLescRequestHandler());
		BT_CHECK(ctx, engine.KeyGenCalls() == 2);
		BT_CHECK(ctx, BtLescPubKeyGet() != nullptr);
	});

	ctx.Run("DHKey request stays deferred to main loop", [&]() {
		engine.Script(CRYPTO_STATUS_OK, CRYPTO_STATUS_OK);
		BtLescSetCryptoEngine(&engine);
		BT_CHECK(ctx, BtLescInit());
		ResetReplies();

		ble_gap_lesc_p256_pk_t peer = {};
		for (size_t i = 0; i < sizeof(peer.pk); i++)
		{
			peer.pk[i] = (uint8_t)(0x80U + i);
		}
		ble_evt_t evt = MakeDhRequest(1, &peer);
		BtLescOnBleEvt(&evt);
		BT_CHECK(ctx, s_DhReplyCount == 0);
		BT_CHECK(ctx, BtLescRequestHandler());
		BT_CHECK(ctx, s_DhReplyCount == 1);
		BT_CHECK(ctx, s_LastDhStatus == BLE_GAP_SEC_STATUS_SUCCESS);
		BT_CHECK(ctx, s_LastDhHadKey);
		BT_CHECK(ctx, engine.AgreeCalls() == 1);
	});

	return ctx.Finish();
}

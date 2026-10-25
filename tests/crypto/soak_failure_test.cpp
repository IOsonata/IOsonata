// An aborted P-256 soak must not report unexecuted checks as passing.
#define CRYPTO_TEST_NO_MAIN
#include <cassert>
#include "../../exemples/crypto/crypto_engine_test.cpp"

class RefuseImport : public CryptoUecc {
public:
	CRYPTO_STATUS KeyImport(CRYPTO_CURVE, void *, const uint8_t *, uint8_t *) override
	{
		return CRYPTO_STATUS_FAIL;
	}
	CRYPTO_STATUS Agree(CRYPTO_CURVE, void *, const uint8_t *, uint8_t *, bool) override
	{
		assert(false);
		return CRYPTO_STATUS_FAIL;
	}
};

int main()
{
	RefuseImport engine;
	TimerCfg_t cfg = {};

	cfg.DevNo = TimerGetHighFreqDevNo();
	cfg.Freq = 1000000U;
	assert(s_Timer.Init(cfg));
	s_pass = s_fail = s_skip = 0;
	PkaSoak(&engine, &engine, &engine, &engine);
	assert(s_pass == 0 && s_fail == 3 && s_skip == 0);
	puts("Expected import failure: all three incomplete soak checks rejected");
	s_Timer.Disable();
	return 0;
}

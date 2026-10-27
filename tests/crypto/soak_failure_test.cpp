/**-------------------------------------------------------------------------
@file	soak_failure_test.cpp

@brief	Host test support: soak failure test.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/

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

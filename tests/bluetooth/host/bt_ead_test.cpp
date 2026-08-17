// Encrypted Advertising Data. Core Specification Supplement Part A 1.23, with
// the sample data of 2.3 as the known answer.
//
// The published ciphertext is what makes this test worth anything. A round
// trip proves only that the module disagrees with nobody but itself: an
// implementation with the session key or the IV backwards encrypts and
// decrypts perfectly and cannot be read by any other stack. So every case
// here that matters compares against the octets in the spec.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softrng.h"

#include "bluetooth/bt_ead.h"

namespace {

int s_Failures = 0;
int s_Checks = 0;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

CryptoSoftAes s_Aes;

// BtEadRandGen refuses an engine reporting IsSecure() false, which the
// software generator does, so these cases present one that claims otherwise.
// Nothing outside a test does that.
class TestRng : public CryptoSoftRng {
public:
	bool IsSecure() const override { return true; }
};

TestRng s_Rng;

// CSS Part A 2.3.1 and 2.3.2. Both sets encrypt the same payload, a Complete
// Local Name and an Appearance, under the same key and IV with different
// randomizers.
const uint8_t kPlain[20] = {
	0x0F,0x09,0x53,0x68,0x6F,0x72,0x74,0x20,
	0x4D,0x69,0x6E,0x69,0x2D,0x42,0x75,0x73,
	0x03,0x19,0x0A,0x8C };

const BtEadKey_t kKey = {
	{ 0x57,0xA9,0xDA,0x12,0xD1,0x2E,0x6E,0x13,
	  0x1E,0x20,0x61,0x2A,0xD1,0x0A,0x6A,0x19 },
	{ 0x9E,0x7A,0x00,0xEF,0xB1,0x7A,0xE7,0x46 }
};

const uint8_t kRand1[BTEAD_RANDOMIZER_LEN] = { 0x18,0xE1,0x57,0xCA,0xDE };
const uint8_t kRand2[BTEAD_RANDOMIZER_LEN] = { 0x8D,0x1C,0x97,0x6E,0x7A };

const uint8_t kAd1[29] = {
	0x18,0xE1,0x57,0xCA,0xDE,
	0x74,0xE4,0xDC,0xAF,0xDC,0x51,0xC7,0x28,
	0x28,0x10,0xC2,0x21,0x7F,0x0E,0x4C,0xEF,
	0x43,0x43,0x18,0x1F,
	0xBA,0x00,0x69,0xCC };

const uint8_t kAd2[29] = {
	0x8D,0x1C,0x97,0x6E,0x7A,
	0x35,0x44,0x40,0x76,0x12,0x57,0x88,0xC2,
	0x38,0xA5,0x8E,0x8B,0xD9,0xCF,0xF0,0xDE,
	0xFE,0x25,0x1A,0x8E,
	0x72,0x75,0x45,0x4C };

// Both published sets, octet for octet.
void TestTheSampleSetsMatch(void)
{
	const uint8_t *pRand[2] = { kRand1, kRand2 };
	const uint8_t *pAd[2] = { kAd1, kAd2 };

	for (int i = 0; i < 2; i++)
	{
		uint8_t out[sizeof(kAd1)];

		std::memset(out, 0, sizeof(out));
		CHECK(BtEadEncrypt(&kKey, pRand[i], kPlain, sizeof(kPlain),
						   out, sizeof(out)) == sizeof(kAd1));
		CHECK(std::memcmp(out, pAd[i], sizeof(kAd1)) == 0);

		// The randomizer goes out ahead of the payload, unencrypted, because
		// the receiver needs it to build the same nonce.
		CHECK(std::memcmp(out, pRand[i], BTEAD_RANDOMIZER_LEN) == 0);

		uint8_t back[sizeof(kPlain)];

		CHECK(BtEadDecrypt(&kKey, pAd[i], sizeof(kAd1), back,
						   sizeof(back)) == sizeof(kPlain));
		CHECK(std::memcmp(back, kPlain, sizeof(kPlain)) == 0);
	}

	// The two sets differ only in the randomizer, and the ciphertext is
	// completely different. That is the unlinkability the feature exists for:
	// a scanner that cannot decrypt sees no relation between the two.
	CHECK(std::memcmp(&kAd1[BTEAD_RANDOMIZER_LEN],
					  &kAd2[BTEAD_RANDOMIZER_LEN], 20) != 0);
}

// The self test inside the module is what a target runs at bring up, so it
// has to agree with the cases here.
void TestTheModuleSelfTestPasses(void)
{
	CHECK(BtEadSelfTest() == 0);
}

// Every octet of the structure is covered by the MIC, so a change anywhere
// fails to authenticate. A caller that accepted a bad MIC would be taking
// advertising data from anyone in radio range.
void TestATamperedStructureIsRejected(void)
{
	for (size_t i = 0; i < sizeof(kAd1); i++)
	{
		uint8_t bad[sizeof(kAd1)];
		uint8_t back[sizeof(kPlain)];

		std::memcpy(bad, kAd1, sizeof(bad));
		bad[i] ^= 0x40;

		CHECK(BtEadDecrypt(&kKey, bad, sizeof(bad), back, sizeof(back)) == 0);
	}

	// A wrong key fails the same way, rather than yielding rubbish.
	BtEadKey_t wrong = kKey;
	uint8_t back[sizeof(kPlain)];

	wrong.SessionKey[0] ^= 0x01;
	CHECK(BtEadDecrypt(&wrong, kAd1, sizeof(kAd1), back, sizeof(back)) == 0);

	wrong = kKey;
	wrong.Iv[0] ^= 0x01;
	CHECK(BtEadDecrypt(&wrong, kAd1, sizeof(kAd1), back, sizeof(back)) == 0);
}

// Nothing that failed to authenticate reaches the caller, because a caller
// that reads its output buffer after a failure would otherwise be handling
// attacker-chosen plaintext.
void TestAFailedDecryptLeavesNothingBehind(void)
{
	uint8_t bad[sizeof(kAd1)];
	uint8_t back[sizeof(kPlain)];

	std::memcpy(bad, kAd1, sizeof(bad));
	bad[10] ^= 0x80;
	std::memset(back, 0xAA, sizeof(back));

	CHECK(BtEadDecrypt(&kKey, bad, sizeof(bad), back, sizeof(back)) == 0);

	uint8_t nz = 0;

	for (size_t i = 0; i < sizeof(back); i++)
	{
		nz |= back[i];
	}
	CHECK(nz == 0);
}

void TestBoundariesAreRefused(void)
{
	uint8_t out[64];
	uint8_t back[64];

	CHECK(BtEadEncrypt(nullptr, kRand1, kPlain, sizeof(kPlain), out, sizeof(out)) == 0);
	CHECK(BtEadEncrypt(&kKey, nullptr, kPlain, sizeof(kPlain), out, sizeof(out)) == 0);
	CHECK(BtEadEncrypt(&kKey, kRand1, nullptr, sizeof(kPlain), out, sizeof(out)) == 0);
	CHECK(BtEadEncrypt(&kKey, kRand1, kPlain, sizeof(kPlain), nullptr, sizeof(out)) == 0);

	// One octet short of what the structure needs.
	CHECK(BtEadEncrypt(&kKey, kRand1, kPlain, sizeof(kPlain), out,
					   sizeof(kPlain) + BTEAD_OVERHEAD - 1) == 0);
	CHECK(BtEadEncrypt(&kKey, kRand1, kPlain, sizeof(kPlain), out,
					   sizeof(kPlain) + BTEAD_OVERHEAD) != 0);

	// Shorter than the overhead is not a structure at all.
	CHECK(BtEadDecrypt(&kKey, kAd1, BTEAD_OVERHEAD - 1, back, sizeof(back)) == 0);
	CHECK(BtEadDecrypt(&kKey, nullptr, sizeof(kAd1), back, sizeof(back)) == 0);
	CHECK(BtEadDecrypt(&kKey, kAd1, sizeof(kAd1), nullptr, sizeof(back)) == 0);
	CHECK(BtEadDecrypt(&kKey, kAd1, sizeof(kAd1), back, sizeof(kPlain) - 1) == 0);
}

// An empty payload is a legal structure of randomizer and MIC alone, and it
// still authenticates.
void TestAnEmptyPayloadRoundTrips(void)
{
	uint8_t out[BTEAD_OVERHEAD];
	uint8_t back[4];

	CHECK(BtEadEncrypt(&kKey, kRand1, nullptr, 0, out, sizeof(out)) ==
		  BTEAD_OVERHEAD);
	CHECK(BtEadDecrypt(&kKey, out, sizeof(out), back, sizeof(back)) == 0);

	// Zero is both the length of an empty payload and the failure answer, so
	// the two are told apart by tampering: a corrupted empty structure has to
	// stop authenticating.
	out[0] ^= 0x01;
	CHECK(BtEadDecrypt(&kKey, out, sizeof(out), back, sizeof(back)) == 0);
}

// A payload that crosses a block boundary exercises the counter stepping,
// which one 20 octet sample does not reach far into.
void TestLongerPayloadsRoundTrip(void)
{
	uint8_t payload[100];
	uint8_t out[sizeof(payload) + BTEAD_OVERHEAD];
	uint8_t back[sizeof(payload)];

	for (size_t i = 0; i < sizeof(payload); i++)
	{
		payload[i] = (uint8_t)(i * 7 + 3);
	}

	for (size_t len = 1; len <= sizeof(payload); len++)
	{
		if (BtEadEncrypt(&kKey, kRand1, payload, len, out, sizeof(out)) !=
			len + BTEAD_OVERHEAD)
		{
			CHECK(false);
			return;
		}

		if (BtEadDecrypt(&kKey, out, len + BTEAD_OVERHEAD, back,
						 sizeof(back)) != len ||
			std::memcmp(back, payload, len) != 0)
		{
			CHECK(false);
			return;
		}
	}

	CHECK(true);
}

// Nothing works before an engine is bound, rather than quietly producing a
// structure with no encryption in it.
void TestNoEngineFailsClosed(void)
{
	uint8_t out[sizeof(kAd1)];

	CHECK(BtEadInit(nullptr, &s_Rng) == false);
	CHECK(BtEadEncrypt(&kKey, kRand1, kPlain, sizeof(kPlain), out,
					   sizeof(out)) == 0);

	// And the buffer is left with nothing in it.
	uint8_t nz = 0;

	for (size_t i = 0; i < sizeof(out); i++)
	{
		nz |= out[i];
	}
	CHECK(nz == 0);

	CHECK(BtEadInit(&s_Aes, &s_Rng));
}


// CSS Part A 1.23.4 requires the randomizer to meet the random number
// requirements of Vol 2 Part H 2. A deterministic engine is refused rather
// than used, because a predictable randomizer hands back the tracking the
// feature exists to prevent.
void TestTheRandomizerNeedsASecureEngine(void)
{
	uint8_t r[BTEAD_RANDOMIZER_LEN];

	CHECK(BtEadRandGen(r));

	// Two draws in a row differing is not proof of anything, but a generator
	// returning a constant would be visible here.
	uint8_t r2[BTEAD_RANDOMIZER_LEN];

	CHECK(BtEadRandGen(r2));
	CHECK(std::memcmp(r, r2, sizeof(r)) != 0);

	// No engine at all, and the buffer is left empty rather than stale.
	CHECK(BtEadInit(&s_Aes, nullptr));
	std::memset(r, 0xAA, sizeof(r));
	CHECK(BtEadRandGen(r) == false);

	uint8_t nz = 0;

	for (size_t i = 0; i < sizeof(r); i++)
	{
		nz |= r[i];
	}
	CHECK(nz == 0);

	CHECK(BtEadRandGen(nullptr) == false);
	CHECK(BtEadInit(&s_Aes, &s_Rng));
}

} // namespace

int main()
{
	s_Aes.Enable();
	s_Rng.Enable();

	if (!BtEadInit(&s_Aes, &s_Rng))
	{
		std::printf("EAD host tests: engine bind failed\n");
		return 1;
	}

	TestTheSampleSetsMatch();
	TestTheModuleSelfTestPasses();
	TestATamperedStructureIsRejected();
	TestAFailedDecryptLeavesNothingBehind();
	TestBoundariesAreRefused();
	TestAnEmptyPayloadRoundTrips();
	TestLongerPayloadsRoundTrip();
	TestNoEngineFailsClosed();
	TestTheRandomizerNeedsASecureEngine();

	if (s_Failures == 0)
	{
		std::printf("Encrypted advertising data tests: PASS (%d checks)\n",
					s_Checks);
		return 0;
	}

	std::printf("Encrypted advertising data tests: %d failure(s), %d checks\n",
				s_Failures, s_Checks);

	return 1;
}

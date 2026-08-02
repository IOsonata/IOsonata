#include <signal.h>
#include <stdint.h>

#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_smp.h"
#include "crypto/crypto_softaes.h"
#include "syslog.h"

namespace {

CryptoSoftAes s_Aes;

__attribute__((constructor)) void IgnoreBrokenPipeSignal()
{
	// A worker can terminate after reporting a fatal protocol error. The parent
	// still owns the pipe long enough to collect that result; writing the final
	// stop message must return EPIPE rather than terminate the whole test process.
	signal(SIGPIPE, SIG_IGN);
}

} // namespace

extern "C" {

// bt_smp.cpp is compiled with its initializer renamed for this host test. The
// wrapper keeps deterministic ECDH and RNG from each worker but replaces the
// temporary test mixer with IOsonata's production AES-128 implementation.
bool BtSmpInitReal(KeyAgreeEngine *pEcdh, CipherEngine *pAes, RngEngine *pRng);

bool BtSmpInit(KeyAgreeEngine *pEcdh, CipherEngine *pAes, RngEngine *pRng)
{
	(void)pAes;
	if (!s_Aes.Enable())
	{
		return false;
	}
	return BtSmpInitReal(pEcdh, &s_Aes, pRng);
}

void BtGattCccdRestoreBonded(uint16_t ConnHdl)
{
	(void)ConnHdl;
}

SysLog_t *SysLogGet(void)
{
	static SysLog_t log = {};
	return &log;
}

int SysLogPrintf(SysLog_t * const pLog, const char *pFormat, ...)
{
	(void)pLog;
	(void)pFormat;
	return 0;
}

} // extern "C"

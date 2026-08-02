#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_smp.h"
#include "crypto/crypto_softaes.h"
#include "syslog.h"

namespace {
CryptoSoftAes s_Aes;
__attribute__((constructor)) void IgnoreBrokenPipeSignal() { signal(SIGPIPE, SIG_IGN); }
}

extern "C" {
bool BtSmpInitReal(KeyAgreeEngine *pEcdh, CipherEngine *pAes, RngEngine *pRng);
bool BtSmpInit(KeyAgreeEngine *pEcdh, CipherEngine *pAes, RngEngine *pRng)
{
	(void)pAes;
	return s_Aes.Enable() && BtSmpInitReal(pEcdh, &s_Aes, pRng);
}
void BtGattCccdRestoreBonded(uint16_t ConnHdl) { (void)ConnHdl; }
SysLog_t *SysLogGet(void) { static SysLog_t log = {}; return &log; }
int SysLogPrintf(SysLog_t * const pLog, const char *pFormat, ...)
{
	(void)pLog;
	static unsigned count;
	if (pFormat == nullptr || count++ >= 80) return 0;
	fprintf(stderr, "[smp %ld] ", (long)getpid());
	va_list args;
	va_start(args, pFormat);
	int rc = vfprintf(stderr, pFormat, args);
	va_end(args);
	return rc;
}
}

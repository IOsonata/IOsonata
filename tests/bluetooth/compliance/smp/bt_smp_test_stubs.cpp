#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "bluetooth/bt_gatt.h"
#include "syslog.h"

namespace {

__attribute__((constructor)) void IgnoreBrokenPipeSignal()
{
	// A worker can terminate after reporting a fatal protocol error. The parent
	// still owns the pipe long enough to collect that result; writing the final
	// stop message must return EPIPE rather than terminate the whole test process.
	signal(SIGPIPE, SIG_IGN);
}

} // namespace

extern "C" {

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
	static unsigned count;
	if (count++ >= 24 || pFormat == nullptr)
	{
		return 0;
	}

	fprintf(stderr, "[smp-worker %ld] ", (long)getpid());
	va_list args;
	va_start(args, pFormat);
	int result = vfprintf(stderr, pFormat, args);
	va_end(args);
	return result;
}

} // extern "C"

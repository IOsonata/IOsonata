#include <signal.h>
#include <stdint.h>

#include "bluetooth/bt_gatt.h"

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

} // extern "C"

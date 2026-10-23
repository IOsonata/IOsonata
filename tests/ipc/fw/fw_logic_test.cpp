/**-------------------------------------------------------------------------
@file	fw_logic_test.cpp

@brief	Runs the IpcIntrf test firmware (exemples/ipc/ipc_intrf_test.cpp) on the host

The owner and the peer role are two translation units of the same firmware
source. They share the memory block through the synchronous IpcIntrf port, and
every delay the peer takes lets the owner run, so one thread plays both cores.
This checks the logic of the test itself, so a failure on the DK points at the
link and not at the test. The exit code is 0 when the peer prints RESULT PASS.

@license	MIT, Copyright (c) 2026, I-SYST inc.
----------------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "syslog.h"
#include "ipc/ipc_intrf.h"

#include "../../bluetooth/host/ipc_sync_port.h"

alignas(8) uint8_t g_HostIpcMem[0x2000];

#include "nrf.h"

HostReset_t g_HostReset;

bool IpcTestOwnerInit(void);
void IpcTestOwnerStep(void);
extern int peer_main(void);

static bool s_OwnerUp;
static unsigned long s_IdleCalls;
static int s_Led[4];

extern "C" void HostIdle(uint32_t)
{
	if (s_OwnerUp == false && g_HostReset.NETWORK.FORCEOFF == RESET_NETWORK_FORCEOFF_FORCEOFF_Release)
	{
		s_OwnerUp = IpcTestOwnerInit();
		if (s_OwnerUp == false)
		{
			printf("owner init failed\n");
			exit(2);
		}
	}
	if (s_OwnerUp)
	{
		for (int i = 0; i < 4; i++)
		{
			IpcTestOwnerStep();
		}
	}
	if (++s_IdleCalls > 200000000UL)
	{
		printf("no progress, giving up\n");
		exit(3);
	}
}

extern "C" void HostPin(int, int Pin, int Level)
{
	if (Pin >= 28 && Pin <= 31)
	{
		s_Led[Pin - 28] = Level;
	}
}

extern "C" void HostPinToggle(int, int Pin)
{
	if (Pin >= 28 && Pin <= 31)
	{
		s_Led[Pin - 28] ^= 1;
	}
}

extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}

extern "C" void UARTSetCtrlLineState(UARTDev_t * const, uint32_t)
{
}

extern "C" bool UARTInit(UARTDev_t * const, const UARTCfg_t * const)
{
	return true;
}

// The trace of both cores goes through SysLog; here it is the line check
// of the run.
static SysLog_t s_HostLog;

extern "C" SysLog_t *SysLogGet(void)
{
	return &s_HostLog;
}

extern "C" bool SysLogInit(SysLog_t * const, const SysLogCfg_t * const,
						   DevIntrf_t * const, uint32_t, TimerDev_t * const, uint32_t)
{
	return true;
}

extern "C" void SysLogSetSink(SysLog_t * const, DevIntrf_t * const, uint32_t)
{
}

extern "C" int SysLogFlush(SysLog_t * const)
{
	return 0;
}

extern "C" int SysLogPrintf(SysLog_t * const, const char *pFormat, ...)
{
	char line[512];
	va_list vl;

	va_start(vl, pFormat);
	vsnprintf(line, sizeof(line), pFormat, vl);
	va_end(vl);
	fputs(line, stdout);

	if (strstr(line, "RESULT PASS") != nullptr)
	{
		// LED4 (index 3) is on when active low is 0 and LED3 stayed off
		int ledOk = s_Led[3] == 0 && s_Led[2] == 1;

		printf("leds: link=%d error=%d done=%d\n", s_Led[0] == 0, s_Led[2] == 0, s_Led[3] == 0);
		exit(ledOk ? 0 : 4);
	}
	if (strstr(line, "RESULT FAIL") != nullptr)
	{
		exit(1);
	}
	if (strstr(line, "STALL") != nullptr)
	{
		exit(5);
	}

	return (int)strlen(line);
}

int main(void)
{
	// The LEDs start dark: active low
	for (int i = 0; i < 4; i++)
	{
		s_Led[i] = 1;
	}
	peer_main();
	return 6;
}

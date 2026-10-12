/* Shared LPC IAP EEPROM page command test. No hardware is programmed. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "lpc_iap.h"

typedef void (*IAPENTRY)(IAP_CMD_STATUS *, IAP_CMD_STATUS *);

#ifdef __cplusplus
extern "C" {
#endif
extern IAPENTRY IapEntry;
uint32_t SystemCoreClock = 12000000;
#ifdef __cplusplus
}
#endif

static uint32_t s_Status;
static uint32_t s_Page;
static uint32_t s_ClockKhz;
static unsigned s_Calls;
static uint32_t s_Data[32];

static void RomCall(IAP_CMD_STATUS *pCmd, IAP_CMD_STATUS *pRes)
{
	assert(pCmd->CmdStatus == 81);
	assert(pCmd->Data[0] == s_Page);
	assert(pCmd->Data[1] == (uint32_t)(uintptr_t)s_Data);
	assert(pCmd->Data[2] == s_ClockKhz);
	s_Calls++;
	pRes->CmdStatus = s_Status;
}

int main(void)
{
	const uint32_t clocks[] = {12000000, 48000000};
	const uint32_t statuses[] = {IAPSTATUS_SUCCESS, IAPSTATUS_INVALID_SECT,
		IAPSTATUS_SRC_ADDR_NOT_MAPPED, IAPSTATUS_INVALID_COMMAND};

	IapEntry = RomCall;
	for (unsigned c = 0; c < sizeof(clocks) / sizeof(clocks[0]); c++)
	{
		SystemCoreClock = clocks[c];
		s_ClockKhz = clocks[c] / 1000;
		for (unsigned i = 0; i < sizeof(statuses) / sizeof(statuses[0]); i++)
		{
			s_Status = statuses[i];
			s_Page = i;
			s_Calls = 0;
			assert((uint32_t)IAPEepromWritePage(s_Page, (uint8_t *)s_Data) == s_Status);
			assert(s_Calls == 1);
		}
	}
	s_Calls = 0;
	assert(IAPEepromWritePage(0, 0) == IAPSTATUS_PARAM_ERROR);
	assert(s_Calls == 0);
	puts("Shared LPC IAP EEPROM page: linkage, parameters and status passed");
	return 0;
}

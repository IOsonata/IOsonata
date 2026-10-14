/* Shared LPC IAP signature regression; ROM calls are substituted. */
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
static uint32_t s_Command = 73;
static unsigned s_Calls;
static const uint32_t s_Signature[4] = {
	0x12345678, 0x87654321, 0xa5a55a5a, 0xff00ff00
};

static void RomCall(IAP_CMD_STATUS *pCmd, IAP_CMD_STATUS *pRes)
{
	assert(pCmd->CmdStatus == s_Command);
	if (s_Command == 40)
	{
		assert(pCmd->Data[0] == 0x40060010);
	}
	if (s_Command == 73)
	{
		assert(pCmd->Data[0] == 0x1000);
		assert(pCmd->Data[1] == 0x1fff);
		assert(pCmd->Data[2] == 3);
		assert(pCmd->Data[3] == 0);
	}
	s_Calls++;
	pRes->CmdStatus = s_Status;
	// Error-result words must not be copied to the caller.
	for (unsigned i = 0; i < 4; i++)
	{
		pRes->Data[i] = 0xbad00bad;
	}
	if (s_Status == IAPSTATUS_SUCCESS)
	{
		for (unsigned i = 0; i < 4; i++)
		{
			pRes->Data[i] = s_Signature[i];
		}
	}
}

int main(void)
{
	uint32_t output[6] = {0xdeadbeef, 0, 0, 0, 0, 0xcafebabe};

	IapEntry = RomCall;
	s_Status = IAPSTATUS_SUCCESS;
	assert(IAPReadExtFlashSignature(0x1000, 0x1fff, 3, &output[1]) == IAPSTATUS_SUCCESS);
	assert(s_Calls == 1);
	for (unsigned i = 0; i < 4; i++)
	{
		assert(output[i + 1] == s_Signature[i]);
	}
	assert(output[0] == 0xdeadbeef);
	assert(output[5] == 0xcafebabe);

	s_Status = IAPSTATUS_INVALID_COMMAND;
	assert(IAPReadExtFlashSignature(0x1000, 0x1fff, 3, &output[1]) == IAPSTATUS_INVALID_COMMAND);
	assert(s_Calls == 2);
	for (unsigned i = 0; i < 4; i++)
	{
		assert(output[i + 1] == s_Signature[i]);
	}
	assert(IAPReadExtFlashSignature(0x1000, 0x1fff, 3, 0) == IAPSTATUS_PARAM_ERROR);
	assert(s_Calls == 2);
	s_Command = 70;
	s_Status = IAPSTATUS_SUCCESS;
	output[1] = 0;
	assert(IAPReadFlashSignature(&output[1]) == IAPSTATUS_SUCCESS);
	assert(s_Calls == 3);
	assert(output[1] == s_Signature[0]);
	assert(output[0] == 0xdeadbeef);
	assert(output[2] == s_Signature[1]);

	s_Status = IAPSTATUS_INVALID_COMMAND;
	assert(IAPReadFlashSignature(&output[1]) == IAPSTATUS_INVALID_COMMAND);
	assert(s_Calls == 4);
	assert(output[1] == s_Signature[0]);
	assert(IAPReadFlashSignature(0) == IAPSTATUS_PARAM_ERROR);
	assert(s_Calls == 4);
	s_Command = 40;
	s_Status = IAPSTATUS_SUCCESS;
	output[1] = 0;
	assert(IAPReadFactorySettings(0x40060010, &output[1]) == IAPSTATUS_SUCCESS);
	assert(s_Calls == 5);
	assert(output[1] == s_Signature[0]);
	assert(output[0] == 0xdeadbeef);
	assert(output[2] == s_Signature[1]);

	const uint32_t errors[] = {IAPSTATUS_PARAM_ERROR, IAPSTATUS_INVALID_COMMAND, IAPSTATUS_BUSY};
	for (unsigned i = 0; i < sizeof(errors) / sizeof(errors[0]); i++)
	{
		s_Status = errors[i];
		assert((uint32_t)IAPReadFactorySettings(0x40060010, &output[1]) == s_Status);
		assert(s_Calls == 6 + i);
		assert(output[1] == s_Signature[0]);
	}
	assert(IAPReadFactorySettings(0x40060010, 0) == IAPSTATUS_PARAM_ERROR);
	assert(s_Calls == 8);
	puts("Shared LPC IAP reads: linkage, parameters and output passed");
	return 0;
}

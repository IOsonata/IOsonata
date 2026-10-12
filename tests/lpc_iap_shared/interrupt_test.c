/* Shared LPC IAP interrupt-state regression. No ROM or flash is accessed. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "lpc_iap.h"
#include "coredev/interrupt.h"

typedef void (*IAPENTRY)(IAP_CMD_STATUS *, IAP_CMD_STATUS *);
#ifdef __cplusplus
extern "C" {
#endif
extern IAPENTRY IapEntry;
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
uint32_t SystemCoreClock = 48000000;
#ifdef __cplusplus
}
#endif
static uint32_t s_Mask;
static uint32_t s_Calls;
static uint32_t s_Command;
static uint32_t s_Result;
static uint32_t s_Disables;
static uint32_t s_Restores;

uint32_t DisableInterrupt(void)
{
	uint32_t previous = s_Mask;

	s_Mask = 1;
	s_Disables++;
	return previous;
}

void EnableInterrupt(uint32_t State)
{
	s_Mask = State;
	s_Restores++;
}

static void RomCall(IAP_CMD_STATUS *pCmd, IAP_CMD_STATUS *pRes)
{
	assert(s_Mask == 1);
	assert(pCmd->CmdStatus == s_Command);
	s_Calls++;
	pRes->CmdStatus = s_Result;
	for (unsigned i = 0; i < 4; i++)
	{
		pRes->Data[i] = 0x12345678;
	}
}

int main(void)
{
	IAP_CMD_STATUS result;
	IAPSTATUS status;
	uint32_t output[4];
	const uint32_t commands[] = {40, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 61, 62, 70, 73, 81};

	IapEntry = RomCall;
	for (uint32_t mask = 0; mask <= 1; mask++)
	{
		for (uint32_t failure = 0; failure <= 1; failure++)
		{
			for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); i++)
			{
				s_Mask = mask;
				s_Result = failure ? IAPSTATUS_BUSY : IAPSTATUS_SUCCESS;
				s_Command = commands[i];
				s_Calls = 0;
				s_Disables = 0;
				s_Restores = 0;
				switch (s_Command)
				{
					case 40:
						status = IAPReadFactorySettings(0x40060010, output);
						break;
					case 70:
						status = IAPReadFlashSignature(output);
						break;
					case 73:
						status = IAPReadExtFlashSignature(0x1000, 0x1fff, 3, output);
						break;
					case 81:
						status = IAPEepromWritePage(0, (uint8_t *)0x20000100);
						break;
					case 50:
						status = IAPPrepSectWrite(1, 1);
						break;
					case 51:
						status = IAPCopyRamToFlash((uint8_t *)0x1000, (uint8_t *)0x10000100, 256);
						break;
					case 52:
						status = IAPEraseSector(1, 1);
						break;
					case 53:
						status = IAPBlankCheck(1, 1, &result);
						break;
					case 54:
						status = IAPReadPartId(&result);
						break;
					case 55:
						status = IAPReadBootCodeVers(&result);
						break;
					case 56:
						status = IAPCompare((uint8_t *)0x1000, (uint8_t *)0x10000100, 256, &result);
						break;
					case 57:
						status = IAPJumpISP();
						break;
					case 58:
						status = IAPReadUID(&result);
						break;
					case 59:
						status = IAPErasePage(16, 16);
						break;
					case 61:
						status = IAPEepromWrite(0, (uint8_t *)0x10000100, 16);
						break;
					default:
						status = IAPEepromRead(0, (uint8_t *)0x10000100, 16);
						break;
				}
				assert((uint32_t)status == s_Result);
				assert(s_Mask == mask);
				assert(s_Calls == 1);
				assert(s_Disables == 1);
				assert(s_Restores == 1);
			}
		}
	}
	puts("Shared LPC IAP: all 16 wrappers preserve interrupt state on success/error");
	return 0;
}

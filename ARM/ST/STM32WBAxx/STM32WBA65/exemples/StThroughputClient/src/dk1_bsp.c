/* The automatic comparison does not use Nucleo buttons or LEDs. */
#include "app_common.h"
#include "app_bsp.h"

void APP_BSP_Init(void)
{
}

void APP_BSP_PostIdle(void)
{
}

void APP_BSP_StandbyExit(void)
{
}

uint8_t APP_BSP_SerialCmdExecute(uint8_t *pRxBuffer, uint16_t iRxBufferSize)
{
	(void)pRxBuffer;
	(void)iRxBufferSize;
	return 0;
}

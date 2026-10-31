/* DK1 serial controls for ST's throughput client. No IOsonata stack calls. */
#include "app_common.h"
#include "app_bsp.h"
#include "ble_core.h"
#include "gatt_client_app.h"
#include "stm32_seq.h"

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
	if (pRxBuffer == NULL || iRxBufferSize != 2)
	{
		return 0;
	}

	/* UART reception can run in interrupt context. Defer all actions. */
	if (pRxBuffer[0] == 'B' && pRxBuffer[1] == '1')
	{
		UTIL_SEQ_RegTask(1U << CFG_TASK_BSP_BUTTON_B1, UTIL_SEQ_RFU, DTC_Button1TriggerReceived);
		UTIL_SEQ_SetTask(1U << CFG_TASK_BSP_BUTTON_B1, CFG_SEQ_PRIO_0);
		return 1;
	}
	if (pRxBuffer[0] == 'B' && pRxBuffer[1] == '2')
	{
		UTIL_SEQ_RegTask(1U << CFG_TASK_BSP_BUTTON_B2, UTIL_SEQ_RFU, DTC_Button2TriggerReceived);
		UTIL_SEQ_SetTask(1U << CFG_TASK_BSP_BUTTON_B2, CFG_SEQ_PRIO_0);
		return 1;
	}
	if (pRxBuffer[0] == 'B' && pRxBuffer[1] == '3')
	{
		UTIL_SEQ_RegTask(1U << CFG_TASK_BSP_BUTTON_B3, UTIL_SEQ_RFU, DTC_Button3TriggerReceived);
		UTIL_SEQ_SetTask(1U << CFG_TASK_BSP_BUTTON_B3, CFG_SEQ_PRIO_0);
		return 1;
	}
	return 0;
}

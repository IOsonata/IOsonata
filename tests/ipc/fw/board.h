/* Host build of the IpcIntrf test firmware: both roles in one process. */
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdint.h>

#ifndef IPCTEST_OWNER
#error "IPCTEST_OWNER comes from the command line of each translation unit"
#endif

extern uint8_t g_HostIpcMem[];

#define IPCTEST_MEM_ADDR		((uintptr_t)g_HostIpcMem)
#define IPCTEST_MEM_SIZE		0x2000U

#define IPCTEST_OWNER_TX_CHAN	0
#define IPCTEST_OWNER_RX_CHAN	1
#define IPCTEST_PEER_TX_CHAN	1
#define IPCTEST_PEER_RX_CHAN	0

#define LED1_PORT		0
#define LED1_PIN		28
#define LED1_PINOP		0
#define LED2_PORT		0
#define LED2_PIN		29
#define LED2_PINOP		0
#define LED3_PORT		0
#define LED3_PIN		30
#define LED3_PINOP		0
#define LED4_PORT		0
#define LED4_PIN		31
#define LED4_PINOP		0

#if !IPCTEST_OWNER
#define CORE_PINS { {1, 1, 8}, {1, 1, 9} }
#endif

#define UART_DEVNO		0
#define UART_BAUDRATE	1000000

#define UART_PINS	{ \
	{0, 22, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{0, 20, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

#endif

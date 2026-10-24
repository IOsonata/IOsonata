/**-------------------------------------------------------------------------
@file board.h

@brief Thingy:91 board integration test definitions.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#define UART_DEVNO					0
#define UART_RX_PORT				0
#define UART_RX_PIN					19
#define UART_RX_PINOP				1
#define UART_TX_PORT				0
#define UART_TX_PIN					18
#define UART_TX_PINOP				1

/* The board test emits one short tone at startup. */
#define THINGY91_TEST_BUZZER			1
#define THINGY91_TEST_BUZZER_FREQ		440
#define THINGY91_TEST_BUZZER_MS		100

#endif

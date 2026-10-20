#ifndef __BOARD_H__
#define __BOARD_H__
#define BOARD NUCLEO_H753ZI
#include "stm32_boards.h"
// USB2 FS on CN13: PA9 VBUS, PA11 D-, PA12 D+.
#define USB_REMOTE_WAKEUP false
// Diagnostic output through the ST-LINK virtual COM port.
#define USB_LOG_UART_DEVNO BOARD_VCP_DEVNO
#define USB_LOG_UART_PINS BOARD_VCP_PINS
#endif

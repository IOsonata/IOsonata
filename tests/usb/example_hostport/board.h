// Host-only shared-example fixture. Hardware wiring belongs in target board.h.
#ifndef USB_EXAMPLE_HOST_BOARD_H
#define USB_EXAMPLE_HOST_BOARD_H

// Pin map for uart_usb_bridge.cpp. The host build only compiles the example, so
// the port and pin numbers are placeholders.
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			0
#define UART_RX_PINOP		0
#define UART_TX_PORT		0
#define UART_TX_PIN			1
#define UART_TX_PINOP		0
#define UART_PINS { \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define USB_VBUS_PORT		0
#define USB_VBUS_PIN		2
#define USB_PINS { \
	{USB_VBUS_PORT, USB_VBUS_PIN, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif

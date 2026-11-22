/**-------------------------------------------------------------------------
@example board.h

@brief nRF5340 network-core resumable standby hardware profile

The app-core NetCoreStart image releases the network core and assigns
the network UARTE pins. The shared oscillator may remain owned by the
application core. LF timer wake uses the selected system LFCLK source.

MIT License, Copyright (c) 2026, I-SYST inc.
----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#define UART_DEVNO			0

#if defined(NRF5340_DK_PCA10095)
// nRF5340 DK network-core VCOM
#define UART_RX_PORT		1
#define UART_RX_PIN			0
#define UART_RX_PINOP		1
#define UART_TX_PORT		1
#define UART_TX_PIN			1
#define UART_TX_PINOP		1
#else
// nRF5340 Audio DK network-core VCOM
#define UART_RX_PORT		1
#define UART_RX_PIN			8
#define UART_RX_PINOP		1
#define UART_TX_PORT		1
#define UART_TX_PIN			9
#define UART_TX_PINOP		1
#endif

// Network core supports resumable SystemSleep rather than independent
// application-core System OFF and reset-wake validation.

#endif // __BOARD_H__

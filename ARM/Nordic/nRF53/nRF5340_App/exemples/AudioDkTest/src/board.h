#ifndef __BOARD_H__
#define __BOARD_H__

// Nordic PCA10121 nRF5340 Audio DK, application core.
#define UART_RETARGET
#define UART_DEVNO 0
#define UART_RX_PORT 1
#define UART_RX_PIN 4
#define UART_RX_PINOP 0
#define UART_TX_PORT 1
#define UART_TX_PIN 5
#define UART_TX_PINOP 0

#define AUDIO_I2C_DEVNO 1
#define AUDIO_SPI_DEVNO 4
#define AUDIO_CODEC_CS 1
#define AUDIO_CARD_CS 0
#define AUDIO_CODEC_IF_PORT 0
#define AUDIO_CODEC_IF_PIN 21
#define AUDIO_NPM_ISET_PORT 0
#define AUDIO_NPM_ISET_PIN 23

#endif

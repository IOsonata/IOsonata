#ifndef __BOARD_H__
#define __BOARD_H__

#define UART_DEVNO			0
#define UART_RX_PORT			0
#define UART_RX_PIN			0
#define UART_RX_PINOP			1
#define UART_TX_PORT			0
#define UART_TX_PIN			1
#define UART_TX_PINOP			1

#define I2C_DEVNO			2
#define I2C_SDA_PORT			0
#define I2C_SDA_PIN			9
#define I2C_SDA_PINOP			1
#define I2C_SCL_PORT			0
#define I2C_SCL_PIN			8
#define I2C_SCL_PINOP			1

#define SPI_DEVNO			3
#define SPI_SCK_PORT			0
#define SPI_SCK_PIN			13
#define SPI_SCK_PINOP			1
#define SPI_MISO_PORT			0
#define SPI_MISO_PIN			15
#define SPI_MISO_PINOP			1
#define SPI_MOSI_PORT			0
#define SPI_MOSI_PIN			14
#define SPI_MOSI_PINOP			1

/* Keep the shared SPI3 CS ordering identical to the Thingy:91 X schematic/DTS. */
#define SPI_FLASH_CS_PORT		0
#define SPI_FLASH_CS_PIN		12
#define SPI_FLASH_CS_PINOP		1
#define SPI_WIFI_CS_PORT		0
#define SPI_WIFI_CS_PIN		17
#define SPI_WIFI_CS_PINOP		1
#define SPI_BMI270_CS_PORT		0
#define SPI_BMI270_CS_PIN		10
#define SPI_BMI270_CS_PINOP		1
#define BMI270_SPI_CS			2

#define BMI270_IRQ_PORT			0
#define BMI270_IRQ_PIN			6

#endif

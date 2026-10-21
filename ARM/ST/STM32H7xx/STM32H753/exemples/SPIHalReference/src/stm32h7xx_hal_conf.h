#ifndef SPI_REFERENCE_HAL_CONF_H
#define SPI_REFERENCE_HAL_CONF_H
#define HAL_SPI_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define USE_HAL_SPI_REGISTER_CALLBACKS 0U
#define USE_SPI_CRC 0U
#define assert_param(expr) ((void)0U)
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_spi.h"
#endif

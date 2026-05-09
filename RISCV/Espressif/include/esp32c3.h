/**-------------------------------------------------------------------------
@file	esp32c3.h

@brief	ESP32-C3 definition


@author	Nguyen Hoan Hoang
@date	May 9, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __ESP32C3_H__
#define __ESP32C3_H__

// Peripheral register base addresses
#define ESP32C3_UART0_BASE					0x60000000
#define ESP32C3_SPI1_BASE					0x60002000
#define ESP32C3_SPI0_BASE					0x60003000
#define ESP32C3_GPIO_BASE					0x60004000
#define ESP32C3_LP_MGNT_BASE				0x60008000
#define ESP32C3_IOMUX_BASE					0x60009000
#define ESP32C3_UART1_BASE					0x60010000
#define ESP32C3_I2C_BASE					0x60013000
#define ESP32C3_UHCIO_BASE					0x60014000
#define ESP32C3_RMT_BASE					0x60016000	//!< Remote Control Peripheral
#define ESP32C3_LEDC_BASE					0x60019000	//!< LED PWM Controller
#define ESP32C3_EFUSE_BASE					0x6001A000
#define ESP32C3_TIMG0_BASE					0x6001F000	//!< Timer Group 0
#define ESP32C3_TIMG1_BASE					0x60020000	//!< Timer Group 1
#define ESP32C3_SYSTIMER_BASE				0x60023000	//!< System Timer
#define ESP32C3_SPI2_BASE					0x60024000
#define ESP32C3_SYSCON_BASE					0x60026000
#define ESP32C3_TWAI_BASE					0x6002B000	//!< Two-wire Automotive Interface
#define ESP32C3_I2S_BASE					0x6002D000
#define ESP32C3_AES_BASE					0x6003A000
#define ESP32C3_SHA_BASE					0x6003B000
#define ESP32C3_RSA_BASE					0x6003C000
#define ESP32C3_RSA_DS_BASE					0x6003D000
#define ESP32C3_HMAC_BASE					0x6003E000
#define ESP32C3_GDMA_BASE					0x6003F000
#define ESP32C3_ADC_BASE					0x60040000
#define ESP32C3_USB_JTAG_BASE				0x60043000
#define ESP32C3_SYSTEM_BASE					0x600C0000
#define ESP32C3_PMS_BASE					0x600C1000
#define ESP32C3_INTMTX_BASE					0x600C2000
#define ESP32C3_XTS_AES_BASE				0x600CC000	//!< External Memory Encryption and Decryption
#define ESP32C3_ASSIST_DEBUG_BASE			0x600CE000
#define ESP32C3_WORLD_CTRL_BASE				0x600D0000




// Peripheral instance counts
#define ESP32C3_CPU_CORE_COUNT              1U
#define ESP32C3_CPU_INTERRUPT_COUNT         32U
#define ESP32C3_UART_COUNT                  2U
#define ESP32C3_HP_UART_COUNT               2U
#define ESP32C3_LP_UART_COUNT               0U
#define ESP32C3_SPI_MEM_COUNT               2U     //!< SPI0/SPI1 memory controllers.
#define ESP32C3_GP_SPI_COUNT                1U     //!< SPI2 general-purpose controller.
#define ESP32C3_I2C_COUNT                   1U
#define ESP32C3_HP_I2C_COUNT                1U
#define ESP32C3_LP_I2C_COUNT                0U
#define ESP32C3_I2S_COUNT                   1U
#define ESP32C3_TWAI_COUNT                  1U
#define ESP32C3_RMT_COUNT                   1U
#define ESP32C3_LEDC_COUNT                  1U
#define ESP32C3_TIMER_GROUP_COUNT           2U
#define ESP32C3_TIMER_PER_GROUP_COUNT       1U
#define ESP32C3_TIMER_COUNT                 2U
#define ESP32C3_SYSTIMER_COUNT              1U
#define ESP32C3_SYSTIMER_COUNTER_COUNT      2U
#define ESP32C3_SYSTIMER_ALARM_COUNT        3U
#define ESP32C3_GDMA_COUNT                  1U
#define ESP32C3_GDMA_PAIR_COUNT             3U
#define ESP32C3_GDMA_TX_CHANNEL_COUNT       3U
#define ESP32C3_GDMA_RX_CHANNEL_COUNT       3U
#define ESP32C3_GPIO_PORT_COUNT             1U
#define ESP32C3_GPIO_PIN_COUNT              22U
#define ESP32C3_GPIO_PIN_MAX                21U
#define ESP32C3_GPIO_PIN_REG_COUNT          22U
#define ESP32C3_GPIO_FUNC_IN_COUNT          128U
#define ESP32C3_GPIO_FUNC_OUT_COUNT         22U
#define ESP32C3_GPIO_FUNC_OUT_REG_COUNT     22U
#define ESP32C3_ADC_COUNT                   2U
#define ESP32C3_ADC_DIGI_CONTROLLER_COUNT   1U
#define ESP32C3_UHCI_COUNT                  1U
#define ESP32C3_USB_SERIAL_JTAG_COUNT       1U
#define ESP32C3_AES_COUNT                   1U
#define ESP32C3_SHA_COUNT                   1U
#define ESP32C3_RSA_COUNT                   1U
#define ESP32C3_RSA_DS_COUNT                1U
#define ESP32C3_HMAC_COUNT                  1U
#define ESP32C3_EFUSE_COUNT                 1U
#define ESP32C3_INTMTX_COUNT                1U

// Common ESP32 RISC-V selection used by shared drivers.
#define ESP32_UART_COUNT                    ESP32C3_UART_COUNT
#define ESP32_HP_UART_COUNT                 ESP32C3_HP_UART_COUNT
#define ESP32_LP_UART_COUNT                 ESP32C3_LP_UART_COUNT
#define ESP32_SPI_MEM_COUNT                 ESP32C3_SPI_MEM_COUNT
#define ESP32_GP_SPI_COUNT                  ESP32C3_GP_SPI_COUNT
#define ESP32_I2C_COUNT                     ESP32C3_I2C_COUNT
#define ESP32_HP_I2C_COUNT                  ESP32C3_HP_I2C_COUNT
#define ESP32_LP_I2C_COUNT                  ESP32C3_LP_I2C_COUNT
#define ESP32_I2S_COUNT                     ESP32C3_I2S_COUNT
#define ESP32_TWAI_COUNT                    ESP32C3_TWAI_COUNT
#define ESP32_RMT_COUNT                     ESP32C3_RMT_COUNT
#define ESP32_LEDC_COUNT                    ESP32C3_LEDC_COUNT
#define ESP32_TIMER_GROUP_COUNT             ESP32C3_TIMER_GROUP_COUNT
#define ESP32_TIMER_PER_GROUP_COUNT         ESP32C3_TIMER_PER_GROUP_COUNT
#define ESP32_TIMER_COUNT                   ESP32C3_TIMER_COUNT
#define ESP32_SYSTIMER_COUNT                ESP32C3_SYSTIMER_COUNT
#define ESP32_SYSTIMER_COUNTER_COUNT        ESP32C3_SYSTIMER_COUNTER_COUNT
#define ESP32_SYSTIMER_ALARM_COUNT          ESP32C3_SYSTIMER_ALARM_COUNT
#define ESP32_GDMA_COUNT                    ESP32C3_GDMA_COUNT
#define ESP32_GDMA_PAIR_COUNT               ESP32C3_GDMA_PAIR_COUNT
#define ESP32_GDMA_TX_CHANNEL_COUNT         ESP32C3_GDMA_TX_CHANNEL_COUNT
#define ESP32_GDMA_RX_CHANNEL_COUNT         ESP32C3_GDMA_RX_CHANNEL_COUNT
#define ESP32_ADC_COUNT                     ESP32C3_ADC_COUNT
#define ESP32_ADC_DIGI_CONTROLLER_COUNT     ESP32C3_ADC_DIGI_CONTROLLER_COUNT
#define ESP32_UHCI_COUNT                    ESP32C3_UHCI_COUNT
#define ESP32_USB_SERIAL_JTAG_COUNT         ESP32C3_USB_SERIAL_JTAG_COUNT
#define ESP32_GPIO_PORT_COUNT               ESP32C3_GPIO_PORT_COUNT
#define ESP32_IOMUX_BASE                    ESP32C3_IOMUX_BASE
#define ESP32_GPIO_BASE                     ESP32C3_GPIO_BASE
#define ESP32_GPIO_PIN_COUNT                ESP32C3_GPIO_PIN_COUNT
#define ESP32_GPIO_PIN_MAX                  ESP32C3_GPIO_PIN_MAX
#define ESP32_GPIO_PIN_REG_COUNT            ESP32C3_GPIO_PIN_REG_COUNT
#define ESP32_GPIO_FUNC_IN_COUNT            ESP32C3_GPIO_FUNC_IN_COUNT
#define ESP32_GPIO_FUNC_OUT_COUNT           ESP32C3_GPIO_FUNC_OUT_COUNT
#define ESP32_GPIO_FUNC_OUT_REG_COUNT       ESP32C3_GPIO_FUNC_OUT_REG_COUNT
#define ESP32_AES_COUNT                     ESP32C3_AES_COUNT
#define ESP32_SHA_COUNT                     ESP32C3_SHA_COUNT
#define ESP32_RSA_COUNT                     ESP32C3_RSA_COUNT
#define ESP32_RSA_DS_COUNT                  ESP32C3_RSA_DS_COUNT
#define ESP32_HMAC_COUNT                    ESP32C3_HMAC_COUNT
#define ESP32_EFUSE_COUNT                   ESP32C3_EFUSE_COUNT
#define ESP32_INTMTX_COUNT                  ESP32C3_INTMTX_COUNT

// Backward-compatible aliases used by older ESP32 pin code.
#define ESP32_PIN_MAX                       ESP32_GPIO_PIN_COUNT
#define ESP32_MAX_PIN                       ESP32_GPIO_PIN_MAX

#endif // __ESP32C3_H__

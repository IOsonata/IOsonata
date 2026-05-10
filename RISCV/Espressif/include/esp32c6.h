/**-------------------------------------------------------------------------
@file	esp32c6.h

@brief	ESP32-C6 definition


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
#ifndef __ESP32C6_H__
#define __ESP32C6_H__

// Peripheral register base addresses
#define ESP32C6_UART0_BASE					0x60000000	//!< UART0
#define ESP32C6_UART1_BASE					0x60001000	//!< UART1
#define ESP32C6_XTS_AES_BASE				0x60002000	//!< External Memory Encryption and Decryption
#define ESP32C6_I2C_BASE					0x60004000
#define ESP32C6_UHCI_BASE					0x60005000
#define ESP32C6_RMT_BASE					0x60006000	//!< Remote Control Peripheral
#define ESP32C6_LEDC_BASE					0x60007000	//!< LED PWM Controller
#define ESP32C6_TIMG0_BASE					0x60008000	//!< Timer Group 0
#define ESP32C6_TIMG1_BASE					0x60009000	//!< Timer Group 1
#define ESP32C6_SYSTIMER_BASE				0x6000A000	//!< System Timer
#define ESP32C6_TWAI0_BASE					0x6000B000	//!< Two-wire Automotive Interface 0
#define ESP32C6_I2S_BASE					0x6000C000
#define ESP32C6_TWAI1_BASE					0x6000D000	//!< Two-wire Automotive Interface 1
#define ESP32C6_SAR_ADC_BASE				0x6000E000	//!< Successive approximation ADC
#define ESP32C6_USB_JTAG_BASE				0x6000F000
#define ESP32C6_INTMTX_BASE					0x60010000	//!< Interrupt matrix
#define ESP32C6_PCNT_BASE					0x60012000	//!< Pulse Count Controller
#define ESP32C6_SOC_ETM_BASE				0x60013000	//!< Event Task Matrix
#define ESP32C6_MCPWM_BASE					0x60014000	//!< Motor Control PWM
#define ESP32C6_PARL_IO_BASE				0x60015000	//!< Parallel IO Controller
#define ESP32C6_SDIO_HINF_BASE				0x60016000
#define ESP32C6_SDIO_SLC_BASE				0x60017000
#define ESP32C6_SDIO_SLCHOST_BASE			0x60018000
#define ESP32C6_GDMA_BASE					0x60080000
#define ESP32C6_GP_SPI2_BASE				0x60081000	//!< General Purpose SPI2
#define ESP32C6_AES_BASE					0x60088000	//!< AES accelerator
#define ESP32C6_SHA_BASE					0x60089000
#define ESP32C6_RSA_BASE					0x6008A000
#define ESP32C6_ECC_BASE					0x6008B000
#define ESP32C6_RSA_DS_BASE					0x6008C000	//!< RSA Digital Signature Peripheral
#define ESP32C6_HMAC_BASE					0x6008D000
#define ESP32C6_IOMUX_BASE					0x60090000
#define ESP32C6_GPIO_MATRIX_BASE			0x60091000
#define ESP32C6_GPIO_BASE					ESP32C6_GPIO_MATRIX_BASE	//!< Naming alias for cross-chip code; same address as ESP32C6_GPIO_MATRIX_BASE.
#define ESP32C6_MEM_MONITOR_BASE			0x60092000
#define ESP32C6_HP_SYSREG_BASE				0x60095000
#define ESP32C6_PCR_BASE					0x60096000	//!< Power/Clock/Reset
#define ESP32C6_TEE_BASE					0x60098000	//!< Trusted Execution Environment
#define ESP32C6_HP_APM_BASE					0x60099000	//!< Access Permission Management Controller
#define ESP32C6_PMU_BASE					0x600B0000	//!< Power Management Unit
#define ESP32C6_LP_CLKRST_BASE				0x600B0400	//!< Low-power Clock/Reset Register
#define ESP32C6_EFUSE_BASE					0x600B0800
#define ESP32C6_RTC_TIMER_BASE				0x600B0C00	//!< RTC timer
#define ESP32C6_LP_AON_BASE					0x600B1000	//!< Low-power Always-on Register
#define ESP32C6_LP_UART_BASE				0x600B1400	//!< Low-power UART
#define ESP32C6_LP_I2C_BASE					0x600B1800
#define ESP32C6_RTC_WDT_BASE				0x600B1C00	//!< RTC Watch Dog Timer
#define ESP32C6_LP_IOMUX_BASE				0x600B2000
#define ESP32C6_I2C_ANA_MST_BASE			0x600B2400	//!< I2C Analog Master
#define ESP32C6_LPPERI_BASE					0x600B2800	//!< Low-power Peripheral
#define ESP32C6_LP_AON_BASE_PERI_BASE		0x600B2C00	//!< Low-power Analog Peripheral
#define ESP32C6_LP_TEE_BASE					0x600B3400	//!< Low-power Trusted Execution Environment
#define ESP32C6_LP_APM_BASE					0x600B3800	//!< Low-power Access Permission Management
#define ESP32C6_TRACE_BASE					0x600C0000	//!< RISC-V Trace Encoder
#define ESP32C6_ASSIST_DEBUG_BASE			0x600C2000
#define ESP32C6_INTPRI_BASE					0x600C5000	//!< Interrupt Priority Register

// Peripheral instance counts
#define ESP32C6_CPU_CORE_COUNT              1U
#define ESP32C6_CPU_INTERRUPT_COUNT         32U
#define ESP32C6_UART_COUNT                  3U     //!< 2 HP UART + 1 LP UART.
#define ESP32C6_HP_UART_COUNT               2U
#define ESP32C6_LP_UART_COUNT               1U
#define ESP32C6_SPI_MEM_COUNT               1U
#define ESP32C6_GP_SPI_COUNT                1U     //!< GP-SPI2.
#define ESP32C6_I2C_COUNT                   2U     //!< 1 HP I2C + 1 LP I2C.
#define ESP32C6_HP_I2C_COUNT                1U
#define ESP32C6_LP_I2C_COUNT                1U
#define ESP32C6_I2S_COUNT                   1U
#define ESP32C6_TWAI_COUNT                  2U
#define ESP32C6_RMT_COUNT                   1U
#define ESP32C6_LEDC_COUNT                  1U
#define ESP32C6_PCNT_COUNT                  1U
#define ESP32C6_ETM_COUNT                   1U
#define ESP32C6_MCPWM_COUNT                 1U
#define ESP32C6_PARL_IO_COUNT               1U
#define ESP32C6_TIMER_GROUP_COUNT           2U
#define ESP32C6_TIMER_PER_GROUP_COUNT       1U
#define ESP32C6_TIMER_COUNT                 2U
#define ESP32C6_SYSTIMER_COUNT              1U
#define ESP32C6_SYSTIMER_COUNTER_COUNT      2U
#define ESP32C6_SYSTIMER_ALARM_COUNT        3U
#define ESP32C6_GDMA_COUNT                  1U
#define ESP32C6_GDMA_PAIR_COUNT             3U
#define ESP32C6_GDMA_TX_CHANNEL_COUNT       3U
#define ESP32C6_GDMA_RX_CHANNEL_COUNT       3U
#define ESP32C6_GPIO_PORT_COUNT             1U
#define ESP32C6_GPIO_PIN_COUNT              31U
#define ESP32C6_GPIO_PIN_MAX                30U
#define ESP32C6_GPIO_PIN_REG_COUNT          31U
#define ESP32C6_GPIO_FUNC_IN_COUNT          128U
#define ESP32C6_GPIO_FUNC_OUT_COUNT         31U
#define ESP32C6_GPIO_FUNC_OUT_REG_COUNT     31U
#define ESP32C6_ADC_COUNT                   1U
#define ESP32C6_ADC_DIGI_CONTROLLER_COUNT   1U
#define ESP32C6_UHCI_COUNT                  1U
#define ESP32C6_USB_SERIAL_JTAG_COUNT       1U
#define ESP32C6_AES_COUNT                   1U
#define ESP32C6_SHA_COUNT                   1U
#define ESP32C6_RSA_COUNT                   1U
#define ESP32C6_RSA_DS_COUNT                1U
#define ESP32C6_HMAC_COUNT                  1U
#define ESP32C6_ECC_COUNT                   1U
#define ESP32C6_EFUSE_COUNT                 1U
#define ESP32C6_INTMTX_COUNT                1U
#define ESP32C6_INTPRI_COUNT                1U

// Common ESP32 RISC-V selection used by shared drivers.
#define ESP32_UART_COUNT                    ESP32C6_UART_COUNT
#define ESP32_HP_UART_COUNT                 ESP32C6_HP_UART_COUNT
#define ESP32_LP_UART_COUNT                 ESP32C6_LP_UART_COUNT
#define ESP32_SPI_MEM_COUNT                 ESP32C6_SPI_MEM_COUNT
#define ESP32_GP_SPI_COUNT                  ESP32C6_GP_SPI_COUNT
#define ESP32_I2C_COUNT                     ESP32C6_I2C_COUNT
#define ESP32_HP_I2C_COUNT                  ESP32C6_HP_I2C_COUNT
#define ESP32_LP_I2C_COUNT                  ESP32C6_LP_I2C_COUNT
#define ESP32_I2S_COUNT                     ESP32C6_I2S_COUNT
#define ESP32_TWAI_COUNT                    ESP32C6_TWAI_COUNT
#define ESP32_RMT_COUNT                     ESP32C6_RMT_COUNT
#define ESP32_LEDC_COUNT                    ESP32C6_LEDC_COUNT
#define ESP32_PCNT_COUNT                    ESP32C6_PCNT_COUNT
#define ESP32_ETM_COUNT                     ESP32C6_ETM_COUNT
#define ESP32_MCPWM_COUNT                   ESP32C6_MCPWM_COUNT
#define ESP32_PARL_IO_COUNT                 ESP32C6_PARL_IO_COUNT
#define ESP32_TIMER_GROUP_COUNT             ESP32C6_TIMER_GROUP_COUNT
#define ESP32_TIMER_PER_GROUP_COUNT         ESP32C6_TIMER_PER_GROUP_COUNT
#define ESP32_TIMER_COUNT                   ESP32C6_TIMER_COUNT
#define ESP32_SYSTIMER_COUNT                ESP32C6_SYSTIMER_COUNT
#define ESP32_SYSTIMER_COUNTER_COUNT        ESP32C6_SYSTIMER_COUNTER_COUNT
#define ESP32_SYSTIMER_ALARM_COUNT          ESP32C6_SYSTIMER_ALARM_COUNT
#define ESP32_GDMA_COUNT                    ESP32C6_GDMA_COUNT
#define ESP32_GDMA_PAIR_COUNT               ESP32C6_GDMA_PAIR_COUNT
#define ESP32_GDMA_TX_CHANNEL_COUNT         ESP32C6_GDMA_TX_CHANNEL_COUNT
#define ESP32_GDMA_RX_CHANNEL_COUNT         ESP32C6_GDMA_RX_CHANNEL_COUNT
#define ESP32_ADC_COUNT                     ESP32C6_ADC_COUNT
#define ESP32_ADC_DIGI_CONTROLLER_COUNT     ESP32C6_ADC_DIGI_CONTROLLER_COUNT
#define ESP32_UHCI_COUNT                    ESP32C6_UHCI_COUNT
#define ESP32_USB_SERIAL_JTAG_COUNT         ESP32C6_USB_SERIAL_JTAG_COUNT
#define ESP32_GPIO_PORT_COUNT               ESP32C6_GPIO_PORT_COUNT
#define ESP32_IOMUX_BASE                    ESP32C6_IOMUX_BASE
#define ESP32_GPIO_BASE                     ESP32C6_GPIO_BASE
#define ESP32_GPIO_PIN_COUNT                ESP32C6_GPIO_PIN_COUNT
#define ESP32_GPIO_PIN_MAX                  ESP32C6_GPIO_PIN_MAX
#define ESP32_GPIO_PIN_REG_COUNT            ESP32C6_GPIO_PIN_REG_COUNT
#define ESP32_GPIO_FUNC_IN_COUNT            ESP32C6_GPIO_FUNC_IN_COUNT
#define ESP32_GPIO_FUNC_OUT_COUNT           ESP32C6_GPIO_FUNC_OUT_COUNT
#define ESP32_GPIO_FUNC_OUT_REG_COUNT       ESP32C6_GPIO_FUNC_OUT_REG_COUNT
#define ESP32_AES_COUNT                     ESP32C6_AES_COUNT
#define ESP32_SHA_COUNT                     ESP32C6_SHA_COUNT
#define ESP32_RSA_COUNT                     ESP32C6_RSA_COUNT
#define ESP32_RSA_DS_COUNT                  ESP32C6_RSA_DS_COUNT
#define ESP32_HMAC_COUNT                    ESP32C6_HMAC_COUNT
#define ESP32_ECC_COUNT                     ESP32C6_ECC_COUNT
#define ESP32_EFUSE_COUNT                   ESP32C6_EFUSE_COUNT
#define ESP32_INTMTX_COUNT                  ESP32C6_INTMTX_COUNT
#define ESP32_INTPRI_COUNT                  ESP32C6_INTPRI_COUNT

// Backward-compatible aliases used by older ESP32 pin code.
#define ESP32_PIN_MAX                       ESP32_GPIO_PIN_COUNT
#define ESP32_MAX_PIN                       ESP32_GPIO_PIN_MAX

#endif // __ESP32C6_H__

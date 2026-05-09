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
#define ESP32C3_REMOTE_CTRL_BASE			0x60016000
#define ESP32C3_LED_PWM_BASE				0x60019000
#define ESP32C3_EFUSE_BASE					0x6001A000
#define ESP32C3_TIMER0_BASE					0x6001F000
#define ESP32C3_TIMER1_BASE					0x60020000
#define ESP32C3_SYST_TIMER_BASE				0x60023000
#define ESP32C3_SPI2_BASE					0x60024000
#define ESP32C3_SYSCON_BASE					0x60026000
#define ESP32C3_TWI_AUTO_BASE				0x6002B000
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
#define ESP32C3_EXTMEM_ENCRYPT_BASE			0x600CC000
#define ESP32C3_ASSIST_DEBUG_BASE			0x600CE000
#define ESP32C3_WORLD_CTRL_BASE				0x600D0000

#endif // __ESP32C3_H__
/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

#define LED_RED_PORT					0
#define LED_RED_PIN						7
#define LED_RED_PINOP					0
#define LED_GREEN_PORT					0
#define LED_GREEN_PIN					11
#define LED_GREEN_PINOP					0
#define LED_BLUE_PORT					0
#define LED_BLUE_PIN					12
#define LED_BLUE_PINOP					0

#define I2C_DEVNO						0
#define I2C_SDA_PORT					0
#define I2C_SDA_PIN						29
#define I2C_SDA_PINOP					1
#define I2C_SCL_PORT					0
#define I2C_SCL_PIN						26
#define I2C_SCL_PINOP					1

#define SPI_DEVNO            			2
#define SPI_MISO_PORT        			0
#define SPI_MISO_PIN         			15
#define SPI_MISO_PINOP       			1
#define SPI_MOSI_PORT        			0
#define SPI_MOSI_PIN         			6
#define SPI_MOSI_PINOP       			1
#define SPI_SCK_PORT         			0
#define SPI_SCK_PIN          			16
#define SPI_SCK_PINOP        			1

#define SPI_BMI323_CS_PORT         		0
#define SPI_BMI323_CS_PIN          		24
#define SPI_BMI323_CS_PINOP        		1

#define VDD_EN_PORT						1
#define VDD_EN_PIN						12
#define VDD_EN_PINOP					0

#define VDDIO_EN_PORT					0
#define VDDIO_EN_PIN					2
#define VDDIO_EN_PINOP					0

#define LS_EN_PORT						0
#define LS_EN_PIN						3
#define LS_EN_PINOP						0

#endif // __BOARD_H__


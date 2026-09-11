/**-------------------------------------------------------------------------
@file	board.h

@brief	Bosch BMI323 application board pins for the USB HID 3D mouse demo

@author	Hoang Nguyen Hoan
@date	Sep. 10, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __USB_HID_3D_MOUSE_BOARD_H__
#define __USB_HID_3D_MOUSE_BOARD_H__

#define BMI323_SPI_DEVNO		2
#define BMI323_SPI_MISO_PORT	0
#define BMI323_SPI_MISO_PIN	15
#define BMI323_SPI_MISO_PINOP	1
#define BMI323_SPI_MOSI_PORT	0
#define BMI323_SPI_MOSI_PIN	6
#define BMI323_SPI_MOSI_PINOP	1
#define BMI323_SPI_SCK_PORT	0
#define BMI323_SPI_SCK_PIN	16
#define BMI323_SPI_SCK_PINOP	1
#define BMI323_SPI_CS_PORT	0
#define BMI323_SPI_CS_PIN	24
#define BMI323_SPI_CS_PINOP	1

#define BMI323_VDD_EN_PORT	1
#define BMI323_VDD_EN_PIN	12
#define BMI323_VDD_EN_PINOP	0
#define BMI323_VDDIO_EN_PORT	0
#define BMI323_VDDIO_EN_PIN	2
#define BMI323_VDDIO_EN_PINOP	0
#define BMI323_LS_EN_PORT		0
#define BMI323_LS_EN_PIN		3
#define BMI323_LS_EN_PINOP	0

#endif // __USB_HID_3D_MOUSE_BOARD_H__

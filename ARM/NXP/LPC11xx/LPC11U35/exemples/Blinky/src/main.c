/**-------------------------------------------------------------------------
@example	main.c

@brief	LPC11U35 GPIO toggle and delay test.

@author	Hoang Nguyen Hoan
@date	Sep. 5, 2019

@license

MIT License

Copyright (c) 2019, I-SYST inc., all rights reserved

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
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "idelay.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"

#define SWCLK_TCK_PORT				0
#define SWCLK_TCK_PIN				14
#define SWCLK_TCK_PINOP				1

int main(void)
{

	IOPinConfig(SWCLK_TCK_PORT, SWCLK_TCK_PIN, SWCLK_TCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

	while (1)
	{
		LPC_GPIO->SET[SWCLK_TCK_PORT] = (1 << SWCLK_TCK_PIN);
		nsDelay(300);
		LPC_GPIO->CLR[SWCLK_TCK_PORT] = (1 << SWCLK_TCK_PIN);
	}
/*
	IOPinConfig(0, BLUEIO_LED_BLUE_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_BLUE_PIN);
	IOPinConfig(BLUEIO_LED_GREEN_PORT, 17, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(BLUEIO_LED_GREEN_PORT, 17);//BLUEIO_LED_GREEN_PIN);
	IOPinConfig(0, BLUEIO_LED_RED_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_RED_PIN);
  
	while(true)
	{
		IOPinClear(0, BLUEIO_LED_BLUE_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_BLUE_PIN);
		IOPinClear(BLUEIO_LED_GREEN_PORT, 17);//BLUEIO_LED_GREEN_PIN);
		usDelay(1000000);
		IOPinSet(BLUEIO_LED_GREEN_PORT, 17);//BLUEIO_LED_GREEN_PIN);
		IOPinClear(0, BLUEIO_LED_RED_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_RED_PIN);
		usDelay(1000000);
	}
	*/
}

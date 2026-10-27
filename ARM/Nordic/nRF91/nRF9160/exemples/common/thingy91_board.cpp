/**-------------------------------------------------------------------------
@file thingy91_board.cpp

@brief Thingy:91 application-level user I/O and companion-reset helpers.

@license
MIT License

Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#include "iopinctrl.h"

#include "thingy91_board.h"

void Thingy91UserIoInit()
{
	IOPinClear(THINGY91_LED_RED_PORT, THINGY91_LED_RED_PIN);
	IOPinClear(THINGY91_LED_GREEN_PORT, THINGY91_LED_GREEN_PIN);
	IOPinClear(THINGY91_LED_BLUE_PORT, THINGY91_LED_BLUE_PIN);

	IOPinConfig(THINGY91_LED_RED_PORT, THINGY91_LED_RED_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(THINGY91_LED_GREEN_PORT, THINGY91_LED_GREEN_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(THINGY91_LED_BLUE_PORT, THINGY91_LED_BLUE_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(THINGY91_BUTTON1_PORT, THINGY91_BUTTON1_PIN,
		IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
}

void Thingy91LedSet(bool Red, bool Green, bool Blue)
{
	if (Red)
		IOPinSet(THINGY91_LED_RED_PORT, THINGY91_LED_RED_PIN);
	else
		IOPinClear(THINGY91_LED_RED_PORT, THINGY91_LED_RED_PIN);

	if (Green)
		IOPinSet(THINGY91_LED_GREEN_PORT, THINGY91_LED_GREEN_PIN);
	else
		IOPinClear(THINGY91_LED_GREEN_PORT, THINGY91_LED_GREEN_PIN);

	if (Blue)
		IOPinSet(THINGY91_LED_BLUE_PORT, THINGY91_LED_BLUE_PIN);
	else
		IOPinClear(THINGY91_LED_BLUE_PORT, THINGY91_LED_BLUE_PIN);
}

bool Thingy91ButtonPressed()
{
	return IOPinRead(THINGY91_BUTTON1_PORT, THINGY91_BUTTON1_PIN) == 0;
}

void Thingy91Nrf52840ResetInit()
{
	IOPinSet(THINGY91_NRF52840_RESET_PORT, THINGY91_NRF52840_RESET_PIN);
	IOPinConfig(THINGY91_NRF52840_RESET_PORT, THINGY91_NRF52840_RESET_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

void Thingy91Nrf52840Reset(bool Assert)
{
	if (Assert)
		IOPinClear(THINGY91_NRF52840_RESET_PORT, THINGY91_NRF52840_RESET_PIN);
	else
		IOPinSet(THINGY91_NRF52840_RESET_PORT, THINGY91_NRF52840_RESET_PIN);
}

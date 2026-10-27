/**-------------------------------------------------------------------------
@file thingy91_board.h

@brief Thingy:91 application-level user I/O and companion-reset mappings.

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
#ifndef __THINGY91_BOARD_H__
#define __THINGY91_BOARD_H__

#include <stdbool.h>

#define THINGY91_BUTTON1_PORT				0
#define THINGY91_BUTTON1_PIN				26

#define THINGY91_LED_RED_PORT				0
#define THINGY91_LED_RED_PIN				29
#define THINGY91_LED_GREEN_PORT				0
#define THINGY91_LED_GREEN_PIN				30
#define THINGY91_LED_BLUE_PORT				0
#define THINGY91_LED_BLUE_PIN				31

#define THINGY91_BUZZER_PWM_DEVNO			1
#define THINGY91_BUZZER_PWM_CHAN			0
#define THINGY91_BUZZER_PORT				0
#define THINGY91_BUZZER_PIN				28

#define THINGY91_NRF52840_RESET_PORT			0
#define THINGY91_NRF52840_RESET_PIN			10

void Thingy91UserIoInit();
void Thingy91LedSet(bool Red, bool Green, bool Blue);
bool Thingy91ButtonPressed();
void Thingy91Nrf52840ResetInit();
void Thingy91Nrf52840Reset(bool Assert);

#endif

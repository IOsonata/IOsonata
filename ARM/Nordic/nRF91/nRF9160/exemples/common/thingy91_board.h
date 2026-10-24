/**-------------------------------------------------------------------------
@file thingy91_board.h

@brief Thingy:91 application-level user I/O and companion-reset mappings.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
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
